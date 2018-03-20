/*
 *  Copyright 2017, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  abstract_navigation_server.tcc
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <nav_msgs/Path.h>
#include "mbf_abstract_nav/abstract_navigation_server.h"

namespace mbf_abstract_nav
{

AbstractNavigationServer::AbstractNavigationServer(
  const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
  typename AbstractPlannerExecution::Ptr planning_ptr,
  typename AbstractControllerExecution::Ptr moving_ptr,
  typename AbstractRecoveryExecution::Ptr recovery_ptr) :
  tf_listener_ptr_(tf_listener_ptr),
  planning_ptr_(planning_ptr),
  moving_ptr_(moving_ptr),
  recovery_ptr_(recovery_ptr),
  private_nh_("~"),
  path_seq_count_(0),
  action_client_exe_path_(private_nh_, name_action_exe_path),
  action_client_get_path_(private_nh_, name_action_get_path),
  action_client_recovery_(private_nh_, name_action_recovery)
{
  ros::NodeHandle nh;

  // non-dynamically reconfigurable parameters
  private_nh_.param("robot_frame", robot_frame_, std::string("base_link"));
  private_nh_.param("global_frame", global_frame_, std::string("map"));
  private_nh_.param("tf_timeout", tf_timeout_, 3.0);

  // informative topics: current goal and global path
  path_pub_ = nh.advertise<nav_msgs::Path>("global_path", 1);
  current_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("current_goal", 1);

  // oscillation timeout and distance
  double oscillation_timeout;
  private_nh_.param("oscillation_timeout", oscillation_timeout, 0.0);
  oscillation_timeout_ = ros::Duration(oscillation_timeout);
  private_nh_.param("oscillation_distance", oscillation_distance_, 0.02);

  action_server_get_path_ptr_ = ActionServerGetPathPtr(
    new ActionServerGetPath(
      private_nh_,
      name_action_get_path,
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionGetPath, this, _1),
      false));

  action_server_exe_path_ptr_ = ActionServerExePathPtr(
    new ActionServerExePath(
      private_nh_,
      name_action_exe_path,
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionExePath, this, _1),
      false));

  action_server_recovery_ptr_ = ActionServerRecoveryPtr(
    new ActionServerRecovery(
      private_nh_,
      name_action_recovery,
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionRecovery, this, _1),
      false));

  action_server_move_base_ptr_ = ActionServerMoveBasePtr(
    new ActionServerMoveBase(
      private_nh_,
      name_action_move_base,
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionMoveBase, this, _1),
      false));

  // XXX note that we don't start a dynamic reconfigure server, to avoid colliding with the one possibly created by
  // the base class. If none, it should call startDynamicReconfigureServer method to start the one defined here for
  // providing just the abstract server parameters
}

void AbstractNavigationServer::initializeServerComponents()
{
  if(!moving_ptr_->initialize())
  {
    ROS_WARN_STREAM("No controller has been loaded and initialized successfully!");
  }
  if(!planning_ptr_->initialize())
  {
    ROS_WARN_STREAM("No planner has been loaded and initialized successfully!");
  }
  if(!recovery_ptr_->initialize())
  {
    ROS_WARN_STREAM("No recovery behavior has been loaded and initialized successfully!");
  }
}

AbstractNavigationServer::~AbstractNavigationServer()
{
  moving_ptr_->stopMoving();
  planning_ptr_->stopPlanning();
  recovery_ptr_->stopRecovery();
}

void AbstractNavigationServer::startActionServers()
{
  action_server_get_path_ptr_->start();
  action_server_exe_path_ptr_->start();
  action_server_recovery_ptr_->start();
  action_server_move_base_ptr_->start();
}

void AbstractNavigationServer::startDynamicReconfigureServer()
{
  // dynamic reconfigure server
  dsrv_ = boost::make_shared<dynamic_reconfigure::Server<mbf_abstract_nav::MoveBaseFlexConfig> >(private_nh_);
  dsrv_->setCallback(boost::bind(&AbstractNavigationServer::reconfigure, this, _1, _2));
}

void AbstractNavigationServer::reconfigure(
  mbf_abstract_nav::MoveBaseFlexConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

  // Make sure we have the original configuration the first time we're called, so we can restore it if needed
  if (!setup_reconfigure_)
  {
    default_config_ = config;
    setup_reconfigure_ = true;
  }

  if (config.restore_defaults)
  {
    config = default_config_;
    // if someone sets restore defaults on the parameter server, prevent looping
    config.restore_defaults = false;
  }

  planning_ptr_->reconfigure(config);
  moving_ptr_->reconfigure(config);
  recovery_ptr_->reconfigure(config);
  oscillation_timeout_ = ros::Duration(config.oscillation_timeout);
  oscillation_distance_ = config.oscillation_distance;
  recovery_enabled_ = config.recovery_enabled;

  last_config_ = config;
}

void AbstractNavigationServer::publishPath(
  std::vector<geometry_msgs::PoseStamped> &plan)
{
  if (plan.empty())
  {
    return;
  }
  nav_msgs::Path path;
  path.poses = plan;
  path.header.frame_id = plan.front().header.frame_id;
  path.header.stamp = plan.front().header.stamp;
  path_pub_.publish(path);
}

bool AbstractNavigationServer::transformPlanToGlobalFrame(
  std::vector<geometry_msgs::PoseStamped> &plan, std::vector<geometry_msgs::PoseStamped> &global_plan)
{
  global_plan.clear();
  std::vector<geometry_msgs::PoseStamped>::iterator iter;
  bool tf_success = false;
  for (iter = plan.begin(); iter != plan.end(); ++iter)
  {
    geometry_msgs::PoseStamped global_pose;
    tf_success = mbf_utility::transformPose(*tf_listener_ptr_, global_frame_, iter->header.stamp,
                                                 ros::Duration(tf_timeout_), *iter, global_frame_, global_pose);
    if (!tf_success)
    {
      ROS_ERROR_STREAM("Can not transform pose from the \"" << iter->header.frame_id << "\" frame into the \""
                                                            << global_frame_ << "\" frame !");
      return false;
    }
    global_plan.push_back(global_pose);
  }
  return true;
}

bool AbstractNavigationServer::getRobotPose(
  geometry_msgs::PoseStamped &robot_pose)
{
  bool tf_success = mbf_utility::getRobotPose(*tf_listener_ptr_, robot_frame_, global_frame_,
                                                   ros::Duration(tf_timeout_), robot_pose);
  robot_pose.header.stamp = ros::Time::now(); // would be 0 if not, as we ask tf listener for the last pose available
  if (!tf_success)
  {
    ROS_ERROR_STREAM("Can not get the robot pose in the global frame. - robot frame: \""
                     << robot_frame_ << "\"   global frame: \"" << global_frame_ << std::endl);
    return false;
  }
  return true;
}

void AbstractNavigationServer::callActionGetPath(
  const mbf_msgs::GetPathGoalConstPtr &goal)
{
  ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Start action "  << name_action_get_path);

  mbf_msgs::GetPathResult result;
  geometry_msgs::PoseStamped start_pose, goal_pose;

  result.path.header.seq = path_seq_count_++;
  result.path.header.frame_id = global_frame_;
  goal_pose = goal->target_pose;
  current_goal_pub_.publish(goal_pose);

  double tolerance = goal->tolerance;
  bool use_start_pose = goal->use_start_pose;

  // Try to switch the planner if a special planner is specified in the action goal.
  if(!goal->planner.empty()){

    if(planning_ptr_->switchPlanner(goal->planner))
    {
      ROS_INFO_STREAM("Using the planner \"" << goal->planner << "\".");
    }
    else
    {
      result.outcome = mbf_msgs::GetPathResult::INVALID_PLUGIN;
      result.message = "Could not switch to the planner \"" + goal->planner + "\"!";
      action_server_get_path_ptr_->setAborted(result, result.message);
      return;
    }
  }

  active_planning_ = true;

  if(use_start_pose)
  {
    start_pose = goal->start_pose;
    geometry_msgs::Point p = start_pose.pose.position;
    ROS_INFO_STREAM_NAMED(name_action_get_path, "Use the given start pose ("
                          << p.x << ", " << p.y << ", " << p.z << ").");
  }
  else
  {
    // get the current robot pose
    if (!getRobotPose(start_pose))
    {
      result.outcome = mbf_msgs::GetPathResult::TF_ERROR;
      result.message = "Could not get the current robot pose!";
      action_server_get_path_ptr_->setAborted(result, result.message);
      ROS_ERROR_STREAM_NAMED(name_action_get_path, result.message << " Canceling the action call.");
      return;
    }
    else
    {
      geometry_msgs::Point p = start_pose.pose.position;
      ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Got the current robot pose at ("
                             << p.x << ", " << p.y << ", " << p.z << ").");
    }
  }

  ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Starting the planning thread.");
  if (!planning_ptr_->startPlanning(start_pose, goal_pose, tolerance))
  {
    result.outcome = mbf_msgs::GetPathResult::INTERNAL_ERROR;
    result.message = "Another thread is still planning!";
    action_server_get_path_ptr_->setAborted(result, result.message);
    ROS_ERROR_STREAM_NAMED(name_action_get_path, result.message << " Canceling the action call.");
    return;
  }

  AbstractPlannerExecution::PlanningState state_planning_input;

  std::vector<geometry_msgs::PoseStamped> plan, global_plan;
  double cost;

  int feedback_cnt = 0;

  while (active_planning_ && ros::ok())
  {
    // get the current state of the planning thread
    state_planning_input = planning_ptr_->getState();

    switch (state_planning_input)
    {
      case AbstractPlannerExecution::INITIALIZED:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot_navigation state: initialized");
        break;

      case AbstractPlannerExecution::STARTED:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot_navigation state: started");
        break;

      case AbstractPlannerExecution::STOPPED:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot navigation state: stopped");
        ROS_WARN_STREAM_NAMED(name_action_get_path, "Planning has been stopped rigorously!");
        result.outcome = mbf_msgs::GetPathResult::STOPPED;
        result.message = "Global planner has been stopped!";
        action_server_get_path_ptr_->setAborted(result, result.message);
        active_planning_ = false;
        break;

      case AbstractPlannerExecution::CANCELED:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot navigation state: canceled");
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Global planner has been canceled successfully");
        result.path.header.stamp = ros::Time::now();
        result.outcome = mbf_msgs::GetPathResult::CANCELED;
        result.message = "Global planner has been preempted!";
        action_server_get_path_ptr_->setPreempted(result, result.message);
        active_planning_ = false;
        break;

        // in progress
      case AbstractPlannerExecution::PLANNING:
        if (planning_ptr_->isPatienceExceeded())
        {
          ROS_INFO_STREAM_NAMED(name_action_get_path, "Global planner patience has been exceeded! "
            << "Cancel planning...");
          if (!planning_ptr_->cancel())
          {
            ROS_WARN_STREAM_THROTTLE_NAMED(2.0, name_action_get_path, "Cancel planning failed or is not supported; "
              "must wait until current plan finish!");
          }
        }
        else
        {
          ROS_DEBUG_THROTTLE_NAMED(2.0, name_action_get_path, "robot navigation state: planning");
        }
        break;

        // found a new plan
      case AbstractPlannerExecution::FOUND_PLAN:
        // set time stamp to now
        result.path.header.stamp = ros::Time::now();
        plan = planning_ptr_->getPlan();
        publishPath(result.path.poses);

        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot navigation state: found plan with cost: " << cost);

        if (!transformPlanToGlobalFrame(plan, global_plan))
        {
          result.outcome = mbf_msgs::GetPathResult::TF_ERROR;
          result.message = "Cloud not transform the plan to the global frame!";

          ROS_ERROR_STREAM_NAMED(name_action_get_path, result.message << " Canceling the action call.");
          action_server_get_path_ptr_->setAborted(result, result.message);
          active_planning_ = false;
          break;
        }

        if (global_plan.empty())
        {
          result.outcome = mbf_msgs::GetPathResult::EMPTY_PATH;
          result.message = "Global planner returned an empty path!";

          ROS_ERROR_STREAM_NAMED(name_action_get_path, result.message);
          action_server_get_path_ptr_->setAborted(result, result.message);
          active_planning_ = false;
          break;
        }

        result.path.poses = global_plan;
        result.cost = planning_ptr_->getCost();
        result.outcome = planning_ptr_->getOutcome();
        result.message = planning_ptr_->getMessage();
        action_server_get_path_ptr_->setSucceeded(result, result.message);

        active_planning_ = false;
        break;

        // no plan found
      case AbstractPlannerExecution::NO_PLAN_FOUND:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot navigation state: no plan found");
        result.outcome = planning_ptr_->getOutcome();
        result.message = planning_ptr_->getMessage();
        action_server_get_path_ptr_->setAborted(result, result.message);
        active_planning_ = false;
        break;

      case AbstractPlannerExecution::MAX_RETRIES:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Global planner reached the maximum number of retries");
        result.outcome = planning_ptr_->getOutcome();
        result.message = planning_ptr_->getMessage();
        action_server_get_path_ptr_->setAborted(result, result.message);
        active_planning_ = false;
        break;

      case AbstractPlannerExecution::PAT_EXCEEDED:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Global planner exceeded the patience time");
        result.outcome = mbf_msgs::GetPathResult::PAT_EXCEEDED;
        result.message = "Global planner exceeded the patience time";
        action_server_get_path_ptr_->setAborted(result, result.message);
        active_planning_ = false;
        break;

      case AbstractPlannerExecution::INTERNAL_ERROR:
        ROS_FATAL_STREAM_NAMED(name_action_get_path, "Internal error: Unknown error thrown by the plugin!"); // TODO getMessage from planning
        active_recovery_ = false;
        result.outcome = mbf_msgs::GetPathResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown error thrown by the plugin!";
        action_server_get_path_ptr_->setAborted(result, result.message);
        break;

      default:
        result.outcome = mbf_msgs::GetPathResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown state in a move base flex planner execution with the number: " + state_planning_input;
        ROS_FATAL_STREAM_NAMED(name_action_get_path, result.message);
        action_server_get_path_ptr_->setAborted(result, result.message);
        active_planning_ = false;
    }

    // if preempt requested while we are planning
    if (action_server_get_path_ptr_->isPreemptRequested()
      && state_planning_input == AbstractPlannerExecution::PLANNING)
    {
      if (!planning_ptr_->cancel())
      {
        ROS_WARN_STREAM_THROTTLE_NAMED(2.0, name_action_get_path, "Cancel planning failed or is not supported; "
          << "Wait until the current plan finished");
      }
    }

    if (active_planning_)
    {
      // try to sleep a bit
      // normally this thread should be woken up from the planner execution thread
      // in order to transfer the results to the controller.
      boost::mutex mutex;
      boost::unique_lock<boost::mutex> lock(mutex);
      condition_.wait_for(lock, boost::chrono::milliseconds(500));
    }
  }  // while (active_planning_ && ros::ok())

  if (!active_planning_)
  {
    ROS_DEBUG_STREAM_NAMED(name_action_get_path, "\"GetPath\" action ended properly.");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_action_get_path, "\"GetPath\" action has been stopped!");
  }
}

void AbstractNavigationServer::callActionExePath(
  const mbf_msgs::ExePathGoalConstPtr &goal)
{
  ROS_DEBUG_STREAM_NAMED(name_action_exe_path, "Start action "  << name_action_exe_path);

  mbf_msgs::ExePathResult result;
  mbf_msgs::ExePathFeedback feedback;

  typename AbstractControllerExecution::ControllerState state_moving_input;

  std::vector<geometry_msgs::PoseStamped> plan = goal->path.poses;
  if (plan.empty())
  {
    result.outcome = mbf_msgs::ExePathResult::INVALID_PATH;
    result.message = "Local planner started with an empty plan!";
    action_server_exe_path_ptr_->setAborted(result, result.message);
    ROS_ERROR_STREAM_NAMED(name_action_exe_path, result.message << " Canceling the action call.");
    return;
  }

  // Try to switch the planner if a special planner is specified in the action goal.
  if(!goal->controller.empty()){

    if(moving_ptr_->switchController(goal->controller))
    {
      ROS_INFO_STREAM("Using the controller \"" << goal->controller << "\".");
    }
    else
    {
      result.outcome = mbf_msgs::ExePathResult::INVALID_PLUGIN;
      result.message = "Could not switch to the controller \"" + goal->controller + "\"!";
      action_server_exe_path_ptr_->setAborted(result, result.message);
      return;
    }
  }

  goal_pose_ = plan.back();
  ROS_DEBUG_STREAM_NAMED(name_action_exe_path, "Called action \""
    << name_action_exe_path << "\" with plan:" << std::endl
    << "frame: \"" << goal->path.header.frame_id << "\" " << std::endl
    << "stamp: " << goal->path.header.stamp << std::endl
    << "poses: " << goal->path.poses.size() << std::endl
    << "goal: (" << goal_pose_.pose.position.x << ", "
    << goal_pose_.pose.position.y << ", "
    << goal_pose_.pose.position.z << ")");

  moving_ptr_->setNewPlan(plan);
  moving_ptr_->startMoving();

  active_moving_ = true;

  geometry_msgs::PoseStamped oscillation_pose;
  ros::Time last_oscillation_reset = ros::Time::now();

  bool first_cycle = true;

  while (active_moving_ && ros::ok())
  {
    if (!getRobotPose(robot_pose_))
    {
      active_moving_ = false;
      result.outcome = mbf_msgs::ExePathResult::TF_ERROR;
      result.message = "Could not get the robot pose!";
      action_server_exe_path_ptr_->setAborted(result, result.message);
      ROS_ERROR_STREAM_NAMED(name_action_exe_path, result.message << " Canceling the action call.");
      break;
    }

    if (first_cycle)
    {
      // init oscillation pose
      oscillation_pose = robot_pose_;
    }

    // check preempt requested
    if (action_server_exe_path_ptr_->isPreemptRequested())
    {
      if (action_server_exe_path_ptr_->isNewGoalAvailable())
      {
        // This probably means that we are continuously replanning, so we don't stop navigation and log as DEBUG
        ROS_DEBUG_STREAM("Action \"ExePath\" preempted with a new path; switching...");
      }
      else
      {
        moving_ptr_->stopMoving();
        ROS_INFO_STREAM("Action \"ExePath\" preempted");
      }

      fillExePathResult(mbf_msgs::ExePathResult::CANCELED, "Local planner preempted", result);
      action_server_exe_path_ptr_->setPreempted(result, result.message);
      break;
    }


    state_moving_input = moving_ptr_->getState();

    switch (state_moving_input)
    {
      case AbstractControllerExecution::STOPPED:
        // TODO when this realy happens?   not when continuously replanning, for sure
        ROS_WARN_STREAM("The moving has been stopped!");
        break;

      case AbstractControllerExecution::STARTED:
        ROS_DEBUG_STREAM_NAMED(name_action_exe_path, "The moving has been started!");
        break;

      // in progress
      case AbstractControllerExecution::PLANNING:
        if (moving_ptr_->isPatienceExceeded())
        {
          ROS_DEBUG_STREAM_NAMED(name_action_exe_path, "Local planner patience has been exceeded! Stopping controller...");
          // TODO planner is stuck, but we don't have currently any way to cancel it!
          // We will try to stop the thread, but does nothing with DWA or TR controllers
          moving_ptr_->stopMoving();
        }
        break;

      case AbstractControllerExecution::MAX_RETRIES:
        ROS_WARN_STREAM_NAMED(name_action_exe_path, "The local planner has been aborted after it exceeded the maximum number of retries!");
        active_moving_ = false;
        fillExePathResult(moving_ptr_->getOutcome(), moving_ptr_->getMessage(), result);
        action_server_exe_path_ptr_->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::PAT_EXCEEDED:
        ROS_WARN_STREAM_NAMED(name_action_exe_path, "The local planner has been aborted after it exceeded the "
          << "patience time ");
        ROS_WARN("################################################################################");
        ROS_WARN(" PAT_EXCEEDED!  how I manage to provoke this?");
        ROS_WARN("################################################################################");

        active_moving_ = false;
        fillExePathResult(mbf_msgs::ExePathResult::PAT_EXCEEDED, "Local planner exceeded allocated time", result);
        action_server_exe_path_ptr_->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::NO_PLAN:
        ROS_WARN_STREAM_NAMED(name_action_exe_path, "The local planner has been started without any plan!");
        active_moving_ = false;
        fillExePathResult(mbf_msgs::ExePathResult::INVALID_PATH, "Local planner started without a path", result);
        action_server_exe_path_ptr_->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::EMPTY_PLAN:
        ROS_WARN_STREAM_NAMED(name_action_exe_path, "The local planner has received an empty plan");
        active_moving_ = false;
        fillExePathResult(mbf_msgs::ExePathResult::INVALID_PATH, "Local planner started with an empty plan", result);
        action_server_exe_path_ptr_->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::INVALID_PLAN:
        ROS_WARN_STREAM_NAMED(name_action_exe_path, "The local planner has received an invalid plan");
        active_moving_ = false;
        fillExePathResult(mbf_msgs::ExePathResult::INVALID_PATH, "Local planner started with an invalid plan", result);
        action_server_exe_path_ptr_->setAborted(result, result.message);
        break;

      case AbstractControllerExecution::NO_LOCAL_CMD:
        ROS_WARN_STREAM_THROTTLE_NAMED(3, name_action_exe_path, "No velocity command received from local planner!");
        break;

      case AbstractControllerExecution::GOT_LOCAL_CMD:
        if (!oscillation_timeout_.isZero() && !active_move_base_)
        {
          // check if oscillating only if move_base action is not active, as it has his own wider-scope detector
          if (mbf_utility::distance(robot_pose_, oscillation_pose) >= oscillation_distance_)
          {
            last_oscillation_reset = ros::Time::now();
            oscillation_pose = robot_pose_;
          }
          else if (last_oscillation_reset + oscillation_timeout_ < ros::Time::now())
          {
            ROS_WARN_STREAM_NAMED(name_action_exe_path, "The local planner is oscillating for "
              << (ros::Time::now() - last_oscillation_reset).toSec() << "s");
            moving_ptr_->stopMoving();
            active_moving_ = false;
            fillExePathResult(mbf_msgs::ExePathResult::OSCILLATION, "Oscillation detected!", result);
            action_server_exe_path_ptr_->setAborted(result, result.message);
            break;
          }
        }
        feedback.current_twist = moving_ptr_->getLastValidCmdVel();
        feedback.current_pose = robot_pose_;
        feedback.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal_pose_));
        feedback.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal_pose_));
        action_server_exe_path_ptr_->publishFeedback(feedback);
        break;

      case AbstractControllerExecution::ARRIVED_GOAL:
        ROS_DEBUG_STREAM_NAMED(name_action_exe_path, "Local planner succeeded; arrived to goal");
        active_moving_ = false;
        fillExePathResult(mbf_msgs::ExePathResult::SUCCESS, "Local planner succeeded; arrived to goal!", result);
        action_server_exe_path_ptr_->setSucceeded(result, result.message);
        break;

      case AbstractControllerExecution::INTERNAL_ERROR:
        ROS_FATAL_STREAM_NAMED(name_action_exe_path, "Internal error: Unknown error thrown by the plugin!"); // TODO getMessage from controller
        active_moving_ = false;
        fillExePathResult(mbf_msgs::ExePathResult::INTERNAL_ERROR, "Internal error: Unknown error thrown by the plugin!", result);
        action_server_exe_path_ptr_->setAborted(result, result.message);
        break;

      default:
        result.outcome = mbf_msgs::ExePathResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown state in a move base flex controller execution with the number: " + state_moving_input;
        ROS_FATAL_STREAM_NAMED(name_action_exe_path, result.message);
        action_server_exe_path_ptr_->setAborted(result, result.message);
        active_moving_ = false;
    }

    if (active_moving_)
    {
      // try to sleep a bit
      // normally this thread should be woken up from the controller execution thread
      // in order to transfer the results to the controller
      boost::mutex mutex;
      boost::unique_lock<boost::mutex> lock(mutex);
      condition_.wait_for(lock, boost::chrono::milliseconds(500));
    }

    first_cycle = false;
  }  // while (active_moving_ && ros::ok())

  if (!active_moving_)
  {
    ROS_DEBUG_STREAM_NAMED(name_action_exe_path, "\"ExePath\" action ended properly.");
  }
  else
  {
    // normal on continuous replanning
    ROS_DEBUG_STREAM_NAMED(name_action_exe_path, "\"ExePath\" action has been stopped!");
  }
}

void AbstractNavigationServer::callActionRecovery(
  const mbf_msgs::RecoveryGoalConstPtr &goal)
{
  ROS_DEBUG_STREAM_NAMED(name_action_recovery, "Start action "  << name_action_recovery);

  mbf_msgs::RecoveryResult result;
  std::string behavior = goal->behavior;
  recovery_ptr_->startRecovery(behavior);
  active_recovery_ = true;

  typename AbstractRecoveryExecution::RecoveryState state_recovery_input;

  while (active_recovery_ && ros::ok())
  {
    state_recovery_input = recovery_ptr_->getState();
    switch (state_recovery_input)
    {
      case AbstractRecoveryExecution::STOPPED:
        // Recovery behavior doesn't support or didn't answered to cancel and has been ruthlessly stopped
        ROS_WARN_STREAM("Recovering \"" << behavior << "\" exceeded the patience time and has been stopped!");
        active_recovery_ = false; // stopping the action
        result.outcome = mbf_msgs::RecoveryResult::CANCELED;
        result.message = "Recovery \"" + behavior + "\" exceeded the patience time";
        action_server_recovery_ptr_->setSucceeded(result, result.message);
        break;

      case AbstractRecoveryExecution::STARTED:
        ROS_DEBUG_STREAM_NAMED(name_action_recovery, "Recovering \"" << behavior << "\" was started");
        break;

      case AbstractRecoveryExecution::RECOVERING:
        // check preempt requested; we let for the next iteration to set the action as preempted to
        // give time to the executing thread to finish, set the CANCELED state and notify condition
        if (action_server_recovery_ptr_->isPreemptRequested() && recovery_ptr_->cancel())
        {
          ROS_DEBUG_STREAM_NAMED(name_action_recovery, "Recovering \"" << behavior << "\" canceled!");
        }
        else if (recovery_ptr_->isPatienceExceeded())
        {
          ROS_INFO_STREAM("Recovery behavior \"" << behavior << "\" patience exceeded! Cancel recovering...");
          if (!recovery_ptr_->cancel())
          {
            ROS_WARN_STREAM("Cancel recovering \"" << behavior << "\" failed or not supported; interrupt it!");
            recovery_ptr_->stopRecovery();
          }
        }
        else
        {
          // we keep silent if cancel failed because most recovery behaviors don't support canceling
          ROS_DEBUG_STREAM_THROTTLE_NAMED(3, name_action_move_base, "Recovering with: " << behavior);
        }
        break;

      case AbstractRecoveryExecution::WRONG_NAME:
        active_recovery_ = false; // stopping the action
        result.outcome = mbf_msgs::RecoveryResult::INVALID_NAME;
        result.message = "No recovery plugin loaded with the given name\"" + behavior + "\"!";
        action_server_recovery_ptr_->setAborted(result, result.message);
        ROS_ERROR_STREAM_NAMED(name_action_recovery, result.message);
        break;

      case AbstractRecoveryExecution::CANCELED:
        // Recovery behavior supports cancel and it worked
        active_recovery_ = false; // stopping the action
        result.outcome = mbf_msgs::RecoveryResult::CANCELED;
        result.message = "Recovering \"" + behavior + "\" preempted!";
        action_server_recovery_ptr_->setPreempted(result, result.message);
        ROS_DEBUG_STREAM_NAMED(name_action_recovery, result.message);
        break;

      case AbstractRecoveryExecution::RECOVERY_DONE:
        active_recovery_ = false; // stopping the action
        result.outcome = mbf_msgs::RecoveryResult::SUCCESS;
        result.message = "Recovery \"" + behavior + "\" done!";
        ROS_DEBUG_STREAM_NAMED(name_action_recovery, result.message);
        action_server_recovery_ptr_->setSucceeded(result, result.message);
        break;

      case AbstractRecoveryExecution::INTERNAL_ERROR:
        ROS_FATAL_STREAM_NAMED(name_action_recovery, "Internal error: Unknown error thrown by the plugin!"); // TODO getMessage from recovery
        active_recovery_ = false;
        result.outcome = mbf_msgs::RecoveryResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown error thrown by the plugin!";
        action_server_recovery_ptr_->setAborted(result, result.message);
        break;

      default:
        result.outcome = mbf_msgs::RecoveryResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown state in a move base flex recovery execution with the number: " + state_recovery_input;
        ROS_FATAL_STREAM_NAMED(name_action_recovery, result.message);
        action_server_recovery_ptr_->setAborted(result, result.message);
        active_recovery_ = false;
    }

    if (active_recovery_)
    {
      // try to sleep a bit
      // normally the thread should be woken up from the recovery unit
      // in order to transfer the results to the controller
      boost::mutex mutex;
      boost::unique_lock<boost::mutex> lock(mutex);
      condition_.wait_for(lock, boost::chrono::milliseconds(500));
    }
  }  // while (active_recovery_ && ros::ok())

  if (!active_recovery_)
  {
    ROS_DEBUG_STREAM_NAMED(name_action_recovery, "\"Recovery\" action ended properly.");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_action_recovery, "\"Recovery\" action has been stopped!");
  }
}

void AbstractNavigationServer::callActionMoveBase(
  const mbf_msgs::MoveBaseGoalConstPtr &goal)
{
  ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Start action "  << name_action_move_base);

  const geometry_msgs::PoseStamped target_pose = goal->target_pose;

  mbf_msgs::GetPathGoal get_path_goal;
  mbf_msgs::ExePathGoal exe_path_goal;
  mbf_msgs::RecoveryGoal recovery_goal;

  mbf_msgs::MoveBaseResult move_base_result;
  mbf_msgs::GetPathResult get_path_result;
  mbf_msgs::ExePathResult exe_path_result;
  mbf_msgs::RecoveryResult recovery_result;

  if(!goal->controller.empty() && !moving_ptr_->switchController(goal->controller))
  {
    std::stringstream ss;
    ss << "No controller with the name \"" << goal->controller << "\" loaded! ";
    ROS_ERROR_STREAM_NAMED(name_action_move_base, ss.str() << " Please load the controller before using it!");
    move_base_result.outcome = mbf_msgs::MoveBaseResult::INVALID_PLUGIN;
    move_base_result.message = ss.str();
    action_server_move_base_ptr_->setAborted(move_base_result, ss.str());
    return;
  }

  if(!goal->planner.empty() && !planning_ptr_->switchPlanner(goal->planner))
  {
    std::stringstream ss;
    ss << "No planner with the name \"" << goal->planner << "\" loaded! ";
    ROS_ERROR_STREAM_NAMED(name_action_move_base, ss.str() << " Please load the planner before using it!");
    move_base_result.outcome = mbf_msgs::MoveBaseResult::INVALID_PLUGIN;
    move_base_result.message = ss.str();
    action_server_move_base_ptr_->setAborted(move_base_result, ss.str());
    return;
  }

  for(std::vector<std::string>::const_iterator iter = goal->recovery_behaviors.begin();
      iter != goal->recovery_behaviors.end(); ++iter)
  {
    if(!recovery_ptr_->hasRecoveryBehavior(*iter))
    {
      std::stringstream ss;
      ss << "No recovery behavior with the name \"" << *iter << "\" loaded! ";
      ROS_ERROR_STREAM_NAMED(name_action_move_base, ss.str() << " Please load the behaviors before using them!");
      move_base_result.outcome = mbf_msgs::MoveBaseResult::INVALID_PLUGIN;
      move_base_result.message = ss.str();
      action_server_move_base_ptr_->setAborted(move_base_result, ss.str());
      return;
    }
  }


  get_path_goal.target_pose = target_pose;
  get_path_goal.use_start_pose = false; // use the robot pose

  ros::Duration connection_timeout(1.0);

  // start recovering with the first behavior, use the recovery behaviors from the action request, if specified,
  // otherwise all loaded behaviors.
  std::vector<std::string> recovery_behaviors =
    goal->recovery_behaviors.empty() ? recovery_ptr_->listRecoveryBehaviors() : goal->recovery_behaviors;
  std::vector<std::string>::iterator current_recovery_behavior = recovery_behaviors.begin();

  // get the current robot pose only at the beginning, as exe_path will keep updating it as we move
  if (!getRobotPose(robot_pose_))
  {
    ROS_ERROR_STREAM_NAMED(name_action_move_base, "Could not get the current robot pose!");
    move_base_result.message = "Could not get the current robot pose!";
    move_base_result.outcome = mbf_msgs::MoveBaseResult::TF_ERROR;
    action_server_move_base_ptr_->setAborted(move_base_result, move_base_result.message);
    return;
  }

  enum MoveBaseActionState
  {
    NONE,
    GET_PATH,
    EXE_PATH,
    RECOVERY,
    OSCILLATING
  };

  // wait for server connections
  if (!action_client_get_path_.waitForServer(connection_timeout) ||
      !action_client_exe_path_.waitForServer(connection_timeout) ||
      !action_client_recovery_.waitForServer(connection_timeout))
  {
    ROS_ERROR_STREAM_NAMED(name_action_move_base, "Could not connect to one or more of move_base_flex actions:"
                           << "\"" << name_action_get_path
                           << "\", " << "\"" << name_action_exe_path
                           << "\", " << "\"" << name_action_recovery << "\"!");
    move_base_result.outcome = mbf_msgs::MoveBaseResult::INTERNAL_ERROR;
    move_base_result.message = "Could not connect to the move_base_flex actions!";
    action_server_move_base_ptr_->setAborted(move_base_result, move_base_result.message);
    return;
  }

  active_move_base_ = true;

  // call get_path action server to get a first plan
  action_client_get_path_.sendGoal(get_path_goal);

  //set up the planner's thread
  bool has_new_plan = false;
  boost::mutex planner_mutex;
  boost::condition_variable planner_cond;
  boost::unique_lock<boost::mutex> planner_lock(planner_mutex);
  boost::thread planner_thread(boost::bind(&AbstractNavigationServer::plannerThread, this,
                                           boost::ref(planner_cond), boost::ref(planner_lock),
                                           boost::ref(get_path_goal), boost::ref(get_path_result),
                                           boost::ref(has_new_plan)));

  // init goal states with dummy values;
  actionlib::SimpleClientGoalState get_path_state(actionlib::SimpleClientGoalState::PENDING);
  actionlib::SimpleClientGoalState exe_path_state(actionlib::SimpleClientGoalState::PENDING);
  actionlib::SimpleClientGoalState recovery_state(actionlib::SimpleClientGoalState::PENDING);

  MoveBaseActionState state = GET_PATH;
  MoveBaseActionState recovery_trigger = NONE;

  bool run = true;
  bool preempted = false;
  ros::Duration wait(0.05);

  // we create a navigation-level oscillation detection independent of the exe_path action one,
  // as the later doesn't handle oscillations created by quickly failing repeated plans
  geometry_msgs::PoseStamped oscillation_pose = robot_pose_;
  ros::Time last_oscillation_reset = ros::Time::now();

  std::string type; // recovery behavior type

  while (ros::ok() && run)
  {
    bool try_recovery = false;
    switch (state)
    {
      case GET_PATH:
        if (!action_client_get_path_.waitForResult(wait))
        { // no result -> action server is still running
          if (action_server_move_base_ptr_->isPreemptRequested() && !preempted)
          {
            action_client_get_path_.cancelGoal();
            preempted = true;
          }
        }
        else
        {
          get_path_state = action_client_get_path_.getState();
          switch (get_path_state.state_)
          {
            case actionlib::SimpleClientGoalState::PENDING:
              // TODO -> not implemented // should not be reached!
              break;

            case actionlib::SimpleClientGoalState::SUCCEEDED:

              get_path_result = *action_client_get_path_.getResult();
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Action \""
                << name_action_move_base << "\" received a path from \""
                << name_action_get_path << "\": " << get_path_state.getText());

              exe_path_goal.path = get_path_result.path;
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Action \""
                << name_action_move_base << "\" sends the path to \""
                << name_action_exe_path << "\".");

              if (recovery_trigger == GET_PATH)
              {
                ROS_INFO_NAMED(name_action_move_base, "Recovered from planner failure: restart recovery behaviors");
                current_recovery_behavior = recovery_behaviors.begin();
                recovery_trigger = NONE;
              }

              action_client_exe_path_.sendGoal(
                exe_path_goal,
                ActionClientExePath::SimpleDoneCallback(),
                ActionClientExePath::SimpleActiveCallback(),
                boost::bind(&mbf_abstract_nav::AbstractNavigationServer::actionMoveBaseExePathFeedback, this, _1));

              planner_cond.notify_one();

              state = EXE_PATH;
              break;

            case actionlib::SimpleClientGoalState::ABORTED:
              get_path_result = *action_client_get_path_.getResult();

              // copy result from get_path action
              move_base_result.outcome = get_path_result.outcome;
              move_base_result.message = get_path_result.message;
              move_base_result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, target_pose));
              move_base_result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, target_pose));
              move_base_result.final_pose = robot_pose_;

              if (!recovery_enabled_)
              {
                ROS_WARN_STREAM_NAMED(name_action_move_base, "Recovery behaviors are disabled!");
                ROS_WARN_STREAM_NAMED(name_action_move_base, "Abort the execution of the planner: "
                  << get_path_result.message);
                run = false;
                action_server_move_base_ptr_->setAborted(move_base_result, get_path_state.getText());
                break;
              }
              else if (current_recovery_behavior == recovery_behaviors.end())
              {
                if (recovery_behaviors.empty())
                {
                  ROS_WARN_STREAM_NAMED(name_action_move_base, "No Recovery Behaviors loaded! Abort controlling: "
                    << exe_path_result.message);
                }
                else
                {
                  ROS_WARN_STREAM_NAMED(name_action_move_base, "Executed all available recovery behaviors! "
                    << "Abort planning: " << get_path_result.message);
                }
                run = false;
                action_server_move_base_ptr_->setAborted(move_base_result, get_path_state.getText());
                break;
              }
              else
              {
                recovery_goal.behavior = *current_recovery_behavior;
                recovery_ptr_->getTypeOfBehavior(*current_recovery_behavior, type);
                ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Start recovery behavior\""
                  << *current_recovery_behavior << "\" of the type \"" << type << "\".");
                action_client_recovery_.sendGoal(recovery_goal);
                recovery_trigger = GET_PATH;
                state = RECOVERY;
              }
              break;

            case actionlib::SimpleClientGoalState::PREEMPTED:
              // the get_path action has been preempted.
              get_path_result = *action_client_get_path_.getResult();

              // copy result from get_path action
              move_base_result.outcome = get_path_result.outcome;
              move_base_result.message = get_path_result.message;
              move_base_result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, target_pose));
              move_base_result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, target_pose));
              move_base_result.final_pose = robot_pose_;
              run = false;
              action_server_move_base_ptr_->setPreempted(move_base_result, get_path_state.getText());
              break;

            case actionlib::SimpleClientGoalState::RECALLED:
            case actionlib::SimpleClientGoalState::REJECTED:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "The states RECALLED and REJECTED are not implemented in the SimpleActionServer!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;

            case actionlib::SimpleClientGoalState::LOST:
              // TODO
              break;

            default:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "Reached unreachable case! Unknown SimpleActionServer state!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;
          }
        }

        break;

      case EXE_PATH:

        if (has_new_plan)
        {
          ROS_DEBUG("Have new plan; restarting moving action");
          exe_path_goal.path = get_path_result.path;
          action_client_exe_path_.sendGoal(
            exe_path_goal,
            ActionClientExePath::SimpleDoneCallback(),
            ActionClientExePath::SimpleActiveCallback(),
            boost::bind(&mbf_abstract_nav::AbstractNavigationServer::actionMoveBaseExePathFeedback, this, _1));

          has_new_plan = false;
        }
        planner_cond.notify_one();

        if (!action_client_exe_path_.waitForResult(wait))
        {
          // no result -> action server is still running

          if (!oscillation_timeout_.isZero())
          {
            // check if oscillating
            if (mbf_utility::distance(robot_pose_, oscillation_pose) >= oscillation_distance_)
            {
              last_oscillation_reset = ros::Time::now();
              oscillation_pose = robot_pose_;

              if (recovery_trigger == OSCILLATING)
              {
                ROS_INFO_NAMED(name_action_move_base, "Recovered from robot oscillation: restart recovery behaviors");
                current_recovery_behavior = recovery_behaviors.begin();
                recovery_trigger = NONE;
              }
            }
            else if (last_oscillation_reset + oscillation_timeout_ < ros::Time::now())
            {
              ROS_WARN_STREAM_NAMED(name_action_exe_path, "Robot is oscillating for "
                << (ros::Time::now() - last_oscillation_reset).toSec() << "s");
              last_oscillation_reset = ros::Time::now();
              moving_ptr_->stopMoving();

              recovery_trigger = OSCILLATING;
              try_recovery = true;
            }
          }

          if (recovery_trigger == EXE_PATH && moving_ptr_->isMoving())
          {
            ROS_INFO_NAMED(name_action_move_base, "Recovered from controller failure: restart recovery behaviors");
            current_recovery_behavior = recovery_behaviors.begin();
            recovery_trigger = NONE;
          }

          if (action_server_move_base_ptr_->isPreemptRequested() && !preempted)
          {
            action_client_exe_path_.cancelGoal();
            preempted = true;
          }
        }
        else
        {
          exe_path_state = action_client_exe_path_.getState();
          switch (exe_path_state.state_)
          {
            // copy result from get_path action
            exe_path_result = *action_client_exe_path_.getResult();
            move_base_result.outcome = exe_path_result.outcome;
            move_base_result.message = exe_path_result.message;
            move_base_result.dist_to_goal = exe_path_result.dist_to_goal;
            move_base_result.angle_to_goal = exe_path_result.angle_to_goal;
            move_base_result.final_pose = exe_path_result.final_pose;

            case actionlib::SimpleClientGoalState::PENDING:
              //TODO
              break;

            case actionlib::SimpleClientGoalState::SUCCEEDED:
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Action \""
                << name_action_move_base << "\" received a result from \""
                << name_action_exe_path << "\": " << exe_path_state.getText());
              exe_path_goal.path = get_path_result.path;
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Action \"" << name_action_move_base << "\" succeeded.");

              move_base_result.outcome = mbf_msgs::MoveBaseResult::SUCCESS;
              move_base_result.message = "MoveBase action succeeded!";
              action_server_move_base_ptr_->setSucceeded(move_base_result, move_base_result.message);
              run = false;
              break;

            case actionlib::SimpleClientGoalState::ABORTED:
              switch (exe_path_result.outcome)
              {
                case mbf_msgs::ExePathResult::INVALID_PATH:
                case mbf_msgs::ExePathResult::TF_ERROR:
                case mbf_msgs::ExePathResult::CANCELED:
                case mbf_msgs::ExePathResult::NOT_INITIALIZED:
                case mbf_msgs::ExePathResult::INVALID_PLUGIN:
                case mbf_msgs::ExePathResult::INTERNAL_ERROR:
                  // none of these errors is recoverable
                  run = false;
                  action_server_move_base_ptr_->setAborted(move_base_result, exe_path_state.getText());
                  break;

                default:
                  // all the rest are, so we start calling the recovery behaviors in sequence
                  recovery_trigger = EXE_PATH;
                  try_recovery = true;
                  break;
              }
              break;

            case actionlib::SimpleClientGoalState::PREEMPTED:
              // action was preempted successfully!
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "The action \""
                << name_action_move_base << "\" was preempted successfully!");
              action_server_move_base_ptr_->setPreempted();
              run = false;
              break;

            case actionlib::SimpleClientGoalState::RECALLED:
            case actionlib::SimpleClientGoalState::REJECTED:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "The states RECALLED and REJECTED are not implemented in the SimpleActionServer!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;

            case actionlib::SimpleClientGoalState::LOST:
              // TODO
              break;

            default:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "Reached unreachable case! Unknown SimpleActionServer state!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;
          }
        }

        if (try_recovery)
        {
          state = RECOVERY;
          if (!recovery_enabled_)
          {
            ROS_WARN_STREAM_NAMED(name_action_move_base, "Recovery behaviors are disabled!");
            ROS_WARN_STREAM_NAMED(name_action_move_base, "Abort the execution of the controller: "
              << exe_path_result.message);
            run = false;
            action_server_move_base_ptr_->setAborted(move_base_result, exe_path_state.getText());
            break;
          }
          else if (current_recovery_behavior == recovery_behaviors.end())
          {
            if (recovery_behaviors.empty())
            {
              ROS_WARN_STREAM_NAMED(name_action_move_base, "No Recovery Behaviors loaded! Abort controlling: "
                << exe_path_result.message);
            }
            else
            {
              ROS_WARN_STREAM_NAMED(name_action_move_base,
                                    "Executed all available recovery behaviors! Abort controlling: "
                                      << exe_path_result.message);
            }
            run = false;
            action_server_move_base_ptr_->setAborted(move_base_result, exe_path_state.getText());
            break;
          }
          else
          {
            recovery_goal.behavior = *current_recovery_behavior;
            recovery_ptr_->getTypeOfBehavior(*current_recovery_behavior, type);
            ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Start recovery behavior\""
              << *current_recovery_behavior << "\" of the type \"" << type << "\".");
            action_client_recovery_.sendGoal(recovery_goal);
            state = RECOVERY;
          }
        }
        break;

      case RECOVERY:
        if (!action_client_recovery_.waitForResult(wait))
        {
          if (action_server_move_base_ptr_->isPreemptRequested() && !preempted)
          {
            preempted = true;
            action_client_recovery_.cancelGoal();
          }
        }
        else
        {
          recovery_state = action_client_recovery_.getState();
          switch (recovery_state.state_)
          {
            case actionlib::SimpleClientGoalState::PENDING:
              //TODO
              break;
            case actionlib::SimpleClientGoalState::ABORTED:
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Recovery behavior aborted!");
              recovery_result = *action_client_recovery_.getResult();
              recovery_ptr_->getTypeOfBehavior(*current_recovery_behavior, type);
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "The recovery behavior \""
                << *current_recovery_behavior << "\"" << "of the type \"" << type << "\" failed. ");
              ROS_DEBUG_STREAM("Recovery behavior message: " << recovery_result.message
                                                             << ", outcome: " << recovery_result.outcome);

              current_recovery_behavior++; // use next behavior;
              if (current_recovery_behavior == recovery_behaviors.end())
              {
                ROS_DEBUG_STREAM_NAMED(name_action_move_base,
                                       "All recovery behaviours failed. Abort recovering and abort the move_base action");
                action_server_move_base_ptr_->setAborted(move_base_result, "All recovery behaviors failed.");
                run = false;
              }
              else
              {
                recovery_goal.behavior = *current_recovery_behavior;
                recovery_ptr_->getTypeOfBehavior(*current_recovery_behavior, type);

                ROS_INFO_STREAM_NAMED(name_action_move_base, "Run the next recovery behavior\""
                  << *current_recovery_behavior << "\" of the type \"" << type << "\".");
                action_client_recovery_.sendGoal(recovery_goal);
              }
              break;
            case actionlib::SimpleClientGoalState::SUCCEEDED:
              recovery_result = *action_client_recovery_.getResult();
              //go to planning state
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Execution of the recovery behavior \""
                << *current_recovery_behavior << "\" succeeded!");
              ROS_DEBUG_STREAM_NAMED(name_action_move_base,
                                     "Try planning again and increment the current recovery behavior in the list.");
              state = GET_PATH;
              current_recovery_behavior++; // use next behavior, the next time;
              action_client_get_path_.sendGoal(get_path_goal);
              break;
            case actionlib::SimpleClientGoalState::PREEMPTED:
              run = false;
              action_server_move_base_ptr_->setPreempted();
              break;
            case actionlib::SimpleClientGoalState::RECALLED:
            case actionlib::SimpleClientGoalState::REJECTED:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "The states RECALLED and REJECTED are not implemented in the SimpleActionServer!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;
            case actionlib::SimpleClientGoalState::LOST:
            default:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "Reached unreachable case! Unknown SimpleActionServer state!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;
          }
        }

        break;

      default:
        move_base_result.outcome = mbf_msgs::MoveBaseResult::INTERNAL_ERROR;
        move_base_result.message = "Reached a undefined case! Please report the bug!";
        action_server_move_base_ptr_->setAborted(move_base_result, move_base_result.message);
        ROS_FATAL_STREAM_NAMED(name_action_move_base, move_base_result.message);
        break;
    }
  }

  planner_thread.interrupt();
  planner_thread.join();

  active_move_base_ = false;
}

void AbstractNavigationServer::plannerThread(boost::condition_variable &cond, boost::unique_lock<boost::mutex> &lock,
                                             const mbf_msgs::GetPathGoal &goal, mbf_msgs::GetPathResult &result,
                                             bool &has_new_plan)
{
  if (planning_ptr_->getFrequency() <= 0.0)
    return;

  ros::Rate rate(planning_ptr_->getFrequency());  // TODO: will ignore dyn. reconf. until next run

  while (ros::ok())
  {
    ROS_DEBUG("Planner thread waiting...");
    cond.wait(lock);
    ROS_DEBUG("Planner thread started!");

    // keep calling get_path action server at planner_frequency Hz to get updated plans
    action_client_get_path_.sendGoal(goal);
    action_client_get_path_.waitForResult();
    result = *action_client_get_path_.getResult();
    if (result.outcome < 10)
      has_new_plan = true;
    rate.sleep();
  }
}

void AbstractNavigationServer::actionMoveBaseExePathFeedback(
  const mbf_msgs::ExePathFeedbackConstPtr &feedback)
{
  mbf_msgs::MoveBaseFeedback feedback_out;
  feedback_out.angle_to_goal = feedback->angle_to_goal;
  feedback_out.dist_to_goal = feedback->dist_to_goal;
  feedback_out.current_pose = feedback->current_pose;
  feedback_out.current_twist = feedback->current_twist;
  action_server_move_base_ptr_->publishFeedback(feedback_out);
}

void AbstractNavigationServer::fillExePathResult(uint32_t outcome, const std::string &message,
                                                 mbf_msgs::ExePathResult &result)
{
  result.outcome = outcome;
  result.message = message;
  result.final_pose = robot_pose_;
  result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal_pose_));
  result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal_pose_));
}


} /* namespace mbf_abstract_nav */
