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

#ifndef MOVE_BASE_FLEX__IMPL__ABSTRACT_NAVIGATION_SERVER_TCC_
#define MOVE_BASE_FLEX__IMPL__ABSTRACT_NAVIGATION_SERVER_TCC_

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

namespace move_base_flex
{

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::AbstractNavigationServer(
      const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
      typename AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::Ptr planning_ptr,
      typename AbstractControllerExecution<LOCAL_PLANNER_BASE>::Ptr moving_ptr,
      typename AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::Ptr recovery_ptr) :
      tf_listener_ptr_(tf_listener_ptr), planning_ptr_(planning_ptr), moving_ptr_(moving_ptr), recovery_ptr_(recovery_ptr)
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
    private_nh.param("global_frame", global_frame_, std::string("map"));
    private_nh.param("goal_tolerance", goal_tolerance_, 0.2);
    private_nh.param("tf_timeout", tf_timeout_, 3.0);

    path_pub_ = nh.advertise<nav_msgs::Path>("global_path", 1);

    // oscillation timeout and distance
    double oscillation_timeout;
    private_nh.param("oscillation_timeout", oscillation_timeout, 0.0);
    oscillation_timeout_ = ros::Duration(oscillation_timeout);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.02);

    action_server_get_path_ptr_ = ActionServerGetPathPtr(
        new ActionServerGetPath(
            private_nh,
            name_action_get_path,
            boost::bind(&move_base_flex::AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE,
                        RECOVERY_BEHAVIOR_BASE>::callActionGetPath, this, _1),
            false));

    action_server_exe_path_ptr_ = ActionServerExePathPtr(
        new ActionServerExePath(
            private_nh,
            name_action_exe_path,
            boost::bind(&move_base_flex::AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE,
                        RECOVERY_BEHAVIOR_BASE>::callActionExePath, this, _1),
            false));

    action_server_recovery_ptr_ = ActionServerRecoveryPtr(
        new ActionServerRecovery(
            private_nh,
            name_action_recovery,
            boost::bind(&move_base_flex::AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE,
                        RECOVERY_BEHAVIOR_BASE>::callActionRecovery, this, _1),
            false));

    action_server_move_base_ptr_ = ActionServerMoveBasePtr(
        new ActionServerMoveBase(
            private_nh,
            name_action_move_base,
            boost::bind(&move_base_flex::AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE,
                        RECOVERY_BEHAVIOR_BASE>::callActionMoveBase, this, _1),
            false));

    // dynamic reconfigure server
    dsrv_ = boost::make_shared<dynamic_reconfigure::Server<move_base_flex::MoveBaseFlexConfig> >(private_nh);
    dynamic_reconfigure::Server<move_base_flex::MoveBaseFlexConfig>::CallbackType cb =
        boost::bind(&AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::reconfigure,
                    this, _1, _2);
    dsrv_->setCallback(cb);
  }

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  void AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::initializeServerComponents()
  {
    planning_ptr_->initialize();
    moving_ptr_->initialize();
    recovery_ptr_->initialize();
  }

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::~AbstractNavigationServer()
  {
    moving_ptr_->stopMoving();
    planning_ptr_->stopPlanning();
    recovery_ptr_->stopRecovery();
  }

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  void AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::startActionServers()
  {
    action_server_get_path_ptr_->start();
    action_server_exe_path_ptr_->start();
    action_server_recovery_ptr_->start();
    action_server_move_base_ptr_->start();
  }

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  void AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::reconfigure(
      move_base_flex::MoveBaseFlexConfig &config, uint32_t level)
  {
    boost::recursive_mutex::scoped_lock sl(configuration_mutex_);
    // The first time we're called, we just want to make sure we have the original configuration
    if (!setup_reconfigure_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_reconfigure_ = true;
      return;
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
    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;

    last_config_ = config;
  }

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  void AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::publishPath(
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

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  bool AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::transformPlanToGlobalFrame(
      std::vector<geometry_msgs::PoseStamped> &plan, std::vector<geometry_msgs::PoseStamped> &global_plan)
  {
    global_plan.clear();
    std::vector<geometry_msgs::PoseStamped>::iterator iter;
    bool tf_success = false;
    for (iter = plan.begin(); iter != plan.end(); ++iter)
    {
      geometry_msgs::PoseStamped global_pose;
      tf_success = move_base_flex::transformPose(*tf_listener_ptr_, global_frame_, iter->header.stamp,
                                                 ros::Duration(tf_timeout_), *iter, global_frame_, global_pose);
      if (!tf_success)
      {
        ROS_ERROR("Can not transform pose from %s frame into %s frame !", iter->header.frame_id.c_str(),
                  global_frame_.c_str());
        return false;
      }
      global_plan.push_back(global_pose);
    }
    return true;
  }

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  bool AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::getRobotPose(
      geometry_msgs::PoseStamped &robot_pose)
  {
    bool tf_success = move_base_flex::getRobotPose(*tf_listener_ptr_, robot_frame_, global_frame_,
                                                   ros::Duration(tf_timeout_), robot_pose);

    if (!tf_success)
    {
      ROS_ERROR_STREAM("Can not get the robot pose in the global frame. - robot frame: \""
                       << robot_frame_ << "\"   global frame: \"" << global_frame_ << std::endl);
      return false;
    }
    return true;
  }

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  void AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::callActionGetPath(
      const move_base_flex_msgs::GetPathGoalConstPtr &goal)
  {
    ROS_INFO_STREAM("Starting action \"get_path\"");
    ROS_INFO_STREAM( goal->target_pose );
    ROS_INFO_STREAM( goal->tolerance );

    move_base_flex_msgs::GetPathResult result;

    geometry_msgs::PoseStamped start_pose, goal_pose;
    bool use_start_pose;

    result.path.header.seq = 0; // TODO check for a more meaningful id
    result.path.header.frame_id = global_frame_;

    goal_pose = goal->target_pose;
    use_start_pose = goal->use_start_pose;

    ROS_INFO("GOAL POSE: %f %f %f",goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);

    //current_goal_pub_.publish(goal_pose);

    double tolerance = goal->tolerance ; // TODO add tolerance field to action

    active_planning_ = true;

    // get the current robot pose
    if(!use_start_pose)
    {
      // dont use given start pose. get current robot pose from tf
      if (!getRobotPose(start_pose))
      {
        ROS_ERROR_STREAM("Failed to get the current robot pose!");
        action_server_get_path_ptr_->setAborted(result, "Could not get the current robot pose!");
        active_planning_ = false;
      }
      else
      {
        geometry_msgs::Point p = start_pose.pose.position;
        ROS_INFO_STREAM("Got the current robot pose at (" << p.x << ", " << p.y << ", " << p.z << ")");
        ROS_INFO_STREAM("Starting the planning thread");
        if (!planning_ptr_->startPlanning(start_pose, goal_pose, tolerance))
        {
          ROS_ERROR_STREAM("Another thread is still planning. Canceling the service call.");
          return;
        }
      }
    }else{
      // use given start pose
      geometry_msgs::Point p = goal->start_pose.pose.position;
      ROS_INFO_STREAM("Got the current robot pose at (" << p.x << ", " << p.y << ", " << p.z << ")");
      ROS_INFO_STREAM("Starting the planning thread");
      if (!planning_ptr_->startPlanning(start_pose, goal_pose, tolerance))
      {
        ROS_ERROR_STREAM("Another thread is still planning. Canceling the service call.");
        return;
      }
    }
    

    typename AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::PlanningState state_planning_input;

    std::vector<geometry_msgs::PoseStamped> plan, global_plan;
    double costs;

    int feedback_cnt = 0;

    while (active_planning_ && ros::ok())
    {
      // get the current state of the planning thread
      state_planning_input = planning_ptr_->getState();

      switch (state_planning_input)
      {
        case AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::INITIALIZED:
          ROS_INFO("robot_navigation state: initialized");
          break;

        case AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::STARTED:
          ROS_INFO("robot_navigation state: started");
          break;

        case AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::STOPPED:
          ROS_INFO("robot navigation state: stopped");
          ROS_INFO("the planning has been aborted!");
          result.path.header.stamp = ros::Time::now();
          result.server_code = move_base_flex_msgs::GetPathResult::STOPPED;
          result.server_msg = "Global planner has been stopped!";
          planning_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
          action_server_get_path_ptr_->setAborted(result, result.server_msg);
          active_planning_ = false;
          break;

        case AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::CANCELED:
          ROS_INFO("robot navigation state: canceled");
          ROS_INFO("Global planner has been canceled successfully");
          result.path.header.stamp = ros::Time::now();
          result.server_code = move_base_flex_msgs::GetPathResult::CANCELED;
          result.server_msg = "Global planner has been preempted!";
          planning_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
          action_server_get_path_ptr_->setPreempted(result, result.server_msg);
          active_planning_ = false;
          break;

          // in progress
        case AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::PLANNING:
          if (planning_ptr_->isPatienceExceeded())
          {
            ROS_INFO_STREAM("Global planner patience has been exceeded! Cancel planning...");
            if (!planning_ptr_->cancel())
            {
              ROS_WARN_THROTTLE(2.0, "Cancel planning failed or is not supported; must wait until current plan finish!");
            }
          }
          else
          {
            ROS_INFO_THROTTLE(2.0, "robot navigation state: planning");
          }
          break;

          // found a new plan
        case AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::FOUND_PLAN:
          // set time stamp to now
          result.path.header.stamp = ros::Time::now();

          ROS_INFO("robot navigation state: found plan");

          planning_ptr_->getNewPlan(plan, costs);
          planning_ptr_->stopPlanning();
          planning_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);

          if (costs > 0)
          {
            ROS_INFO_STREAM("Found a path with the costs: " << costs);
          }

          if (!transformPlanToGlobalFrame(plan, global_plan))
          {
            result.server_code = move_base_flex_msgs::GetPathResult::TF_ERROR;
            result.server_msg = "Cloud not transform the plan to the global frame!";

            ROS_ERROR_STREAM(result.server_msg);
            action_server_get_path_ptr_->setAborted(result, result.server_msg);
            active_planning_ = false;
            break;
          }

          if (global_plan.empty())
          {
            result.server_code = move_base_flex_msgs::GetPathResult::EMPTY_PATH;
            result.server_msg = "Global planner returned an empty path!";

            ROS_ERROR_STREAM(result.server_msg);
            action_server_get_path_ptr_->setAborted(result, result.server_msg);
            active_planning_ = false;
            break;
          }

          result.path.poses = global_plan;
          result.server_msg = move_base_flex_msgs::GetPathResult::FOUND_PATH;
          result.server_msg = "Succeeded: Found a valid plan!";
          action_server_get_path_ptr_->setSucceeded(result, result.server_msg);

          active_planning_ = false;
          break;

          // no plan found
        case AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::NO_PLAN_FOUND:
          ROS_INFO("robot navigation state: no plan found");
          result.path.header.stamp = ros::Time::now();
          result.server_code = move_base_flex_msgs::GetPathResult::NO_PATH_FOUND;
          result.server_msg = "No path found";
          planning_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
          action_server_get_path_ptr_->setAborted(result, result.server_msg);
          active_planning_ = false;
          break;

        case AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::MAX_RETRIES:
          ROS_INFO("Global planner reached the maximum number of retries");
          result.path.header.stamp = ros::Time::now();
          result.server_code = move_base_flex_msgs::GetPathResult::MAX_RETRIES;
          result.server_msg = "Global planner reached the maximum number of retries";
          planning_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
          action_server_get_path_ptr_->setAborted(result, result.server_msg);
          active_planning_ = false;
          break;

        case AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::PAT_EXCEEDED:
          ROS_INFO("Global planner exceeded the patience time");
          result.path.header.stamp = ros::Time::now();
          result.server_code = move_base_flex_msgs::GetPathResult::PAT_EXCEEDED;
          result.server_msg = "Global planner exceeded the patience time";
          planning_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
          action_server_get_path_ptr_->setAborted(result, result.server_msg);
          active_planning_ = false;
          break;

        default:
          ROS_FATAL_STREAM("Unknown state in move base flex controller with the number:" << state_planning_input);
          active_planning_ = false;
      }

      // if preempt requested while we are planning
      if (action_server_get_path_ptr_->isPreemptRequested()
          && state_planning_input == AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::PLANNING)
      {
        if (!planning_ptr_->cancel())
        {
          ROS_WARN_THROTTLE(2.0, "Cancel planning failed or is not supported; Wait until the current plan finished");
        }
        /// XXX  this crash with SBPL planner      planning_ptr_->stopPlanning();
      }

      if (active_planning_)
      {
        // try to sleep a bit
        // normally the thread should be woken up from the moving unit
        // in order to transfer the results to the controller.
        boost::mutex mutex;
        boost::unique_lock<boost::mutex> lock(mutex);
        condition_.wait_for(lock, boost::chrono::milliseconds(500));
      }
    }  // while (active_planning_ && ros::ok())

    if (!active_planning_)
    {
      ROS_INFO_STREAM("\"GetPath\" action ended properly.");
    }
    else
    {
      ROS_ERROR_STREAM("\"GetPath\" action has been stopped!");
    }
  }

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  void AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::callActionExePath(
      const move_base_flex_msgs::ExePathGoalConstPtr &goal)
  {

    move_base_flex_msgs::ExePathResult result;
    move_base_flex_msgs::ExePathFeedback feedback;

    typename AbstractControllerExecution<LOCAL_PLANNER_BASE>::ControllerState state_moving_input;

    ros::Time last_oscillation_reset = ros::Time::now();

    std::vector<geometry_msgs::PoseStamped> plan = goal->path.poses;
    geometry_msgs::PoseStamped goal_pose = plan.back();
    ROS_INFO_STREAM("Called action \"ExePath\" with plan:" << std::endl
                    << "frame: \"" << goal->path.header.frame_id << "\" " << std::endl
                    << "stamp: " << goal->path.header.stamp << std::endl
                    << "num poses: " << goal->path.poses.size()
                    << "goal: (" << goal_pose.pose.position.x << ", "
                                 << goal_pose.pose.position.y << ", "
                                 << goal_pose.pose.position.z << ")");

    moving_ptr_->setNewPlan(plan);
    if (!moving_ptr_->startMoving())
    {
      ROS_ERROR_STREAM("Could not start moving, because another moving thread is already / still running!");
      result.server_code = move_base_flex_msgs::ExePathResult::INTERNAL_ERROR;
      result.server_msg = "Could not start moving, because another moving thread is already / still running!";
      action_server_exe_path_ptr_->setAborted(result, result.server_msg);
      return;
    }

    active_moving_ = true;

    geometry_msgs::PoseStamped robot_pose;
    geometry_msgs::PoseStamped oscillation_pose;
    bool first_cycle = true;

    while (active_moving_ && ros::ok())
    {
      if (!getRobotPose(robot_pose))
      {
        ROS_ERROR_STREAM("Could not get the robot pose!");
        result.pose = robot_pose;
        result.server_code = move_base_flex_msgs::ExePathResult::TF_ERROR;
        result.server_msg = "Could not get the robot pose";
        action_server_exe_path_ptr_->setAborted(result, result.server_msg);
        break;
      }
      else
      {
        result.pose = robot_pose;
        feedback.pose = robot_pose;
      }


      if (first_cycle)
      {
        // init oscillation pose
        oscillation_pose = robot_pose;
      }

      // check preempt requested
      if (action_server_exe_path_ptr_->isPreemptRequested())
      {
        moving_ptr_->stopMoving();
      }

      state_moving_input = moving_ptr_->getState();

      switch (state_moving_input)
      {
        case AbstractControllerExecution<LOCAL_PLANNER_BASE>::STOPPED:
          ROS_WARN_STREAM("The moving has been stopped!");
          result.server_code = move_base_flex_msgs::ExePathResult::STOPPED;
          result.server_msg = "Local planner preempted";
          moving_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
          action_server_exe_path_ptr_->setPreempted(result, result.server_msg);
          ROS_INFO_STREAM("Action \"ExePath\" preempted");
          break;

        case AbstractControllerExecution<LOCAL_PLANNER_BASE>::STARTED:
          ROS_INFO_STREAM("The moving has been started!");
          break;

          // in progress
        case AbstractControllerExecution<LOCAL_PLANNER_BASE>::PLANNING:
          if (moving_ptr_->isPatienceExceeded())
          {
            ROS_INFO_STREAM("Local planner patience has been exceeded!");
            // TODO planner is stuck, but we don't have currently any way to cancel it!
          }
          break;

        case AbstractControllerExecution<LOCAL_PLANNER_BASE>::MAX_RETRIES:
          ROS_WARN_STREAM("The local planner has been aborted after it exceeded the maximum number of retries!");
          active_moving_ = false;
          result.server_msg = "The local planner has been aborted after it exceeded the maxinum number if retries!";
          result.server_code = move_base_flex_msgs::ExePathResult::MAX_RETRIES;
          moving_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
          action_server_exe_path_ptr_->setAborted(result, result.server_msg);
          break;

        case AbstractControllerExecution<LOCAL_PLANNER_BASE>::PAT_EXCEEDED:
          ROS_WARN_STREAM("The local planner has been aborted after it exceeded the patience time ");
          active_moving_ = false;
          result.server_code = move_base_flex_msgs::ExePathResult::PAT_EXCEEDED;
          result.server_msg = "Local planner exceeded allocated time";
          moving_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
          action_server_exe_path_ptr_->setAborted(result, result.server_msg);
          break;

        case AbstractControllerExecution<LOCAL_PLANNER_BASE>::NO_PLAN:
          ROS_WARN_STREAM("The local planner has been started without any plan!");
          active_moving_ = false;
          result.server_code = move_base_flex_msgs::ExePathResult::INTERNAL_ERROR;
          result.server_msg = "No plan provided to local planner";
          moving_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
          action_server_exe_path_ptr_->setAborted(result, result.server_msg);
          break;

        case AbstractControllerExecution<LOCAL_PLANNER_BASE>::EMPTY_PLAN:
          ROS_WARN_STREAM("The local planner has received an empty plan");
          active_moving_ = false;
          result.server_code = move_base_flex_msgs::ExePathResult::INVALID_PATH;
          result.server_msg = "Empty plan provided to local planner";
          moving_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
          action_server_exe_path_ptr_->setAborted(result, result.server_msg);
          break;

        case AbstractControllerExecution<LOCAL_PLANNER_BASE>::NO_LOCAL_CMD:
          ROS_WARN_STREAM_THROTTLE(3, "Have not received a velocity command from the local planner!");
          moving_ptr_->getVelocityCmd(feedback.twist);
          feedback.dist_to_goal = (float)move_base_flex::distance(robot_pose, goal_pose);
          feedback.angle_to_goal = (float)move_base_flex::angle(robot_pose, goal_pose);
          action_server_exe_path_ptr_->publishFeedback(feedback);
          break;

        case AbstractControllerExecution<LOCAL_PLANNER_BASE>::GOT_LOCAL_CMD:
          if (move_base_flex::distance(robot_pose, oscillation_pose) >= oscillation_distance_)
          {
            last_oscillation_reset = ros::Time::now();
            oscillation_pose = robot_pose;
          }

          moving_ptr_->getVelocityCmd(feedback.twist);
          feedback.dist_to_goal = (float)move_base_flex::distance(robot_pose, goal_pose);
          feedback.angle_to_goal = (float)move_base_flex::angle(robot_pose, goal_pose);
          action_server_exe_path_ptr_->publishFeedback(feedback);

          // check if oscillating
          if (oscillation_timeout_ > ros::Duration(0.0)
              && last_oscillation_reset + oscillation_timeout_ < ros::Time::now())
          {
            ROS_WARN_STREAM("The local planner is oscillating for "
                            << (ros::Time::now() - last_oscillation_reset).toSec() << "s");
            moving_ptr_->stopMoving();
            active_moving_ = false;
            result.server_code = move_base_flex_msgs::ExePathResult::OSCILLATION;
            result.server_msg = "Oscillation detected!";
            moving_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
            action_server_exe_path_ptr_->setAborted(result, result.server_msg);
          }
          break;

        case AbstractControllerExecution<LOCAL_PLANNER_BASE>::ARRIVED_GOAL:
          ROS_INFO_STREAM("Local planner succeeded; arrived to goal");
          active_moving_ = false;
          result.server_code = move_base_flex_msgs::ExePathResult::ARRIVED;
          result.server_msg = "Local planner succeeded; arrived to goal!";
          moving_ptr_->getPluginInfo(result.plugin_code, result.plugin_msg);
          action_server_exe_path_ptr_->setSucceeded(result, result.server_msg);
          break;
      }

      if (active_moving_)
      {
        // try to sleep a bit
        // normally the thread should be woken up from the moving unit
        // in order to transfer the results to the controller
        boost::mutex mutex;
        boost::unique_lock<boost::mutex> lock(mutex);
        condition_.wait_for(lock, boost::chrono::milliseconds(500));
      }

      first_cycle = false;
    }  // while (active_moving_ && ros::ok())

    if (!active_moving_)
    {
      ROS_INFO_STREAM("\"ExePath\" action ended properly.");
    }
    else
    {
      ROS_ERROR_STREAM("\"ExePath\" action has been stopped!");
    }
  }

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  void AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::callActionRecovery(
      const move_base_flex_msgs::RecoveryGoalConstPtr &goal)
  {
    move_base_flex_msgs::RecoveryResult result;
    std::string behavior = goal->behavior;
    recovery_ptr_->startRecovery(behavior);
    active_recovery_ = true;

    typename AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::RecoveryState state_recovery_input;

    while (active_recovery_ && ros::ok())
    {
      state_recovery_input = recovery_ptr_->getState();
      switch (state_recovery_input)
      {
        case AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::STOPPED:
          ROS_WARN_STREAM("Recovering \"" << behavior << "\" has been stopped!");
          active_recovery_ = false; // stopping the action
          break;

        case AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::STARTED:
          ROS_INFO_STREAM("Recovering \"" << behavior << "\" was started");
          break;

        case AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::RECOVERING:
          // check preempt requested; we let for the next iteration to set the action as preempted to
          // give time to the executing thread to finish, set the CANCELED state and notify condition
          if (action_server_recovery_ptr_->isPreemptRequested() && recovery_ptr_->cancel())
          {
            ROS_INFO_STREAM("Recovering \"" << behavior << "\" canceled");
          }
          else
          {
            // we keep silent if cancel failed because most recovery behaviors don't support canceling
            ROS_INFO_STREAM_THROTTLE(3, "Recovering with: " << behavior);
          }
          break;

        case AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::WRONG_NAME:
          active_recovery_ = false; // stopping the action
          ROS_ERROR_STREAM("Recovery has no plugin found with the name \"" << behavior << "\"!");
          result.server_code = move_base_flex_msgs::RecoveryResult::INVALID_NAME;
          result.server_msg = "No recovery plugin loaded with the given name\"" + behavior + "\"!";
          action_server_recovery_ptr_->setAborted(result, result.server_msg);
          break;

        case AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::CANCELED:
          // Recovery behavior supports cancel and it worked
          active_recovery_ = false; // stopping the action
          result.server_code = move_base_flex_msgs::RecoveryResult::CANCELED;
          result.server_msg = "Recovering \"" + behavior + "\" preempted!";
          action_server_recovery_ptr_->setPreempted(result, result.server_msg);
          ROS_INFO_STREAM(result.server_msg);
          break;

        case AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::RECOVERY_DONE:
          active_recovery_ = false; // stopping the action
          result.server_code = move_base_flex_msgs::RecoveryResult::RECOVERY_DONE;
          result.server_msg = "Recovery \"" + behavior + "\" done!";
          action_server_recovery_ptr_->setSucceeded(result, result.server_msg);
          break;
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
      ROS_INFO_STREAM("\"Recovery\" action ended properly.");
    }
    else
    {
      ROS_ERROR_STREAM("\"Recovery\" action has been stopped!");
    }
  }

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  void AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::callActionMoveBase(
      const move_base_flex_msgs::MoveBaseGoalConstPtr &goal)
  {
    const geometry_msgs::PoseStamped target_pose = goal->target_pose;
    geometry_msgs::PoseStamped robot_pose;

    ros::NodeHandle private_nh("~");

    ActionClientExePath action_client_exe_path(private_nh, name_action_exe_path);
    ActionClientGetPath action_client_get_path(private_nh, name_action_get_path);
    ActionClientRecovery action_client_recovery(private_nh, name_action_recovery);

    move_base_flex_msgs::GetPathGoal get_path_goal;
    move_base_flex_msgs::ExePathGoal exe_path_goal;
    move_base_flex_msgs::RecoveryGoal recovery_goal;

    get_path_goal.target_pose = target_pose;
    get_path_goal.use_start_pose = false; // use the robot pose

    ros::Duration connection_timeout(1.0);

    move_base_flex_msgs::MoveBaseResult move_base_result;
    move_base_flex_msgs::GetPathResult get_path_result;
    move_base_flex_msgs::ExePathResult exe_path_result;
    move_base_flex_msgs::RecoveryResult recovery_result;

    std::vector<std::string> mbf_behaviors;
    if (private_nh.getParam("mbf_action_recovery_behaviors", mbf_behaviors))
    {
      ROS_WARN_STREAM("No recovery behaviors given for the action \"" << name_action_move_base << "\"!");
    }

    bool all_rb_loaded = true;
    std::string names = "";

    std::vector<std::string>::iterator iter;

    for (iter = mbf_behaviors.begin(); iter != mbf_behaviors.end(); ++iter)
    {
      if (!recovery_ptr_->hasRecoveryBehavior(*iter))
      {
        ROS_ERROR_STREAM("The recovery behavior for the given name \"" << *iter << "\" has not been loaded!");
        all_rb_loaded = false;
        names += names == "" ? *iter : ", " + *iter;
      }
    }

    if (!all_rb_loaded)
    {
      std::stringstream ss;
      ss << "The recovery behaviors for the names " << names << " has not been loaded!";
      ROS_ERROR_STREAM(ss.str());
      move_base_result.server_code = move_base_flex_msgs::MoveBaseResult::WRONG_CONFIG;
      move_base_result.server_msg = ss.str();
      move_base_result.angle_to_goal = 0;
      move_base_result.dist_to_goal = 0;
      move_base_result.final_pose = geometry_msgs::PoseStamped();
      action_server_move_base_ptr_->setAborted(move_base_result, ss.str());
      return;
    }

    // start recovering with the first behaviour
    std::vector<std::string>::iterator current_recovery_behavior = mbf_behaviors.begin();

    // get the current robot pose
    if (!getRobotPose(robot_pose))
    {
      ROS_ERROR_STREAM("Could not get the current robot pose!");
      move_base_result.server_msg = "Could not get the current robot pose!";
      move_base_result.server_code = move_base_flex_msgs::MoveBaseResult::INTERNAL_ERROR;
      move_base_result.angle_to_goal = 0;
      move_base_result.dist_to_goal = 0;
      move_base_result.final_pose = geometry_msgs::PoseStamped();
      action_server_move_base_ptr_->setAborted(move_base_result, move_base_result.server_msg);
      return;
    }

    enum MoveBaseActionState
    {
      GET_PATH,
      EXE_PATH,
      RECOVERY
    };

    // wait for server connections
    if (!action_client_get_path.waitForServer(connection_timeout) ||
        !action_client_exe_path.waitForServer(connection_timeout) ||
        !action_client_recovery.waitForServer(connection_timeout))
    {
      ROS_ERROR_STREAM("Could not connect to one or more of move_base_flex actions:"
                       << "\"" << name_action_get_path
                       << "\", " << "\"" << name_action_exe_path
                       << "\", " << "\"" << name_action_recovery << "\"!");
      move_base_result.dist_to_goal = NAN;
      move_base_result.angle_to_goal = NAN;
      move_base_result.server_code = move_base_flex_msgs::MoveBaseResult::INTERNAL_ERROR;
      move_base_result.server_msg = "Could not connect to the move_base_flex actions!";
      action_server_move_base_ptr_->setAborted(move_base_result, move_base_result.server_msg);
      return;
    }

    // call get_path action server
    action_client_get_path.sendGoal(get_path_goal);

    // init goal states with dummy values;
    actionlib::SimpleClientGoalState get_path_state(actionlib::SimpleClientGoalState::PENDING);
    actionlib::SimpleClientGoalState exe_path_state(actionlib::SimpleClientGoalState::PENDING);
    actionlib::SimpleClientGoalState recovery_state(actionlib::SimpleClientGoalState::PENDING);

    MoveBaseActionState state = GET_PATH;

    bool run = true;
    bool preempted = false;
    ros::Duration wait(0.5);

    std::string type; // recovery behavior type
    bool try_recovery = false; // init with false

    while (ros::ok() && run)
    {
      switch (state)
      {
        case GET_PATH:
          if (!action_client_get_path.waitForResult(wait))
          { // no result -> action server is still running
            if (action_server_move_base_ptr_->isPreemptRequested() && !preempted)
            {
              action_client_get_path.cancelGoal();
              preempted = true;
            }
          }
          else
          {
            get_path_state = action_client_get_path.getState();
            switch (get_path_state.state_)
            {
              case actionlib::SimpleClientGoalState::PENDING:
                // TODO -> not implemented // should not be reached!
                break;
              case actionlib::SimpleClientGoalState::SUCCEEDED:
                get_path_result = *action_client_get_path.getResult();
                ROS_INFO_STREAM("Action \"" << name_action_move_base << "\" received a path from \""
                                << name_action_get_path << "\": " << get_path_state.getText());
                exe_path_goal.path = get_path_result.path;
                ROS_INFO_STREAM("Action \"" << name_action_move_base << "\" sends the path to \""
                                << name_action_exe_path << "\".");
                action_client_exe_path.sendGoal(
                    exe_path_goal,
                    ActionClientExePath::SimpleDoneCallback(),
                    ActionClientExePath::SimpleActiveCallback(),
                    boost::bind(&move_base_flex::AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE,
                                RECOVERY_BEHAVIOR_BASE>::actionMoveBaseExePathFeedback,this, _1));
                state = EXE_PATH;
                break;
              case actionlib::SimpleClientGoalState::ABORTED:
                get_path_result = *action_client_get_path.getResult();

                // copy result from get_path action
                move_base_result.dist_to_goal = (float)move_base_flex::distance(robot_pose, target_pose);
                move_base_result.angle_to_goal = (float)move_base_flex::angle(robot_pose, target_pose);
                move_base_result.server_code = get_path_result.server_code;
                move_base_result.server_msg = get_path_result.server_msg;
                move_base_result.plugin_code = get_path_result.plugin_code;
                move_base_result.plugin_msg = get_path_result.plugin_msg;
                move_base_result.final_pose = robot_pose;
                run = false;
                action_server_move_base_ptr_->setAborted(move_base_result, get_path_state.getText());
                break;
              case actionlib::SimpleClientGoalState::PREEMPTED:
                // the get_path action has been preempted.
                run = false;
                action_server_move_base_ptr_->setPreempted();
                break;
              case actionlib::SimpleClientGoalState::RECALLED:
              case actionlib::SimpleClientGoalState::REJECTED:
                ROS_FATAL_STREAM("The states RECALLED and REJECTED are not implemented in the SimpleActionServer!");
                run = false;
                action_server_move_base_ptr_->setAborted();
                break;
              case actionlib::SimpleClientGoalState::LOST:
                // TODO
                break;

              default:
                ROS_FATAL_STREAM("Reached unreachable case! Unknown SimpleActionServer state!");
                run = false;
                action_server_move_base_ptr_->setAborted();
                break;
            }
          }

          break;

        case EXE_PATH:
          if (!action_client_exe_path.waitForResult(wait))
          {  // no result -> action server is still running
            if (action_server_move_base_ptr_->isPreemptRequested() && !preempted)
            {
              action_client_exe_path.cancelGoal();
              preempted = true;
            }
          }
          else
          {
            exe_path_state = action_client_exe_path.getState();
            switch (exe_path_state.state_)
            {
              case actionlib::SimpleClientGoalState::PENDING:
                //TODO
                break;
              case actionlib::SimpleClientGoalState::SUCCEEDED:
                exe_path_result = *action_client_exe_path.getResult();
                ROS_INFO_STREAM("Action \"" << name_action_move_base << "\" received a result from \""
                                << name_action_exe_path << "\": " << exe_path_state.getText());
                exe_path_goal.path = get_path_result.path;
                ROS_INFO_STREAM("Action \"" << name_action_move_base << "\" succeeded.");
                move_base_result.angle_to_goal = exe_path_result.angle_to_goal;
                move_base_result.dist_to_goal = exe_path_result.dist_to_goal;
                move_base_result.final_pose = exe_path_result.pose;
                move_base_result.server_msg = "MoveBase action succeeded!";
                move_base_result.plugin_code = move_base_flex_msgs::MoveBaseResult::DO_NOT_APPLY;
                move_base_result.server_code = move_base_flex_msgs::MoveBaseResult::SUCCESS;
                action_server_move_base_ptr_->setSucceeded(move_base_result, move_base_result.server_msg);
                run = false;
                break;
              case actionlib::SimpleClientGoalState::ABORTED:
                // copy result from get_path action
                move_base_result.dist_to_goal = exe_path_result.dist_to_goal;
                move_base_result.angle_to_goal = exe_path_result.angle_to_goal;
                move_base_result.server_code = exe_path_result.server_code;
                move_base_result.server_msg = exe_path_result.server_msg;
                move_base_result.plugin_code = exe_path_result.plugin_code;
                move_base_result.plugin_msg = exe_path_result.plugin_msg;
                move_base_result.final_pose = exe_path_result.pose;
                exe_path_result = *action_client_exe_path.getResult();
                switch (exe_path_result.server_code)
                {
                  case move_base_flex_msgs::ExePathResult::OSCILLATION:
                    current_recovery_behavior = mbf_behaviors.begin();

                  case move_base_flex_msgs::ExePathResult::PAT_EXCEEDED:
                  case move_base_flex_msgs::ExePathResult::MAX_RETRIES:
                  case move_base_flex_msgs::ExePathResult::INVALID_PATH:
                    if (!clearing_rotation_allowed_)
                    {
                      recovery_ptr_->getTypeOfBehavior(*current_recovery_behavior, type);
                      if (type.find("RotateRecovery") != std::string::npos)
                      {
                        ROS_INFO_STREAM("clearing_rotation_allowed is disabled: The current recovery behavior "
                                        << *current_recovery_behavior << "will be skipped!");
                        current_recovery_behavior++;
                      }
                    }
                    try_recovery = true;
                    break;
                  default:
                    try_recovery = false;
                    ROS_INFO_STREAM("Abort the execution of the exe_path!" << exe_path_result.server_msg);
                    run = false;
                    action_server_move_base_ptr_->setAborted(move_base_result, exe_path_state.getText());
                    break;
                }
                if (try_recovery)
                {
                  if (!recovery_behavior_enabled_)
                  {
                    ROS_WARN_STREAM("Recovery behaviors are disabled!");
                    ROS_INFO_STREAM("Abort the execution of the controller: " << exe_path_result.server_msg);
                    run = false;
                    action_server_move_base_ptr_->setAborted(move_base_result, exe_path_state.getText());
                    break;
                  }
                  else if (current_recovery_behavior == mbf_behaviors.end())
                  {
                    if (mbf_behaviors.empty())
                    {
                      ROS_WARN_STREAM("No Recovery Behaviors loaded! Abort controlling: " << exe_path_result.server_msg);
                    }
                    else
                    {
                      ROS_WARN_STREAM("Executed all available recovery behaviors! Abort controlling: "
                                      << exe_path_result.server_msg);
                    }
                    run = false;
                    action_server_move_base_ptr_->setAborted(move_base_result, exe_path_state.getText());
                    break;
                  }
                  else
                  {
                    recovery_goal.behavior = *current_recovery_behavior;
                    recovery_ptr_->getTypeOfBehavior(*current_recovery_behavior, type);
                    ROS_INFO_STREAM("Start recovery behavior\"" << *current_recovery_behavior
                                    << "\" of the type \"" << type << "\".");
                    action_client_recovery.sendGoal(recovery_goal);
                    state = RECOVERY;
                  }
                }
                break;
              case actionlib::SimpleClientGoalState::PREEMPTED:
                // action was preempted successfully!
                ROS_INFO_STREAM("The action \"" << name_action_move_base << "\" was preempted successfully!");
                action_server_move_base_ptr_->setPreempted();
                run = false;
                break;
              case actionlib::SimpleClientGoalState::RECALLED:
              case actionlib::SimpleClientGoalState::REJECTED:
                ROS_FATAL_STREAM("The states RECALLED and REJECTED are not implemented in the SimpleActionServer!");
                run = false;
                action_server_move_base_ptr_->setAborted();
                break;
              case actionlib::SimpleClientGoalState::LOST:
                // TODO
                break;
              default:
                ROS_FATAL_STREAM("Reached unreachable case! Unknown SimpleActionServer state!");
                run = false;
                action_server_move_base_ptr_->setAborted();
                break;
            }
          }
          break;

        case RECOVERY:
          if (!action_client_recovery.waitForResult(wait))
          {
            if (action_server_move_base_ptr_->isPreemptRequested() && !preempted)
            {
              preempted = true;
              action_client_recovery.cancelGoal();
            }
          }
          else
          {
            recovery_state = action_client_recovery.getState();
            switch (recovery_state.state_)
            {
              case actionlib::SimpleClientGoalState::PENDING:
                //TODO
                break;
              case actionlib::SimpleClientGoalState::ABORTED:
                ROS_WARN_STREAM("Execution of the recovery behavior \"" << *current_recovery_behavior << " failed!");
                ROS_INFO_STREAM("Try planning again and increment the current recovery behavior in the list.");
                state = GET_PATH;
                action_client_get_path.sendGoal(get_path_goal);
                current_recovery_behavior++; // use next behavior, the next time;
                break;
              case actionlib::SimpleClientGoalState::SUCCEEDED:
                //go to planning state
                ROS_INFO_STREAM("Execution of the recovery behavior \"" << *current_recovery_behavior << "\" succeeded!");
                ROS_INFO_STREAM("Try planning again and increment the current recovery behavior in the list.");
                state = GET_PATH;
                action_client_get_path.sendGoal(get_path_goal);
                current_recovery_behavior++; // use next behavior, the next time;
                break;
              case actionlib::SimpleClientGoalState::PREEMPTED:
                run = false;
                action_server_move_base_ptr_->setPreempted();
                break;
              case actionlib::SimpleClientGoalState::RECALLED:
              case actionlib::SimpleClientGoalState::REJECTED:
                ROS_FATAL_STREAM("The states RECALLED and REJECTED are not implemented in the SimpleActionServer!");
                run = false;
                action_server_move_base_ptr_->setAborted();
                break;
              case actionlib::SimpleClientGoalState::LOST:
              default:
                ROS_FATAL_STREAM("Reached unreachable case! Unknown SimpleActionServer state!");
                run = false;
                action_server_move_base_ptr_->setAborted();
                break;
            }
          }

          break;

        default:
          // TODO
          break;
      }
    }
  }

template<class LOCAL_PLANNER_BASE, class GLOBAL_PLANNER_BASE, class RECOVERY_BEHAVIOR_BASE>
  void AbstractNavigationServer<LOCAL_PLANNER_BASE, GLOBAL_PLANNER_BASE, RECOVERY_BEHAVIOR_BASE>::actionMoveBaseExePathFeedback(
      const move_base_flex_msgs::ExePathFeedbackConstPtr &feedback)
  {
    move_base_flex_msgs::MoveBaseFeedback feedback_out;
    feedback_out.angle_to_goal = feedback->angle_to_goal;
    feedback_out.dist_to_goal = feedback->dist_to_goal;
    feedback_out.current_pose = feedback->pose;
    feedback_out.current_twist = feedback->twist;
    action_server_move_base_ptr_->publishFeedback(feedback_out);
  }

} /* namespace move_base_flex */

#endif /* MOVE_BASE_FLEX__IMPL__ABSTRACT_NAVIGATION_SERVER_TCC_ */
