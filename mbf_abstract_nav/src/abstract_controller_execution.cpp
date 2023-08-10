/*
 *  Copyright 2018, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
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
 *  abstract_controller_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <mbf_msgs/ExePathResult.h>

#include "mbf_abstract_nav/abstract_controller_execution.h"

namespace mbf_abstract_nav
{

const double AbstractControllerExecution::DEFAULT_CONTROLLER_FREQUENCY = 100.0; // 100 Hz

AbstractControllerExecution::AbstractControllerExecution(
    const std::string &name,
    const mbf_abstract_core::AbstractController::Ptr &controller_ptr,
    const mbf_utility::RobotInformation &robot_info,
    const ros::Publisher &vel_pub,
    const ros::Publisher &goal_pub,
    const MoveBaseFlexConfig &config) :
  AbstractExecutionBase(name, robot_info),
    controller_(controller_ptr),
    state_(INITIALIZED), moving_(false), max_retries_(0), patience_(0), vel_pub_(vel_pub), current_goal_pub_(goal_pub),
    loop_rate_(DEFAULT_CONTROLLER_FREQUENCY)
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // non-dynamically reconfigurable parameters
  private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
  private_nh.param("map_frame", global_frame_, std::string("map"));
  private_nh.param("force_stop_at_goal", force_stop_at_goal_, false);
  private_nh.param("force_stop_on_retry", force_stop_on_retry_, true);
  private_nh.param("force_stop_on_cancel", force_stop_on_cancel_, false);
  private_nh.param("mbf_tolerance_check", mbf_tolerance_check_, false);
  private_nh.param("dist_tolerance", dist_tolerance_, 0.1);
  private_nh.param("angle_tolerance", angle_tolerance_, M_PI / 18.0);
  private_nh.param("tf_timeout", tf_timeout_, 1.0);
  private_nh.param("cmd_vel_ignored_tolerance", cmd_vel_ignored_tolerance_, 5.0);

  // dynamically reconfigurable parameters
  reconfigure(config);
}

AbstractControllerExecution::~AbstractControllerExecution()
{
}

bool AbstractControllerExecution::setControllerFrequency(double frequency)
{
  // set the calling duration by the moving frequency
  if (frequency <= 0.0)
  {
    ROS_ERROR("Controller frequency must be greater than 0.0! No change of the frequency!");
    return false;
  }
  loop_rate_ = ros::Rate(frequency);
  return true;
}

void AbstractControllerExecution::reconfigure(const MoveBaseFlexConfig &config)
{
  boost::lock_guard<boost::mutex> guard(configuration_mutex_);
  // Timeout granted to the controller. We keep calling it up to this time or up to max_retries times
  // If it doesn't return within time, the navigator will cancel it and abort the corresponding action
  patience_ = ros::Duration(config.controller_patience);

  setControllerFrequency(config.controller_frequency);

  max_retries_ = config.controller_max_retries;
}


bool AbstractControllerExecution::start()
{
  setState(STARTED);
  if (moving_)
  {
    return false; // thread is already running.
  }
  moving_ = true;
  return AbstractExecutionBase::start();
}


void AbstractControllerExecution::setState(ControllerState state)
{
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  state_ = state;
}

typename AbstractControllerExecution::ControllerState AbstractControllerExecution::getState() const
{
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  return state_;
}

void AbstractControllerExecution::setNewPlan(
  const std::vector<geometry_msgs::PoseStamped> &plan,
  bool tolerance_from_action,
  double action_dist_tolerance,
  double action_angle_tolerance)
{
  if (moving_)
  {
    // This is fine on continuous replanning
    ROS_DEBUG("Setting new plan while moving");
  }
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  new_plan_ = true;

  plan_ = plan;
  tolerance_from_action_ = tolerance_from_action;
  action_dist_tolerance_ = action_dist_tolerance;
  action_angle_tolerance_ = action_angle_tolerance;
}


bool AbstractControllerExecution::hasNewPlan()
{
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  return new_plan_;
}


std::vector<geometry_msgs::PoseStamped> AbstractControllerExecution::getNewPlan()
{
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  new_plan_ = false;
  return plan_;
}

uint32_t AbstractControllerExecution::computeVelocityCmd(const geometry_msgs::PoseStamped &robot_pose,
                                                         const geometry_msgs::TwistStamped &robot_velocity,
                                                         geometry_msgs::TwistStamped &vel_cmd,
                                                         std::string &message)
{
  return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
}


void AbstractControllerExecution::setVelocityCmd(const geometry_msgs::TwistStamped &vel_cmd)
{
  boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
  vel_cmd_stamped_ = vel_cmd;
  if (vel_cmd_stamped_.header.stamp.isZero())
    vel_cmd_stamped_.header.stamp = ros::Time::now();
  // TODO what happen with frame id?
  // TODO Add a queue here for handling the outcome, message and cmd_vel values bundled,
  // TODO so there should be no loss of information in the feedback stream
}

bool AbstractControllerExecution::checkCmdVelIgnored(const geometry_msgs::Twist& cmd_vel)
{
  // check if the velocity ignored check is enabled or not
  if (cmd_vel_ignored_tolerance_ <= 0.0)
  { 
    return false;
  }

  const bool robot_stopped = robot_info_.isRobotStopped(1e-3, 1e-3);

  // compute linear and angular velocity magnitude
  const double cmd_linear = std::hypot(cmd_vel.linear.x, cmd_vel.linear.y);
  const double cmd_angular = std::abs(cmd_vel.angular.z);

  const bool cmd_is_zero = cmd_linear < 1e-3 && cmd_angular < 1e-3;

  if (!robot_stopped || cmd_is_zero)
  {
    // velocity is not being ignored
    first_ignored_time_ = ros::Time();
    return false;
  }

  if (first_ignored_time_.is_zero())
  {
    // set first_ignored_time_ to now if it was zero
    first_ignored_time_ = ros::Time::now();
  }

  const double ignored_duration = (ros::Time::now() - first_ignored_time_).toSec();

  if (ignored_duration > cmd_vel_ignored_tolerance_)
  {
    ROS_ERROR("Robot is ignoring velocity commands for more than %.2f seconds. Tolerance exceeded!",
              cmd_vel_ignored_tolerance_);
    return true;
  }
  else if (ignored_duration > 1.0)
  {
    ROS_WARN_THROTTLE(1,
                      "Robot is ignoring velocity commands for %.2f seconds (last command: vx=%.2f, vy=%.2f, w=%.2f)",
                      ignored_duration, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  }

  return false;
}

geometry_msgs::TwistStamped AbstractControllerExecution::getVelocityCmd() const
{
  boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
  return vel_cmd_stamped_;
}

ros::Time AbstractControllerExecution::getLastPluginCallTime() const
{
  boost::lock_guard<boost::mutex> guard(lct_mtx_);
  return last_call_time_;
}

bool AbstractControllerExecution::isPatienceExceeded() const
{
  boost::lock_guard<boost::mutex> guard(lct_mtx_);
  if(!patience_.isZero() && ros::Time::now() - start_time_ > patience_) // not zero -> activated, start_time handles init case
  {
    if(ros::Time::now() - last_call_time_ > patience_)
    {
      ROS_WARN_STREAM_THROTTLE(3, "The controller plugin \"" << name_ << "\" needs more time to compute in one run than the patience time!");
      return true;
    }
    if(ros::Time::now() - last_valid_cmd_time_ > patience_)
    {
      ROS_DEBUG_STREAM("The controller plugin \"" << name_ << "\" does not return a success state (outcome < 10) for more than the patience time in multiple runs!");
      return true;
    }
  }
  return false;
}

bool AbstractControllerExecution::isMoving() const
{
  return moving_;
}

bool AbstractControllerExecution::reachedGoalCheck()
{
  //if action has a specific tolerance, check goal reached with those tolerances
  if (tolerance_from_action_)
  {
    return controller_->isGoalReached(action_dist_tolerance_, action_angle_tolerance_) ||
        (mbf_tolerance_check_ && mbf_utility::distance(robot_pose_, plan_.back()) < action_dist_tolerance_
        && mbf_utility::angle(robot_pose_, plan_.back()) < action_angle_tolerance_);
  }

  // Otherwise, check whether the controller plugin returns goal reached or if mbf should check for goal reached.
  return controller_->isGoalReached(dist_tolerance_, angle_tolerance_) || (mbf_tolerance_check_
      && mbf_utility::distance(robot_pose_, plan_.back()) < dist_tolerance_
      && mbf_utility::angle(robot_pose_, plan_.back()) < angle_tolerance_);
}

bool AbstractControllerExecution::cancel()
{
  // Request the controller to cancel; it will return true if it takes care of stopping, returning CANCELED on
  // computeVelocityCmd when done. This allows for smooth, controlled stops.
  // If false (meaning cancel is not implemented, or that the controller defers handling it) MBF will take care.
  if (controller_->cancel())
  {
    ROS_INFO("Controller will take care of stopping");
  }
  else
  {
    ROS_WARN("Controller defers handling cancel; force it and wait until the current control cycle finished");
    cancel_ = true;
    // wait for the control cycle to stop
    if (waitForStateUpdate(boost::chrono::milliseconds(500)) == boost::cv_status::timeout)
    {
      // this situation should never happen; if it does, the action server will be unready for goals immediately sent
      ROS_WARN_STREAM("Timeout while waiting for control cycle to stop; immediately sent goals can get stuck");
      return false;
    }
  }
  return true;
}


void AbstractControllerExecution::run()
{
  start_time_ = ros::Time::now();

  // init plan
  std::vector<geometry_msgs::PoseStamped> plan;
  if (!hasNewPlan())
  {
    setState(NO_PLAN);
    moving_ = false;
    ROS_ERROR("robot navigation moving has no plan!");
  }

  last_valid_cmd_time_ = ros::Time();
  int retries = 0;
  int seq = 0;
  first_ignored_time_ = ros::Time();

  try
  {
    while (moving_ && ros::ok())
    {
      if (cancel_)
      {
        if (force_stop_on_cancel_)
        {
          publishZeroVelocity(); // command the robot to stop on canceling navigation
        }
        setState(CANCELED);
        moving_ = false;
        condition_.notify_all();
        return;
      }

      if (!safetyCheck())
      {
        // the specific implementation must have detected a risk situation; at this abstract level, we
        // cannot tell what the problem is, but anyway we command the robot to stop to avoid crashes
        publishZeroVelocity();
        loop_rate_.sleep();
        continue;
      }

      // update plan dynamically
      if (hasNewPlan())
      {
        plan = getNewPlan();

        // check if plan is empty
        if (plan.empty())
        {
          setState(EMPTY_PLAN);
          moving_ = false;
          condition_.notify_all();
          return;
        }

        // check if plan could be set
        if (!controller_->setPlan(plan))
        {
          setState(INVALID_PLAN);
          moving_ = false;
          condition_.notify_all();
          return;
        }
        current_goal_pub_.publish(plan.back());
      }

      // compute robot pose and store it in robot_pose_
      if (!robot_info_.getRobotPose(robot_pose_))
      {
        message_ = "Could not get the robot pose";
        outcome_ = mbf_msgs::ExePathResult::TF_ERROR;
        publishZeroVelocity();
        setState(INTERNAL_ERROR);
        moving_ = false;
        condition_.notify_all();
        return;
      }

      // ask planner if the goal is reached
      if (reachedGoalCheck())
      {
        ROS_DEBUG_STREAM_NAMED("abstract_controller_execution", "Reached the goal!");
        if (force_stop_at_goal_)
        {
          publishZeroVelocity();
        }
        setState(ARRIVED_GOAL);
        // goal reached, tell it the controller
        moving_ = false;
        condition_.notify_all();
        // if not, keep moving
      }
      else
      {
        setState(PLANNING);

        // save time and call the plugin
        lct_mtx_.lock();
        last_call_time_ = ros::Time::now();
        lct_mtx_.unlock();

        // call plugin to compute the next velocity command
        geometry_msgs::TwistStamped cmd_vel_stamped;
        geometry_msgs::TwistStamped robot_velocity;
        robot_info_.getRobotVelocity(robot_velocity);
        outcome_ = computeVelocityCmd(robot_pose_, robot_velocity, cmd_vel_stamped, message_ = "");

        if (outcome_ < 10)
        {
          setState(GOT_LOCAL_CMD);
          vel_pub_.publish(cmd_vel_stamped.twist);
          last_valid_cmd_time_ = ros::Time::now();
          retries = 0;
          // check if robot is ignoring velocity command
          if (checkCmdVelIgnored(cmd_vel_stamped.twist))
          {
            setState(ROBOT_DISABLED);
            moving_ = false;
          }
        }
        else if (outcome_ == mbf_msgs::ExePathResult::CANCELED)
        {
          ROS_INFO_STREAM("Controller-handled cancel completed");
          cancel_ = true;
          continue;
        }
        else
        {
          boost::lock_guard<boost::mutex> guard(configuration_mutex_);
          if (max_retries_ > 0 && ++retries > max_retries_)
          {
            setState(MAX_RETRIES);
            moving_ = false;
          }
          else if (isPatienceExceeded())
          {
            // patience limit enabled and running controller for more than patience without valid commands
            setState(PAT_EXCEEDED);
            moving_ = false;
          }
          else
          {
            setState(NO_LOCAL_CMD); // useful for server feedback
            // keep trying if we have > 0 or -1 (infinite) retries
            moving_ = max_retries_;
          }

          // could not compute a valid velocity command
          if (!moving_ || force_stop_on_retry_)
          {
            publishZeroVelocity(); // command the robot to stop; we still feedback command calculated by the plugin
          }
          else
          {
            // we are retrying compute velocity commands; we keep sending the command calculated by the plugin
            // with the expectation that it's a sensible one (e.g. slow down while respecting acceleration limits)
            vel_pub_.publish(cmd_vel_stamped.twist);
          }
        }

        // set stamped values; timestamp and frame_id should be set by the plugin; otherwise setVelocityCmd will do
        cmd_vel_stamped.header.seq = seq++; // sequence number
        setVelocityCmd(cmd_vel_stamped);
        condition_.notify_all();
      }

      if (moving_)
      {
        // The nanosleep used by ROS time is not interruptable, therefore providing an interrupt point before and after
        boost::this_thread::interruption_point();
        if (!loop_rate_.sleep())
        {
          ROS_WARN_THROTTLE(1.0, "Calculation needs too much time to stay in the moving frequency! (%.4fs > %.4fs)",
                            loop_rate_.cycleTime().toSec(), loop_rate_.expectedCycleTime().toSec());
        }
        boost::this_thread::interruption_point();
      }
    }
  }
  catch (const boost::thread_interrupted &ex)
  {
    // Controller thread interrupted; in most cases we have started a new plan
    // Can also be that robot is oscillating or we have exceeded planner patience
    ROS_DEBUG_STREAM("Controller thread interrupted!");
    publishZeroVelocity();
    setState(STOPPED);
    condition_.notify_all();
    moving_ = false;
  }
  catch (...)
  {
    message_ = "Unknown error occurred: " + boost::current_exception_diagnostic_information();
    ROS_FATAL_STREAM(message_);
    setState(INTERNAL_ERROR);
    moving_ = false;
    condition_.notify_all();
  }
}


void AbstractControllerExecution::publishZeroVelocity()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = 0;
  vel_pub_.publish(cmd_vel);
}

} /* namespace mbf_abstract_nav */
