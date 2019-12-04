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
 *  controller_action.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "mbf_abstract_nav/controller_action.h"

namespace mbf_abstract_nav{


ControllerAction::ControllerAction(
    const std::string &action_name,
    const RobotInformation &robot_info)
    : AbstractAction(action_name, robot_info, boost::bind(&mbf_abstract_nav::ControllerAction::run, this, _1, _2))
{
}

void ControllerAction::start(
    GoalHandle &goal_handle,
    typename AbstractControllerExecution::Ptr execution_ptr
)
{
  if(goal_handle.getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLING)
  {
    goal_handle.setCanceled();
    return;
  }
  uint8_t slot = goal_handle.getGoal()->concurrency_slot;

  bool update_plan = false;
  slot_map_mtx_.lock();
  std::map<uint8_t, ConcurrencySlot>::iterator slot_it = concurrency_slots_.find(slot);
  if(slot_it != concurrency_slots_.end())
  {
    boost::lock_guard<boost::mutex> goal_guard(goal_mtx_);
    if(slot_it->second.execution->getName() == goal_handle.getGoal()->controller ||
       goal_handle.getGoal()->controller.empty())
    {
      update_plan = true;
      // Goal requests to run the same controller on the same concurrency slot:
      // we update the goal handle and pass the new plan to the execution without stopping it
      execution_ptr = slot_it->second.execution;
      const forklift_interfaces::NavigateGoal &goal = *(goal_handle.getGoal().get());
      std::vector<geometry_msgs::PoseStamped> goal_path;
      for(std::size_t it = 0; it<goal.path.checkpoints.size(); it++)
      {
        goal_path.push_back(goal.path.checkpoints[it].pose);
      }
      execution_ptr->setNewPlan(goal_path);
      // Update also goal pose, so the feedback remains consistent
      goal_pose_ = goal_path.back();
      forklift_interfaces::NavigateResult result;
      fillNavigateResult(forklift_interfaces::NavigateResult::CANCELED, "Goal preempted by a new plan", result);
      concurrency_slots_[slot].goal_handle.setCanceled(result, result.remarks);
      concurrency_slots_[slot].goal_handle = goal_handle;
      concurrency_slots_[slot].goal_handle.setAccepted();
    }
  }
  slot_map_mtx_.unlock();
  if(!update_plan)
  {
      // Otherwise run parent version of this method
      AbstractAction::start(goal_handle, execution_ptr);
  }
}

void ControllerAction::run(GoalHandle &goal_handle, AbstractControllerExecution &execution)
{
  goal_mtx_.lock();
  // Note that we always use the goal handle stored on the concurrency slots map, as it can change when replanning
  uint8_t slot = goal_handle.getGoal()->concurrency_slot;
  goal_mtx_.unlock();
  ROS_DEBUG_STREAM_NAMED(name_, "Start action "  << name_);

  // ensure we don't provide values from previous execution on case of error before filling both poses
  goal_pose_ = geometry_msgs::PoseStamped();
  robot_pose_ = geometry_msgs::PoseStamped();

  ros::NodeHandle private_nh("~");

  double oscillation_timeout_tmp;
  private_nh.param("oscillation_timeout", oscillation_timeout_tmp, 0.0);
  ros::Duration oscillation_timeout(oscillation_timeout_tmp);

  double oscillation_distance;
  private_nh.param("oscillation_distance", oscillation_distance, 0.03);

  forklift_interfaces::NavigateResult result;
  forklift_interfaces::NavigateFeedback feedback;

  typename AbstractControllerExecution::ControllerState state_moving_input;
  bool controller_active = true;
  goal_mtx_.lock();
  const forklift_interfaces::NavigateGoal &goal = *(goal_handle.getGoal().get());
  std::vector<geometry_msgs::PoseStamped> goal_path;
  for (int it = 0; it < goal.path.checkpoints.size(); it++)
  {
    goal_path.push_back(goal.path.checkpoints[it].pose);
  }
  
  const std::vector<geometry_msgs::PoseStamped> &plan = goal_path;
  if (plan.empty())
  {
    fillNavigateResult(forklift_interfaces::NavigateResult::INVALID_PATH, "Controller started with an empty plan!", result);
    goal_handle.setAborted(result, result.remarks);
    ROS_ERROR_STREAM_NAMED(name_, result.remarks << " Canceling the action call.");
    controller_active = false;
  }
  goal_pose_ = plan.back();
  ROS_DEBUG_STREAM_NAMED(name_, "Called action \""
      << name_ << "\" with plan:" << std::endl
      << "frame: \"" << goal.path.header.frame_id << "\" " << std::endl
      << "stamp: " << goal.path.header.stamp << std::endl
      << "poses: " << goal.path.checkpoints.size() << std::endl
      << "goal: (" << goal_pose_.pose.position.x << ", "
      << goal_pose_.pose.position.y << ", "
      << goal_pose_.pose.position.z << ")");

  goal_mtx_.unlock();


  geometry_msgs::PoseStamped oscillation_pose;
  ros::Time last_oscillation_reset = ros::Time::now();

  bool first_cycle = true;

  while (controller_active && ros::ok())
  {
    // goal_handle could change between the loop cycles due to adapting the plan
    // with a new goal received for the same concurrency slot
    if (!robot_info_.getRobotPose(robot_pose_))
    {
      controller_active = false;
      fillNavigateResult(forklift_interfaces::NavigateResult::TF_ERROR, "Could not get the robot pose!", result);
      goal_mtx_.lock();
      goal_handle.setAborted(result, result.remarks);
      goal_mtx_.unlock();
      ROS_ERROR_STREAM_NAMED(name_, result.remarks << " Canceling the action call.");
      break;
    }

    if (first_cycle)
    {
      // init oscillation pose
      oscillation_pose = robot_pose_;
    }

    goal_mtx_.lock();
    state_moving_input = execution.getState();

    switch (state_moving_input)
    {
      case AbstractControllerExecution::INITIALIZED:
        execution.setNewPlan(plan);
        execution.start();
        break;

      case AbstractControllerExecution::STOPPED:
        ROS_WARN_STREAM_NAMED(name_, "The controller has been stopped!");
        //TODO Check if paused feedback to be sent
        controller_active = false;
        break;

      case AbstractControllerExecution::CANCELED:
        ROS_INFO_STREAM("Action \"navigate_path\" canceled");
        fillNavigateResult(forklift_interfaces::NavigateResult::CANCELED, "Controller canceled", result);
        goal_handle.setCanceled(result, result.remarks);
        controller_active = false;
        break;

      case AbstractControllerExecution::STARTED:
        ROS_DEBUG_STREAM_NAMED(name_, "The moving has been started!");
        break;

        // in progress
      case AbstractControllerExecution::PLANNING:
        if (execution.isPatienceExceeded())
        {
          ROS_INFO_STREAM_NAMED(name_, "The controller patience has been exceeded! Stopping controller...");
          // TODO planner is stuck, but we don't have currently any way to cancel it!
          // We will try to stop the thread, but does nothing with DWA, TR or TEB controllers
          // Note that this is not the same situation as in case AbstractControllerExecution::PAT_EXCEEDED,
          // as there is the controller itself reporting that it cannot find a valid command after trying
          // for more than patience seconds. But after stopping controller execution, it should ideally
          // report PAT_EXCEEDED as his state on next iteration.
          execution.stop();
        }
        break;

      case AbstractControllerExecution::MAX_RETRIES:
        ROS_WARN_STREAM_NAMED(name_, "The controller has been aborted after it exceeded the maximum number of retries!");
        controller_active = false;
        fillNavigateResult(execution.getOutcome(), execution.getMessage(), result);
        goal_handle.setAborted(result, result.remarks);
        break;

      case AbstractControllerExecution::PAT_EXCEEDED:
        ROS_WARN_STREAM_NAMED(name_, "The controller has been aborted after it exceeded the patience time");
        controller_active = false;
        fillNavigateResult(forklift_interfaces::NavigateResult::PAT_EXCEEDED, execution.getMessage(), result);
        goal_handle.setAborted(result, result.remarks);
        break;

      case AbstractControllerExecution::NO_PLAN:
        ROS_WARN_STREAM_NAMED(name_, "The controller has been started without a plan!");
        controller_active = false;
        fillNavigateResult(forklift_interfaces::NavigateResult::INVALID_PATH, "Controller started without a path", result);
        goal_handle.setAborted(result, result.remarks);
        break;

      case AbstractControllerExecution::EMPTY_PLAN:
        ROS_WARN_STREAM_NAMED(name_, "The controller has received an empty plan");
        controller_active = false;
        fillNavigateResult(forklift_interfaces::NavigateResult::INVALID_PATH, "Controller started with an empty plan", result);
        goal_handle.setAborted(result, result.remarks);
        break;

      case AbstractControllerExecution::INVALID_PLAN:
        ROS_WARN_STREAM_NAMED(name_, "The controller has received an invalid plan");
        controller_active = false;
        fillNavigateResult(forklift_interfaces::NavigateResult::INVALID_PATH, "Controller started with an invalid plan", result);
        goal_handle.setAborted(result, result.remarks);
        break;

      case AbstractControllerExecution::NO_LOCAL_CMD:
        ROS_WARN_STREAM_THROTTLE_NAMED(3, name_, "No velocity command received from controller! "
            << execution.getMessage());
        publishNavigateFeedback(goal_handle, execution.getOutcome(), execution.getMessage(), execution.getVelocityCmd());
        break;

      case AbstractControllerExecution::GOT_LOCAL_CMD:
        if (!oscillation_timeout.isZero())
        {
          // check if oscillating
          if (mbf_utility::distance(robot_pose_, oscillation_pose) >= oscillation_distance)
          {
            last_oscillation_reset = ros::Time::now();
            oscillation_pose = robot_pose_;
          }
          else if (last_oscillation_reset + oscillation_timeout < ros::Time::now())
          {
            ROS_WARN_STREAM_NAMED(name_, "The controller is oscillating for "
                << (ros::Time::now() - last_oscillation_reset).toSec() << "s");
            execution.stop();
            controller_active = false;
            fillNavigateResult(forklift_interfaces::NavigateResult::OSCILLATION, "Oscillation detected!", result);
            goal_handle.setAborted(result, result.remarks);
            break;
          }
        }
        publishNavigateFeedback(goal_handle, execution.getOutcome(), execution.getMessage(), execution.getVelocityCmd());
        break;

      case AbstractControllerExecution::ARRIVED_GOAL:
        ROS_DEBUG_STREAM_NAMED(name_, "Controller succeeded; arrived at goal");
        controller_active = false;
        fillNavigateResult(forklift_interfaces::NavigateResult::SUCCESS, "Controller succeeded; arrived at goal!", result);
        goal_handle.setSucceeded(result, result.remarks);
        break;

      case AbstractControllerExecution::INTERNAL_ERROR:
        ROS_FATAL_STREAM_NAMED(name_, "Internal error: Unknown error thrown by the plugin: " << execution.getMessage());
        controller_active = false;
        fillNavigateResult(forklift_interfaces::NavigateResult::INTERNAL_ERROR, "Internal error: Unknown error thrown by the plugin!", result);
        goal_handle.setAborted(result, result.remarks);
        break;

      default:
        std::stringstream ss;
        ss << "Internal error: Unknown state in a move base flex controller execution with the number: "
           << static_cast<int>(state_moving_input);
        fillNavigateResult(forklift_interfaces::NavigateResult::INTERNAL_ERROR, ss.str(), result);
        ROS_FATAL_STREAM_NAMED(name_, result.remarks);
        goal_handle.setAborted(result, result.remarks);
        controller_active = false;
    }
    goal_mtx_.unlock();

    if (controller_active)
    {
      // try to sleep a bit
      // normally this thread should be woken up from the controller execution thread
      // in order to transfer the results to the controller
      execution.waitForStateUpdate(boost::chrono::milliseconds(500));
    }

    first_cycle = false;
  }  // while (controller_active && ros::ok())

  if (!controller_active)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "\"" << name_ << "\" action ended properly.");
  }
  else
  {
    // normal on continuous replanning
    ROS_DEBUG_STREAM_NAMED(name_, "\"" << name_ << "\" action has been stopped!");
  }
}

void ControllerAction::publishNavigateFeedback(
        GoalHandle& goal_handle,
        uint32_t outcome, const std::string &message,
        const geometry_msgs::TwistStamped& current_twist)
{
  forklift_interfaces::NavigateFeedback feedback;
  feedback.status = outcome;
  feedback.remarks = message;

  feedback.velocity = current_twist;
  if (feedback.velocity.header.stamp.isZero())
    feedback.velocity.header.stamp = ros::Time::now();

  feedback.current_pose = robot_pose_;
  feedback.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal_pose_));
  feedback.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal_pose_));
  goal_handle.publishFeedback(feedback);
}

void ControllerAction::fillNavigateResult(
        uint32_t outcome, const std::string &message,
        forklift_interfaces::NavigateResult &result)
{
  result.status = outcome;
  result.remarks = message;
  result.final_pose = robot_pose_;
  result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal_pose_));
  result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal_pose_));
}

}

