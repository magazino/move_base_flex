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
 *  recovery_action.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "mbf_abstract_nav/recovery_action.h"

namespace mbf_abstract_nav
{

RecoveryAction::RecoveryAction(const std::string &name, const mbf_utility::RobotInformation &robot_info)
  : AbstractActionBase(name, robot_info){}

void RecoveryAction::runImpl(GoalHandle &goal_handle, AbstractRecoveryExecution &execution)
{
  ROS_DEBUG_STREAM_NAMED(name_, "Start action "  << name_);

  const mbf_msgs::RecoveryGoal &goal = *goal_handle.getGoal();
  mbf_msgs::RecoveryResult result;
  result.used_plugin = goal.behavior;
  bool recovery_active = true;

  typename AbstractRecoveryExecution::RecoveryState state_recovery_input;

  while (recovery_active && ros::ok())
  {
    state_recovery_input = execution.getState();
    switch (state_recovery_input)
    {
      case AbstractRecoveryExecution::INITIALIZED:
        ROS_DEBUG_STREAM_NAMED(name_, "Recovery behavior \"" << goal.behavior << "\" initialized.");
        execution.start();
        break;

      case AbstractRecoveryExecution::STOPPED:
        ROS_DEBUG_STREAM_NAMED(name_, "Recovery behavior stopped rigorously");
        result.outcome = mbf_msgs::RecoveryResult::STOPPED;
        result.message = "Recovery has been stopped!";
        goal_handle.setAborted(result, result.message);
        recovery_active = false;
        break;

      case AbstractRecoveryExecution::STARTED:
        ROS_DEBUG_STREAM_NAMED(name_, "Recovery behavior \"" << goal.behavior << "\" was started");
        break;

      case AbstractRecoveryExecution::RECOVERING:

        if (execution.isPatienceExceeded())
        {
          ROS_INFO_STREAM("Recovery behavior \"" << goal.behavior << "\" patience exceeded! Cancel recovering...");
          execution.cancel();
        }

        ROS_DEBUG_STREAM_THROTTLE_NAMED(3, name_, "Recovering with: " << goal.behavior);
        break;

      case AbstractRecoveryExecution::CANCELED:
        // Recovery behavior supports cancel and it worked
        recovery_active = false; // stopping the action
        result.outcome = mbf_msgs::RecoveryResult::CANCELED;
        result.message = "Recovery behaviour \"" + goal.behavior + "\" canceled!";
        goal_handle.setCanceled(result, result.message);
        ROS_DEBUG_STREAM_NAMED(name_, result.message);
        break;

      case AbstractRecoveryExecution::RECOVERY_DONE:
        recovery_active = false; // stopping the action
        result.outcome = execution.getOutcome();
        result.message = execution.getMessage();
        if (result.message.empty())
        {
          if (result.outcome < 10)
            result.message = "Recovery \"" + goal.behavior + "\" done";
          else
            result.message = "Recovery \"" + goal.behavior + "\" FAILED";
        }

        ROS_DEBUG_STREAM_NAMED(name_, result.message);
        goal_handle.setSucceeded(result, result.message);
        break;

      case AbstractRecoveryExecution::INTERNAL_ERROR:
        ROS_FATAL_STREAM_NAMED(name_, "Internal error: Unknown error thrown by the plugin!"); // TODO getMessage from recovery
        recovery_active = false;
        result.outcome = mbf_msgs::RecoveryResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown error thrown by the plugin!";
        goal_handle.setAborted(result, result.message);
        break;

      default:
        result.outcome = mbf_msgs::RecoveryResult::INTERNAL_ERROR;
        std::stringstream ss;
        ss << "Internal error: Unknown state in a move base flex recovery execution with the number: "
           << static_cast<int>(state_recovery_input);
        result.message = ss.str();
        ROS_FATAL_STREAM_NAMED(name_, result.message);
        goal_handle.setAborted(result, result.message);
        recovery_active = false;
    }

    if (recovery_active)
    {
      // try to sleep a bit
      // normally the thread should be woken up from the recovery unit
      // in order to transfer the results to the controller
      execution.waitForStateUpdate(boost::chrono::milliseconds(500));
    }
  }  // while (recovery_active && ros::ok())

  if (!recovery_active)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "\"" << name_ << "\" action ended properly.");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_, "\"" << name_ << "\" action has been stopped!");
  }
}

} /* namespace mbf_abstract_nav */
