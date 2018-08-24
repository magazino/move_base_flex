#include "mbf_abstract_nav/recovery_action.h"

namespace mbf_abstract_nav{

RecoveryAction::RecoveryAction(const std::string &name, const RobotInformation &robot_info)
  : AbstractAction(name, robot_info, boost::bind(&mbf_abstract_nav::RecoveryAction::run, this, _1, _2)){}

void RecoveryAction::run(GoalHandle &goal_handle, AbstractRecoveryExecution &execution)
{
  ROS_DEBUG_STREAM_NAMED(name_, "Start action "  << name_);

  const mbf_msgs::RecoveryGoal &goal = *(goal_handle.getGoal().get());
  mbf_msgs::RecoveryResult result;
  bool recovery_active = true;

  typename AbstractRecoveryExecution::RecoveryState state_recovery_input;

  while (recovery_active && ros::ok())
  {
    state_recovery_input = execution.getState();
    switch (state_recovery_input)
    {
      case AbstractRecoveryExecution::INITIALIZED:execution.start();
        break;
      case AbstractRecoveryExecution::STOPPED:
        // Recovery behavior doesn't support or didn't answered to cancel and has been ruthlessly stopped
        ROS_WARN_STREAM("Recovering \"" << goal.behavior << "\" exceeded the patience time and has been stopped!");
        recovery_active = false; // stopping the action
        result.outcome = mbf_msgs::RecoveryResult::CANCELED;
        result.message = "Recovery \"" + goal.behavior + "\" exceeded the patience time";
        goal_handle.setSucceeded(result, result.message);
        break;

      case AbstractRecoveryExecution::STARTED:
        ROS_DEBUG_STREAM_NAMED(name_, "Recovering \"" << goal.behavior << "\" was started");
        break;

      case AbstractRecoveryExecution::RECOVERING:

        if (execution.isPatienceExceeded())
        {
          ROS_INFO_STREAM("Recovery behavior \"" << goal.behavior << "\" patience exceeded! Cancel recovering...");
          if (!execution.cancel())
          {
            ROS_WARN_STREAM("Cancel recovering \"" << goal.behavior << "\" failed or not supported; interrupt it!");
            execution.stop();
            //TODO goal_handle.setAborted
          }
        }

        ROS_DEBUG_STREAM_THROTTLE_NAMED(3, name_, "Recovering with: " << goal.behavior);
        break;

      case AbstractRecoveryExecution::CANCELED:
        // Recovery behavior supports cancel and it worked
        recovery_active = false; // stopping the action
        result.outcome = mbf_msgs::RecoveryResult::CANCELED;
        result.message = "Recovering \"" + goal.behavior + "\" preempted!";
        goal_handle.setCanceled(result, result.message);
        ROS_DEBUG_STREAM_NAMED(name_, result.message);
        break;

      case AbstractRecoveryExecution::RECOVERY_DONE:
        recovery_active = false; // stopping the action
        result.outcome = mbf_msgs::RecoveryResult::SUCCESS;
        result.message = "Recovery \"" + goal.behavior + "\" done!";
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

