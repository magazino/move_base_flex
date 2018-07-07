#include "mbf_abstract_nav/controller_action.h"

namespace mbf_abstract_nav{


ControllerAction::ControllerAction(
    const std::string &action_name,
    const RobotInformation &robot_info)
    : AbstractAction(action_name, robot_info, boost::bind(&mbf_abstract_nav::ControllerAction::run, this, _1, _2))
{
}

void ControllerAction::run(GoalHandle &goal_handle, AbstractControllerExecution &execution)
{
  ROS_DEBUG_STREAM_NAMED(name_, "Start action "  << name_);

  ros::NodeHandle private_nh("~");

  double oscillation_timeout_tmp;
  private_nh.param("oscillation_timeout", oscillation_timeout_tmp, 0.0);
  ros::Duration oscillation_timeout(oscillation_timeout_tmp);

  double oscillation_distance;
  private_nh.param("oscillation_distance", oscillation_distance, 0.03);

  mbf_msgs::ExePathResult result;
  mbf_msgs::ExePathFeedback feedback;

  typename AbstractControllerExecution::ControllerState state_moving_input;

  const mbf_msgs::ExePathGoal &goal = *(goal_handle.getGoal().get());
  const std::vector<geometry_msgs::PoseStamped> &plan = goal.path.poses;
  if (plan.empty())
  {
    result.outcome = mbf_msgs::ExePathResult::INVALID_PATH;
    result.message = "Local planner started with an empty plan!";

    goal_handle.setAborted(result, result.message);
    ROS_ERROR_STREAM_NAMED(name_, result.message << " Canceling the action call.");
    return;
  }

  geometry_msgs::PoseStamped goal_pose = plan.back();
  ROS_DEBUG_STREAM_NAMED(name_, "Called action \""
      << name_ << "\" with plan:" << std::endl
      << "frame: \"" << goal.path.header.frame_id << "\" " << std::endl
      << "stamp: " << goal.path.header.stamp << std::endl
      << "poses: " << goal.path.poses.size() << std::endl
      << "goal: (" << goal_pose.pose.position.x << ", "
      << goal_pose.pose.position.y << ", "
      << goal_pose.pose.position.z << ")");

  execution.setNewPlan(plan);
  execution.startMoving();

  bool controller_active = true;

  geometry_msgs::PoseStamped oscillation_pose;
  ros::Time last_oscillation_reset = ros::Time::now();

  bool first_cycle = true;

  geometry_msgs::PoseStamped robot_pose;

  while (controller_active && ros::ok())
  {
    if (!robot_info_.getRobotPose(robot_pose))
    {
      controller_active = false;
      result.outcome = mbf_msgs::ExePathResult::TF_ERROR;
      result.message = "Could not get the robot pose!";
      goal_handle.setAborted(result, result.message);
      ROS_ERROR_STREAM_NAMED(name_, result.message << " Canceling the action call.");
      break;
    }

    if (first_cycle)
    {
      // init oscillation pose
      oscillation_pose = robot_pose;
    }

    state_moving_input = execution.getState();

    switch (state_moving_input)
    {
      case AbstractControllerExecution::STOPPED:
        ROS_WARN_STREAM_NAMED(name_, "The controller has been stopped!");
        controller_active = false;
        break;

      case AbstractControllerExecution::CANCELED:
        ROS_INFO_STREAM("Action \"ExePath\" canceled");
        result.outcome = mbf_msgs::ExePathResult::CANCELED;
        result.message = "Local planner canceled";
        goal_handle.setCanceled(result, result.message);
        break;

      case AbstractControllerExecution::STARTED:
        ROS_DEBUG_STREAM_NAMED(name_, "The moving has been started!");
        break;

        // in progress
      case AbstractControllerExecution::PLANNING:
        if (execution.isPatienceExceeded())
        {
          ROS_DEBUG_STREAM_NAMED(name_, "Local planner patience has been exceeded! Stopping controller...");
          // TODO planner is stuck, but we don't have currently any way to cancel it!
          // We will try to stop the thread, but does nothing with DWA or TR controllers
          execution.stopMoving();
        }
        break;

      case AbstractControllerExecution::MAX_RETRIES:
        ROS_WARN_STREAM_NAMED(name_, "The local planner has been aborted after it exceeded the maximum number of retries!");
        controller_active = false;
        result.outcome = execution.getOutcome();
        result.message = execution.getMessage();
        goal_handle.setAborted(result, result.message);
        break;

      case AbstractControllerExecution::PAT_EXCEEDED:
        ROS_WARN_STREAM_NAMED(name_, "The controller has been aborted after it exceeded the patience time ");
        controller_active = false;
        result.outcome = mbf_msgs::ExePathResult::PAT_EXCEEDED;
        result.message = "Controller exceeded allocated time";
        goal_handle.setAborted(result, result.message);
        break;

      case AbstractControllerExecution::NO_PLAN:
        ROS_WARN_STREAM_NAMED(name_, "The local planner has been started without any plan!");
        controller_active = false;
        result.outcome = mbf_msgs::ExePathResult::INVALID_PATH;
        result.message = "Controller started without a path";
        goal_handle.setAborted(result, result.message);
        break;

      case AbstractControllerExecution::EMPTY_PLAN:
        ROS_WARN_STREAM_NAMED(name_, "The controller has received an empty plan");
        controller_active = false;
        result.outcome = mbf_msgs::ExePathResult::INVALID_PATH;
        result.message = "Local planner started with an empty plan";
        goal_handle.setAborted(result, result.message);
        break;

      case AbstractControllerExecution::INVALID_PLAN:
        ROS_WARN_STREAM_NAMED(name_, "The controller has received an invalid plan");
        controller_active = false;
        result.outcome = mbf_msgs::ExePathResult::INVALID_PATH;
        result.message = "Controller started with an invalid plan";
        goal_handle.setAborted(result, result.message);
        break;

      case AbstractControllerExecution::NO_LOCAL_CMD:
        ROS_WARN_STREAM_THROTTLE_NAMED(3, name_, "No velocity command received from controller!");
        break;

      case AbstractControllerExecution::GOT_LOCAL_CMD:
        if (!oscillation_timeout.isZero())
        {
          // check if oscillating
          if (mbf_utility::distance(robot_pose, oscillation_pose) >= oscillation_distance)
          {
            last_oscillation_reset = ros::Time::now();
            oscillation_pose = robot_pose;
          }
          else if (last_oscillation_reset + oscillation_timeout < ros::Time::now())
          {
            ROS_WARN_STREAM_NAMED(name_, "The controller is oscillating for "
                << (ros::Time::now() - last_oscillation_reset).toSec() << "s");
            execution.stopMoving();
            controller_active = false;
            result.outcome = mbf_msgs::ExePathResult::OSCILLATION;
            result.message = "Oscillation detected!",
            goal_handle.setAborted(result, result.message);
            break;
          }
        }
        feedback.current_twist = execution.getLastValidCmdVel();
        feedback.current_pose = robot_pose;
        feedback.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose, goal_pose));
        feedback.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose, goal_pose));
        goal_handle.publishFeedback(feedback);
        break;

      case AbstractControllerExecution::ARRIVED_GOAL:
        ROS_DEBUG_STREAM_NAMED(name_, "Controller succeeded; arrived to goal");
        controller_active = false;
        result.outcome = mbf_msgs::ExePathResult::SUCCESS;
        result.message = "Controller succeeded; arrived to goal!";
        goal_handle.setSucceeded(result, result.message);
        break;

      case AbstractControllerExecution::INTERNAL_ERROR:
        ROS_FATAL_STREAM_NAMED(name_, "Internal error: Unknown error thrown by the plugin!"); // TODO getMessage from controller
        controller_active = false;
        result.outcome = mbf_msgs::ExePathResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown error thrown by the plugin!";
        goal_handle.setAborted(result, result.message);
        break;

      default:
        result.outcome = mbf_msgs::ExePathResult::INTERNAL_ERROR;
        std::stringstream ss;
        ss << "Internal error: Unknown state in a move base flex controller execution with the number: "
           << static_cast<int>(state_moving_input);
        result.message = ss.str();
        ROS_FATAL_STREAM_NAMED(name_, result.message);
        goal_handle.setAborted(result, result.message);
        controller_active = false;
    }

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

}

