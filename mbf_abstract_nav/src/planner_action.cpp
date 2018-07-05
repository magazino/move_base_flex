#include "mbf_abstract_nav/planner_action.h"

namespace mbf_abstract_nav{

PlannerAction::PlannerAction(
    const std::string &name,
    const RobotInformation &robot_info)
  : AbstractAction(name, robot_info, boost::bind(&mbf_abstract_nav::PlannerAction::run, this, _1, _2)), path_seq_count_(0)
{

  ros::NodeHandle nh;
  // informative topics: current goal and global path
  path_pub_ = nh.advertise<nav_msgs::Path>("global_path", 1);
  current_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("current_goal", 1);

}

void PlannerAction::run(GoalHandle &goal_handle, AbstractPlannerExecution &execution)
{
  const mbf_msgs::GetPathGoal& goal = *(goal_handle.getGoal().get());

  ROS_DEBUG_STREAM_NAMED(name_, "Start action "  << name_);

  mbf_msgs::GetPathResult result;
  geometry_msgs::PoseStamped start_pose;

  result.path.header.seq = path_seq_count_++;
  result.path.header.frame_id = robot_info_.getGlobalFrame();

  double tolerance = goal.tolerance;
  bool use_start_pose = goal.use_start_pose;
  current_goal_pub_.publish(goal.target_pose);

  bool planner_active = true;

  if(use_start_pose)
  {
    start_pose = goal.start_pose;
    const geometry_msgs::Point& p = start_pose.pose.position;
    ROS_INFO_STREAM_NAMED(name_, "Use the given start pose (" << p.x << ", " << p.y << ", " << p.z << ").");
  }
  else
  {
    // get the current robot pose
    if (!robot_info_.getRobotPose(start_pose))
    {
      result.outcome = mbf_msgs::GetPathResult::TF_ERROR;
      result.message = "Could not get the current robot pose!";
      goal_handle.setAborted(result, result.message);
      ROS_ERROR_STREAM_NAMED(name_, result.message << " Canceling the action call.");
      return;
    }
    else
    {
      const geometry_msgs::Point& p = start_pose.pose.position;
      ROS_DEBUG_STREAM_NAMED(name_, "Got the current robot pose at ("
          << p.x << ", " << p.y << ", " << p.z << ").");
    }
  }

  ROS_DEBUG_STREAM_NAMED(name_, "Starting the planning thread.");
  if (!execution.startPlanning(start_pose, goal.target_pose, tolerance))
  {
    result.outcome = mbf_msgs::GetPathResult::INTERNAL_ERROR;
    result.message = "Another thread is still planning!";
    goal_handle.setAborted(result, result.message);
    ROS_ERROR_STREAM_NAMED(name_, result.message << " Canceling the action call.");
    return;
  }

  AbstractPlannerExecution::PlanningState state_planning_input;

  std::vector<geometry_msgs::PoseStamped> plan, global_plan;
  double cost;

  while (planner_active && ros::ok())
  {
    // get the current state of the planning thread
    state_planning_input = execution.getState();

    switch (state_planning_input)
    {
      case AbstractPlannerExecution::INITIALIZED:
        ROS_DEBUG_STREAM_NAMED(name_, "planner state: initialized");
        break;

      case AbstractPlannerExecution::STARTED:
        ROS_DEBUG_STREAM_NAMED(name_, "planner state: started");
        break;

      case AbstractPlannerExecution::STOPPED:
        ROS_DEBUG_STREAM_NAMED(name_, "planner state: stopped");
        ROS_WARN_STREAM_NAMED(name_, "Planning has been stopped rigorously!");
        result.outcome = mbf_msgs::GetPathResult::STOPPED;
        result.message = "Global planner has been stopped!";
        goal_handle.setAborted(result, result.message);
        planner_active = false;
        break;

      case AbstractPlannerExecution::CANCELED:
        ROS_DEBUG_STREAM_NAMED(name_, "planner state: canceled");
        ROS_DEBUG_STREAM_NAMED(name_, "Global planner has been canceled successfully");
        result.path.header.stamp = ros::Time::now();
        result.outcome = mbf_msgs::GetPathResult::CANCELED;
        result.message = "Global planner has been canceled!";
        goal_handle.setCanceled(result, result.message);
        planner_active = false;
        break;

        // in progress
      case AbstractPlannerExecution::PLANNING:
        if (execution.isPatienceExceeded())
        {
          ROS_INFO_STREAM_NAMED(name_, "Global planner patience has been exceeded! "
              << "Cancel planning...");
          if (!execution.cancel())
          {
            ROS_WARN_STREAM_THROTTLE_NAMED(2.0, name_, "Cancel planning failed or is not supported; "
                "must wait until current plan finish!");
          }
        }
        else
        {
          ROS_DEBUG_THROTTLE_NAMED(2.0, name_, "planner state: planning");
        }
        break;

        // found a new plan
      case AbstractPlannerExecution::FOUND_PLAN:
        // set time stamp to now
        result.path.header.stamp = ros::Time::now();
        plan = execution.getPlan();
        publishPath(result.path.poses);

        ROS_DEBUG_STREAM_NAMED(name_, "planner state: found plan with cost: " << cost);

        if (!transformPlanToGlobalFrame(plan, global_plan))
        {
          result.outcome = mbf_msgs::GetPathResult::TF_ERROR;
          result.message = "Could not transform the plan to the global frame!";

          ROS_ERROR_STREAM_NAMED(name_, result.message << " Canceling the action call.");
          goal_handle.setAborted(result, result.message);
          planner_active = false;
          break;
        }

        if (global_plan.empty())
        {
          result.outcome = mbf_msgs::GetPathResult::EMPTY_PATH;
          result.message = "Global planner returned an empty path!";

          ROS_ERROR_STREAM_NAMED(name_, result.message);
          goal_handle.setAborted(result, result.message);
          planner_active = false;
          break;
        }

        result.path.poses = global_plan;
        result.cost = execution.getCost();
        result.outcome = execution.getOutcome();
        result.message = execution.getMessage();
        goal_handle.setSucceeded(result, result.message);

        planner_active = false;
        break;

        // no plan found
      case AbstractPlannerExecution::NO_PLAN_FOUND:
        ROS_DEBUG_STREAM_NAMED(name_, "planner state: no plan found");
        result.outcome = execution.getOutcome();
        result.message = execution.getMessage();
        goal_handle.setAborted(result, result.message);
        planner_active = false;
        break;

      case AbstractPlannerExecution::MAX_RETRIES:
        ROS_DEBUG_STREAM_NAMED(name_, "Global planner reached the maximum number of retries");
        result.outcome = execution.getOutcome();
        result.message = execution.getMessage();
        goal_handle.setAborted(result, result.message);
        planner_active = false;
        break;

      case AbstractPlannerExecution::PAT_EXCEEDED:
        ROS_DEBUG_STREAM_NAMED(name_, "Global planner exceeded the patience time");
        result.outcome = mbf_msgs::GetPathResult::PAT_EXCEEDED;
        result.message = "Global planner exceeded the patience time";
        goal_handle.setAborted(result, result.message);
        planner_active = false;
        break;

      case AbstractPlannerExecution::INTERNAL_ERROR:
        ROS_FATAL_STREAM_NAMED(name_, "Internal error: Unknown error thrown by the plugin!"); // TODO getMessage from planning
        planner_active = false;
        result.outcome = mbf_msgs::GetPathResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown error thrown by the plugin!";
        goal_handle.setAborted(result, result.message);
        break;

      default:
        result.outcome = mbf_msgs::GetPathResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown state in a move base flex planner execution with the number: " + state_planning_input;
        ROS_FATAL_STREAM_NAMED(name_, result.message);
        goal_handle.setAborted(result, result.message);
        planner_active = false;
    }


    if (planner_active)
    {
      // try to sleep a bit
      // normally this thread should be woken up from the planner execution thread
      // in order to transfer the results to the controller.
      boost::mutex mutex;
      boost::unique_lock<boost::mutex> lock(mutex);
      execution.waitForStateUpdate(boost::chrono::milliseconds(500));
    }
  }  // while (planner_active && ros::ok())

  if (!planner_active)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "\"" << name_ << "\" action ended properly.");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_, "\"" << name_ << "\" action has been stopped!");
  }
}

void PlannerAction::publishPath(
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

bool PlannerAction::transformPlanToGlobalFrame(
    std::vector<geometry_msgs::PoseStamped> &plan, std::vector<geometry_msgs::PoseStamped> &global_plan)
{
  global_plan.clear();
  std::vector<geometry_msgs::PoseStamped>::iterator iter;
  bool tf_success = false;
  for (iter = plan.begin(); iter != plan.end(); ++iter)
  {
    geometry_msgs::PoseStamped global_pose;
    tf_success = mbf_utility::transformPose(robot_info_.getTransformListener(), robot_info_.getGlobalFrame(), iter->header.stamp,
                                            robot_info_.getTfTimeout(), *iter, robot_info_.getGlobalFrame(), global_pose);
    if (!tf_success)
    {
      ROS_ERROR_STREAM("Can not transform pose from the \"" << iter->header.frame_id << "\" frame into the \""
                                                            << robot_info_.getGlobalFrame() << "\" frame !");
      return false;
    }
    global_plan.push_back(global_pose);
  }
  return true;
}

} /* namespace mbf_abstract_nav */

