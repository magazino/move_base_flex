#include <mbf_utility/navigation_utility.h>

#include "mbf_abstract_nav/MoveBaseFlexConfig.h"
#include "mbf_abstract_nav/move_base_action.h"

namespace mbf_abstract_nav
{

MoveBaseAction::MoveBaseAction(const std::string &name,
                               const RobotInformation &robot_info,
                               const std::vector<std::string> &behaviors)
  :  name_(name), robot_info_(robot_info), private_nh_("~"),
     action_client_exe_path_(private_nh_, "exe_path"),
     action_client_get_path_(private_nh_, "get_path"),
     action_client_recovery_(private_nh_, "recovery"),
     oscillation_timeout_(0),
     oscillation_distance_(0),
     recovery_enabled_(true),
     behaviors_(behaviors),
     action_state_(NONE),
     recovery_trigger_(NONE),
     replanning_(false),
     replanning_rate_(1.0)
{
}

MoveBaseAction::~MoveBaseAction()
{
}

void MoveBaseAction::reconfigure(
    mbf_abstract_nav::MoveBaseFlexConfig &config, uint32_t level)
{
  if (config.planner_frequency > 0.0)
  {
    boost::lock_guard<boost::mutex> lock_guard(replanning_mtx_);
    if (!replanning_)
    {
      replanning_ = true;
      if (action_state_ == EXE_PATH)
      {
        // exe_path is running and user has enabled replanning
        ROS_INFO_STREAM_NAMED("move_base", "Start replanning, using the \"get_path\" action!");
        action_client_get_path_.sendGoal(
            get_path_goal_,
            boost::bind(&MoveBaseAction::actionGetPathReplanningDone, this, _1, _2)
        );
      }
    }
    replanning_rate_ = ros::Rate(config.planner_frequency);
  }
  else
    replanning_ = false;
  oscillation_timeout_ = ros::Duration(config.oscillation_timeout);
  oscillation_distance_ = config.oscillation_distance;
  recovery_enabled_ = config.recovery_enabled;
}

void MoveBaseAction::cancel()
{
  action_state_ = CANCELED;

  if(!action_client_get_path_.getState().isDone())
  {
    action_client_get_path_.cancelGoal();
  }

  if(!action_client_exe_path_.getState().isDone())
  {
    action_client_exe_path_.cancelGoal();
  }

  if(!action_client_recovery_.getState().isDone())
  {
    action_client_recovery_.cancelGoal();
  }
}

void MoveBaseAction::start(GoalHandle &goal_handle)
{
  action_state_ = GET_PATH;

  goal_handle.setAccepted();

  goal_handle_ = goal_handle;

  ROS_DEBUG_STREAM_NAMED("move_base", "Start action "  << "move_base");

  const mbf_msgs::MoveBaseGoal& goal = *(goal_handle.getGoal().get());

  mbf_msgs::MoveBaseResult move_base_result;

  get_path_goal_.target_pose = goal.target_pose;
  get_path_goal_.use_start_pose = false; // use the robot pose
  get_path_goal_.planner = goal.planner;
  exe_path_goal_.controller = goal.controller;

  ros::Duration connection_timeout(1.0);

  last_oscillation_reset_ = ros::Time::now();

  // start recovering with the first behavior, use the recovery behaviors from the action request, if specified,
  // otherwise, use all loaded behaviors.

  recovery_behaviors_ = goal.recovery_behaviors.empty() ? behaviors_ : goal.recovery_behaviors;
  current_recovery_behavior_ = recovery_behaviors_.begin();

  geometry_msgs::PoseStamped robot_pose;
  // get the current robot pose only at the beginning, as exe_path will keep updating it as we move
  if (!robot_info_.getRobotPose(robot_pose))
  {
    ROS_ERROR_STREAM_NAMED("move_base", "Could not get the current robot pose!");
    move_base_result.message = "Could not get the current robot pose!";
    move_base_result.outcome = mbf_msgs::MoveBaseResult::TF_ERROR;
    goal_handle.setAborted(move_base_result, move_base_result.message);
    return;
  }

  // wait for server connections
  if (!action_client_get_path_.waitForServer(connection_timeout) ||
      !action_client_exe_path_.waitForServer(connection_timeout) ||
      !action_client_recovery_.waitForServer(connection_timeout))
  {
    ROS_ERROR_STREAM_NAMED("move_base", "Could not connect to one or more of move_base_flex actions:"
        "\"get_path\" , \"exe_path\", \"recovery \"!");
    move_base_result.outcome = mbf_msgs::MoveBaseResult::INTERNAL_ERROR;
    move_base_result.message = "Could not connect to the move_base_flex actions!";
    goal_handle.setAborted(move_base_result, move_base_result.message);
    return;
  }

  // call get_path action server to get a first plan
  action_client_get_path_.sendGoal(
      get_path_goal_,
      boost::bind(&MoveBaseAction::actionGetPathDone, this, _1, _2));
}

void MoveBaseAction::actionExePathActive()
{
  ROS_DEBUG_STREAM_NAMED("move_base", "The \"exe_path\" action is active.");

  if(replanning_)
  {
    // exe_path started; time to start replanning
    boost::lock_guard<boost::mutex> lock_guard(replanning_mtx_);
    ROS_INFO_STREAM_NAMED("move_base", "Start replanning, using the \"get_path\" action!");
    action_client_get_path_.sendGoal(
        get_path_goal_,
        boost::bind(&MoveBaseAction::actionGetPathReplanningDone, this, _1, _2)
    );
  }
}

void MoveBaseAction::actionExePathFeedback(
    const mbf_msgs::ExePathFeedbackConstPtr &feedback)
{
  move_base_feedback_.angle_to_goal = feedback->angle_to_goal;
  move_base_feedback_.dist_to_goal = feedback->dist_to_goal;
  move_base_feedback_.current_pose = feedback->current_pose;
  move_base_feedback_.current_twist = feedback->current_twist;
  robot_pose_ = feedback->current_pose;
  goal_handle_.publishFeedback(move_base_feedback_);

  // we create a navigation-level oscillation detection using exe_path action's feedback,
  // as the later doesn't handle oscillations created by quickly failing repeated plans

  // if oscillation detection is enabled by osciallation_timeout != 0
  if (!oscillation_timeout_.isZero())
  {
    // check if oscillating
    // moved more than the minimum oscillation distance
    if (mbf_utility::distance(robot_pose_, last_oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      last_oscillation_pose_ = robot_pose_;

      if (recovery_trigger_ == OSCILLATING)
      {
        ROS_INFO_NAMED("move_base", "Recovered from robot oscillation: restart recovery behaviors");
        current_recovery_behavior_ = recovery_behaviors_.begin();
        recovery_trigger_ = NONE;
      }
    }
    else if (last_oscillation_reset_ + oscillation_timeout_ < ros::Time::now())
    {
      std::stringstream oscillation_msgs;
      oscillation_msgs << "Robot is oscillating for " << ((ros::Time::now() - last_oscillation_reset_).toSec()) << "s!";
      ROS_WARN_STREAM_NAMED("exe_path", oscillation_msgs.str());
      action_client_exe_path_.cancelGoal();

      if (attemptRecovery())
      {
        recovery_trigger_ = OSCILLATING;
      }
      else
      {
        mbf_msgs::MoveBaseResult move_base_result;
        move_base_result.outcome = OSCILLATING;
        if(recovery_enabled_)
          move_base_result.message = oscillation_msgs.str() + " No recovery behaviors for the move_base action are defined!";
        else
          move_base_result.message = oscillation_msgs.str() + " Recovery is disabled for the move_base action! use the param \"enable_recovery\"";
        move_base_result.final_pose = robot_pose_;
        move_base_result.angle_to_goal = move_base_feedback_.angle_to_goal;
        move_base_result.dist_to_goal = move_base_feedback_.dist_to_goal;
        goal_handle_.setAborted(move_base_result, move_base_result.message);
      }
    }
  }
}

void MoveBaseAction::actionGetPathDone(
    const actionlib::SimpleClientGoalState &state,
    const mbf_msgs::GetPathResultConstPtr &result_ptr)
{
  action_state_ =  FAILED;

  const mbf_msgs::GetPathResult &result = *(result_ptr.get());
  const mbf_msgs::MoveBaseGoal& goal = *(goal_handle_.getGoal().get());
  mbf_msgs::MoveBaseResult move_base_result;
  switch (state.state_)
  {
    case actionlib::SimpleClientGoalState::PENDING:
      ROS_FATAL_STREAM_NAMED("move_base", "get_path PENDING state not implemented, this should not be reachable!");
      break;

    case actionlib::SimpleClientGoalState::SUCCEEDED:
      ROS_DEBUG_STREAM_NAMED("move_base", "Action \""
          << "move_base\" received a path from \""
          << "get_path\": " << state.getText());

      exe_path_goal_.path = result.path;
      ROS_DEBUG_STREAM_NAMED("move_base", "Action \""
          << "move_base\" sends the path to \""
          << "exe_path\".");

      if (recovery_trigger_ == GET_PATH)
      {
        ROS_WARN_NAMED("move_base", "Recovered from planner failure: restart recovery behaviors");
        current_recovery_behavior_ = recovery_behaviors_.begin();
        recovery_trigger_ = NONE;
      }

      action_client_exe_path_.sendGoal(
          exe_path_goal_,
          boost::bind(&MoveBaseAction::actionExePathDone, this, _1, _2),
          boost::bind(&MoveBaseAction::actionExePathActive, this),
          boost::bind(&MoveBaseAction::actionExePathFeedback, this, _1));
      action_state_ = EXE_PATH;
      break;

    case actionlib::SimpleClientGoalState::ABORTED:

      if (attemptRecovery())
      {
        recovery_trigger_ = GET_PATH;
      }
      else
      {
        // copy result from get_path action
        move_base_result.outcome = result.outcome;
        move_base_result.message = result.message;
        move_base_result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal.target_pose));
        move_base_result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal.target_pose));
        move_base_result.final_pose = robot_pose_;

        ROS_WARN_STREAM_NAMED("move_base", "Abort the execution of the planner: " << result.message);
        goal_handle_.setAborted(move_base_result, state.getText());
      }
      break;

    case actionlib::SimpleClientGoalState::PREEMPTED:
      // the get_path action has been preempted.

      // copy result from get_path action
      move_base_result.outcome = result.outcome;
      move_base_result.message = result.message;
      move_base_result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal.target_pose));
      move_base_result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal.target_pose));
      move_base_result.final_pose = robot_pose_;
      goal_handle_.setCanceled(move_base_result, state.getText());
      break;

    case actionlib::SimpleClientGoalState::RECALLED:
    case actionlib::SimpleClientGoalState::REJECTED:
      ROS_FATAL_STREAM_NAMED("move_base", "The states RECALLED and REJECTED are not implemented in the SimpleActionServer!");
      goal_handle_.setAborted();
      break;

    case actionlib::SimpleClientGoalState::LOST:
      ROS_FATAL_STREAM_NAMED("move_base", "Connection lost to the action \"get_path\"!");
      goal_handle_.setAborted();
      break;

    default:
      ROS_FATAL_STREAM_NAMED("move_base", "Reached unknown action server state!");
      goal_handle_.setAborted();
      break;
  }

}

void MoveBaseAction::actionExePathDone(
    const actionlib::SimpleClientGoalState &state,
    const mbf_msgs::ExePathResultConstPtr &result_ptr)
{
  action_state_ =  FAILED;

  ROS_DEBUG_STREAM_NAMED("move_base", "Action \"exe_path\" finished.");

  const mbf_msgs::ExePathResult& result = *(result_ptr.get());
  const mbf_msgs::MoveBaseGoal& goal = *(goal_handle_.getGoal().get());
  mbf_msgs::MoveBaseResult move_base_result;

  // copy result from get_path action
  move_base_result.outcome = result.outcome;
  move_base_result.message = result.message;
  move_base_result.dist_to_goal = result.dist_to_goal;
  move_base_result.angle_to_goal = result.angle_to_goal;
  move_base_result.final_pose = result.final_pose;

  ROS_DEBUG_STREAM_NAMED("exe_path", "Current state:" << state.toString());

  switch (state.state_)
  {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      move_base_result.outcome = mbf_msgs::MoveBaseResult::SUCCESS;
      move_base_result.message = "Action \"move_base\" succeeded!";
      ROS_INFO_STREAM_NAMED("move_base", move_base_result.message);
      goal_handle_.setSucceeded(move_base_result, move_base_result.message);
      action_state_ = SUCCEEDED;
      break;

    case actionlib::SimpleClientGoalState::ABORTED:
      switch (result.outcome)
      {
        case mbf_msgs::ExePathResult::INVALID_PATH:
        case mbf_msgs::ExePathResult::TF_ERROR:
        case mbf_msgs::ExePathResult::NOT_INITIALIZED:
        case mbf_msgs::ExePathResult::INVALID_PLUGIN:
        case mbf_msgs::ExePathResult::INTERNAL_ERROR:
          // none of these errors is recoverable
          goal_handle_.setAborted(move_base_result, state.getText());
          break;

        default:
          // all the rest are, so we start calling the recovery behaviors in sequence

          if (attemptRecovery())
          {
            recovery_trigger_ = EXE_PATH;
          }
          else
          {
            ROS_WARN_STREAM_NAMED("move_base", "Abort the execution of the controller: " << result.message);
            goal_handle_.setAborted(move_base_result, state.getText());
          }
          break;
      }
      break;

    case actionlib::SimpleClientGoalState::PREEMPTED:
      // action was preempted successfully!
      ROS_DEBUG_STREAM_NAMED("move_base", "The action \""
          << "exe_path" << "\" was preempted successfully!");
      // TODO
      break;

    case actionlib::SimpleClientGoalState::RECALLED:
      ROS_DEBUG_STREAM_NAMED("move_base", "The action \""
          << "exe_path" << "\" was recalled!");
      // TODO
      break;

    case actionlib::SimpleClientGoalState::REJECTED:
      ROS_DEBUG_STREAM_NAMED("move_base", "The action \""
          << "exe_path" << "\" was rejected!");
      // TODO
      break;

    case actionlib::SimpleClientGoalState::LOST:
      // TODO
      break;

    default:
      ROS_FATAL_STREAM_NAMED("move_base",
                             "Reached unreachable case! Unknown SimpleActionServer state!");
      goal_handle_.setAborted();
      break;
  }
}

bool MoveBaseAction::attemptRecovery()
{
  if (!recovery_enabled_)
  {
    ROS_WARN_STREAM_NAMED("move_base", "Recovery behaviors are disabled!");
    return false;
  }

  if (current_recovery_behavior_ == recovery_behaviors_.end())
  {
    if (recovery_behaviors_.empty())
    {
      ROS_WARN_STREAM_NAMED("move_base", "No Recovery Behaviors loaded!");
    }
    else
    {
      ROS_WARN_STREAM_NAMED("move_base", "Executed all available recovery behaviors!");
    }
    return false;
  }

  recovery_goal_.behavior = *current_recovery_behavior_;
  ROS_DEBUG_STREAM_NAMED("move_base", "Start recovery behavior\""
      << *current_recovery_behavior_ <<"\".");
  action_client_recovery_.sendGoal(
      recovery_goal_,
      boost::bind(&MoveBaseAction::actionRecoveryDone, this, _1, _2)
  );
  action_state_ = RECOVERY;
  return true;
}

void MoveBaseAction::actionRecoveryDone(
    const actionlib::SimpleClientGoalState &state,
    const mbf_msgs::RecoveryResultConstPtr &result_ptr)
{
  action_state_ =  FAILED;  // unless recovery succeeds or gets canceled...

  const mbf_msgs::RecoveryResult& result = *(result_ptr.get());
  const mbf_msgs::MoveBaseGoal& goal = *(goal_handle_.getGoal().get());
  mbf_msgs::MoveBaseResult move_base_result;

  // copy result from get_path action
  move_base_result.outcome = result.outcome;
  move_base_result.message = result.message;
  move_base_result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal.target_pose));
  move_base_result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal.target_pose));
  move_base_result.final_pose = robot_pose_;

  switch (state.state_)
  {
    case actionlib::SimpleClientGoalState::ABORTED:
      ROS_DEBUG_STREAM_NAMED("move_base", "Recovery behavior aborted!");
      ROS_DEBUG_STREAM_NAMED("move_base", "The recovery behavior \""
          << *current_recovery_behavior_ << "\" failed. ");
      ROS_DEBUG_STREAM("Recovery behavior message: " << result.message
                                                     << ", outcome: " << result.outcome);

      current_recovery_behavior_++; // use next behavior;
      if (current_recovery_behavior_ == recovery_behaviors_.end())
      {
        ROS_DEBUG_STREAM_NAMED("move_base",
                               "All recovery behaviors failed. Abort recovering and abort the move_base action");
        goal_handle_.setAborted(move_base_result, "All recovery behaviors failed.");
      }
      else
      {
        recovery_goal_.behavior = *current_recovery_behavior_;

        ROS_INFO_STREAM_NAMED("move_base", "Run the next recovery behavior\""
            << *current_recovery_behavior_ << "\".");
        action_client_recovery_.sendGoal(
            recovery_goal_,
            boost::bind(&MoveBaseAction::actionRecoveryDone, this, _1, _2)
        );
      }
      break;
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      //go to planning state
      ROS_DEBUG_STREAM_NAMED("move_base", "Execution of the recovery behavior \""
          << *current_recovery_behavior_ << "\" succeeded!");
      ROS_DEBUG_STREAM_NAMED("move_base",
                             "Try planning again and increment the current recovery behavior in the list.");
      action_state_ = GET_PATH;
      current_recovery_behavior_++; // use next behavior, the next time;
      action_client_get_path_.sendGoal(
          get_path_goal_,
          boost::bind(&MoveBaseAction::actionGetPathDone, this, _1, _2)
      );
      break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
      ROS_INFO_STREAM_NAMED("move_base",
                             "The recovery action has been preempted!");
      if(action_state_ == CANCELED)
        goal_handle_.setCanceled();
      break;

    case actionlib::SimpleClientGoalState::RECALLED:
      ROS_INFO_STREAM_NAMED("move_base",
                            "The recovery action has been recalled!");
      if(action_state_ == CANCELED)
        goal_handle_.setCanceled();
      break;

    case actionlib::SimpleClientGoalState::REJECTED:
      ROS_FATAL_STREAM_NAMED("move_base",
                             "The recovery action has been rejected!");
      goal_handle_.setRejected();
      break;
    case actionlib::SimpleClientGoalState::LOST:
      ROS_FATAL_STREAM_NAMED("move_base",
                             "The recovery action has lost the connection to the server!");
      goal_handle_.setAborted();
      break;
    default:
      ROS_FATAL_STREAM_NAMED("move_base",
                             "Reached unreachable case! Unknown state!");
      goal_handle_.setAborted();
      break;
  }
}

void MoveBaseAction::actionGetPathReplanningDone(
    const actionlib::SimpleClientGoalState &state,
    const mbf_msgs::GetPathResultConstPtr &result)
{
  if (!replanning_ || action_state_ != EXE_PATH)
    return; // replan only while following a path and if replanning is enabled (can be disabled by dynamic reconfigure)

  if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_DEBUG_STREAM_NAMED("move_base", "Replanning succeeded; sending a goal to \"exe_path\" with the new plan");
    exe_path_goal_.path = result->path;
    mbf_msgs::ExePathGoal goal(exe_path_goal_);
    action_client_exe_path_.sendGoal(
        goal,
        boost::bind(&MoveBaseAction::actionExePathDone, this, _1, _2),
        boost::bind(&MoveBaseAction::actionExePathActive, this),
        boost::bind(&MoveBaseAction::actionExePathFeedback, this, _1));
  }

  replanning_mtx_.lock();
  replanning_rate_.sleep();
  replanning_mtx_.unlock();

  if (!replanning_ || action_state_ != EXE_PATH)
    return; // another chance to stop replannings after waiting for replanning period

  ROS_DEBUG_STREAM_NAMED("move_base", "Next replanning cycle, using the \"get_path\" action!");
  action_client_get_path_.sendGoal(
      get_path_goal_,
      boost::bind(&MoveBaseAction::actionGetPathReplanningDone, this, _1, _2)); // replanning
}


} /* namespace mbf_abstract_nav */

