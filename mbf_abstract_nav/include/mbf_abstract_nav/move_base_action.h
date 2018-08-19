#ifndef MBF_ABSTRACT_NAV__MOVE_BASE_ACTION_H_
#define MBF_ABSTRACT_NAV__MOVE_BASE_ACTION_H_

#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/RecoveryAction.h>

#include "mbf_abstract_nav/MoveBaseFlexConfig.h"
#include "mbf_abstract_nav/robot_information.h"


namespace mbf_abstract_nav
{

class MoveBaseAction
{
 public:

  //! Action clients for the MoveBase action
  typedef actionlib::SimpleActionClient<mbf_msgs::GetPathAction> ActionClientGetPath;
  typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> ActionClientExePath;
  typedef actionlib::SimpleActionClient<mbf_msgs::RecoveryAction> ActionClientRecovery;

  typedef actionlib::ActionServer<mbf_msgs::MoveBaseAction>::GoalHandle GoalHandle;

  MoveBaseAction(const std::string &name, const RobotInformation &robot_info, const std::vector<std::string> &controllers);

  ~MoveBaseAction();

  void start(GoalHandle &goal_handle);

  void cancel();

  void reconfigure(
      mbf_abstract_nav::MoveBaseFlexConfig &config, uint32_t level);

 protected:

  void actionExePathFeedback(const mbf_msgs::ExePathFeedbackConstPtr &feedback);

  void actionGetPathDone(
      const actionlib::SimpleClientGoalState &state,
      const mbf_msgs::GetPathResultConstPtr &result);

  void actionExePathActive();

  void actionExePathDone(
      const actionlib::SimpleClientGoalState &state,
      const mbf_msgs::ExePathResultConstPtr &result);

  void actionGetPathReplanningDone(
      const actionlib::SimpleClientGoalState &state,
      const mbf_msgs::GetPathResultConstPtr &result);

  void actionRecoveryDone(
      const actionlib::SimpleClientGoalState &state,
      const mbf_msgs::RecoveryResultConstPtr &result);

  bool attemptRecovery();

  mbf_msgs::ExePathGoal exe_path_goal_;
  mbf_msgs::GetPathGoal get_path_goal_;
  mbf_msgs::RecoveryGoal recovery_goal_;

  geometry_msgs::PoseStamped last_oscillation_pose_;
  ros::Time last_oscillation_reset_;

  ros::Duration oscillation_timeout_;

  double oscillation_distance_;

  GoalHandle goal_handle_;

  std::string name_;

  RobotInformation robot_info_;

  geometry_msgs::PoseStamped robot_pose_;

  ros::NodeHandle private_nh_;

  //! Action client used by the move_base action
  ActionClientExePath action_client_exe_path_;

  //! Action client used by the move_base action
  ActionClientGetPath action_client_get_path_;

  //! Action client used by the move_base action
  ActionClientRecovery action_client_recovery_;

  bool replanning_;
  ros::Rate replanning_rate_;
  boost::mutex replanning_mtx_;

  bool recovery_enabled_;

  mbf_msgs::MoveBaseFeedback move_base_feedback_;

  std::vector<std::string> recovery_behaviors_;

  std::vector<std::string>::iterator current_recovery_behavior_;

  const std::vector<std::string> &behaviors_;

  enum MoveBaseActionState
  {
    NONE,
    GET_PATH,
    EXE_PATH,
    RECOVERY,
    OSCILLATING,
    SUCCEEDED,
    CANCELED,
    FAILED
  };

  MoveBaseActionState action_state_;
  MoveBaseActionState recovery_trigger_;
};

} /* mbf_abstract_nav */

#endif //MBF_ABSTRACT_NAV__MOVE_BASE_ACTION_H_
