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
 *  move_base_action.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */
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

  //! timeout after a oscillation is detected
  ros::Duration oscillation_timeout_;

  //! minimal move distance to not detect an oscillation
  double oscillation_distance_;

  GoalHandle goal_handle_;

  std::string name_;

  RobotInformation robot_info_;

  //! current robot pose; updated with exe_path action feedback
  geometry_msgs::PoseStamped robot_pose_;

  //! current goal pose; used to compute remaining distance and angle
  geometry_msgs::PoseStamped goal_pose_;

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

  //! true, if recovery behavior for the MoveBase action is enabled.
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
