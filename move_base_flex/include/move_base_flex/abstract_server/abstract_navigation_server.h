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
 *  abstract_navigation_server.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MOVE_BASE_FLEX__ABSTRACT_NAVIGATION_SERVER_H_
#define MOVE_BASE_FLEX__ABSTRACT_NAVIGATION_SERVER_H_

#include "abstract_planner_execution.h"
#include "abstract_controller_execution.h"
#include "abstract_recovery_execution.h"

#include "move_base_flex/MoveBaseFlexConfig.h"

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_flex_msgs/GetPathAction.h>
#include <move_base_flex_msgs/ExePathAction.h>
#include <move_base_flex_msgs/RecoveryAction.h>
#include <move_base_flex_msgs/MoveBaseAction.h>

#include "move_base_flex/navigation_utility.h"
#include "move_base_flex/abstract_server/abstract_navigation_server.h"

namespace move_base_flex
{

// Get Path Action
typedef actionlib::SimpleActionServer<move_base_flex_msgs::GetPathAction> ActionServerGetPath;
typedef boost::shared_ptr<ActionServerGetPath> ActionServerGetPathPtr;

// Exe Path Action
typedef actionlib::SimpleActionServer<move_base_flex_msgs::ExePathAction> ActionServerExePath;
typedef boost::shared_ptr<ActionServerExePath> ActionServerExePathPtr;

// Recovery Action Server
typedef actionlib::SimpleActionServer<move_base_flex_msgs::RecoveryAction> ActionServerRecovery;
typedef boost::shared_ptr<ActionServerRecovery> ActionServerRecoveryPtr;

// MoveBase Action Server
typedef actionlib::SimpleActionServer<move_base_flex_msgs::MoveBaseAction> ActionServerMoveBase;
typedef boost::shared_ptr<ActionServerMoveBase> ActionServerMoveBasePtr;

// Action Clients
typedef actionlib::SimpleActionClient<move_base_flex_msgs::GetPathAction> ActionClientGetPath;
typedef actionlib::SimpleActionClient<move_base_flex_msgs::ExePathAction> ActionClientExePath;
typedef actionlib::SimpleActionClient<move_base_flex_msgs::RecoveryAction> ActionClientRecovery;

// Action Names
const std::string name_action_exe_path = "exe_path";
const std::string name_action_get_path = "get_path";
const std::string name_action_recovery = "recovery";
const std::string name_action_move_base = "move_base";

template<typename LOCAL_PLANNER_BASE, typename GLOBAL_PLANNER_BASE, typename RECOVERY_BEHAVIOR_BASE>
  class AbstractNavigationServer
  {
  public:

    AbstractNavigationServer(const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
                             typename AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::Ptr planning_ptr,
                             typename AbstractControllerExecution<LOCAL_PLANNER_BASE>::Ptr moving_ptr,
                             typename AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::Ptr recovery_ptr);

    virtual ~AbstractNavigationServer();

    virtual void callActionGetPath(const move_base_flex_msgs::GetPathGoalConstPtr &goal);

    virtual void callActionExePath(const move_base_flex_msgs::ExePathGoalConstPtr &goal);

    virtual void callActionRecovery(const move_base_flex_msgs::RecoveryGoalConstPtr &goal);

    virtual void callActionMoveBase(const move_base_flex_msgs::MoveBaseGoalConstPtr &goal);

    virtual void actionMoveBaseExePathFeedback(const move_base_flex_msgs::ExePathFeedbackConstPtr &feedback);

    virtual void startActionServers();

    virtual void initializeControllerComponents();

    bool getRobotPose(geometry_msgs::PoseStamped &robot_pose);

  protected:

    ActionServerRecoveryPtr action_server_recovery_ptr_;
    ActionServerExePathPtr action_server_exe_path_ptr_;
    ActionServerGetPathPtr action_server_get_path_ptr_;
    ActionServerMoveBasePtr action_server_move_base_ptr_;

    ros::Publisher current_goal_pub_;

    void publishPath(std::vector<geometry_msgs::PoseStamped> &plan);

    bool transformPlanToGlobalFrame(std::vector<geometry_msgs::PoseStamped> &plan,
                                    std::vector<geometry_msgs::PoseStamped> &global_plan);

    virtual void reconfigure(move_base_flex::MoveBaseFlexConfig &config, uint32_t level);

    // dynamic reconfigure attributes and methods
    boost::shared_ptr<dynamic_reconfigure::Server<move_base_flex::MoveBaseFlexConfig> > dsrv_;
    boost::recursive_mutex configuration_mutex_;
    move_base_flex::MoveBaseFlexConfig last_config_;
    move_base_flex::MoveBaseFlexConfig default_config_;
    bool setup_reconfigure_;

    // condition to wake up control thread
    boost::condition_variable condition_;

    // frames to get the current global robot pose
    std::string robot_frame_;
    std::string global_frame_;

    double goal_tolerance_;
    double tf_timeout_;

    const boost::shared_ptr<tf::TransformListener> tf_listener_ptr_;

    typename AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::Ptr planning_ptr_;
    typename AbstractControllerExecution<LOCAL_PLANNER_BASE>::Ptr moving_ptr_;
    typename AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::Ptr recovery_ptr_;

    bool active_moving_;
    bool active_planning_;
    bool active_recovery_;

    ros::Duration oscillation_timeout_;
    double oscillation_distance_;

    bool recovery_behavior_enabled_;
    bool clearing_rotation_allowed_;

  private:
    typename AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::PlanningState state_planning_input_;
    typename AbstractControllerExecution<LOCAL_PLANNER_BASE>::ControllerState state_moving_input_;
    typename AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::RecoveryState state_recovery_input_;

    ros::Publisher path_pub_;
  };

} /* namespace move_base_flex */

#include "move_base_flex/abstract_server/impl/abstract_navigation_server.tcc"

#endif /* navigation_controller.h */
