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

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_NAVIGATION_SERVER_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_NAVIGATION_SERVER_H_

#include <string>
#include <stdint.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <actionlib/server/action_server.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <mbf_utility/navigation_utility.h>

#include "mbf_abstract_nav/abstract_plugin_manager.h"
#include "mbf_abstract_nav/abstract_planner_execution.h"
#include "mbf_abstract_nav/abstract_controller_execution.h"
#include "mbf_abstract_nav/abstract_recovery_execution.h"

#include "mbf_abstract_nav/planner_action.h"
#include "mbf_abstract_nav/controller_action.h"
#include "mbf_abstract_nav/recovery_action.h"
#include "mbf_abstract_nav/move_base_action.h"

#include "mbf_abstract_nav/MoveBaseFlexConfig.h"

namespace mbf_abstract_nav
{
/**
 * @defgroup abstract_server Abstract Server
 * @brief Classes belonging to the Abstract Server level.
 */

/**
 * @defgroup navigation_server Navigation Server Classes.
 * @brief Classes combining the core logic and providing concrete implementations.
 */


//! GetPath action server
typedef actionlib::ActionServer<mbf_msgs::GetPathAction> ActionServerGetPath;
typedef boost::shared_ptr<ActionServerGetPath> ActionServerGetPathPtr;

//! ExePath action server
typedef actionlib::ActionServer<mbf_msgs::ExePathAction> ActionServerExePath;
typedef boost::shared_ptr<ActionServerExePath> ActionServerExePathPtr;

//! Recovery action server
typedef actionlib::ActionServer<mbf_msgs::RecoveryAction> ActionServerRecovery;
typedef boost::shared_ptr<ActionServerRecovery> ActionServerRecoveryPtr;

//! MoveBase action server
typedef actionlib::ActionServer<mbf_msgs::MoveBaseAction> ActionServerMoveBase;
typedef boost::shared_ptr<ActionServerMoveBase> ActionServerMoveBasePtr;

//! ExePath action topic name
const std::string name_action_exe_path = "exe_path";
//! GetPath action topic name
const std::string name_action_get_path = "get_path";
//! Recovery action topic name
const std::string name_action_recovery = "recovery";
//! MoveBase action topic name
const std::string name_action_move_base = "move_base";


typedef boost::shared_ptr<dynamic_reconfigure::Server<mbf_abstract_nav::MoveBaseFlexConfig> > DynamicReconfigureServer;

/**
 * @brief The AbstractNavigationServer is the abstract base class for all navigation servers in move_base_flex
 *        and bundles the @ref controller_execution "controller execution classes",the @ref planner_execution
 *        "planner execution classes" and the @ref recovery_execution "recovery execution classes". It provides
 *        the following action servers ActionServerGetPath -> callActionGetPath(), ActionServerExePath -> callActionExePath(),
 *        ActionServerRecovery -> callActionRecovery() and ActionServerMoveBase -> callActionMoveBase().
 *
 * @ingroup abstract_server navigation_server
 */
  class AbstractNavigationServer
  {
  public:

    /**
     * @brief Constructor, reads all parameters and initializes all action servers and creates the plugin instances.
     *        Parameters are the concrete implementations of the abstract classes.
     * @param tf_listener_ptr shared pointer to the common TransformListener buffering transformations
     * @param planning_ptr shared pointer to an object of the concrete derived implementation of the AbstractPlannerExecution
     * @param moving_ptr shared pointer to an object of the concrete derived implementation of the AbstractControllerExecution
     * @param recovery_ptr shared pointer to an object of the concrete derived implementation of the AbstractRecoveryExecution
     */
    AbstractNavigationServer(const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr);
    /**
     * @brief Destructor
     */
    virtual ~AbstractNavigationServer();

    virtual void stop();

    //! shared pointer to a new @ref planner_execution "PlannerExecution"
    virtual mbf_abstract_nav::AbstractPlannerExecution::Ptr newPlannerExecution(
        const mbf_abstract_core::AbstractPlanner::Ptr plugin_ptr);

    //! shared pointer to a new @ref controller_execution "ControllerExecution"
    virtual mbf_abstract_nav::AbstractControllerExecution::Ptr newControllerExecution(
        const mbf_abstract_core::AbstractController::Ptr plugin_ptr);

    //! shared pointer to a new @ref recovery_execution "RecoveryExecution"
    virtual mbf_abstract_nav::AbstractRecoveryExecution::Ptr newRecoveryExecution(
        const mbf_abstract_core::AbstractRecovery::Ptr plugin_ptr);

    /**
     * @brief Loads the plugin associated with the given planner_type parameter.
     * @param planner_type The type of the planner plugin to load.
     * @return Pointer to the loaded plugin
     */
    virtual mbf_abstract_core::AbstractPlanner::Ptr loadPlannerPlugin(const std::string& planner_type) = 0;

    /**
     * @brief Loads the plugin associated with the given controller type parameter
     * @param controller_type The type of the controller plugin
     * @return A shared pointer to a new loaded controller, if the controller plugin was loaded successfully,
     *         an empty pointer otherwise.
     */
    virtual mbf_abstract_core::AbstractController::Ptr loadControllerPlugin(const std::string& controller_type) = 0;

    /**
     * @brief Loads a Recovery plugin associated with given recovery type parameter
     * @param recovery_name The name of the Recovery plugin
     * @return A shared pointer to a Recovery plugin, if the plugin was loaded successfully, an empty pointer otherwise.
     */
    virtual mbf_abstract_core::AbstractRecovery::Ptr loadRecoveryPlugin(const std::string& recovery_type) = 0;

    /**
     * @brief Pure virtual method, the derived class has to implement. Depending on the plugin base class,
     *        some plugins need to be initialized!
     * @param name The name of the planner
     * @param planner_ptr pointer to the planner object which corresponds to the name param
     * @return true if init succeeded, false otherwise
     */
    virtual bool initializePlannerPlugin(
        const std::string& name,
        const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr
    ) = 0;

    /**
     * @brief Pure virtual method, the derived class has to implement. Depending on the plugin base class,
     *        some plugins need to be initialized!
     * @param name The name of the controller
     * @param controller_ptr pointer to the controller object which corresponds to the name param
     * @return true if init succeeded, false otherwise
     */
    virtual bool initializeControllerPlugin(
        const std::string& name,
        const mbf_abstract_core::AbstractController::Ptr& controller_ptr
    ) = 0;

    /**
     * @brief Pure virtual method, the derived class has to implement. Depending on the plugin base class,
     *        some plugins need to be initialized!
     * @param name The name of the recovery behavior
     * @param behavior_ptr pointer to the recovery behavior object which corresponds to the name param
     * @return true if init succeeded, false otherwise
     */
    virtual bool initializeRecoveryPlugin(
        const std::string& name,
        const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr
    ) = 0;


    /**
     * @brief GetPath action execution method. This method will be called if the action server receives a goal
     * @param goal SimpleActionServer goal containing all necessary parameters for the action execution. See the action
     *        definitions in mbf_msgs.
     */
    virtual void callActionGetPath(ActionServerGetPath::GoalHandle &goal_handle);

    virtual void cancelActionGetPath(ActionServerGetPath::GoalHandle &goal_handle);

    /**
     * @brief ExePath action execution method. This method will be called if the action server receives a goal
     * @param goal SimpleActionServer goal containing all necessary parameters for the action execution. See the action
     *        definitions in mbf_msgs.
     */
    virtual void callActionExePath(ActionServerExePath::GoalHandle &goal_handle);

    virtual void cancelActionExePath(ActionServerExePath::GoalHandle &goal_handle);

    /**
     * @brief Recovery action execution method. This method will be called if the action server receives a goal
     * @param goal SimpleActionServer goal containing all necessary parameters for the action execution. See the action
     *        definitions in mbf_msgs.
     */
    virtual void callActionRecovery(ActionServerRecovery::GoalHandle &goal_handle);

    virtual void cancelActionRecovery(ActionServerRecovery::GoalHandle &goal_handle);

    /**
     * @brief MoveBase action execution method. This method will be called if the action server receives a goal
     * @param goal SimpleActionServer goal containing all necessary parameters for the action execution. See the action
     *        definitions in mbf_msgs.
     */
    virtual void callActionMoveBase(ActionServerMoveBase::GoalHandle &goal_handle);

    virtual void cancelActionMoveBase(ActionServerMoveBase::GoalHandle &goal_handle);

    /**
     * @brief starts all action server.
     */
    virtual void startActionServers();

    /**
     * @brief initializes all server components. Initializing the plugins of the @ref planner_execution "Planner", the
     *        @ref controller_execution "Controller", and the @ref recovery_execution "Recovery Behavior".
     */
    virtual void initializeServerComponents();

    /**
     * @brief Computes the current robot pose (robot_frame_) in the global frame (global_frame_).
     * @param robot_pose Reference to the robot_pose message object to be filled.
     * @return true, if the current robot pose could be computed, false otherwise.
     */
    bool getRobotPose(geometry_msgs::PoseStamped &robot_pose);

  protected:

    /**
     * @brief Transforms a plan to the global frame (global_frame_) coord system.
     * @param plan Input plan to be transformed.
     * @param global_plan Output plan, which is then transformed to the global frame.
     * @return true, if the transformation succeeded, false otherwise
     */
    bool transformPlanToGlobalFrame(std::vector<geometry_msgs::PoseStamped> &plan,
                                    std::vector<geometry_msgs::PoseStamped> &global_plan);

    /**
     * @brief Utility method to fill the ExePath action result in a single line
     * @param outcome ExePath action outcome
     * @param message ExePath action message
     * @param result The action result to fill
     */
    void fillExePathResult(uint32_t outcome, const std::string &message, mbf_msgs::ExePathResult &result);

    /**
     * @brief Start a dynamic reconfigure server.
     * This must be called only if the extending doesn't create its own.
     */
    virtual void startDynamicReconfigureServer();

    /**
     * @brief Reconfiguration method called by dynamic reconfigure
     * @param config Configuration parameters. See the MoveBaseFlexConfig definition.
     * @param level bit mask, which parameters are set.
     */
    virtual void reconfigure(mbf_abstract_nav::MoveBaseFlexConfig &config, uint32_t level);

    /**
     * @brief Continuous replanning thread. Keeps calling get_path action server at planner_frequency Hz to
     * get updated plans
     * @param cond Condition variable used to retain/release planning
     * @param lock Lock used in conjuntion with the condition variable
     * @param goal GetPath action goal; remains the same during navigation
     * @param result GetPath result, containing the updated plans
     * @param has_new_plan Flag signaling when a new plan is available
     */
    void plannerThread(boost::condition_variable &cond, boost::unique_lock<boost::mutex> &lock,
                       const mbf_msgs::GetPathGoal &goal, mbf_msgs::GetPathResult &result, bool &has_new_plan);

    //! Private node handle
    ros::NodeHandle private_nh_;

    AbstractPluginManager<mbf_abstract_core::AbstractPlanner>planner_plugin_manager_;
    AbstractPluginManager<mbf_abstract_core::AbstractController>controller_plugin_manager_;
    AbstractPluginManager<mbf_abstract_core::AbstractRecovery>recovery_plugin_manager_;

    //! shared pointer to the Recovery action server
    ActionServerRecoveryPtr action_server_recovery_ptr_;

    //! shared pointer to the ExePath action server
    ActionServerExePathPtr action_server_exe_path_ptr_;

    //! shared pointer to the GetPath action server
    ActionServerGetPathPtr action_server_get_path_ptr_;

    //! shared pointer to the MoveBase action server
    ActionServerMoveBasePtr action_server_move_base_ptr_;

    //! dynamic reconfigure server
    DynamicReconfigureServer dsrv_;

    //! configuration mutex for derived classes and other threads.
    boost::recursive_mutex configuration_mutex_;

    //! last configuration save
    mbf_abstract_nav::MoveBaseFlexConfig last_config_;

    //! the default parameter configuration save
    mbf_abstract_nav::MoveBaseFlexConfig default_config_;

    //! true, if the dynamic reconfigure has been setup.
    bool setup_reconfigure_;

    //! the robot frame, to get the current robot pose in the global_frame_
    std::string robot_frame_;

    //! the global frame, in which the robot is moving
    std::string global_frame_;

    //! timeout after tf returns without a result
    ros::Duration tf_timeout_;

    //! shared pointer to the common TransformListener
    const boost::shared_ptr<tf::TransformListener> tf_listener_ptr_;

    //! current robot pose; moving controller is responsible to update it by calling getRobotPose
    geometry_msgs::PoseStamped robot_pose_;

    //! current goal pose; used to compute remaining distance and angle
    geometry_msgs::PoseStamped goal_pose_;

    //! timeout after a oscillation is detected
    ros::Duration oscillation_timeout_;

    //! minimal move distance to not detect an oscillation
    double oscillation_distance_;

    //! true, if recovery behavior for the MoveBase action is enabled.
    bool recovery_enabled_;

    //! true, if clearing rotate is allowed.
    bool clearing_rotation_allowed_;


    RobotInformation robot_info_;

    ControllerAction controller_action_;
    PlannerAction planner_action_;
    RecoveryAction recovery_action_;
    MoveBaseAction move_base_action_;
  };

} /* namespace mbf_abstract_nav */

#endif /* navigation_controller.h */
