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
 *  abstract_controller_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_CONTROLLER_EXECUTION_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_CONTROLLER_EXECUTION_H_

#include <map>
#include <stdint.h>
#include <string>
#include <vector>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <mbf_utility/navigation_utility.h>
#include <mbf_abstract_core/abstract_controller.h>

#include "mbf_abstract_nav/MoveBaseFlexConfig.h"
#include "mbf_abstract_nav/abstract_execution_base.h"

namespace mbf_abstract_nav
{
/**
 * @defgroup controller_execution Controller Execution Classes
 * @brief The controller execution classes are derived from the AbstractControllerExecution and extends the
 *        functionality. The base controller execution code is located in the AbstractControllerExecution.
 */

/**
 * @brief The AbstractControllerExecution class loads and binds the local planner plugin. It contains a thread
 *        running the plugin in a cycle to move the robot. An internal state is saved and will be pulled by server,
 *        which controls the local planner execution. Due to a state change it wakes up all threads connected to the
 *        condition variable.
 *
 * @ingroup abstract_server controller_execution
 */
  class AbstractControllerExecution : public AbstractExecutionBase
  {
  public:

    static const double DEFAULT_CONTROLLER_FREQUENCY;

    typedef boost::shared_ptr<AbstractControllerExecution > Ptr;

    /**
     * @brief Constructor
     * @param condition Thread sleep condition variable, to wake up connected threads
     * @param controller_plugin_type The plugin type associated with the plugin to load
     * @param tf_listener_ptr Shared pointer to a common tf listener
     */
    AbstractControllerExecution(
        const mbf_abstract_core::AbstractController::Ptr& controller_ptr,
        const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
        const MoveBaseFlexConfig &config,
        boost::function<void()> setup_fn,
        boost::function<void()> cleanup_fn);

    /**
     * @brief Destructor
     */
    virtual ~AbstractControllerExecution();

    /**
     * @brief Starts the controller, a valid plan should be given in advance.
     * @return false if the thread is already running, true if starting the controller succeeded!
     */
    virtual bool start();

    /**
     * @brief Sets a new plan to the controller execution
     * @param plan A vector of stamped poses.
     */
    void setNewPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

    /**
     * @brief Cancel the planner execution. This calls the cancel method of the planner plugin. This could be useful if the
     * computation takes too much time.
     * @return true, if the planner plugin tries / tried to cancel the planning step.
     */
    virtual bool cancel();

    /**
     * @brief Internal states
     */
    enum ControllerState
    {
      INITIALIZED,  ///< Controller has been initialized successfully.
      STARTED,      ///< Controller has been started.
      PLANNING,     ///< Executing the plugin.
      NO_PLAN,      ///< The controller has been started without a plan.
      MAX_RETRIES,  ///< Exceeded the maximum number of retries without a valid command.
      PAT_EXCEEDED, ///< Exceeded the patience time without a valid command.
      EMPTY_PLAN,   ///< Received an empty plan.
      INVALID_PLAN, ///< Received an invalid plan that the local planner rejected.
      NO_LOCAL_CMD, ///< Received no velocity command by the plugin, in the current cycle.
      GOT_LOCAL_CMD,///< Got a valid velocity command from the plugin.
      ARRIVED_GOAL, ///< The robot arrived the goal.
      CANCELED,     ///< The controller has been canceled.
      STOPPED,      ///< The controller has been stopped!
      INTERNAL_ERROR///< An internal error occurred.
    };

    /**
     * @brief Return the current state of the controller execution. Thread communication safe.
     * @return current state, enum value of ControllerState
     */
    ControllerState getState();

    /**
     * @brief Returns the time of the last plugin call
     * @return Time of the last plugin call
     */
    ros::Time getLastPluginCallTime();

    /**
     * @brief Returns the time, the last time a valid velocity command has been received
     * @return Time, the last time a valid cmd_vel has been received.
     */
    ros::Time getLastValidCmdVelTime();

    /**
     * @brief Returns the last valid velocity command set by setVelocityCmd method
     * @return The last valid velocity command.
     */
    geometry_msgs::TwistStamped getLastValidCmdVel();

    /**
     * @brief Checks whether the patience duration time has been exceeded, ot not
     * @return true, if the patience has been exceeded.
     */
    bool isPatienceExceeded();

    /**
     * @brief Sets the controller frequency
     * @param frequency The controller frequency
     * @return true, if the controller frequency has been changed / set succesfully, false otherwise
     */
    bool setControllerFrequency(double frequency);

    /**
     * @brief Is called by the server thread to reconfigure the controller execution,
     *        if a user uses dynamic reconfigure to reconfigure the current state.
     * @param config MoveBaseFlexConfig object
     */
    void reconfigure(const MoveBaseFlexConfig &config);

    /**
     * @brief Returns whether the robot should normally move or not. True if the local planner seems to work properly.
     * @return true, if the robot should normally move, false otherwise
     */
    bool isMoving();

  protected:

    /**
     * @brief Request plugin for a new velocity command, given the current position, orientation, and velocity of the
     * robot. We use this virtual method to give concrete implementations as move_base the chance to override it and do
     * additional stuff, for example locking the costmap.
     * @param pose the current pose of the robot.
     * @param velocity the current velocity of the robot.
     * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
     * @param message Optional more detailed outcome as a string.
     * @return Result code as described on ExePath action result and plugin's header.
     */
    virtual uint32_t computeVelocityCmd(const geometry_msgs::PoseStamped& pose,
                                        const geometry_msgs::TwistStamped& velocity,
                                        geometry_msgs::TwistStamped& vel_cmd, std::string& message);

    /**
     * @brief Sets the velocity command, to make it available for another thread
     * @param vel_cmd_stamped current velocity command
     */
    void setVelocityCmd(const geometry_msgs::TwistStamped &vel_cmd_stamped);

    //! the name of the loaded plugin
    std::string plugin_name_;

    //! the local planer to calculate the velocity command
    mbf_abstract_core::AbstractController::Ptr controller_;

    //! shared pointer to the shared tf listener
    const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr;

    //! The current cycle start time of the last cycle run. Will by updated each cycle.
    ros::Time last_call_time_;

    //! The time the controller has been started.
    ros::Time start_time_;

    //! The maximum number of retries
    int max_retries_;

    //! The time / duration of patience, before changing the state.
    ros::Duration patience_;

    /**
     * @brief The main run method, a thread will execute this method. It contains the main controller execution loop.
     */
    virtual void run();

  private:


    /**
     * publishes a velocity command with zero values to stop the robot.
     */
    void publishZeroVelocity();

    /**
     * @brief Checks whether the goal has been reached in the range of tolerance or not
     * @return true if the goal has been reached, false otherwise
     */
    bool reachedGoalCheck();

    /**
     * @brief Computes the robot pose;
     * @return true if the robot pose has been computed successfully, false otherwise.
     */
    bool computeRobotPose();

    /**
     * @brief Sets the controller state. This method makes the communication of the state thread safe.
     * @param state The current controller state.
     */
    void setState(ControllerState state);

    //! mutex to handle safe thread communication for the current value of the state
    boost::mutex state_mtx_;

    //! mutex to handle safe thread communication for the current plan
    boost::mutex plan_mtx_;

    //! mutex to handle safe thread communication for the current velocity command
    boost::mutex vel_cmd_mtx_;

    //! mutex to handle safe thread communication for the last plugin call time
    boost::mutex lct_mtx_;

    //! true, if a new plan is available. See hasNewPlan()!
    bool new_plan_;

    /**
     * @brief Returns true if a new plan is available, false otherwise! A new plan is set by another thread!
     * @return true, if a new plan has been set, false otherwise.
     */
    bool hasNewPlan();

    /**
     * @brief Gets the new available plan. This method is thread safe.
     * @return The plan
     */
    std::vector<geometry_msgs::PoseStamped> getNewPlan();

    //! the last calculated velocity command
    geometry_msgs::TwistStamped vel_cmd_stamped_;

    //! the last set plan which is currently processed by the controller
    std::vector<geometry_msgs::PoseStamped> plan_;

    //! the duration which corresponds with the controller frequency.
    boost::chrono::microseconds calling_duration_;

    //! the frame of the robot, which will be used to determine its position.
    std::string robot_frame_;

    //! the global frame the robot is controlling in.
    std::string global_frame_;

    //! publisher for the current velocity command
    ros::Publisher vel_pub_;

    //! publisher for the current goal
    ros::Publisher current_goal_pub_;

    //! the current controller state
    AbstractControllerExecution::ControllerState state_;

    //! time before a timeout used for tf requests
    double tf_timeout_;

    //! dynamic reconfigure config mutex, thread safe param reading and writing
    boost::mutex configuration_mutex_;

    //! main controller loop variable, true if the controller is running, false otherwise
    bool moving_;

    //! whether move base flex should check for the goal tolerance or not.
    bool mbf_tolerance_check_;

    //! distance tolerance to the given goal pose
    double dist_tolerance_;

    //! angle tolerance to the given goal pose
    double angle_tolerance_;

    //! current robot pose;
    geometry_msgs::PoseStamped robot_pose_;

  };

} /* namespace mbf_abstract_nav */

#endif /* MBF_ABSTRACT_NAV__ABSTRACT_CONTROLLER_EXECUTION_H_ */
