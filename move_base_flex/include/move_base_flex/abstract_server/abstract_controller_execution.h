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

#ifndef MOVE_BASE_FLEX__ABSTRACT_CONTROLLER_EXECUTION_H_
#define MOVE_BASE_FLEX__ABSTRACT_CONTROLLER_EXECUTION_H_

#include <pluginlib/class_loader.h>
#include <boost/chrono/thread_clock.hpp>
#include <boost/chrono/duration.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/abstract_local_planner.h>

#include "move_base_flex/navigation_utility.h"
#include "move_base_flex/MoveBaseFlexConfig.h"

namespace move_base_flex
{
/**
 * @defgroup controller_execution Controller Execution Classes
 * @brief The controller execution classes are derived from the AbstractControllerExecution and extends the
 *        functionality. The base controller execution code is located in the AbstractControllerExecution.
 */

/**
 * @brief The AbstractControllerExecution class loads and binds the local planner plugin. It contains a thread running
 *        the plugin in a cycle to move the robot. An internal state is saved and will be pulled by server, which
 *        controls the local planner execution. Due to a state change it wakes up all threads connected to the ondition
 *        variable.
 *
 * @tparam LOCAL_PLANNER_BASE The base class derived from the AbstractLocalPlanner class. The local planner plugin
 *         needs to implement that interface base class to be compatible with move_base_flex
 *
 * @ingroup abstract_server controller_execution
 */
template<typename LOCAL_PLANNER_BASE>
  class AbstractControllerExecution
  {
  public:

    typedef boost::shared_ptr<AbstractControllerExecution<LOCAL_PLANNER_BASE> > Ptr;

    /**
     * @brief Constructor
     * @param condition Thread sleep condition variable, to wake up connected threads
     * @param tf_listener_ptr Shared pointer to a common tf listener
     * @param package Package name, which contains the base class interface of the plugin
     * @param class_name Class name of the base class interface of the plugin
     */
    AbstractControllerExecution(boost::condition_variable &condition,
                                const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
                                std::string package, std::string class_name);

    /**
     * @brief Destructor
     */
    virtual ~AbstractControllerExecution();

    /**
     * @brief Starts the controller, a valid plan should be given in advance.
     * @return false if the thread is already running, true if starting the controller succeeded!
     */
    bool startMoving();

    /**
     * @brief Stopping the thread, by interrupting it
     */
    void stopMoving();

    /**
     * @brief Sets a new plan to the controller execution
     * @param plan A vector of stamped poses.
     */
    void setNewPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

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
      NO_LOCAL_CMD, ///< Received no velocity command by the plugin, in the current cycle.
      GOT_LOCAL_CMD,///< Got a valid velocity command from the plugin.
      ARRIVED_GOAL, ///< The robot arrived the goal.
      STOPPED,      ///< The controller has been stopped!
    };

    /**
     * Return the current state of the controller execution. Thread communication safe.
     * @return current state, enum value of ControllerState
     */
    ControllerState getState();

    /**
     * @brief pulls the current plugin information, plugin code and plugin message!
     * @param plugin_code Returns the last read code provided py the plugin
     * @param plugin_msg Returns the last read message provided by the plugin
     */
    void getPluginInfo(uint8_t& plugin_code, std::string& plugin_msg);

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
     * @brief Checks whether the patience duration time has been exceeded, ot not
     * @return true, if the patience has been exceeded.
     */
    bool isPatienceExceeded();

    /**
     * @brief Reads the current velocity command
     * @param vel_cmd_stamped Returns the current velocity command.
     */
    void getVelocityCmd(geometry_msgs::TwistStamped &vel_cmd_stamped);

    /**
     * @brief Loads the plugin given by the parameter "local_planner"
     */
    void initialize();

    /**
     * @brief Is called by the server thread to reconfigure the controller execution, if a user uses dynamic reconfigure
     *        to reconfigure the current state.
     * @param config MoveBaseFlexConfig object
     */
    void reconfigure(move_base_flex::MoveBaseFlexConfig &config);

  protected:

    /**
     * @brief Sets the velocity command, to make it available for another thread
     * @param vel_cmd_stamped current velocity command
     */
    void setVelocityCmd(const geometry_msgs::TwistStamped &vel_cmd_stamped);

    /**
     * @brief Sets the plugin code and the plugin msg. This method is for thread communication
     * @param plugin_code
     * @param plugin_msg
     */
    void setPluginInfo(const uint8_t& plugin_code, const std::string& plugin_msg);

    //! the name of the loaded plugin
    std::string plugin_name_;

    //! class loader, to load the local planner plugin
    pluginlib::ClassLoader<LOCAL_PLANNER_BASE> class_loader_local_planner_;

    //! the local planer to calculate the velocity command
    boost::shared_ptr<LOCAL_PLANNER_BASE> local_planner_;

    //! shared pointer to the shared tf listener
    const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr;

    //! The current cycle start time of the last cycle run. Will by updated each cycle.
    ros::Time last_call_time_;

    //! The maximum number of retries
    int max_retries_;

    //! The time / duration of patience, before changing the state.
    ros::Duration patience_;

  private:

    /**
     * @brief The main run method, a thread will execute this method. It contains the main controller execution loop.
     */
    virtual void run();

    /**
     * @brief Pure virtual method, the derived class has to implement. Depending on the plugin base class,
     *        some plugins need to be initialized!
     */
    virtual void initLocalPlannerPlugin() = 0;

    /**
     * publishes a velocity command with zero values to stop the robot.
     */
    void publishZeroVelocity();

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

    //! mutex to handle safe thread communication for the current plugin code
    boost::mutex pcode_mtx_;

    //! true, if a new plan is available. See hasNewPlan()!
    bool new_plan_;

    /**
     * @brief Returns true if a new plan is available, false otherwise! A new plan is set by another thread!
     * @return true, if a new plan has been set, false otherwise.
     */
    bool hasNewPlan();

    /**
     * @brief Gets the new available plan. This method is thread safe.
     * @param plan A reference to a plan which will then be filled with the new plan
     */
    void getNewPlan(std::vector<geometry_msgs::PoseStamped> &plan);

    //! the last calculated velocity command
    geometry_msgs::TwistStamped vel_cmd_stamped_;

    //! the last set plan which is currently processed by the controller
    std::vector<geometry_msgs::PoseStamped> plan_;

    //! condition variable to wake up control thread
    boost::condition_variable &condition_;

    //! the controlling thread object
    boost::thread thread_;

    //! the duration which corresponds with the controller frequency.
    boost::chrono::microseconds calling_duration_;

    //! the frame of the robot, which will be used to determine its position.
    std::string robot_frame_;

    //! the global frame the robot is controlling in.
    std::string global_frame_;

    //! publisher for the current velocity command
    ros::Publisher vel_pub_;

    //! the current controller state
    AbstractControllerExecution::ControllerState state_;

    //! the last received plugin code
    uint8_t plugin_code_;

    //! the last received plugin message
    std::string plugin_msg_;

    //! time before a timeout used for tf requests
    double tf_timeout_;

    //! dynamic reconfigure config mutex, thread safe param reading and writing
    boost::recursive_mutex configuration_mutex_;

    //! main controller loop variable, true if the controller is running, false otherwise
    bool moving_;

    //! distance tolerance to the given goal pose
    double dist_tolerance_;

    //! angle tolerance to the given goal pose
    double angle_tolerance_;
  };

} /* namespace move_base_flex */

#include "move_base_flex/abstract_server/impl/abstract_controller_execution.tcc"

#endif /* MOVE_BASE_FLEX__ABSTRACT_CONTROLLER_EXECUTION_H_ */
