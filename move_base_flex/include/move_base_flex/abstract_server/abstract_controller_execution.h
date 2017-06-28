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
 * @brief The AbstractPlannerExecution class loads and binds the local planner plugin. It contains a thread running
 *   the plugin in a cycle to move the robot. An internal state is saved and will be pulled by server, which controls
 *   the local planner execution. Due to a state change it wakes up all threads connected to the ondition variable.
 * @tparam LOCAL_PLANNER_BASE The base class derived from the AbstractLocalPlanner class,
 *   which determines the plugin class
 */

template<typename LOCAL_PLANNER_BASE>
  class AbstractControllerExecution
  {
  public:

    typedef boost::shared_ptr<AbstractControllerExecution<LOCAL_PLANNER_BASE> > Ptr;

    /**
     * @brief Constructor
     * @param condition Thread sleep condition variable, to wake up connected threads
     * @param tf_listener_ptr A common tf listener
     * @param package Package name, which contains the plugin
     * @param class_name Class name of the plugin
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
      INITIALIZED,
      STARTED,
      PLANNING,
      NO_PLAN,
      MAX_RETRIES,
      PAT_EXCEEDED,
      EMPTY_PLAN,
      NO_LOCAL_CMD,
      GOT_LOCAL_CMD,
      ARRIVED_GOAL,
      STOPPED,
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
     * @brief Returns the time of the last cycle start
     * @return Time of the last cycle start
     */
    ros::Time getLastCycleStartTime();

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
     * Loads the plugin given by the parameter name "local_planner"
     */
    void initialize();

    /**
     * Is called by the server thread to reconfigure the controller execution.
     * @param config MoveBaseFlexConfig object
     */
    void reconfigure(move_base_flex::MoveBaseFlexConfig &config);

  protected:

    std::string plugin_name_;

    // class loader, to load the local planner plugin
    pluginlib::ClassLoader<LOCAL_PLANNER_BASE> class_loader_local_planner_;

    // the local planer to calculate the velocity command
    boost::shared_ptr<LOCAL_PLANNER_BASE> local_planner_;

    const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr;

    void setVelocityCmd(const geometry_msgs::TwistStamped &vel_cmd_stamped);

    void setPluginInfo(const uint8_t& plugin_code, const std::string& plugin_msg);

    // main thread function -> moving the robot
    virtual void run();

    ros::Time cycle_start_time;

    int max_retries_;

    ros::Duration patience_;

  private:

    virtual void initMovingPlugin() = 0;

    // stops the robot
    void publishZeroVelocity();

    boost::mutex state_mtx_;

    void setState(ControllerState state);

    boost::mutex plan_mtx_;
    boost::mutex vel_cmd_mtx_;
    boost::mutex lct_mtx_;
    boost::mutex pcode_mtx_;

    bool new_plan_;

    bool hasNewPlan();

    void getNewPlan(std::vector<geometry_msgs::PoseStamped> &plan);

    geometry_msgs::TwistStamped vel_cmd_stamped_;

    // current global path
    std::vector<geometry_msgs::PoseStamped> plan_;

    // condition to wake up control thread
    boost::condition_variable &condition_;

    // thread for moving
    boost::thread thread_;

    // timing of the moving thread
    boost::chrono::microseconds calling_duration_;

    // frames to get the current global robot pose
    std::string robot_frame_;
    std::string global_frame_;

    // subscribers and publishers
    ros::Publisher vel_pub_;

    // internal state
    AbstractControllerExecution::ControllerState state_;

    // plugin feedback info
    uint8_t plugin_code_;
    std::string plugin_msg_;

    double tf_timeout_;

    // dynamic reconfigure attributes and methods
    boost::recursive_mutex configuration_mutex_;

    // is the moving thread running
    bool moving_;
  };

} /* namespace move_base_flex */

#include "move_base_flex/abstract_server/impl/abstract_controller_execution.tcc"

#endif /* MOVE_BASE_FLEX__ABSTRACT_CONTROLLER_EXECUTION_H_ */
