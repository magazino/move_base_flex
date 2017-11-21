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
 *  abstract_planner_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MOVE_BASE_FLEX__ABSTRACT_PLANNER_EXECUTION_H_
#define MOVE_BASE_FLEX__ABSTRACT_PLANNER_EXECUTION_H_

#include <pluginlib/class_loader.h>
#include <boost/chrono/thread_clock.hpp>
#include <boost/chrono/duration.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_flex_core/abstract_planner.h>

#include "move_base_flex/navigation_utility.h"
#include "move_base_flex/MoveBaseFlexConfig.h"

namespace move_base_flex
{

/**
 * @defgroup planner_execution Planner Execution Classes
 * @brief The planner execution classes are derived from the AbstractPlannerExecution and extends the functionality.
 *        The base planner execution code is located in the AbstractPlannerExecution.
 */

/**
 * @brief The AbstractPlannerExecution class loads and binds the global planner plugin. It contains a thread running
 *        the plugin in a cycle to plan and re-plan. An internal state is saved and will be pulled by the server, which
 *        controls the global planner execution. Due to a state change it wakes up all threads connected to the
 *        condition variable.
 *
 * @tparam GLOBAL_PLANNER_BASE The base class derived from the AbstractGlobalPlanner class. The local planner plugin
 *         has to implement that interface base class to be compatible with move_base_flex
 *
 * @ingroup abstract_server planner_execution
 */
template<typename GLOBAL_PLANNER_BASE>
  class AbstractPlannerExecution
  {
  public:

    //! shared pointer type to the @ref planner_execution "planner execution".
    typedef boost::shared_ptr<AbstractPlannerExecution<GLOBAL_PLANNER_BASE> > Ptr;

    /**
     * @brief Constructor
     * @param condition Thread sleep condition variable, to wake up connected threads
     * @param package Package name, which contains the base class interface of the plugin
     * @param class_name Class name of the base class interface of the plugin
     */
    AbstractPlannerExecution(boost::condition_variable &condition, std::string package, std::string class_name);

    /**
     * @brief Destructor
     */
    virtual ~AbstractPlannerExecution();

    /**
     * @brief Returns a new plan, if one is available.
     * @param plan A reference to a plan, which then will be filled.
     * @param cost A reference to the costs, which then will be filled.
     */
    void getNewPlan(std::vector<geometry_msgs::PoseStamped> &plan, double &cost);

    /**
     * @brief Returns the last time a valid plan was available.
     * @return time, the last valid plan was available.
     */
    ros::Time getLastValidPlanTime();

    /**
     * @brief Returns the time of the last cycle start.
     * @return time, the last cycle started.
     */
    ros::Time getLastCycleStartTime();

    /**
     * @brief Checks whether the patience was exceeded.
     * @return true, if the patience duration was exceeded.
     */
    bool isPatienceExceeded();

    /**
     * @brief Internal states
     */
    enum PlanningState
    {
      INITIALIZED,  ///< Planner initialized.
      STARTED,      ///< Planner started.
      PLANNING,     ///< Executing the plugin.
      FOUND_PLAN,   ///< Found a valid plan.
      MAX_RETRIES,  ///< Exceeded the maximum number of retries without a valid command.
      PAT_EXCEEDED, ///< Exceeded the patience time without a valid command.
      NO_PLAN_FOUND,///< No plan has been found (MAX_RETRIES and PAT_EXCEEDED are 0).
      CANCELED,     ///< The planner has been canceled.
      STOPPED       ///< The planner has been stopped.
    };

    /**
     * @brief Returns the current internal state
     * @return the current internal state
     */
    PlanningState getState();

    /**
     * @brief Cancel the planner execution. This calls the cancel method of the planner plugin. This could be useful if the
     * computation takes to much time.
     * @return true, if the planner plugin tries / tried to cancel the planning step.
     */
    bool cancel();

    /**
     * @brief Sets a new goal pose for the planner execution
     * @param goal the new goal pose
     * @param tolerance tolerance to the goal for the planning
     */
    void setNewGoal(const geometry_msgs::PoseStamped &goal, double tolerance);

    /**
     * @brief Sets a new start pose for the planner execution
     * @param start new start pose
     */
    void setNewStart(const geometry_msgs::PoseStamped &start);

    /**
     * @brief Sets a new star and goal pose for the planner execution
     * @param start new start pose
     * @param goal new goal pose
     * @param tolerance tolerance to the new goal for the planning
     */
    void setNewStartAndGoal(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                            double tolerance);

    /**
     * @brief Starts the planner execution thread with the given parameters.
     * @param start start pose for the planning
     * @param goal goal pose for the planning
     * @param tolerance tolerance to the goal pose for the planning
     * @return true, if the planner thread has been started, false if the thread is already running.
     */
    bool startPlanning(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                       double tolerance);

    /**
     * @brief Copys the plugin info to the references.
     * @param plugin_code Reference to a variable, to which the code will be copied.
     * @param plugin_msg Reference to a variable, to which the plugin message will be copied.
     */
    void getPluginInfo(uint32_t &plugin_code, std::string &plugin_msg);

    /**
     * @brief Tries to stop the current planner execution by an thread interrupt.
     */
    void stopPlanning();

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

    //! class loader, to load the global planner plugin
    pluginlib::ClassLoader<GLOBAL_PLANNER_BASE> class_loader_global_planner_;

    //! the local planer to calculate the velocity command
    boost::shared_ptr<GLOBAL_PLANNER_BASE> global_planner_;

    //! the name of the loaded planner plugin
    std::string plugin_name_;

    //! true, if the planner execution has been canceled.
    bool cancel_;

    /**
     * @brief The main run method, a thread will execute this method. It contains the main planner execution loop.
     */
    virtual void run();

    /**
     * @brief load a parameters from the ros parameter server
     */
    void loadParams();

   /**
    * @brief Saves the current plugin code and plugin message. This method is for a safe thread communication.
    * @param plugin_code plugin code received from the plugin
    * @param plugin_msg plugin message received from the plugin
    */
    void setPluginInfo(const uint32_t &plugin_code, const std::string &plugin_msg);


  private:

    /**
     * @brief Saves the last cycle state. This method is for a safe thread communication.
     */
    void setLastCycleStartTime();

    /**
     * @brief Loads the plugin defined in the parameter server
     * @return true, if the local planner plugin was successfully loaded.
     */
    virtual bool loadPlugin();

    /**
     * @brief Pure virtual method, the derived class has to implement. Depending on the plugin base class,
     *        some plugins need to be initialized!
     */
    virtual void initPlugin() = 0;

    /**
     * @brief Sets the internal state, thread communication safe
     * @param state the current state
     */
    void setState(PlanningState state);

    /**
     * @brief Saves the plan, after a plan has been found. Thread communication safe.
     * @param plan The computed plan to be transferred.
     * @param cost The computed costs to be transfered.
     */
    void setNewPlan(const std::vector<geometry_msgs::PoseStamped> &plan, double cost);

    //! mutex to handle safe thread communication for the current state
    boost::mutex state_mtx_;

    //! mutex to handle safe thread communication for the plan and plan-costs
    boost::mutex plan_mtx_;

    //! mutex to handle safe thread communication for the plugin code and the plugin message.
    boost::mutex pcode_mtx_;

    //! mutex to handle safe thread communication for the goal and start pose.
    boost::mutex goal_start_mtx_;

    //! mutex to handle safe thread communication for the last cycle start time.
    boost::mutex lct_mtx_;

    //! true, if a new goal pose has been set, until it is used.
    bool has_new_goal_;

    //! true, if a new start pose has been set, until it is used.
    bool has_new_start_;

    //! the last cycle start time, updated each cycle.
    ros::Time last_cycle_start_time_;

    //! the last time a valid plan has been computed.
    ros::Time last_valid_plan_time_;

    //! current global plan
    std::vector<geometry_msgs::PoseStamped> plan_;

    //! current global plan cost
    double cost_;

    //! the current plugin code
    uint32_t plugin_code_;

    //! the current plugin message
    std::string plugin_msg_;

    //! the current start pose used for planning
    geometry_msgs::PoseStamped start_;

    //! the current goal pose used for planning
    geometry_msgs::PoseStamped goal_;

    //! optional goal tolerance, in meters
    double tolerance_;

    //! planning cycle frequency
    double frequency_;

    //! planning patience duration time
    ros::Duration patience_;

    //! planning max retries
    int max_retries_;

    //! main cycle variable of the execution loop
    bool planning_;

    //! condition variable to wake up server thread
    boost::condition_variable &condition_;

    //! thread for planning
    boost::thread thread_;

    //! timing of the planning thread
    boost::chrono::microseconds calling_duration_;

    //! robot frame used for computing the current robot pose
    std::string robot_frame_;

    //! the global frame in which the planner needs to plan
    std::string global_frame_;

    //! shared pointer to a common TransformListener
    const boost::shared_ptr<tf::TransformListener> tf_listener_ptr_;

    //! current internal state
    PlanningState state_;

    //! dynamic reconfigure mutex for a thread safe communication
    boost::recursive_mutex configuration_mutex_;
  };

} /* namespace move_base_flex */

#include "move_base_flex/abstract_server/impl/abstract_planner_execution.tcc"

#endif /* MOVE_BASE_FLEX__ABSTRACT_PLANNER_EXECUTION_H_ */
