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

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_PLANNER_EXECUTION_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_PLANNER_EXECUTION_H_

#include <map>
#include <stdint.h>
#include <string>
#include <vector>

#include <boost/chrono/duration.hpp>
#include <boost/chrono/thread_clock.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <mbf_abstract_core/abstract_planner.h>
#include <mbf_utility/navigation_utility.h>

#include "mbf_abstract_nav/MoveBaseFlexConfig.h"

namespace mbf_abstract_nav
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
 * @ingroup abstract_server planner_execution
 */
  class AbstractPlannerExecution
  {
  public:

    //! shared pointer type to the @ref planner_execution "planner execution".
    typedef boost::shared_ptr<AbstractPlannerExecution > Ptr;

    /**
     * @brief Constructor
     * @param condition Thread sleep condition variable, to wake up connected threads
     */
    AbstractPlannerExecution(boost::condition_variable &condition);

    /**
     * @brief Destructor
     */
    virtual ~AbstractPlannerExecution();

    /**
     * @brief Returns a new plan, if one is available.
     * @param plan A reference to a plan, which then will be filled.
     * @param cost A reference to the costs, which then will be filled.
     */
    std::vector<geometry_msgs::PoseStamped> getPlan();

    /**
     * @brief Returns the last time a valid plan was available.
     * @return time, the last valid plan was available.
     */
    ros::Time getLastValidPlanTime();

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
      STOPPED,      ///< The planner has been stopped.
      INTERNAL_ERROR///< An internal error occurred.
    };

    /**
     * @brief Returns the current internal state
     * @return the current internal state
     */
    PlanningState getState();

    /**
     * @brief Gets the current plugin execution outcome
     */
    uint32_t getOutcome() { return outcome_; };

    /**
     * @brief Gets the current plugin execution message
     */
    std::string getMessage() { return message_; };

    /**
     * @brief Gets planning frequency
     */
    double getFrequency() { return frequency_; };

    /**
     * @brief Gets computed costs
     * @return The costs of the computed path
     */
    double getCost();

    /**
     * @brief Cancel the planner execution. This calls the cancel method of the planner plugin. This could be useful if the
     * computation takes too much time.
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
     * @brief Tries to stop the current planner execution by an thread interrupt.
     */
    void stopPlanning();

    /**
     * @brief Loads the plugin given by the parameter "local_planner"
     * @return true, if successful
     */
    bool initialize();

    /**
     * @brief Is called by the server thread to reconfigure the controller execution, if a user uses dynamic reconfigure
     *        to reconfigure the current state.
     * @param config MoveBaseFlexConfig object
     */
    void reconfigure(const MoveBaseFlexConfig &config);

    /**
     * @brief Switches the planner to planner with the given name
     * @param name The name of the planner in the planners list
     * @return true if the switch was successful, false otherwise.
     */
    bool switchPlanner(const std::string& name);

  protected:

    //! the local planer to calculate the velocity command
    mbf_abstract_core::AbstractPlanner::Ptr planner_;

    //! map to store the planners. Each planner can be accessed by its corresponding name
    std::map<std::string, mbf_abstract_core::AbstractPlanner::Ptr > planners_;

    //! map to store the type of the planner as string
    std::map<std::string, std::string> planners_type_;

    //! the name of the loaded planner plugin
    std::string plugin_name_;

    //! true, if the planner execution has been canceled.
    bool cancel_;

    /**
     * @brief The main run method, a thread will execute this method. It contains the main planner execution loop.
     */
    virtual void run();


  private:

    /**
     * @brief Loads the plugin associated with the given planner_type parameter.
     * @param planner_type The type of the planner plugin to load.
     * @return Pointer to the loaded plugin
     */
    virtual mbf_abstract_core::AbstractPlanner::Ptr loadPlannerPlugin(const std::string& planner_type) = 0;

    /**
     * @brief Pure virtual method, the derived class has to implement. Depending on the plugin base class,
     *        some plugins need to be initialized!
     * @param name The name of the planner
     * @param planner_ptr pointer to the planner object which corresponds to the name param
     * @return true if init succeeded, false otherwise
     */
    virtual bool initPlugin(
        const std::string& name,
        const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr
    ) = 0;

    /**
     * @brief calls the planner plugin to make a plan from the start pose to the goal pose with the given tolerance,
     *        if a goal tolerance is enabled in the planner plugin.
     * @param start The start pose for planning
     * @param goal The goal pose for planning
     * @param tolerance The goal tolerance
     * @param plan The computed plan by the plugin
     * @param cost The computed costs for the corresponding plan
     * @param message An optional message which should correspond with the returned outcome
     * @return An outcome number, see also the action definition in the GetPath.action file
     */
    virtual uint32_t makePlan(
        const mbf_abstract_core::AbstractPlanner::Ptr &planner_ptr,
        const geometry_msgs::PoseStamped &start,
        const geometry_msgs::PoseStamped &goal,
        double tolerance,
        std::vector<geometry_msgs::PoseStamped> &plan,
        double &cost,
        std::string &message);

    /**
     * @brief Loads the plugins defined in the parameter server
     * @return true, if all planners have been loaded successfully.
     */
    bool loadPlugins();

    /**
     * @brief Sets the internal state, thread communication safe
     * @param state the current state
     */
    void setState(PlanningState state);

    //! mutex to handle safe thread communication for the current state
    boost::mutex state_mtx_;

    //! mutex to handle safe thread communication for the plan and plan-costs
    boost::mutex plan_mtx_;

    //! mutex to handle safe thread communication for the goal and start pose.
    boost::mutex goal_start_mtx_;

    //! mutex to handle safe thread communication for the planning_ flag.
    boost::mutex planning_mtx_;

    //! true, if a new goal pose has been set, until it is used.
    bool has_new_goal_;

    //! true, if a new start pose has been set, until it is used.
    bool has_new_start_;

    //! the last call start time, updated each cycle.
    ros::Time last_call_start_time_;

    //! the last time a valid plan has been computed.
    ros::Time last_valid_plan_time_;

    //! current global plan
    std::vector<geometry_msgs::PoseStamped> plan_;

    //! current global plan cost
    double cost_;

    //! the last received plugin execution outcome
    uint32_t outcome_;

    //! the last received plugin execution message
    std::string message_;

    //! the current start pose used for planning
    geometry_msgs::PoseStamped start_;

    //! the current goal pose used for planning
    geometry_msgs::PoseStamped goal_;

    //! optional goal tolerance, in meters
    double tolerance_;

    //! planning cycle frequency (used only when running full navigation; we store here for grouping parameters nicely)
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

} /* namespace mbf_abstract_nav */

#endif /* MBF_ABSTRACT_NAV__ABSTRACT_PLANNER_EXECUTION_H_ */
