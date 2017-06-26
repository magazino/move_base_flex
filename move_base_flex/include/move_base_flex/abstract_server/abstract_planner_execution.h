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
#include <nav_core/abstract_global_planner.h>

#include "move_base_flex/navigation_utility.h"
#include "move_base_flex/MoveBaseFlexConfig.h"

namespace move_base_flex
{

template<typename GLOBAL_PLANNER_BASE>
  class AbstractPlannerExecution
  {
  public:

    typedef boost::shared_ptr<AbstractPlannerExecution<GLOBAL_PLANNER_BASE> > Ptr;

    AbstractPlannerExecution(boost::condition_variable &condition, std::string package, std::string class_name);

    virtual ~AbstractPlannerExecution();

    void getNewPlan(std::vector<geometry_msgs::PoseStamped> &plan, double &cost);

    ros::Time getLastValidPlanTime();

    ros::Time getLastCycleStartTime();

    bool isPatienceExceeded();

    enum PlanningState
    {
      INITIALIZED,
      STARTED,
      PLANNING,
      FOUND_PLAN,
      MAX_RETRIES,
      PAT_EXCEEDED,
      NO_PLAN_FOUND,
      CANCELED,
      STOPPED
    };

    PlanningState getState();

    bool cancel();

    void setNewGoal(const geometry_msgs::PoseStamped &goal, const double &tolerance);

    void setNewStart(const geometry_msgs::PoseStamped &start);

    void setNewStartAndGoal(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                            const double &tolerance);

    bool startPlanning(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                       const double &tolerance);

    void stopPlanning();

    void initialize();

    void reconfigure(move_base_flex::MoveBaseFlexConfig &config);

  protected:

    pluginlib::ClassLoader<GLOBAL_PLANNER_BASE> class_loader_global_planner_;

    // new abstract global planer to plan a global path
    boost::shared_ptr<GLOBAL_PLANNER_BASE> global_planner_;

    std::string planner_name_;

    bool cancel_;

    virtual void run();

    void loadParams();

  private:

    void setLastCycleStartTime();

    // virtual abstract method -> load the specific plugin class
    virtual void initPlannerPlugin() = 0;

    boost::mutex state_mtx_;
    boost::mutex thread_mtx_; // do not allow multi call of the run method
    void setState(PlanningState state);

    boost::mutex plan_mtx_;

    void setNewPlan(const std::vector<geometry_msgs::PoseStamped> &plan, const double &cost);

    bool has_new_goal_;
    bool has_new_start_;

    boost::mutex goal_start_mtx_;
    boost::mutex lct_mtx_;
    ros::Time last_cycle_start_time_;
    ros::Time last_valid_plan_time_;

    // current global plan
    std::vector<geometry_msgs::PoseStamped> plan_;

    // current global plan cost
    double cost_;

    geometry_msgs::PoseStamped start_, goal_;
    double tolerance_;

    // planning frequency
    double frequency_;

    // planning patience
    ros::Duration patience_;

    // planning max retries
    int max_retries_;

    bool planning_;

    // condition to wake up control thread
    boost::condition_variable &condition_;

    // thread for planning
    boost::thread thread_;

    // timing of the planning thread
    boost::chrono::microseconds calling_duration_;

    // frames to get the current global robot pose
    std::string robot_frame_;
    std::string global_frame_;

    const boost::shared_ptr<tf::TransformListener> tf_listener_ptr_;

    // current internal state
    PlanningState state_;

    // dynamic reconfigure attributes and methods
    boost::recursive_mutex configuration_mutex_;
  };

} /* namespace move_base_flex */

#include "move_base_flex/abstract_server/impl/abstract_planner_execution.tcc"

#endif /* MOVE_BASE_FLEX__ABSTRACT_PLANNER_EXECUTION_H_ */
