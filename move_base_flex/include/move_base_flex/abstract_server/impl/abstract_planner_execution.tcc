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
 *  abstract_planner_execution.tcc
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MOVE_BASE_FLEX__IMPL__ABSTRACT_PLANNER_EXECUTION_TCC_
#define MOVE_BASE_FLEX__IMPL__ABSTRACT_PLANNER_EXECUTION_TCC_

namespace move_base_flex
{

template<class GLOBAL_PLANNER_BASE>
  AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::AbstractPlannerExecution(boost::condition_variable &condition,
                                                                          std::string package, std::string class_name) :
      condition_(condition), state_(STOPPED), planning_(false), has_new_start_(false), has_new_goal_(false),
      class_loader_global_planner_(package, class_name), plugin_code_(255)
  {
    loadParams();
  }

template<class GLOBAL_PLANNER_BASE>
  AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::~AbstractPlannerExecution()
  {
  }

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::initialize()
  {
    ROS_INFO("Load global planner plugin.");
    try
    {
      global_planner_ = class_loader_global_planner_.createInstance(plugin_name_);
    }
    catch (const pluginlib::PluginlibException &ex)
    {
      ROS_FATAL_STREAM("Failed to load the " << plugin_name_ << " global planner, are you sure it is properly registered"
                       << " and that the containing library is built? Exception: " << ex.what());
      exit(1);  // TODO: do not exit directly, so we can just show a WARN on reconfigure
    }
    ROS_INFO("Global planner plugin loaded.");

    initPlannerPlugin();
    setState(INITIALIZED);
  }

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::reconfigure(move_base_flex::MoveBaseFlexConfig &config)
  {
    boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

    if (config.global_planner != plugin_name_)
    {
      plugin_name_ = config.global_planner;
      initialize();
    }

    max_retries_ = config.global_planner_max_retries;
    patience_ = ros::Duration(config.global_planner_patience);

    // replanning chrono setup
    if (config.global_planner_frequency > 0.0)
    {
      calling_duration_ = boost::chrono::microseconds((int)(1e6 / config.global_planner_frequency));
    }
    else
    {
      calling_duration_.zero();
    }
  }

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::loadParams()
  {
    ros::NodeHandle private_nh("~");

    double patience, frequency;

    private_nh.param("global_planner", plugin_name_, std::string("navfn/NavfnROS"));
    private_nh.param("robot_frame", robot_frame_, std::string("base_footprint"));
    private_nh.param("map_frame", global_frame_, std::string("map"));
    private_nh.param("global_planner_max_retries", max_retries_, 10);
    private_nh.param("global_planner_patience", patience, 5.0);
    private_nh.param("global_planner_frequency", frequency, 0.0);

    // Timeout granted to the global planner. We keep calling it up to this time or up to max_retries times
    // If it doesn't return within time, the navigator will cancel it and abort the corresponding action
    patience_ = ros::Duration(patience);

    // replanning chrono setup
    if (frequency > 0.0)
    {
      calling_duration_ = boost::chrono::microseconds((int)(1e6 / frequency));
    }
  }


template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::setPluginInfo(const uint8_t& plugin_code, const std::string& plugin_msg)
{
  boost::lock_guard<boost::mutex> guard(pcode_mtx_);
  plugin_code_ =  plugin_code;
  plugin_msg_ = plugin_msg;
}

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::getPluginInfo(uint8_t& plugin_code, std::string& plugin_msg)
{
  boost::lock_guard<boost::mutex> guard(pcode_mtx_);
  plugin_code = plugin_code_;
  plugin_msg = plugin_msg_;
}

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::setState(PlanningState state)
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    state_ = state;
  }

template<class GLOBAL_PLANNER_BASE>
  typename AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::PlanningState AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::getState()
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    return state_;
  }

template<class GLOBAL_PLANNER_BASE>
  ros::Time AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::getLastValidPlanTime()
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    return last_valid_plan_time_;
  }

template<class GLOBAL_PLANNER_BASE>
  ros::Time AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::getLastCycleStartTime()
  {
    boost::lock_guard<boost::mutex> guard(lct_mtx_);
    return last_cycle_start_time_;
  }

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::setLastCycleStartTime()
  {
    boost::lock_guard<boost::mutex> guard(lct_mtx_);
    last_cycle_start_time_ = ros::Time::now();
  }

template<class GLOBAL_PLANNER_BASE>
  bool AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::isPatienceExceeded()
  {
    boost::lock_guard<boost::mutex> guard(lct_mtx_);
    return (patience_ > ros::Duration(0)) && (ros::Time::now() - last_cycle_start_time_ > patience_);
  }

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::getNewPlan(std::vector<geometry_msgs::PoseStamped> &plan,
                                                                 double &cost)
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    // copy plan and costs to output
    plan = plan_;
    cost = cost_;
  }

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::setNewPlan(const std::vector<geometry_msgs::PoseStamped> &plan,
                                                                 const double &cost)
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    plan_ = plan;
    cost_ = cost;
    last_valid_plan_time_ = ros::Time::now();
  }

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::setNewGoal(const geometry_msgs::PoseStamped &goal,
                                                                 const double &tolerance)
  {
    boost::lock_guard<boost::mutex> guard(goal_start_mtx_);
    goal_ = goal;
    tolerance_ = tolerance;
    has_new_goal_ = true;
  }

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::setNewStart(const geometry_msgs::PoseStamped &start)
  {
    boost::lock_guard<boost::mutex> guard(goal_start_mtx_);
    start_ = start;
    has_new_start_ = true;
  }

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::setNewStartAndGoal(const geometry_msgs::PoseStamped &start,
                                                                         const geometry_msgs::PoseStamped &goal,
                                                                         const double &tolerance)
  {
    boost::lock_guard<boost::mutex> guard(goal_start_mtx_);
    start_ = start;
    goal_ = goal;
    tolerance_ = tolerance;
    has_new_start_ = true;
    has_new_goal_ = true;
  }

template<class GLOBAL_PLANNER_BASE>
  bool AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::startPlanning(const geometry_msgs::PoseStamped &start,
                                                                    const geometry_msgs::PoseStamped &goal,
                                                                    const double &tolerance)
  {
    if (planning_)
    {
      return false;
    }
    planning_ = true;
    cancel_ = false;
    start_ = start;
    goal_ = goal;
    tolerance_ = tolerance;

    geometry_msgs::Point s = start.pose.position;
    geometry_msgs::Point g = goal.pose.position;

    ROS_INFO_STREAM("Start planning from the start pose: (" << s.x << ", " << s.y << ", " << s.z << ")"
                    << " to the goal pose: ("<< g.x << ", " << g.y << ", " << g.z << ")");

    setState(STARTED);
    thread_ = boost::thread(&AbstractPlannerExecution::run, this);
    return true;
  }

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::stopPlanning()
  {
    // only useful if there are any interruption points in the global planner
    thread_.interrupt();
  }

template<class GLOBAL_PLANNER_BASE>
  bool AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::cancel()
  {
    cancel_ = true;  // force cancel immediately, as the call to cancel in the planner can take a while
    cancel_ = global_planner_->cancel();
    return cancel_;
  }

template<class GLOBAL_PLANNER_BASE>
  void AbstractPlannerExecution<GLOBAL_PLANNER_BASE>::run()
  {
    int retries = 0;
    geometry_msgs::PoseStamped current_start = start_;
    geometry_msgs::PoseStamped current_goal = goal_;
    double current_tolerance = tolerance_;
    bool success = false;
    bool make_plan = false;
    bool exceeded = false;

    last_valid_plan_time_ = ros::Time::now();

    try
    {
      while (planning_ && ros::ok())
      {
        boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

        boost::chrono::thread_clock::time_point start_time = boost::chrono::thread_clock::now();

        setLastCycleStartTime();
        // call the planner
        std::vector<geometry_msgs::PoseStamped> plan;
        double cost;

        // lock goal start mutex
        goal_start_mtx_.lock();
        if (has_new_start_)
        {
          has_new_start_ = false;
          current_start = start_;
          ROS_INFO_STREAM("A new start pose is available. Planning with the new start pose!");
          exceeded = false;
          geometry_msgs::Point s = start_.pose.position;
          ROS_INFO_STREAM("New planning start pose: (" << s.x << ", " << s.y << ", " << s.z << ")");
        }
        if (has_new_goal_)
        {
          has_new_goal_ = false;
          current_goal = goal_;
          current_tolerance = tolerance_;
          ROS_INFO_STREAM("A new goal pose is available. Planning with the new goal pose and the tolerance: "
                          << current_tolerance);
          exceeded = false;
          geometry_msgs::Point g = goal_.pose.position;
          ROS_INFO_STREAM("New goal pose: (" << g.x << ", " << g.y << ", " << g.z << ")");
        }

        make_plan = !(success || exceeded) || has_new_start_ || has_new_goal_;

        // unlock goal
        goal_start_mtx_.unlock();
        setState(PLANNING);
        //ROS_INFO_STREAM("Start planning");
        if (make_plan)
        {
          //ROS_INFO_STREAM("Start planning");

          uint8_t plugin_code = 255;
          std::string plugin_msg;
          success = global_planner_->makePlan(current_start, current_goal, current_tolerance, plan, cost,
                                              plugin_code, plugin_msg);

          setPluginInfo(plugin_code, plugin_msg);

          if (cancel_ && !isPatienceExceeded())
          {
            setState(CANCELED);
            ROS_INFO_STREAM("The global planner has been canceled!"); // but not due to patience exceeded
            planning_ = false;
            condition_.notify_all();
          }
          else if (success)
          {
            ROS_INFO_STREAM("Successfully found a plan.");
            exceeded = false;
            planning_ = false;

            setNewPlan(plan, cost);
            setState(FOUND_PLAN);
            condition_.notify_all(); // notify observer
          }
          else if (max_retries_ > 0 && ++retries > max_retries_)
          {
            ROS_INFO_STREAM("Planning reached max retries!");
            setState(MAX_RETRIES);
            exceeded = true;
            planning_ = false;
            condition_.notify_all(); // notify observer
          }
          else if (isPatienceExceeded())
          {
            ROS_INFO_STREAM("Planning patience has been exceeded" << (cancel_ ? "; planner canceled!" : "!"));
            setState(PAT_EXCEEDED);
            exceeded = true;
            planning_ = false;
            condition_.notify_all(); // notify observer
          }
          else if (max_retries_ == 0 && patience_ == ros::Duration(0))
          {
            ROS_INFO_STREAM("Planning could not find a plan!");
            exceeded = true;
            setState(NO_PLAN_FOUND);
            condition_.notify_all(); // notify observer
            planning_ = false;
          }
          else
          {
            exceeded = false;
            //ROS_INFO_STREAM("Planning could not find a plan! Trying again.");
          }
        }
        else if (cancel_)
        {
          ROS_INFO_STREAM("The global planner has been canceled!");
          setState(CANCELED);
          planning_ = false;
          condition_.notify_all();
        }

        //compute sleep time
        boost::chrono::thread_clock::time_point end_time = boost::chrono::thread_clock::now();
        boost::chrono::microseconds execution_duration =
            boost::chrono::duration_cast<boost::chrono::microseconds>(end_time - start_time);
        boost::chrono::microseconds sleep_time = calling_duration_ - execution_duration;
        if (planning_ && ros::ok())
        { // do not sleep if finished
          if (sleep_time > boost::chrono::microseconds(0))
          {
            // interruption point
            boost::this_thread::sleep_for(sleep_time);
          }
          else
          {
            ROS_WARN_THROTTLE(100, "Planning needs to much time to stay in the planning frequency!");
          }
        }
      } // while (planning_ && ros::ok())
    }
    catch (boost::thread_interrupted &ex)
    {
      setState(STOPPED);
      condition_.notify_all(); // notify observer
      planning_ = false;
    }
  }
} /* namespace move_base_flex */

#endif /* MOVE_BASE_FLEX__IMPL__ABSTRACT_PLANNER_EXECUTION_TCC_ */
