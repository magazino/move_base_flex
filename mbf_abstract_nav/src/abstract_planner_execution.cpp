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

#include "mbf_abstract_nav/abstract_planner_execution.h"
#include <xmlrpcpp/XmlRpcException.h>
#include <boost/exception/diagnostic_information.hpp>

namespace mbf_abstract_nav
{


  AbstractPlannerExecution::AbstractPlannerExecution(boost::condition_variable &condition) :
      condition_(condition), state_(STOPPED), planning_(false),
      has_new_start_(false), has_new_goal_(false), outcome_(255)
  {
    ros::NodeHandle private_nh("~");

    // non-dynamically reconfigurable parameters
    private_nh.param("robot_frame", robot_frame_, std::string("base_footprint"));
    private_nh.param("map_frame", global_frame_, std::string("map"));
  }

  AbstractPlannerExecution::~AbstractPlannerExecution()
  {
  }


  bool AbstractPlannerExecution::initialize()
  {
    return loadPlugins();
  }

  bool AbstractPlannerExecution::switchPlanner(const std::string& name){
    if(name == plugin_name_)
    {
      ROS_DEBUG_STREAM("No planner switch necessary, \"" << name << "\" already set.");
      return true;
    }
    std::map<std::string, mbf_abstract_core::AbstractPlanner::Ptr>::iterator new_planner
        = planners_.find(name);
    if(new_planner != planners_.end())
    {
      plugin_name_ = new_planner->first;
      planner_ = new_planner->second;
      ROS_INFO_STREAM("Switched the planner plugin to \"" << new_planner->first << "\" with "
          << "the type \"" << planners_type_[new_planner->first] << "\"");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("The planner \"" << name << "\" has not yet been loaded!"
          << " No switch of the planner!");
      return false;
    }
  }

  bool AbstractPlannerExecution::loadPlugins()
  {
    ros::NodeHandle private_nh("~");

    XmlRpc::XmlRpcValue planners_param_list;
    if(!private_nh.getParam("planners", planners_param_list))
    {
      ROS_WARN_STREAM("No planners configured! - Use the param \"planners\", which must be a list of tuples with a name and a type.");
      return false;
    }

    try
    {
      for (int i = 0; i < planners_param_list.size(); i++)
      {
        XmlRpc::XmlRpcValue elem = planners_param_list[i];

        std::string name = elem["name"];
        std::string type = elem["type"];

        if (planners_.find(name) != planners_.end())
        {
          ROS_ERROR_STREAM("The planner \"" << name << "\" has already been loaded! Names must be unique!");
          return false;
        }
        mbf_abstract_core::AbstractPlanner::Ptr planner_ptr = loadPlannerPlugin(type);
        if(planner_ptr && initPlugin(name, planner_ptr))
        {
          // set default planner to the first in the list
          if(!planner_)
          {
            planner_ = planner_ptr;
            plugin_name_ = name;
            setState(INITIALIZED);
          }

          planners_.insert(
              std::pair<std::string, mbf_abstract_core::AbstractPlanner::Ptr>(name, planner_ptr));

          planners_type_.insert(std::pair<std::string, std::string>(name, type)); // save name to type mapping

          ROS_INFO_STREAM("The planner with the type \"" << type << "\" has been loaded successfully under the name \""
              << name << "\".");
        }
        else
        {
          ROS_ERROR_STREAM("Could not load the plugin with the name \""
              << name << "\" and the type \"" << type << "\"!");
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("Invalid parameter structure. The \"planners\" parameter has to be a list of structs "
                           << "with fields \"name\" and \"type\" of !");
      ROS_ERROR_STREAM(e.getMessage());
      return false;
    }
    // is there any planner initialized?
    return planner_ ? true : false;
  }

  double AbstractPlannerExecution::getCost()
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    // copy plan and costs to output
    // if the planner plugin do not compute costs compute costs by discrete path length
    if(cost_ == 0 && !plan_.empty())
    {
      ROS_DEBUG_STREAM("Compute costs by discrete path length!");
      double cost = 0;

      geometry_msgs::PoseStamped prev_pose = plan_.front();
      for(std::vector<geometry_msgs::PoseStamped>::iterator iter = plan_.begin() + 1; iter != plan_.end(); ++iter)
      {
        cost += mbf_utility::distance(prev_pose, *iter);
        prev_pose = *iter;
      }
      return cost;
    }
    return cost_;
  }

  void AbstractPlannerExecution::reconfigure(const MoveBaseFlexConfig &config)
  {
    boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

    max_retries_ = config.planner_max_retries;
    frequency_ = config.planner_frequency;

    // Timeout granted to the global planner. We keep calling it up to this time or up to max_retries times
    // If it doesn't return within time, the navigator will cancel it and abort the corresponding action
    patience_ = ros::Duration(config.planner_patience);
  }


  typename AbstractPlannerExecution::PlanningState AbstractPlannerExecution::getState()
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    return state_;
  }

  void AbstractPlannerExecution::setState(PlanningState state)
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    state_ = state;
  }


  ros::Time AbstractPlannerExecution::getLastValidPlanTime()
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    return last_valid_plan_time_;
  }


  bool AbstractPlannerExecution::isPatienceExceeded()
  {
    return !patience_.isZero() && (ros::Time::now() - last_call_start_time_ > patience_);
  }


  std::vector<geometry_msgs::PoseStamped> AbstractPlannerExecution::getPlan()
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    // copy plan and costs to output
    return plan_;
  }


  void AbstractPlannerExecution::setNewGoal(const geometry_msgs::PoseStamped &goal, double tolerance)
  {
    boost::lock_guard<boost::mutex> guard(goal_start_mtx_);
    goal_ = goal;
    tolerance_ = tolerance;
    has_new_goal_ = true;
  }


  void AbstractPlannerExecution::setNewStart(const geometry_msgs::PoseStamped &start)
  {
    boost::lock_guard<boost::mutex> guard(goal_start_mtx_);
    start_ = start;
    has_new_start_ = true;
  }


  void AbstractPlannerExecution::setNewStartAndGoal(const geometry_msgs::PoseStamped &start,
                                                    const geometry_msgs::PoseStamped &goal,
                                                    double tolerance)
  {
    boost::lock_guard<boost::mutex> guard(goal_start_mtx_);
    start_ = start;
    goal_ = goal;
    tolerance_ = tolerance;
    has_new_start_ = true;
    has_new_goal_ = true;
  }


  bool AbstractPlannerExecution::startPlanning(const geometry_msgs::PoseStamped &start,
                                               const geometry_msgs::PoseStamped &goal,
                                               double tolerance)
  {
    if (planning_)
    {
      return false;
    }
    boost::lock_guard<boost::mutex> guard(planning_mtx_);
    planning_ = true;
    cancel_ = false;
    start_ = start;
    goal_ = goal;
    tolerance_ = tolerance;

    geometry_msgs::Point s = start.pose.position;
    geometry_msgs::Point g = goal.pose.position;

    ROS_DEBUG_STREAM("Start planning from the start pose: (" << s.x << ", " << s.y << ", " << s.z << ")"
                                   << " to the goal pose: ("<< g.x << ", " << g.y << ", " << g.z << ")");

    setState(STARTED);
    thread_ = boost::thread(&AbstractPlannerExecution::run, this);
    return true;
  }


  void AbstractPlannerExecution::stopPlanning()
  {
    // only useful if there are any interruption points in the global planner
    ROS_WARN_STREAM("Try to stop the planning rigorously by interrupting the thread!");
    thread_.interrupt();
  }


  bool AbstractPlannerExecution::cancel()
  {
    cancel_ = true;  // force cancel immediately, as the call to cancel in the planner can take a while

    // returns false if cancel is not implemented or rejected by the planner (will run until completion)
    return planner_->cancel();
  }

  uint32_t AbstractPlannerExecution::makePlan(const mbf_abstract_core::AbstractPlanner::Ptr &planner_ptr,
                                              const geometry_msgs::PoseStamped &start,
                                              const geometry_msgs::PoseStamped &goal,
                                              double tolerance,
                                              std::vector<geometry_msgs::PoseStamped> &plan,
                                              double &cost,
                                              std::string &message)
  {
    return planner_ptr->makePlan(start, goal, tolerance, plan, cost, message);
  }

  void AbstractPlannerExecution::run()
  {
    boost::lock_guard<boost::mutex> guard(planning_mtx_);
    int retries = 0;
    geometry_msgs::PoseStamped current_start = start_;
    geometry_msgs::PoseStamped current_goal = goal_;
    double current_tolerance = tolerance_;

    bool success = false;
    bool make_plan = false;
    bool exceeded = false;

    last_call_start_time_ = ros::Time::now();
    last_valid_plan_time_ = ros::Time::now();

    try
    {
      while (planning_ && ros::ok())
      {
        boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

        boost::chrono::thread_clock::time_point start_time = boost::chrono::thread_clock::now();

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
        if (make_plan)
        {
          outcome_ = makePlan(planner_, current_start, current_goal, current_tolerance, plan, cost, message_);
          success = outcome_ < 10;

          if (cancel_ && !isPatienceExceeded())
          {
            setState(CANCELED);
            ROS_INFO_STREAM("The global planner has been canceled!"); // but not due to patience exceeded
            planning_ = false;
            condition_.notify_all();
          }
          else if (success)
          {
            ROS_DEBUG_STREAM("Successfully found a plan.");
            exceeded = false;
            planning_ = false;

            plan_mtx_.lock();
            plan_ = plan;
            cost_ = cost;
            last_valid_plan_time_ = ros::Time::now();
            plan_mtx_.unlock();
            setState(FOUND_PLAN);

            condition_.notify_all(); // notify observer
          }
          else if (max_retries_ >= 0 && ++retries > max_retries_)
          {
            ROS_INFO_STREAM("Planning reached max retries! (" << max_retries_ << ")");
            setState(MAX_RETRIES);
            exceeded = true;
            planning_ = false;
            condition_.notify_all(); // notify observer
          }
          else if (isPatienceExceeded())
          {
            // Patience exceeded is handled at two levels: here to stop retrying planning when max_retries is
            // disabled, and on the navigation server when the planner doesn't return for more that patience seconds.
            // In the second case, the navigation server has tried to cancel planning (possibly without success, as
            // old nav_core-based planners do not support canceling), and we add here the fact to the log for info
            ROS_INFO_STREAM("Planning patience (" << patience_.toSec() << "s) has been exceeded"
                                                  << (cancel_ ? "; planner canceled!" : ""));
            setState(PAT_EXCEEDED);
            exceeded = true;
            planning_ = false;
            condition_.notify_all(); // notify observer
          }
          else if (max_retries_ == 0 && patience_.isZero())
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
            ROS_DEBUG_STREAM("Planning could not find a plan! Trying again...");
          }
        }
        else if (cancel_)
        {
          ROS_INFO_STREAM("The global planner has been canceled!");
          setState(CANCELED);
          planning_ = false;
          condition_.notify_all();
        }
      } // while (planning_ && ros::ok())
    }
    catch (const boost::thread_interrupted &ex)
    {
      // Planner thread interrupted; probably we have exceeded planner patience
      ROS_WARN_STREAM("Planner thread interrupted!");
      setState(STOPPED);
      condition_.notify_all(); // notify observer
      planning_ = false;
    }
    catch (...)
    {
      ROS_FATAL_STREAM("Unknown error occurred: " << boost::current_exception_diagnostic_information());
      setState(INTERNAL_ERROR);
    }
  }

} /* namespace mbf_abstract_nav */

