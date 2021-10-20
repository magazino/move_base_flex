/*
 *  Copyright 2018, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
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
 *  abstract_planner_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "mbf_abstract_nav/abstract_planner_execution.h"

namespace mbf_abstract_nav
{
AbstractPlannerExecution::AbstractPlannerExecution(const std::string& name,
                                                   const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr,
                                                   const MoveBaseFlexConfig& config)
  : AbstractExecutionBase(name)
  , planner_(planner_ptr)
  , state_(INITIALIZED)
  , max_retries_(0)
  , planning_(false)
  , has_new_start_(false)
  , has_new_goal_(false)
{
  ros::NodeHandle private_nh("~");

  // non-dynamically reconfigurable parameters
  private_nh.param("robot_frame", robot_frame_, std::string("base_footprint"));
  private_nh.param("map_frame", global_frame_, std::string("map"));

  // dynamically reconfigurable parameters
  reconfigure(config);
}

AbstractPlannerExecution::AbstractPlannerExecution(const std::string& name,
                                                   const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr,
                                                   const TFPtr& tf_listener_ptr, const MoveBaseFlexConfig& config)
  : AbstractExecutionBase(name)
  , planner_(planner_ptr)
  , state_(INITIALIZED)
  , max_retries_(0)
  , planning_(false)
  , tf_listener_ptr_(tf_listener_ptr)
  , has_new_start_(false)
  , has_new_goal_(false)
{
  ros::NodeHandle private_nh("~");

  // non-dynamically reconfigurable parameters
  private_nh.param("robot_frame", robot_frame_, std::string("base_footprint"));
  private_nh.param("map_frame", global_frame_, std::string("map"));

  // dynamically reconfigurable parameters
  reconfigure(config);
}

AbstractPlannerExecution::~AbstractPlannerExecution()
{
}

template <typename _Iter>
double sumDistance(_Iter _begin, _Iter _end)
{
  // helper function to get the distance of a path.
  // in C++11, we could add static_assert on the interator_type.
  double dist = 0.;

  // minimum length of the path is 2.
  if (std::distance(_begin, _end) < 2)
    return dist;

  // two pointer iteration
  for (_Iter next = _begin + 1; next != _end; ++_begin, ++next)
    dist += mbf_utility::distance(*_begin, *next);

  return dist;
}

double AbstractPlannerExecution::getCost() const
{
  return cost_;
}

void AbstractPlannerExecution::reconfigure(const MoveBaseFlexConfig &config)
{
  boost::lock_guard<boost::mutex> guard(configuration_mutex_);

  max_retries_ = config.planner_max_retries;
  frequency_ = config.planner_frequency;

  // Timeout granted to the global planner. We keep calling it up to this time or up to max_retries times
  // If it doesn't return within time, the navigator will cancel it and abort the corresponding action
  try
  {
    patience_ = ros::Duration(config.planner_patience);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_STREAM("Failed to set planner_patience: " << ex.what());
    patience_ = ros::Duration(0);
  }
}


typename AbstractPlannerExecution::PlanningState AbstractPlannerExecution::getState() const
{
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  return state_;
}

void AbstractPlannerExecution::setState(PlanningState state, bool signalling)
{
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  state_ = state;

  // we exit planning if we are signalling.
  planning_ = !signalling;

  // some states are quiet, most aren't
  if(signalling)
    condition_.notify_all();
}


ros::Time AbstractPlannerExecution::getLastValidPlanTime() const
{
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  return last_valid_plan_time_;
}


bool AbstractPlannerExecution::isPatienceExceeded() const
{
  return !patience_.isZero() && (ros::Time::now() - last_call_start_time_ > patience_);
}


std::vector<geometry_msgs::PoseStamped> AbstractPlannerExecution::getPlan() const
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


bool AbstractPlannerExecution::start(const geometry_msgs::PoseStamped &start,
                                     const geometry_msgs::PoseStamped &goal,
                                     double tolerance)
{
  if (planning_)
  {
    return false;
  }
  boost::lock_guard<boost::mutex> guard(planning_mtx_);
  planning_ = true;
  start_ = start;
  goal_ = goal;
  tolerance_ = tolerance;

  const geometry_msgs::Point& s = start.pose.position;
  const geometry_msgs::Point& g = goal.pose.position;

  ROS_DEBUG_STREAM("Start planning from the start pose: (" << s.x << ", " << s.y << ", " << s.z << ")"
                                 << " to the goal pose: ("<< g.x << ", " << g.y << ", " << g.z << ")");

  return AbstractExecutionBase::start();
}


bool AbstractPlannerExecution::cancel()
{
  cancel_ = true; // force cancel immediately, as the call to cancel in the planner can take a while

  // returns false if cancel is not implemented or rejected by the planner (will run until completion)
  if (!planner_->cancel())
  {
    ROS_WARN_STREAM("Cancel planning failed or is not supported by the plugin. "
        << "Wait until the current planning finished!");

    return false;
  }
  return true;
}

uint32_t AbstractPlannerExecution::makePlan(const geometry_msgs::PoseStamped &start,
                                            const geometry_msgs::PoseStamped &goal,
                                            double tolerance,
                                            std::vector<geometry_msgs::PoseStamped> &plan,
                                            double &cost,
                                            std::string &message)
{
  return planner_->makePlan(start, goal, tolerance, plan, cost, message);
}

void AbstractPlannerExecution::run()
{
  setState(STARTED, false);
  boost::lock_guard<boost::mutex> guard(planning_mtx_);
  int retries = 0;
  geometry_msgs::PoseStamped current_start = start_;
  geometry_msgs::PoseStamped current_goal = goal_;
  double current_tolerance = tolerance_;

  last_call_start_time_ = ros::Time::now();
  last_valid_plan_time_ = ros::Time::now();

  try
  {
    while (planning_ && ros::ok())
    {
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
        const geometry_msgs::Point& s = start_.pose.position;
        ROS_INFO_STREAM("New planning start pose: (" << s.x << ", " << s.y << ", " << s.z << ")");
      }
      if (has_new_goal_)
      {
        has_new_goal_ = false;
        current_goal = goal_;
        current_tolerance = tolerance_;
        ROS_INFO_STREAM("A new goal pose is available. Planning with the new goal pose and the tolerance: "
                        << current_tolerance);
        const geometry_msgs::Point& g = goal_.pose.position;
        ROS_INFO_STREAM("New goal pose: (" << g.x << ", " << g.y << ", " << g.z << ")");
      }

      // unlock goal
      goal_start_mtx_.unlock();
      if (cancel_)
      {
        ROS_INFO_STREAM("The global planner has been canceled!");
        setState(CANCELED, true);
      }
      else
      {
        setState(PLANNING, false);

        outcome_ = makePlan(current_start, current_goal, current_tolerance, plan, cost, message_);
        bool success = outcome_ < 10;

        boost::lock_guard<boost::mutex> guard(configuration_mutex_);

        if (cancel_ && !isPatienceExceeded())
        {
          ROS_INFO_STREAM("The planner \"" << name_ << "\" has been canceled!"); // but not due to patience exceeded
          setState(CANCELED, true);
        }
        else if (success)
        {
          ROS_DEBUG_STREAM("Successfully found a plan.");

          boost::lock_guard<boost::mutex> plan_mtx_guard(plan_mtx_);
          plan_ = plan;
          cost_ = cost;
          // estimate the cost based on the distance if its zero.
          if (cost_ == 0)
            cost_ = sumDistance(plan_.begin(), plan_.end());

          last_valid_plan_time_ = ros::Time::now();
          setState(FOUND_PLAN, true);
        }
        else if (max_retries_ > 0 && ++retries > max_retries_)
        {
          ROS_INFO_STREAM("Planning reached max retries! (" << max_retries_ << ")");
          setState(MAX_RETRIES, true);
        }
        else if (isPatienceExceeded())
        {
          // Patience exceeded is handled at two levels: here to stop retrying planning when max_retries is
          // disabled, and on the navigation server when the planner doesn't return for more that patience seconds.
          // In the second case, the navigation server has tried to cancel planning (possibly without success, as
          // old nav_core-based planners do not support canceling), and we add here the fact to the log for info
          ROS_INFO_STREAM("Planning patience (" << patience_.toSec() << "s) has been exceeded"
                                                << (cancel_ ? "; planner canceled!" : ""));
          setState(PAT_EXCEEDED, true);
        }
        else if (max_retries_ == 0 && patience_.isZero())
        {
          ROS_INFO_STREAM("Planning could not find a plan!");
          setState(NO_PLAN_FOUND, true);
        }
        else
        {
          ROS_DEBUG_STREAM("Planning could not find a plan! Trying again...");
        }
      }
    } // while (planning_ && ros::ok())
  }
  catch (const boost::thread_interrupted &ex)
  {
    // Planner thread interrupted; probably we have exceeded planner patience
    ROS_WARN_STREAM("Planner thread interrupted!");
    setState(STOPPED, true);
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Unknown error occurred: " << boost::current_exception_diagnostic_information());
    setState(INTERNAL_ERROR, true);
  }
}

} /* namespace mbf_abstract_nav */

