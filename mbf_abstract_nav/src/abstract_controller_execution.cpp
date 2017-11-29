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
 *  abstract_controller_execution.tcc
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "mbf_abstract_nav/abstract_controller_execution.h"

namespace move_base_flex
{


  AbstractControllerExecution::AbstractControllerExecution(
      boost::condition_variable &condition, const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr) :
      condition_(condition), tf_listener_ptr(tf_listener_ptr), state_(STOPPED), moving_(false), plugin_code_(255)
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double patience, frequency;

    if(!private_nh.getParam("local_planner", plugin_name_))
    {
      ROS_ERROR_STREAM("Parameter \"local_planner\" not set!");
      exit(0);
    }
    private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
    private_nh.param("map_frame", global_frame_, std::string("map"));
    private_nh.param("controller_max_retries", max_retries_, 10);
    private_nh.param("controller_patience", patience, 1.0);
    private_nh.param("controller_frequency", frequency, 10.0);
    private_nh.param("dist_tolerance", dist_tolerance_, 0.1);
    private_nh.param("angle_tolerance", angle_tolerance_, M_PI / 18.0);

    // Timeout granted to the local planner. We keep calling it up to this time or up to max_retries times
    // If it doesn't return within time, the navigator will cancel it and abort the corresponding action
    patience_ = ros::Duration(patience);

    if (frequency <= 0.0)
    {
      ROS_ERROR("Movement frequency must be greater than 0.0!");
      exit(0);
    }
    // set the calling duration by the moving frequency
    calling_duration_ = boost::chrono::microseconds((int)(1e6 / frequency));

    // init cmd_vel publisher for the robot velocity t
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  }


  AbstractControllerExecution::~AbstractControllerExecution()
  {
  }


  void AbstractControllerExecution::initialize()
  {
    controller_ = loadControllerPlugin(plugin_name_);
    if (!controller_)
    {
      exit(1);  // TODO: do not exit directly, so we can just show a WARN on reconfigure
    }

    initPlugin();
    setState(INITIALIZED);
  }

  void AbstractControllerExecution::reconfigure(mbf_abstract_nav::MoveBaseFlexConfig &config)
  {
    boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

    if (config.local_planner != plugin_name_)
    {
      plugin_name_ = config.local_planner;
      initialize();
      new_plan_ = true;  // ensure we reset the current plan (if any) to the new controller
    }

    patience_ = ros::Duration(config.controller_patience);

    if (config.controller_frequency > 0.0)
    {
      calling_duration_ = boost::chrono::microseconds((int)(1e6 / config.controller_frequency));
    }
    else
      ROS_ERROR("Movement frequency must be greater than 0.0!");

    max_retries_ = config.controller_max_retries;
  }


  bool AbstractControllerExecution::startMoving()
  {
    setState(STARTED);
    if (moving_)
    {
      return false; // thread is already running.
    }
    plugin_code_ = 255;
    plugin_msg_ = "";
    moving_ = true;
    thread_ = boost::thread(&AbstractControllerExecution::run, this);
    return true;
  }


  void AbstractControllerExecution::stopMoving()
  {
    thread_.interrupt();
  }


  void AbstractControllerExecution::setState(ControllerState state)
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    state_ = state;
  }


  typename AbstractControllerExecution::ControllerState
  AbstractControllerExecution::getState()
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    return state_;
  }


  void AbstractControllerExecution::setPluginInfo(const uint32_t &plugin_code, const std::string &plugin_msg)
  {
    boost::lock_guard<boost::mutex> guard(pcode_mtx_);
    plugin_code_ =  plugin_code;
    plugin_msg_ = plugin_msg;
  }


  void AbstractControllerExecution::getPluginInfo(uint32_t &plugin_code, std::string &plugin_msg)
  {
    boost::lock_guard<boost::mutex> guard(pcode_mtx_);
    plugin_code = plugin_code_;
    plugin_msg = plugin_msg_;
  }


  void AbstractControllerExecution::setNewPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
  {
    // TODO make a better warning here!
    if (getState() == GOT_LOCAL_CMD)
    {
      ROS_WARN("Setting new path while moving!");
    }
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    new_plan_ = true;

    plan_ = plan;
  }


  bool AbstractControllerExecution::hasNewPlan()
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    return new_plan_;
  }


  void AbstractControllerExecution::getNewPlan(std::vector<geometry_msgs::PoseStamped> &plan)
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    new_plan_ = false;
    plan = plan_;
  }


  uint32_t AbstractControllerExecution::computeVelocityCmd(geometry_msgs::TwistStamped &vel_cmd,
                                                                           std::string& message)
  {
    return controller_->computeVelocityCommands(vel_cmd, message);
  }


  void AbstractControllerExecution::setVelocityCmd(const geometry_msgs::TwistStamped &vel_cmd)
  {
    boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
    vel_cmd_stamped_ = vel_cmd;
  }


  void AbstractControllerExecution::getLastValidCmdVel(geometry_msgs::TwistStamped &vel_cmd)
  {
    boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
    vel_cmd = vel_cmd_stamped_;
  }


  ros::Time AbstractControllerExecution::getLastPluginCallTime()
  {
    boost::lock_guard<boost::mutex> guard(lct_mtx_);
    return last_call_time_;
  }


  ros::Time AbstractControllerExecution::getLastValidCmdVelTime()
  {
    boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
    return vel_cmd_stamped_.header.stamp;
  }


  bool AbstractControllerExecution::isPatienceExceeded()
  {
    boost::lock_guard<boost::mutex> guard(lct_mtx_);
    return (patience_ > ros::Duration(0)) && (ros::Time::now() - last_call_time_ > patience_);
  }


  bool AbstractControllerExecution::isMoving()
  {
    return moving_ && start_time_ < getLastValidCmdVelTime()
        && !isPatienceExceeded();
  }


  void AbstractControllerExecution::run()
  {

    start_time_ = ros::Time::now();

    // init plan
    std::vector<geometry_msgs::PoseStamped> plan;
    if (!hasNewPlan())
    {
      setState(NO_PLAN);
      moving_ = false;
      ROS_ERROR("robot navigation moving has no plan!");
    }

    int retries = 0;
    int seq = 0;

    try
    {
      while (moving_ && ros::ok())
      {
        boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

        boost::chrono::thread_clock::time_point loop_start_time = boost::chrono::thread_clock::now();

        // update plan dynamically
        if (hasNewPlan())
        {
          getNewPlan(plan);

          // check if plan is empty
          if (plan.empty())
          {
            setState(EMPTY_PLAN);
            condition_.notify_all();
            moving_ = false;
            return;
          }

          // check if plan could be set
          if(!controller_->setPlan(plan))
          {
            setState(INVALID_PLAN);
            condition_.notify_all();
            moving_ = false;
            return;
          }

        }

        // ask planner if the goal is reached
        if (controller_->isGoalReached(dist_tolerance_, angle_tolerance_))
        {
          setState(ARRIVED_GOAL);
          // goal reached, tell it the controller
          condition_.notify_all();
          moving_ = false;
          // if not, keep moving
        }
        else
        {
          setState(PLANNING);

          // save time and call the plugin
          lct_mtx_.lock();
          last_call_time_ = ros::Time::now();
          lct_mtx_.unlock();

          // call plugin to compute the next velocity command
          std::string message;
          geometry_msgs::TwistStamped cmd_vel_stamped;
          uint32_t outcome = computeVelocityCmd(cmd_vel_stamped, message);
          setPluginInfo(outcome, message);

          if (outcome < 10)
          {
            // set stamped values: frame id, time stamp and sequence number
            cmd_vel_stamped.header.seq = seq++;
            setVelocityCmd(cmd_vel_stamped);
            setState(GOT_LOCAL_CMD);
            vel_pub_.publish(cmd_vel_stamped.twist);
            condition_.notify_all();
            retries = 0;
          }
          else
          {
            if (++retries > max_retries_)
            {
              setState(MAX_RETRIES);
              moving_ = false;
              condition_.notify_all();
            }
            else if (ros::Time::now() - getLastValidCmdVelTime() > patience_
                && ros::Time::now() - start_time_ > patience_)  // why not isPatienceExceeded() ?
            {
              setState(PAT_EXCEEDED);
              moving_ = false;
              condition_.notify_all();
            }
            else
            {
              setState(NO_LOCAL_CMD); // useful for server feedback
              condition_.notify_all();
            }
            // could not compute a valid velocity command -> stop moving the robot
            publishZeroVelocity(); // command the robot to stop
          }
        }

        boost::chrono::thread_clock::time_point end_time = boost::chrono::thread_clock::now();
        boost::chrono::microseconds execution_duration =
            boost::chrono::duration_cast<boost::chrono::microseconds>(end_time - loop_start_time);
        boost::chrono::microseconds sleep_time = calling_duration_ - execution_duration;
        if (moving_ && ros::ok())
        {
          if (sleep_time > boost::chrono::microseconds(0))
          {
            // interruption point
            boost::this_thread::sleep_for(sleep_time);
          }
          else
          {
            ROS_WARN_THROTTLE(1.0, "Calculation needs to much time to stay in the moving frequency!");
          }
        }
      }
    }
    catch (const boost::thread_interrupted &ex)
    {
      // Controller thread interrupted; probably robot is oscillating or we have exceeded planner patience
      ROS_WARN_STREAM("Controller thread interrupted!");
      publishZeroVelocity();
      setState(STOPPED);
      condition_.notify_all();
      moving_ = false;
    }
  }


  void AbstractControllerExecution::publishZeroVelocity()
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    vel_pub_.publish(cmd_vel);
  }

} /* namespace move_base_flex */
