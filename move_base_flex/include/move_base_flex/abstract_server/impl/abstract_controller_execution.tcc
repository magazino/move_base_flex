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

#ifndef MOVE_BASE_FLEX__IMPL__ABSTRACT_CONTROLLER_EXECUTION_TCC_
#define MOVE_BASE_FLEX__IMPL__ABSTRACT_CONTROLLER_EXECUTION_TCC_

namespace move_base_flex
{

template<class CONTROLLER_BASE>
  AbstractControllerExecution<CONTROLLER_BASE>::AbstractControllerExecution(
      boost::condition_variable &condition, const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
      std::string package, std::string class_name) :
      condition_(condition), tf_listener_ptr(tf_listener_ptr), state_(STOPPED),
      class_loader_controller_(package, class_name), moving_(false), plugin_code_(255)
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

template<class CONTROLLER_BASE>
  AbstractControllerExecution<CONTROLLER_BASE>::~AbstractControllerExecution()
  {
  }

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::initialize()
  {
    if (!loadPlugin())
    {
      exit(1);  // TODO: do not exit directly, so we can just show a WARN on reconfigure
    }

    initPlugin();
    setState(INITIALIZED);
  }

template<class CONTROLLER_BASE>
  bool AbstractControllerExecution<CONTROLLER_BASE>::loadPlugin()
  {
    // try to load and init local planner
    ROS_INFO("Load local planner plugin.");
    try
    {
      controller_ = class_loader_controller_.createInstance(plugin_name_);
    }
    catch (const pluginlib::PluginlibException &ex)
    {
      ROS_FATAL_STREAM("Failed to load the " << plugin_name_ << " local planner, are you sure it's properly registered"
                       << " and that the containing library is built? Exception: " << ex.what());
      return false;
    }
    ROS_INFO("Local planner plugin loaded.");

    return true;
  }

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::reconfigure(move_base_flex::MoveBaseFlexConfig &config)
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

template<class CONTROLLER_BASE>
  bool AbstractControllerExecution<CONTROLLER_BASE>::startMoving()
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

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::stopMoving()
  {
    thread_.interrupt();
  }

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::setState(ControllerState state)
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    state_ = state;
  }

template<class CONTROLLER_BASE>
  typename AbstractControllerExecution<CONTROLLER_BASE>::ControllerState
  AbstractControllerExecution<CONTROLLER_BASE>::getState()
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    return state_;
  }

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::setPluginInfo(const uint32_t &plugin_code, const std::string &plugin_msg)
  {
    boost::lock_guard<boost::mutex> guard(pcode_mtx_);
    plugin_code_ =  plugin_code;
    plugin_msg_ = plugin_msg;
  }

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::getPluginInfo(uint32_t &plugin_code, std::string &plugin_msg)
  {
    boost::lock_guard<boost::mutex> guard(pcode_mtx_);
    plugin_code = plugin_code_;
    plugin_msg = plugin_msg_;
  }

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::setNewPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
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

template<class CONTROLLER_BASE>
  bool AbstractControllerExecution<CONTROLLER_BASE>::hasNewPlan()
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    return new_plan_;
  }

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::getNewPlan(std::vector<geometry_msgs::PoseStamped> &plan)
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    new_plan_ = false;
    plan = plan_;
  }

template<class CONTROLLER_BASE>
  uint32_t AbstractControllerExecution<CONTROLLER_BASE>::computeVelocityCmd(geometry_msgs::TwistStamped &vel_cmd,
                                                                           std::string& message)
  {
    return controller_->computeVelocityCommands(vel_cmd, message);
  }

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::setVelocityCmd(const geometry_msgs::TwistStamped &vel_cmd)
  {
    boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
    vel_cmd_stamped_ = vel_cmd;
  }

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::getLastValidCmdVel(geometry_msgs::TwistStamped &vel_cmd)
  {
    boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
    vel_cmd = vel_cmd_stamped_;
  }

template<class CONTROLLER_BASE>
  ros::Time AbstractControllerExecution<CONTROLLER_BASE>::getLastPluginCallTime()
  {
    boost::lock_guard<boost::mutex> guard(lct_mtx_);
    return last_call_time_;
  }

template<class CONTROLLER_BASE>
  ros::Time AbstractControllerExecution<CONTROLLER_BASE>::getLastValidCmdVelTime()
  {
    boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
    return vel_cmd_stamped_.header.stamp;
  }

template<class CONTROLLER_BASE>
  bool AbstractControllerExecution<CONTROLLER_BASE>::isPatienceExceeded()
  {
    boost::lock_guard<boost::mutex> guard(lct_mtx_);
    return (patience_ > ros::Duration(0)) && (ros::Time::now() - last_call_time_ > patience_);
  }

template<class CONTROLLER_BASE>
  bool AbstractControllerExecution<CONTROLLER_BASE>::isMoving()
  {
    return moving_ && start_time_ < getLastValidCmdVelTime()
        && !isPatienceExceeded();
  }

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::run()
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

template<class CONTROLLER_BASE>
  void AbstractControllerExecution<CONTROLLER_BASE>::publishZeroVelocity()
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

#endif /* MOVE_BASE_FLEX__IMPL__ABSTRACT_CONTROLLER_EXECUTION_TCC_ */
