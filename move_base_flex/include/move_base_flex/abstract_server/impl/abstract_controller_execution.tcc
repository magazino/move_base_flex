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

template<class LOCAL_PLANNER_BASE>
  AbstractControllerExecution<LOCAL_PLANNER_BASE>::AbstractControllerExecution(
      boost::condition_variable &condition, const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
      std::string package, std::string class_name) :
      condition_(condition), tf_listener_ptr(tf_listener_ptr), state_(STOPPED),
      class_loader_local_planner_(package, class_name), moving_(false), plugin_code_(255)
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
    private_nh.param("local_planner_max_retries", max_retries_, 10);
    private_nh.param("local_planner_patience", patience, 1.0);
    private_nh.param("local_planner_frequency", frequency, 10.0);
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

template<class LOCAL_PLANNER_BASE>
  AbstractControllerExecution<LOCAL_PLANNER_BASE>::~AbstractControllerExecution()
  {
  }

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::initialize()
  {
    if (!loadPlugin())
    {
      exit(1);  // TODO: do not exit directly, so we can just show a WARN on reconfigure
    }

    initPlugin();
    setState(INITIALIZED);
  }

template<class LOCAL_PLANNER_BASE>
  bool AbstractControllerExecution<LOCAL_PLANNER_BASE>::loadPlugin()
  {
    // try to load and init local planner
    ROS_INFO("Load local planner plugin.");
    try
    {
      local_planner_ = class_loader_local_planner_.createInstance(plugin_name_);
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

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::reconfigure(move_base_flex::MoveBaseFlexConfig &config)
  {
    boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

    if (config.local_planner != plugin_name_)
    {
      plugin_name_ = config.local_planner;
      initialize();
      new_plan_ = true;  // ensure we reset the current plan (if any) to the new controller
    }

    patience_ = ros::Duration(config.local_planner_patience);

    if (config.local_planner_frequency > 0.0)
    {
      calling_duration_ = boost::chrono::microseconds((int)(1e6 / config.local_planner_frequency));
    }
    else
      ROS_ERROR("Movement frequency must be greater than 0.0!");

    max_retries_ = config.local_planner_max_retries;
  }

template<class LOCAL_PLANNER_BASE>
  bool AbstractControllerExecution<LOCAL_PLANNER_BASE>::startMoving()
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

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::stopMoving()
  {
    thread_.interrupt();
  }

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::setState(ControllerState state)
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    state_ = state;
  }

template<class LOCAL_PLANNER_BASE>
  typename AbstractControllerExecution<LOCAL_PLANNER_BASE>::ControllerState
  AbstractControllerExecution<LOCAL_PLANNER_BASE>::getState()
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    return state_;
  }

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::setPluginInfo(const uint32_t &plugin_code, const std::string &plugin_msg)
  {
    boost::lock_guard<boost::mutex> guard(pcode_mtx_);
    plugin_code_ =  plugin_code;
    plugin_msg_ = plugin_msg;
  }

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::getPluginInfo(uint32_t &plugin_code, std::string &plugin_msg)
  {
    boost::lock_guard<boost::mutex> guard(pcode_mtx_);
    plugin_code = plugin_code_;
    plugin_msg = plugin_msg_;
  }

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::setNewPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
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

template<class LOCAL_PLANNER_BASE>
  bool AbstractControllerExecution<LOCAL_PLANNER_BASE>::hasNewPlan()
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    return new_plan_;
  }

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::getNewPlan(std::vector<geometry_msgs::PoseStamped> &plan)
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    new_plan_ = false;
    plan = plan_;
  }

template<class LOCAL_PLANNER_BASE>
  uint32_t AbstractControllerExecution<LOCAL_PLANNER_BASE>::getVelocityCmd(geometry_msgs::TwistStamped &vel_cmd,
                                                                           std::string& message)
  {
    return local_planner_->computeVelocityCommands(vel_cmd, message);
  }

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::setVelocityCmd(const geometry_msgs::TwistStamped &vel_cmd)
  {
    boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
    vel_cmd_stamped_ = vel_cmd;
  }

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::getLastValidCmdVel(geometry_msgs::TwistStamped &vel_cmd)
  {
    boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
    vel_cmd = vel_cmd_stamped_;
  }

template<class LOCAL_PLANNER_BASE>
  ros::Time AbstractControllerExecution<LOCAL_PLANNER_BASE>::getLastPluginCallTime()
  {
    boost::lock_guard<boost::mutex> guard(lct_mtx_);
    return last_call_time_;
  }

template<class LOCAL_PLANNER_BASE>
  ros::Time AbstractControllerExecution<LOCAL_PLANNER_BASE>::getLastValidCmdVelTime()
  {
    boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
    return vel_cmd_stamped_.header.stamp;
  }

template<class LOCAL_PLANNER_BASE>
  bool AbstractControllerExecution<LOCAL_PLANNER_BASE>::isPatienceExceeded()
  {
    boost::lock_guard<boost::mutex> guard(lct_mtx_);
    return (patience_ > ros::Duration(0)) && (ros::Time::now() - last_call_time_ > patience_);
  }

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::run()
  {

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

    // init time here for patience
    geometry_msgs::TwistStamped cmd_vel_stamped;
    cmd_vel_stamped.header.stamp = ros::Time::now();
    setVelocityCmd(cmd_vel_stamped);

    try
    {
      while (moving_ && ros::ok())
      {
        boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

        boost::chrono::thread_clock::time_point start_time = boost::chrono::thread_clock::now();

        // update plan dynamically
        if (hasNewPlan())
        {
          getNewPlan(plan);
          if (plan.empty() || !local_planner_->setPlan(plan))
          {
            setState(plan.empty() ? EMPTY_PLAN : INVALID_PLAN);
            condition_.notify_all();
            moving_ = false;
            return;
          }
          else
          {
            local_planner_->setPlan(plan);
          }
        }

        // ask planner if the goal is reached
        if (local_planner_->isGoalReached(dist_tolerance_, angle_tolerance_))
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
          uint32_t outcome = getVelocityCmd(cmd_vel_stamped, message);
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
            else if (ros::Time::now() - getLastValidCmdVelTime() > patience_)  // why not isPatienceExceeded() ?
            {
              setState(PAT_EXCEEDED);
              moving_ = false;
              condition_.notify_all();
            }
            else
            {
              setState(NO_LOCAL_CMD);
              condition_.notify_all();
            }
            publishZeroVelocity(); // command the robot to stop
          }
        }

        boost::chrono::thread_clock::time_point end_time = boost::chrono::thread_clock::now();
        boost::chrono::microseconds execution_duration =
            boost::chrono::duration_cast<boost::chrono::microseconds>(end_time - start_time);
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

template<class LOCAL_PLANNER_BASE>
  void AbstractControllerExecution<LOCAL_PLANNER_BASE>::publishZeroVelocity()
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
