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
#include <xmlrpcpp/XmlRpcException.h>
#include <mbf_msgs/ExePathResult.h>
#include <boost/exception/diagnostic_information.hpp>

namespace mbf_abstract_nav
{


  AbstractControllerExecution::AbstractControllerExecution(
      boost::condition_variable &condition, const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr) :
      condition_(condition), tf_listener_ptr(tf_listener_ptr), state_(STOPPED), moving_(false), outcome_(255)
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // non-dynamically reconfigurable parameters
    private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
    private_nh.param("map_frame", global_frame_, std::string("map"));
    private_nh.param("mbf_tolerance_check", mbf_tolerance_check_, false);
    private_nh.param("dist_tolerance", dist_tolerance_, 0.1);
    private_nh.param("angle_tolerance", angle_tolerance_, M_PI / 18.0);
    private_nh.param("tf_timeout", tf_timeout_, 1.0);

    // init cmd_vel publisher for the robot velocity t
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  }


  AbstractControllerExecution::~AbstractControllerExecution()
  {
  }


  bool AbstractControllerExecution::initialize()
  {
    return loadPlugins();
  }

  bool AbstractControllerExecution::loadPlugins()
  {
    ros::NodeHandle private_nh("~");

    XmlRpc::XmlRpcValue controllers_param_list;
    if(!private_nh.getParam("controllers", controllers_param_list))
    {
      ROS_WARN_STREAM("No controllers configured! - Use the param \"controllers\", which must be a list of tuples with a name and a type.");
      return false;
    }

    try
    {
      for (int i = 0; i < controllers_param_list.size(); i++)
      {
        XmlRpc::XmlRpcValue elem = controllers_param_list[i];

        std::string name = elem["name"];
        std::string type = elem["type"];

        if (controllers_.find(name) != controllers_.end())
        {
          ROS_ERROR_STREAM("The controller \"" << name << "\" has already been loaded! Names must be unique!");
          return false;
        }
        // load and init
        mbf_abstract_core::AbstractController::Ptr controller_ptr = loadControllerPlugin(type);
        if(controller_ptr && initPlugin(name, controller_ptr))
        {
          // set default controller to the first in the list
          if(!controller_)
          {
            controller_ = controller_ptr;
            plugin_name_ = name;
            setState(INITIALIZED);
          }

          controllers_.insert(
              std::pair<std::string, mbf_abstract_core::AbstractController::Ptr>(name, controller_ptr));

          controllers_type_.insert(std::pair<std::string, std::string>(name, type)); // save name to type mapping

          ROS_INFO_STREAM("The controller with the type \"" << type << "\" has been loaded and initialized"
              << " successfully under the name \"" << name << "\".");

        }
        else
        {
          ROS_ERROR_STREAM("Could not load and initialize the plugin with the name \""
              << name << "\" and the type \"" << type << "\"!");
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("Invalid parameter structure. The \"controllers\" parameter has to be a list of structs "
                           << "with fields \"name\" and \"type\" of !");
      ROS_ERROR_STREAM(e.getMessage());
      return false;
    }
    // is there any controller initialized?
    return controller_ ? true : false;
  }

  bool AbstractControllerExecution::switchController(const std::string& name)
  {
    if(name == plugin_name_)
    {
      ROS_DEBUG_STREAM("No controller switch necessary, \"" << name << "\" already set.");
      return true;
    }
    std::map<std::string, mbf_abstract_core::AbstractController::Ptr>::iterator new_controller
        = controllers_.find(name);
    if(new_controller != controllers_.end())
    {
      plugin_name_ = new_controller->first;
      controller_ = new_controller->second;
      new_plan_ = true;  // ensure we reset the current plan (if any) for the new controller
      ROS_INFO_STREAM("Switched the controller plugin to \"" << new_controller->first
          << "\" with the type \"" << controllers_type_[new_controller->first] << "\".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("The controller \"" << name << "\" has not yet been loaded!"
          << " No switch of the controller!");
      return false;
    }
  }

  void AbstractControllerExecution::reconfigure(const MoveBaseFlexConfig &config)
  {
    boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

    // Timeout granted to the local planner. We keep calling it up to this time or up to max_retries times
    // If it doesn't return within time, the navigator will cancel it and abort the corresponding action
    patience_ = ros::Duration(config.controller_patience);

    // set the calling duration by the moving frequency
    if (config.controller_frequency > 0.0)
      calling_duration_ = boost::chrono::microseconds((int)(1e6 / config.controller_frequency));
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
    outcome_ = 255;
    message_ = "";
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

  void AbstractControllerExecution::setNewPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
  {
    if (moving_)
    {
      // This is fine on continuous replanning
      ROS_DEBUG("Setting new plan while moving");
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


  std::vector<geometry_msgs::PoseStamped> AbstractControllerExecution::getNewPlan()
  {
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    new_plan_ = false;
    return plan_;
  }


  bool AbstractControllerExecution::computeRobotPose()
  {
    bool tf_success = mbf_utility::getRobotPose(*tf_listener_ptr, robot_frame_, global_frame_,
                                                     ros::Duration(tf_timeout_), robot_pose_);
    // would be 0 if not, as we ask tf listener for the last pose available
    robot_pose_.header.stamp = ros::Time::now();
    if (!tf_success)
    {
      ROS_ERROR_STREAM("Could not get the robot pose in the global frame. - robot frame: \""
                           << robot_frame_ << "\"   global frame: \"" << global_frame_ << std::endl);
      message_ = "Could not get the robot pose";
      outcome_ = mbf_msgs::ExePathResult::TF_ERROR;
      return false;
    }
    return true;
  }


  uint32_t AbstractControllerExecution::computeVelocityCmd(geometry_msgs::TwistStamped &vel_cmd, std::string& message)
  {
    // TODO compute velocity
    geometry_msgs::TwistStamped robot_velocity;
    return controller_->computeVelocityCommands(robot_pose_, robot_velocity, vel_cmd, message);
  }


  void AbstractControllerExecution::setVelocityCmd(const geometry_msgs::TwistStamped &vel_cmd)
  {
    boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
    vel_cmd_stamped_ = vel_cmd;
    if (vel_cmd_stamped_.header.stamp.isZero())
      vel_cmd_stamped_.header.stamp = ros::Time::now();
  }


  geometry_msgs::TwistStamped AbstractControllerExecution::getLastValidCmdVel()
  {
    boost::lock_guard<boost::mutex> guard(vel_cmd_mtx_);
    return vel_cmd_stamped_;
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
    return moving_ && start_time_ < getLastValidCmdVelTime() && !isPatienceExceeded();
  }

  bool AbstractControllerExecution::reachedGoalCheck()
  {
    // check whether the controller plugin returns goal reached or if mbf should check for goal reached.
    return controller_->isGoalReached(dist_tolerance_, angle_tolerance_) || (mbf_tolerance_check_
        && mbf_utility::distance(robot_pose_, plan_.back()) < dist_tolerance_
        && mbf_utility::angle(robot_pose_, plan_.back()) < angle_tolerance_);
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
          plan = getNewPlan();

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

        // compute robot pose and store it in robot_pose_
        computeRobotPose();

        // ask planner if the goal is reached
        if (reachedGoalCheck())
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
          geometry_msgs::TwistStamped cmd_vel_stamped;
          outcome_ = computeVelocityCmd(cmd_vel_stamped, message_);

          if (outcome_ < 10)
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
            ROS_WARN_THROTTLE(1.0, "Calculation needs too much time to stay in the moving frequency!");
          }
        }
      }
    }
    catch (const boost::thread_interrupted &ex)
    {
      // Controller thread interrupted; in most cases we have started a new plan
      // Can also be that robot is oscillating or we have exceeded planner patience
      ROS_DEBUG_STREAM("Controller thread interrupted!");
      // publishZeroVelocity();  TODO comment this makes sense for continuous replanning
      setState(STOPPED);
      condition_.notify_all();
      moving_ = false;
    }
    catch (...){
      message_ = "Unknown error occurred: " + boost::current_exception_diagnostic_information();
      ROS_FATAL_STREAM(message_);
      setState(INTERNAL_ERROR);
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

} /* namespace mbf_abstract_nav */
