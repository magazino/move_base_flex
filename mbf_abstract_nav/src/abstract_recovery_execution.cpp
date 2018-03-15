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
 *  abstract_recovery_execution.tcc
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <XmlRpcException.h>
#include <boost/exception/diagnostic_information.hpp>

#include <mbf_abstract_nav/abstract_recovery_execution.h>

namespace mbf_abstract_nav
{


  AbstractRecoveryExecution::AbstractRecoveryExecution(
      boost::condition_variable &condition,
      const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr) :
      condition_(condition), tf_listener_ptr_(tf_listener_ptr), state_(STOPPED), canceled_(false)
  {
  }

  AbstractRecoveryExecution::~AbstractRecoveryExecution()
  {
  }


  bool AbstractRecoveryExecution::initialize()
  {
    return loadPlugins();
  }


  void AbstractRecoveryExecution::reconfigure(const MoveBaseFlexConfig &config)
  {
    boost::lock_guard<boost::mutex> guard(conf_mtx_);

    // Maximum time allowed to recovery behaviors. Intended as a safeward for the case a behavior hangs.
    // If it doesn't return within time, the navigator will cancel it and abort the corresponding action.
    patience_ = ros::Duration(config.recovery_patience);

    // Nothing else to do here, as recovery_enabled is loaded and used in the navigation server
  }


  bool AbstractRecoveryExecution::loadPlugins()
  {
    ros::NodeHandle private_nh("~");

    XmlRpc::XmlRpcValue recovery_behaviors_param_list;
    if(!private_nh.getParam("recovery_behaviors", recovery_behaviors_param_list))
    {
      ROS_WARN_STREAM("No recovery bahaviors configured! - Use the param \"recovery_behaviors\", which must be a list of tuples with a name and a type.");
      return true;
    }

    try
    {
      for (int i = 0; i < recovery_behaviors_param_list.size(); i++)
      {
        XmlRpc::XmlRpcValue elem = recovery_behaviors_param_list[i];

        std::string name = elem["name"];
        std::string type = elem["type"];

        if (recovery_behaviors_.find(name) != recovery_behaviors_.end())
        {
          ROS_ERROR_STREAM("The recovery behavior \"" << name << "\" has already been loaded! Names must be unique!");
          return false;
        }
        mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr = loadRecoveryPlugin(type);
        if(recovery_ptr && initPlugin(name, recovery_ptr))
        {
          if(!current_behavior_)
          {
            current_behavior_ = recovery_ptr;
            setState(INITIALIZED);
          }
          recovery_behaviors_.insert(
              std::pair<std::string, mbf_abstract_core::AbstractRecovery::Ptr>(name, recovery_ptr));

          recovery_behaviors_type_.insert(std::pair<std::string, std::string>(name, type)); // save name to type mapping

          ROS_INFO_STREAM("The recovery behavior \"" << type << "\" has been loaded successfully under the name \""
                          << name << "\".");
        }
        else
        {
          ROS_ERROR_STREAM("Could not load the plugin with the name \"" << name << "\" and the type \"" << type << "\"!");
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("Invalid parameter structure. The recovery_behaviors parameter has to be a list of structs "
                       << "with fields \"name\" and \"type\" of the recovery behavior!");
      ROS_ERROR_STREAM(e.getMessage());
      return false;
    }
    // Is there any recovery behavior initialized?
    return current_behavior_ ? true : false;
  }


  void AbstractRecoveryExecution::setState(RecoveryState state)
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    state_ = state;
  }


  typename AbstractRecoveryExecution::RecoveryState AbstractRecoveryExecution::getState()
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    return state_;
  }


  void AbstractRecoveryExecution::startRecovery(const std::string &name)
  {
    requested_behavior_name_ = name;
    setState(STARTED);
    thread_ = boost::thread(&AbstractRecoveryExecution::run, this);
  }


  void AbstractRecoveryExecution::stopRecovery()
  {
    thread_.interrupt();
    setState(STOPPED);
  }


  bool AbstractRecoveryExecution::cancel()
  {
    canceled_ = true;
    if (current_behavior_)
    {
      // returns false if cancel is not implemented or rejected by the recovery behavior (will run until completion)
      return current_behavior_->cancel();
    }
    return false;
  }


  bool AbstractRecoveryExecution::hasRecoveryBehavior(const std::string &name)
  {
    return recovery_behaviors_.find(name) != recovery_behaviors_.end();
  }


  std::vector<std::string> AbstractRecoveryExecution::listRecoveryBehaviors()
  {
    std::vector<std::string> recovery_behaviors;
    std::map<std::string, std::string>::iterator it = recovery_behaviors_type_.begin();
    for (; it != recovery_behaviors_type_.end(); ++it)
    {
      recovery_behaviors.push_back(it->first);
    }
    return recovery_behaviors;
  }


  bool AbstractRecoveryExecution::getTypeOfBehavior(const std::string &name, std::string &type)
  {
    std::map<std::string, std::string>::iterator finder = recovery_behaviors_type_.find(name);
    if (finder != recovery_behaviors_type_.end())
    {
      type = finder->second;
      return true;
    }
    return false;
  }

  bool AbstractRecoveryExecution::isPatienceExceeded()
  {
    boost::lock_guard<boost::mutex> guard1(conf_mtx_);
    boost::lock_guard<boost::mutex> guard2(time_mtx_);
    return (patience_ > ros::Duration(0)) && (ros::Time::now() - start_time_ > patience_);
  }

  void AbstractRecoveryExecution::run()
  {
    canceled_ = false; // (re)set the canceled state

    typename std::map<std::string, boost::shared_ptr<mbf_abstract_core::AbstractRecovery> >::iterator find_iter;
    find_iter = recovery_behaviors_.find(requested_behavior_name_);

    if (find_iter == recovery_behaviors_.end())
    {
      // no such recovery behavior
      ROS_ERROR_STREAM("No recovery behavior for the given name: \"" << requested_behavior_name_ << "\"!");
      setState(WRONG_NAME);
      condition_.notify_one();
      return;
    }

    time_mtx_.lock();
    start_time_ = ros::Time::now();
    time_mtx_.unlock();
    current_behavior_ = find_iter->second;
    setState(RECOVERING);
    try
    {
      // TODO use outcome and message
      std::string message;
      uint32_t outcome = current_behavior_->runBehavior(message);
      if (canceled_)
      {
        setState(CANCELED);
      }
      else
      {
        setState(RECOVERY_DONE);
      }
    }
    catch (boost::thread_interrupted &ex)
    {
      setState(STOPPED);
    }
    catch (...){
      ROS_FATAL_STREAM("Unknown error occurred: " << boost::current_exception_diagnostic_information());
      setState(INTERNAL_ERROR);
    }
    condition_.notify_one();
    current_behavior_.reset();
  }
} /* namespace mbf_abstract_nav */
