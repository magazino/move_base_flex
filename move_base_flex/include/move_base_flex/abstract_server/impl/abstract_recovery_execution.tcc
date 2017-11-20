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

#ifndef MOVE_BASE_FLEX__IMPL__ABSTRACT_RECOVERY_EXECUTION_TCC_
#define MOVE_BASE_FLEX__IMPL__ABSTRACT_RECOVERY_EXECUTION_TCC_

#include <XmlRpcException.h>

#include "move_base_flex/abstract_server/abstract_recovery_execution.h"

namespace move_base_flex
{

template<class RECOVERY_BEHAVIOR_BASE>
  AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::AbstractRecoveryExecution(
      boost::condition_variable &condition,
      const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
      std::string package, std::string class_name) :
      condition_(condition), tf_listener_ptr_(tf_listener_ptr), state_(STOPPED),
      class_loader_recovery_behaviors_(package, class_name), canceled_(false)
  {
  }

template<class RECOVERY_BEHAVIOR_BASE>
  AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::~AbstractRecoveryExecution()
  {
  }

template<class RECOVERY_BEHAVIOR_BASE>
  void AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::initialize()
  {
    if (!loadPlugins())
    {
      ROS_ERROR_STREAM("Could not load the recovery behaviors!");
      // TODO load default recovery behavior plugins
    }
    else
    {
      if(!recovery_behaviors_.empty())
      {
        ROS_INFO_STREAM("All recovery behavior plugins has been loaded successfully!");
      }
    }

    initPlugins();
    setState(INITIALIZED);
  }

template<class RECOVERY_BEHAVIOR_BASE>
  void AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::reconfigure(move_base_flex::MoveBaseFlexConfig &config)
  {
    boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

    // Disabled on move_base... TODO: try to enable in mbf if turns to be useful
    // config.recovery_behaviors;
  }

template<class RECOVERY_BEHAVIOR_BASE>
  bool AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::loadPlugins()
  {
    ros::NodeHandle private_nh("~");

    XmlRpc::XmlRpcValue recovery_behaviors_param_list;
    if(!private_nh.getParam("recovery_behaviors", recovery_behaviors_param_list)){
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
        try
        {
          recovery_behaviors_.insert(
              std::pair<std::string, boost::shared_ptr<RECOVERY_BEHAVIOR_BASE> >(
                  name, class_loader_recovery_behaviors_.createInstance(type)));

          recovery_behaviors_type_.insert(std::pair<std::string, std::string>(name, type)); // save name to type mapping

          ROS_INFO_STREAM("The recovery behavior \"" << type << "\" has been loaded successfully under the name \""
                          << name << "\".");
        }
        catch (pluginlib::LibraryLoadException &e)
        {
          ROS_ERROR_STREAM("Could not load the library of the specified behavior \"" << name << "\" - \""
                           << type << "\"!");
          ROS_ERROR_STREAM(e.what());
          return false;
        }
        catch (pluginlib::CreateClassException &e)
        {
          ROS_ERROR_STREAM("Could not create the class of the specified behavior \"" << name << "\" - \""
                           << type << "\"!");
          ROS_ERROR_STREAM(e.what());
          return false;
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
    return true;
  }

template<class RECOVERY_BEHAVIOR_BASE>
  void AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::setState(RecoveryState state)
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    state_ = state;
  }

template<class RECOVERY_BEHAVIOR_BASE>
  typename AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::RecoveryState AbstractRecoveryExecution<
      RECOVERY_BEHAVIOR_BASE>::getState()
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    return state_;
  }

template<class RECOVERY_BEHAVIOR_BASE>
  void AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::startRecovery(const std::string name)
  {
    requested_behavior_name_ = name;
    setState(STARTED);
    thread_ = boost::thread(&AbstractRecoveryExecution::run, this);
  }

template<class RECOVERY_BEHAVIOR_BASE>
  void AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::stopRecovery()
  {
    thread_.interrupt();
    setState(STOPPED);
  }

template<class RECOVERY_BEHAVIOR_BASE>
  bool AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::cancel()
  {
    canceled_ = true;
    if (current_behavior_)
    {
      // returns false if cancel is not implemented or rejected by the recovery behavior (will run until completion)
      return current_behavior_->cancel();
    }
    return false;
  }

template<class RECOVERY_BEHAVIOR_BASE>
  bool AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::hasRecoveryBehavior(const std::string &name)
  {
    return recovery_behaviors_.find(name) != recovery_behaviors_.end();
  }

template<class RECOVERY_BEHAVIOR_BASE>
  std::vector<std::string> AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::listRecoveryBehaviors()
  {
    std::vector<std::string> recovery_behaviors;
    std::map<std::string, std::string>::iterator it = recovery_behaviors_type_.begin();
    for (; it != recovery_behaviors_type_.end(); ++it)
    {
      recovery_behaviors.push_back(it->first);
    }
    return recovery_behaviors;
  }

template<class RECOVERY_BEHAVIOR_BASE>
  bool AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::getTypeOfBehavior(const std::string &name, std::string &type)
  {
    std::map<std::string, std::string>::iterator finder = recovery_behaviors_type_.find(name);
    if (finder != recovery_behaviors_type_.end())
    {
      type = finder->second;
      return true;
    }
    return false;
  }

template<class RECOVERY_BEHAVIOR_BASE>
  void AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::run()
  {
    boost::recursive_mutex::scoped_lock sl(configuration_mutex_);
    canceled_ = false; // (re)set the canceled state

    typename std::map<std::string, boost::shared_ptr<RECOVERY_BEHAVIOR_BASE> >::iterator find_iter;
    find_iter = recovery_behaviors_.find(requested_behavior_name_);

    if (find_iter == recovery_behaviors_.end())
    {
      // no such recovery behavior
      ROS_ERROR_STREAM("No recovery behavior for the given name: \"" << requested_behavior_name_ << "\"!");
      setState(WRONG_NAME);
      condition_.notify_one();
      return;
    }

    current_behavior_ = find_iter->second;
    setState(RECOVERING);
    try
    {
      current_behavior_->runBehavior();
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
    condition_.notify_one();
    current_behavior_.reset();
  }
} /* namespace move_base_flex */

#endif /* MOVE_BASE_FLEX__IMPL__ABSTRACT_RECOVERY_EXECUTION_TCC_ */
