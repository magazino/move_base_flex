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
      mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr,
      const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr) :
      behavior_(recovery_ptr), tf_listener_ptr_(tf_listener_ptr), state_(INITIALIZED), canceled_(false)
  {
  }

  AbstractRecoveryExecution::~AbstractRecoveryExecution()
  {
  }


  void AbstractRecoveryExecution::reconfigure(const MoveBaseFlexConfig &config)
  {
    boost::lock_guard<boost::mutex> guard(conf_mtx_);

    // Maximum time allowed to recovery behaviors. Intended as a safeward for the case a behavior hangs.
    // If it doesn't return within time, the navigator will cancel it and abort the corresponding action.
    patience_ = ros::Duration(config.recovery_patience);

    // Nothing else to do here, as recovery_enabled is loaded and used in the navigation server
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


  void AbstractRecoveryExecution::startRecovery()
  {
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
    // returns false if cancel is not implemented or rejected by the recovery behavior (will run until completion)
    return behavior_->cancel();
  }

  bool AbstractRecoveryExecution::isPatienceExceeded()
  {
    boost::lock_guard<boost::mutex> guard1(conf_mtx_);
    boost::lock_guard<boost::mutex> guard2(time_mtx_);
    ROS_INFO_STREAM("Patience: " << patience_ << ", Start Time: " << start_time_ << " now: " << ros::Time::now());
    return (patience_ > ros::Duration(0)) && (ros::Time::now() - start_time_ > patience_);
  }

  void AbstractRecoveryExecution::run()
  {
    canceled_ = false; // (re)set the canceled state

    time_mtx_.lock();
    start_time_ = ros::Time::now();
    time_mtx_.unlock();
    setState(RECOVERING);
    try
    {
      // TODO use outcome and message
      std::string message;
      uint32_t outcome = behavior_->runBehavior(message);
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
  }
} /* namespace mbf_abstract_nav */
