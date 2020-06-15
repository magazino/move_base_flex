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
 *  abstract_recovery_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <boost/exception/diagnostic_information.hpp>

#include <mbf_abstract_nav/abstract_recovery_execution.h>

namespace mbf_abstract_nav
{

AbstractRecoveryExecution::AbstractRecoveryExecution(
    const std::string &name,
    const mbf_abstract_core::AbstractRecovery::Ptr &recovery_ptr,
    const TFPtr &tf_listener_ptr,
    const MoveBaseFlexConfig &config) :
  AbstractExecutionBase(name),
    behavior_(recovery_ptr), tf_listener_ptr_(tf_listener_ptr), state_(INITIALIZED)
{
  // dynamically reconfigurable parameters
  reconfigure(config);
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

bool AbstractRecoveryExecution::cancel()
{
  cancel_ = true;
  // returns false if cancel is not implemented or rejected by the recovery behavior (will run until completion)
  if (!behavior_->cancel())
  {
    ROS_WARN_STREAM("Cancel recovery behavior \"" << name_ << "\" failed or is not supported by the plugin. "
                        << "Wait until the current recovery behavior finished!");
    return false;
  }
  return true;
}

bool AbstractRecoveryExecution::isPatienceExceeded()
{
  boost::lock_guard<boost::mutex> guard1(conf_mtx_);
  boost::lock_guard<boost::mutex> guard2(time_mtx_);
  ROS_DEBUG_STREAM("Patience: " << patience_ << ", start time: " << start_time_ << " now: " << ros::Time::now());
  return !patience_.isZero() && (ros::Time::now() - start_time_ > patience_);
}

void AbstractRecoveryExecution::run()
{
  cancel_ = false; // reset the canceled state

  time_mtx_.lock();
  start_time_ = ros::Time::now();
  time_mtx_.unlock();
  setState(RECOVERING);
  try
  {
    outcome_ = behavior_->runBehavior(message_);
    if (cancel_)
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
    ROS_WARN_STREAM("Recovery \"" << name_ << "\" interrupted!");
    setState(STOPPED);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("Unknown error occurred in recovery behavior \"" << name_ << "\": " << boost::current_exception_diagnostic_information());
    setState(INTERNAL_ERROR);
  }
  condition_.notify_one();
}

} /* namespace mbf_abstract_nav */
