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
 *  simple_recovery_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "mbf_simple_nav/simple_recovery_execution.h"
#include <pluginlib/class_loader.h>

namespace mbf_simple_nav
{

SimpleRecoveryExecution::SimpleRecoveryExecution(boost::condition_variable &condition,
                                                 const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr) :
    mbf_abstract_nav::AbstractRecoveryExecution(condition, tf_listener_ptr)
{
}

SimpleRecoveryExecution::~SimpleRecoveryExecution()
{
}


mbf_abstract_core::AbstractRecovery::Ptr SimpleRecoveryExecution::loadRecoveryPlugin(
    const std::string& recovery_type)
{
  static pluginlib::ClassLoader<mbf_abstract_core::AbstractRecovery>
      class_loader("mbf_abstract_core", "mbf_abstract_core::AbstractRecovery");
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr;

  try
  {
    recovery_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractRecovery>(
        class_loader.createInstance(recovery_type));
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_FATAL_STREAM("Failed to load the " << recovery_type << " recovery behavior, are you sure it's properly registered"
        << " and that the containing library is built? Exception: " << ex.what());
  }
  return recovery_ptr;
}

bool SimpleRecoveryExecution::initPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr)
{
  return true;
}

} /* namespace mbf_simple_nav */
