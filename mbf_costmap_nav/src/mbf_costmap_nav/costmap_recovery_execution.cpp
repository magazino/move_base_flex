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
 *  move_base_recovery_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */
#include <nav_core/recovery_behavior.h>
#include <xmlrpcpp/XmlRpc.h>
#include "nav_core_wrapper/wrapper_recovery_behavior.h"
#include "mbf_costmap_nav/costmap_recovery_execution.h"

namespace mbf_costmap_nav
{

CostmapRecoveryExecution::CostmapRecoveryExecution(boost::condition_variable &condition,
                                                     const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
                                                     CostmapPtr &global_costmap, CostmapPtr &local_costmap) :
    AbstractRecoveryExecution(condition, tf_listener_ptr),
    global_costmap_(global_costmap), local_costmap_(local_costmap)
{
}

CostmapRecoveryExecution::~CostmapRecoveryExecution()
{
}

mbf_abstract_core::AbstractRecovery::Ptr CostmapRecoveryExecution::loadRecoveryPlugin(
    const std::string& recovery_type)
{
  static pluginlib::ClassLoader<mbf_costmap_core::CostmapRecovery>
      class_loader("mbf_costmap_core", "mbf_costmap_core::CostmapRecovery");
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr;

  try
  {
    recovery_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractRecovery>(
        class_loader.createInstance(recovery_type));
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_DEBUG_STREAM("Failed to load the " << recovery_type << " recovery behavior as a mbf_abstract_core-based plugin;"
                                           << " Retry to load as a nav_core-based plugin. Exception: " << ex.what());
    try
    {
      // For plugins still based on old nav_core API, we load them and pass to a new MBF API that will act as wrapper
      static pluginlib::ClassLoader<nav_core::RecoveryBehavior> nav_core_class_loader(
          "nav_core", "nav_core::RecoveryBehavior");
      boost::shared_ptr<nav_core::RecoveryBehavior> nav_core_recovery_ptr =
          nav_core_class_loader.createInstance(recovery_type);

      recovery_ptr = boost::make_shared<mbf_nav_core_wrapper::WrapperRecoveryBehavior>(nav_core_recovery_ptr);

    }
    catch (const pluginlib::PluginlibException &ex)
    {
      ROS_FATAL_STREAM("Failed to load the " << recovery_type << " recovery behavior, are you sure it's properly registered"
                                             << " and that the containing library is built? Exception: " << ex.what());
    }
  }

  return recovery_ptr;
}

bool CostmapRecoveryExecution::initPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr)
{
  mbf_costmap_core::CostmapRecovery::Ptr behavior =
      boost::static_pointer_cast<mbf_costmap_core::CostmapRecovery>(behavior_ptr);
  behavior->initialize(name, tf_listener_ptr_.get(), global_costmap_.get(), local_costmap_.get());
  return true;
}

} /* namespace mbf_costmap_nav */
