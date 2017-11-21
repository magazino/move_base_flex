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

#include "move_base_flex/move_base_server/move_base_recovery_execution.h"

namespace move_base_flex
{

MoveBaseRecoveryExecution::MoveBaseRecoveryExecution(boost::condition_variable &condition,
                                                     const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
                                                     CostmapPtr &global_costmap, CostmapPtr &local_costmap) :
    AbstractRecoveryExecution(condition, tf_listener_ptr, "move_base_flex_core", "move_base_flex_core::MoveBaseRecovery"),
    global_costmap_(global_costmap), local_costmap_(local_costmap)
{
}

MoveBaseRecoveryExecution::~MoveBaseRecoveryExecution()
{
}

bool MoveBaseRecoveryExecution::loadPlugins()
{
  ros::NodeHandle private_nh("~");

  XmlRpc::XmlRpcValue recovery_behaviors_param_list;
  if(!private_nh.getParam("recovery_behaviors", recovery_behaviors_param_list)){
    ROS_WARN_STREAM("No recovery behaviors configured! - Use the param \"recovery_behaviors\", which must be a list of tuples with a name and a type.");
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
            std::pair<std::string, boost::shared_ptr<move_base_flex_core::MoveBaseRecovery> >(
                name, class_loader_recovery_behaviors_.createInstance(type)));

        recovery_behaviors_type_.insert(std::pair<std::string, std::string>(name, type)); // save name to type mapping

        ROS_INFO_STREAM("move_base_flex_core-based recovery behavior \"" << type << "\" successfully loaded with name \""
                        << name << "\".");
      }
      catch (pluginlib::PluginlibException &ex)
      {
        ROS_DEBUG_STREAM("Failed to load the " << name << " recovery behavior as a mbf_core-based plugin;"
                         << " Retry to load as a nav_core-based plugin. Exception: " << ex.what());
        try
        {
          // For plugins still based on old nav_core API, we load them and pass to a new MBF API that will act as wrapper
          static pluginlib::ClassLoader<nav_core::RecoveryBehavior> class_loader("nav_core", "nav_core::RecoveryBehavior");
          boost::shared_ptr<nav_core::RecoveryBehavior> plugin = class_loader.createInstance(type);

          recovery_behaviors_.insert(
              std::pair<std::string, boost::shared_ptr<move_base_flex_core::MoveBaseRecovery> >(
                  name, boost::make_shared<move_base_flex_core::MoveBaseRecovery>(plugin)));

          recovery_behaviors_type_.insert(std::pair<std::string, std::string>(name, type)); // save name to type mapping

          ROS_INFO_STREAM("nav_core-based recovery behavior \"" << type << "\" successfully loaded with name \""
                          << name << "\".");
        }
        catch (const pluginlib::PluginlibException &ex)
        {
          ROS_FATAL_STREAM("Failed to load the " << name << " recovery behavior, are you sure it's properly registered"
                           << " and that the containing library is built? Exception: " << ex.what());
          return false;
        }
      }
    }
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR_STREAM("Invalid parameter structure. The recovery_behaviors parameter has to be a list of structs "
                     << "with fields \"name\" and \"type\" of the recovery behavior!");
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }
  return true;
}

void MoveBaseRecoveryExecution::initPlugins()
{
  for (std::map<std::string, move_base_flex_core::MoveBaseRecovery::Ptr>::iterator iter =
      recovery_behaviors_.begin(); iter != recovery_behaviors_.end(); ++iter)
  {
    move_base_flex_core::MoveBaseRecovery::Ptr behavior = iter->second;
    std::string name = iter->first;

    behavior->initialize(name, tf_listener_ptr_.get(), global_costmap_.get(), local_costmap_.get());
  }
}

} /* namespace move_base_flex */
