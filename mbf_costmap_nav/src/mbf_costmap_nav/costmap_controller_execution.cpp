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
 *  move_base_controller_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */
#include <nav_core_wrapper/wrapper_local_planner.h>
#include "mbf_costmap_nav/costmap_controller_execution.h"

namespace move_base_flex
{

MoveBaseControllerExecution::MoveBaseControllerExecution(
    boost::condition_variable &condition, const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
    CostmapPtr &costmap_ptr) :
    AbstractControllerExecution(condition, tf_listener_ptr),
    costmap_ptr_(costmap_ptr)
{
}

MoveBaseControllerExecution::~MoveBaseControllerExecution()
{
}

mbf_core::AbstractController::Ptr MoveBaseControllerExecution::loadControllerPlugin(const std::string& controller_type)
{
  static pluginlib::ClassLoader<mbf_core::MoveBaseController>
      class_loader("mbf_core", "mbf_core::MoveBaseController");
  mbf_core::AbstractController::Ptr controller_ptr;
  // try to load and init local planner
  ROS_DEBUG("Load local planner plugin.");
  try
  {
    controller_ptr = class_loader.createInstance(controller_type);
    controller_name_ = class_loader.getName(controller_type);
    ROS_INFO_STREAM("MBF_core-based local planner plugin " << controller_name_ << " loaded");
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    ROS_INFO_STREAM("Failed to load the " << controller_type << " local planner as a mbf_core-based plugin;"
                     << "  we will retry to load as a nav_core-based plugin. Exception: " << ex.what());
    try
    {
      // For plugins still based on old nav_core API, we load them and pass to a new MBF API that will act as wrapper
      static pluginlib::ClassLoader<nav_core::BaseLocalPlanner>
          nav_core_class_loader("nav_core", "nav_core::BaseLocalPlanner");
      boost::shared_ptr<nav_core::BaseLocalPlanner> nav_core_controller_ptr
          = nav_core_class_loader.createInstance(controller_type);
      controller_ptr = boost::make_shared<mbf_core::WrapperLocalPlanner>(nav_core_controller_ptr);
      controller_name_ = nav_core_class_loader.getName(controller_type);
      ROS_INFO_STREAM("Nav_core-based local planner plugin " << controller_name_ << " loaded");
    }
    catch (const pluginlib::PluginlibException &ex)
    {
      ROS_FATAL_STREAM("Failed to load the " << controller_type
          << " local planner, are you sure it's properly registered"
          << " and that the containing library is built? Exception: " << ex.what());
    }
  }
  return controller_ptr;
}

void MoveBaseControllerExecution::initPlugin()
{
  ROS_INFO_STREAM("Initialize controller \"" << controller_name_ << "\".");

  if (!tf_listener_ptr)
  {
    ROS_ERROR_STREAM("The tf listener pointer has not been initialized!");
    exit(1);
  }

  if (!costmap_ptr_)
  {
    ROS_ERROR_STREAM("The costmap pointer has not been initialized!");
    exit(1);
  }

  // TODO check this
  ros::NodeHandle private_nh("~");
  private_nh.param("controller_lock_costmap", lock_costmap_, true);

  mbf_core::MoveBaseController::Ptr controller_ptr
      = boost::static_pointer_cast<mbf_core::MoveBaseController>(controller_);
  controller_ptr->initialize(controller_name_, tf_listener_ptr.get(), costmap_ptr_.get());
  ROS_INFO_STREAM("Controller plugin \"" << controller_name_ << "\" initialized.");
}

uint32_t MoveBaseControllerExecution::computeVelocityCmd(geometry_msgs::TwistStamped& vel_cmd, std::string& message)
{
  // Lock the costmap while planning, but following issue #4, we allow to move the responsibility to the planner itself
  if (lock_costmap_)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr_->getCostmap()->getMutex()));
    return controller_->computeVelocityCommands(vel_cmd, message);
  }
  return controller_->computeVelocityCommands(vel_cmd, message);
}

} /* namespace move_base_nav_moving */
