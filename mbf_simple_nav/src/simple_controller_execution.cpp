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
 *  simple_controller_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "mbf_simple_nav/simple_controller_execution.h"
#include <pluginlib/class_loader.h>

namespace mbf_simple_nav
{

SimpleControllerExecution::SimpleControllerExecution(boost::condition_variable &condition,
                                                     const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr) :
    mbf_abstract_nav::AbstractControllerExecution(condition, tf_listener_ptr)
{
}

mbf_abstract_core::AbstractController::Ptr SimpleControllerExecution::loadControllerPlugin(
    const std::string& controller_type)
{
  static pluginlib::ClassLoader<mbf_abstract_core::AbstractController>
      class_loader("mbf_abstract_core", "mbf_abstract_core::AbstractController");
  mbf_abstract_core::AbstractController::Ptr controller_ptr;
  ROS_DEBUG("Load controller plugin.");
  try
  {
    controller_ptr = class_loader.createInstance(controller_type);
    ROS_INFO_STREAM("MBF_core-based local planner plugin " << controller_type << " loaded");
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    ROS_FATAL_STREAM("Failed to load the " << controller_type
        << " local planner, are you sure it's properly registered"
        << " and that the containing library is built? Exception: " << ex.what());
  }
  return controller_ptr;
}

bool SimpleControllerExecution::initPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractController::Ptr& controller_ptr)
{
  return true;
}

SimpleControllerExecution::~SimpleControllerExecution()
{
}

} /* namespace move_base_nav_moving */
