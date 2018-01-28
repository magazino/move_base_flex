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
 *  move_base_planner_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */
#include <nav_core/base_global_planner.h>
#include <nav_core_wrapper/wrapper_global_planner.h>

#include "mbf_costmap_nav/costmap_planner_execution.h"

namespace mbf_costmap_nav
{

CostmapPlannerExecution::CostmapPlannerExecution(boost::condition_variable &condition, CostmapPtr &costmap_ptr) :
    AbstractPlannerExecution(condition), costmap_ptr_(costmap_ptr)
{
}

CostmapPlannerExecution::~CostmapPlannerExecution()
{
}

mbf_abstract_core::AbstractPlanner::Ptr CostmapPlannerExecution::loadPlannerPlugin(const std::string& planner_type)
{
  static pluginlib::ClassLoader<mbf_costmap_core::CostmapPlanner>
      class_loader("mbf_costmap_core", "mbf_costmap_core::CostmapPlanner");
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr;
  // try to load and init global planner
  ROS_DEBUG("Load global planner plugin.");
  try
  {
    planner_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractPlanner>(
        class_loader.createInstance(planner_type));
    planner_name_ = class_loader.getName(planner_type);
    ROS_INFO_STREAM("MBF_core-based global planner plugin " << planner_name_ << " loaded");
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    ROS_INFO_STREAM("Failed to load the " << planner_type << " planner as a mbf_abstract_core-based plugin."
                     << " Try to load as a nav_core-based plugin. Exception: " << ex.what());
    try
    {
      // For plugins still based on old nav_core API, we load them and pass to a new MBF API that will act as wrapper
      static pluginlib::ClassLoader<nav_core::BaseGlobalPlanner>
          nav_core_class_loader("nav_core", "nav_core::BaseGlobalPlanner");
      boost::shared_ptr<nav_core::BaseGlobalPlanner> nav_core_planner_ptr = nav_core_class_loader.createInstance(planner_type);
      planner_ptr = boost::make_shared<mbf_nav_core_wrapper::WrapperGlobalPlanner>(nav_core_planner_ptr);
      planner_name_ = nav_core_class_loader.getName(planner_type);
      ROS_INFO_STREAM("Nav_core-based global planner plugin " << planner_name_ << " loaded");
    }
    catch (const pluginlib::PluginlibException &ex)
    {
      ROS_FATAL_STREAM("Failed to load the " << planner_type << " planner, are you sure it's properly registered"
                       << " and that the containing library is built? Exception: " << ex.what());
    }
  }

  return planner_ptr;
}

bool CostmapPlannerExecution::initPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr
)
{
  mbf_costmap_core::CostmapPlanner::Ptr costmap_planner_ptr
      = boost::static_pointer_cast<mbf_costmap_core::CostmapPlanner>(planner_ptr);
  ROS_INFO_STREAM("Initialize planner \"" << name << "\".");

  if (!costmap_ptr_)
  {
    ROS_ERROR_STREAM("The costmap pointer has not been initialized!");
    return false;
  }

  costmap_planner_ptr->initialize(name, costmap_ptr_.get());
  ROS_INFO("Global planner plugin initialized.");
  return true;
}

uint32_t CostmapPlannerExecution::makePlan(const mbf_abstract_core::AbstractPlanner::Ptr &planner_ptr,
                                           const geometry_msgs::PoseStamped start,
                                           const geometry_msgs::PoseStamped goal,
                                           double tolerance,
                                           std::vector<geometry_msgs::PoseStamped> &plan,
                                           double &cost,
                                           std::string &message)
{

  ros::NodeHandle private_nh("~");
  private_nh.param("planner_lock_costmap", lock_costmap_, true);

  if (lock_costmap_)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr_->getCostmap()->getMutex()));
    return planner_ptr->makePlan(start, goal, tolerance, plan, cost, message);
  }
  return planner_ptr->makePlan(start, goal, tolerance, plan, cost, message);
}

} /* namespace mbf_costmap_nav */
