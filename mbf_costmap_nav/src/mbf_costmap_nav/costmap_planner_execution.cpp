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
#include <nav_core_wrapper/wrapper_global_planner.h>

#include "mbf_costmap_nav/costmap_planner_execution.h"

namespace mbf_costmap_nav
{

CostmapPlannerExecution::CostmapPlannerExecution(
    const mbf_costmap_core::CostmapPlanner::Ptr &planner_ptr,
    CostmapPtr &costmap_ptr,
    const MoveBaseFlexConfig &config,
    boost::function<void()> setup_fn,
    boost::function<void()> cleanup_fn)
      : AbstractPlannerExecution(planner_ptr, toAbstract(config), setup_fn, cleanup_fn),
        costmap_ptr_(costmap_ptr)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("planner_lock_costmap", lock_costmap_, true);
}

CostmapPlannerExecution::~CostmapPlannerExecution()
{
}

mbf_abstract_nav::MoveBaseFlexConfig CostmapPlannerExecution::toAbstract(const MoveBaseFlexConfig &config)
{
  // copy the planner-related abstract configuration common to all MBF-based navigation
  mbf_abstract_nav::MoveBaseFlexConfig abstract_config;
  abstract_config.planner_frequency = config.planner_frequency;
  abstract_config.planner_patience = config.planner_patience;
  abstract_config.planner_max_retries = config.planner_max_retries;
  return abstract_config;
}

uint32_t CostmapPlannerExecution::makePlan(const geometry_msgs::PoseStamped& start,
                                           const geometry_msgs::PoseStamped& goal,
                                           double tolerance,
                                           std::vector<geometry_msgs::PoseStamped> &plan,
                                           double &cost,
                                           std::string &message)
{
  if (lock_costmap_)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr_->getCostmap()->getMutex()));
    return planner_->makePlan(start, goal, tolerance, plan, cost, message);
  }
  return planner_->makePlan(start, goal, tolerance, plan, cost, message);
}

} /* namespace mbf_costmap_nav */
