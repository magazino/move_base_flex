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
 *  wrapper_local_planner.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "nav_core_wrapper/wrapper_local_planner.h"

namespace mbf_nav_core_wrapper
{

uint32_t WrapperLocalPlanner::computeVelocityCommands(
    const geometry_msgs::PoseStamped &robot_pose,
    const geometry_msgs::TwistStamped &robot_velocity,
    geometry_msgs::TwistStamped &cmd_vel,
    std::string &message)
{
  bool success = nav_core_plugin_->computeVelocityCommands(cmd_vel.twist);
  message = success ? "Valid command" : "Controller failed";
  return success ? 0 : 100;  // SUCCESS | FAILURE
}

bool WrapperLocalPlanner::isGoalReached()
{
  return nav_core_plugin_->isGoalReached();
}

bool WrapperLocalPlanner::isGoalReached(double xy_tolerance, double yaw_tolerance)
{
  return isGoalReached();
}

bool WrapperLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
  return nav_core_plugin_->setPlan(plan);
}

bool WrapperLocalPlanner::cancel()
{
  ROS_WARN_STREAM("The cancel method is not implemented. "
                  "Note: you are running a nav_core based plugin, "
                  "which is wrapped into the MBF interface.");
  return false;
}

void WrapperLocalPlanner::initialize(std::string name,
                                     TF *tf,
                                     costmap_2d::Costmap2DROS *costmap_ros)
{
  nav_core_plugin_->initialize(name, tf, costmap_ros);
}

WrapperLocalPlanner::WrapperLocalPlanner(boost::shared_ptr<nav_core::BaseLocalPlanner> plugin)
    : nav_core_plugin_(plugin)
{}

WrapperLocalPlanner::~WrapperLocalPlanner()
{}

};  /* namespace mbf_abstract_core */
