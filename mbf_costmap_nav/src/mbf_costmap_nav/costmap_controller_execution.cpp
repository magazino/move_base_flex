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
 *  costmap_controller_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */
#include "mbf_costmap_nav/costmap_controller_execution.h"

namespace mbf_costmap_nav
{

CostmapControllerExecution::CostmapControllerExecution(
    const std::string &controller_name,
    const mbf_costmap_core::CostmapController::Ptr &controller_ptr,
    const mbf_utility::RobotInformation &robot_info,
    const ros::Publisher &vel_pub,
    const ros::Publisher &goal_pub,
    const CostmapWrapper::Ptr &costmap_ptr,
    const MoveBaseFlexConfig &config)
      : AbstractControllerExecution(controller_name, controller_ptr, robot_info, vel_pub, goal_pub, toAbstract(config)),
        costmap_ptr_(costmap_ptr)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("controller_lock_costmap", lock_costmap_, true);
}

CostmapControllerExecution::~CostmapControllerExecution()
{
}

mbf_abstract_nav::MoveBaseFlexConfig CostmapControllerExecution::toAbstract(const MoveBaseFlexConfig &config)
{
  // copy the controller-related abstract configuration common to all MBF-based navigation
  mbf_abstract_nav::MoveBaseFlexConfig abstract_config;
  abstract_config.controller_frequency = config.controller_frequency;
  abstract_config.controller_patience = config.controller_patience;
  abstract_config.controller_max_retries = config.controller_max_retries;
  abstract_config.oscillation_timeout = config.oscillation_timeout;
  abstract_config.oscillation_distance = config.oscillation_distance;
  return abstract_config;
}

uint32_t CostmapControllerExecution::computeVelocityCmd(
    const geometry_msgs::PoseStamped &robot_pose,
    const geometry_msgs::TwistStamped &robot_velocity,
    geometry_msgs::TwistStamped &vel_cmd,
    std::string &message)
{
  // Lock the costmap while planning, but following issue #4, we allow to move the responsibility to the planner itself
  if (lock_costmap_)
  {
    boost::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr_->getCostmap()->getMutex()));
    return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
  }
  return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
}

bool CostmapControllerExecution::safetyCheck()
{
  // Check that the observation buffers for the costmap are current, we don't want to drive blind
  if (!costmap_ptr_->isCurrent())
  {
    ROS_WARN("Sensor data is out of date, we're not going to allow commanding of the base for safety");
    return false;
  }
  return true;
}

} /* namespace mbf_costmap_nav */
