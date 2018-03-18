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
 *  move_base_navigation_server.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/footprint_helper.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_abstract_nav/MoveBaseFlexConfig.h>
#include <actionlib/client/simple_action_client.h>

#include "mbf_costmap_nav/costmap_navigation_server.h"

namespace mbf_costmap_nav
{


CostmapNavigationServer::CostmapNavigationServer(const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr) :
  AbstractNavigationServer(tf_listener_ptr,
                           CostmapPlannerExecution::Ptr(
                                new CostmapPlannerExecution(condition_, global_costmap_ptr_)),
                           CostmapControllerExecution::Ptr(
                                new CostmapControllerExecution(condition_, tf_listener_ptr,
                                                               local_costmap_ptr_)),
                           CostmapRecoveryExecution::Ptr(
                                new CostmapRecoveryExecution(condition_, tf_listener_ptr,
                                                             global_costmap_ptr_,
                                                             local_costmap_ptr_))),
    global_costmap_ptr_(new costmap_2d::Costmap2DROS("global_costmap", *tf_listener_ptr_)),
    local_costmap_ptr_(new costmap_2d::Costmap2DROS("local_costmap", *tf_listener_ptr_))
{
  // even if shutdown_costmaps is a dynamically reconfigurable parameter, we
  // need it here to decide weather to start or not the costmaps on starting up
  private_nh_.param("shutdown_costmaps", shutdown_costmaps_, false);

  // initialize costmaps (stopped if shutdown_costmaps is true)
  if (!shutdown_costmaps_)
  {
    local_costmap_active_ = true;
    global_costmap_active_ = true;
  }
  else
  {
    local_costmap_ptr_->stop();
    global_costmap_ptr_->stop();
    local_costmap_active_ = false;
    global_costmap_active_ = false;
  }

  // initialize all plugins
  initializeServerComponents();

  // start all action servers
  startActionServers();

  // advertise services and current goal topic
  check_pose_cost_srv_ = private_nh_.advertiseService("check_pose_cost",
                                                      &CostmapNavigationServer::callServiceCheckPoseCost, this);
  clear_costmaps_srv_ = private_nh_.advertiseService("clear_costmaps",
                                                     &CostmapNavigationServer::callServiceClearCostmaps, this);

  current_goal_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 0);

  // dynamic reconfigure server for mbf_costmap_nav configuration; also include abstract server parameters
  dsrv_costmap_ = boost::make_shared<dynamic_reconfigure::Server<mbf_costmap_nav::MoveBaseFlexConfig> >(private_nh_);
  dsrv_costmap_->setCallback(boost::bind(&CostmapNavigationServer::reconfigure, this, _1, _2));
}

CostmapNavigationServer::~CostmapNavigationServer()
{
  local_costmap_ptr_->stop();
  global_costmap_ptr_->stop();
}

void CostmapNavigationServer::reconfigure(mbf_costmap_nav::MoveBaseFlexConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

  // Make sure we have the original configuration the first time we're called, so we can restore it if needed
  if (!setup_reconfigure_)
  {
    default_config_ = config;
    setup_reconfigure_ = true;
  }

  if (config.restore_defaults)
  {
    config = default_config_;
    // if someone sets restore defaults on the parameter server, prevent looping
    config.restore_defaults = false;
  }

  // fill the abstract configuration common to all MBF-based navigation
  mbf_abstract_nav::MoveBaseFlexConfig abstract_config;
  abstract_config.planner_frequency = config.planner_frequency;
  abstract_config.planner_patience = config.planner_patience;
  abstract_config.planner_max_retries = config.planner_max_retries;
  abstract_config.controller_frequency = config.controller_frequency;
  abstract_config.controller_patience = config.controller_patience;
  abstract_config.controller_max_retries = config.controller_max_retries;
  abstract_config.recovery_enabled = config.recovery_enabled;
  abstract_config.recovery_patience = config.recovery_patience;
  abstract_config.oscillation_timeout = config.oscillation_timeout;
  abstract_config.oscillation_distance = config.oscillation_distance;
  abstract_config.restore_defaults = config.restore_defaults;
  mbf_abstract_nav::AbstractNavigationServer::reconfigure(abstract_config, level);

  // handle costmap activation reconfiguration here.
  shutdown_costmaps_delay_ = ros::Duration(config.shutdown_costmaps_delay);
  if (shutdown_costmaps_delay_.isZero())
    ROS_WARN("Zero shutdown costmaps delay is not recommended, as it forces us to enable costmaps on each action");

  if (shutdown_costmaps_ && !config.shutdown_costmaps)
  {
    checkActivateCostmaps();
    shutdown_costmaps_ = config.shutdown_costmaps;
  }
  if (!shutdown_costmaps_ && config.shutdown_costmaps)
  {
    shutdown_costmaps_ = config.shutdown_costmaps;
    checkDeactivateCostmaps();
  }

  last_config_ = config;
}

bool CostmapNavigationServer::callServiceCheckPoseCost(mbf_msgs::CheckPose::Request &request,
                                                       mbf_msgs::CheckPose::Response &response)
{
  // selecting the requested costmap
  CostmapPtr costmap;
  std::string costmap_name;
  switch (request.costmap)
  {
    case mbf_msgs::CheckPose::Request::LOCAL_COSTMAP:
      costmap = local_costmap_ptr_;
      costmap_name = "local costmap";
      break;
    case mbf_msgs::CheckPose::Request::GLOBAL_COSTMAP:
      costmap = global_costmap_ptr_;
      costmap_name = "global costmap";
      break;
    default:
      ROS_ERROR_STREAM("No valid costmap provided; options are "
                       << mbf_msgs::CheckPose::Request::LOCAL_COSTMAP << ": local costmap, "
                       << mbf_msgs::CheckPose::Request::GLOBAL_COSTMAP << ": global costmap");
      return false;
  }

  // get target pose or current robot pose as x, y, yaw coordinates
  std::string costmap_frame = costmap->getGlobalFrameID();

  geometry_msgs::PoseStamped pose;
  if (request.current_pose)
  {
    if (! mbf_utility::getRobotPose(*tf_listener_ptr_, robot_frame_, costmap_frame, ros::Duration(0.5), pose))
    {
      ROS_ERROR_STREAM("Get robot pose on " << costmap_name << " frame '" << costmap_frame << "' failed");
      return false;
    }
  }
  else
  {
    if (! mbf_utility::transformPose(*tf_listener_ptr_, costmap_frame, request.pose.header.stamp,
                                          ros::Duration(0.5), request.pose, global_frame_, pose))
    {
      ROS_ERROR_STREAM("Transform target pose to " << costmap_name << " frame '" << costmap_frame << "' failed");
      return false;
    }
  }

  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double yaw = tf::getYaw(pose.pose.orientation);

  // lock costmap so content doesn't change while adding cell costs
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getCostmap()->getMutex()));

  // ensure it's active so cost reflects latest sensor readings
  checkActivateCostmaps();

  // pad raw footprint to the requested safety distance; note that we discard footprint_padding parameter effect
  std::vector<geometry_msgs::Point> footprint = costmap->getUnpaddedRobotFootprint();
  costmap_2d::padFootprint(footprint, request.safety_dist);

  // use a footprint helper instance to get all the cells totally or partially within footprint polygon
  base_local_planner::FootprintHelper fph;
  std::vector<base_local_planner::Position2DInt> footprint_cells =
    fph.getFootprintCells(Eigen::Vector3f(x, y, yaw), footprint, *costmap->getCostmap(), true);
  response.state = mbf_msgs::CheckPose::Response::FREE;
  if (footprint_cells.empty())
  {
    // no cells within footprint polygon must mean that robot is completely outside of the map
    response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::OUTSIDE));
  }
  else
  {
    // integrate the cost of all cells; state value precedence is UNKNOWN > LETHAL > INSCRIBED > FREE
    for (int i = 0; i < footprint_cells.size(); ++i)
    {
      unsigned char cost = costmap->getCostmap()->getCost(footprint_cells[i].x, footprint_cells[i].y);
      switch (cost)
      {
        case costmap_2d::NO_INFORMATION:
          response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::UNKNOWN));
          response.cost += cost;
          break;
        case costmap_2d::LETHAL_OBSTACLE:
          response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::LETHAL));
          response.cost += cost;
          break;
        case costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
          response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::INSCRIBED));
          response.cost += cost;
          break;
        default:response.cost += cost;
          break;
      }
    }
  }

  // Provide some details of the outcome
  switch (response.state)
  {
    case mbf_msgs::CheckPose::Response::OUTSIDE:
      ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is outside the map (cost = " << response.cost
                                << "; safety distance = " << request.safety_dist << ")");
      break;
    case mbf_msgs::CheckPose::Response::UNKNOWN:
      ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is in unknown space! (cost = " << response.cost
                                << "; safety distance = " << request.safety_dist << ")");
      break;
    case mbf_msgs::CheckPose::Response::LETHAL:
      ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is in collision! (cost = " << response.cost
                                << "; safety distance = " << request.safety_dist << ")");
      break;
    case mbf_msgs::CheckPose::Response::INSCRIBED:
      ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is near an obstacle (cost = " << response.cost
                                << "; safety distance = " << request.safety_dist << ")");
      break;
    case mbf_msgs::CheckPose::Response::FREE:
      ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is free (cost = " << response.cost
                                << "; safety distance = " << request.safety_dist << ")");
      break;
  }

  checkDeactivateCostmaps();

  return true;
}

bool CostmapNavigationServer::callServiceClearCostmaps(std_srvs::Empty::Request &request,
                                                       std_srvs::Empty::Response &response)
{
  local_costmap_ptr_->resetLayers();
  global_costmap_ptr_->resetLayers();
  return true;
}

void CostmapNavigationServer::checkActivateCostmaps()
{
  shutdown_costmaps_timer_.stop();

  // Activate costmaps if we shutdown them when not moving and they are not already active
  if (shutdown_costmaps_ && !local_costmap_active_)
  {
    local_costmap_ptr_->start();
    local_costmap_active_ = true;
    ROS_DEBUG_STREAM("Local costmap activated.");
  }

  if (shutdown_costmaps_ && !global_costmap_active_)
  {
    global_costmap_ptr_->start();
    global_costmap_active_ = true;
    ROS_DEBUG_STREAM("Global costmap activated.");
  }
}

void CostmapNavigationServer::checkDeactivateCostmaps()
{
  if (shutdown_costmaps_ &&
      ((local_costmap_active_ || global_costmap_active_) && !(active_planning_ || active_moving_ || active_recovery_)))
  {
    // Delay costmaps shutdown by shutdown_costmaps_delay so we don't need to enable at each step of a normal
    // navigation sequence, what is terribly inneficient; the timer is stopped on costmaps re-activation and
    // reset after every new call to deactivate
    shutdown_costmaps_timer_ =
      private_nh_.createTimer(shutdown_costmaps_delay_, &CostmapNavigationServer::deactivateCostmaps, this, true);
  }
}

void CostmapNavigationServer::deactivateCostmaps(const ros::TimerEvent &event)
{
  local_costmap_ptr_->stop();
  local_costmap_active_ = false;
  ROS_DEBUG_STREAM("Local costmap deactivated.");
  global_costmap_ptr_->stop();
  global_costmap_active_ = false;
  ROS_DEBUG_STREAM("Global costmap deactivated.");
}

void CostmapNavigationServer::callActionGetPath(const mbf_msgs::GetPathGoalConstPtr &goal)
{
  checkActivateCostmaps();
  AbstractNavigationServer::callActionGetPath(goal);
  checkDeactivateCostmaps();
}

void CostmapNavigationServer::callActionExePath(const mbf_msgs::ExePathGoalConstPtr &goal)
{
  checkActivateCostmaps();
  AbstractNavigationServer::callActionExePath(goal);
  checkDeactivateCostmaps();
}

void CostmapNavigationServer::callActionRecovery(const mbf_msgs::RecoveryGoalConstPtr &goal)
{
  checkActivateCostmaps();
  AbstractNavigationServer::callActionRecovery(goal);
  checkDeactivateCostmaps();
}

} /* namespace mbf_costmap_nav */
