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
 *  costmap_navigation_server.cpp
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
#include <nav_core_wrapper/wrapper_global_planner.h>
#include <nav_core_wrapper/wrapper_local_planner.h>
#include <nav_core_wrapper/wrapper_recovery_behavior.h>

#include "mbf_costmap_nav/costmap_navigation_server.h"

namespace mbf_costmap_nav
{


CostmapNavigationServer::CostmapNavigationServer(const TFPtr &tf_listener_ptr) :
  AbstractNavigationServer(tf_listener_ptr),
  recovery_plugin_loader_("mbf_costmap_core", "mbf_costmap_core::CostmapRecovery"),
  nav_core_recovery_plugin_loader_("nav_core", "nav_core::RecoveryBehavior"),
  controller_plugin_loader_("mbf_costmap_core", "mbf_costmap_core::CostmapController"),
  nav_core_controller_plugin_loader_("nav_core", "nav_core::BaseLocalPlanner"),
  planner_plugin_loader_("mbf_costmap_core", "mbf_costmap_core::CostmapPlanner"),
  nav_core_planner_plugin_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
  global_costmap_ptr_(new costmap_2d::Costmap2DROS("global_costmap", *tf_listener_ptr_)),
  local_costmap_ptr_(new costmap_2d::Costmap2DROS("local_costmap", *tf_listener_ptr_)),
  setup_reconfigure_(false), shutdown_costmaps_(false)
{
  // even if shutdown_costmaps is a dynamically reconfigurable parameter, we
  // need it here to decide whether to start or not the costmaps on starting up
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

  // advertise services and current goal topic
  check_pose_cost_srv_ = private_nh_.advertiseService("check_pose_cost",
                                                      &CostmapNavigationServer::callServiceCheckPoseCost, this);
  check_path_cost_srv_ = private_nh_.advertiseService("check_path_cost",
                                                      &CostmapNavigationServer::callServiceCheckPathCost, this);
  clear_costmaps_srv_ = private_nh_.advertiseService("clear_costmaps",
                                                     &CostmapNavigationServer::callServiceClearCostmaps, this);

  // dynamic reconfigure server for mbf_costmap_nav configuration; also include abstract server parameters
  dsrv_costmap_ = boost::make_shared<dynamic_reconfigure::Server<mbf_costmap_nav::MoveBaseFlexConfig> >(private_nh_);
  dsrv_costmap_->setCallback(boost::bind(&CostmapNavigationServer::reconfigure, this, _1, _2));

  // initialize all plugins
  initializeServerComponents();

  // start all action servers
  startActionServers();
}

mbf_abstract_nav::AbstractPlannerExecution::Ptr CostmapNavigationServer::newPlannerExecution(
    const std::string name,
    const mbf_abstract_core::AbstractPlanner::Ptr plugin_ptr)
{
  return boost::make_shared<mbf_costmap_nav::CostmapPlannerExecution>(
      name,
      boost::static_pointer_cast<mbf_costmap_core::CostmapPlanner>(plugin_ptr),
      boost::ref(global_costmap_ptr_),
      last_config_,
      boost::bind(&CostmapNavigationServer::checkActivateCostmaps, this),
      boost::bind(&CostmapNavigationServer::checkDeactivateCostmaps, this));
}

mbf_abstract_nav::AbstractControllerExecution::Ptr CostmapNavigationServer::newControllerExecution(
    const std::string name,
    const mbf_abstract_core::AbstractController::Ptr plugin_ptr)
{
  return boost::make_shared<mbf_costmap_nav::CostmapControllerExecution>(
      name,
      boost::static_pointer_cast<mbf_costmap_core::CostmapController>(plugin_ptr),
      tf_listener_ptr_,
      boost::ref(local_costmap_ptr_),
      last_config_,
      boost::bind(&CostmapNavigationServer::checkActivateCostmaps, this),
      boost::bind(&CostmapNavigationServer::checkDeactivateCostmaps, this));
}

mbf_abstract_nav::AbstractRecoveryExecution::Ptr CostmapNavigationServer::newRecoveryExecution(
    const std::string name,
    const mbf_abstract_core::AbstractRecovery::Ptr plugin_ptr)
{
  return boost::make_shared<mbf_costmap_nav::CostmapRecoveryExecution>(
      name,
      boost::static_pointer_cast<mbf_costmap_core::CostmapRecovery>(plugin_ptr),
      tf_listener_ptr_,
      boost::ref(global_costmap_ptr_),
      boost::ref(local_costmap_ptr_),
      last_config_,
      boost::bind(&CostmapNavigationServer::checkActivateCostmaps, this),
      boost::bind(&CostmapNavigationServer::checkDeactivateCostmaps, this));
}

mbf_abstract_core::AbstractPlanner::Ptr CostmapNavigationServer::loadPlannerPlugin(const std::string& planner_type)
{
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr;
  try
  {
    planner_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractPlanner>(
        planner_plugin_loader_.createInstance(planner_type));
    std::string planner_name = planner_plugin_loader_.getName(planner_type);
    ROS_DEBUG_STREAM("mbf_costmap_core-based planner plugin " << planner_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException &ex_mbf_core)
  {
    ROS_DEBUG_STREAM("Failed to load the " << planner_type << " planner as a mbf_costmap_core-based plugin."
                                          << " Try to load as a nav_core-based plugin. " << ex_mbf_core.what());
    try
    {
      // For plugins still based on old nav_core API, we load them and pass to a new MBF API that will act as wrapper
      boost::shared_ptr<nav_core::BaseGlobalPlanner> nav_core_planner_ptr = nav_core_planner_plugin_loader_.createInstance(planner_type);
      planner_ptr = boost::make_shared<mbf_nav_core_wrapper::WrapperGlobalPlanner>(nav_core_planner_ptr);
      std::string planner_name = nav_core_planner_plugin_loader_.getName(planner_type);
      ROS_DEBUG_STREAM("nav_core-based planner plugin " << planner_name << " loaded");
    }
    catch (const pluginlib::PluginlibException &ex_nav_core)
    {
      ROS_FATAL_STREAM("Failed to load the " << planner_type << " planner, are you sure it's properly registered"
          << " and that the containing library is built? " << ex_mbf_core.what() << " " << ex_nav_core.what());
    }
  }

  return planner_ptr;
}

bool CostmapNavigationServer::initializePlannerPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr
)
{
  mbf_costmap_core::CostmapPlanner::Ptr costmap_planner_ptr
      = boost::static_pointer_cast<mbf_costmap_core::CostmapPlanner>(planner_ptr);
  ROS_DEBUG_STREAM("Initialize planner \"" << name << "\".");

  if (!global_costmap_ptr_)
  {
    ROS_FATAL_STREAM("The costmap pointer has not been initialized!");
    return false;
  }

  costmap_planner_ptr->initialize(name, global_costmap_ptr_.get());
  ROS_DEBUG("Planner plugin initialized.");
  return true;
}


mbf_abstract_core::AbstractController::Ptr CostmapNavigationServer::loadControllerPlugin(const std::string& controller_type)
{
  mbf_abstract_core::AbstractController::Ptr controller_ptr;
  try
  {
    controller_ptr = controller_plugin_loader_.createInstance(controller_type);
    std::string controller_name = controller_plugin_loader_.getName(controller_type);
    ROS_DEBUG_STREAM("mbf_costmap_core-based controller plugin " << controller_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException &ex_mbf_core)
  {
    ROS_DEBUG_STREAM("Failed to load the " << controller_type << " controller as a mbf_costmap_core-based plugin;"
                                          << "  we will retry to load as a nav_core-based plugin. " << ex_mbf_core.what());
    try
    {
      // For plugins still based on old nav_core API, we load them and pass to a new MBF API that will act as wrapper
      boost::shared_ptr<nav_core::BaseLocalPlanner> nav_core_controller_ptr
          = nav_core_controller_plugin_loader_.createInstance(controller_type);
      controller_ptr = boost::make_shared<mbf_nav_core_wrapper::WrapperLocalPlanner>(nav_core_controller_ptr);
      std::string controller_name = nav_core_controller_plugin_loader_.getName(controller_type);
      ROS_DEBUG_STREAM("nav_core-based controller plugin " << controller_name << " loaded.");
    }
    catch (const pluginlib::PluginlibException &ex_nav_core)
    {
      ROS_FATAL_STREAM("Failed to load the " << controller_type << " controller, are you sure it's properly registered"
          << " and that the containing library is built? " << ex_mbf_core.what() << " " << ex_nav_core.what());
    }
  }
  return controller_ptr;
}

bool CostmapNavigationServer::initializeControllerPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractController::Ptr& controller_ptr)
{
  ROS_DEBUG_STREAM("Initialize controller \"" << name << "\".");

  if (!tf_listener_ptr_)
  {
    ROS_FATAL_STREAM("The tf listener pointer has not been initialized!");
    return false;
  }

  if (!local_costmap_ptr_)
  {
    ROS_FATAL_STREAM("The costmap pointer has not been initialized!");
    return false;
  }

  mbf_costmap_core::CostmapController::Ptr costmap_controller_ptr
      = boost::static_pointer_cast<mbf_costmap_core::CostmapController>(controller_ptr);
  costmap_controller_ptr->initialize(name, tf_listener_ptr_.get(), local_costmap_ptr_.get());
  ROS_DEBUG_STREAM("Controller plugin \"" << name << "\" initialized.");
  return true;
}

mbf_abstract_core::AbstractRecovery::Ptr CostmapNavigationServer::loadRecoveryPlugin(
    const std::string& recovery_type)
{
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr;

  try
  {
    recovery_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractRecovery>(
        recovery_plugin_loader_.createInstance(recovery_type));
    std::string recovery_name = recovery_plugin_loader_.getName(recovery_type);
    ROS_DEBUG_STREAM("mbf_costmap_core-based recovery behavior plugin " << recovery_name << " loaded.");
  }
  catch (pluginlib::PluginlibException &ex_mbf_core)
  {
    ROS_DEBUG_STREAM("Failed to load the " << recovery_type << " recovery behavior as a mbf_costmap_core-based plugin;"
        << " Retry to load as a nav_core-based plugin. " << ex_mbf_core.what());
    try
    {
      // For plugins still based on old nav_core API, we load them and pass to a new MBF API that will act as wrapper
      boost::shared_ptr<nav_core::RecoveryBehavior> nav_core_recovery_ptr =
          nav_core_recovery_plugin_loader_.createInstance(recovery_type);

      recovery_ptr = boost::make_shared<mbf_nav_core_wrapper::WrapperRecoveryBehavior>(nav_core_recovery_ptr);
      std::string recovery_name = recovery_plugin_loader_.getName(recovery_type);
      ROS_DEBUG_STREAM("nav_core-based recovery behavior plugin " << recovery_name << " loaded.");

    }
    catch (const pluginlib::PluginlibException &ex_nav_core)
    {
      ROS_FATAL_STREAM("Failed to load the " << recovery_type << " recovery behavior, are you sure it's properly registered"
          << " and that the containing library is built? " << ex_mbf_core.what() << " " << ex_nav_core.what());
    }
  }

  return recovery_ptr;
}

bool CostmapNavigationServer::initializeRecoveryPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr)
{
  ROS_DEBUG_STREAM("Initialize recovery behavior \"" << name << "\".");

  if (!tf_listener_ptr_)
  {
    ROS_FATAL_STREAM("The tf listener pointer has not been initialized!");
    return false;
  }

  if (!local_costmap_ptr_)
  {
    ROS_FATAL_STREAM("The local costmap pointer has not been initialized!");
    return false;
  }

  if (!global_costmap_ptr_)
  {
    ROS_FATAL_STREAM("The global costmap pointer has not been initialized!");
    return false;
  }

  mbf_costmap_core::CostmapRecovery::Ptr behavior =
      boost::static_pointer_cast<mbf_costmap_core::CostmapRecovery>(behavior_ptr);
  behavior->initialize(name, tf_listener_ptr_.get(), global_costmap_ptr_.get(), local_costmap_ptr_.get());
  ROS_DEBUG_STREAM("Recovery behavior plugin \"" << name << "\" initialized.");
  return true;
}


void CostmapNavigationServer::stop()
{
  AbstractNavigationServer::stop();
  ROS_INFO_STREAM_NAMED("mbf_costmap_nav", "Stopping local and global costmap for shutdown");
  local_costmap_ptr_->stop();
  global_costmap_ptr_->stop();
}


CostmapNavigationServer::~CostmapNavigationServer()
{
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

  // ensure costmaps are active so cost reflects latest sensor readings
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
    // lock costmap so content doesn't change while adding cell costs
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getCostmap()->getMutex()));

    // integrate the cost of all cells; state value precedence is UNKNOWN > LETHAL > INSCRIBED > FREE
    for (int i = 0; i < footprint_cells.size(); ++i)
    {
      unsigned char cost = costmap->getCostmap()->getCost(footprint_cells[i].x, footprint_cells[i].y);
      switch (cost)
      {
        case costmap_2d::NO_INFORMATION:
          response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::UNKNOWN));
          response.cost += cost * (request.unknown_cost_mult ? request.unknown_cost_mult : 1.0);
          break;
        case costmap_2d::LETHAL_OBSTACLE:
          response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::LETHAL));
          response.cost += cost * (request.lethal_cost_mult ? request.lethal_cost_mult : 1.0);
          break;
        case costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
          response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::INSCRIBED));
          response.cost += cost * (request.inscrib_cost_mult ? request.inscrib_cost_mult : 1.0);
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

bool CostmapNavigationServer::callServiceCheckPathCost(mbf_msgs::CheckPath::Request &request,
                                                       mbf_msgs::CheckPath::Response &response)
{
  // selecting the requested costmap
  CostmapPtr costmap;
  std::string costmap_name;
  switch (request.costmap)
  {
    case mbf_msgs::CheckPath::Request::LOCAL_COSTMAP:
      costmap = local_costmap_ptr_;
      costmap_name = "local costmap";
      break;
    case mbf_msgs::CheckPath::Request::GLOBAL_COSTMAP:
      costmap = global_costmap_ptr_;
      costmap_name = "global costmap";
      break;
    default:ROS_ERROR_STREAM("No valid costmap provided; options are "
                             << mbf_msgs::CheckPath::Request::LOCAL_COSTMAP << ": local costmap, "
                             << mbf_msgs::CheckPath::Request::GLOBAL_COSTMAP << ": global costmap");
      return false;
  }

  // ensure costmaps are active so cost reflects latest sensor readings
  checkActivateCostmaps();

  // get target pose or current robot pose as x, y, yaw coordinates
  std::string costmap_frame = costmap->getGlobalFrameID();

  // use a footprint helper instance to get all the cells totally or partially within footprint polygon
  base_local_planner::FootprintHelper fph;

  std::vector<geometry_msgs::Point> footprint;
  if (!request.path_cells_only)
  {
    // unless we want to check just the cells directly traversed by the path, pad raw footprint
    // to the requested safety distance; note that we discard footprint_padding parameter effect
    footprint = costmap->getUnpaddedRobotFootprint();
    costmap_2d::padFootprint(footprint, request.safety_dist);
  }

  // lock costmap so content doesn't change while adding cell costs
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getCostmap()->getMutex()));

  geometry_msgs::PoseStamped pose;

  response.state = mbf_msgs::CheckPath::Response::FREE;

  for (int i = 0; i < request.path.poses.size(); ++i)
  {
    response.last_checked = i;

    if (! mbf_utility::transformPose(*tf_listener_ptr_, costmap_frame, request.path.header.stamp,
                                     ros::Duration(0.5), request.path.poses[i], global_frame_, pose))
    {
      ROS_ERROR_STREAM("Transform target pose to " << costmap_name << " frame '" << costmap_frame << "' failed");
      return false;
    }

    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double yaw = tf::getYaw(pose.pose.orientation);
    std::vector<base_local_planner::Position2DInt> cells_to_check;
    if (request.path_cells_only)
    {
      base_local_planner::Position2DInt cell;
      if (costmap->getCostmap()->worldToMap(x, y, (unsigned int&)cell.x, (unsigned int&)cell.y))
        cells_to_check.push_back(cell);  // out of map if false; cells_to_check will be empty
    }
    else
    {
      cells_to_check = fph.getFootprintCells(Eigen::Vector3f(x, y, yaw), footprint, *costmap->getCostmap(), true);
    }

    if (cells_to_check.empty())
    {
      // if path_cells_only is true, this means that current path's pose is outside the map
      // if not, no cells within footprint polygon means that robot is completely outside of the map
      response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPath::Response::OUTSIDE));
    }
    else
    {
      // integrate the cost of all cells; state value precedence is UNKNOWN > LETHAL > INSCRIBED > FREE
      // we apply the requested cost multipliers if different from zero (default value)
      for (int j = 0; j < cells_to_check.size(); ++j)
      {
        unsigned char cost = costmap->getCostmap()->getCost(cells_to_check[j].x, cells_to_check[j].y);
        switch (cost)
        {
          case costmap_2d::NO_INFORMATION:
            response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::UNKNOWN));
            response.cost += cost * (request.unknown_cost_mult ? request.unknown_cost_mult : 1.0);
            break;
          case costmap_2d::LETHAL_OBSTACLE:
            response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPath::Response::LETHAL));
            response.cost += cost * (request.lethal_cost_mult ? request.lethal_cost_mult : 1.0);
            break;
          case costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
            response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPath::Response::INSCRIBED));
            response.cost += cost * (request.inscrib_cost_mult ? request.inscrib_cost_mult : 1.0);
            break;
          default:response.cost += cost;
            break;
        }
      }
    }

    if (request.return_on && response.state >= request.return_on)
    {
      // i-th pose state is bad enough for the client, so provide some details of the outcome and abort checking
      switch (response.state)
      {
        case mbf_msgs::CheckPath::Response::OUTSIDE:
          ROS_DEBUG_STREAM("At pose " << i << " [" << x << ", " << y << ", " << yaw << "] path goes outside the map "
                           << "(cost = " << response.cost << "; safety distance = " << request.safety_dist << ")");
          break;
        case mbf_msgs::CheckPath::Response::UNKNOWN:
          ROS_DEBUG_STREAM("At pose " << i << " [" << x << ", " << y << ", " << yaw << "] path goes in unknown space! "
                           << "(cost = " << response.cost << "; safety distance = " << request.safety_dist << ")");
          break;
        case mbf_msgs::CheckPath::Response::LETHAL:
          ROS_DEBUG_STREAM("At pose " << i << " [" << x << ", " << y << ", " << yaw << "] path goes in collision! "
                           << "(cost = " << response.cost << "; safety distance = " << request.safety_dist << ")");
          break;
        case mbf_msgs::CheckPath::Response::INSCRIBED:
          ROS_DEBUG_STREAM("At pose " << i << " [" << x << ", " << y << ", " << yaw << "] path goes near an obstacle "
                           << "(cost = " << response.cost << "; safety distance = " << request.safety_dist << ")");
          break;
        case mbf_msgs::CheckPath::Response::FREE:
          ROS_DEBUG_STREAM("Path is entirely free (maximum cost = "
                           << response.cost << "; safety distance = " << request.safety_dist << ")");
          break;
      }

      break;
    }

    i += request.skip_poses;  // skip some poses to speedup processing (disabled by default)
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
      ((local_costmap_active_ || global_costmap_active_)))
  {
    // Delay costmaps shutdown by shutdown_costmaps_delay so we don't need to enable at each step of a normal
    // navigation sequence, what is terribly inefficient; the timer is stopped on costmaps re-activation and
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

} /* namespace mbf_costmap_nav */
