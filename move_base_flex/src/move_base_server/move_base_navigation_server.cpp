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
#include <base_local_planner/costmap_model.h>
#include <move_base_flex_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "move_base_flex/move_base_server/move_base_navigation_server.h"

namespace move_base_flex
{


MoveBaseNavigationServer::MoveBaseNavigationServer(const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr) :
  AbstractNavigationServer(tf_listener_ptr,
                           MoveBasePlannerExecution::Ptr(
                                new MoveBasePlannerExecution(condition_, costmap_global_planner_ptr_)),
                           MoveBaseControllerExecution::Ptr(
                                new MoveBaseControllerExecution(condition_, tf_listener_ptr,
                                                                costmap_local_planner_ptr_)),
                           MoveBaseRecoveryExecution::Ptr(
                                new MoveBaseRecoveryExecution(condition_, tf_listener_ptr,
                                                              costmap_global_planner_ptr_,
                                                              costmap_local_planner_ptr_))),
    costmap_global_planner_ptr_(new costmap_2d::Costmap2DROS("global_costmap", *tf_listener_ptr_)),
    costmap_local_planner_ptr_(new costmap_2d::Costmap2DROS("local_costmap", *tf_listener_ptr_))
{
  ros::NodeHandle private_nh("~");

  // shutdown costmaps
  private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);

  costmap_global_planner_ptr_->pause();
  costmap_local_planner_ptr_->pause();

  // initialize all plugins
  initializeServerComponents();

  // start all action servers
  startActionServers();

  // start costmaps
  costmap_global_planner_ptr_->start();
  costmap_local_planner_ptr_->start();

  local_costmap_active_ = true;
  global_costmap_active_ = true;

  // stop updating costmaps when not planning, moving, or recovering
  if (shutdown_costmaps_)
  {
    costmap_local_planner_ptr_->stop();
    costmap_global_planner_ptr_->stop();
    local_costmap_active_ = false;
    global_costmap_active_ = false;
  }

  // advertise services
  check_pose_cost_srv_ = private_nh.advertiseService("check_pose_cost",
                                                     &MoveBaseNavigationServer::callServiceCheckPoseCost, this);
  clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps",
                                                    &MoveBaseNavigationServer::callServiceClearCostmaps, this);
  make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBaseNavigationServer::callServiceMakePlan, this);

  current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0);
}

MoveBaseNavigationServer::~MoveBaseNavigationServer()
{
  costmap_local_planner_ptr_->stop();
  costmap_global_planner_ptr_->stop();
}

void MoveBaseNavigationServer::reconfigure(move_base_flex::MoveBaseFlexConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

  AbstractNavigationServer::reconfigure(config, level);

  // handle costmap activation reconfiguration here.
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

}

bool MoveBaseNavigationServer::callServiceCheckPoseCost(move_base_flex_msgs::CheckPose::Request &request,
                                                        move_base_flex_msgs::CheckPose::Response &response)
{
  // selecting the requested costmap
  CostmapPtr costmap;
  switch (request.costmap)
  {
    case move_base_flex_msgs::CheckPose::Request::LOCAL_COSTMAP:
      costmap = costmap_local_planner_ptr_;
      if (shutdown_costmaps_ && !local_costmap_active_)
      {
        ROS_WARN_STREAM("Calling check_pose_cost on local costmap while costmap not active;"
                        << " cost won't reflect latest sensor readings");
        costmap->updateMap();
      }
      break;
    case move_base_flex_msgs::CheckPose::Request::GLOBAL_COSTMAP:
      costmap = costmap_global_planner_ptr_;
      if (shutdown_costmaps_ && !global_costmap_active_)
      {
        ROS_WARN_STREAM("Calling check_pose_cost on global costmap while costmap not active;"
                        << " cost won't reflect latest sensor readings");
        costmap->updateMap();
      }
      break;
  }

  double x = request.pose.pose.position.x;
  double y = request.pose.pose.position.y;
  double yaw = tf::getYaw(request.pose.pose.orientation);

  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getCostmap()->getMutex()));

  std::vector<geometry_msgs::Point> footprint = costmap->getUnpaddedRobotFootprint();
  costmap_2d::padFootprint(footprint, request.safety_dist);

  response.cost = base_local_planner::CostmapModel(*(costmap->getCostmap())).footprintCost(x, y, yaw, footprint);
  if (response.cost < 0.0)
  {
    ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is in collision! (cost = " << response.cost
                              << "; safety distance = " << request.safety_dist << ")");
    response.colliding = true;
  }
  else
  {
    ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is safe for navigation (cost = " << response.cost
                              << "; safety distance = " << request.safety_dist << ")");
    response.colliding = false;
  }

  return true;
}

bool MoveBaseNavigationServer::callServiceMakePlan(nav_msgs::GetPlan::Request &request,
                                                   nav_msgs::GetPlan::Response &response)
{

  ROS_ERROR_STREAM("The Service \"make_plan\" has been deprecated."
                   << " Please use the new actions provided by move_base_flex!");

  // TODO !!! Replace by get plan action!

  geometry_msgs::PoseStamped start = request.start;
  geometry_msgs::PoseStamped goal = request.goal;

  float tolerance = request.tolerance;

  geometry_msgs::PoseStamped global_start_pose;
  geometry_msgs::PoseStamped global_goal_pose;

  if (!costmap_global_planner_ptr_)
  {
    ROS_ERROR_STREAM("The global costmap is not initialized! Canceling service call.");
    return false;
  }

  // TODO improve the shutdown costmaps...
  if (shutdown_costmaps_)
  {
    costmap_local_planner_ptr_->start();
    costmap_global_planner_ptr_->start();
  }

  ros::Duration tf_timeout(0.3);

  transformPose(*tf_listener_ptr_, global_frame_, start.header.stamp, tf_timeout, start, global_frame_,
                global_start_pose);

  transformPose(*tf_listener_ptr_, global_frame_, goal.header.stamp, tf_timeout, start, global_frame_,
                global_start_pose);

  // if no start pose is defined (frame id not set) take the current robot pose
  if (start.header.frame_id == "")
  {
    if (!getRobotPose(start))
    {
      ROS_ERROR_STREAM("Could not get the robot pose! Canceling the service call.");
      return false;
    }
  }

  // get the current robot pose
  if (!getRobotPose(global_start_pose))
  {
    ROS_ERROR_STREAM("Failed to get the current robot pose!");
    return false;
  }

  if (action_server_get_path_ptr_->isActive())
  {
    ROS_ERROR_STREAM("Global planner action server is still running; canceling the service call");
    return false;
  }

  checkActivateCostmaps();

  if (!planning_ptr_->startPlanning(global_start_pose, global_goal_pose, tolerance))
  {
    ROS_ERROR_STREAM("Another thread is still planning. Canceling the service call.");
    return false;
  }

  MoveBasePlannerExecution::PlanningState state_planning_input_;

  std::vector<geometry_msgs::PoseStamped> plan, global_plan;
  double plan_cost;

  active_planning_ = true;

  bool success = false;

  nav_msgs::Path path;

  while (active_planning_ && ros::ok())
  {
    // get the current state of the planning thread
    state_planning_input_ = planning_ptr_->getState();

    switch (state_planning_input_)
    {
      case MoveBasePlannerExecution::STOPPED:
        ROS_INFO("robot navigation state: stopped");
        ROS_INFO("the active_planning_ has been aborted!");
        active_planning_ = false;
        success = false;
        break;

        // in progress
      case MoveBasePlannerExecution::PLANNING:
        ROS_INFO_THROTTLE(2.0, "robot navigation state: planning");
        break;

        // found a new plan
      case MoveBasePlannerExecution::FOUND_PLAN:
        planning_ptr_->getNewPlan(plan, plan_cost);
/////  XXX  don't make sense to me  Sebastian?          planning_ptr_->cancel(); // cancel the re-planning loop!
        ROS_INFO_STREAM("robot navigation state: found plan with cost: " << plan_cost);
        active_planning_ = false;
        success = true;

        if (!transformPlanToGlobalFrame(plan, global_plan))
        {
          ROS_ERROR_STREAM("Cloud not transform the plan to the global frame!");
          success = false;
          break;
        }

        path.header.stamp = ros::Time::now();
        path.header.seq = 0; // TODO check for a more meaningful id
        path.header.frame_id = global_frame_;
        path.poses = global_plan;

        if (path.poses.empty())
        {
          ROS_ERROR_STREAM("The path (list of poses) is empty!");
          success = false;
          break;
        }
        response.plan = path;
        ROS_INFO_STREAM("Got path with " << path.poses.size() << " poses!");
        break;

        // no plan found
      case MoveBasePlannerExecution::NO_PLAN_FOUND:
        ROS_INFO("robot navigation state: no plan found");
        active_planning_ = false;

        path.header.stamp = ros::Time::now();
        path.header.seq = 0; // TODO check for a more meaningful id
        path.header.frame_id = global_frame_;
        path.poses.clear();
        success = true; // service success? TODO
        break;
      default:
        ROS_INFO_STREAM("robot navigation state: " << state_planning_input_);

        ROS_FATAL_STREAM("Unknown case in switch case in callServiceMakePlan:" << state_planning_input_);
        exit(1);  // IX -> add the state to the switch case   XXX TODO :  we should never exit like this!
    }

    // try to sleep a bit
    // normally the thread should be woken up from the moving unit
    // in order to transfer the results to the controller.
    if (active_planning_ && ros::ok())
    {
      boost::mutex mutex;
      boost::unique_lock<boost::mutex> lock(mutex);
      condition_.wait_for(lock, boost::chrono::milliseconds(500));
    }
  }

  checkDeactivateCostmaps();

  if (!active_planning_)
  {
    ROS_INFO_STREAM("\"make_plan\" service ended properly.");
  }
  else
  {
    ROS_ERROR_STREAM("\"make_plan\" service has been stopped!");
  }

  return success;
}

bool MoveBaseNavigationServer::callServiceClearCostmaps(std_srvs::Empty::Request &request,
                                                        std_srvs::Empty::Response &response)
{
  costmap_local_planner_ptr_->resetLayers();
  costmap_global_planner_ptr_->resetLayers();
  return true;
}

void MoveBaseNavigationServer::checkActivateCostmaps()
{
  if (shutdown_costmaps_ && !local_costmap_active_)
  {
    costmap_local_planner_ptr_->start();
    ROS_DEBUG_STREAM("Activating local costmap.");
    local_costmap_active_ = true;
  }

  if (shutdown_costmaps_ && !global_costmap_active_)
  {
    costmap_global_planner_ptr_->start();
    ROS_DEBUG_STREAM("Activating global costmap.");
    global_costmap_active_ = true;
  }
}

void MoveBaseNavigationServer::checkDeactivateCostmaps()
{
  if (!ros::ok() ||
      (shutdown_costmaps_ && local_costmap_active_ && !(active_planning_ || active_moving_ || active_recovery_)))
  {
    costmap_local_planner_ptr_->stop();
    ROS_DEBUG_STREAM("Deactivating local costmap.");
    local_costmap_active_ = false;
  }

  if (!ros::ok() ||
      (shutdown_costmaps_ && global_costmap_active_ && !(active_planning_ || active_moving_ || active_recovery_)))
  {
    costmap_global_planner_ptr_->stop();
    ROS_DEBUG_STREAM("Deactivating global costmap.");
    global_costmap_active_ = false;
  }
}

void MoveBaseNavigationServer::callActionGetPath(const move_base_flex_msgs::GetPathGoalConstPtr &goal)
{
  checkActivateCostmaps();
  AbstractNavigationServer::callActionGetPath(goal);
  checkDeactivateCostmaps();
}

void MoveBaseNavigationServer::callActionExePath(const move_base_flex_msgs::ExePathGoalConstPtr &goal)
{
  checkActivateCostmaps();
  AbstractNavigationServer::callActionExePath(goal);
  checkDeactivateCostmaps();
}

void MoveBaseNavigationServer::callActionRecovery(const move_base_flex_msgs::RecoveryGoalConstPtr &goal)
{
  checkActivateCostmaps();
  AbstractNavigationServer::callActionRecovery(goal);
  checkDeactivateCostmaps();
}
} /* namespace move_base_flex */
