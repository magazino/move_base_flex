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
 *  abstract_navigation_server.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <nav_msgs/Path.h>

#include "mbf_abstract_nav/abstract_navigation_server.h"

namespace mbf_abstract_nav
{

AbstractNavigationServer::AbstractNavigationServer(const TFPtr &tf_listener_ptr)
    : tf_listener_ptr_(tf_listener_ptr), private_nh_("~"),
      planner_plugin_manager_("planners",
          boost::bind(&AbstractNavigationServer::loadPlannerPlugin, this, _1),
          boost::bind(&AbstractNavigationServer::initializePlannerPlugin, this, _1, _2)),
      controller_plugin_manager_("controllers",
          boost::bind(&AbstractNavigationServer::loadControllerPlugin, this, _1),
          boost::bind(&AbstractNavigationServer::initializeControllerPlugin, this, _1, _2)),
      recovery_plugin_manager_("recovery_behaviors",
          boost::bind(&AbstractNavigationServer::loadRecoveryPlugin, this, _1),
          boost::bind(&AbstractNavigationServer::initializeRecoveryPlugin, this, _1, _2)),
      tf_timeout_(private_nh_.param<double>("tf_timeout", 3.0)),
      global_frame_(private_nh_.param<std::string>("global_frame", "map")),
      robot_frame_(private_nh_.param<std::string>("robot_frame", "base_link")),
      robot_info_(*tf_listener_ptr, global_frame_, robot_frame_, tf_timeout_,
                  private_nh_.param<std::string>("odom_topic", "odom")),
      controller_action_(name_action_exe_path, robot_info_),
      planner_action_(name_action_get_path, robot_info_),
      recovery_action_(name_action_recovery, robot_info_),
      move_base_action_(name_action_move_base, robot_info_, recovery_plugin_manager_.getLoadedNames())
{
  // init cmd_vel publisher for the robot velocity
  vel_pub_ = ros::NodeHandle().advertise<geometry_msgs::Twist>("cmd_vel", 1);

  action_server_get_path_ptr_ = ActionServerGetPathPtr(
    new ActionServerGetPath(
      private_nh_,
      name_action_get_path,
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionGetPath, this, _1),
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::cancelActionGetPath, this, _1),
      false));

  action_server_exe_path_ptr_ = ActionServerExePathPtr(
    new ActionServerExePath(
      private_nh_,
      name_action_exe_path,
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionExePath, this, _1),
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::cancelActionExePath, this, _1),
      false));

  action_server_recovery_ptr_ = ActionServerRecoveryPtr(
    new ActionServerRecovery(
      private_nh_,
      name_action_recovery,
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionRecovery, this, _1),
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::cancelActionRecovery, this, _1),
      false));

  action_server_move_base_ptr_ = ActionServerMoveBasePtr(
    new ActionServerMoveBase(
      private_nh_,
      name_action_move_base,
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::callActionMoveBase, this, _1),
      boost::bind(&mbf_abstract_nav::AbstractNavigationServer::cancelActionMoveBase, this, _1),
      false));

  // XXX note that we don't start a dynamic reconfigure server, to avoid colliding with the one possibly created by
  // the base class. If none, it should call startDynamicReconfigureServer method to start the one defined here for
  // providing just the abstract server parameters
}

void AbstractNavigationServer::initializeServerComponents()
{
  planner_plugin_manager_.loadPlugins();
  controller_plugin_manager_.loadPlugins();
  recovery_plugin_manager_.loadPlugins();
}

AbstractNavigationServer::~AbstractNavigationServer()
{

}

void AbstractNavigationServer::callActionGetPath(ActionServerGetPath::GoalHandle goal_handle)
{
  const mbf_msgs::GetPathGoal &goal = *(goal_handle.getGoal().get());
  const geometry_msgs::Point &p = goal.target_pose.pose.position;

  std::string planner_name;
  if(!planner_plugin_manager_.getLoadedNames().empty())
  {
    planner_name = goal.planner.empty() ? planner_plugin_manager_.getLoadedNames().front() : goal.planner;
  }
  else
  {
    mbf_msgs::GetPathResult result;
    result.outcome = mbf_msgs::GetPathResult::INVALID_PLUGIN;
    result.message = "No plugins loaded at all!";
    ROS_WARN_STREAM_NAMED("get_path", result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  if(!planner_plugin_manager_.hasPlugin(planner_name))
  {
    mbf_msgs::GetPathResult result;
    result.outcome = mbf_msgs::GetPathResult::INVALID_PLUGIN;
    result.message = "No plugin loaded with the given name \"" + goal.planner + "\"!";
    ROS_ERROR_STREAM_NAMED("get_path", result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  mbf_abstract_core::AbstractPlanner::Ptr planner_plugin = planner_plugin_manager_.getPlugin(planner_name);
  ROS_DEBUG_STREAM_NAMED("get_path", "Start action \"get_path\" using planner \"" << planner_name
                        << "\" of type \"" << planner_plugin_manager_.getType(planner_name) << "\"");


  if(planner_plugin)
  {
    mbf_abstract_nav::AbstractPlannerExecution::Ptr planner_execution
        = newPlannerExecution(planner_name, planner_plugin);

    //start another planning action
    planner_action_.start(goal_handle, planner_execution);
  }
  else
  {
    mbf_msgs::GetPathResult result;
    result.outcome = mbf_msgs::GetPathResult::INTERNAL_ERROR;
    result.message = "Internal Error: \"planner_plugin\" pointer should not be a null pointer!";
    ROS_FATAL_STREAM_NAMED("get_path", result.message);
    goal_handle.setRejected(result, result.message);
  }
}

void AbstractNavigationServer::cancelActionGetPath(ActionServerGetPath::GoalHandle goal_handle)
{
  ROS_INFO_STREAM_NAMED("get_path", "Cancel action \"get_path\"");
  planner_action_.cancel(goal_handle);
}

void AbstractNavigationServer::callActionExePath(ActionServerExePath::GoalHandle goal_handle)
{
  const mbf_msgs::ExePathGoal &goal = *(goal_handle.getGoal().get());

  std::string controller_name;
  if(!controller_plugin_manager_.getLoadedNames().empty())
  {
    controller_name = goal.controller.empty() ? controller_plugin_manager_.getLoadedNames().front() : goal.controller;
  }
  else
  {
    mbf_msgs::ExePathResult result;
    result.outcome = mbf_msgs::ExePathResult::INVALID_PLUGIN;
    result.message = "No plugins loaded at all!";
    ROS_WARN_STREAM_NAMED("exe_path", result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  if(!controller_plugin_manager_.hasPlugin(controller_name))
  {
    mbf_msgs::ExePathResult result;
    result.outcome = mbf_msgs::ExePathResult::INVALID_PLUGIN;
    result.message = "No plugin loaded with the given name \"" + goal.controller + "\"!";
    ROS_ERROR_STREAM_NAMED("exe_path", result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  mbf_abstract_core::AbstractController::Ptr controller_plugin = controller_plugin_manager_.getPlugin(controller_name);
  ROS_DEBUG_STREAM_NAMED("exe_path", "Start action \"exe_path\" using controller \"" << controller_name
                        << "\" of type \"" << controller_plugin_manager_.getType(controller_name) << "\"");


  if(controller_plugin)
  {
    mbf_abstract_nav::AbstractControllerExecution::Ptr controller_execution
        = newControllerExecution(controller_name, controller_plugin);

    // starts another controller action
    controller_action_.start(goal_handle, controller_execution);
  }
  else
  {
    mbf_msgs::ExePathResult result;
    result.outcome = mbf_msgs::ExePathResult::INTERNAL_ERROR;
    result.message = "Internal Error: \"controller_plugin\" pointer should not be a null pointer!";
    ROS_FATAL_STREAM_NAMED("exe_path", result.message);
    goal_handle.setRejected(result, result.message);
  }
}

void AbstractNavigationServer::cancelActionExePath(ActionServerExePath::GoalHandle goal_handle)
{
  ROS_INFO_STREAM_NAMED("exe_path", "Cancel action \"exe_path\"");
  controller_action_.cancel(goal_handle);
}

void AbstractNavigationServer::callActionRecovery(ActionServerRecovery::GoalHandle goal_handle)
{
  const mbf_msgs::RecoveryGoal &goal = *(goal_handle.getGoal().get());

  std::string recovery_name;

  if(!recovery_plugin_manager_.getLoadedNames().empty())
  {
    recovery_name = goal.behavior.empty() ? recovery_plugin_manager_.getLoadedNames().front() : goal.behavior;
  }
  else
  {
    mbf_msgs::RecoveryResult result;
    result.outcome = mbf_msgs::RecoveryResult::INVALID_PLUGIN;
    result.message = "No plugins loaded at all!";
    ROS_WARN_STREAM_NAMED("recovery", result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  if(!recovery_plugin_manager_.hasPlugin(recovery_name))
  {
    mbf_msgs::RecoveryResult result;
    result.outcome = mbf_msgs::RecoveryResult::INVALID_PLUGIN;
    result.message = "No plugin loaded with the given name \"" + goal.behavior + "\"!";
    ROS_ERROR_STREAM_NAMED("recovery", result.message);
    goal_handle.setRejected(result, result.message);
    return;
  }

  mbf_abstract_core::AbstractRecovery::Ptr recovery_plugin = recovery_plugin_manager_.getPlugin(recovery_name);
  ROS_DEBUG_STREAM_NAMED("recovery", "Start action \"recovery\" using recovery \"" << recovery_name
                        << "\" of type \"" << recovery_plugin_manager_.getType(recovery_name) << "\"");


  if(recovery_plugin)
  {
    mbf_abstract_nav::AbstractRecoveryExecution::Ptr recovery_execution
        = newRecoveryExecution(recovery_name, recovery_plugin);

    recovery_action_.start(goal_handle, recovery_execution);
  }
  else
  {
    mbf_msgs::RecoveryResult result;
    result.outcome = mbf_msgs::RecoveryResult::INTERNAL_ERROR;
    result.message = "Internal Error: \"recovery_plugin\" pointer should not be a null pointer!";
    ROS_FATAL_STREAM_NAMED("recovery", result.message);
    goal_handle.setRejected(result, result.message);
  }
}

void AbstractNavigationServer::cancelActionRecovery(ActionServerRecovery::GoalHandle goal_handle)
{
  ROS_INFO_STREAM_NAMED("recovery", "Cancel action \"recovery\"");
  recovery_action_.cancel(goal_handle);
}

void AbstractNavigationServer::callActionMoveBase(ActionServerMoveBase::GoalHandle goal_handle)
{
  ROS_DEBUG_STREAM_NAMED("move_base", "Start action \"move_base\"");
  move_base_action_.start(goal_handle);
}

void AbstractNavigationServer::cancelActionMoveBase(ActionServerMoveBase::GoalHandle goal_handle)
{
  ROS_INFO_STREAM_NAMED("move_base", "Cancel action \"move_base\"");
  move_base_action_.cancel();
  ROS_DEBUG_STREAM_NAMED("move_base", "Cancel action \"move_base\" completed");
}

mbf_abstract_nav::AbstractPlannerExecution::Ptr AbstractNavigationServer::newPlannerExecution(
    const std::string &plugin_name,
    const mbf_abstract_core::AbstractPlanner::Ptr &plugin_ptr)
{
  return boost::make_shared<mbf_abstract_nav::AbstractPlannerExecution>(plugin_name, plugin_ptr,
                                                                        robot_info_, last_config_);
}

mbf_abstract_nav::AbstractControllerExecution::Ptr AbstractNavigationServer::newControllerExecution(
    const std::string &plugin_name,
    const mbf_abstract_core::AbstractController::Ptr &plugin_ptr)
{
  return boost::make_shared<mbf_abstract_nav::AbstractControllerExecution>(plugin_name, plugin_ptr, robot_info_,
                                                                           vel_pub_, last_config_);
}

mbf_abstract_nav::AbstractRecoveryExecution::Ptr AbstractNavigationServer::newRecoveryExecution(
    const std::string &plugin_name,
    const mbf_abstract_core::AbstractRecovery::Ptr &plugin_ptr)
{
  return boost::make_shared<mbf_abstract_nav::AbstractRecoveryExecution>(plugin_name, plugin_ptr,
                                                                         robot_info_, last_config_);
}

void AbstractNavigationServer::startActionServers()
{
  action_server_get_path_ptr_->start();
  action_server_exe_path_ptr_->start();
  action_server_recovery_ptr_->start();
  action_server_move_base_ptr_->start();
}

void AbstractNavigationServer::startDynamicReconfigureServer()
{
  // dynamic reconfigure server
  dsrv_ = boost::make_shared<dynamic_reconfigure::Server<mbf_abstract_nav::MoveBaseFlexConfig> >(private_nh_);
  dsrv_->setCallback(boost::bind(&AbstractNavigationServer::reconfigure, this, _1, _2));
}

void AbstractNavigationServer::reconfigure(
  mbf_abstract_nav::MoveBaseFlexConfig &config, uint32_t level)
{
  boost::lock_guard<boost::mutex> guard(configuration_mutex_);

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
  planner_action_.reconfigure(config, level);
  controller_action_.reconfigure(config, level);
  recovery_action_.reconfigure(config, level);
  move_base_action_.reconfigure(config, level);

  last_config_ = config;
}

void AbstractNavigationServer::stop(){
  planner_action_.cancelAll();
  controller_action_.cancelAll();
  recovery_action_.cancelAll();
  move_base_action_.cancel();
}

} /* namespace mbf_abstract_nav */
