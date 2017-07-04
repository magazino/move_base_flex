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
 *  move_base_navigation_server.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MOVE_BASE_FLEX__MOVE_BASE_NAVIGATION_SERVER_H_
#define MOVE_BASE_FLEX__MOVE_BASE_NAVIGATION_SERVER_H_

#include <nav_msgs/GetPlan.h>
#include <std_srvs/Empty.h>

#include <move_base_flex_msgs/CheckPose.h>

#include "move_base_flex/abstract_server/abstract_navigation_server.h"
#include "move_base_planner_execution.h"
#include "move_base_controller_execution.h"
#include "move_base_recovery_execution.h"

namespace move_base_flex
{
/**
 * @defgroup move_base_server Move Base Server
 *           Classes belonging to the Move Base Server level.
 */

/**
 * @ingroup navigation_server move_base_server
 */
class MoveBaseNavigationServer : public AbstractNavigationServer<nav_core::BaseLocalPlanner,
                                                                nav_core::BaseGlobalPlanner,
                                                                nav_core::RecoveryBehavior>
{
public:

  typedef boost::shared_ptr<costmap_2d::Costmap2DROS> CostmapPtr;

  MoveBaseNavigationServer(const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr);

  virtual ~MoveBaseNavigationServer();

protected:

  void checkActivateCostmaps();

  void checkDeactivateCostmaps();

  bool callServiceCheckPoseCost(move_base_flex_msgs::CheckPose::Request &request,
                                move_base_flex_msgs::CheckPose::Response &response);

  bool callServiceMakePlan(nav_msgs::GetPlan::Request &request, nav_msgs::GetPlan::Response &response);

  bool callServiceClearCostmaps(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  // overwrites and extends the base class action methods
  virtual void callActionGetPath(const move_base_flex_msgs::GetPathGoalConstPtr &goal);

  virtual void callActionExePath(const move_base_flex_msgs::ExePathGoalConstPtr &goal);

  virtual void callActionRecovery(const move_base_flex_msgs::RecoveryGoalConstPtr &goal);

  virtual void callActionMoveBase(const move_base_flex_msgs::MoveBaseGoalConstPtr &goal);

  // overwrites and extends the base class reconfigure method
  virtual void reconfigure(move_base_flex::MoveBaseFlexConfig &config, uint32_t level);

  void run();

  CostmapPtr costmap_local_planner_ptr_;
  CostmapPtr costmap_global_planner_ptr_;

  bool local_costmap_active_;
  bool global_costmap_active_;

  ros::ServiceServer check_pose_cost_srv_;
  ros::ServiceServer clear_costmaps_srv_;
  ros::ServiceServer make_plan_srv_;

  // stop updating costmaps when not planning or moving
  bool shutdown_costmaps_;

private:

};

} /* namespace move_base_flex */

#endif /* MOVE_BASE_FLEX__MOVE_BASE_NAVIGATION_SERVER_H_ */
