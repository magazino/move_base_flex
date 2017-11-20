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
namespace mbf_msgs = move_base_flex_msgs;

#include "move_base_flex/abstract_server/abstract_navigation_server.h"
#include "move_base_planner_execution.h"
#include "move_base_controller_execution.h"
#include "move_base_recovery_execution.h"

namespace move_base_flex
{
/**
 * @defgroup move_base_server Move Base Server
 * @brief Classes belonging to the Move Base Server level.
 */

/**
 * @brief The MoveBaseNavigationServer makes Move Base Flex backwards compatible to the old move_base. It combines the
 *        execution classes which use the nav_core/BaseLocalPlanner, nav_core/BaseGlobalPlanner and the 
 *        nav_core/RecoveryBehavior base classes as plugin interfaces. These plugin interface are the same for the 
 *        old move_base
 *
 * @ingroup navigation_server move_base_server
 */
class MoveBaseNavigationServer : public AbstractNavigationServer<move_base_flex_core::LocalPlanner,
                                                                 move_base_flex_core::GlobalPlanner,
                                                                 move_base_flex_core::RecoveryBehavior>
{
public:

  typedef boost::shared_ptr<costmap_2d::Costmap2DROS> CostmapPtr;

  /**
   * @brief Constructor
   * @param tf_listener_ptr Shared pointer to a common TransformListener
   */
  MoveBaseNavigationServer(const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr);

  /**
   * @brief Destructor
   */
  virtual ~MoveBaseNavigationServer();

protected:

  /**
   * check whether the costmaps should be activated.
   */
  void checkActivateCostmaps();

  /**
   * @brief checks whether the costmaps should and could be deactivated
   */
  void checkDeactivateCostmaps();

  /**
   * @brief Callback method for the check_pose_cost service
   * @param request Request object, see the move_base_flex_msgs/CheckPose service definition file.
   * @param response Response object, see the move_base_flex_msgs/CheckPose service definition file.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceCheckPoseCost(mbf_msgs::CheckPose::Request &request,
                                mbf_msgs::CheckPose::Response &response);

  /**
   * @brief Callback method for the make_plan service
   * @param request Request object, see the nav_msgs/GetPlan service definition file.
   * @param response Response object, see the nav_msgs/GetPlan service definition file.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceMakePlan(nav_msgs::GetPlan::Request &request, nav_msgs::GetPlan::Response &response);

  /**
   * @brief Callback method for the make_plan service
   * @param request Empty request object.
   * @param response Empty response object.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceClearCostmaps(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  /**
   * @brief GetPath action execution method. This method will be called if the action server receives a goal. It
   *        extends the base class method by calling the checkActivateCostmaps() and checkDeactivateCostmaps().
   * @param goal SimpleActionServer goal containing all necessary parameters for the action execution. See the action
   *        definitions in move_base_flex_msgs.
   */
  virtual void callActionGetPath(const move_base_flex_msgs::GetPathGoalConstPtr &goal);

  /**
   * @brief ExePath action execution method. This method will be called if the action server receives a goal. It
   *        extends the base class method by calling the checkActivateCostmaps() and checkDeactivateCostmaps().
   * @param goal SimpleActionServer goal containing all necessary parameters for the action execution. See the action
   *        definitions in move_base_flex_msgs.
   */
  virtual void callActionExePath(const move_base_flex_msgs::ExePathGoalConstPtr &goal);

  /**
   * @brief Recovery action execution method. This method will be called if the action server receives a goal. It
   *        extends the base class method by calling the checkActivateCostmaps() and checkDeactivateCostmaps().
   * @param goal SimpleActionServer goal containing all necessary parameters for the action execution. See the action
   *        definitions in move_base_flex_msgs.
   */
  virtual void callActionRecovery(const move_base_flex_msgs::RecoveryGoalConstPtr &goal);

  /**
   *
   * @brief Reconfiguration method called by dynamic reconfigure. Overwrites and extends the base class reconfigure
   *        method.
   * @param config Configuration parameters. See the MoveBaseFlexConfig definition.
   * @param level bit mask, which parameters are set.
   */
  virtual void reconfigure(move_base_flex::MoveBaseFlexConfig &config, uint32_t level);

  //! Shared pointer to the common local costmap
  CostmapPtr costmap_local_planner_ptr_;

  //! Shared pointer to the common global costmap
  CostmapPtr costmap_global_planner_ptr_;

  //! true, if the local costmap is active
  bool local_costmap_active_;

  //! true, if the global costmap is active
  bool global_costmap_active_;

  //! Service Server for the check_pose_cost service
  ros::ServiceServer check_pose_cost_srv_;

  //! Service Server for the clear_costmap service
  ros::ServiceServer clear_costmaps_srv_;

  //! Service Server for the make_plan service
  ros::ServiceServer make_plan_srv_;

  //! stop updating costmaps when not planning or controlling, if true
  bool shutdown_costmaps_;

};

} /* namespace move_base_flex */

#endif /* MOVE_BASE_FLEX__MOVE_BASE_NAVIGATION_SERVER_H_ */
