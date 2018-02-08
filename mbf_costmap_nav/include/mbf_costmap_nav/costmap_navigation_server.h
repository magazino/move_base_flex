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

#ifndef MBF_COSTMAP_NAV__COSTMAP_NAVIGATION_SERVER_H_
#define MBF_COSTMAP_NAV__COSTMAP_NAVIGATION_SERVER_H_

#include <mbf_abstract_nav/abstract_navigation_server.h>

#include "costmap_planner_execution.h"
#include "costmap_controller_execution.h"
#include "costmap_recovery_execution.h"

#include <mbf_costmap_nav/MoveBaseFlexConfig.h>
#include <std_srvs/Empty.h>
#include <mbf_msgs/CheckPose.h>

namespace mbf_costmap_nav
{
/**
 * @defgroup move_base_server Move Base Server
 * @brief Classes belonging to the Move Base Server level.
 */


typedef boost::shared_ptr<dynamic_reconfigure::Server<mbf_costmap_nav::MoveBaseFlexConfig> > DynamicReconfigureServerCostmapNav;

/**
 * @brief The CostmapNavigationServer makes Move Base Flex backwards compatible to the old move_base. It combines the
 *        execution classes which use the nav_core/BaseLocalPlanner, nav_core/BaseCostmapPlanner and the
 *        nav_core/RecoveryBehavior base classes as plugin interfaces. These plugin interface are the same for the
 *        old move_base
 *
 * @ingroup navigation_server move_base_server
 */
class CostmapNavigationServer : public mbf_abstract_nav::AbstractNavigationServer
{
public:

  typedef boost::shared_ptr<costmap_2d::Costmap2DROS> CostmapPtr;

  /**
   * @brief Constructor
   * @param tf_listener_ptr Shared pointer to a common TransformListener
   */
  CostmapNavigationServer(const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr);

  /**
   * @brief Destructor
   */
  virtual ~CostmapNavigationServer();

private:

  /**
   * @brief Check whether the costmaps should be activated.
   */
  void checkActivateCostmaps();

  /**
   * @brief Check whether the costmaps should and could be deactivated
   */
  void checkDeactivateCostmaps();

  /**
   * @brief Timer-triggered deactivation of both costmaps.
   */
  void deactivateCostmaps(const ros::TimerEvent &event);

  /**
   * @brief Callback method for the check_pose_cost service
   * @param request Request object, see the mbf_msgs/CheckPose service definition file.
   * @param response Response object, see the mbf_msgs/CheckPose service definition file.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceCheckPoseCost(mbf_msgs::CheckPose::Request &request,
                                mbf_msgs::CheckPose::Response &response);

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
   *        definitions in mbf_msgs.
   */
  virtual void callActionGetPath(const mbf_msgs::GetPathGoalConstPtr &goal);

  /**
   * @brief ExePath action execution method. This method will be called if the action server receives a goal. It
   *        extends the base class method by calling the checkActivateCostmaps() and checkDeactivateCostmaps().
   * @param goal SimpleActionServer goal containing all necessary parameters for the action execution. See the action
   *        definitions in mbf_msgs.
   */
  virtual void callActionExePath(const mbf_msgs::ExePathGoalConstPtr &goal);

  /**
   * @brief Recovery action execution method. This method will be called if the action server receives a goal. It
   *        extends the base class method by calling the checkActivateCostmaps() and checkDeactivateCostmaps().
   * @param goal SimpleActionServer goal containing all necessary parameters for the action execution. See the action
   *        definitions in mbf_msgs.
   */
  virtual void callActionRecovery(const mbf_msgs::RecoveryGoalConstPtr &goal);

  /**
   * @brief Reconfiguration method called by dynamic reconfigure.
   * @param config Configuration parameters. See the MoveBaseFlexConfig definition.
   * @param level bit mask, which parameters are set.
   */
  void reconfigure(mbf_costmap_nav::MoveBaseFlexConfig &config, uint32_t level);

  //! Dynamic reconfigure server for the mbf_costmap2d_specific part
  DynamicReconfigureServerCostmapNav dsrv_costmap_;

  //! last configuration save
  mbf_costmap_nav::MoveBaseFlexConfig last_config_;

  //! the default parameter configuration save
  mbf_costmap_nav::MoveBaseFlexConfig default_config_;

  //! true, if the dynamic reconfigure has been setup.
  bool setup_reconfigure_;

  //! Shared pointer to the common local costmap
  CostmapPtr local_costmap_ptr_;

  //! Shared pointer to the common global costmap
  CostmapPtr global_costmap_ptr_;

  //! true, if the local costmap is active
  bool local_costmap_active_;

  //! true, if the global costmap is active
  bool global_costmap_active_;

  //! Service Server for the check_pose_cost service
  ros::ServiceServer check_pose_cost_srv_;

  //! Service Server for the clear_costmap service
  ros::ServiceServer clear_costmaps_srv_;

  //! Stop updating costmaps when not planning or controlling, if true
  bool shutdown_costmaps_;
  ros::Timer shutdown_costmaps_timer_;    //!< delayed shutdown timer
  ros::Duration shutdown_costmaps_delay_; //!< delayed shutdown delay

};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_NAVIGATION_SERVER_H_ */
