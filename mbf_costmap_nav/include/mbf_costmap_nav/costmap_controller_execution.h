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
 *  costmap_controller_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_COSTMAP_NAV__COSTMAP_CONTROLLER_EXECUTION_H_
#define MBF_COSTMAP_NAV__COSTMAP_CONTROLLER_EXECUTION_H_

#include <mbf_abstract_nav/abstract_controller_execution.h>
#include <mbf_costmap_core/costmap_controller.h>

#include "mbf_costmap_nav/MoveBaseFlexConfig.h"
#include "mbf_costmap_nav/costmap_wrapper.h"


namespace mbf_costmap_nav
{
/**
 * @brief The CostmapControllerExecution binds a local costmap to the AbstractControllerExecution and uses the
 *        nav_core/BaseLocalPlanner class as base plugin interface.
 * This class makes move_base_flex compatible to the old move_base.
 *
 * @ingroup controller_execution move_base_server
 */
class CostmapControllerExecution : public mbf_abstract_nav::AbstractControllerExecution
{
public:

  /**
   * @brief Constructor.
   * @param controller_name Name of the controller to use.
   * @param controller_ptr Shared pointer to the plugin to use.
   * @param robot_info Current robot state
   * @param vel_pub Velocity commands publisher.
   * @param goal_pub Goal pose publisher (just vor visualization).
   * @param costmap_ptr Shared pointer to the local costmap.
   * @param config Current server configuration (dynamic).
   */
  CostmapControllerExecution(
      const std::string &controller_name,
      const mbf_costmap_core::CostmapController::Ptr &controller_ptr,
      const mbf_utility::RobotInformation &robot_info,
      const ros::Publisher &vel_pub,
      const ros::Publisher &goal_pub,
      const CostmapWrapper::Ptr &costmap_ptr,
      const MoveBaseFlexConfig &config);

  /**
   * @brief Destructor
   */
  virtual ~CostmapControllerExecution();

private:

  /**
   * @brief Implementation-specific setup function, called right before execution.
   * This method overrides abstract execution empty implementation with underlying map-specific setup code.
   */
  void preRun()
  {
    costmap_ptr_->checkActivate();
  };

  /**
   * @brief Implementation-specific cleanup function, called right after execution.
   * This method overrides abstract execution empty implementation with underlying map-specific cleanup code.
   */
  void postRun()
  {
    costmap_ptr_->checkDeactivate();
  };

  /**
   * @brief Implementation-specific safety check, called during execution to ensure it's safe to drive.
   * This method overrides abstract execution empty implementation with underlying map-specific checks,
   * more precisely if controller costmap is current.
   * @return True if costmap is current, false otherwise.
   */
  bool safetyCheck();

  /**
   * @brief Request plugin for a new velocity command. We override this method so we can lock the local costmap
   *        before calling the planner.
   * @param pose the current pose of the robot.
   * @param velocity the current velocity of the robot.
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
   * @param message Optional more detailed outcome as a string.
   * @return Result code as described on ExePath action result and plugin's header.
   */
  virtual uint32_t computeVelocityCmd(
      const geometry_msgs::PoseStamped &robot_pose,
      const geometry_msgs::TwistStamped &robot_velocity,
      geometry_msgs::TwistStamped &vel_cmd,
      std::string &message);

  mbf_abstract_nav::MoveBaseFlexConfig toAbstract(const MoveBaseFlexConfig &config);

  //! Shared pointer to thr local costmap
  const CostmapWrapper::Ptr &costmap_ptr_;

  //! Whether to lock costmap before calling the controller (see issue #4 for details)
  bool lock_costmap_;

  //! name of the controller plugin assigned by the class loader
  std::string controller_name_;
};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_CONTROLLER_EXECUTION_H_ */
