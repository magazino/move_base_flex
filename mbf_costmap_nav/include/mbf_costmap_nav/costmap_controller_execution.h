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

#include <costmap_2d/costmap_2d_ros.h>
#include <mbf_costmap_nav/MoveBaseFlexConfig.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <mbf_abstract_nav/abstract_controller_execution.h>

namespace mbf_costmap_nav
{
/**
 * @brief The CostmapControllerExecution binds a local costmap to the AbstractControllerExecution and uses the
 *        nav_core/BaseLocalPlanner class as base plugin interface. This class makes move_base_flex compatible to
 *        the old move_base.
 *
 * @ingroup controller_execution move_base_server
 */
class CostmapControllerExecution : public mbf_abstract_nav::AbstractControllerExecution
{
public:

  typedef boost::shared_ptr<costmap_2d::Costmap2DROS> CostmapPtr;

  /**
   * @brief Constructor
   * @param condition Thread sleep condition variable, to wake up connected threads
   * @param tf_listener_ptr Shared pointer to a common tf listener
   * @param costmap_ptr Shared pointer to the costmap.
   */
  CostmapControllerExecution(
      const mbf_costmap_core::CostmapController::Ptr &controller_ptr,
      const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
      CostmapPtr &costmap_ptr,
      const MoveBaseFlexConfig &config,
      boost::function<void()> setup_fn,
      boost::function<void()> cleanup_fn);

  /**
   * @brief Destructor
   */
  virtual ~CostmapControllerExecution();

protected:

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
      const geometry_msgs::PoseStamped& robot_pose,
      const geometry_msgs::TwistStamped& robot_velocity,
      geometry_msgs::TwistStamped& vel_cmd,
      std::string& message);

private:

  mbf_abstract_nav::MoveBaseFlexConfig toAbstract(const MoveBaseFlexConfig &config);

  //! costmap for 2d navigation planning
  CostmapPtr &costmap_ptr_;

  //! Whether to lock costmap before calling the controller (see issue #4 for details)
  bool lock_costmap_;

  //! name of the controller plugin assigned by the class loader
  std::string controller_name_;
};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_CONTROLLER_EXECUTION_H_ */
