/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef MBF_CORE_BASE_LOCAL_PLANNER_H
#define MBF_CORE_BASE_LOCAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include "move_base_flex_core/abstract_local_planner.h"

namespace move_base_flex_core {
  /**
   * @class BaseLocalPlanner
   * @brief Provides an interface for local planners used in navigation.
   * All local planners written to work as MBF plugins must adhere to this interface. Alternatively, this
   * class can also operate as a wrapper for old API nav_core-based plugins, providing backward compatibility.
   */
  class BaseLocalPlanner : public AbstractLocalPlanner{
    public:

      typedef boost::shared_ptr< ::move_base_flex_core::BaseLocalPlanner > Ptr;

      /**
       * @brief Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @remark New on MBF API; replaces version without output code and message
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @param message Optional more detailed outcome as a string
       * @return Result code as described on ExePath action result:
       *         SUCCESS        = 0
       *         1..9 are reserved as plugin specific non-error results
       *         NO_VALID_CMD   = 100
       *         CANCELED       = 101
       *         PAT_EXCEEDED   = 102
       *         COLLISION      = 103
       *         OSCILLATION    = 104
       *         ROBOT_STUCK    = 105
       *         MISSED_GOAL    = 106
       *         MISSED_PATH    = 107
       *         BLOCKED_PATH   = 108
       *         INVALID_PATH   = 109
       *         INTERNAL_ERROR = 110
       *         111..149 are reserved as plugin specific errors
       */
      virtual uint32_t computeVelocityCommands(geometry_msgs::TwistStamped& cmd_vel, std::string& message)
      {
        if (!backward_compatible_plugin)
          throw std::runtime_error("MBF API computeVelocityCommands method not overridden \
                                   nor backward compatible plugin provided");

        return  backward_compatible_plugin->computeVelocityCommands(cmd_vel.twist) ? 0 : 60;  // SUCCESS | NO_VALID_CMD
      }

      /**
       * @brief Check if the goal pose has been achieved by the local planner
       * @return True if achieved, false otherwise
       */
      virtual bool isGoalReached()
      {
        if (!backward_compatible_plugin)
          throw std::runtime_error("MBF API isGoalReached method not overridden \
                                   nor backward compatible plugin provided");

        return backward_compatible_plugin->isGoalReached();
      }

      /**
       * @brief Check if the goal pose has been achieved by the local planner within tolerance limits
       * @remark New on MBF API
       * @param xy_tolerance Distance tolerance in meters
       * @param yaw_tolerance Heading tolerance in radians
       * @return True if achieved, false otherwise
       */
      virtual bool isGoalReached(double xy_tolerance, double yaw_tolerance)
      {
        return isGoalReached();
      }

      /**
       * @brief  Set the plan that the local planner is following
       * @param plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
      {
        if (!backward_compatible_plugin)
          throw std::runtime_error("MBF API setPlan method not overridden \
                                   nor backward compatible plugin provided");

        return backward_compatible_plugin->setPlan(plan);
      }

      /**
       * @brief Requests the planner to cancel, e.g. if it takes to much time
       * @remark New on MBF API
       * @return True if a cancel has been successfully requested, false if not implemented.
       */
      virtual bool cancel()
      {
        return false;
      }

      /**
       * @brief Constructs the local planner
       * @param name The name to give this instance of the local planner
       * @param tf A pointer to a transform listener
       * @param costmap_ros The cost map to use for assigning costs to local plans
       */
      virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
      {
        if (!backward_compatible_plugin)
          throw std::runtime_error("MBF API initialize method not overridden \
                                   nor backward compatible plugin provided");

        backward_compatible_plugin->initialize(name, tf, costmap_ros);
      }

      /**
       * @brief Public constructor used for handling a nav_core-based plugin
       * @param plugin Backward compatible plugin
       */
      BaseLocalPlanner(boost::shared_ptr< nav_core::BaseLocalPlanner > plugin)
      {
        backward_compatible_plugin = plugin;

        if (!backward_compatible_plugin)
          throw std::runtime_error("Invalid backward compatible plugin provided");
      }

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~BaseLocalPlanner(){}

    protected:
      BaseLocalPlanner(){}

    private:
      boost::shared_ptr< nav_core::BaseLocalPlanner > backward_compatible_plugin;
  };
};  // namespace move_base_flex_core

#endif  // MBF_CORE_BASE_LOCAL_PLANNER_H
