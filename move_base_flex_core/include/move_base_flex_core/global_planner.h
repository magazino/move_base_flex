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
#ifndef MBF_CORE_GLOBAL_PLANNER_H
#define MBF_CORE_GLOBAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include "move_base_flex_core/abstract_global_planner.h"

namespace move_base_flex_core {
  /**
   * @class GlobalPlanner
   * @brief Provides an interface for global planners used in navigation.
   * All global planners written to work as MBF plugins must adhere to this interface. Alternatively, this
   * class can also operate as a wrapper for old API nav_core-based plugins, providing backward compatibility.
   */
  class GlobalPlanner : public AbstractGlobalPlanner{
    public:

      typedef boost::shared_ptr< ::move_base_flex_core::GlobalPlanner > Ptr;

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
       *        in x and y before failing
       * @param plan The plan... filled by the planner
       * @param cost The cost for the the plan
       * @param message Optional more detailed outcome as a string
       * @return Result code as described on GetPath action result:
       *         SUCCESS         = 0
       *         1..9 are reserved as plugin specific non-error results
       *         FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
       *         CANCELED        = 51
       *         INVALID_START   = 52
       *         INVALID_GOAL    = 53
       *         NO_PATH_FOUND   = 54
       *         PAT_EXCEEDED    = 55
       *         EMPTY_PATH      = 56
       *         TF_ERROR        = 57
       *         NOT_INITIALIZED = 58
       *         INVALID_PLUGIN  = 59
       *         INTERNAL_ERROR  = 60
       *         71..99 are reserved as plugin specific errors
       */
      virtual uint32_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                                std::string& message)
      {
        if (!backward_compatible_plugin)
          throw std::runtime_error("MBF API makePlan method not overridden nor backward compatible plugin provided");

        bool success = backward_compatible_plugin->makePlan(start, goal, plan, cost);
        message = success ? "Plan found" : "Planner failed";
        return success ? 0 : 50;  // SUCCESS | FAILURE
      }

      /**
       * @brief Requests the planner to cancel, e.g. if it takes to much time.
       * @remark New on MBF API
       * @return True if a cancel has been successfully requested, false if not implemented.
       */
      virtual bool cancel()
      {
        return false;
      }

      /**
       * @brief Initialization function for the GlobalPlanner
       * @param name The name of this planner
       * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
      {
        if (!backward_compatible_plugin)
          throw std::runtime_error("MBF API initialize method not overridden nor backward compatible plugin provided");

        backward_compatible_plugin->initialize(name, costmap_ros);
      }

      /**
       * @brief Public constructor used for handling a nav_core-based plugin
       * @param plugin Backward compatible plugin
       */
      GlobalPlanner(boost::shared_ptr< nav_core::BaseGlobalPlanner > plugin)
      {
        backward_compatible_plugin = plugin;

        if (!backward_compatible_plugin)
          throw std::runtime_error("Invalid backward compatible plugin provided");
      }

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~GlobalPlanner(){}

    protected:
      GlobalPlanner(){}

    private:
      boost::shared_ptr< nav_core::BaseGlobalPlanner > backward_compatible_plugin;
  };
};  // namespace move_base_flex_core

#endif  // MBF_CORE_GLOBAL_PLANNER_H
