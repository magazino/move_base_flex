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
 *  wrapper_global_planner.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_COSTMAP_NAV__WRAPPER_GLOBAL_PLANNER_H_
#define MBF_COSTMAP_NAV__WRAPPER_GLOBAL_PLANNER_H_

#include <nav_core/base_global_planner.h>
#include "mbf_costmap_core/costmap_planner.h"

namespace mbf_nav_core_wrapper {
  /**
   * @class CostmapPlanner
   * @brief Provides an interface for global planners used in navigation.
   * All global planners written to work as MBF plugins must adhere to this interface. Alternatively, this
   * class can also operate as a wrapper for old API nav_core-based plugins, providing backward compatibility.
   */
  class WrapperGlobalPlanner : public mbf_costmap_core::CostmapPlanner{
    public:

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
       *        in x and y before failing
       * @param plan The plan... filled by the planner
       * @param cost The cost for the the plan
       * @param message Optional more detailed outcome as a string
       * @return Result code as described on GetPath action result, As this is a wrapper to the nav_core,
       *         only 0 (SUCCESS) and 50 (FAILURE) are supported
       */
      virtual uint32_t makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                double tolerance, std::vector<geometry_msgs::PoseStamped> &plan, double &cost,
                                std::string &message);

      /**
       * @brief Requests the planner to cancel, e.g. if it takes too much time.
       * @remark New on MBF API
       * @return True if a cancel has been successfully requested, false if not implemented.
       */
      virtual bool cancel();

      /**
       * @brief Initialization function for the CostmapPlanner
       * @param name The name of this planner
       * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      virtual void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

      /**
       * @brief Public constructor used for handling a nav_core-based plugin
       * @param plugin Backward compatible plugin
       */
      WrapperGlobalPlanner(boost::shared_ptr< nav_core::BaseGlobalPlanner > plugin);

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~WrapperGlobalPlanner();

    private:
      boost::shared_ptr< nav_core::BaseGlobalPlanner > nav_core_plugin_;
  };
}  /* namespace mbf_nav_core_wrapper */

#endif  /* MBF_COSTMAP_NAV__WRAPPER_GLOBAL_PLANNER_H_ */
