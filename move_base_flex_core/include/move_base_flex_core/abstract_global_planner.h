/*
 *  Copyright 2017, Sebastian Pütz
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
 *  abstract_global_planner.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_CORE_ABSTRACT_GLOBAL_PLANNER_H_
#define MBF_CORE_ABSTRACT_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>

namespace move_base_flex_core
{

  class AbstractGlobalPlanner{

    public:
      typedef boost::shared_ptr< ::move_base_flex_core::AbstractGlobalPlanner > Ptr;

      /**
       * @brief Destructor
       */
      virtual ~AbstractGlobalPlanner(){};

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param waypoints Intermediate poses the plan should pass by
       * @param waypoints_tolerance If a waypoint is obstructed, how many meters the plan can pass away of it
       *        Defaults to goal_tolerance. If only one value is provided, it is used for all the waypoints
       * @param goal_tolerance If the goal is obstructed, how many meters the planner can relax the constraint
       *        in x and y before failing
       * @param plan The plan... filled by the planner
       * @param cost The cost for the the plan
       * @param message Optional more detailed outcome as a string
       * @return Result code as described on GetPath action result:
       *         SUCCESS        = 0
       *         1..9 are reserved as plugin specific non-error results
       *         NO_PATH_FOUND  = 40
       *         CANCELED       = 41
       *         PAT_EXCEEDED   = 42
       *         EMPTY_PATH     = 43
       *         INTERNAL_ERROR = 44
       *         51..59 are reserved as plugin specific errors
       */
      virtual uint8_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               const std::vector<geometry_msgs::PoseStamped>& waypoints,
                               const std::vector<double> waypoints_tolerance, double goal_tolerance,
                               std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                               std::string& message) = 0;

      /**
       * @brief Requests the planner to cancel, e.g. if it takes to much time.
       * @return True if a cancel has been successfully requested, false if not implemented.
       */
      virtual bool cancel() = 0;

    protected:
      /**
       * @brief Constructor
       */
      AbstractGlobalPlanner(){};
  };
} /* namespace move_base_flex_core */

#endif /* abstract_global_planner.h */
