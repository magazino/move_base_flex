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
 *  move_base_planner_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_COSTMAP_NAV__COSTMAP_PLANNER_EXECUTION_H_
#define MBF_COSTMAP_NAV__COSTMAP_PLANNER_EXECUTION_H_

#include <mbf_abstract_nav/abstract_planner_execution.h>
#include <mbf_costmap_core/costmap_planner.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace mbf_costmap_nav
{
/**
 * @brief The CostmapPlannerExecution binds a global costmap to the AbstractPlannerExecution and uses the
 *        nav_core/BaseCostmapPlanner class as base plugin interface. This class makes move_base_flex compatible to the old move_base.
 *
 * @ingroup planner_execution move_base_server
 */
class CostmapPlannerExecution : public mbf_abstract_nav::AbstractPlannerExecution
{
public:
  typedef boost::shared_ptr<costmap_2d::Costmap2DROS> CostmapPtr;

  /**
   * @brief Constructor
   * @param condition Thread sleep condition variable, to wake up connected threads
   * @param costmap Shared pointer to the costmap.
   */
  CostmapPlannerExecution(boost::condition_variable &condition, CostmapPtr &costmap);

  /**
   * @brief Destructor
   */
  virtual ~CostmapPlannerExecution();

private:

  /**
   * @brief Loads the plugin associated with the given planner_type parameter.
   * @param planner_type The type of the planner plugin to load.
   * @return true, if the local planner plugin was successfully loaded.
   */
  virtual mbf_abstract_core::AbstractPlanner::Ptr loadPlannerPlugin(const std::string& planner_type);

  /**
   * @brief Initializes the controller plugin with its name and pointer to the costmap
   * @param name The name of the planner
   * @param planner_ptr pointer to the planner object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initPlugin(
      const std::string& name,
      const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr
  );

  /**
   * @brief calls the planner plugin to make a plan from the start pose to the goal pose with the given tolerance,
   *        if a goal tolerance is enabled in the planner plugin.
   * @param start The start pose for planning
   * @param goal The goal pose for planning
   * @param tolerance The goal tolerance
   * @param plan The computed plan by the plugin
   * @param cost The computed costs for the corresponding plan
   * @param message An optional message which should correspond with the returned outcome
   * @return An outcome number, see also the action definition in the GetPath.action file
   */
  virtual uint32_t makePlan(
      const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr,
      const geometry_msgs::PoseStamped start,
      const geometry_msgs::PoseStamped goal,
      double tolerance,
      std::vector<geometry_msgs::PoseStamped> &plan,
      double &cost,
      std::string &message);

  //! Shared pointer to the global planner costmap
  CostmapPtr &costmap_ptr_;

  //! Whether to lock costmap before calling the planner (see issue #4 for details)
  bool lock_costmap_;

  //! Name of the planner assigned by the class loader
  std::string planner_name_;
};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_PLANNER_EXECUTION_H_ */
