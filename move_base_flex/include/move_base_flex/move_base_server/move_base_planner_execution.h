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

#ifndef MOVE_BASE_FLEX__MOVE_BASE_PLANNER_EXECUTION_H_
#define MOVE_BASE_FLEX__MOVE_BASE_PLANNER_EXECUTION_H_

#include <costmap_2d/costmap_2d_ros.h>
#include "nav_core/base_global_planner.h"
#include "move_base_flex/abstract_server/abstract_planner_execution.h"

namespace move_base_flex
{
/**
 * @brief The MoveBasePlannerExecution binds a global costmap to the AbstractPlannerExecution and uses the
 *        nav_core/BaseGlobalPlanner class as base plugin interface. This class makes move_base_flex compatible to the old move_base.
 *
 * @ingroup planner_execution move_base_server
 */
class MoveBasePlannerExecution : public AbstractPlannerExecution<nav_core::BaseGlobalPlanner>
{
public:
  typedef boost::shared_ptr<costmap_2d::Costmap2DROS> CostmapPtr;

  /**
   * @brief Constructor
   * @param condition Thread sleep condition variable, to wake up connected threads
   * @param costmap Shared pointer to the costmap.
   */
  MoveBasePlannerExecution(boost::condition_variable &condition, CostmapPtr &costmap);

  /**
   * @brief Destructor
   */
  virtual ~MoveBasePlannerExecution();

protected:

  /**
   * @brief The main run method, a thread will execute this method. It contains the main planner execution loop. This
   *        method overwrites (and extends) the base class method.
   */
  virtual void run();

private:

  /**
   * @brief Initializes the global planner plugin with its name and pointer to the costmap
   */
  virtual void initPlannerPlugin();

  //! Shared pointer to the global planner costmap
  CostmapPtr &costmap_ptr_;
};

} /* namespace move_base_flex */

#endif /* MOVE_BASE_FLEX__MOVE_BASE_PLANNER_EXECUTION_H_ */
