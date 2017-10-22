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
 *  move_base_recovery_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MOVE_BASE_FLEX__MOVE_BASE_RECOVERY_EXECUTION_H_
#define MOVE_BASE_FLEX__MOVE_BASE_RECOVERY_EXECUTION_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <move_base_flex_core/recovery_behavior.h>
#include "move_base_flex/abstract_server/abstract_recovery_execution.h"


namespace move_base_flex
{
/**
 * @brief The MoveBaseRecoveryExecution binds a local and a global costmap to the AbstractRecoveryExecution and uses the
 *        nav_core/RecoveryBehavior class as base plugin interface. This class makes move_base_flex compatible to the old move_base.
 *
 * @ingroup recovery_execution move_base_server
 */
class MoveBaseRecoveryExecution : public AbstractRecoveryExecution<move_base_flex_core::RecoveryBehavior>
{

public:
  typedef boost::shared_ptr<costmap_2d::Costmap2DROS> CostmapPtr;
  typedef boost::shared_ptr<MoveBaseRecoveryExecution> Ptr;

  /**
   * @brief Constructor
   * @param condition Thread sleep condition variable, to wake up connected threads
   * @param tf_listener_ptr Shared pointer to a common tf listener
   * @param global_costmap Shared pointer to the global costmap.
   * @param local_costmap Shared pointer to the local costmap.
   */
  MoveBaseRecoveryExecution(boost::condition_variable &condition,
                            const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
                            CostmapPtr &global_costmap,
                            CostmapPtr &local_costmap);

  /**
   * Destructor
   */
  virtual ~MoveBaseRecoveryExecution();

protected:

  //! Shared pointer to the global costmap
  CostmapPtr &global_costmap_;
  
  //! Shared pointer to thr local costmap
  CostmapPtr &local_costmap_;

private:

  /**
   * @brief Initializes a recovery behavior plugin with its name and pointers to the global and local costmaps
   */
  virtual void initPlugins();

  /**
   * loads the plugins defined in the parameter server
   * @remark Override abstract class method to allow loading nav_core-based plugins, wrapped with the MBF base class
   * @return true, if all recovery behavior could be read successfully.
   */
  virtual bool loadPlugins();
};

} /* namespace move_base_flex */

#endif /* MOVE_BASE_FLEX__MOVE_BASE_RECOVERY_EXECUTION_H_ */
