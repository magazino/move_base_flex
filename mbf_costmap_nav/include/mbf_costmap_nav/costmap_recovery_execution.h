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
 *  costmap_recovery_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_COSTMAP_NAV__COSTMAP_RECOVERY_EXECUTION_H_
#define MBF_COSTMAP_NAV__COSTMAP_RECOVERY_EXECUTION_H_

#include <mbf_abstract_nav/abstract_recovery_execution.h>
#include <mbf_costmap_core/costmap_recovery.h>

#include "mbf_costmap_nav/MoveBaseFlexConfig.h"
#include "mbf_costmap_nav/costmap_wrapper.h"


namespace mbf_costmap_nav
{
/**
 * @brief The CostmapRecoveryExecution binds a local and a global costmap to the AbstractRecoveryExecution and uses the
 *        nav_core/CostmapRecovery class as base plugin interface.
 * This class makes move_base_flex compatible to the old move_base.
 *
 * @ingroup recovery_execution move_base_server
 */
class CostmapRecoveryExecution : public mbf_abstract_nav::AbstractRecoveryExecution
{

public:
  typedef boost::shared_ptr<CostmapRecoveryExecution> Ptr;

  /**
   * @brief Constructor.
   * @param recovery_name Name of the recovery behavior to run.
   * @param recovery_ptr Shared pointer to the plugin to use.
   * @param robot_info Current robot state
   * @param global_costmap Shared pointer to the global costmap.
   * @param local_costmap Shared pointer to the local costmap.
   * @param config Current server configuration (dynamic).
   */
  CostmapRecoveryExecution(
      const std::string &recovery_name,
      const mbf_costmap_core::CostmapRecovery::Ptr &recovery_ptr,
      const mbf_utility::RobotInformation &robot_info,
      const CostmapWrapper::Ptr &global_costmap,
      const CostmapWrapper::Ptr &local_costmap,
      const MoveBaseFlexConfig &config);

  /**
   * Destructor
   */
  virtual ~CostmapRecoveryExecution();

private:
  /**
   * @brief Implementation-specific setup function, called right before execution.
   * This method overrides abstract execution empty implementation with underlying map-specific setup code.
   */
  void preRun()
  {
    local_costmap_->checkActivate();
    global_costmap_->checkActivate();
  };

  /**
   * @brief Implementation-specific cleanup function, called right after execution.
   * This method overrides abstract execution empty implementation with underlying map-specific cleanup code.
   */
  void postRun()
  {
    local_costmap_->checkDeactivate();
    global_costmap_->checkDeactivate();
  };

  mbf_abstract_nav::MoveBaseFlexConfig toAbstract(const MoveBaseFlexConfig &config);

  //! Shared pointer to the global costmap
  const CostmapWrapper::Ptr &global_costmap_;

  //! Shared pointer to thr local costmap
  const CostmapWrapper::Ptr &local_costmap_;
};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_RECOVERY_EXECUTION_H_ */
