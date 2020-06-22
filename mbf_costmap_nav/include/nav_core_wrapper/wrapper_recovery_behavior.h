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
 *  wrapper_recovery_behavior.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_COSTMAP_NAV__WRAPPER_RECOVERY_BEHAVIOR_H_
#define MBF_COSTMAP_NAV__WRAPPER_RECOVERY_BEHAVIOR_H_

#include <nav_core/recovery_behavior.h>
#include "mbf_costmap_core/costmap_recovery.h"

#include <mbf_utility/types.h>
namespace mbf_nav_core_wrapper {
  /**
   * @class CostmapRecovery
   * @brief Provides an interface for recovery behaviors used in navigation.
   * All recovery behaviors written to work as MBF plugins must adhere to this interface. Alternatively, this
   * class can also operate as a wrapper for old API nav_core-based plugins, providing backward compatibility.
   */
  class WrapperRecoveryBehavior : public mbf_costmap_core::CostmapRecovery{
    public:

      /**
       * @brief Initialization function for the CostmapRecovery
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack
       * @param local_costmap A pointer to the local_costmap used by the navigation stack
       */
      virtual void initialize(std::string name, TF* tf,
                              costmap_2d::Costmap2DROS* global_costmap,
                              costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief Runs the CostmapRecovery
       */
      virtual uint32_t runBehavior(std::string &message);

      /**
       * @brief Requests the planner to cancel, e.g. if it takes too much time
       * @remark New on MBF API
       * @return True if a cancel has been successfully requested, false if not implemented.
       */
      virtual bool cancel();

      /**
       * @brief Public constructor used for handling a nav_core-based plugin
       * @param plugin Backward compatible plugin
       */
      WrapperRecoveryBehavior(boost::shared_ptr< nav_core::RecoveryBehavior > plugin);

      /**
       * @brief Virtual destructor for the interface
       */
      virtual ~WrapperRecoveryBehavior();

    private:
      boost::shared_ptr< nav_core::RecoveryBehavior > nav_core_plugin_;
  };
};  /* namespace mbf_nav_core_wrapper */

#endif  /* MBF_COSTMAP_NAV__WRAPPER_RECOVERY_BEHAVIOR_H_ */
