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

#include <mbf_msgs/RecoveryResult.h>
#include "nav_core_wrapper/wrapper_recovery_behavior.h"

namespace mbf_nav_core_wrapper
{
void WrapperRecoveryBehavior::initialize(std::string name, TF *tf,
                                         costmap_2d::Costmap2DROS *global_costmap,
                                         costmap_2d::Costmap2DROS *local_costmap)
{
  nav_core_plugin_->initialize(name, tf, global_costmap, local_costmap);
}

uint32_t WrapperRecoveryBehavior::runBehavior(std::string &message)
{
  nav_core_plugin_->runBehavior();
  // TODO return a code for old API
  return mbf_msgs::RecoveryResult::SUCCESS;
}

bool WrapperRecoveryBehavior::cancel()
{
  return false;
}

WrapperRecoveryBehavior::WrapperRecoveryBehavior(boost::shared_ptr<nav_core::RecoveryBehavior> plugin)
    : nav_core_plugin_(plugin)
{}

WrapperRecoveryBehavior::~WrapperRecoveryBehavior()
{}

};  /* namespace mbf_abstract_core */
