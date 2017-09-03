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
 *  simple_controller_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MOVE_BASE_FLEX__SIMPLE_CONTROLLER_EXECUTION_H_
#define MOVE_BASE_FLEX__SIMPLE_CONTROLLER_EXECUTION_H_

#include "move_base_flex_core/abstract_local_planner.h"
#include "move_base_flex/abstract_server/abstract_controller_execution.h"

namespace move_base_flex
{
/**
 * @brief The SimpleControllerExecution basically uses the AbstractControllerExecution and loads local planner plugins,
 *        which implements the base class interface AbstractLocalPlanner. This implementation allows planners, which
 *        do not initialize map representations via Move Base Flex.
 *
 * @ingroup controller_execution simple_server
 */
class SimpleControllerExecution : public AbstractControllerExecution<move_base_flex_core::AbstractLocalPlanner>
{
public:

  /**
   * @brief Constructor
   * @param condition Condition variable for waking up all listeners, e.g. the navigation server, due to a state change
   * @param tf_listener_ptr Shared pointer to a common TransformListener
   */
  SimpleControllerExecution(boost::condition_variable &condition,
                            const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr);

  /**
   * @brief Destructor
   */
  virtual ~SimpleControllerExecution();

private:

  /**
   * @brief Empty init method. Nothing to initialize.
   */
  virtual void initLocalPlannerPlugin();

};

} /* namespace move_base_flex */

#endif /* MOVE_BASE_FLEX__SIMPLE_CONTROLLER_EXECUTION_H_ */
