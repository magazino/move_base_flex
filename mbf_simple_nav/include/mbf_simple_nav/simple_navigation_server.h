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
 *  simple_navigation_server.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_SIMPLE_NAV__SIMPLE_NAVIGATION_SERVER_H_
#define MBF_SIMPLE_NAV__SIMPLE_NAVIGATION_SERVER_H_

#include <mbf_abstract_nav/abstract_navigation_server.h>
#include "simple_planner_execution.h"
#include "simple_controller_execution.h"
#include "simple_recovery_execution.h"

namespace mbf_simple_nav
{
/**
 * @defgroup simple_server Simple Server
 * @brief Classes belonging to the Simple Server level.
 */

/**
 * @brief The SimpleNavigationServer provides a simple navigation server, which does not bind a map representation to
 *        Move Base Flex. It combines the execution classes which use the mbf_abstract_core/AbstractController,
 *        mbf_abstract_core/AbstractPlanner and the mbf_abstract_core/AbstractRecovery base classes
 *        as plugin interfaces.
 *
 * @ingroup navigation_server simple_server
 */
class SimpleNavigationServer : public mbf_abstract_nav::AbstractNavigationServer
{
public:

  /**
   * @brief Constructor
   * @param tf_listener_ptr Shared pointer to a common TransformListener
   */
  SimpleNavigationServer(const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr);

  /**
   * @brief Destructor
   */
  virtual ~SimpleNavigationServer();
};

} /* namespace mbf_simple_nav */

#endif /* MBF_SIMPLE_NAV__SIMPLE_NAVIGATION_SERVER_H_ */
