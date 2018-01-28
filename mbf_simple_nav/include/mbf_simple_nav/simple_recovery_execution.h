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
 *  simple_recovery_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_SIMPLE_NAV__SIMPLE_RECOVERY_EXECUTION_H_
#define MBF_SIMPLE_NAV__SIMPLE_RECOVERY_EXECUTION_H_

#include <mbf_abstract_nav/abstract_recovery_execution.h>
#include <mbf_abstract_core/abstract_recovery.h>

namespace mbf_simple_nav
{
/**
 * @brief The SimpleRecoveryExecution basically uses the AbstractRecoveryExecution and loads recovery behavior plugins,
 *        which implements the base class interface AbstractRecovery. This implementation allows recovery
 *        behaviors, which do not initialize map representations via Move Base Flex.
 *
 * @ingroup recovery_execution simple_server
 */
class SimpleRecoveryExecution : public mbf_abstract_nav::AbstractRecoveryExecution
{

public:
  typedef boost::shared_ptr<SimpleRecoveryExecution> Ptr;

  /**
   * @brief Constructor
   * @param condition Condition variable for waking up all listeners, e.g. the navigation server, due to a state change
   * @param tf_listener_ptr Shared pointer to a common TransformListener
   */
  SimpleRecoveryExecution(boost::condition_variable &condition,
                          const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr);

  /**
   * @brief Destructor
   */
  virtual ~SimpleRecoveryExecution();

private:

  /**
   * @brief Loads a Recovery plugin associated with given recovery type parameter
   * @param recovery_name The name of the Recovery plugin
   * @return A shared pointer to a Recovery plugin, if the plugin was loaded successfully, an empty pointer otherwise.
   */
  virtual mbf_abstract_core::AbstractRecovery::Ptr loadRecoveryPlugin(const std::string& recovery_type);

  /**
   * @brief Pure virtual method, the derived class has to implement. Depending on the plugin base class,
   *        some plugins need to be initialized!
   * @param name The name of the recovery behavior
   * @param behavior_ptr pointer to the recovery behavior object which corresponds to the name param
   * @return true always
   */
  virtual bool initPlugin(
      const std::string& name,
      const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr
  );

};

} /* namespace mbf_simple_nav */

#endif /* MBF_SIMPLE_NAV__SIMPLE_RECOVERY_EXECUTION_H_ */
