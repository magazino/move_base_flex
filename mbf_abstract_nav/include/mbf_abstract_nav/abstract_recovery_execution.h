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
 *  abstract_recovery_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_RECOVERY_EXECUTION_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_RECOVERY_EXECUTION_H_

#include <map>
#include <string>
#include <stdint.h>
#include <vector>

#include <tf/transform_listener.h>

#include <mbf_abstract_core/abstract_recovery.h>
#include <mbf_utility/types.h>

#include <mbf_utility/navigation_utility.h>

#include "mbf_abstract_nav/MoveBaseFlexConfig.h"
#include "mbf_abstract_nav/abstract_execution_base.h"


namespace mbf_abstract_nav
{
/**
 * @defgroup recovery_execution Recovery Execution Classes
 * @brief The recovery execution classes are derived from the RecoveryPlannerExecution and extends the functionality.
 *        The base recovery execution code is located in the AbstractRecoveryExecution.
 */

/**
 * @brief The AbstractRecoveryExecution class loads and binds the recovery behavior plugin. It contains a thread
 *        running the plugin, executing the recovery behavior. An internal state is saved and will be pulled by the
 *        server, which controls the recovery behavior execution. Due to a state change it wakes up all threads
 *        connected to the condition variable.
 *
 * @ingroup abstract_server recovery_execution
 */
  class AbstractRecoveryExecution : public AbstractExecutionBase
  {
  public:

    typedef boost::shared_ptr<AbstractRecoveryExecution > Ptr;

    /**
     * @brief Constructor
     * @param condition Thread sleep condition variable, to wake up connected threads
     * @param tf_listener_ptr Shared pointer to a common tf listener
     */
    AbstractRecoveryExecution(const std::string name,
                              const mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr,
                              const TFPtr &tf_listener_ptr,
                              const MoveBaseFlexConfig &config,
                              boost::function<void()> setup_fn,
                              boost::function<void()> cleanup_fn);

    /**
     * @brief Destructor
     */
    virtual ~AbstractRecoveryExecution();

    /**
     * @brief Checks whether the patience was exceeded.
     * @return true, if the patience duration was exceeded.
     */
    bool isPatienceExceeded();

    /**
     * @brief Cancel the planner execution. This calls the cancel method of the planner plugin. This could be useful if the
     * computation takes too much time.
     * @return true, if the planner plugin tries / tried to cancel the planning step.
     */
    virtual bool cancel();

    /**
     * @brief internal state.
     */
    enum RecoveryState
    {
      INITIALIZED,   ///< The recovery execution has been initialized.
      STARTED,       ///< The recovery execution thread has been started.
      RECOVERING,    ///< The recovery behavior plugin is running.
      WRONG_NAME,    ///< The given name could not be associated with a load behavior.
      RECOVERY_DONE, ///< The recovery behavior execution is done.
      CANCELED,      ///< The recovery execution was canceled.
      STOPPED,       ///< The recovery execution has been stopped.
      INTERNAL_ERROR ///< An internal error occurred.
    };

    /**
     * @brief Returns the current state, thread-safe communication
     * @return current internal state
     */
    AbstractRecoveryExecution::RecoveryState getState();

    /**
     * @brief Reconfigures the current configuration and reloads all parameters. This method is called from a dynamic
     *        reconfigure tool.
     * @param config Current MoveBaseFlexConfig object. See the MoveBaseFlex.cfg definition.
     */
    void reconfigure(const MoveBaseFlexConfig &config);

  protected:

    /**
     * @brief Main execution method which will be executed by the recovery execution thread_.
     */
    virtual void run();

    //! the current loaded recovery behavior
    mbf_abstract_core::AbstractRecovery::Ptr behavior_;

    //! shared pointer to common TransformListener
    const TFPtr tf_listener_ptr_;

  private:

    /**
     * @brief Sets the current internal state. This method is thread communication safe
     * @param state The state to set.
     */
    void setState(RecoveryState state);

    //! mutex to handle safe thread communication for the current state
    boost::mutex state_mtx_;

    //! dynamic reconfigure and start time mutexes to mutually exclude read/write configuration
    boost::mutex conf_mtx_;
    boost::mutex time_mtx_;

    //! recovery behavior allowed time
    ros::Duration patience_;

    //! recovery behavior start time
    ros::Time start_time_;

    //! current internal state
    RecoveryState state_;
  };

} /* namespace mbf_abstract_nav */

#endif /* MBF_ABSTRACT_NAV__ABSTRACT_RECOVERY_EXECUTION_H_ */
