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

#include <boost/chrono/thread_clock.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <mbf_abstract_core/abstract_recovery.h>
#include <mbf_utility/navigation_utility.h>

#include "mbf_abstract_nav/MoveBaseFlexConfig.h"

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
  class AbstractRecoveryExecution
  {
  public:

    typedef boost::shared_ptr<AbstractRecoveryExecution > Ptr;

    /**
     * @brief Constructor
     * @param condition Thread sleep condition variable, to wake up connected threads
     * @param tf_listener_ptr Shared pointer to a common tf listener
     */
    AbstractRecoveryExecution(boost::condition_variable &condition,
                              const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr);

    /**
     * @brief Destructor
     */
    virtual ~AbstractRecoveryExecution();

    /**
     * @brief starts the recovery behavior thread, which calls the recovery behavior plugin.
     * @param name The name of the recovery behavior loaded.
     */
    void startRecovery(const std::string &name);

    /**
     * @brief Tries to stop the recovery behavior thread by an interrupt
     */
    void stopRecovery();

    /**
     * @brief Checks whether the patience was exceeded.
     * @return true, if the patience duration was exceeded.
     */
    bool isPatienceExceeded();

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
     * @brief Reads the parameter server and tries to load and initialize the recovery behaviors
     * @return true, if successful
     */
    bool initialize();

    /**
     * @brief Reconfigures the current configuration and reloads all parameters. This method is called from a dynamic
     *        reconfigure tool.
     * @param config Current MoveBaseFlexConfig object. See the MoveBaseFlex.cfg definition.
     */
    void reconfigure(const MoveBaseFlexConfig &config);

    /**
     * @brief Tries to cancel the execution of the recovery behavior plugin.
     * @return true, if the plugin tries or canceled the behavior, false otherwise.
     */
    bool cancel();

    /**
     * @brief Returns true is the given name has been loaded as recovery behavior.
     * @param name The name of the recovery behavior.
     * @return true, if the recovery behavior exists, false otherwise.
     */
    bool hasRecoveryBehavior(const std::string &name);

    /**
     * @brief Returns a list of all loaded recovery behavior.
     * @return list of all loaded recovery behavior names.
     */
    std::vector<std::string> listRecoveryBehaviors();

    /**
     * @brief Returns the type for the corresponding name
     * @param name Name of the plugin
     * @param type Type of the plugin, returned
     * @return true, if the name exists and a type could be written.
     */
    bool getTypeOfBehavior(const std::string &name, std::string &type);

  protected:

    /**
     * @brief Main execution method which will be executed by the recovery execution thread_.
     */
    virtual void run();

    //! map to store the recovery behaviors. Each behavior can be accessed by its corresponding name
    std::map<std::string, boost::shared_ptr<mbf_abstract_core::AbstractRecovery> > recovery_behaviors_;

    //! map to store the type of the behavior as string
    std::map<std::string, std::string> recovery_behaviors_type_;

    //! the current loaded recovery behavior
    mbf_abstract_core::AbstractRecovery::Ptr current_behavior_;

    //! shared pointer to common TransformListener
    const boost::shared_ptr<tf::TransformListener> tf_listener_ptr_;

  private:

    /**
     * @brief Sets the current internal state. This method is thread communication safe
     * @param state The state to set.
     */
    void setState(RecoveryState state);

    /**
     * @brief Pure virtual method, the derived class has to implement. Depending on the plugin base class,
     *        some plugins need to be initialized!
     * @param name The name of the recovery behavior
     * @param behavior_ptr pointer to the recovery behavior object which corresponds to the name param
     * @return true if init succeeded, false otherwise
     */
    virtual bool initPlugin(
        const std::string& name,
        const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr
    ) = 0;

    /**
     * @brief Loads a Recovery plugin associated with given recovery type parameter
     * @param recovery_name The name of the Recovery plugin
     * @return A shared pointer to a Recovery plugin, if the plugin was loaded successfully, an empty pointer otherwise.
     */
    virtual mbf_abstract_core::AbstractRecovery::Ptr loadRecoveryPlugin(const std::string& recovery_type) = 0;

    /**
     * @brief Loads the plugins defined in the parameter server
     * @return true, if all recovery behavior have been loaded successfully.
     */
    bool loadPlugins();

    //! mutex to handle safe thread communication for the current state
    boost::mutex state_mtx_;

    //! dynamic reconfigure and start time mutexes to mutually exclude read/write configuration
    boost::mutex conf_mtx_;
    boost::mutex time_mtx_;

    //! recovery behavior allowed time
    ros::Duration patience_;

    //! recovery behavior start time
    ros::Time start_time_;

    //! the last requested recovery behavior to start
    std::string requested_behavior_name_;

    //! condition variable to wake up control thread
    boost::condition_variable &condition_;

    //! thread for running recovery behaviors
    boost::thread thread_;

    //! current internal state
    RecoveryState state_;

    //! current canceled state
    bool canceled_;
  };

} /* namespace mbf_abstract_nav */

#endif /* MBF_ABSTRACT_NAV__ABSTRACT_RECOVERY_EXECUTION_H_ */
