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

#ifndef MOVE_BASE_FLEX__ABSTRACT_RECOVERY_EXECUTION_H_
#define MOVE_BASE_FLEX__ABSTRACT_RECOVERY_EXECUTION_H_

#include <pluginlib/class_loader.h>
#include <boost/chrono/thread_clock.hpp>
#include <tf/transform_listener.h>
#include <nav_core/abstract_recovery_behavior.h>

#include "move_base_flex/navigation_utility.h"
#include "move_base_flex/MoveBaseFlexConfig.h"

namespace move_base_flex
{
/**
 * @defgroup recovery_execution Recovery Execution Classes
 *           The recovery execution classes are derived from the RecoveryPlannerExecution and extends the functionality.
 *           The base recovery execution code is located in the AbstractRecoveryExecution.
 */

/**
 *
 * @tparam RECOVERY_BEHAVIOR_BASE
 *
 * @ingroup abstract_server recovery_execution
 */
template<typename RECOVERY_BEHAVIOR_BASE>
  class AbstractRecoveryExecution
  {
  public:

    typedef boost::shared_ptr<AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE> > Ptr;

    /**
     * @brief Constructor
     * @param condition Thread sleep condition variable, to wake up connected threads
     * @param tf_listener_ptr Shared pointer to a common tf listener
     * @param package Package name, which contains the base class interface of the plugin
     * @param class_name Class name of the base class interface of the plugin
     */
    AbstractRecoveryExecution(boost::condition_variable &condition,
                              const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
                              std::string package,
                              std::string class_name);

    virtual ~AbstractRecoveryExecution();

    void startRecovery(const std::string name);

    void stopRecovery();

    enum RecoveryState
    {
      INITIALIZED,
      STARTED,
      RECOVERING,
      WRONG_NAME,
      RECOVERY_DONE,
      CANCELED,
      STOPPED,
    };

    void setState(RecoveryState state);

    AbstractRecoveryExecution<RECOVERY_BEHAVIOR_BASE>::RecoveryState getState();

    void initialize();

    void reconfigure(move_base_flex::MoveBaseFlexConfig &config);

    bool cancel();

    bool hasRecoveryBehavior(const std::string &name);

    bool getTypeOfBehavior(const std::string &name, std::string &type);

  protected:

    pluginlib::ClassLoader<RECOVERY_BEHAVIOR_BASE> class_loader_recovery_behaviors_;

    std::map<std::string, boost::shared_ptr<RECOVERY_BEHAVIOR_BASE> > recovery_behaviors_;

    std::map<std::string, std::string> recovery_behaviors_type_;

    nav_core::AbstractRecoveryBehavior::Ptr current_behavior_;

    const boost::shared_ptr<tf::TransformListener> tf_listener_ptr_;

    virtual void run();

  private:

    // virtual abstract method -> load the specific plugin classes
    virtual void initRecoveryPlugins() = 0;

    bool loadRecoveryPlugins();

    boost::mutex state_mtx_;

    std::string requested_behavior_name_;

    // condition to wake up control thread
    boost::condition_variable &condition_;

    // thread for running recovery behavior
    boost::thread thread_;

    // current internal state
    RecoveryState state_;

    // current canceled state
    bool canceled_;

    // dynamic reconfigure attributes and methods
    boost::recursive_mutex configuration_mutex_;
  };

} /* namespace move_base_flex */

#include "move_base_flex/abstract_server/impl/abstract_recovery_execution.tcc"

#endif /* MOVE_BASE_FLEX__ABSTRACT_RECOVERY_EXECUTION_H_ */
