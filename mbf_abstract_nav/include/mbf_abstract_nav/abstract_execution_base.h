/*
 *  Copyright 2018, Sebastian Pütz
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
 *  abstract_execution_base.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_ABSTRACT_NAV__ABSTRACT_EXECUTION_BASE_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_EXECUTION_BASE_H_

#include <boost/thread.hpp>
#include <boost/chrono/duration.hpp>
#include <boost/chrono/thread_clock.hpp>

#include <mbf_abstract_nav/MoveBaseFlexConfig.h>

namespace mbf_abstract_nav
{

class AbstractExecutionBase
{
 public:

  AbstractExecutionBase(std::string name);

  virtual bool start();

  virtual void stop();

  /**
   * @brief Cancel the plugin execution.
   * @return true, if the plugin tries / tried to cancel the computation.
   */
  virtual bool cancel() = 0;

  void join();

  void waitForStateUpdate(boost::chrono::microseconds const &duration);

  /**
   * @brief Gets the current plugin execution outcome
   */
  uint32_t getOutcome();

  /**
   * @brief Gets the current plugin execution message
   */
  std::string getMessage();

  /**
   * @brief Returns the name of the corresponding plugin
   */
  std::string getName();

  /**
   * @brief Optional implementation-specific setup function, called right before execution.
   */
  virtual void preRun() { };

  /**
   * @brief Optional implementation-specific cleanup function, called right after execution.
   */
  virtual void postRun() { };

protected:
  virtual void run() = 0;

  //! condition variable to wake up control thread
  boost::condition_variable condition_;

  //! the controlling thread object
  boost::thread thread_;

  //! flag for canceling controlling
  bool cancel_;

  //! the last received plugin execution outcome
  uint32_t outcome_;

  //! the last received plugin execution message
  std::string message_;

  //! the plugin name; not the plugin type!
  std::string name_;
};

} /* namespace mbf_abstract_nav */

#endif  /* MBF_ABSTRACT_NAV__ABSTRACT_EXECUTION_BASE_H_ */
