#ifndef MBF_ABSTRACT_NAV__ABSTRACT_EXECUTION_BASE_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_EXECUTION_BASE_H_

#include <mbf_abstract_nav/MoveBaseFlexConfig.h>
#include <boost/thread.hpp>
#include <boost/chrono/duration.hpp>
#include <boost/chrono/thread_clock.hpp>

namespace mbf_abstract_nav{

class AbstractExecutionBase{

 public:

  AbstractExecutionBase(boost::function<void()> setup_fn,
                        boost::function<void()> cleanup_fn);

  virtual bool start();

  virtual void stop();

  /**
   * @brief Cancel the plugin execution.
   * @return true, if the plugin tries / tried to cancel the computation.
   */
  virtual bool cancel() = 0;

  void join();

  void waitForStateUpdate(boost::chrono::microseconds const &duration);

  virtual void reconfigure(const MoveBaseFlexConfig &config) = 0;

  /**
   * @brief Implementation-specific setup function called right before execution; empty on abstract server
   */
  boost::function<void()> setup_fn_;

  /**
   * @brief Implementation-specific cleanup function called right after execution; empty on abstract server
   */
  boost::function<void()> cleanup_fn_;

  /**
   * @brief Gets the current plugin execution outcome
   */
  uint32_t getOutcome();

  /**
   * @brief Gets the current plugin execution message
   */
  std::string getMessage();

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
};
} /* namespace mbf_abstract_nav */

#endif  /* MBF_ABSTRACT_NAV__ABSTRACT_EXECUTION_BASE_H_ */
