#include "mbf_abstract_nav/abstract_execution_base.h"

namespace mbf_abstract_nav{

  AbstractExecutionBase::AbstractExecutionBase()
    : outcome_(255), cancel_(false)
  {

  }

  void AbstractExecutionBase::join(){
    thread_.join();
  }

  void AbstractExecutionBase::waitForStateUpdate(boost::chrono::microseconds const &duration)
  {
    boost::mutex mutex;
    boost::unique_lock<boost::mutex> lock(mutex);
    condition_.wait_for(lock, duration);
  }

  uint32_t AbstractExecutionBase::getOutcome()
  {
    return outcome_;
  }

  std::string AbstractExecutionBase::getMessage()
  {
    return message_;
  }


} /* namespace mbf_abstract_nav */
