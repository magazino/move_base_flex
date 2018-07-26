#include "mbf_abstract_nav/abstract_execution_base.h"

namespace mbf_abstract_nav{

  AbstractExecutionBase::AbstractExecutionBase()
    : outcome_(255), cancel_(false)
  {

  }

  bool AbstractExecutionBase::start()
  {
    //setState(STARTED); // TODO
    thread_ = boost::thread(&AbstractExecutionBase::run, this);
    return true;
  }

  void AbstractExecutionBase::stop()
  {
    ROS_WARN_STREAM("Trying to stop the planning rigorously by interrupting the thread!");
    thread_.interrupt();
    //setState(STOPPED); // TODO
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
