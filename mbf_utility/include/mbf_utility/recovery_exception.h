#ifndef MBF_UTILITY__RECOVERY_EXCEPTION_H_
#define MBF_UTILITY__RECOVERY_EXCEPTION_H_

#include <exception>
#include <mbf_msgs/RecoveryResult.h>

namespace mbf_utility
{

struct RecoveryException : public std::exception
{
  RecoveryException(unsigned int error_code) : outcome(error_code){}

  const char * what () const throw () {
    switch(outcome)
    {
      case mbf_msgs::RecoveryResult::FAILURE: return "Failure";
      case mbf_msgs::RecoveryResult::CANCELED: return "Canceled";
      case mbf_msgs::RecoveryResult::PAT_EXCEEDED: return "Patience exceeded";
      case mbf_msgs::RecoveryResult::TF_ERROR: return "TF Error";
      case mbf_msgs::RecoveryResult::NOT_INITIALIZED: return "Not initialized";
      case mbf_msgs::RecoveryResult::INVALID_PLUGIN: return "Invalid Plugin";
      case mbf_msgs::RecoveryResult::INTERNAL_ERROR: return "Internal Error";
      case mbf_msgs::RecoveryResult::STOPPED: return "Stopped";
      default: return "unknown error code: " + outcome;
    }
  }
  unsigned int outcome;
};

} /* namespace mbf_utility */

#endif // MBF_UTILITY__RECOVERY_EXCEPTION_H_
