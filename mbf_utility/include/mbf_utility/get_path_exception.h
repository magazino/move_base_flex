#ifndef MBF_UTILITY__GET_PATH_EXCEPTION_H_
#define MBF_UTILITY__GET_PATH_EXCEPTION_H_

#include <exception>
#include <mbf_msgs/GetPathResult.h>

namespace mbf_utility
{

struct GetPathException : public std::exception
{
  GetPathException(unsigned int error_code) : outcome(error_code){}

  const char * what () const throw () {
    switch(outcome)
    {
      case mbf_msgs::GetPathResult::FAILURE: return "Failure";
      case mbf_msgs::GetPathResult::CANCELED: return "Canceled";
      case mbf_msgs::GetPathResult::INVALID_START: return "Invalid start"
      case mbf_msgs::GetPathResult::INVALID_GOAL: return "Invalid goal"
      case mbf_msgs::GetPathResult::NO_PATH_FOUND: return "No path found";
      case mbf_msgs::GetPathResult::PAT_EXCEEDED: return "Patience exceeded";
      case mbf_msgs::GetPathResult::EMPTY_PATH return "Empty Path";
      case mbf_msgs::GetPathResult::TF_ERROR: return "TF Error";
      case mbf_msgs::GetPathResult::NOT_INITIALIZED: return "Not initialized";
      case mbf_msgs::GetPathResult::INVALID_PLUGIN: return "Invalid Plugin";
      case mbf_msgs::GetPathResult::INTERNAL_ERROR: return "Internal Error";
      case mbf_msgs::GetPathResult::STOPPED: return "Stopped";
      case mbf_msgs::GetPathResult::OUT_OF_MAP: return "Out of map";
      case mbf_msgs::GetPathResult::MAP_ERROR: return "Map error";
      default: return "unknown error code: " + outcome;
    }
  }
  unsigned int outcome;
};

} /* namespace mbf_utility */

#endif // MBF_UTILITY__GET_PATH_EXCEPTION_H_
