#ifndef MBF_UTILITY__EXE_PATH_EXCEPTION_H_
#define MBF_UTILITY__EXE_PATH_EXCEPTION_H_

#include <exception>
#include <mbf_msgs/ExePathResult.h>

namespace mbf_utility
{

struct ExePathException : public std::exception
{
  ExePathException(unsigned int error_code) : outcome(error_code){}

  const char * what () const throw () {
    switch(outcome)
    {
      case mbf_msgs::ExePathResult::FAILURE: return "Failure";
      case mbf_msgs::ExePathResult::CANCELED: return "Canceled";
      case mbf_msgs::ExePathResult::NO_VALID_CMD: return "No valid command";
      case mbf_msgs::ExePathResult::PAT_EXCEEDED: return "Patience exceeded";
      case mbf_msgs::ExePathResult::COLLISION: return "Collision";
      case mbf_msgs::ExePathResult::OSCILLATION: return "Oscillation";
      case mbf_msgs::ExePathResult::ROBOT_STUCK: return "Robot stuck";
      case mbf_msgs::ExePathResult::MISSED_GOAL: return "Missed Goal";
      case mbf_msgs::ExePathResult::MISSED_PATH: return "Missed Path";
      case mbf_msgs::ExePathResult::BLOCKED_PATH: return "Blocked Path";
      case mbf_msgs::ExePathResult::INVALID_PATH: return "Invalid Path";
      case mbf_msgs::ExePathResult::TF_ERROR: return "TF Error";
      case mbf_msgs::ExePathResult::NOT_INITIALIZED: return "Not initialized";
      case mbf_msgs::ExePathResult::INVALID_PLUGIN: return "Invalid Plugin";
      case mbf_msgs::ExePathResult::INTERNAL_ERROR: return "Internal Error";
      case mbf_msgs::ExePathResult::STOPPED: return "Stopped";
      case mbf_msgs::ExePathResult::OUT_OF_MAP: return "Out of map";
      case mbf_msgs::ExePathResult::MAP_ERROR: return "Map error";
      default: return "unknown error code: " + outcome;
    }
  }
  unsigned int outcome;
};

} /* namespace mbf_utility */

#endif // MBF_UTILITY__EXE_PATH_EXCEPTION_H_
