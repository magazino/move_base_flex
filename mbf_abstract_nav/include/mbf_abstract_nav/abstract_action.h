#ifndef MBF_ABSTRACT_NAV__ABSTRACT_ACTION_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_ACTION_H_

#include <actionlib/server/action_server.h>
#include <boost/bimap/bimap.hpp>
#include "mbf_abstract_nav/robot_information.h"

namespace mbf_abstract_nav{


template <typename Action, typename Execution>
class AbstractAction
{
 public:
  typedef boost::bimaps::bimap<uint8_t, std::string> SlotGoalIdMap;

  typedef boost::shared_ptr<AbstractAction> Ptr;

  typedef typename actionlib::ActionServer<Action>::GoalHandle GoalHandle;
  typedef boost::function<void (GoalHandle &goal_handle, Execution &execution)> RunMethod;

  AbstractAction(
      const std::string& name,
      const RobotInformation &robot_info,
      const RunMethod run_method
  ) : name_(name), robot_info_(robot_info), run_(run_method){}

  void start(
      GoalHandle goal_handle,
      typename Execution::Ptr execution_ptr
  )
  {
    typename SlotGoalIdMap::left_const_iterator slot
        = concurrency_slots_.left.find(goal_handle.getGoal()->concurrency_slot);
    if(slot != concurrency_slots_.left.end()) // if there is a plugin running on the same slot, cancel it // TODO make thread safe
    {
      typename std::map<const std::string, const typename Execution::Ptr>::const_iterator elem
          = executions_.find(slot->second);
      if(elem != executions_.end())
      {
        elem->second->cancel();
      }
      concurrency_slots_.left.erase(slot->first);
    }
    concurrency_slots_.insert(SlotGoalIdMap::value_type(goal_handle.getGoal()->concurrency_slot, goal_handle.getGoalID().id));
    executions_.insert(std::pair<const std::string, const typename Execution::Ptr>(goal_handle.getGoalID().id, execution_ptr));
    threads_.create_thread(boost::bind(&AbstractAction::runAndCleanUp, this, boost::ref(goal_handle), boost::ref(execution_ptr)));
  }

  void cancel(GoalHandle &goal_handle){
    typename std::map<const std::string, const typename Execution::Ptr>::const_iterator
        elem = executions_.find(goal_handle.getGoalID().id);
    if(elem != executions_.end())
    {
      elem->second->cancel();
    }

  }

  void runAndCleanUp(GoalHandle goal_handle, typename Execution::Ptr execution_ptr){
    ROS_DEBUG_STREAM_NAMED("get_path", "New thread started for slot \""
        << static_cast<int>(concurrency_slots_.right.find(goal_handle.getGoalID().id)->second) << "\".");
    run_(goal_handle, *execution_ptr);
    ROS_DEBUG_STREAM_NAMED("get_path", "Finished action run method, cleanup maps...");
    executions_.erase(goal_handle.getGoalID().id);
    concurrency_slots_.right.erase(goal_handle.getGoalID().id);
  }


  const std::string &name_;
  const RobotInformation &robot_info_;

  RunMethod run_;
  boost::thread_group threads_;
  std::map<const std::string, const typename Execution::Ptr> executions_;
  SlotGoalIdMap concurrency_slots_;

};


}


#endif //MBF_ABSTRACT_NAV__ABSTRACT_ACTION_H_
