#ifndef MBF_ABSTRACT_NAV__ABSTRACT_ACTION_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_ACTION_H_

#include <actionlib/server/action_server.h>
#include "mbf_abstract_nav/robot_information.h"

namespace mbf_abstract_nav{


template <typename Action, typename Execution>
class AbstractAction
{
 public:

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
    executions_.insert(std::pair<const std::string, const typename Execution::Ptr>(goal_handle.getGoalID().id, execution_ptr));
    threads_.create_thread(boost::bind(run_, boost::ref(goal_handle), boost::ref(*execution_ptr)));
  }

  void cancel(GoalHandle &goal_handle){
    typename std::map<const std::string, const typename Execution::Ptr>::const_iterator
        elem = executions_.find(goal_handle.getGoalID().id);
    if(elem != executions_.end())
    {
      elem->second->cancel();
    }

  }


  const std::string &name_;
  const RobotInformation &robot_info_;

  RunMethod run_;
  std::map<const std::string, const typename Execution::Ptr>executions_;
  boost::thread_group threads_;

};


}


#endif //MBF_ABSTRACT_NAV__ABSTRACT_ACTION_H_
