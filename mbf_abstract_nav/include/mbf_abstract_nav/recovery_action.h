#ifndef MBF_ABSTRACT_NAV__RECOVERY_ACTION_H_
#define MBF_ABSTRACT_NAV__RECOVERY_ACTION_H_

#include "mbf_abstract_nav/abstract_action.h"
#include "mbf_abstract_nav/abstract_recovery_execution.h"
#include "mbf_abstract_nav/robot_information.h"
#include <actionlib/server/action_server.h>
#include <mbf_msgs/RecoveryAction.h>
#include <boost/thread/condition_variable.hpp>

namespace mbf_abstract_nav{

class RecoveryAction : public AbstractAction<mbf_msgs::RecoveryAction, AbstractRecoveryExecution>
{
 public:

  typedef boost::shared_ptr<RecoveryAction> Ptr;

  RecoveryAction(const std::string& name, const RobotInformation &robot_info);

  void run(GoalHandle &goal_handle, AbstractRecoveryExecution &execution);

};

}


#endif //MBF_ABSTRACT_NAV__RECOVERY_ACTION_H_
