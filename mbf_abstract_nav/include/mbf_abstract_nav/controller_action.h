#ifndef MBF_ABSTRACT_NAV__CONTROLLER_ACTION_H_
#define MBF_ABSTRACT_NAV__CONTROLLER_ACTION_H_

#include "mbf_abstract_nav/abstract_action.h"
#include "mbf_abstract_nav/abstract_controller_execution.h"
#include "mbf_abstract_nav/robot_information.h"
#include <actionlib/server/action_server.h>
#include <mbf_msgs/ExePathAction.h>

namespace mbf_abstract_nav{

class ControllerAction :
    public AbstractAction<mbf_msgs::ExePathAction, AbstractControllerExecution>
{
 public:

  typedef boost::shared_ptr<ControllerAction> Ptr;

  ControllerAction(const std::string &name,
                   const RobotInformation &robot_info);

  void run(GoalHandle &goal_handle, AbstractControllerExecution &execution);

};
}



#endif //MBF_ABSTRACT_NAV__CONTROLLER_ACTION_H_
