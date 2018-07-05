#ifndef MBF_ABSTRACT_NAV__PLANNER_ACTION_H_
#define MBF_ABSTRACT_NAV__PLANNER_ACTION_H_

#include "mbf_abstract_nav/abstract_action.h"
#include "mbf_abstract_nav/abstract_planner_execution.h"
#include "mbf_abstract_nav/robot_information.h"
#include <actionlib/server/action_server.h>
#include <mbf_msgs/GetPathAction.h>

namespace mbf_abstract_nav{


class PlannerAction : public AbstractAction<mbf_msgs::GetPathAction, AbstractPlannerExecution>
{
 public:

  typedef boost::shared_ptr<PlannerAction> Ptr;

  PlannerAction(
      const std::string& name,
      const RobotInformation &robot_info
  );

  void run(GoalHandle &goal_handle, AbstractPlannerExecution &execution);

 protected:

  /**
   * @brief Publishes the given path / plan
   * @param plan The plan, a list of stamped poses, to be published
   */
  void publishPath(std::vector<geometry_msgs::PoseStamped> &plan);

  /**
   * @brief Transforms a plan to the global frame (global_frame_) coord system.
   * @param plan Input plan to be transformed.
   * @param global_plan Output plan, which is then transformed to the global frame.
   * @return true, if the transformation succeeded, false otherwise
   */
  bool transformPlanToGlobalFrame(std::vector<geometry_msgs::PoseStamped> &plan,
                                  std::vector<geometry_msgs::PoseStamped> &global_plan);


 private:

  //! Publisher to publish the current goal pose, which is used for path planning
  ros::Publisher current_goal_pub_;

  //! Publisher to publish the current computed path
  ros::Publisher path_pub_;

  //! Path sequence counter
  unsigned int path_seq_count_;
};


}


#endif //MBF_ABSTRACT_NAV__PLANNER_ACTION_H_
