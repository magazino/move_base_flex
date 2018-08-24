#ifndef MBF_ABSTRACT_NAV__ROBOT_INFORMATION_H_
#define MBF_ABSTRACT_NAV__ROBOT_INFORMATION_H_

#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/duration.h>
#include <string>
#include <tf/transform_listener.h>


namespace mbf_abstract_nav{

class RobotInformation{

 public:

  typedef boost::shared_ptr<RobotInformation> Ptr;

  RobotInformation(
      tf::TransformListener &tf_listener,
      const std::string &global_frame,
      const std::string &robot_frame,
      const ros::Duration &tf_timeout);

  bool getRobotPose(geometry_msgs::PoseStamped &robot_pose) const;

  bool getRobotVelocity(geometry_msgs::TwistStamped &robot_velocity, ros::Duration look_back_duration) const;

  const std::string& getGlobalFrame() const;

  const std::string& getRobotFrame() const;

  const tf::TransformListener& getTransformListener() const;

  const ros::Duration& getTfTimeout() const;

 private:
  const tf::TransformListener& tf_listener_;

  const std::string &global_frame_;

  const std::string &robot_frame_;

  const ros::Duration &tf_timeout_;

};

}
#endif //MBF_ABSTRACT_NAV__ROBOT_INFORMATION_H_
