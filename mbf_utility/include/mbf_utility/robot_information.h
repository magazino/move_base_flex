/*
 *  Copyright 2018, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  robot_information.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_UTILITY__ROBOT_INFORMATION_H_
#define MBF_UTILITY__ROBOT_INFORMATION_H_

#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/duration.h>
#include <string>

#include "mbf_utility/types.h"

namespace mbf_utility
{

class RobotInformation
{
 public:

  typedef boost::shared_ptr<RobotInformation> Ptr;

  RobotInformation(
      TF &tf_listener,
      const std::string &global_frame,
      const std::string &robot_frame,
      const ros::Duration &tf_timeout);

  /**
   * @brief Computes the current robot pose (robot_frame_) in the global frame (global_frame_).
   * @param robot_pose Reference to the robot_pose message object to be filled.
   * @return true, if the current robot pose could be computed, false otherwise.
   */
  bool getRobotPose(geometry_msgs::PoseStamped &robot_pose) const;

  bool getRobotVelocity(geometry_msgs::TwistStamped &robot_velocity, ros::Duration look_back_duration) const;

  const std::string& getGlobalFrame() const;

  const std::string& getRobotFrame() const;

  const TF& getTransformListener() const;

  const ros::Duration& getTfTimeout() const;

 private:
  const TF& tf_listener_;

  const std::string &global_frame_;

  const std::string &robot_frame_;

  const ros::Duration &tf_timeout_;

};

} /* mbf_utility */

#endif /* MBF_UTILITY__ROBOT_INFORMATION_H_ */
