/*
 *  Copyright 2017, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
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
 *  navigation_utility.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MOVE_BASE_FLEX__NAVIGATION_STATE_H_
#define MOVE_BASE_FLEX__NAVIGATION_STATE_H_

#include <tf/transform_listener.h>

namespace move_base_flex
{

/**
 * @brief Transforms a pose from one frame into another
 * @param tf_listener TransformListener
 * @param target_frame Target frame for the pose
 * @param target_time Time, in that the frames should be used
 * @param timeout Timeout for looking up the transformation
 * @param in Pose to transform
 * @param fixed_frame Fixed frame of the source and target frame.
 * @param out Transformed pose.
 * @return true, if the transformation succeeded.
 */
bool transformPose(const tf::TransformListener &tf_listener,
                   const std::string &target_frame,
                   const ros::Time &target_time,
                   const ros::Duration &timeout,
                   const geometry_msgs::PoseStamped &in,
                   const std::string &fixed_frame,
                   geometry_msgs::PoseStamped &out);

bool getRobotPose(const tf::TransformListener &tf_listener,
                  const std::string &robot_frame,
                  const std::string &global_frame,
                  const ros::Duration &timeout,
                  geometry_msgs::PoseStamped &robot_pose);

double distance(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2);

double angle(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2);

} /* namespace move_base_flex */

#endif /* navigation_state.h */
