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

#ifndef MBF_UTILITY__NAVIGATION_UTILITY_H_
#define MBF_UTILITY__NAVIGATION_UTILITY_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <string>
#include <tf/transform_listener.h>

namespace mbf_utility
{

/**
 * @brief Transforms a pose from one frame into another.
 * @param tf_listener TransformListener.
 * @param target_frame Target frame for the pose.
 * @param target_time Time, in that the frames should be used.
 * @param timeout Timeout for looking up the transformation.
 * @param in Pose to transform.
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

/**
 * @brief Computes the robot pose.
 * @param tf_listener TransformListener.
 * @param robot_frame frame of the robot.
 * @param global_frame global frame in which the robot is located.
 * @param timeout Timeout for looking up the transformation.
 * @param robot_pose the computed rebot pose in the global frame.
 * @return true, if succeeded, false otherwise.
 */
bool getRobotPose(const tf::TransformListener &tf_listener,
                  const std::string &robot_frame,
                  const std::string &global_frame,
                  const ros::Duration &timeout,
                  geometry_msgs::PoseStamped &robot_pose);
/**
 * @brief Computes the Euclidean-distance between two poses.
 * @param pose1 pose 1
 * @param pose2 pose 2
 * @return Euclidean distance between pose 1 and pose 2.
 */
double distance(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2);

/**
 * @brief computes the smallest angle between two poses.
 * @param pose1 pose 1
 * @param pose2 pose 2
 * @return smallest angle between pose 1 and pose 2.
 */
double angle(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2);

} /* namespace mbf_utility */

#endif /* navigation_utility.h */
