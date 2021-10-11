/*
 *  Copyright 2018, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
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
 *  navigation_utility.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <tf/tf.h>

#include "mbf_utility/navigation_utility.h"

#include <cmath>

namespace mbf_utility
{

bool getRobotPose(const TF &tf,
                  const std::string &robot_frame,
                  const std::string &global_frame,
                  const ros::Duration &timeout,
                  geometry_msgs::PoseStamped &robot_pose)
{
  geometry_msgs::PoseStamped local_pose;
  local_pose.header.frame_id = robot_frame;
  local_pose.header.stamp = ros::Time(0); // most recent available
  local_pose.pose.orientation.w = 1.0;
  bool success = transformPose(tf,
                               global_frame,
                               timeout,
                               local_pose,
                               robot_pose);
  if (success && ros::Time::now() - robot_pose.header.stamp > timeout)
  {
    ROS_WARN("Most recent robot pose is %gs old (tolerance %gs)",
             (ros::Time::now() - robot_pose.header.stamp).toSec(), timeout.toSec());
    return false;
  }
  return success;
}

/**
 * @brief Returns true, if the given quaternion is normalized.
 *
 * @param _q The quaternion to check.
 * @param _epsilon The epsilon (squared distance to 1).
 */
static bool isNormalized(const geometry_msgs::Quaternion& _q, double _epsilon)
{
  const double sq_sum = std::pow(_q.x, 2) + std::pow(_q.y, 2) + std::pow(_q.z, 2) + std::pow(_q.w, 2);
  return std::abs(sq_sum - 1.) <= _epsilon;
}

bool transformPose(const TF &tf,
                   const std::string &target_frame,
                   const ros::Duration &timeout,
                   const geometry_msgs::PoseStamped &in,
                   geometry_msgs::PoseStamped &out)
{
  // Note: The tf-library does not check if the input is well formed.
  if (!isNormalized(in.pose.orientation, 0.01))
  {
    ROS_WARN_STREAM("The given quaterinon " << in.pose.orientation << " is not normalized");
    return false;
  }

  std::string error_msg;

#ifdef USE_OLD_TF
  bool success = tf.waitForTransform(target_frame,
                                     in.header.frame_id,
                                     in.header.stamp,
                                     timeout,
                                     ros::Duration(0.01),
                                     &error_msg);
#else
  bool success = tf.canTransform(target_frame,
                                 in.header.frame_id,
                                 in.header.stamp,
                                 timeout,
                                 &error_msg);
#endif

  if (!success)
  {
    ROS_WARN_STREAM("Failed to look up transform from frame '" << in.header.frame_id << "' into frame '" << target_frame
                    << "': " << error_msg);
    return false;
  }

  try
  {
#ifdef USE_OLD_TF
    tf.transformPose(target_frame, in, out);
#else
    tf.transform(in, out, target_frame);
#endif
  }
  catch (const TFException &ex)
  {
    ROS_WARN_STREAM("Failed to transform pose from frame '" <<  in.header.frame_id << " ' into frame '"
                    << target_frame << "' with exception: " << ex.what());
    return false;
  }
  return true;
}

bool transformPoint(const TF &tf,
                    const std::string &target_frame,
                    const ros::Duration &timeout,
                    const geometry_msgs::PointStamped &in,
                    geometry_msgs::PointStamped &out)
{
  std::string error_msg;

#ifdef USE_OLD_TF
  bool success = tf.waitForTransform(target_frame,
                                     in.header.frame_id,
                                     in.header.stamp,
                                     timeout,
                                     ros::Duration(0.01),
                                     &error_msg);
#else
  bool success = tf.canTransform(target_frame,
                                 in.header.frame_id,
                                 in.header.stamp,
                                 timeout,
                                 &error_msg);
#endif

  if (!success)
  {
    ROS_WARN_STREAM("Failed to look up transform from frame '" << in.header.frame_id << "' into frame '" << target_frame
                                                               << "': " << error_msg);
    return false;
  }

  try
  {
#ifdef USE_OLD_TF
    tf.transformPoint(target_frame, in, out);
#else
    tf.transform(in, out, target_frame);
#endif
  }
  catch (const TFException &ex)
  {
    ROS_WARN_STREAM("Failed to transform point from frame '" <<  in.header.frame_id << " ' into frame '"
                                                            << target_frame << "' with exception: " << ex.what());
    return false;
  }
  return true;
}

double distance(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2)
{
  const geometry_msgs::Point &p1 = pose1.pose.position;
  const geometry_msgs::Point &p2 = pose2.pose.position;
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  const double dz = p1.z - p2.z;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

double angle(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2)
{
  const geometry_msgs::Quaternion &q1 = pose1.pose.orientation;
  const geometry_msgs::Quaternion &q2 = pose2.pose.orientation;
  tf::Quaternion rot1, rot2;
  tf::quaternionMsgToTF(q1, rot1);
  tf::quaternionMsgToTF(q2, rot2);
  return rot1.angleShortestPath(rot2);
}

} /* namespace mbf_utility */
