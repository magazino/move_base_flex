/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TKruse
 *********************************************************************/
#include <mbf_utility/odometry_helper.h>

namespace mbf_utility
{

OdometryHelper::OdometryHelper(const std::string& odom_topic)
{
  setOdomTopic(odom_topic);
}

void OdometryHelper::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO_STREAM_ONCE("Odometry received on topic " << getOdomTopic());

  // we assume that the odometry is published in the frame of the base
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom_ = *msg;
  if (base_odom_.header.stamp.isZero())
    base_odom_.header.stamp = ros::Time::now();
}

void OdometryHelper::getOdom(nav_msgs::Odometry& base_odom) const
{
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom = base_odom_;
}

void OdometryHelper::setOdomTopic(const std::string& odom_topic)
{
  if (odom_topic != odom_topic_)
  {
    odom_topic_ = odom_topic;

    if (!odom_topic_.empty())
    {
      ros::NodeHandle gn;
      odom_sub_ =
          gn.subscribe<nav_msgs::Odometry>(odom_topic_, 1, boost::bind(&OdometryHelper::odomCallback, this, _1));
    }
    else
    {
      odom_sub_.shutdown();
    }
  }
}

}  // namespace mbf_utility
