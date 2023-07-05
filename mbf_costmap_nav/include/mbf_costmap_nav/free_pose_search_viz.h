/*
 *  Copyright 2023, Rapyuta Robotics Co., Ltd., Renan Salles
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
 *  free_pose_search_viz.h
 *
 *  authors:
 *    Renan Salles <renan028@gmail.com>
 *
 */

#ifndef SEARCH_HELPER_VIZ_H_
#define SEARCH_HELPER_VIZ_H_

// ros
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

namespace mbf_costmap_nav
{
class FreePoseSearchViz
{
private:
  using IdType = visualization_msgs::Marker::_id_type;
  static constexpr auto BLOCKED_NS = "blocked_footprints";
  static constexpr auto SOLUTION_NS = "solution";

  ros::NodeHandle& pnh_;
  std::string frame_id_;

  IdType marker_id_{ 0 };
  visualization_msgs::MarkerArray marker_array_;
  ros::Publisher marker_pub_;

  void addMarker(const geometry_msgs::Pose2D& pose_2d, const std::vector<geometry_msgs::Point>& footprint,
                 const std::string& ns, const std_msgs::ColorRGBA& color);

public:
  FreePoseSearchViz(ros::NodeHandle& pnh, const std::string& frame_id);

  /**
   * @brief It adds a red marker for a footprint that is blocked by the obstacle in namespace "blocked_footprints"
   * @param pose_2d pose of the footprint
   * @param frame_id frame of the pose
   * @param footprint the footprint
   */
  void addBlocked(const geometry_msgs::Pose2D& pose_2d, const std::vector<geometry_msgs::Point>& footprint);

  /**
   * @brief It adds a green marker for a footprint that is not blocked by the obstacle in namespace "solution"
   * @param pose_2d pose of the footprint
   * @param frame_id frame of the pose
   * @param footprint the footprint
   */
  void addSolution(const geometry_msgs::Pose2D& pose_2d, const std::vector<geometry_msgs::Point>& footprint);

  /**
   * @brief It clear the previous viz and publishes the markers
   */
  void publish();

  /**
   * @brief It clears the previous viz
   */
  void deleteMarkers();
};

} /* namespace mbf_costmap_nav */
#endif /* SEARCH_HELPER_VIZ_H_ */
