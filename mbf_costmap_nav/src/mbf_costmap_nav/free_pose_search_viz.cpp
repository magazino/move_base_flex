// mbf
#include "mbf_costmap_nav/free_pose_search_viz.h"

// ros
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mbf_costmap_nav
{
FreePoseSearchViz::FreePoseSearchViz(ros::NodeHandle& pnh, const std::string& frame_id)
  : pnh_(pnh)
  , frame_id_(frame_id)
  , marker_pub_(pnh_.advertise<visualization_msgs::MarkerArray>("search_markers", 1, false))
{
}

void FreePoseSearchViz::addMarker(const geometry_msgs::Pose2D& pose_2d,
                                  const std::vector<geometry_msgs::Point>& footprint, const std::string& ns,
                                  const std_msgs::ColorRGBA& color)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, pose_2d.theta);

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = marker_id_++;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pose_2d.x;
  marker.pose.position.y = pose_2d.y;
  marker.pose.position.z = 0;
  marker.pose.orientation = tf2::toMsg(q);
  marker.scale.x = 0.05;
  marker.color = color;
  marker.lifetime = ros::Duration(0);

  for (const auto& point : footprint)
  {
    geometry_msgs::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = 0;
    marker.points.push_back(p);
  }
  marker.points.push_back(marker.points.front());
  marker_array_.markers.push_back(marker);
}

void FreePoseSearchViz::deleteMarkers()
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.ns = BLOCKED_NS;
  marker_array.markers.push_back(marker);
  marker.ns = SOLUTION_NS;
  marker_array.markers.push_back(marker);
  marker_pub_.publish(marker_array);
}

void FreePoseSearchViz::addBlocked(const geometry_msgs::Pose2D& pose_2d,
                                   const std::vector<geometry_msgs::Point>& footprint)
{
  std_msgs::ColorRGBA color;
  color.r = 1;
  color.a = 0.5;
  addMarker(pose_2d, footprint, BLOCKED_NS, color);
}

void FreePoseSearchViz::addSolution(const geometry_msgs::Pose2D& pose_2d,
                                    const std::vector<geometry_msgs::Point>& footprint)
{
  std_msgs::ColorRGBA color;
  color.g = 1;
  color.a = 0.5;
  addMarker(pose_2d, footprint, SOLUTION_NS, color);
}

void FreePoseSearchViz::publish()
{
  marker_pub_.publish(marker_array_);
  ROS_DEBUG("Published %zu markers", marker_array_.markers.size());
  marker_array_.markers.clear();
  marker_id_ = 0;
}
}  // namespace mbf_costmap_nav
