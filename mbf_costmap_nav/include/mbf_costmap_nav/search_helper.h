/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Willow Garage, Inc.
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
 *********************************************************************/

#ifndef SEARCH_HELPER_H_
#define SEARCH_HELPER_H_

// mbf_costmap_nav
#include "mbf_costmap_nav/footprint_helper.h"

// std
#include <optional>

// ros
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace mbf_costmap_nav
{

struct SearchConfig
{
  double angle_increment;
  double angle_tolerance;
  double linear_tolerance;
  bool use_padded_fp;
  double safety_dist;
  geometry_msgs::Pose2D goal;
};

class EuclideanCompare
{
private:
  Cell start_;
  int start_x_;
  int start_y_;

public:
  EuclideanCompare(const Cell& start) : start_(start)
  {
    start_x_ = static_cast<int>(start.x);
    start_y_ = static_cast<int>(start.y);
  }
  bool operator()(const Cell& a, const Cell& b)
  {
    int ax = static_cast<int>(a.x);
    int bx = static_cast<int>(b.x);
    int ay = static_cast<int>(a.y);
    int by = static_cast<int>(b.y);
    return std::hypot(ax - start_x_, ay - start_y_) > std::hypot(bx - start_x_, by - start_y_);
  }
};

/**
 * @brief It performs a search on the costmap from start to given distance until it finds a cell where we can place the
 * robot without colliding with obstacles.
 * It is a Breadth-depth search performed in a spiral pattern (neighboring cells added in a priority queue), starting
 * from the start (given) cell. And the cells are ordered by euclidean distance from the start cell (by default but one
 * can add a new strategy).
 * The search stops when it finds a cell where the robot can be placed without colliding with obstacles (footprint +
 * padding).
 * For each cell, we test multiple angles (by default 8).
 * The search is performed on the costmap given in the constructor.
 */
class SearchHelper
{
private:
  const costmap_2d::Costmap2DROS* costmap_;
  SearchConfig config_;
  std::function<bool(const Cell, const Cell)> compare_strategy_;

public:
  SearchHelper(const costmap_2d::Costmap2DROS* costmap, const SearchConfig& config,
               const std::optional<std::function<bool(const Cell, const Cell)>>& compare_strategy = std::nullopt);

  /**
   * @brief It returns the eight neighbors of the given cell
   */
  static std::vector<Cell> getNeighbors(const costmap_2d::Costmap2D* costmap_2d, const Cell& cell);

  /**
   * @brief it pads the footprint with the given safety distance
   * @param costmap_2dros
   * @param use_padded_fp If true, it uses the padded footprint, otherwise the unpadded footprint
   * @param safety_dist The safety distance to pad the footprint
   * @return The padded footprint
   */
  static std::vector<geometry_msgs::Point> safetyPadding(const costmap_2d::Costmap2DROS* costmap_2dros,
                                                         const bool use_padded_fp, const double safety_dist);

  /**
   * @brief It checks if the pose of the footprint is valid, i.e., if cost !=
   * INSCRIBED and cost != LETHAL and cost != NO_INFORMATION (if unknown_is_valid is false)
   * @param costmap The costmap2D
   * @param footprint The footprint to check
   * @param pose_2d The pose to check the footprint
   * @return True if the pose is valid, false otherwise
   */
  static bool isPoseValid(const costmap_2d::Costmap2D* costmap_2d, const std::vector<geometry_msgs::Point>& footprint,
                          const geometry_msgs::Pose2D& pose_2d);

  /**
   * @brief It loops in the given angle increments and checks if the pose of the footprint is valid (collision free)
   * It returns the first valid pose found.
   * @param costmap_2d The costmap2d
   * @param footprint The footprint to check
   * @param pose_2d The pose to check the footprint
   * @param config The search configuration
   * @return The first valid pose found, or an empty optional if no valid pose was found
   */
  static std::optional<geometry_msgs::Pose2D> findValidOrientation(const costmap_2d::Costmap2D* costmap_2d,
                                                                   const std::vector<geometry_msgs::Point>& footprint,
                                                                   const geometry_msgs::Pose2D& pose_2d,
                                                                   const SearchConfig& config);

  /**
   * @brief It performs the search on the costmap, see the class description for more details.
   * @param start The start cell
   * @param[out] best The best cell found
   * @return True if a cell was found, false otherwise
   */
  bool search(geometry_msgs::Pose2D& best) const;
};

} /* namespace mbf_costmap_nav */
#endif /* SEARCH_HELPER_H_ */
