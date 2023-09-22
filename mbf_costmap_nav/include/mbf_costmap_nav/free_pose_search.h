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
 *  free_pose_search.h
 *
 *  authors:
 *    Renan Salles <renan028@gmail.com>
 *
 */

#ifndef SEARCH_HELPER_H_
#define SEARCH_HELPER_H_

// mbf
#include "mbf_costmap_nav/footprint_helper.h"
#include "mbf_costmap_nav/free_pose_search_viz.h"

// std
#include <optional>
#include <cstdint>
#include <string_view>

// ros
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

namespace mbf_costmap_nav
{

struct SearchConfig
{
  double angle_max_step_size{ 5 * M_PI / 180 };  // 5 degrees
  double angle_tolerance{ M_PI_2 };              // 90 degrees
  double linear_tolerance{ 1.0 };                // 1 meter
  bool use_padded_fp{ true };                    // Padded footprint by default
  double safety_dist{ 0.1 };                     // 10 cm
  geometry_msgs::Pose2D goal;
};

struct SearchState
{
  unsigned char cost;
  std::uint8_t state{ State::UNSET };

  enum State
  {
    FREE = 0,
    INSCRIBED = 1,
    LETHAL = 2,
    UNKNOWN = 3,
    OUTSIDE = 4,
    UNSET = 5
  };
};

struct SearchSolution
{
  geometry_msgs::Pose2D pose;
  SearchState search_state;
};

/**
 * @brief Euclidean Compare parameter for priority queue, defined such that it returns true if its first argument comes
 * last its second argument. The Euclidean distance is calculated from the start cell.
 * The start cell is given in the constructor, and it is the goal of the search.
 */
class EuclideanCompare
{
private:
  Cell start_;
  int start_x_;
  int start_y_;

public:
  explicit EuclideanCompare(const Cell& start) : start_(start)
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
 * For each cell, we test multiple angles.
 * The search is performed on the costmap given in the constructor.
 */
class FreePoseSearch
{
protected:
  static constexpr std::string_view LOGNAME = "free_pose_search";

  costmap_2d::Costmap2DROS& costmap_;
  SearchConfig config_;
  std::function<bool(const Cell, const Cell)> compare_strategy_;

  mutable std::optional<FreePoseSearchViz> viz_;

public:
  FreePoseSearch(costmap_2d::Costmap2DROS& costmap, const SearchConfig& config,
                 const std::optional<std::function<bool(const Cell&, const Cell&)>>& compare_strategy = std::nullopt,
                 const std::optional<FreePoseSearchViz>& viz = std::nullopt);
  virtual ~FreePoseSearch() = default;

  /**
   * @brief It returns the eight neighbors of the given cell
   */
  static std::vector<Cell> getNeighbors(const costmap_2d::Costmap2D& costmap_2d, const Cell& cell);

  /**
   * @brief it pads the footprint with the given safety distance
   * @param costmap_2d_ros
   * @param use_padded_fp If true, it uses the padded footprint, otherwise the unpadded footprint
   * @param safety_dist The safety distance to pad the footprint
   * @return The padded footprint
   */
  static std::vector<geometry_msgs::Point> safetyPadding(costmap_2d::Costmap2DROS& costmap_2d_ros,
                                                         const bool use_padded_fp, const double safety_dist);

  /**
   * @brief It gets the cost and state of the footprint by checking the max cost of all cells that the footprint covers
   * It returns costmap_2d::LETHAL_OBSTACLE if any of the cells is lethal; otherwise, returns costmap_2d::NO_INFORMATION
   * if any of the cells is unknown; otherwise the maximum cost of all cells.
   * See FindValidPose msg for possible state values.
   * @param costmap The costmap2D
   * @param footprint The footprint to check
   * @param pose_2d The pose to check the footprint
   * @return The SearchState of the footprint (FindValidPose.msg state and costmap cost)
   */
  static SearchState getFootprintState(const costmap_2d::Costmap2D& costmap_2d,
                                       const std::vector<geometry_msgs::Point>& footprint,
                                       const geometry_msgs::Pose2D& pose_2d);

  /**
   * @brief It loops in the given angle increments and checks if the pose of the footprint is valid (collision free)
   * It returns the first valid pose found.
   * @param costmap_2d The costmap2d
   * @param footprint The footprint to check
   * @param pose_2d The pose to check the footprint
   * @param config The search configuration
   * @param viz The visualization object
   * @return A search solution for the given pose: best pose, state and cost
   */
  static SearchSolution findValidOrientation(const costmap_2d::Costmap2D& costmap_2d,
                                             const std::vector<geometry_msgs::Point>& footprint,
                                             const geometry_msgs::Pose2D& pose_2d, const SearchConfig& config,
                                             std::optional<FreePoseSearchViz>& viz);

  /**
   * @brief It performs the search on the costmap, see the class description for more details.
   * @param goal The start cell
   * @return A search solution for the given pose: best pose, state and cost
   */
  virtual SearchSolution search() const;

private:
  /**
   * @brief Check if pose is collision-free; criteria: cost is not LETHAL, INSCRIBED and NO_INFORMATION
   * @param cost
   * @return true if cost is not LETHAL, INSCRIBED and NO_INFORMATION
   */
  bool isPoseValid(const unsigned char cost) const;

  /**
   * @brief Check if state is non-lethal; criteria: state free or inscribed
   * @param state
   * @return true if state is free or inscribed
   */
  bool isStateValid(const std::uint8_t state) const;
};

} /* namespace mbf_costmap_nav */
#endif /* SEARCH_HELPER_H_ */
