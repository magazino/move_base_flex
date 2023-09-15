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
 *  free_pose_search.cpp
 *
 *  authors:
 *    Renan Salles <renan028@gmail.com>
 *
 */

// mbf_costmap_nav
#include "mbf_costmap_nav/free_pose_search.h"

// std
#include <queue>
#include <unordered_set>

namespace mbf_costmap_nav
{

FreePoseSearch::FreePoseSearch(costmap_2d::Costmap2DROS& costmap, const SearchConfig& config,
                               const std::optional<std::function<bool(const Cell&, const Cell&)>>& compare_strategy,
                               const std::optional<FreePoseSearchViz>& viz)
  : costmap_(costmap), config_(config), viz_(viz)
{
  Cell start;
  costmap_.getCostmap()->worldToMap(config_.goal.x, config_.goal.y, start.x, start.y);
  ROS_DEBUG_STREAM("Start cell: " << start.x << ", " << start.y);
  EuclideanCompare euclidean_compare(start);
  compare_strategy_ = std::move(compare_strategy.value_or(euclidean_compare));
}

bool FreePoseSearch::isPoseValid(const unsigned char cost) const
{
  return cost != costmap_2d::LETHAL_OBSTACLE && cost != costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
         cost != costmap_2d::NO_INFORMATION;
}

bool FreePoseSearch::isStateValid(const std::uint8_t state) const
{
  return state == mbf_costmap_nav::SearchState::FREE || state == mbf_costmap_nav::SearchState::INSCRIBED;
}

std::vector<Cell> FreePoseSearch::getNeighbors(const costmap_2d::Costmap2D& costmap_2d, const Cell& cell)
{
  std::vector<Cell> neighbors;
  neighbors.reserve(8);
  for (int dx = -1; dx <= 1; ++dx)
  {
    for (int dy = -1; dy <= 1; ++dy)
    {
      if (dx == 0 && dy == 0)
      {
        continue;
      }
      unsigned int x = cell.x + dx;
      unsigned int y = cell.y + dy;
      if (x < costmap_2d.getSizeInCellsX() && y < costmap_2d.getSizeInCellsY() && x >= 0 && y >= 0)
      {
        neighbors.push_back(Cell{ x, y, costmap_2d.getCost(x, y) });
      }
    }
  }
  return neighbors;
}

std::vector<geometry_msgs::Point> FreePoseSearch::safetyPadding(costmap_2d::Costmap2DROS& costmap_2d_ros,
                                                                const bool use_padded_fp, const double safety_dist)

{
  std::vector<geometry_msgs::Point> footprint =
      use_padded_fp ? costmap_2d_ros.getRobotFootprint() : costmap_2d_ros.getUnpaddedRobotFootprint();
  costmap_2d::padFootprint(footprint, safety_dist);
  return footprint;
}

SearchState FreePoseSearch::getFootprintState(const costmap_2d::Costmap2D& costmap_2d,
                                              const std::vector<geometry_msgs::Point>& footprint,
                                              const geometry_msgs::Pose2D& pose_2d)
{
  const auto cells_to_check =
      FootprintHelper::getFootprintCells(pose_2d.x, pose_2d.y, pose_2d.theta, footprint, costmap_2d, true);
  if (cells_to_check.empty())
  {
    return { costmap_2d::NO_INFORMATION, SearchState::OUTSIDE };
  }

  // create a map of cells to avoid duplicates
  std::unordered_map<int, Cell> cells_to_check_map;
  for (const auto& cell : cells_to_check)
  {
    cells_to_check_map[costmap_2d.getIndex(cell.x, cell.y)] = cell;
  }

  unsigned char max_cost = 0;
  for (const auto& [_, cell] : cells_to_check_map)
  {
    unsigned char cost = costmap_2d.getCost(cell.x, cell.y);
    switch (cost)
    {
      case costmap_2d::LETHAL_OBSTACLE:
        return { costmap_2d::LETHAL_OBSTACLE, SearchState::LETHAL };
        break;
      default:
        max_cost = std::max(max_cost, cost);
        break;
    }
  }
  uint8_t state = max_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ? SearchState::INSCRIBED :
                  max_cost == costmap_2d::NO_INFORMATION              ? SearchState::UNKNOWN :
                                                                        SearchState::FREE;
  return { max_cost, state };
}

SearchSolution FreePoseSearch::findValidOrientation(const costmap_2d::Costmap2D& costmap_2d,
                                                    const std::vector<geometry_msgs::Point>& footprint,
                                                    const geometry_msgs::Pose2D& pose_2d, const SearchConfig& config,
                                                    std::optional<FreePoseSearchViz>& viz)
{
  bool outside_or_unknown = false;
  SearchSolution sol;
  SearchSolution outside_or_unknown_sol;
  sol.pose = pose_2d;

  const double reduced_tol = std::min(config.angle_tolerance - std::numeric_limits<double>::epsilon() * M_PI, M_PI);
  const int num_steps = 1 + std::max(0, static_cast<int>(std::ceil(reduced_tol / config.angle_max_step_size)));
  const double increment = reduced_tol / std::max(1, num_steps - 1);
  for (int i = 0; i < num_steps; ++i)
  {
    const std::vector<double> thetas =
        i == 0 ? std::vector<double>{ pose_2d.theta } :
                 std::vector<double>{ pose_2d.theta + i * increment, pose_2d.theta - i * increment };

    for (const auto& theta : thetas)
    {
      sol.pose.theta = theta;
      SearchState search_state = getFootprintState(costmap_2d, footprint, sol.pose);

      switch (search_state.state)
      {
        case SearchState::FREE:
        case SearchState::INSCRIBED:
          if (viz)
          {
            viz->addSolution(sol.pose, footprint);
          }
          ROS_DEBUG_STREAM_NAMED(LOGNAME.data(), "Found solution at: " << sol.pose.x << ", " << sol.pose.y << ", "
                                                                       << sol.pose.theta
                                                                       << " with state: " << search_state.state);
          return { sol.pose, search_state };
        case SearchState::UNKNOWN:
        case SearchState::OUTSIDE:
          if (!outside_or_unknown)
          {
            // we save the first unknown/outside pose, but we continue searching for a free pose
            outside_or_unknown_sol = { sol.pose, search_state };
            outside_or_unknown = true;
          }
          break;
        case SearchState::LETHAL:
          // if we didn't find a free pose or unknown/outside, we return lethal
          if (!outside_or_unknown)
          {
            sol.search_state = search_state;
          }
          if (viz)
          {
            viz->addBlocked(sol.pose, footprint);
          }
          break;
        default:
          ROS_ERROR_STREAM("Unknown state: " << search_state.state);
          break;
      }
    }
  }

  if (outside_or_unknown)
  {
    ROS_DEBUG_STREAM_COND_NAMED(outside_or_unknown_sol.search_state.state == SearchState::UNKNOWN, LOGNAME.data(),
                                "Solution is in unknown space for pose x-y-theta ("
                                    << pose_2d.x << ", " << pose_2d.y << ", " << pose_2d.theta << ") with tolerance "
                                    << config.angle_tolerance << " and increment " << config.angle_max_step_size);

    ROS_DEBUG_STREAM_COND_NAMED(outside_or_unknown_sol.search_state.state == SearchState::OUTSIDE, LOGNAME.data(),
                                "Solution is partially outside the map for pose x-y-theta ("
                                    << pose_2d.x << ", " << pose_2d.y << ", " << pose_2d.theta << ") with tolerance "
                                    << config.angle_tolerance << " and increment " << config.angle_max_step_size);
    return outside_or_unknown_sol;
  }

  ROS_DEBUG_STREAM_COND_NAMED(sol.search_state.state == SearchState::LETHAL, LOGNAME.data(),
                              "No valid orientation found for pose x-y-theta ("
                                  << pose_2d.x << ", " << pose_2d.y << ", " << pose_2d.theta << ") with tolerance "
                                  << config.angle_tolerance << " and increment " << config.angle_max_step_size);

  return sol;
}

SearchSolution FreePoseSearch::search() const
{
  // restart markers
  if (viz_)
  {
    viz_->deleteMarkers();
  }

  const auto costmap2d = costmap_.getCostmap();

  // lock costmap so content doesn't change while adding cell costs
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap2d->getMutex()));

  std::unordered_set<int> in_queue_or_visited;
  std::priority_queue<Cell, std::vector<Cell>, decltype(compare_strategy_)> queue(compare_strategy_);

  const std::vector<geometry_msgs::Point> footprint =
      safetyPadding(costmap_, config_.use_padded_fp, config_.safety_dist);

  // enforce bounds if goal is outside the map; and get the pose of the center of the cell
  int goal_cell_x, goal_cell_y;
  costmap2d->worldToMapEnforceBounds(config_.goal.x, config_.goal.y, goal_cell_x, goal_cell_y);
  Cell goal_cell = { static_cast<unsigned int>(goal_cell_x), static_cast<unsigned int>(goal_cell_y),
                     costmap2d->getCost(goal_cell_x, goal_cell_y) };

  // this is the pose of the center of the goal cell (not the goal pose)
  geometry_msgs::Pose2D goal_cell_pose;
  costmap2d->mapToWorld(goal_cell_x, goal_cell_y, goal_cell_pose.x, goal_cell_pose.y);

  // add goal cell to queue if it is within linear tolerance
  if (std::hypot(goal_cell_pose.x - config_.goal.x, goal_cell_pose.y - config_.goal.y) <= config_.linear_tolerance)
  {
    queue.push(goal_cell);
    in_queue_or_visited.insert(costmap2d->getIndex(goal_cell_x, goal_cell_y));
  }

  // don't start the search from the goal pose if goal is not within bounds
  unsigned int dummy_x, dummy_y;
  bool test_goal_pose = costmap2d->worldToMap(config_.goal.x, config_.goal.y, dummy_x, dummy_y);

  SearchSolution sol;
  std::optional<SearchSolution> no_info_sol;
  while (!queue.empty() || test_goal_pose)
  {
    sol.pose.theta = config_.goal.theta;
    Cell test_cell;
    if (test_goal_pose)
    {
      test_cell = goal_cell;
      sol.pose.x = config_.goal.x;
      sol.pose.y = config_.goal.y;
      test_goal_pose = false;
    }
    else
    {
      test_cell = queue.top();
      queue.pop();
      costmap2d->mapToWorld(test_cell.x, test_cell.y, sol.pose.x, sol.pose.y);
    }

    ROS_DEBUG_STREAM_NAMED(LOGNAME.data(), "Checking Cell: (" << test_cell.x << ", " << test_cell.y << ") with pose: ("
                                                              << sol.pose.x << ", " << sol.pose.y << ", "
                                                              << sol.pose.theta << ")");

    // Note: if the center of the robot is in costmap_2d::NO_INFORMATION, we don't accept it as a solution
    if (isPoseValid(test_cell.cost))
    {
      const auto tested_sol = findValidOrientation(*costmap2d, footprint, sol.pose, config_, viz_);
      // if footprint is free or inscribed, we return the solution
      if (isStateValid(tested_sol.search_state.state))
      {
        ROS_DEBUG_STREAM_NAMED(LOGNAME.data(), "Found solution pose: " << tested_sol.pose.x << ", " << tested_sol.pose.y
                                                                       << ", " << tested_sol.pose.theta);
        if (viz_)
        {
          viz_->publish();
        }
        return tested_sol;
      }

      // if the state is outside or unknown, we save the first one we find
      if ((tested_sol.search_state.state == SearchState::OUTSIDE ||
           tested_sol.search_state.state == SearchState::UNKNOWN) &&
          !no_info_sol)
      {
        ROS_DEBUG_STREAM_NAMED(LOGNAME.data(), "Found unknown/outside pose: " << tested_sol.pose.x << ", "
                                                                              << tested_sol.pose.y << ", "
                                                                              << tested_sol.pose.theta);
        no_info_sol = tested_sol;
      }

      ROS_DEBUG_STREAM_COND_NAMED(tested_sol.search_state.state == SearchState::LETHAL, LOGNAME.data(),
                                  "Footprint in cell " << test_cell.x << ", " << test_cell.y << "; pose: ("
                                                       << sol.pose.x << ", " << sol.pose.y << ", " << sol.pose.theta
                                                       << ") is in lethal obstacle or unknown; skipping");
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(LOGNAME.data(), "Cell: (" << test_cell.x << ", " << test_cell.y << ") with pose: ("
                                                       << sol.pose.x << ", " << sol.pose.y << ", " << sol.pose.theta
                                                       << ") is in lethal obstacle or in unknown space; skipping");
    }

    // adding neighbors to queue
    const std::vector<Cell> neighbors = getNeighbors(*costmap2d, test_cell);
    for (const auto& neighbor : neighbors)
    {
      int cell_index = costmap2d->getIndex(neighbor.x, neighbor.y);
      if (in_queue_or_visited.find(cell_index) != in_queue_or_visited.end())
      {
        ROS_DEBUG_STREAM("Cell " << neighbor.x << ", " << neighbor.y << " already in queue or visited; skipping");
        continue;
      }

      // check if in linear tolerance distance
      geometry_msgs::Pose2D cell_world_pose;
      costmap2d->mapToWorld(neighbor.x, neighbor.y, cell_world_pose.x, cell_world_pose.y);
      if (std::hypot(cell_world_pose.x - config_.goal.x, cell_world_pose.y - config_.goal.y) > config_.linear_tolerance)
      {
        ROS_DEBUG_STREAM("Cell " << neighbor.x << ", " << neighbor.y
                                 << " is not within linear tolerance of goal; skipping");
        continue;
      }
      ROS_DEBUG_STREAM("Adding cell " << neighbor.x << ", " << neighbor.y << " to queue");
      in_queue_or_visited.insert(cell_index);
      queue.push(neighbor);
    }
  }  // end while

  if (no_info_sol)
  {
    // the solution is a no information pose or outside
    ROS_DEBUG_STREAM_COND_NAMED(no_info_sol->search_state.state == SearchState::UNKNOWN, LOGNAME.data(),
                                "The best solution found has NO_INFORMATION cost");
    ROS_DEBUG_STREAM_COND_NAMED(no_info_sol->search_state.state == SearchState::OUTSIDE, LOGNAME.data(),
                                "The best solution found is OUTSIDE the map");
    if (viz_)
    {
      viz_->addSolution(no_info_sol->pose, footprint);
      viz_->publish();
    }
    return no_info_sol.value();
  }

  ROS_DEBUG_STREAM("No solution found within tolerance of goal; ending search");
  if (viz_)
  {
    viz_->publish();
  }
  sol.search_state.state = SearchState::LETHAL;
  sol.search_state.cost = costmap_2d::LETHAL_OBSTACLE;
  sol.pose = config_.goal;
  return sol;
}
} /* namespace mbf_costmap_nav */
