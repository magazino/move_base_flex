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

std::vector<geometry_msgs::Point> FreePoseSearch::safetyPadding(const costmap_2d::Costmap2DROS& costmap_2dros,
                                                                const bool use_padded_fp, const double safety_dist)

{
  std::vector<geometry_msgs::Point> footprint;
  footprint = use_padded_fp ? costmap_2dros.getRobotFootprint() : costmap_2dros.getUnpaddedRobotFootprint();
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
  for (const auto& [index, cell] : cells_to_check_map)
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
  bool found_unknown{ false };
  SearchSolution sol;
  sol.pose = pose_2d;

  for (double dyaw = 0; dyaw <= config.angle_tolerance; dyaw += config.angle_increment)
  {
    const std::initializer_list<double> thetas =
        dyaw == 0 ? std::initializer_list<double>{ pose_2d.theta } :
                    std::initializer_list<double>{ pose_2d.theta + dyaw, pose_2d.theta - dyaw };
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
          return { sol.pose, search_state };
          break;
        case SearchState::UNKNOWN:
        case SearchState::OUTSIDE:
          if (!found_unknown)
          {
            // we save the first unknown/outside pose, but we continue searching for a free pose
            sol = { sol.pose, search_state };
            found_unknown = true;
          }
          break;
        case SearchState::LETHAL:
          // if we didn't find a free pose or unknown/outside, we return lethal
          if (!found_unknown)
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

  ROS_DEBUG_STREAM_COND_NAMED(sol.search_state.state == SearchState::LETHAL, LOGNAME.data(),
                              "No valid orientation found for pose x-y-theta ("
                                  << pose_2d.x << ", " << pose_2d.y << ", " << pose_2d.theta << ") with tolerance "
                                  << config.angle_tolerance << " and increment " << config.angle_increment);

  ROS_DEBUG_STREAM_COND_NAMED(sol.search_state.state == SearchState::UNKNOWN, LOGNAME.data(),
                              "Solution is in unknown space for pose x-y-theta ("
                                  << pose_2d.x << ", " << pose_2d.y << ", " << pose_2d.theta << ") with tolerance "
                                  << config.angle_tolerance << " and increment " << config.angle_increment);
  return sol;
}

SearchSolution FreePoseSearch::search() const
{
  const auto costmap2d = costmap_.getCostmap();

  // lock costmap so content doesn't change while adding cell costs
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap2d->getMutex()));

  std::unordered_set<int> in_queue_or_visited;

  const std::vector<geometry_msgs::Point> footprint =
      safetyPadding(costmap_, config_.use_padded_fp, config_.safety_dist);

  SearchSolution sol;
  sol.pose.theta = config_.goal.theta;
  bool found_unknown{ false };

  // initializing queue with the goal cell
  Cell start;
  costmap2d->worldToMap(config_.goal.x, config_.goal.y, start.x, start.y);
  start.cost = costmap2d->getCost(start.x, start.y);
  in_queue_or_visited.insert(costmap2d->getIndex(start.x, start.y));

  std::priority_queue<Cell, std::vector<Cell>, decltype(compare_strategy_)> queue(compare_strategy_);
  queue.push(start);

  // restart markers
  if (viz_)
  {
    viz_->deleteMarkers();
  }

  while (!queue.empty())
  {
    Cell cell = queue.top();
    queue.pop();

    costmap2d->mapToWorld(cell.x, cell.y, sol.pose.x, sol.pose.y);
    ROS_DEBUG_STREAM_NAMED(LOGNAME.data(), "Checking Cell: (" << cell.x << ", " << cell.y << ") with pose: ("
                                                              << sol.pose.x << ", " << sol.pose.y << ", "
                                                              << sol.pose.theta << ")");

    // Note: if the center of the robot is in costmap_2d::NO_INFORMATION, we don't accept it as a solution
    if (cell.cost != costmap_2d::LETHAL_OBSTACLE && cell.cost != costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
        cell.cost != costmap_2d::NO_INFORMATION)
    {
      const auto tested_sol = findValidOrientation(*costmap2d, footprint, sol.pose, config_, viz_);
      // if footprint is free or inscribed, we return the solution
      if (tested_sol.search_state.state == SearchState::FREE || tested_sol.search_state.state == SearchState::INSCRIBED)
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
          !found_unknown)
      {
        ROS_DEBUG_STREAM_NAMED(LOGNAME.data(), "Found unknown/outside pose: " << tested_sol.pose.x << ", "
                                                                              << tested_sol.pose.y << ", "
                                                                              << tested_sol.pose.theta);
        sol = tested_sol;
        found_unknown = true;
      }

      ROS_DEBUG_STREAM_COND_NAMED(tested_sol.search_state.state == SearchState::LETHAL, LOGNAME.data(),
                                  "Footprint in cell " << cell.x << ", " << cell.y << "; pose: (" << sol.pose.x << ", "
                                                       << sol.pose.y << ", " << sol.pose.theta
                                                       << ") is in lethal obstacle or unknown; skipping");
    }

    else
    {
      ROS_DEBUG_STREAM_NAMED(LOGNAME.data(), "Cell: (" << cell.x << ", " << cell.y << ") with pose: (" << sol.pose.x
                                                       << ", " << sol.pose.y << ", " << sol.pose.theta
                                                       << ") is in lethal obstacle or in unknown space; skipping");
    }

    // adding neighbors to queue
    const std::vector<Cell> neighbors = getNeighbors(*costmap2d, cell);
    for (const Cell& neighbor : neighbors)
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
  }

  if (found_unknown)
  {
    // the solution is a no information pose or outside
    ROS_INFO_STREAM_COND_NAMED(sol.search_state.state == SearchState::UNKNOWN, LOGNAME.data(),
                               "The best solution found has NO_INFORMATION cost");
    ROS_INFO_STREAM_COND_NAMED(sol.search_state.state == SearchState::OUTSIDE, LOGNAME.data(),
                               "The best solution found is outside the map");
    if (viz_)
    {
      viz_->addSolution(sol.pose, footprint);
      viz_->publish();
    }
    return sol;
  }

  ROS_INFO_STREAM("No solution found within tolerance of goal; ending search");
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
