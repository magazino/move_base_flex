// mbf_costmap_nav
#include "mbf_costmap_nav/search_helper.h"

// std
#include <queue>
#include <unordered_set>

namespace mbf_costmap_nav
{

SearchHelper::SearchHelper(const costmap_2d::Costmap2DROS* costmap, const SearchConfig& config,
                           const std::optional<std::function<bool(const Cell, const Cell)>>& compare_strategy,
                           const std::optional<SearchHelperViz>& viz)
  : costmap_(costmap), config_(config), viz_(viz)
{
  Cell start;
  costmap_->getCostmap()->worldToMap(config_.goal.x, config_.goal.y, start.x, start.y);
  ROS_DEBUG_STREAM("Start cell: " << start.x << ", " << start.y);
  EuclideanCompare euclidean_compare(start);
  compare_strategy_ = std::move(compare_strategy.value_or(euclidean_compare));
}

std::vector<Cell> SearchHelper::getNeighbors(const costmap_2d::Costmap2D* costmap_2d, const Cell& cell)
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
      if (x < costmap_2d->getSizeInCellsX() && y < costmap_2d->getSizeInCellsY() && x >= 0 && y >= 0)
      {
        neighbors.push_back(Cell{ x, y, costmap_2d->getCost(x, y) });
      }
    }
  }
  return neighbors;
}

std::vector<geometry_msgs::Point> SearchHelper::safetyPadding(const costmap_2d::Costmap2DROS* costmap_2dros,
                                                              const bool use_padded_fp, const double safety_dist)

{
  std::vector<geometry_msgs::Point> footprint;
  footprint = use_padded_fp ? costmap_2dros->getRobotFootprint() : costmap_2dros->getUnpaddedRobotFootprint();
  costmap_2d::padFootprint(footprint, safety_dist);
  return footprint;
}

bool SearchHelper::isPoseValid(const costmap_2d::Costmap2D* costmap_2d,
                               const std::vector<geometry_msgs::Point>& footprint, const geometry_msgs::Pose2D& pose_2d)
{
  const auto cells_to_check =
      FootprintHelper::getFootprintCells(pose_2d.x, pose_2d.y, pose_2d.theta, footprint, *costmap_2d, true);
  if (cells_to_check.empty())
  {
    return false;
  }

  for (int j = 0; j < cells_to_check.size(); ++j)
  {
    unsigned char cost = costmap_2d->getCost(cells_to_check[j].x, cells_to_check[j].y);
    switch (cost)
    {
      case costmap_2d::NO_INFORMATION:
        return false;
        break;
      case costmap_2d::LETHAL_OBSTACLE:
        return false;
        break;
      default:
        break;
    }
  }
  return true;
}

std::optional<geometry_msgs::Pose2D> SearchHelper::findValidOrientation(
    const costmap_2d::Costmap2D* costmap_2d, const std::vector<geometry_msgs::Point>& footprint,
    const geometry_msgs::Pose2D& pose_2d, const SearchConfig& config, std::optional<SearchHelperViz>& viz)
{
  // loop through angle increments and check if footprint is valid. If it is, return the pose
  geometry_msgs::Pose2D test_pose_2d = pose_2d;
  double dyaw = 0;
  for (; dyaw <= config.angle_tolerance; dyaw += config.angle_increment)
  {
    for (const auto& theta : { pose_2d.theta + dyaw, pose_2d.theta - dyaw })
    {
      test_pose_2d.theta = theta;
      if (isPoseValid(costmap_2d, footprint, test_pose_2d))
      {
        if (viz)
        {
          viz->addSolution(test_pose_2d, footprint);
        }
        return test_pose_2d;
      }
      else
      {
        if (viz)
        {
          viz->addBlocked(test_pose_2d, footprint);
        }
      }
    }
  }
  ROS_DEBUG_STREAM("No valid orientation found for pose x-y-theta ("
                   << pose_2d.x << ", " << pose_2d.y << ", " << pose_2d.theta << ") with tolerance "
                   << config.angle_tolerance << " and increment " << config.angle_increment);
  return std::nullopt;
}

bool SearchHelper::search(geometry_msgs::Pose2D& best) const
{
  const auto costmap2d = costmap_->getCostmap();

  // lock costmap so content doesn't change while adding cell costs
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap2d->getMutex()));

  std::unordered_set<int> in_queue_or_visited;

  std::vector<geometry_msgs::Point> footprint = safetyPadding(costmap_, config_.use_padded_fp, config_.safety_dist);

  geometry_msgs::Pose2D pose_2d;
  pose_2d.theta = config_.goal.theta;

  Cell start;
  costmap2d->worldToMap(config_.goal.x, config_.goal.y, start.x, start.y);
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

    costmap2d->mapToWorld(cell.x, cell.y, pose_2d.x, pose_2d.y);
    ROS_DEBUG_STREAM("Checking Cell: (" << cell.x << ", " << cell.y << ") with pose: (" << pose_2d.x << ", "
                                        << pose_2d.y << ", " << pose_2d.theta << ")");
    if (cell.cost != costmap_2d::LETHAL_OBSTACLE && cell.cost != costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
        cell.cost != costmap_2d::NO_INFORMATION)
    {
      const auto pose = findValidOrientation(costmap2d, footprint, pose_2d, config_, viz_);
      if (pose)
      {
        ROS_DEBUG_STREAM("Found solution pose: " << pose->x << ", " << pose->y << ", " << pose->theta);
        best = pose.value();
        if (viz_)
        {
          viz_->publish();
        }
        return true;
      }
    }
    else
    {
      ROS_DEBUG_STREAM("Cell " << cell.x << ", " << cell.y << "; pose: (" << pose_2d.x << ", " << pose_2d.y << ", "
                               << pose_2d.theta << ") is an obstacle or unknown; skipping");
      if (viz_)
      {
        viz_->addBlocked(pose_2d, footprint);
      }
    }

    // adding neighbors to queue
    std::vector<Cell> neighbors = getNeighbors(costmap2d, cell);
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
  ROS_DEBUG_STREAM("No solution found within tolerance of goal; ending search");
  if (viz_)
  {
    viz_->publish();
  }
  return false;
}
} /* namespace mbf_costmap_nav */
