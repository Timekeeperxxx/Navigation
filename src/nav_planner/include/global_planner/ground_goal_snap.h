#ifndef GLOBAL_PLANNER__GROUND_GOAL_SNAP_H_
#define GLOBAL_PLANNER__GROUND_GOAL_SNAP_H_

#include <cmath>
#include <cstddef>
#include <limits>

namespace global_planner
{

struct GroundGoalSnapConfig
{
  double maximum_xy_distance{0.50};
  double maximum_layer_distance{0.75};
  double xy_layer_tie_distance{0.15};
};

struct GroundGoalSnapResult
{
  bool valid{false};
  std::size_t index{0};
  double snapped_z{0.0};
  double xy_distance{std::numeric_limits<double>::infinity()};
  double layer_distance{std::numeric_limits<double>::infinity()};
};

/**
 * @brief Find the ground layer under an XY-only RViz goal.
 *
 * RViz's 2D Goal Pose tool always publishes z=0. A normal 3D radius search
 * therefore rejects valid clicks whenever the map datum is not near zero.
 * First find the spatially closest ground samples, then use preferred_z to
 * disambiguate overlapping floors among samples with nearly identical XY.
 */
template<typename PointContainer>
GroundGoalSnapResult findGroundGoalSnap(
  const PointContainer & points,
  const double goal_x,
  const double goal_y,
  const double preferred_z,
  const GroundGoalSnapConfig & config = GroundGoalSnapConfig())
{
  GroundGoalSnapResult result;
  if (
    !std::isfinite(goal_x) ||
    !std::isfinite(goal_y) ||
    !std::isfinite(preferred_z) ||
    !std::isfinite(config.maximum_xy_distance) ||
    config.maximum_xy_distance <= 0.0 ||
    !std::isfinite(config.maximum_layer_distance) ||
    config.maximum_layer_distance <= 0.0)
  {
    return result;
  }

  const double maximum_xy_squared =
    config.maximum_xy_distance * config.maximum_xy_distance;
  double nearest_xy = std::numeric_limits<double>::infinity();

  for (const auto & point : points)
  {
    const double point_x = static_cast<double>(point.x);
    const double point_y = static_cast<double>(point.y);
    const double point_z = static_cast<double>(point.z);
    if (
      !std::isfinite(point_x) ||
      !std::isfinite(point_y) ||
      !std::isfinite(point_z) ||
      std::abs(point_z - preferred_z) > config.maximum_layer_distance)
    {
      continue;
    }

    const double dx = point_x - goal_x;
    const double dy = point_y - goal_y;
    const double xy_squared = dx * dx + dy * dy;
    if (xy_squared <= maximum_xy_squared) {
      nearest_xy = std::min(nearest_xy, std::sqrt(xy_squared));
    }
  }

  if (!std::isfinite(nearest_xy)) {
    return result;
  }

  const double tie_distance =
    std::isfinite(config.xy_layer_tie_distance) ?
    std::max(0.0, config.xy_layer_tie_distance) : 0.0;
  const double spatial_limit = std::min(
    config.maximum_xy_distance, nearest_xy + tie_distance);
  const double spatial_limit_squared = spatial_limit * spatial_limit;
  constexpr double kTieTolerance = 1e-9;

  std::size_t index = 0;
  for (const auto & point : points)
  {
    const double point_x = static_cast<double>(point.x);
    const double point_y = static_cast<double>(point.y);
    const double point_z = static_cast<double>(point.z);
    if (
      !std::isfinite(point_x) ||
      !std::isfinite(point_y) ||
      !std::isfinite(point_z))
    {
      ++index;
      continue;
    }

    const double dx = point_x - goal_x;
    const double dy = point_y - goal_y;
    const double xy_squared = dx * dx + dy * dy;
    const double layer_distance = std::abs(point_z - preferred_z);
    if (
      xy_squared > spatial_limit_squared ||
      layer_distance > config.maximum_layer_distance)
    {
      ++index;
      continue;
    }

    const double xy_distance = std::sqrt(xy_squared);
    const bool closer_layer =
      !result.valid ||
      layer_distance < result.layer_distance - kTieTolerance;
    const bool same_layer_closer_xy =
      result.valid &&
      std::abs(layer_distance - result.layer_distance) <= kTieTolerance &&
      xy_distance < result.xy_distance;
    if (closer_layer || same_layer_closer_xy)
    {
      result.valid = true;
      result.index = index;
      result.snapped_z = point_z;
      result.xy_distance = xy_distance;
      result.layer_distance = layer_distance;
    }
    ++index;
  }
  return result;
}

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__GROUND_GOAL_SNAP_H_
