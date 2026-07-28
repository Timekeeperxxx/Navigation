#ifndef SCAN_PLANNER__TERRAIN_Z_TRACKER_H_
#define SCAN_PLANNER__TERRAIN_Z_TRACKER_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace scan_planner
{

struct TerrainPathPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TerrainZTrackerConfig
{
  double body_height{0.32};
  double path_timeout{2.0};
  double max_path_slope{0.70};
  double max_z_rate{0.30};
  double max_projection_distance{1.0};
};

struct TerrainZProjection
{
  bool valid{false};
  double target_base_z{0.0};
  double xy_distance{std::numeric_limits<double>::infinity()};
  std::size_t segment_index{0};
};

/**
 * @brief Track the simulated base height from a time-valid body-height path.
 *
 * The controller publishes /scan/execution_path at the SCAN body slice.  This
 * helper projects the simulated base XY position onto that path and subtracts
 * body_height to recover base_footprint z.  It deliberately owns no ROS state
 * so all fail-closed path and rate-limit behavior can be unit tested.
 */
class TerrainZTracker
{
public:
  explicit TerrainZTracker(const TerrainZTrackerConfig &config = TerrainZTrackerConfig())
  : config_(sanitizeConfig(config))
  {
  }

  void configure(const TerrainZTrackerConfig &config)
  {
    config_ = sanitizeConfig(config);
    clearPath();
  }

  const TerrainZTrackerConfig &config() const
  {
    return config_;
  }

  /**
   * @brief Replace the active path after validating every value and segment.
   *
   * Empty paths, non-finite coordinates, vertical XY-degenerate jumps and
   * segments above max_path_slope clear the old path and return false.
   */
  bool setExecutionPath(
      const std::vector<TerrainPathPoint> &points,
      const double receipt_time_seconds)
  {
    clearPath();
    if (points.empty() || !std::isfinite(receipt_time_seconds))
      return false;

    for (const auto &point : points)
    {
      if (!isFinite(point))
        return false;
    }

    constexpr double kMinHorizontalLength = 1e-6;
    constexpr double kMaxDegenerateZDifference = 1e-4;
    for (std::size_t index = 0; index + 1 < points.size(); ++index)
    {
      const double dx = points[index + 1].x - points[index].x;
      const double dy = points[index + 1].y - points[index].y;
      const double dz = points[index + 1].z - points[index].z;
      const double horizontal_length = std::hypot(dx, dy);
      if (horizontal_length <= kMinHorizontalLength)
      {
        if (std::abs(dz) > kMaxDegenerateZDifference)
          return false;
        continue;
      }
      if (std::abs(dz) / horizontal_length > config_.max_path_slope)
        return false;
    }

    points_ = points;
    receipt_time_seconds_ = receipt_time_seconds;
    return true;
  }

  void clearPath()
  {
    points_.clear();
    receipt_time_seconds_ = std::numeric_limits<double>::quiet_NaN();
  }

  bool hasFreshPath(const double now_seconds) const
  {
    if (points_.empty() || !std::isfinite(now_seconds) ||
        !std::isfinite(receipt_time_seconds_))
    {
      return false;
    }
    const double age = now_seconds - receipt_time_seconds_;
    return age >= 0.0 && age <= config_.path_timeout;
  }

  TerrainZProjection project(
      const double base_x, const double base_y, const double current_base_z,
      const double now_seconds) const
  {
    TerrainZProjection result;
    if (!hasFreshPath(now_seconds) || !std::isfinite(base_x) ||
        !std::isfinite(base_y) || !std::isfinite(current_base_z))
    {
      return result;
    }

    auto consider_candidate =
        [&](const double path_x, const double path_y, const double path_z,
            const std::size_t segment_index) {
          const double distance = std::hypot(path_x - base_x, path_y - base_y);
          const double target_base_z = path_z - config_.body_height;
          if (!std::isfinite(distance) || !std::isfinite(target_base_z) ||
              distance > config_.max_projection_distance)
          {
            return;
          }

          constexpr double kDistanceTieTolerance = 1e-9;
          const double candidate_vertical_error =
              std::abs(target_base_z - current_base_z);
          const double current_vertical_error =
              std::abs(result.target_base_z - current_base_z);
          const bool closer =
              !result.valid ||
              distance < result.xy_distance - kDistanceTieTolerance;
          const bool vertically_continuous_tie =
              result.valid &&
              std::abs(distance - result.xy_distance) <=
                  kDistanceTieTolerance &&
              candidate_vertical_error < current_vertical_error;
          if (!closer && !vertically_continuous_tie)
            return;

          result.valid = true;
          result.target_base_z = target_base_z;
          result.xy_distance = distance;
          result.segment_index = segment_index;
        };

    if (points_.size() == 1)
    {
      consider_candidate(
          points_.front().x, points_.front().y, points_.front().z, 0);
      return result;
    }

    constexpr double kMinSegmentLengthSquared = 1e-12;
    for (std::size_t index = 0; index + 1 < points_.size(); ++index)
    {
      const auto &start = points_[index];
      const auto &end = points_[index + 1];
      const double dx = end.x - start.x;
      const double dy = end.y - start.y;
      const double length_squared = dx * dx + dy * dy;
      double ratio = 0.0;
      if (length_squared > kMinSegmentLengthSquared)
      {
        ratio =
            ((base_x - start.x) * dx + (base_y - start.y) * dy) /
            length_squared;
        ratio = std::max(0.0, std::min(1.0, ratio));
      }
      consider_candidate(
          start.x + ratio * dx,
          start.y + ratio * dy,
          start.z + ratio * (end.z - start.z),
          index);
    }
    return result;
  }

  /**
   * @brief Advance z toward the projected terrain with a hard rate limit.
   *
   * Invalid dt, stale/empty/invalid paths, or a projection outside the
   * configured corridor leave current_base_z unchanged.
   */
  double update(
      const double base_x, const double base_y, const double current_base_z,
      const double now_seconds, const double dt_seconds) const
  {
    if (!std::isfinite(current_base_z) || !std::isfinite(dt_seconds) ||
        dt_seconds <= 0.0)
    {
      return current_base_z;
    }

    const TerrainZProjection projection =
        project(base_x, base_y, current_base_z, now_seconds);
    if (!projection.valid)
      return current_base_z;

    const double max_change = config_.max_z_rate * dt_seconds;
    const double requested_change =
        projection.target_base_z - current_base_z;
    const double limited_change =
        std::max(-max_change, std::min(max_change, requested_change));
    const double updated_z = current_base_z + limited_change;
    return std::isfinite(updated_z) ? updated_z : current_base_z;
  }

private:
  static bool isFinite(const TerrainPathPoint &point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
  }

  static TerrainZTrackerConfig sanitizeConfig(
      const TerrainZTrackerConfig &config)
  {
    TerrainZTrackerConfig result = config;
    if (!std::isfinite(result.body_height) || result.body_height < 0.0)
      result.body_height = 0.32;
    if (!std::isfinite(result.path_timeout) || result.path_timeout <= 0.0)
      result.path_timeout = 2.0;
    if (!std::isfinite(result.max_path_slope) ||
        result.max_path_slope <= 0.0)
    {
      result.max_path_slope = 0.70;
    }
    if (!std::isfinite(result.max_z_rate) || result.max_z_rate <= 0.0)
      result.max_z_rate = 0.30;
    if (!std::isfinite(result.max_projection_distance) ||
        result.max_projection_distance <= 0.0)
    {
      result.max_projection_distance = 1.0;
    }
    return result;
  }

  TerrainZTrackerConfig config_;
  std::vector<TerrainPathPoint> points_;
  double receipt_time_seconds_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace scan_planner

#endif  // SCAN_PLANNER__TERRAIN_Z_TRACKER_H_
