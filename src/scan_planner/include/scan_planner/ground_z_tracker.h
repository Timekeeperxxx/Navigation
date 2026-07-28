#ifndef SCAN_PLANNER__GROUND_Z_TRACKER_H_
#define SCAN_PLANNER__GROUND_Z_TRACKER_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace scan_planner
{

struct GroundSurfacePoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct GroundZTrackerConfig
{
  double bucket_size{0.20};
  double xy_tolerance{0.15};
  double maximum_layer_distance{0.50};
  double max_z_rate{0.30};
};

struct GroundZProjection
{
  bool valid{false};
  double target_base_z{0.0};
  double xy_distance{std::numeric_limits<double>::infinity()};
  double vertical_distance{std::numeric_limits<double>::infinity()};
};

/**
 * @brief Track simulated base_footprint height directly from /mapground.
 *
 * Execution trajectories are consumers of simulated odometry, so deriving
 * odometry z back from /scan/execution_path creates a circular dependency.
 * This index instead chooses the nearest XY ground sample on the vertically
 * continuous layer. Overlapping floors are disambiguated by
 * maximum_layer_distance.
 */
class GroundZTracker
{
public:
  explicit GroundZTracker(
      const GroundZTrackerConfig &config = GroundZTrackerConfig())
  : config_(sanitizeConfig(config))
  {
  }

  void configure(const GroundZTrackerConfig &config)
  {
    config_ = sanitizeConfig(config);
    clear();
  }

  const GroundZTrackerConfig &config() const
  {
    return config_;
  }

  bool setGroundPoints(const std::vector<GroundSurfacePoint> &points)
  {
    clear();
    for (const auto &point : points)
    {
      if (!isFinite(point))
        continue;
      buckets_[keyFor(point.x, point.y)].push_back(point);
      ++point_count_;
    }
    return point_count_ > 0;
  }

  void clear()
  {
    buckets_.clear();
    point_count_ = 0;
  }

  bool ready() const
  {
    return point_count_ > 0;
  }

  std::size_t pointCount() const
  {
    return point_count_;
  }

  GroundZProjection project(
      const double base_x,
      const double base_y,
      const double current_base_z) const
  {
    GroundZProjection result;
    if (!ready() ||
        !std::isfinite(base_x) ||
        !std::isfinite(base_y) ||
        !std::isfinite(current_base_z))
    {
      return result;
    }

    const double radius = config_.xy_tolerance;
    const std::int64_t min_bucket_x =
        bucketCoordinate(base_x - radius);
    const std::int64_t max_bucket_x =
        bucketCoordinate(base_x + radius);
    const std::int64_t min_bucket_y =
        bucketCoordinate(base_y - radius);
    const std::int64_t max_bucket_y =
        bucketCoordinate(base_y + radius);

    constexpr double kTieTolerance = 1e-9;
    for (std::int64_t bucket_x = min_bucket_x;
         bucket_x <= max_bucket_x;
         ++bucket_x)
    {
      for (std::int64_t bucket_y = min_bucket_y;
           bucket_y <= max_bucket_y;
           ++bucket_y)
      {
        const auto found = buckets_.find({bucket_x, bucket_y});
        if (found == buckets_.end())
          continue;

        for (const auto &point : found->second)
        {
          const double xy_distance =
              std::hypot(point.x - base_x, point.y - base_y);
          const double vertical_distance =
              std::abs(point.z - current_base_z);
          if (
              xy_distance > config_.xy_tolerance ||
              vertical_distance > config_.maximum_layer_distance)
          {
            continue;
          }

          const bool closer_xy =
              !result.valid ||
              xy_distance < result.xy_distance - kTieTolerance;
          const bool same_xy_closer_layer =
              result.valid &&
              std::abs(xy_distance - result.xy_distance) <= kTieTolerance &&
              vertical_distance < result.vertical_distance;
          if (!closer_xy && !same_xy_closer_layer)
            continue;

          result.valid = true;
          result.target_base_z = point.z;
          result.xy_distance = xy_distance;
          result.vertical_distance = vertical_distance;
        }
      }
    }
    return result;
  }

  double update(
      const double base_x,
      const double base_y,
      const double current_base_z,
      const double dt_seconds) const
  {
    if (
        !std::isfinite(current_base_z) ||
        !std::isfinite(dt_seconds) ||
        dt_seconds <= 0.0)
    {
      return current_base_z;
    }

    const GroundZProjection projection =
        project(base_x, base_y, current_base_z);
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
  struct BucketKey
  {
    std::int64_t x{0};
    std::int64_t y{0};

    bool operator==(const BucketKey &other) const
    {
      return x == other.x && y == other.y;
    }
  };

  struct BucketKeyHash
  {
    std::size_t operator()(const BucketKey &key) const
    {
      const std::size_t x_hash = std::hash<std::int64_t>{}(key.x);
      const std::size_t y_hash = std::hash<std::int64_t>{}(key.y);
      return x_hash ^ (
          y_hash + static_cast<std::size_t>(0x9e3779b9U) +
          (x_hash << 6U) + (x_hash >> 2U));
    }
  };

  static bool isFinite(const GroundSurfacePoint &point)
  {
    return
        std::isfinite(point.x) &&
        std::isfinite(point.y) &&
        std::isfinite(point.z);
  }

  static GroundZTrackerConfig sanitizeConfig(
      const GroundZTrackerConfig &config)
  {
    GroundZTrackerConfig result = config;
    if (!std::isfinite(result.bucket_size) || result.bucket_size <= 0.0)
      result.bucket_size = 0.20;
    if (!std::isfinite(result.xy_tolerance) || result.xy_tolerance <= 0.0)
      result.xy_tolerance = 0.15;
    if (
        !std::isfinite(result.maximum_layer_distance) ||
        result.maximum_layer_distance <= 0.0)
    {
      result.maximum_layer_distance = 0.50;
    }
    if (!std::isfinite(result.max_z_rate) || result.max_z_rate <= 0.0)
      result.max_z_rate = 0.30;
    return result;
  }

  std::int64_t bucketCoordinate(const double coordinate) const
  {
    return static_cast<std::int64_t>(
        std::floor(coordinate / config_.bucket_size));
  }

  BucketKey keyFor(const double x, const double y) const
  {
    return {bucketCoordinate(x), bucketCoordinate(y)};
  }

  GroundZTrackerConfig config_;
  std::unordered_map<
      BucketKey,
      std::vector<GroundSurfacePoint>,
      BucketKeyHash> buckets_;
  std::size_t point_count_{0};
};

}  // namespace scan_planner

#endif  // SCAN_PLANNER__GROUND_Z_TRACKER_H_
