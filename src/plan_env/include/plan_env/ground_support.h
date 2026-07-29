#ifndef PLAN_ENV_GROUND_SUPPORT_H
#define PLAN_ENV_GROUND_SUPPORT_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace plan_env
{

struct GroundSupportPoint
{
  float x;
  float y;
  float z;
};

struct GroundSupportConfig
{
  double bucket_size = 0.15;
  // Includes a 5 mm voxel-centroid/numeric boundary allowance. This remains
  // a nearest-sample match, not permission to miss a centre or inner probe.
  double xy_tolerance = 0.155;
  double z_tolerance = 0.20;
  double planning_height = 0.32;
  double circle_radius = 0.27;
  double circle_offset = 0.205;
  double circle_center_offset = -0.425;
  // Probe outside the physical circle by one point-matching tolerance.  A
  // ground sample at the real footprint boundary can still satisfy the probe,
  // while a footprint hanging over an edge cannot.
  double footprint_probe_margin = 0.14;
  int perimeter_samples = 16;
  int radial_samples = 2;
  // The outermost ring tolerates a limited number of point-cloud sampling
  // holes. The centre and every inner ring remain fail-closed, so broad
  // unsupported areas are still rejected.
  int outer_ring_max_missing_per_circle = 3;
};

class GroundSupportIndex
{
public:
  explicit GroundSupportIndex(const GroundSupportConfig & config = GroundSupportConfig())
  : config_(sanitize(config))
  {
  }

  void configure(const GroundSupportConfig & config)
  {
    config_ = sanitize(config);
    clear();
  }

  void clear()
  {
    buckets_.clear();
    point_count_ = 0;
  }

  void reserve(std::size_t point_count)
  {
    buckets_.reserve(std::max<std::size_t>(1, point_count / 2));
  }

  void addPoint(double x, double y, double z)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      return;

    buckets_[bucketKey(x, y)].push_back(
        GroundSupportPoint{
          static_cast<float>(x),
          static_cast<float>(y),
          static_cast<float>(z)});
    ++point_count_;
  }

  bool empty() const
  {
    return point_count_ == 0;
  }

  std::size_t size() const
  {
    return point_count_;
  }

  const GroundSupportConfig & config() const
  {
    return config_;
  }

  bool hasSupport(double x, double y, double ground_z) const
  {
    if (empty() || !std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(ground_z))
      return false;

    const int center_x = bucketCoordinate(x);
    const int center_y = bucketCoordinate(y);
    const int bucket_radius = std::max(
        1,
        static_cast<int>(
          std::ceil(config_.xy_tolerance / config_.bucket_size)));
    const double xy_tolerance_squared =
        config_.xy_tolerance * config_.xy_tolerance;

    for (int dx = -bucket_radius; dx <= bucket_radius; ++dx)
    {
      for (int dy = -bucket_radius; dy <= bucket_radius; ++dy)
      {
        const auto it = buckets_.find(packKey(center_x + dx, center_y + dy));
        if (it == buckets_.end())
          continue;

        for (const GroundSupportPoint & point : it->second)
        {
          if (std::abs(static_cast<double>(point.z) - ground_z) >
              config_.z_tolerance)
            continue;
          const double point_dx = static_cast<double>(point.x) - x;
          const double point_dy = static_cast<double>(point.y) - y;
          if (point_dx * point_dx + point_dy * point_dy <=
              xy_tolerance_squared)
            return true;
        }
      }
    }
    return false;
  }

  bool isPoseSupported(
      double x, double y, double planning_z, double yaw) const
  {
    if (empty() || !std::isfinite(yaw) || !std::isfinite(planning_z))
      return false;

    const double ground_z = planning_z - config_.planning_height;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double center_x =
        x + config_.circle_center_offset * cos_yaw;
    const double center_y =
        y + config_.circle_center_offset * sin_yaw;
    const double offsets[2] = {
      config_.circle_offset,
      -config_.circle_offset};

    constexpr double kTwoPi = 6.283185307179586476925286766559;
    const double probe_radius =
        config_.circle_radius + config_.footprint_probe_margin;
    for (double offset : offsets)
    {
      const double circle_x = center_x + offset * cos_yaw;
      const double circle_y = center_y + offset * sin_yaw;
      if (!hasSupport(circle_x, circle_y, ground_z))
        return false;

      if (probe_radius <= 1e-9)
        continue;
      for (int ring = 1; ring <= config_.radial_samples; ++ring)
      {
        const double ring_radius =
            probe_radius * static_cast<double>(ring) /
            static_cast<double>(config_.radial_samples);
        const int ring_samples =
            ring == config_.radial_samples
              ? config_.perimeter_samples
              : std::max(4, config_.perimeter_samples / 2);
        int outer_ring_missing = 0;
        for (int sample = 0; sample < ring_samples; ++sample)
        {
          const double angle =
              kTwoPi * static_cast<double>(sample) /
              static_cast<double>(ring_samples);
          const bool supported = hasSupport(
              circle_x + ring_radius * std::cos(angle),
              circle_y + ring_radius * std::sin(angle),
              ground_z);
          if (ring != config_.radial_samples)
          {
            if (!supported)
              return false;
            continue;
          }
          if (!supported &&
              ++outer_ring_missing >
              config_.outer_ring_max_missing_per_circle)
            return false;
        }
      }
    }
    return true;
  }

private:
  static GroundSupportConfig sanitize(GroundSupportConfig config)
  {
    config.bucket_size = std::max(0.02, config.bucket_size);
    config.xy_tolerance = std::max(0.01, config.xy_tolerance);
    config.z_tolerance = std::max(0.01, config.z_tolerance);
    config.planning_height = std::max(0.0, config.planning_height);
    config.circle_radius = std::max(0.0, config.circle_radius);
    config.circle_offset = std::max(0.0, config.circle_offset);
    config.footprint_probe_margin =
        std::max(0.0, config.footprint_probe_margin);
    config.perimeter_samples = std::max(4, config.perimeter_samples);
    config.radial_samples = std::max(1, config.radial_samples);
    config.outer_ring_max_missing_per_circle = std::min(
        config.perimeter_samples,
        std::max(0, config.outer_ring_max_missing_per_circle));
    return config;
  }

  int bucketCoordinate(double coordinate) const
  {
    return static_cast<int>(std::floor(coordinate / config_.bucket_size));
  }

  static std::uint64_t packKey(int x, int y)
  {
    return
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
      static_cast<std::uint32_t>(y);
  }

  std::uint64_t bucketKey(double x, double y) const
  {
    return packKey(bucketCoordinate(x), bucketCoordinate(y));
  }

  GroundSupportConfig config_;
  std::unordered_map<
    std::uint64_t,
    std::vector<GroundSupportPoint>> buckets_;
  std::size_t point_count_ = 0;
};

}  // namespace plan_env

#endif  // PLAN_ENV_GROUND_SUPPORT_H
