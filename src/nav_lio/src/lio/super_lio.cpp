
#include "lio/super_lio.h"

#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <Eigen/SparseCholesky>

using namespace BASIC;

namespace LI2Sup{

inline bool save_pcd_binary_safe(const std::string& filename, const PointCloudType& cloud)
{
  const auto temporary_filename = filename + ".tmp";
  try {
    const auto parent = std::filesystem::path(filename).parent_path();
    if (!parent.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(parent, ec);
      if (ec) {
        LOG(ERROR) << RED << " ---> 创建地图目录失败: "
                   << parent.string() << " error: " << ec.message() << RESET;
        return false;
      }
    }

    std::error_code ec;
    std::filesystem::remove(temporary_filename, ec);
    ec.clear();
    if (pcl::io::savePCDFileBinary(temporary_filename, cloud) != 0) {
      LOG(ERROR) << RED << " ---> 保存 PCD 临时文件失败: " << temporary_filename << RESET;
      return false;
    }

    std::filesystem::rename(temporary_filename, filename, ec);
    if (ec) {
      std::filesystem::remove(temporary_filename);
      LOG(ERROR) << RED << " ---> 提交 PCD 文件失败: "
                 << filename << " error: " << ec.message() << RESET;
      return false;
    }
    return true;
  } catch (const std::exception& e) {
    std::error_code ec;
    std::filesystem::remove(temporary_filename, ec);
    LOG(ERROR) << RED << " ---> 保存 PCD 文件失败: "
               << filename << " error: " << e.what() << RESET;
    return false;
  }
}

inline void normalize_cloud_layout(PointCloudType& cloud)
{
  if (!cloud.empty()) {
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = false;
  }
}

inline void make_map_pcd_cloud(
  const PointCloudType::ConstPtr& source,
  PointCloudType& output,
  const float leaf_size,
  const char* context,
  const bool adapt_leaf_on_overflow = false)
{
  output.clear();
  if (!source || source->empty()) {
    return;
  }

  if (!g_if_filter) {
    output = *source;
    normalize_cloud_layout(output);
    return;
  }

  if (leaf_size <= 0.0f) {
    LOG(WARNING) << YELLOW << " ---> " << context
                 << " map downsample leaf size invalid, use accumulated map directly." << RESET;
    output = *source;
    normalize_cloud_layout(output);
    return;
  }

  bool has_finite_point = false;
  float min_x = 0.0f, min_y = 0.0f, min_z = 0.0f;
  float max_x = 0.0f, max_y = 0.0f, max_z = 0.0f;
  for (const auto& point : source->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    if (!has_finite_point) {
      min_x = max_x = point.x;
      min_y = max_y = point.y;
      min_z = max_z = point.z;
      has_finite_point = true;
      continue;
    }
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    min_z = std::min(min_z, point.z);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    max_z = std::max(max_z, point.z);
  }

  if (!has_finite_point) {
    return;
  }

  const auto voxel_count_x =
    static_cast<long double>(std::floor((max_x - min_x) / leaf_size) + 1.0);
  const auto voxel_count_y =
    static_cast<long double>(std::floor((max_y - min_y) / leaf_size) + 1.0);
  const auto voxel_count_z =
    static_cast<long double>(std::floor((max_z - min_z) / leaf_size) + 1.0);
  const auto max_index = static_cast<long double>(std::numeric_limits<int>::max());
  auto effective_leaf_size = leaf_size;

  if (voxel_count_x * voxel_count_y * voxel_count_z > max_index) {
    if (adapt_leaf_on_overflow) {
      const auto range_x = static_cast<long double>(std::max(max_x - min_x, leaf_size));
      const auto range_y = static_cast<long double>(std::max(max_y - min_y, leaf_size));
      const auto range_z = static_cast<long double>(std::max(max_z - min_z, leaf_size));
      const auto safe_leaf = std::cbrt((range_x * range_y * range_z) / max_index) * 1.1L;
      effective_leaf_size =
        static_cast<float>(std::max(static_cast<long double>(leaf_size), safe_leaf));
      LOG(WARNING) << YELLOW << " ---> " << context
                   << " map preview leaf size increased to avoid voxel index overflow."
                   << " requested_leaf_size: " << leaf_size
                   << " effective_leaf_size: " << effective_leaf_size
                   << " map_size: " << source->size() << RESET;
    } else {
      // PCL VoxelGrid encodes a whole cloud in a signed 32-bit linear voxel
      // index.  A long but sparse route can overflow that product even though
      // every local area is small.  Split on boundaries aligned to the voxel
      // grid and filter each non-empty tile independently; this preserves the
      // requested leaf size instead of silently returning the unfiltered map.
      constexpr std::int64_t kVoxelsPerTile = 512;
      const double tile_extent =
          static_cast<double>(leaf_size) * kVoxelsPerTile;
      using TileKey = std::array<std::int64_t, 3>;
      std::map<TileKey, CloudPtr> tiles;
      for (const auto& point : source->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
          continue;
        }
        const TileKey key{
            static_cast<std::int64_t>(std::floor(point.x / tile_extent)),
            static_cast<std::int64_t>(std::floor(point.y / tile_extent)),
            static_cast<std::int64_t>(std::floor(point.z / tile_extent))};
        auto& tile = tiles[key];
        if (!tile) {
          tile.reset(new PointCloudType());
        }
        tile->push_back(point);
      }

      output.reserve(source->size());
      pcl::VoxelGrid<PointType> tiled_filter;
      std::size_t filtered_tiles = 0;
      for (auto& entry : tiles) {
        auto& tile = entry.second;
        if (!tile || tile->empty()) {
          continue;
        }
        normalize_cloud_layout(*tile);
        PointCloudType filtered_tile;
        tiled_filter.setInputCloud(tile);
        tiled_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        tiled_filter.filter(filtered_tile);
        output += filtered_tile;
        ++filtered_tiles;
      }
      output.header = source->header;
      normalize_cloud_layout(output);
      LOG(INFO) << GREEN << " ---> " << context
                << " map downsampled in aligned tiles to avoid voxel index overflow."
                << " leaf_size: " << leaf_size
                << " tiles: " << filtered_tiles
                << " input_points: " << source->size()
                << " output_points: " << output.size() << RESET;
      return;
    }
  }

  pcl::VoxelGrid<PointType> voxel_filter;
  voxel_filter.setInputCloud(source);
  voxel_filter.setLeafSize(effective_leaf_size, effective_leaf_size, effective_leaf_size);
  voxel_filter.filter(output);
  normalize_cloud_layout(output);
}

inline bool merge_cloud_incrementally(
    CloudPtr& accumulated,
    const CloudPtr& addition,
    const float leaf_size,
    const char* context)
{
  if (!addition || addition->empty()) {
    return true;
  }

  CloudPtr merged(new PointCloudType());
  const std::size_t accumulated_size =
      accumulated ? accumulated->size() : 0;
  merged->reserve(accumulated_size + addition->size());
  if (accumulated && !accumulated->empty()) {
    *merged += *accumulated;
  }
  *merged += *addition;

  CloudPtr filtered(new PointCloudType());
  make_map_pcd_cloud(merged, *filtered, leaf_size, context);
  if (filtered->empty() && !merged->empty()) {
    LOG(ERROR) << RED << " ---> " << context
               << " incremental downsample produced an empty cloud." << RESET;
    return false;
  }
  accumulated = filtered;
  return true;
}

inline bool copy_file_atomic(
    const std::filesystem::path& source,
    const std::filesystem::path& destination)
{
  const auto temporary = destination.string() + ".tmp";
  std::error_code ec;
  std::filesystem::remove(temporary, ec);
  ec.clear();
  std::filesystem::copy_file(
      source,
      temporary,
      std::filesystem::copy_options::overwrite_existing,
      ec);
  if (ec) {
    LOG(ERROR) << RED << " ---> 原子复制地图临时文件失败: "
               << source.string() << " -> " << temporary
               << " error: " << ec.message() << RESET;
    return false;
  }
  std::filesystem::rename(temporary, destination, ec);
  if (ec) {
    std::filesystem::remove(temporary);
    LOG(ERROR) << RED << " ---> 原子提交地图副本失败: "
               << destination.string()
               << " error: " << ec.message() << RESET;
    return false;
  }
  return true;
}

inline Eigen::Matrix4f se3_to_matrix4f(const SE3& pose)
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) = pose.R_.cast<float>();
  matrix.block<3, 1>(0, 3) = pose.t_.cast<float>();
  return matrix;
}

inline SE3 project_gravity_aligned_loop_correction(
    const Eigen::Matrix4f& full_correction,
    const V3& raw_later_position)
{
  // LiDAR-inertial odometry already observes roll and pitch from gravity.
  // Applying a small GICP roll/pitch correction about the world origin can
  // change Z by metres at a distant ramp and flatten real terrain.  Keep the
  // full GICP transform for acceptance validation, then project the graph
  // measurement to the standard gravity-aligned 4-DoF form: XYZ + yaw.
  //
  // Translation is recomputed about the later keyframe origin, so the
  // projected transform sends that origin to exactly the same XYZ selected
  // by full 6-DoF GICP.  Z remains fully optimized; only correction roll and
  // pitch are removed.
  const Eigen::Matrix3f full_rotation =
      full_correction.block<3, 3>(0, 0);
  const float yaw =
      std::atan2(full_rotation(1, 0), full_rotation(0, 0));
  const Eigen::Matrix3f yaw_rotation =
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3f anchor =
      raw_later_position.cast<float>();
  const Eigen::Vector3f corrected_anchor =
      full_rotation * anchor +
      full_correction.block<3, 1>(0, 3);

  Eigen::Matrix4f projected =
      Eigen::Matrix4f::Identity();
  projected.block<3, 3>(0, 0) = yaw_rotation;
  projected.block<3, 1>(0, 3) =
      corrected_anchor - yaw_rotation * anchor;
  return SE3(projected);
}

struct LoopRotationMetrics
{
  float yaw_deg = std::numeric_limits<float>::max();
  float tilt_deg = std::numeric_limits<float>::max();
  float total_deg = std::numeric_limits<float>::max();
};

inline LoopRotationMetrics evaluate_loop_rotation(
    const Eigen::Matrix3f& rotation)
{
  LoopRotationMetrics result;
  if (!rotation.allFinite()) {
    return result;
  }
  const float yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const Eigen::Matrix3f yaw_rotation =
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
          .toRotationMatrix();
  const Eigen::AngleAxisf tilt(yaw_rotation.transpose() * rotation);
  const Eigen::AngleAxisf total(rotation);
  const float radians_to_degrees =
      180.0f / static_cast<float>(M_PI);
  result.yaw_deg = std::abs(yaw) * radians_to_degrees;
  result.tilt_deg = std::abs(tilt.angle()) * radians_to_degrees;
  result.total_deg = std::abs(total.angle()) * radians_to_degrees;
  return result;
}

inline float adaptive_loop_yaw_limit_deg(const float path_length)
{
  // The LIO yaw uncertainty grows with travelled distance.  Keep the legacy
  // value as the short-route allowance and grow continuously under the same
  // drift model used by graph consistency, with a hard safety cap so a
  // repetitive corridor can never authorize an arbitrary rotation.
  return std::min(
      g_loop_max_adaptive_yaw_deg,
      g_loop_max_correction_rotation_deg +
          g_loop_rotation_drift_deg_per_m *
              std::max(0.0f, path_length));
}

inline bool loop_rotation_is_plausible(
    const LoopRotationMetrics& rotation,
    const float yaw_limit_deg)
{
  return rotation.yaw_deg <= yaw_limit_deg &&
      rotation.tilt_deg <= g_loop_max_correction_tilt_deg;
}

inline float robust_median(std::vector<float> values)
{
  if (values.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(
      values.begin(), values.begin() + middle, values.end());
  const float upper = values[middle];
  if ((values.size() & 1U) != 0U) {
    return upper;
  }
  const float lower = *std::max_element(
      values.begin(), values.begin() + middle);
  return 0.5f * (lower + upper);
}

inline CloudPtr extract_loop_ground_envelope(
    const PointCloudType::ConstPtr& cloud,
    const V3& sensor_position)
{
  CloudPtr ground(new PointCloudType());
  if (!cloud || cloud->empty()) {
    return ground;
  }

  // Only build evidence from the neighbourhood actually revisited by the
  // local windows.  The vertical band is relative to the sensor, not world Z,
  // so it follows a ramp and cannot flatten the complete trajectory.  It also
  // prevents a lower floor visible through an opening from winning the local
  // minimum in a multi-floor map.
  const float sample_radius = std::clamp(
      g_loop_keyframe_min_distance *
          static_cast<float>(g_loop_local_window_size + 1),
      3.0f, 8.0f);
  const float max_above_sensor = std::max(
      0.75f, 0.5f * g_loop_icp_max_distance);
  const float max_below_sensor = std::max(
      3.0f, 1.5f * g_loop_icp_max_distance);
  const float radius_squared = sample_radius * sample_radius;
  const float inverse_cell_size = 1.0f / g_loop_ground_z_cell_size;

  // A vertical wall contributes many more returns than the floor. Retaining
  // only the lowest return in each horizontal cell turns the cloud into a
  // local support envelope before estimating a vertical residual.
  std::map<std::pair<int, int>, PointType> lowest_by_cell;
  for (const auto& point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const float dx = point.x - static_cast<float>(sensor_position.x());
    const float dy = point.y - static_cast<float>(sensor_position.y());
    if (dx * dx + dy * dy > radius_squared) {
      continue;
    }
    const float relative_z =
        point.z - static_cast<float>(sensor_position.z());
    if (relative_z > max_above_sensor ||
        relative_z < -max_below_sensor) {
      continue;
    }
    const int cell_x = static_cast<int>(
        std::floor(point.x * inverse_cell_size));
    const int cell_y = static_cast<int>(
        std::floor(point.y * inverse_cell_size));
    const auto key = std::make_pair(cell_x, cell_y);
    const auto existing = lowest_by_cell.find(key);
    if (existing == lowest_by_cell.end() ||
        point.z < existing->second.z) {
      lowest_by_cell[key] = point;
    }
  }

  ground->reserve(lowest_by_cell.size());
  for (const auto& [cell, point] : lowest_by_cell) {
    (void)cell;
    ground->push_back(point);
  }
  normalize_cloud_layout(*ground);
  return ground;
}

struct GroundZRefinement
{
  bool valid = false;
  int source_points = 0;
  int target_points = 0;
  int pair_count = 0;
  int inlier_count = 0;
  float inlier_ratio = 0.0f;
  float z_adjustment = 0.0f;
  float residual_mad = std::numeric_limits<float>::max();
};

// Independent evidence for a correction-field Z measurement.  Unlike
// GroundZRefinement, this estimator never receives an ICP transform: source
// and target are paired at their raw world XY coordinates and only a scalar
// Z offset is estimated.  This avoids circularly "validating" a height that
// was already injected by an anchor-coincident GICP seed, and it also avoids
// turning a discarded planar translation on a ramp into a false Z offset.
struct GroundZOnlyEvidence
{
  bool valid = false;
  // A narrow observation is never allowed to create an edge by itself.  It
  // may only contribute to a spatially distributed group that also contains
  // a full-footprint observation.  Keeping the two qualities separate lets
  // corridors accumulate evidence without weakening the single-anchor gate.
  bool distributed_valid = false;
  // A wide footprint is judged by spatially separated block medians rather
  // than by centimetre-level point roughness.  It is consumed only by the
  // single-wide pure-Z path and never relaxes a local anchor.
  bool wide_valid = false;
  int source_points = 0;
  int target_points = 0;
  int source_to_target_pairs = 0;
  int target_to_source_pairs = 0;
  int pair_count = 0;
  int inlier_count = 0;
  int supported_blocks = 0;
  float inlier_ratio = 0.0f;
  float z_adjustment = 0.0f;
  float residual_mad = std::numeric_limits<float>::max();
  float bidirectional_difference =
      std::numeric_limits<float>::max();
  float block_residual_mad = std::numeric_limits<float>::max();
  float support_span = 0.0f;
  float support_minor_span = 0.0f;
  float support_center_x = 0.0f;
  float support_center_y = 0.0f;
  float pair_xy_distance_p90 =
      std::numeric_limits<float>::max();
  float slope_height_ambiguity =
      std::numeric_limits<float>::max();
  float maximum_slope_height_ambiguity = 0.0f;
  int independent_regions = 0;
  float independent_region_separation = 0.0f;
  float independent_region_z_difference =
      std::numeric_limits<float>::max();
  float source_ground_slope_deg =
      std::numeric_limits<float>::max();
  float target_ground_slope_deg =
      std::numeric_limits<float>::max();
};

inline float estimate_ground_slope_deg(
    const PointCloudType::ConstPtr& ground)
{
  if (!ground || ground->size() < 30) {
    return std::numeric_limits<float>::max();
  }
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  int finite_points = 0;
  for (const auto& point : ground->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    centroid += Eigen::Vector3f(point.x, point.y, point.z);
    ++finite_points;
  }
  if (finite_points < 30) {
    return std::numeric_limits<float>::max();
  }
  centroid /= static_cast<float>(finite_points);
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  for (const auto& point : ground->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const Eigen::Vector3f centered(
        point.x - centroid.x(), point.y - centroid.y(),
        point.z - centroid.z());
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<float>(finite_points);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  if (solver.info() != Eigen::Success ||
      !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite() ||
      solver.eigenvalues()(1) <= 1.0e-4f) {
    return std::numeric_limits<float>::max();
  }
  const Eigen::Vector3f normal = solver.eigenvectors().col(0).normalized();
  const float cosine = std::clamp(std::abs(normal.z()), 0.0f, 1.0f);
  return std::acos(cosine) * 180.0f / static_cast<float>(M_PI);
}

inline GroundZOnlyEvidence estimate_ground_z_only(
    const PointCloudType::ConstPtr& source_ground,
    const PointCloudType::ConstPtr& target_ground)
{
  GroundZOnlyEvidence result;
  result.source_points = source_ground
      ? static_cast<int>(source_ground->size()) : 0;
  result.target_points = target_ground
      ? static_cast<int>(target_ground->size()) : 0;
  if (!g_loop_ground_z_refinement_enable ||
      !source_ground || !target_ground ||
      source_ground->empty() || target_ground->empty()) {
    return result;
  }
  result.source_ground_slope_deg =
      estimate_ground_slope_deg(source_ground);
  result.target_ground_slope_deg =
      estimate_ground_slope_deg(target_ground);
  // Do not use a fixed "flat ground" angle here.  A 2--6 degree ramp is a
  // perfectly valid height reference when the raw-world XY pairs are close.
  // The actual ambiguity is slope * pairing distance and is checked after
  // correspondence outliers have been removed.  A gross cap only prevents a
  // wall/curb envelope from being treated as ground.
  constexpr float kMaximumZOnlyGroundSlopeDeg = 15.0f;
  if (!std::isfinite(result.source_ground_slope_deg) ||
      !std::isfinite(result.target_ground_slope_deg) ||
      result.source_ground_slope_deg > kMaximumZOnlyGroundSlopeDeg ||
      result.target_ground_slope_deg > kMaximumZOnlyGroundSlopeDeg) {
    return result;
  }

  CloudPtr flat_source(new PointCloudType(*source_ground));
  CloudPtr flat_target(new PointCloudType(*target_ground));
  for (auto& point : flat_source->points) {
    point.z = 0.0f;
  }
  for (auto& point : flat_target->points) {
    point.z = 0.0f;
  }
  pcl::search::KdTree<PointType> source_tree;
  pcl::search::KdTree<PointType> target_tree;
  source_tree.setInputCloud(flat_source);
  target_tree.setInputCloud(flat_target);

  struct ResidualSample
  {
    float x = 0.0f;
    float y = 0.0f;
    float residual = 0.0f;
    float xy_distance = 0.0f;
  };
  std::vector<ResidualSample> source_to_target;
  std::vector<ResidualSample> target_to_source;
  const float maximum_xy_squared =
      g_loop_ground_z_pair_xy_distance *
      g_loop_ground_z_pair_xy_distance;
  // A proactive edge must remain local.  The value is derived from the ICP
  // observation radius rather than from a world height: a larger accumulated
  // error needs intermediate anchors instead of one ambiguous floor jump.
  const float maximum_abs_z = std::min(
      3.0f, std::max(0.30f, 1.5f * g_loop_icp_max_distance));
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);

  const auto collect_direction = [&] (
      const PointCloudType::ConstPtr& query_ground,
      pcl::search::KdTree<PointType>& reference_tree,
      const PointCloudType::ConstPtr& reference_ground,
      const bool query_is_source,
      std::vector<ResidualSample>& samples) {
    samples.reserve(query_ground->size());
    for (const auto& query_point : query_ground->points) {
      PointType query = query_point;
      query.z = 0.0f;
      if (reference_tree.nearestKSearch(
              query, 1, nearest_index,
              nearest_squared_distance) <= 0 ||
          nearest_squared_distance[0] > maximum_xy_squared) {
        continue;
      }
      const auto& reference_point =
          reference_ground->points[
              static_cast<std::size_t>(nearest_index[0])];
      const float residual = query_is_source
          ? reference_point.z - query_point.z
          : query_point.z - reference_point.z;
      if (!std::isfinite(residual) ||
          std::abs(residual) > maximum_abs_z) {
        continue;
      }
      ResidualSample sample;
      sample.x = 0.5f * (query_point.x + reference_point.x);
      sample.y = 0.5f * (query_point.y + reference_point.y);
      sample.residual = residual;
      sample.xy_distance = std::sqrt(nearest_squared_distance[0]);
      samples.push_back(sample);
    }
  };
  collect_direction(
      source_ground, target_tree, target_ground, true,
      source_to_target);
  collect_direction(
      target_ground, source_tree, source_ground, false,
      target_to_source);
  result.source_to_target_pairs =
      static_cast<int>(source_to_target.size());
  result.target_to_source_pairs =
      static_cast<int>(target_to_source.size());
  result.pair_count = result.source_to_target_pairs +
      result.target_to_source_pairs;
  const int minimum_direction_pairs = std::max(
      30, g_loop_ground_z_min_pairs / 2);
  if (result.source_to_target_pairs < minimum_direction_pairs ||
      result.target_to_source_pairs < minimum_direction_pairs ||
      result.pair_count < g_loop_ground_z_min_pairs) {
    return result;
  }

  std::vector<float> forward_residuals;
  std::vector<float> reverse_residuals;
  forward_residuals.reserve(source_to_target.size());
  reverse_residuals.reserve(target_to_source.size());
  for (const auto& sample : source_to_target) {
    forward_residuals.push_back(sample.residual);
  }
  for (const auto& sample : target_to_source) {
    reverse_residuals.push_back(sample.residual);
  }
  const float forward_median = robust_median(forward_residuals);
  const float reverse_median = robust_median(reverse_residuals);
  result.bidirectional_difference =
      std::abs(forward_median - reverse_median);
  constexpr float kMaximumBidirectionalDifference = 0.05f;
  if (!std::isfinite(forward_median) ||
      !std::isfinite(reverse_median) ||
      result.bidirectional_difference >
          kMaximumBidirectionalDifference) {
    return result;
  }

  std::vector<ResidualSample> samples;
  samples.reserve(static_cast<std::size_t>(result.pair_count));
  samples.insert(
      samples.end(), source_to_target.begin(), source_to_target.end());
  samples.insert(
      samples.end(), target_to_source.begin(), target_to_source.end());
  std::vector<float> residuals;
  residuals.reserve(samples.size());
  for (const auto& sample : samples) {
    residuals.push_back(sample.residual);
  }
  const float initial_median = robust_median(residuals);
  std::vector<float> deviations;
  deviations.reserve(residuals.size());
  for (const float residual : residuals) {
    deviations.push_back(std::abs(residual - initial_median));
  }
  const float initial_mad = robust_median(deviations);
  const float inlier_threshold = std::max(
      0.03f, 3.0f * 1.4826f * initial_mad);

  std::vector<ResidualSample> inliers;
  std::vector<float> inlier_residuals;
  inliers.reserve(samples.size());
  inlier_residuals.reserve(samples.size());
  for (const auto& sample : samples) {
    if (std::abs(sample.residual - initial_median) <=
        inlier_threshold) {
      inliers.push_back(sample);
      inlier_residuals.push_back(sample.residual);
    }
  }
  result.inlier_count = static_cast<int>(inliers.size());
  result.inlier_ratio = static_cast<float>(result.inlier_count) /
      static_cast<float>(result.pair_count);
  constexpr float kMinimumZOnlyInlierRatio = 0.75f;
  if (result.inlier_count < g_loop_ground_z_min_pairs ||
      result.inlier_ratio < kMinimumZOnlyInlierRatio) {
    return result;
  }
  result.z_adjustment = robust_median(inlier_residuals);
  deviations.clear();
  deviations.reserve(inlier_residuals.size());
  for (const float residual : inlier_residuals) {
    deviations.push_back(std::abs(residual - result.z_adjustment));
  }
  result.residual_mad = robust_median(deviations);
  const float maximum_z_only_mad = std::min(
      0.03f, g_loop_ground_z_max_mad);
  const float maximum_distributed_z_only_mad = std::min(
      0.04f, g_loop_ground_z_max_mad);
  const float maximum_wide_z_only_mad = std::min(
      0.06f, std::max(0.04f, 2.0f * g_loop_ground_z_max_mad));
  if (!std::isfinite(result.z_adjustment) ||
      !std::isfinite(result.residual_mad) ||
      result.residual_mad > maximum_wide_z_only_mad ||
      std::abs(result.z_adjustment) < 0.30f ||
      std::abs(result.z_adjustment) > maximum_abs_z) {
    return result;
  }

  std::vector<float> inlier_xy_distances;
  inlier_xy_distances.reserve(inliers.size());
  for (const auto& sample : inliers) {
    inlier_xy_distances.push_back(sample.xy_distance);
  }
  std::sort(inlier_xy_distances.begin(), inlier_xy_distances.end());
  const std::size_t p90_index = std::min(
      inlier_xy_distances.size() - 1,
      static_cast<std::size_t>(
          std::floor(0.90 * static_cast<double>(
              inlier_xy_distances.size() - 1))));
  result.pair_xy_distance_p90 = inlier_xy_distances[p90_index];
  const float maximum_ground_slope_deg = std::max(
      result.source_ground_slope_deg,
      result.target_ground_slope_deg);
  result.slope_height_ambiguity =
      std::tan(
          maximum_ground_slope_deg *
          static_cast<float>(M_PI) / 180.0f) *
      result.pair_xy_distance_p90;
  result.maximum_slope_height_ambiguity = std::clamp(
      3.0f * 1.4826f * result.residual_mad, 0.03f, 0.08f);
  if (!std::isfinite(result.slope_height_ambiguity) ||
      result.slope_height_ambiguity >
          result.maximum_slope_height_ambiguity) {
    return result;
  }

  struct BlockAccumulator
  {
    std::vector<float> residuals;
  };
  std::map<std::pair<int, int>, BlockAccumulator> blocks;
  const float inverse_block_size =
      1.0f / g_loop_verification_block_size;
  for (const auto& sample : inliers) {
    const auto key = std::make_pair(
        static_cast<int>(std::floor(
            sample.x * inverse_block_size)),
        static_cast<int>(std::floor(
            sample.y * inverse_block_size)));
    blocks[key].residuals.push_back(sample.residual);
  }
  constexpr int kMinimumGroundPairsPerBlock = 5;
  std::vector<float> supported_block_medians;
  std::vector<Eigen::Vector2f,
              Eigen::aligned_allocator<Eigen::Vector2f>>
      supported_block_centers;
  float supported_center_x_sum = 0.0f;
  float supported_center_y_sum = 0.0f;
  const float block_residual_tolerance = std::max(
      0.05f, 3.0f * result.residual_mad);
  for (const auto& [key, block] : blocks) {
    if (static_cast<int>(block.residuals.size()) <
        kMinimumGroundPairsPerBlock) {
      continue;
    }
    const float block_median = robust_median(block.residuals);
    if (std::abs(block_median - result.z_adjustment) >
        block_residual_tolerance) {
      continue;
    }
    ++result.supported_blocks;
    supported_block_medians.push_back(block_median);
    const float center_x =
        (static_cast<float>(key.first) + 0.5f) *
        g_loop_verification_block_size;
    const float center_y =
        (static_cast<float>(key.second) + 0.5f) *
        g_loop_verification_block_size;
    supported_center_x_sum += center_x;
    supported_center_y_sum += center_y;
    supported_block_centers.emplace_back(center_x, center_y);
  }
  if (!supported_block_centers.empty()) {
    result.support_center_x = supported_center_x_sum /
        static_cast<float>(result.supported_blocks);
    result.support_center_y = supported_center_y_sum /
        static_cast<float>(result.supported_blocks);
    const Eigen::Vector2f center(
        result.support_center_x, result.support_center_y);
    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    for (const auto& block_center : supported_block_centers) {
      const Eigen::Vector2f delta = block_center - center;
      covariance.noalias() += delta * delta.transpose();
    }
    covariance /= static_cast<float>(supported_block_centers.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
    if (solver.info() == Eigen::Success &&
        solver.eigenvectors().allFinite()) {
      const Eigen::Vector2f minor_axis = solver.eigenvectors().col(0);
      const Eigen::Vector2f major_axis = solver.eigenvectors().col(1);
      float minimum_major = std::numeric_limits<float>::max();
      float maximum_major = std::numeric_limits<float>::lowest();
      float minimum_minor = std::numeric_limits<float>::max();
      float maximum_minor = std::numeric_limits<float>::lowest();
      for (const auto& block_center : supported_block_centers) {
        const Eigen::Vector2f delta = block_center - center;
        const float major = delta.dot(major_axis);
        const float minor = delta.dot(minor_axis);
        minimum_major = std::min(minimum_major, major);
        maximum_major = std::max(maximum_major, major);
        minimum_minor = std::min(minimum_minor, minor);
        maximum_minor = std::max(maximum_minor, minor);
      }
      result.support_span = maximum_major - minimum_major;
      result.support_minor_span = maximum_minor - minimum_minor;

      // A single wide observation may represent two independent pieces of
      // the same physical ground.  Split its supported blocks along the
      // principal direction and require both halves to estimate the same Z.
      // This is stronger than merely counting many neighbouring cells and is
      // used later only for a pure-Z edge (never for endpoint/XY/yaw votes).
      struct ProjectedGroundBlock
      {
        float major = 0.0f;
        float residual = 0.0f;
      };
      std::vector<ProjectedGroundBlock> projected_blocks;
      projected_blocks.reserve(supported_block_centers.size());
      for (std::size_t block = 0;
           block < supported_block_centers.size(); ++block) {
        projected_blocks.push_back({
            (supported_block_centers[block] - center).dot(major_axis),
            supported_block_medians[block]});
      }
      std::sort(
          projected_blocks.begin(), projected_blocks.end(),
          [](const ProjectedGroundBlock& lhs,
             const ProjectedGroundBlock& rhs) {
            return lhs.major < rhs.major;
          });
      const std::size_t split = projected_blocks.size() / 2;
      if (split >= 6 && projected_blocks.size() - split >= 6 &&
          result.support_span >= 8.0f &&
          result.support_minor_span >=
              2.0f * g_loop_verification_block_size) {
        std::vector<float> first_region_residuals;
        std::vector<float> second_region_residuals;
        first_region_residuals.reserve(split);
        second_region_residuals.reserve(
            projected_blocks.size() - split);
        float first_region_position = 0.0f;
        float second_region_position = 0.0f;
        for (std::size_t block = 0;
             block < projected_blocks.size(); ++block) {
          if (block < split) {
            first_region_residuals.push_back(
                projected_blocks[block].residual);
            first_region_position += projected_blocks[block].major;
          } else {
            second_region_residuals.push_back(
                projected_blocks[block].residual);
            second_region_position += projected_blocks[block].major;
          }
        }
        first_region_position /= static_cast<float>(split);
        second_region_position /= static_cast<float>(
            projected_blocks.size() - split);
        result.independent_region_separation =
            second_region_position - first_region_position;
        result.independent_region_z_difference = std::abs(
            robust_median(first_region_residuals) -
            robust_median(second_region_residuals));
        if (result.independent_region_separation >= 4.0f &&
            result.independent_region_z_difference <= 0.08f) {
          result.independent_regions = 2;
        }
      }
    }
    const float block_median_center =
        robust_median(supported_block_medians);
    std::vector<float> block_median_deviations;
    block_median_deviations.reserve(supported_block_medians.size());
    for (const float block_median : supported_block_medians) {
      block_median_deviations.push_back(
          std::abs(block_median - block_median_center));
    }
    result.block_residual_mad =
        robust_median(block_median_deviations);
  }
  constexpr float kMaximumZOnlyBlockMad = 0.08f;
  result.valid =
      result.residual_mad <= maximum_z_only_mad &&
      result.supported_blocks >= g_loop_min_verification_blocks &&
      result.support_span >= g_loop_min_verification_span &&
      result.support_minor_span >=
          2.0f * g_loop_verification_block_size &&
      result.block_residual_mad <= kMaximumZOnlyBlockMad;
  const float distributed_minimum_span = std::min(
      g_loop_min_verification_span,
      2.0f * g_loop_verification_block_size);
  result.distributed_valid =
      result.residual_mad <= maximum_distributed_z_only_mad &&
      result.supported_blocks >= g_loop_min_verification_blocks &&
      result.support_span >= distributed_minimum_span &&
      result.support_minor_span >= g_loop_verification_block_size &&
      result.block_residual_mad <= kMaximumZOnlyBlockMad;
  result.wide_valid =
      result.residual_mad <= maximum_wide_z_only_mad &&
      result.supported_blocks >=
          2 * g_loop_min_verification_blocks &&
      result.support_span >= 8.0f &&
      result.support_minor_span >=
          2.0f * g_loop_verification_block_size &&
      result.block_residual_mad <= kMaximumZOnlyBlockMad &&
      result.independent_regions >= 2;
  return result;
}

inline GroundZRefinement refine_loop_ground_z(
    const PointCloudType::ConstPtr& source_ground,
    const PointCloudType::ConstPtr& target_ground,
    Eigen::Matrix4f& correction)
{
  GroundZRefinement result;
  result.source_points = source_ground
      ? static_cast<int>(source_ground->size()) : 0;
  result.target_points = target_ground
      ? static_cast<int>(target_ground->size()) : 0;
  if (!g_loop_ground_z_refinement_enable ||
      !source_ground || !target_ground ||
      source_ground->empty() || target_ground->empty()) {
    return result;
  }

  CloudPtr aligned_source(new PointCloudType());
  pcl::transformPointCloud(
      *source_ground, *aligned_source, correction);

  // Search correspondences in XY only. Comparing Z in the nearest-neighbour
  // metric would let the very Z error being estimated select another layer.
  CloudPtr flat_target(new PointCloudType(*target_ground));
  for (auto& point : flat_target->points) {
    point.z = 0.0f;
  }
  pcl::search::KdTree<PointType> target_tree;
  target_tree.setInputCloud(flat_target);

  std::vector<float> residuals;
  residuals.reserve(aligned_source->size());
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);
  const float max_xy_squared =
      g_loop_ground_z_pair_xy_distance *
      g_loop_ground_z_pair_xy_distance;
  for (const auto& aligned_point : aligned_source->points) {
    PointType query = aligned_point;
    query.z = 0.0f;
    if (target_tree.nearestKSearch(
            query, 1, nearest_index,
            nearest_squared_distance) <= 0 ||
        nearest_squared_distance[0] > max_xy_squared) {
      continue;
    }
    const float residual =
        target_ground->points[nearest_index[0]].z - aligned_point.z;
    if (std::isfinite(residual) &&
        std::abs(residual) <= g_loop_icp_max_distance) {
      residuals.push_back(residual);
    }
  }

  result.pair_count = static_cast<int>(residuals.size());
  if (result.pair_count < g_loop_ground_z_min_pairs) {
    return result;
  }

  const float initial_median = robust_median(residuals);
  std::vector<float> deviations;
  deviations.reserve(residuals.size());
  for (const float residual : residuals) {
    deviations.push_back(std::abs(residual - initial_median));
  }
  const float initial_mad = robust_median(deviations);
  const float inlier_threshold = std::max(
      0.03f, 3.0f * 1.4826f * initial_mad);
  std::vector<float> inliers;
  inliers.reserve(residuals.size());
  for (const float residual : residuals) {
    if (std::abs(residual - initial_median) <= inlier_threshold) {
      inliers.push_back(residual);
    }
  }

  result.inlier_count = static_cast<int>(inliers.size());
  result.inlier_ratio = static_cast<float>(result.inlier_count) /
      static_cast<float>(result.pair_count);
  if (result.inlier_count < g_loop_ground_z_min_pairs ||
      result.inlier_ratio < g_loop_ground_z_min_inlier_ratio) {
    return result;
  }

  result.z_adjustment = robust_median(inliers);
  deviations.clear();
  for (const float residual : inliers) {
    deviations.push_back(std::abs(residual - result.z_adjustment));
  }
  result.residual_mad = robust_median(deviations);
  if (!std::isfinite(result.z_adjustment) ||
      !std::isfinite(result.residual_mad) ||
      result.residual_mad > g_loop_ground_z_max_mad ||
      std::abs(result.z_adjustment) >
          g_loop_ground_z_max_adjustment) {
    return result;
  }

  correction(2, 3) += result.z_adjustment;
  result.valid = true;
  return result;
}

struct DirectedLoopVerification
{
  int source_points = 0;
  int matched_points = 0;
  int valid_blocks = 0;
  int supported_blocks = 0;
  float overlap = 0.0f;
  float trimmed_rmse = std::numeric_limits<float>::max();
  float supported_block_ratio = 0.0f;
  float support_span = 0.0f;
  float support_minor_span = 0.0f;
};

struct LoopGeometryVerification
{
  bool valid = false;
  bool main_valid = false;
  bool structural_evidence_available = false;
  bool structural_valid = false;
  bool large_correction = false;
  float anchor_translation = 0.0f;
  float yaw_deg = 0.0f;
  float symmetric_overlap = 0.0f;
  float symmetric_trimmed_rmse = std::numeric_limits<float>::max();
  float structural_symmetric_overlap = 0.0f;
  float structural_symmetric_trimmed_rmse =
      std::numeric_limits<float>::max();
  float confidence = 0.0f;
  DirectedLoopVerification source_to_target;
  DirectedLoopVerification target_to_source;
  DirectedLoopVerification structural_source_to_target;
  DirectedLoopVerification structural_target_to_source;
};

struct VerificationBlockAccumulator
{
  int total = 0;
  int matched = 0;
  double squared_distance_sum = 0.0;
};

inline DirectedLoopVerification verify_loop_direction(
    const PointCloudType::ConstPtr& source,
    const PointCloudType::ConstPtr& target,
    const float minimum_block_overlap,
    const int minimum_block_points = 10)
{
  DirectedLoopVerification result;
  if (!source || !target || source->empty() || target->empty()) {
    return result;
  }

  pcl::search::KdTree<PointType> target_tree;
  target_tree.setInputCloud(target);
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);
  std::vector<float> matched_squared_distances;
  matched_squared_distances.reserve(source->size());
  std::map<std::pair<int, int>, VerificationBlockAccumulator> blocks;
  const float inverse_block_size =
      1.0f / g_loop_verification_block_size;
  const float maximum_squared_distance =
      g_loop_verification_max_distance *
      g_loop_verification_max_distance;

  for (const auto& point : source->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    ++result.source_points;
    const auto block_key = std::make_pair(
        static_cast<int>(std::floor(point.x * inverse_block_size)),
        static_cast<int>(std::floor(point.y * inverse_block_size)));
    auto& block = blocks[block_key];
    ++block.total;
    if (target_tree.nearestKSearch(
            point, 1, nearest_index,
            nearest_squared_distance) <= 0 ||
        nearest_squared_distance[0] > maximum_squared_distance) {
      continue;
    }
    ++result.matched_points;
    ++block.matched;
    block.squared_distance_sum += nearest_squared_distance[0];
    matched_squared_distances.push_back(nearest_squared_distance[0]);
  }

  if (result.source_points == 0 || matched_squared_distances.empty()) {
    return result;
  }
  result.overlap = static_cast<float>(result.matched_points) /
      static_cast<float>(result.source_points);

  // A few perfect wall/floor correspondences must not hide a broad residual
  // tail. Keep 80% of the closest admitted correspondences and report their
  // RMS separately from overlap, which already accounts for unmatched points.
  std::sort(
      matched_squared_distances.begin(),
      matched_squared_distances.end());
  const std::size_t retained = std::max<std::size_t>(
      1, static_cast<std::size_t>(
             std::floor(0.80 * matched_squared_distances.size())));
  const double retained_sum = std::accumulate(
      matched_squared_distances.begin(),
      matched_squared_distances.begin() +
          static_cast<std::ptrdiff_t>(retained),
      0.0);
  result.trimmed_rmse = static_cast<float>(
      std::sqrt(retained_sum / static_cast<double>(retained)));

  bool has_supported_block = false;
  float minimum_x = 0.0f;
  float maximum_x = 0.0f;
  float minimum_y = 0.0f;
  float maximum_y = 0.0f;
  for (const auto& [key, block] : blocks) {
    if (block.total < minimum_block_points) {
      continue;
    }
    ++result.valid_blocks;
    const float block_overlap = static_cast<float>(block.matched) /
        static_cast<float>(block.total);
    const float block_rmse = block.matched > 0
        ? static_cast<float>(std::sqrt(
              block.squared_distance_sum /
              static_cast<double>(block.matched)))
        : std::numeric_limits<float>::max();
    if (block_overlap < minimum_block_overlap ||
        block_rmse > g_loop_max_trimmed_rmse) {
      continue;
    }
    ++result.supported_blocks;
    const float center_x =
        (static_cast<float>(key.first) + 0.5f) *
        g_loop_verification_block_size;
    const float center_y =
        (static_cast<float>(key.second) + 0.5f) *
        g_loop_verification_block_size;
    if (!has_supported_block) {
      minimum_x = maximum_x = center_x;
      minimum_y = maximum_y = center_y;
      has_supported_block = true;
    } else {
      minimum_x = std::min(minimum_x, center_x);
      maximum_x = std::max(maximum_x, center_x);
      minimum_y = std::min(minimum_y, center_y);
      maximum_y = std::max(maximum_y, center_y);
    }
  }
  if (result.valid_blocks > 0) {
    result.supported_block_ratio =
        static_cast<float>(result.supported_blocks) /
        static_cast<float>(result.valid_blocks);
  }
  if (has_supported_block) {
    const float extent_x = maximum_x - minimum_x;
    const float extent_y = maximum_y - minimum_y;
    result.support_span = std::hypot(extent_x, extent_y);
    result.support_minor_span = std::min(extent_x, extent_y);
  }
  return result;
}

inline CloudPtr extract_loop_structural_points(
    const PointCloudType::ConstPtr& cloud)
{
  CloudPtr structure(new PointCloudType());
  if (!cloud || cloud->empty()) {
    return structure;
  }

  // Remove the local lower envelope independently in each XY cell. This is
  // slope-relative: a ramp remains the lower surface in every cell and is not
  // mistaken for a horizontal plane at a fixed world Z. What remains is wall,
  // pole, facade and ceiling evidence that can constrain planar translation.
  const float cell_size = std::max(
      0.4f, 0.5f * g_loop_verification_block_size);
  const float inverse_cell_size = 1.0f / cell_size;
  std::map<std::pair<int, int>, float> lowest_z;
  for (const auto& point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const auto key = std::make_pair(
        static_cast<int>(std::floor(point.x * inverse_cell_size)),
        static_cast<int>(std::floor(point.y * inverse_cell_size)));
    const auto existing = lowest_z.find(key);
    if (existing == lowest_z.end() || point.z < existing->second) {
      lowest_z[key] = point.z;
    }
  }

  structure->reserve(cloud->size() / 2);
  constexpr float kGroundClearance = 0.30f;
  for (const auto& point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const auto key = std::make_pair(
        static_cast<int>(std::floor(point.x * inverse_cell_size)),
        static_cast<int>(std::floor(point.y * inverse_cell_size)));
    const auto lowest = lowest_z.find(key);
    if (lowest != lowest_z.end() &&
        point.z >= lowest->second + kGroundClearance) {
      structure->push_back(point);
    }
  }
  normalize_cloud_layout(*structure);
  return structure;
}

inline LoopGeometryVerification verify_loop_geometry(
    const PointCloudType::ConstPtr& source,
    const PointCloudType::ConstPtr& target,
    const Eigen::Matrix4f& correction,
    const V3& source_anchor)
{
  LoopGeometryVerification result;
  if (!source || !target || source->empty() || target->empty() ||
      !correction.allFinite()) {
    return result;
  }

  CloudPtr aligned_source(new PointCloudType());
  pcl::transformPointCloud(*source, *aligned_source, correction);
  normalize_cloud_layout(*aligned_source);
  result.source_to_target = verify_loop_direction(
      aligned_source, target, g_loop_min_symmetric_overlap);
  result.target_to_source = verify_loop_direction(
      target, aligned_source, g_loop_min_symmetric_overlap);
  result.symmetric_overlap = std::min(
      result.source_to_target.overlap,
      result.target_to_source.overlap);
  result.symmetric_trimmed_rmse = std::max(
      result.source_to_target.trimmed_rmse,
      result.target_to_source.trimmed_rmse);

  const int main_supported_blocks = std::min(
      result.source_to_target.supported_blocks,
      result.target_to_source.supported_blocks);
  const float main_block_ratio = std::min(
      result.source_to_target.supported_block_ratio,
      result.target_to_source.supported_block_ratio);
  const float main_span = std::min(
      result.source_to_target.support_span,
      result.target_to_source.support_span);
  const float main_minor_span = std::min(
      result.source_to_target.support_minor_span,
      result.target_to_source.support_minor_span);
  result.main_valid =
      result.symmetric_overlap >= g_loop_min_symmetric_overlap &&
      result.symmetric_trimmed_rmse <= g_loop_max_trimmed_rmse &&
      main_supported_blocks >= g_loop_min_verification_blocks &&
      main_block_ratio >= g_loop_min_verification_block_ratio &&
      main_span >= g_loop_min_verification_span &&
      main_minor_span >= g_loop_verification_block_size;

  const CloudPtr aligned_structure =
      extract_loop_structural_points(aligned_source);
  const CloudPtr target_structure =
      extract_loop_structural_points(target);
  constexpr int kMinimumStructuralPoints = 200;
  result.structural_evidence_available =
      aligned_structure->size() >= kMinimumStructuralPoints &&
      target_structure->size() >= kMinimumStructuralPoints;
  if (result.structural_evidence_available) {
    result.structural_source_to_target = verify_loop_direction(
        aligned_structure, target_structure,
        g_loop_min_structural_overlap, 6);
    result.structural_target_to_source = verify_loop_direction(
        target_structure, aligned_structure,
        g_loop_min_structural_overlap, 6);
    result.structural_symmetric_overlap = std::min(
        result.structural_source_to_target.overlap,
        result.structural_target_to_source.overlap);
    result.structural_symmetric_trimmed_rmse = std::max(
        result.structural_source_to_target.trimmed_rmse,
        result.structural_target_to_source.trimmed_rmse);
    const int structural_supported_blocks = std::min(
        result.structural_source_to_target.supported_blocks,
        result.structural_target_to_source.supported_blocks);
    const float structural_block_ratio = std::min(
        result.structural_source_to_target.supported_block_ratio,
        result.structural_target_to_source.supported_block_ratio);
    const float structural_span = std::min(
        result.structural_source_to_target.support_span,
        result.structural_target_to_source.support_span);
    result.structural_valid =
        result.structural_symmetric_overlap >=
            g_loop_min_structural_overlap &&
        result.structural_symmetric_trimmed_rmse <=
            g_loop_max_trimmed_rmse &&
        structural_supported_blocks >= 3 &&
        structural_block_ratio >= 0.35f &&
        structural_span >=
            std::max(2.0f, 2.0f * g_loop_verification_block_size);
  }

  const Eigen::Vector3f anchor = source_anchor.cast<float>();
  const Eigen::Vector3f corrected_anchor =
      correction.block<3, 3>(0, 0) * anchor +
      correction.block<3, 1>(0, 3);
  result.anchor_translation = (corrected_anchor - anchor).norm();
  result.yaw_deg = evaluate_loop_rotation(
      correction.block<3, 3>(0, 0)).yaw_deg;
  result.large_correction =
      result.anchor_translation > 1.0f || result.yaw_deg > 1.0f;

  // If structural evidence exists it must agree. A correction large enough to
  // deform the complete map always needs that independent non-ground support;
  // floor-only or one-wall matches are allowed to prove Z, never XY/yaw.
  result.valid = result.main_valid &&
      (!result.structural_evidence_available || result.structural_valid) &&
      (!result.large_correction || result.structural_valid);

  const float overlap_quality = std::clamp(
      (result.symmetric_overlap - g_loop_min_symmetric_overlap) /
          std::max(1.0f - g_loop_min_symmetric_overlap, 1.0e-3f),
      0.0f, 1.0f);
  const float rmse_quality = std::clamp(
      (g_loop_max_trimmed_rmse - result.symmetric_trimmed_rmse) /
          std::max(g_loop_max_trimmed_rmse, 1.0e-3f),
      0.0f, 1.0f);
  const float block_quality = std::clamp(
      main_block_ratio, 0.0f, 1.0f);
  const float structural_quality = result.structural_valid ? 1.0f : 0.0f;
  result.confidence = std::clamp(
      0.35f * overlap_quality + 0.25f * rmse_quality +
          0.25f * block_quality + 0.15f * structural_quality,
      0.0f, 1.0f);
  return result;
}

using InternalVoxelKey = std::tuple<int, int, int>;

struct InternalReciprocalMatch
{
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Vector3f residual = Eigen::Vector3f::Zero();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using InternalReciprocalMatchVector = std::vector<
    InternalReciprocalMatch,
    Eigen::aligned_allocator<InternalReciprocalMatch>>;

struct InternalResidualModeAnalysis
{
  bool sufficient = false;
  bool significant = false;
  bool spatially_overlapping = false;
  int principal_axis = -1;
  int mode0_points = 0;
  int mode1_points = 0;
  int mode0_voxels = 0;
  int mode1_voxels = 0;
  int overlapping_voxels = 0;
  float separation = 0.0f;
  float within_mode_mad = 0.0f;
  float separation_threshold = 0.0f;
  float spatial_overlap_ratio = 0.0f;
};

struct InternalSupportVoxel
{
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  int match_count = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using InternalSupportVoxelVector = std::vector<
    InternalSupportVoxel,
    Eigen::aligned_allocator<InternalSupportVoxel>>;

struct InternalSupportVoxelSummary
{
  InternalSupportVoxelVector voxels;
  int reciprocal_matches = 0;
  int minimum_matches_per_voxel = 2;
  int minimum_total_matches = 0;
  float support_span = 0.0f;
  float support_minor_span = 0.0f;
  bool strict_valid = false;
};

using InternalAnchorCenterVector = std::vector<
    Eigen::Vector3f,
    Eigen::aligned_allocator<Eigen::Vector3f>>;

inline float internal_robust_median(std::vector<float> values)
{
  if (values.empty()) {
    return 0.0f;
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(
      values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle),
      values.end());
  const float upper = values[middle];
  if (values.size() % 2 != 0) {
    return upper;
  }
  const float lower = *std::max_element(
      values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
  return 0.5f * (lower + upper);
}

inline InternalVoxelKey internal_voxel_key(
    const Eigen::Vector3f& point,
    const float voxel_size)
{
  const float inverse_size = 1.0f / std::max(voxel_size, 1.0e-3f);
  return std::make_tuple(
      static_cast<int>(std::floor(point.x() * inverse_size)),
      static_cast<int>(std::floor(point.y() * inverse_size)),
      static_cast<int>(std::floor(point.z() * inverse_size)));
}

// Build a hold-out support set using reciprocal nearest neighbours.  Unlike
// the normal verifier this support is voxelized in XYZ, so two floors at the
// same XY coordinate cannot silently vote as one block.  Z is only a spatial
// coordinate here; no absolute height or planar assumption is introduced.
inline InternalReciprocalMatchVector collect_internal_reciprocal_matches(
    const PointCloudType::ConstPtr& source,
    const PointCloudType::ConstPtr& target,
    const Eigen::Matrix4f& correction)
{
  InternalReciprocalMatchVector matches;
  if (!source || !target || source->empty() || target->empty() ||
      !correction.allFinite()) {
    return matches;
  }

  CloudPtr aligned_source(new PointCloudType());
  pcl::transformPointCloud(*source, *aligned_source, correction);
  normalize_cloud_layout(*aligned_source);
  pcl::search::KdTree<PointType> source_tree;
  pcl::search::KdTree<PointType> target_tree;
  source_tree.setInputCloud(aligned_source);
  target_tree.setInputCloud(target);
  const float maximum_squared_distance =
      g_loop_verification_max_distance *
      g_loop_verification_max_distance;

  std::vector<int> target_to_source(target->size(), -1);
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);
  for (std::size_t target_index = 0;
       target_index < target->size();
       ++target_index) {
    const auto& point = target->points[target_index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    if (source_tree.nearestKSearch(
            point, 1, nearest_index,
            nearest_squared_distance) > 0 &&
        nearest_squared_distance[0] <= maximum_squared_distance) {
      target_to_source[target_index] = nearest_index[0];
    }
  }

  matches.reserve(std::min(source->size(), target->size()));
  for (std::size_t source_index = 0;
       source_index < aligned_source->size();
       ++source_index) {
    const auto& source_point = aligned_source->points[source_index];
    if (!std::isfinite(source_point.x) ||
        !std::isfinite(source_point.y) ||
        !std::isfinite(source_point.z)) {
      continue;
    }
    if (target_tree.nearestKSearch(
            source_point, 1, nearest_index,
            nearest_squared_distance) <= 0 ||
        nearest_squared_distance[0] > maximum_squared_distance) {
      continue;
    }
    const int target_index = nearest_index[0];
    if (target_index < 0 ||
        static_cast<std::size_t>(target_index) >= target_to_source.size() ||
        target_to_source[static_cast<std::size_t>(target_index)] !=
            static_cast<int>(source_index)) {
      continue;
    }
    const auto& target_point =
        target->points[static_cast<std::size_t>(target_index)];
    const Eigen::Vector3f source_xyz(
        source_point.x, source_point.y, source_point.z);
    const Eigen::Vector3f target_xyz(
        target_point.x, target_point.y, target_point.z);
    InternalReciprocalMatch match;
    match.position = 0.5f * (source_xyz + target_xyz);
    match.residual = target_xyz - source_xyz;
    matches.push_back(match);
  }
  return matches;
}

inline InternalSupportVoxelSummary analyze_internal_support_voxels(
    const InternalReciprocalMatchVector& matches)
{
  InternalSupportVoxelSummary result;
  result.reciprocal_matches = static_cast<int>(matches.size());
  result.minimum_total_matches =
      2 * std::max(30, 5 * g_loop_min_verification_blocks);
  struct Accumulator
  {
    int count = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };
  std::map<InternalVoxelKey, Accumulator> accumulators;
  for (const auto& match : matches) {
    auto& accumulator = accumulators[internal_voxel_key(
        match.position, g_loop_verification_block_size)];
    ++accumulator.count;
    accumulator.x += match.position.x();
    accumulator.y += match.position.y();
    accumulator.z += match.position.z();
  }

  // A fixed ten reciprocal pairs per 1 m^3 voxel is appropriate for a full
  // +/-W submap but systematically empties the small, downsampled local
  // crops. Adapt within a deliberately narrow 2..4 range from the observed
  // match density; total-match, spatial-footprint and strict geometry gates
  // below still prevent a tiny patch from becoming a graph edge.
  const float mean_matches_per_occupied_voxel = accumulators.empty()
      ? 0.0f
      : static_cast<float>(matches.size()) /
            static_cast<float>(accumulators.size());
  result.minimum_matches_per_voxel = std::clamp(
      static_cast<int>(std::ceil(
          0.5f * mean_matches_per_occupied_voxel)),
      2, 4);
  result.voxels.reserve(accumulators.size());
  bool has_voxel = false;
  Eigen::Vector3f minimum = Eigen::Vector3f::Zero();
  Eigen::Vector3f maximum = Eigen::Vector3f::Zero();
  for (const auto& [key, accumulator] : accumulators) {
    (void)key;
    if (accumulator.count < result.minimum_matches_per_voxel) {
      continue;
    }
    InternalSupportVoxel voxel;
    voxel.match_count = accumulator.count;
    const float inverse_count =
        1.0f / static_cast<float>(accumulator.count);
    voxel.center = Eigen::Vector3f(
        static_cast<float>(accumulator.x) * inverse_count,
        static_cast<float>(accumulator.y) * inverse_count,
        static_cast<float>(accumulator.z) * inverse_count);
    if (!has_voxel) {
      minimum = maximum = voxel.center;
      has_voxel = true;
    } else {
      minimum = minimum.cwiseMin(voxel.center);
      maximum = maximum.cwiseMax(voxel.center);
    }
    result.voxels.push_back(voxel);
  }
  if (has_voxel) {
    std::array<float, 3> extents = {
        maximum.x() - minimum.x(),
        maximum.y() - minimum.y(),
        maximum.z() - minimum.z()};
    std::sort(extents.begin(), extents.end(), std::greater<float>());
    result.support_span = extents[0];
    result.support_minor_span = extents[1];
  }
  result.strict_valid =
      result.reciprocal_matches >= result.minimum_total_matches &&
      static_cast<int>(result.voxels.size()) >=
          g_loop_min_verification_blocks &&
      result.support_span >= g_loop_min_verification_span &&
      result.support_minor_span >= g_loop_verification_block_size;
  return result;
}

inline InternalResidualModeAnalysis analyze_internal_residual_modes(
    const InternalReciprocalMatchVector& matches)
{
  InternalResidualModeAnalysis result;
  const int minimum_mode_points =
      std::max(30, 5 * g_loop_min_verification_blocks);
  if (static_cast<int>(matches.size()) < 2 * minimum_mode_points) {
    return result;
  }

  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (const auto& match : matches) {
    mean += match.residual;
  }
  mean /= static_cast<float>(matches.size());
  Eigen::Vector3f variance = Eigen::Vector3f::Zero();
  for (const auto& match : matches) {
    const Eigen::Vector3f delta = match.residual - mean;
    variance += delta.cwiseProduct(delta);
  }
  int axis = 0;
  if (variance.y() > variance.x()) {
    axis = 1;
  }
  if (variance.z() > variance(axis)) {
    axis = 2;
  }
  result.principal_axis = axis;

  std::vector<float> projections;
  projections.reserve(matches.size());
  for (const auto& match : matches) {
    projections.push_back(match.residual(axis));
  }
  std::vector<float> sorted = projections;
  std::sort(sorted.begin(), sorted.end());
  float center0 = sorted[sorted.size() / 4];
  float center1 = sorted[(3 * sorted.size()) / 4];
  std::vector<unsigned char> labels(projections.size(), 0);
  for (int iteration = 0; iteration < 8; ++iteration) {
    std::vector<float> mode0;
    std::vector<float> mode1;
    mode0.reserve(projections.size());
    mode1.reserve(projections.size());
    for (std::size_t i = 0; i < projections.size(); ++i) {
      labels[i] = std::abs(projections[i] - center0) <=
              std::abs(projections[i] - center1)
          ? 0U : 1U;
      (labels[i] == 0U ? mode0 : mode1).push_back(projections[i]);
    }
    if (mode0.empty() || mode1.empty()) {
      return result;
    }
    center0 = internal_robust_median(std::move(mode0));
    center1 = internal_robust_median(std::move(mode1));
  }

  std::array<std::vector<float>, 2> mode_values;
  std::array<std::map<InternalVoxelKey, int>, 2> voxel_counts;
  for (std::size_t i = 0; i < projections.size(); ++i) {
    const int mode = labels[i] == 0U ? 0 : 1;
    mode_values[mode].push_back(projections[i]);
    ++voxel_counts[mode][internal_voxel_key(
        matches[i].position, g_loop_verification_block_size)];
  }
  result.mode0_points = static_cast<int>(mode_values[0].size());
  result.mode1_points = static_cast<int>(mode_values[1].size());
  if (result.mode0_points < minimum_mode_points ||
      result.mode1_points < minimum_mode_points) {
    return result;
  }

  const float median0 = internal_robust_median(mode_values[0]);
  const float median1 = internal_robust_median(mode_values[1]);
  std::array<float, 2> mad = {0.0f, 0.0f};
  for (int mode = 0; mode < 2; ++mode) {
    const float median = mode == 0 ? median0 : median1;
    std::vector<float> deviations;
    deviations.reserve(mode_values[mode].size());
    for (const float value : mode_values[mode]) {
      deviations.push_back(std::abs(value - median));
    }
    mad[mode] = internal_robust_median(std::move(deviations));
  }

  constexpr int kMinimumModeMatchesPerVoxel = 5;
  std::array<std::set<InternalVoxelKey>, 2> supported_voxels;
  for (int mode = 0; mode < 2; ++mode) {
    for (const auto& [key, count] : voxel_counts[mode]) {
      if (count >= kMinimumModeMatchesPerVoxel) {
        supported_voxels[mode].insert(key);
      }
    }
  }
  result.mode0_voxels =
      static_cast<int>(supported_voxels[0].size());
  result.mode1_voxels =
      static_cast<int>(supported_voxels[1].size());
  for (const auto& key : supported_voxels[0]) {
    if (supported_voxels[1].count(key) != 0U) {
      ++result.overlapping_voxels;
    }
  }
  const int minimum_voxel_count =
      std::min(result.mode0_voxels, result.mode1_voxels);
  if (minimum_voxel_count > 0) {
    result.spatial_overlap_ratio =
        static_cast<float>(result.overlapping_voxels) /
        static_cast<float>(minimum_voxel_count);
  }

  result.separation = std::abs(median1 - median0);
  result.within_mode_mad = std::max(mad[0], mad[1]);
  result.separation_threshold = std::max(
      2.0f * g_loop_map_ds_size,
      3.0f * result.within_mode_mad);
  result.sufficient =
      result.mode0_voxels >= g_loop_min_verification_blocks &&
      result.mode1_voxels >= g_loop_min_verification_blocks;
  result.significant =
      result.sufficient &&
      result.separation > result.separation_threshold;
  result.spatially_overlapping =
      result.significant &&
      result.spatial_overlap_ratio >=
          0.5f * g_loop_min_verification_block_ratio;
  return result;
}

// Endpoint windows are commonly truncated in different directions: the
// departure window contains points after leaving the place while the arrival
// window contains points before reaching it.  Keep the normal verifier strict
// for every internal loop, but allow an endpoint-only partial-overlap mode
// when the matched evidence is accurate, broad and structural.  These limits
// deliberately use absolute block/span evidence in addition to ratios; a
// small patch on one repeated wall cannot pass this fallback.
inline bool endpoint_partial_geometry_is_valid(
    const LoopGeometryVerification& geometry)
{
  const int supported_blocks = std::min(
      geometry.source_to_target.supported_blocks,
      geometry.target_to_source.supported_blocks);
  const float block_ratio = std::min(
      geometry.source_to_target.supported_block_ratio,
      geometry.target_to_source.supported_block_ratio);
  const float support_span = std::min(
      geometry.source_to_target.support_span,
      geometry.target_to_source.support_span);
  const float support_minor_span = std::min(
      geometry.source_to_target.support_minor_span,
      geometry.target_to_source.support_minor_span);
  // Very long-range endpoint views can contain many non-common blocks even
  // when the absolute common support is extensive. Accept either the normal
  // ratio or a substantially stronger absolute footprint; later endpoint
  // consensus and the internal-only graph still decide whether that
  // correction is allowed to deform the map.
  const bool block_support_valid =
      block_ratio >= 0.30f ||
      (supported_blocks >= 60 && support_span >= 20.0f &&
       support_minor_span >= 10.0f && block_ratio >= 0.20f);
  return geometry.structural_evidence_available &&
      geometry.symmetric_overlap >= 0.38f &&
      geometry.symmetric_trimmed_rmse <= 0.25f &&
      supported_blocks >= 30 &&
      block_support_valid &&
      support_span >= 8.0f &&
      support_minor_span >= 3.0f &&
      geometry.structural_symmetric_overlap >= 0.30f &&
      geometry.structural_symmetric_trimmed_rmse <= 0.28f;
}

inline Eigen::Matrix3f interpolate_correction_rotation(
    const Eigen::Matrix4f& correction,
    const float alpha)
{
  const float a = std::clamp(alpha, 0.0f, 1.0f);
  Eigen::Quaternionf q_target(correction.block<3, 3>(0, 0));
  q_target.normalize();
  Eigen::Quaternionf q = Eigen::Quaternionf::Identity().slerp(a, q_target);
  return q.toRotationMatrix();
}

struct PoseGraphEdge
{
  int from = -1;
  int to = -1;
  // Several spatial anchors extracted from the same revisit are one physical
  // observation.  The optimizer may place those anchors at different nodes,
  // but their total information and all graph-safety decisions are grouped.
  int loop_group_id = -1;
  int anchor_id = -1;
  SE3 measurement;
  float weight = 1.0f;
  bool loop = false;
  bool endpoint = false;
  bool soft_fallback = false;
  int safety_downweight_level = 0;
  bool ground_z_valid = false;
  bool ground_z_planar_hold = false;
  // Proactive evidence produced under the exact pure-Z hypothesis.  It is
  // intentionally distinct from the leave-one-out planar hold above: the
  // five non-Z axes have zero information, rather than a weak equality prior.
  bool proactive_ground_z_only = false;
  // When independently validated ground evidence replaces a degenerate
  // full-SE(3) corridor edge, retain at most one heavily downweighted planar
  // companion.  It carries yaw/XY only: roll/pitch/Z remain completely
  // unconstrained so this edge cannot flatten a ramp or double-count the
  // vertical observation.
  bool proactive_planar_companion = false;
  // Degenerate-corridor endpoint observation.  The point-cloud registration
  // still supplies a full transform, but translation along the corridor is
  // intentionally unobservable and therefore carries zero graph information.
  // The stored axis is expressed in the world/correction tangent frame used
  // by the linear correction graph.
  bool corridor_partial = false;
  float corridor_axis_x = 0.0f;
  float corridor_axis_y = 0.0f;
  bool prediction_consistent = false;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using PoseVector = std::vector<SE3, Eigen::aligned_allocator<SE3>>;
using PoseGraphEdgeVector =
  std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge>>;

inline V6 pose_graph_residual(
    const SE3& from,
    const SE3& to,
    const SE3& measurement)
{
  return (measurement.inverse() * from.inverse() * to).log_vee();
}

inline V6 pose_graph_edge_measurement(const PoseGraphEdge& edge)
{
  V6 measurement = edge.measurement.log_vee();
  if (edge.ground_z_planar_hold || edge.proactive_ground_z_only) {
    // Ground evidence validates only the vertical loop correction. For the
    // other axes, zero means "preserve the raw relative planar geometry":
    // C_to(x/y/yaw) should stay close to C_from(x/y/yaw), rather than remain
    // unconstrained and inherit an older loop correction through the tail.
    measurement.head<5>().setZero();
  }
  return measurement;
}

inline V6 pose_graph_edge_axis_weights(const PoseGraphEdge& edge)
{
  V6 weights = V6::Ones();
  if (edge.proactive_ground_z_only) {
    weights.head<5>().setZero();
    return weights;
  }
  if (edge.proactive_planar_companion) {
    weights(0) = 0.0f;
    weights(1) = 0.0f;
    weights(5) = 0.0f;
    return weights;
  }
  if (edge.ground_z_planar_hold) {
    // A fixed per-edge weight becomes effectively rigid on a long loop: the
    // information of N unit odometry edges in series is 1/N, while the loop
    // information would stay constant. Express planar_hold_weight relative
    // to that accumulated odometry information instead. Consequently the
    // same setting has the same meaning on a short bag and a much larger map,
    // and a downgraded ground-Z loop can only nudge planar correction.
    const float keyframe_span = static_cast<float>(
        std::max(1, std::abs(edge.to - edge.from)));
    const float normalized_axis_weight =
        std::sqrt(
            g_loop_ground_z_planar_hold_weight / keyframe_span) /
        std::max(edge.weight, 1.0e-3f);
    weights.head<5>().setConstant(normalized_axis_weight);
  }
  return weights;
}

inline Eigen::Matrix<float, 6, 6> pose_graph_edge_sqrt_information(
    const PoseGraphEdge& edge)
{
  Eigen::Matrix<float, 6, 6> information_sqrt =
      pose_graph_edge_axis_weights(edge).asDiagonal();
  information_sqrt.topRows<3>() *= 2.0f;
  if (!edge.corridor_partial) {
    return information_sqrt;
  }

  Eigen::Vector2f corridor_axis(
      edge.corridor_axis_x, edge.corridor_axis_y);
  const float axis_norm = corridor_axis.norm();
  if (!std::isfinite(axis_norm) || axis_norm < 1.0e-4f) {
    // A malformed partial edge must fail closed as a normal full edge.  Such
    // an edge is never intentionally constructed, but keeping this fallback
    // avoids silently dropping both planar translation axes.
    return information_sqrt;
  }
  corridor_axis /= axis_norm;
  const Eigen::Vector2f normal(-corridor_axis.y(), corridor_axis.x());
  // n*n^T is an idempotent square-root information matrix: it penalizes only
  // normal displacement and exactly ignores sliding along the corridor.
  information_sqrt.block<2, 2>(3, 3) = normal * normal.transpose();
  return information_sqrt;
}

struct LoopConsistency
{
  float path_length = 0.0f;
  float translation_residual = 0.0f;
  float rotation_residual_deg = 0.0f;
  float weight = 0.0f;
};

inline LoopConsistency evaluate_loop_consistency(
    const PoseGraphEdge& edge,
    const PoseVector& reference_poses,
    const float path_length,
    const V3& later_raw_anchor)
{
  LoopConsistency result;
  result.path_length = std::max(path_length, 1.0e-3f);
  const V6 residual = pose_graph_residual(
      reference_poses[edge.from],
      reference_poses[edge.to],
      edge.measurement);
  // BASIC::SE3::log_vee() stores rotation first and translation second.
  // Its translation component is not the physical displacement error when a
  // correction rotates around the world origin. Compare the measured and
  // predicted corrections at the later raw sensor anchor instead.
  const SE3 predicted_measurement =
      reference_poses[edge.from].inverse() *
      reference_poses[edge.to];
  const V3 anchor_residual =
      (predicted_measurement * later_raw_anchor) -
      (edge.measurement * later_raw_anchor);
  if (edge.corridor_partial) {
    Eigen::Vector2f corridor_axis(
        edge.corridor_axis_x, edge.corridor_axis_y);
    const float axis_norm = corridor_axis.norm();
    if (std::isfinite(axis_norm) && axis_norm >= 1.0e-4f) {
      corridor_axis /= axis_norm;
      const Eigen::Vector2f normal(
          -corridor_axis.y(), corridor_axis.x());
      const float normal_residual = normal.dot(Eigen::Vector2f(
          static_cast<float>(anchor_residual.x()),
          static_cast<float>(anchor_residual.y())));
      result.translation_residual = std::hypot(
          normal_residual, static_cast<float>(anchor_residual.z()));
    } else {
      result.translation_residual = anchor_residual.norm();
    }
  } else {
    result.translation_residual = anchor_residual.norm();
  }
  result.rotation_residual_deg =
      residual.head<3>().norm() * 180.0f / static_cast<float>(M_PI);

  // This is a trajectory uncertainty model, not an XYZ/Z distance gate.
  // Expected uncertainty grows continuously with travelled path length and
  // all translation axes are treated together.  A real ramp or floor change
  // therefore remains untouched; only a loop observation that demands an
  // implausibly different correction from the current graph is switched off.
  const float translation_scale =
      g_loop_translation_drift_ratio * result.path_length;
  const float rotation_scale =
      g_loop_rotation_drift_deg_per_m * result.path_length;
  const float normalized_translation =
      result.translation_residual /
      std::max(translation_scale, 1.0e-6f);
  const float normalized_rotation =
      result.rotation_residual_deg /
      std::max(rotation_scale, 1.0e-6f);
  result.weight = 1.0f /
      (1.0f + normalized_translation * normalized_translation +
       normalized_rotation * normalized_rotation);
  return result;
}

inline float pose_graph_robust_cost(
    const PoseVector& poses,
    const PoseGraphEdgeVector& edges)
{
  float cost = 0.0f;
  for (const auto& edge : edges) {
    V6 residual;
    if (edge.ground_z_planar_hold || edge.proactive_ground_z_only) {
      residual =
          (poses[edge.from].inverse() * poses[edge.to]).log_vee() -
          pose_graph_edge_measurement(edge);
    } else {
      residual = pose_graph_residual(
          poses[edge.from], poses[edge.to], edge.measurement);
    }
    residual = edge.weight *
        pose_graph_edge_sqrt_information(edge) * residual;
    const float norm = residual.norm();
    if (edge.loop && norm > 1.0f) {
      cost += 2.0f * norm - 1.0f;
    } else {
      cost += residual.squaredNorm();
    }
  }
  return cost;
}

inline bool optimize_pose_graph(
    PoseVector& poses,
    const PoseGraphEdgeVector& edges,
    const int max_iterations = 8)
{
  if (poses.size() < 2 || edges.empty()) {
    return false;
  }

  const int variable_count =
      static_cast<int>((poses.size() - 1) * 6);
  // Loop corrections are small rotations (validated before reaching here),
  // so solve the correction graph in se(3):
  //
  //   x_to - x_from = log(G_from_to)
  //
  // Identity measurements on consecutive nodes form a Laplacian smoothness
  // term.  Loop measurements jointly deform the whole trajectory instead of
  // applying a rigid jump at either endpoint.  Solving in double precision is
  // important for a chain with thousands of nodes; the previous float
  // absolute-pose Gauss-Newton system was ill-conditioned.
  std::vector<Eigen::Matrix<double, 6, 1>> measurements;
  measurements.reserve(edges.size());
  for (const auto& edge : edges) {
    measurements.push_back(
        pose_graph_edge_measurement(edge).cast<double>());
  }

  Eigen::VectorXd solution =
      Eigen::VectorXd::Zero(variable_count);
  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(edges.size() * 144 + variable_count);
    Eigen::VectorXd rhs =
        Eigen::VectorXd::Zero(variable_count);

    auto correction_component = [&](
        const int node, const int axis) -> double {
      if (node <= 0) {
        return 0.0;
      }
      return solution((node - 1) * 6 + axis);
    };

    for (std::size_t edge_index = 0;
         edge_index < edges.size();
         ++edge_index) {
      const auto& edge = edges[edge_index];
      const auto& measurement = measurements[edge_index];
      Eigen::Matrix<double, 6, 1> residual;
      for (int axis = 0; axis < 6; ++axis) {
        residual(axis) =
            correction_component(edge.to, axis) -
            correction_component(edge.from, axis) -
            measurement(axis);
      }

      const Eigen::Matrix<double, 6, 6> information_sqrt =
          pose_graph_edge_sqrt_information(edge).cast<double>();
      const Eigen::Matrix<double, 6, 1> scaled_residual =
          static_cast<double>(edge.weight) *
          information_sqrt * residual;
      double robust_weight = 1.0;
      const double residual_norm = scaled_residual.norm();
      if (edge.loop && residual_norm > 1.0) {
        robust_weight = 1.0 / residual_norm;
      }

      const Eigen::Matrix<double, 6, 6> information =
          robust_weight *
          static_cast<double>(edge.weight) *
          static_cast<double>(edge.weight) *
          information_sqrt.transpose() * information_sqrt;
      const Eigen::Matrix<double, 6, 1> weighted_measurement =
          information * measurement;
      const int from_base =
          edge.from > 0 ? (edge.from - 1) * 6 : -1;
      const int to_base = edge.to > 0 ? (edge.to - 1) * 6 : -1;
      for (int row = 0; row < 6; ++row) {
        if (from_base >= 0) {
          rhs(from_base + row) -= weighted_measurement(row);
        }
        if (to_base >= 0) {
          rhs(to_base + row) += weighted_measurement(row);
        }
        for (int column = 0; column < 6; ++column) {
          const double value = information(row, column);
          if (std::abs(value) <= 1.0e-15) {
            continue;
          }
          if (from_base >= 0) {
            triplets.emplace_back(
                from_base + row, from_base + column, value);
          }
          if (to_base >= 0) {
            triplets.emplace_back(
                to_base + row, to_base + column, value);
          }
          if (from_base >= 0 && to_base >= 0) {
            triplets.emplace_back(
                from_base + row, to_base + column, -value);
            triplets.emplace_back(
                to_base + row, from_base + column, -value);
          }
        }
      }
    }

    for (int diagonal = 0;
         diagonal < variable_count;
         ++diagonal) {
      triplets.emplace_back(diagonal, diagonal, 1.0e-10);
    }
    Eigen::SparseMatrix<double> hessian(
        variable_count, variable_count);
    hessian.setFromTriplets(
        triplets.begin(), triplets.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(hessian);
    if (solver.info() != Eigen::Success) {
      return false;
    }
    const Eigen::VectorXd next_solution =
        solver.solve(rhs);
    if (solver.info() != Eigen::Success ||
        !next_solution.allFinite()) {
      return false;
    }
    const double update =
        (next_solution - solution).cwiseAbs().maxCoeff();
    solution = next_solution;
    if (update < 1.0e-7) {
      break;
    }
  }

  poses[0] = SE3();
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const V6 correction =
        solution.segment<6>((i - 1) * 6).cast<float>();
    poses[i] = SE3(correction);
  }
  return true;
}

struct TimedPoseCorrection
{
  double timestamp = 0.0;
  Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using TimedPoseCorrectionVector = std::vector<
  TimedPoseCorrection,
  Eigen::aligned_allocator<TimedPoseCorrection>>;

inline bool save_loop_trajectory_safe(
    const std::string& filename,
    const TimedPoseCorrectionVector& corrections)
{
  const auto temporary_filename = filename + ".tmp";
  try {
    std::ofstream output(
        temporary_filename,
        std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
      return false;
    }
    output << "NAV_LIO_POSE_GRAPH_V1\n";
    output << corrections.size() << "\n";
    output << std::setprecision(17);
    for (const auto& sample : corrections) {
      output << sample.timestamp;
      output << std::setprecision(9);
      for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
          output << " " << sample.correction(row, col);
        }
      }
      output << "\n" << std::setprecision(17);
    }
    output.flush();
    if (!output.good()) {
      output.close();
      std::filesystem::remove(temporary_filename);
      return false;
    }
    output.close();
    std::error_code ec;
    std::filesystem::rename(
        temporary_filename, filename, ec);
    if (ec) {
      std::filesystem::remove(temporary_filename);
      return false;
    }
    return true;
  } catch (...) {
    std::error_code ec;
    std::filesystem::remove(temporary_filename, ec);
    return false;
  }
}

inline bool save_loop_correction_safe(
    const std::string& filename,
    const double candidate_timestamp,
    const double deformation_start_timestamp,
    const double end_timestamp,
    const Eigen::Matrix4f& correction)
{
  const auto temporary_filename = filename + ".tmp";
  try {
    std::ofstream output(temporary_filename, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
      LOG(ERROR) << RED << " ---> 无法创建闭环校正元数据: "
                 << temporary_filename << RESET;
      return false;
    }

    output << "NAV_LIO_LOOP_CORRECTION_V2\n";
    output << std::setprecision(17)
           << candidate_timestamp << " "
           << deformation_start_timestamp << " "
           << end_timestamp << "\n";
    output << std::setprecision(9);
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        if (row != 0 || col != 0) {
          output << " ";
        }
        output << correction(row, col);
      }
    }
    output << "\n";
    output.flush();
    if (!output.good()) {
      output.close();
      std::filesystem::remove(temporary_filename);
      LOG(ERROR) << RED << " ---> 写入闭环校正元数据失败: "
                 << temporary_filename << RESET;
      return false;
    }
    output.close();

    std::error_code ec;
    std::filesystem::rename(temporary_filename, filename, ec);
    if (ec) {
      std::filesystem::remove(temporary_filename);
      LOG(ERROR) << RED << " ---> 提交闭环校正元数据失败: "
                 << filename << " error: " << ec.message() << RESET;
      return false;
    }
    return true;
  } catch (const std::exception& e) {
    std::error_code ec;
    std::filesystem::remove(temporary_filename, ec);
    LOG(ERROR) << RED << " ---> 保存闭环校正元数据失败: "
               << filename << " error: " << e.what() << RESET;
    return false;
  }
}

inline double wrap_period_signed(const double angle, const double period)
{
  return angle - period * std::floor(angle / period + 0.5);
}

inline double mean_manhattan_axis(const std::deque<double>& angles)
{
  double cosine_sum = 0.0;
  double sine_sum = 0.0;
  for (const double angle : angles) {
    cosine_sum += std::cos(4.0 * angle);
    sine_sum += std::sin(4.0 * angle);
  }
  return 0.25 * std::atan2(sine_sum, cosine_sum);
}

struct LidarPoseObservability
{
  double raw_yaw_ratio = 0.0;
  double conditional_yaw_ratio = 0.0;
  double translation_ratio = 0.0;
};

// A distant wall can make the yaw diagonal of H grow with range squared while
// yaw remains strongly coupled to a translation normal to that wall. Marginalize
// the other five pose coordinates before deciding that yaw is independently
// observable. The same strongest-rotation denominator as the historical raw
// ratio keeps the configured weak/strong thresholds dimensionless.
inline LidarPoseObservability evaluate_lidar_pose_observability(
    const M6d& information,
    const V3d& yaw_axis_body,
    const V3d& rotation_eigenvalues,
    const V3d& translation_eigenvalues)
{
  LidarPoseObservability result;
  if (!information.allFinite() || !yaw_axis_body.allFinite()) {
    return result;
  }

  V3d yaw_axis = yaw_axis_body;
  const double yaw_norm = yaw_axis.norm();
  if (!std::isfinite(yaw_norm) || yaw_norm <= 1e-12) {
    return result;
  }
  yaw_axis /= yaw_norm;

  const M3d rotation_information =
      0.5 * (information.block<3, 3>(0, 0) +
             information.block<3, 3>(0, 0).transpose());
  const double strongest_rotation_information = std::max(
      rotation_eigenvalues.allFinite()
          ? rotation_eigenvalues.maxCoeff()
          : 0.0,
      1e-12);
  const double raw_yaw_information = std::max(
      yaw_axis.dot(rotation_information * yaw_axis), 0.0);
  result.raw_yaw_ratio = std::clamp(
      raw_yaw_information / strongest_rotation_information, 0.0, 1.0);
  if (translation_eigenvalues.allFinite() &&
      translation_eigenvalues.maxCoeff() > 1e-12) {
    result.translation_ratio = std::clamp(
        std::max(translation_eigenvalues.minCoeff(), 0.0) /
            translation_eigenvalues.maxCoeff(),
        0.0,
        1.0);
  }

  // First marginalize XYZ in its own units. A pseudo-inverse may only discard
  // a null mode when the yaw/rotation cross block has no component in it;
  // otherwise the apparent rotation information is exactly the unresolved
  // rotation/translation coupling we need to reject.
  const M3d translation_information =
      0.5 * (information.block<3, 3>(3, 3) +
             information.block<3, 3>(3, 3).transpose());
  const M3d rotation_translation_information =
      information.block<3, 3>(0, 3);
  Eigen::SelfAdjointEigenSolver<M3d> translation_solver(
      translation_information);
  if (translation_solver.info() != Eigen::Success ||
      !translation_solver.eigenvalues().allFinite() ||
      !translation_solver.eigenvectors().allFinite()) {
    return result;
  }
  const double largest_translation_eigenvalue = std::max(
      translation_solver.eigenvalues().maxCoeff(), 0.0);
  const double translation_cutoff = std::max(
      1e-12, 1e-12 * largest_translation_eigenvalue);
  const double translation_null_scale = std::max({
      rotation_translation_information.norm(),
      std::sqrt(
          std::max(strongest_rotation_information, 0.0) *
          std::max(largest_translation_eigenvalue, 0.0)),
      1e-12});
  const double translation_null_tolerance =
      1e-6 * translation_null_scale;
  M3d translation_pseudo_inverse = M3d::Zero();
  for (int index = 0; index < 3; ++index) {
    const double eigenvalue = translation_solver.eigenvalues()[index];
    const V3d eigenvector = translation_solver.eigenvectors().col(index);
    if (eigenvalue > translation_cutoff) {
      translation_pseudo_inverse.noalias() +=
          eigenvector * eigenvector.transpose() / eigenvalue;
      continue;
    }
    const V3d null_coupling =
        rotation_translation_information * eigenvector;
    if (null_coupling.norm() > translation_null_tolerance) {
      return result;
    }
  }
  M3d marginalized_rotation_information =
      rotation_information -
      rotation_translation_information * translation_pseudo_inverse *
          rotation_translation_information.transpose();
  marginalized_rotation_information = 0.5 *
      (marginalized_rotation_information +
       marginalized_rotation_information.transpose());
  Eigen::SelfAdjointEigenSolver<M3d> marginalized_rotation_solver(
      marginalized_rotation_information);
  if (marginalized_rotation_solver.info() != Eigen::Success ||
      !marginalized_rotation_solver.eigenvalues().allFinite() ||
      marginalized_rotation_solver.eigenvalues().minCoeff() <
          -1e-7 * std::max(strongest_rotation_information, 1.0)) {
    return result;
  }

  V3d helper = std::abs(yaw_axis.z()) < 0.9
      ? V3d::UnitZ()
      : V3d::UnitX();
  V3d rotation_tangent = helper - yaw_axis * yaw_axis.dot(helper);
  const double tangent_norm = rotation_tangent.norm();
  if (!std::isfinite(tangent_norm) || tangent_norm <= 1e-12) {
    return result;
  }
  rotation_tangent /= tangent_norm;
  const V3d rotation_bitangent = yaw_axis.cross(rotation_tangent).normalized();
  M3d rotation_basis;
  rotation_basis.col(0) = yaw_axis;
  rotation_basis.col(1) = rotation_tangent;
  rotation_basis.col(2) = rotation_bitangent;
  const M3d yaw_basis_information = rotation_basis.transpose() *
      marginalized_rotation_information * rotation_basis;
  const double yaw_diagonal = std::max(yaw_basis_information(0, 0), 0.0);
  const Eigen::Vector2d attitude_cross =
      yaw_basis_information.block<2, 1>(1, 0);
  const Eigen::Matrix2d attitude_nuisance =
      0.5 * (yaw_basis_information.block<2, 2>(1, 1) +
             yaw_basis_information.block<2, 2>(1, 1).transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> attitude_solver(
      attitude_nuisance);
  if (attitude_solver.info() != Eigen::Success ||
      !attitude_solver.eigenvalues().allFinite() ||
      !attitude_solver.eigenvectors().allFinite()) {
    return result;
  }
  const double largest_attitude_eigenvalue = std::max(
      attitude_solver.eigenvalues().maxCoeff(), 0.0);
  const double attitude_cutoff = std::max(
      1e-12, 1e-12 * largest_attitude_eigenvalue);
  const double attitude_null_scale = std::max({
      attitude_cross.norm(),
      std::sqrt(
          std::max(yaw_diagonal, 0.0) *
          std::max(largest_attitude_eigenvalue, 0.0)),
      1e-12});
  const double attitude_null_tolerance = 1e-6 * attitude_null_scale;
  Eigen::Matrix2d attitude_pseudo_inverse = Eigen::Matrix2d::Zero();
  for (int index = 0; index < 2; ++index) {
    const double eigenvalue = attitude_solver.eigenvalues()[index];
    const Eigen::Vector2d eigenvector =
        attitude_solver.eigenvectors().col(index);
    const double null_coupling = eigenvector.dot(attitude_cross);
    if (eigenvalue > attitude_cutoff) {
      attitude_pseudo_inverse.noalias() +=
          eigenvector * eigenvector.transpose() / eigenvalue;
    } else if (std::abs(null_coupling) > attitude_null_tolerance) {
      return result;
    }
  }
  const double conditional_yaw_information = std::clamp(
      yaw_diagonal -
          attitude_cross.dot(attitude_pseudo_inverse * attitude_cross),
      0.0,
      yaw_diagonal);
  result.conditional_yaw_ratio = std::clamp(
      conditional_yaw_information / strongest_rotation_information,
      0.0,
      1.0);

  return result;
}

inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd)
{
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6f) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];

  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1) return false;
  }
  return true;
}


inline bool compute_error(
  const std::array<double, 4>& abcd, const V3& point, 
  const float length, scalar& error)
{
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}


SuperLIO::~SuperLIO()
{
  stopOnlineLoopWorker();
  stopMapWriter();
}


bool SuperLIO::initializeMapPersistence()
{
  const bool fragments_enabled =
      g_map_max_points_in_memory > 0 || g_pcd_save_interval > 0;
  map_persistence_enabled_ =
      fragments_enabled ||
      (g_loop_closure_enable && g_loop_persist_keyframes);
  if (!map_persistence_enabled_) {
    return true;
  }

  map_work_dir_ = std::filesystem::path(g_save_map_dir) / ".mapping_work";
  map_fragment_dir_ = map_work_dir_ / "fragments";
  loop_keyframe_dir_ = map_work_dir_ / "loop_keyframes";

  std::error_code ec;
  std::filesystem::remove_all(map_work_dir_, ec);
  if (ec) {
    LOG(ERROR) << RED << " ---> 清理建图工作目录失败: "
               << map_work_dir_.string()
               << " error: " << ec.message() << RESET;
    return false;
  }
  std::filesystem::create_directories(map_fragment_dir_, ec);
  if (ec) {
    LOG(ERROR) << RED << " ---> 创建地图分片目录失败: "
               << map_fragment_dir_.string()
               << " error: " << ec.message() << RESET;
    return false;
  }
  std::filesystem::create_directories(loop_keyframe_dir_, ec);
  if (ec) {
    LOG(ERROR) << RED << " ---> 创建回环关键帧目录失败: "
               << loop_keyframe_dir_.string()
               << " error: " << ec.message() << RESET;
    return false;
  }

  map_writer_stopping_ = false;
  map_writer_failed_ = false;
  map_writer_failure_logged_ = false;
  map_write_jobs_outstanding_ = 0;
  map_writer_queue_full_count_ = 0;
  map_writer_max_outstanding_ = 0;
  map_writer_thread_ = std::thread(&SuperLIO::mapWriterLoop, this);
  LOG(INFO) << GREEN
            << " ---> [SuperLIO]: bounded map persistence enabled."
            << " max_points_in_memory=" << g_map_max_points_in_memory
            << " save_interval=" << g_pcd_save_interval
            << " max_pending_writes=" << g_map_max_pending_writes
            << " persist_loop_keyframes=" << g_loop_persist_keyframes
            << " work_dir=" << map_work_dir_.string() << RESET;
  return true;
}


SuperLIO::CloudWriteEnqueueStatus SuperLIO::enqueueCloudWrite(
    const std::filesystem::path& path,
    const CloudPtr& cloud,
    const bool downsample,
    const std::string& context,
    const bool wait_for_space)
{
  if (!cloud || cloud->empty()) {
    return CloudWriteEnqueueStatus::Queued;
  }
  if (!map_persistence_enabled_ || !map_writer_thread_.joinable()) {
    return CloudWriteEnqueueStatus::Unavailable;
  }

  std::unique_lock<std::mutex> lock(map_writer_mutex_);
  const auto has_space = [this]() {
    return map_writer_failed_ || map_writer_stopping_ ||
        map_write_jobs_outstanding_ <
            static_cast<std::size_t>(g_map_max_pending_writes);
  };
  if (wait_for_space) {
    map_writer_space_cv_.wait(lock, has_space);
  } else if (!has_space()) {
    ++map_writer_queue_full_count_;
    if (map_writer_queue_full_count_ == 1U ||
        map_writer_queue_full_count_ % 50U == 0U) {
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: background map writer busy; "
                   << "retaining cloud in RAM without blocking LIO frontend."
                   << " context=" << context
                   << " outstanding=" << map_write_jobs_outstanding_
                   << " capacity=" << g_map_max_pending_writes
                   << " busy_events=" << map_writer_queue_full_count_
                   << RESET;
    }
    return CloudWriteEnqueueStatus::Busy;
  }
  if (map_writer_failed_ || map_writer_stopping_) {
    return CloudWriteEnqueueStatus::Unavailable;
  }

  map_write_queue_.push_back(
      CloudWriteJob{path, cloud, downsample, context});
  ++map_write_jobs_outstanding_;
  map_writer_max_outstanding_ = std::max(
      map_writer_max_outstanding_, map_write_jobs_outstanding_);
  lock.unlock();
  map_writer_cv_.notify_one();
  return CloudWriteEnqueueStatus::Queued;
}


void SuperLIO::mapWriterLoop()
{
  while (true) {
    CloudWriteJob job;
    {
      std::unique_lock<std::mutex> lock(map_writer_mutex_);
      map_writer_cv_.wait(lock, [&]() {
        return map_writer_stopping_ || !map_write_queue_.empty();
      });
      if (map_write_queue_.empty()) {
        if (map_writer_stopping_) {
          break;
        }
        continue;
      }
      job = std::move(map_write_queue_.front());
      map_write_queue_.pop_front();
    }

    const auto write_start = std::chrono::steady_clock::now();
    const std::size_t input_points = job.cloud ? job.cloud->size() : 0U;
    CloudPtr output = job.cloud;
    CloudPtr filtered;
    if (job.downsample) {
      filtered.reset(new PointCloudType());
      make_map_pcd_cloud(
          job.cloud,
          *filtered,
          g_map_ds_size,
          job.context.c_str());
      output = filtered;
    }
    const std::string failed_path = job.path.string();
    const bool saved =
        output && !output->empty() &&
        save_pcd_binary_safe(job.path.string(), *output);
    const double write_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - write_start).count();
    if (write_seconds > 0.5) {
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: slow background cloud write."
                   << " context=" << job.context
                   << " seconds=" << write_seconds
                   << " input_points=" << input_points
                   << " output_points=" << (output ? output->size() : 0U)
                   << " path=" << job.path.string() << RESET;
    }

    {
      std::lock_guard<std::mutex> lock(map_writer_mutex_);
      if (!saved) {
        map_writer_failed_ = true;
        failed_cloud_write_jobs_.push_back(std::move(job));
        while (!map_write_queue_.empty()) {
          failed_cloud_write_jobs_.push_back(
              std::move(map_write_queue_.front()));
          map_write_queue_.pop_front();
          if (map_write_jobs_outstanding_ > 0) {
            --map_write_jobs_outstanding_;
          }
        }
        map_writer_stopping_ = true;
      }
      if (map_write_jobs_outstanding_ > 0) {
        --map_write_jobs_outstanding_;
      }
    }
    map_writer_space_cv_.notify_all();

    if (!saved) {
      LOG(ERROR) << RED
                 << " ---> [SuperLIO]: background map write failed; "
                 << "new fragments will remain in RAM until shutdown. path="
                 << failed_path << RESET;
      map_writer_cv_.notify_all();
      break;
    }
  }
}


bool SuperLIO::stopMapWriter()
{
  if (!map_writer_thread_.joinable()) {
    return !map_writer_failed_;
  }
  {
    std::lock_guard<std::mutex> lock(map_writer_mutex_);
    map_writer_stopping_ = true;
  }
  map_writer_cv_.notify_all();
  map_writer_space_cv_.notify_all();
  map_writer_thread_.join();

  std::vector<CloudWriteJob> failed_jobs;
  {
    std::lock_guard<std::mutex> lock(map_writer_mutex_);
    failed_jobs.swap(failed_cloud_write_jobs_);
  }
  bool all_saved = true;
  for (auto& job : failed_jobs) {
    CloudPtr output = job.cloud;
    CloudPtr filtered;
    if (job.downsample) {
      filtered.reset(new PointCloudType());
      make_map_pcd_cloud(
          job.cloud,
          *filtered,
          g_map_ds_size,
          job.context.c_str());
      output = filtered;
    }
    if (!output || output->empty() ||
        !save_pcd_binary_safe(job.path.string(), *output)) {
      all_saved = false;
      std::lock_guard<std::mutex> lock(map_writer_mutex_);
      failed_cloud_write_jobs_.push_back(std::move(job));
    }
  }
  map_writer_failed_ = !all_saved;
  return all_saved;
}


void SuperLIO::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);

  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());

  if(g_save_map){
    point_map_.reset(new PointCloudType());
    map_preview_.reset(new PointCloudType());
    map_preview_pending_.reset(new PointCloudType());
    map_preview_effective_leaf_size_ = std::max(g_map_preview_ds_size, 0.01f);
    if (!initializeMapPersistence()) {
      throw std::runtime_error(
          "failed to initialize bounded map persistence");
    }
    if (!initializeOnlineLoopWorker()) {
      throw std::runtime_error(
          "failed to initialize online loop worker");
    }
  }

  points_world_v3_.reserve(21000);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  effect_mask_.resize(20000, false);
  effect_knn_mask_.resize(20000, false);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  state_fn_ = &SuperLIO::stateWaitKFInit;

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;
}


void SuperLIO::stateWaitKFInit()
{
  if (kf_init()) {
    state_fn_ = &SuperLIO::stateWaitMapInit;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: KF init done" << RESET;
  }
}

void SuperLIO::stateWaitMapInit()
{
  if (map_init()) {
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init done" << RESET;
  }
}

void SuperLIO::process(){
  if(!data_wrapper_->sync_measure(measures_)){
    return;
  }
  (this->*state_fn_)();
  data_wrapper_->finish_measure();
}


bool SuperLIO::kf_init(){
  for(auto& imu: measures_.imu){
    imu_init_window_.push_back(imu);
    while (imu_init_window_.size() > static_cast<std::size_t>(g_imu_init_samples)) {
      imu_init_window_.pop_front();
    }
  }

  if (imu_init_window_.size() < static_cast<std::size_t>(g_imu_init_samples)) {
    return false;
  }

  V3 mean_gyro = V3::Zero();
  V3 mean_acce = V3::Zero();
  for (const auto& imu : imu_init_window_) {
    mean_gyro += imu.gyr;
    mean_acce += imu.acc;
  }
  const scalar sample_count = static_cast<scalar>(imu_init_window_.size());
  mean_gyro /= sample_count;
  mean_acce /= sample_count;

  V3 gyro_variance = V3::Zero();
  V3 accel_variance = V3::Zero();
  for (const auto& imu : imu_init_window_) {
    gyro_variance += (imu.gyr - mean_gyro).cwiseAbs2();
    accel_variance += (imu.acc - mean_acce).cwiseAbs2();
  }
  const scalar variance_denominator = std::max<scalar>(sample_count - 1.0, 1.0);
  const V3 gyro_stddev = (gyro_variance / variance_denominator).cwiseSqrt();
  const V3 accel_stddev = (accel_variance / variance_denominator).cwiseSqrt();
  const double accel_mean_norm = mean_acce.norm();
  const double accel_stddev_ratio =
      accel_mean_norm > 1e-6 ? accel_stddev.maxCoeff() / accel_mean_norm
                             : std::numeric_limits<double>::infinity();
  const bool stationary =
      mean_gyro.norm() <= g_imu_init_max_gyro_norm &&
      gyro_stddev.maxCoeff() <= g_imu_init_max_gyro_stddev &&
      accel_stddev_ratio <= g_imu_init_max_accel_stddev_ratio;
  if (!stationary) {
    const auto now = std::chrono::steady_clock::now();
    if (last_imu_motion_warning_time_ == std::chrono::steady_clock::time_point{} ||
        now - last_imu_motion_warning_time_ > std::chrono::seconds(1)) {
      last_imu_motion_warning_time_ = now;
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: waiting for stationary IMU initialization. "
                   << "gyro_mean_norm=" << mean_gyro.norm()
                   << " gyro_stddev=" << gyro_stddev.transpose()
                   << " accel_stddev_ratio=" << accel_stddev_ratio << RESET;
    }
    return false;
  }

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, - g_gravity_norm);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 

  // init_rot represents the IMU orientation after gravity alignment (level orientation).
  // Perform LiDAR leveling correction, then transform the orientation into the robot frame.
  M3 rot = g_lidar_robo_yaw * R_yaw_inv * init_rot;  

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;
  options.estimate_gravity_ = g_kf_estimate_gravity;

  float imu_scale = g_gravity_norm / mean_acce.norm();
  kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  state.R = SO3(rot);
  state.p = g_odom_robo.t_;        // By default, the robot frame is used as the reference origin.
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  imu_reference_accel_norm_ = mean_acce.norm();
  gravity_direction_window_.clear();
  gravity_reference_world_ = V3::UnitZ();
  gravity_reference_valid_ = false;
  last_gravity_sample_time_ = -1.0;
  level_constraint_accepted_count_ = 0;
  level_constraint_rejected_count_ = 0;
  level_constraint_gated_count_ = 0;
  level_slope_protection_active_ = false;
  level_slope_enter_count_ = 0;
  level_slope_exit_count_ = 0;
  level_slope_pending_invalid_count_ = 0;
  level_slope_recovery_active_ = false;
  level_slope_recovery_count_ = 0;
  level_slope_spatial_path_m_ = 0.0;
  level_slope_spatial_supported_path_m_ = 0.0;
  level_slope_spatial_observed_dz_m_ = 0.0;
  level_slope_spatial_expected_dz_m_ = 0.0;
  level_slope_spatial_mismatch_count_ = 0;
  level_slope_spatial_reentry_blocked_ = false;
  level_slope_spatial_reentry_consistent_count_ = 0;
  level_slope_spatial_last_observed_grade_deg_ = 0.0;
  level_slope_spatial_last_expected_grade_deg_ = 0.0;
  level_slope_spatial_last_error_deg_ = 0.0;
  level_slope_spatial_last_support_ratio_ = 0.0;
  ground_height_continuity_reference_ =
      GroundHeightContinuityReference{};
  ground_height_continuity_accepted_count_ = 0;
  ground_height_continuity_rejected_count_ = 0;
  ground_height_continuity_gated_count_ = 0;
  ground_height_continuity_budget_gated_count_ = 0;
  ground_height_continuity_applied_offset_m_ = 0.0;
  wall_yaw_reference_samples_.clear();
  wall_yaw_references_.clear();
  wall_yaw_recapture_state_ = WallYawRecaptureState{};
  wall_yaw_reference_capacity_warning_logged_ = false;
  wall_yaw_constraint_accepted_count_ = 0;
  wall_yaw_constraint_rejected_count_ = 0;
  wall_yaw_constraint_gated_count_ = 0;
  wall_yaw_extraction_skipped_count_ = 0;
  LOG(INFO) << GREEN << " ---> [SuperLIO]: IMU initialized with "
            << imu_init_window_.size() << " stationary samples, gyro_bias="
            << mean_gyro.transpose() << " gyro_stddev=" << gyro_stddev.transpose()
            << " accel_mean=" << mean_acce.transpose()
            << " accel_stddev_ratio=" << accel_stddev_ratio << RESET;
  imu_init_window_.clear();
  return true;
}


bool SuperLIO::map_init(){
  frame_num_++;

  std::size_t ptsize = measures_.lidar.pc->size();
  points_world_v3_.resize(ptsize);

  const SE3 transform = sys_init_pose_ * g_lidar_imu;

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize),
    [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& point_pcl = measures_.lidar.pc->points[idx];
        V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
        points_world_v3_[idx] = transform * point_body;
      }
    }
  );

  ivox_->insert(points_world_v3_);
  kf_->SetLastObsTime(measures_.lidar.end_time);

  // 20 Hz for 1.0 seconds. Integral coverage area > 70%
  if(frame_num_ > 3){
    last_pose_ = kf_->GetSE3();
    has_last_accepted_pose_ = true;
    last_accepted_state_ = kf_->GetSysState();
    last_accepted_covariance_ = kf_->GetCov();
    has_last_accepted_state_ = true;
    g_flg_map_init = false;
    return true;
  }
  return false;
}


void SuperLIO::stateProcess(){
  frame_num_++;
  // frame_num_ is a legacy algorithm counter and is also incremented at the
  // end of Observe().  Keep a separate once-per-LiDAR sequence for persisted
  // frontend diagnostics so keyframes can be correlated without that double
  // increment.
  ++processed_scan_index_;
  latest_prediction_pose_valid_ = false;
  if(g_time_eva){
    bool undistortion_valid = false;
    time_record_.Evaluate(
      [this, &undistortion_valid](){
        undistortion_valid = Propagation_Undistort();
      }, "[Undistort]");
    if (!undistortion_valid) {
      observation_valid_ = false;
      return;
    }
    time_record_.Evaluate([this]() { DownSample(); }, "[DownSample]");
    time_record_.Evaluate([this]() { Observe(); }, "[Observe]");
    time_record_.Evaluate([this]() { UpdateMap(); }, "[UpdateMap]");
    time_record_.Evaluate([this]() { Output(); }, "[Output]");
    time_record_.Evaluate([this]() { caceData(); }, "[CacheData]");
  }else{
    if (!Propagation_Undistort()) {
      observation_valid_ = false;
      return;
    }
    DownSample();
    Observe();
    UpdateMap();
    Output();
    caceData();
  }
}


bool SuperLIO::flushMapFragment(const bool force)
{
  const bool fragments_enabled =
      g_map_max_points_in_memory > 0 || g_pcd_save_interval > 0;
  if (!fragments_enabled || !point_map_ || point_map_->empty()) {
    return true;
  }

  const bool point_limit_reached =
      g_map_max_points_in_memory > 0 &&
      point_map_->size() >=
          static_cast<std::size_t>(g_map_max_points_in_memory);
  const bool scan_limit_reached =
      g_pcd_save_interval > 0 &&
      map_scans_since_fragment_ >= g_pcd_save_interval;
  if (!force && !point_limit_reached && !scan_limit_reached) {
    return true;
  }

  std::ostringstream filename;
  filename << "fragment_" << std::setw(6) << std::setfill('0')
           << (pcd_index_ + 1) << ".pcd";
  const auto path = map_fragment_dir_ / filename.str();
  CloudPtr fragment = point_map_;
  point_map_.reset(new PointCloudType());
  map_scans_since_fragment_ = 0;

  const auto enqueue_status = enqueueCloudWrite(
      path, fragment, true, "map fragment", force);
  if (enqueue_status != CloudWriteEnqueueStatus::Queued) {
    point_map_ = fragment;
    if (enqueue_status == CloudWriteEnqueueStatus::Unavailable &&
        !map_writer_failure_logged_) {
      map_writer_failure_logged_ = true;
      LOG(ERROR) << RED
                 << " ---> [SuperLIO]: unable to enqueue map fragment; "
                 << "retaining points in RAM. Stop mapping and inspect disk health."
                 << RESET;
    }
    return false;
  }

  ++pcd_index_;
  map_fragment_paths_.push_back(path);
  LOG(INFO) << GREEN
            << " ---> [SuperLIO]: queued map fragment. index=" << pcd_index_
            << " raw_points=" << fragment->size()
            << " reason="
            << (force ? "shutdown" : point_limit_reached ? "point_limit" : "scan_limit")
            << " path=" << path.string() << RESET;
  return true;
}


void SuperLIO::caceData(){
  if(!g_save_map || !observation_valid_) return;
  auto state = kf_->GetNavState();
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  if(g_if_filter){
    pcl::transformPointCloud(*ds_undistort_, *world_pc_, transformation);
  }else{
    pcl::transformPointCloud(*scan_undistort_full_, *world_pc_, transformation);
  }

  if(!world_pc_->empty()){
    *point_map_ += *world_pc_;
    map_scans_since_fragment_++;
    maybeCacheLoopKeyFrame(SE3(state.R.R_, state.p), state.timestamp);

    updateMapPreview(state.timestamp);
    flushMapFragment(false);
  }
}


void SuperLIO::updateMapPreview(const double timestamp){
  if (!map_preview_ || !map_preview_pending_ || !world_pc_ || world_pc_->empty()) {
    return;
  }

  CloudPtr preview_addition = world_pc_;
  if (g_loop_online_enable) {
    preview_addition.reset(new PointCloudType());
    pcl::transformPointCloud(
        *world_pc_, *preview_addition,
        se3_to_matrix4f(online_loop_map_to_odom_));
    normalize_cloud_layout(*preview_addition);
  }
  *map_preview_pending_ += *preview_addition;
  map_preview_scan_count_++;
  if (map_preview_scan_count_ < g_map_preview_publish_interval) {
    return;
  }
  map_preview_scan_count_ = 0;

  CloudPtr merged(new PointCloudType());
  merged->reserve(map_preview_->size() + map_preview_pending_->size());
  *merged += *map_preview_;
  *merged += *map_preview_pending_;
  map_preview_pending_->clear();

  CloudPtr filtered(new PointCloudType());
  auto leaf_size = std::max(map_preview_effective_leaf_size_, 0.01f);
  const auto max_points = static_cast<std::size_t>(g_map_preview_max_points);

  if (g_if_filter) {
    for (int attempt = 0; attempt < 6; ++attempt) {
      make_map_pcd_cloud(merged, *filtered, leaf_size, "bounded preview", true);
      if (filtered->size() <= max_points) {
        break;
      }
      const double ratio = static_cast<double>(filtered->size()) /
                           static_cast<double>(max_points);
      leaf_size *= static_cast<float>(std::max(1.25, std::sqrt(ratio) * 1.05));
    }
  } else {
    *filtered = *merged;
  }

  if (filtered->size() > max_points) {
    CloudPtr limited(new PointCloudType());
    limited->reserve(max_points);
    const std::size_t stride =
      std::max<std::size_t>(1, (filtered->size() + max_points - 1) / max_points);
    for (std::size_t index = 0;
         index < filtered->size() && limited->size() < max_points;
         index += stride) {
      limited->push_back(filtered->points[index]);
    }
    normalize_cloud_layout(*limited);
    filtered = limited;
  }

  if (leaf_size > map_preview_effective_leaf_size_ + 1e-6f) {
    LOG(INFO) << GREEN << " ---> [SuperLIO]: bounded map preview leaf increased from "
              << map_preview_effective_leaf_size_ << " to " << leaf_size
              << ", points=" << filtered->size() << RESET;
  }
  map_preview_effective_leaf_size_ = leaf_size;
  map_preview_ = filtered;

  if (!map_preview_->empty()) {
    data_wrapper_->pub_map_accumulated(map_preview_, timestamp);
  }
}


bool SuperLIO::ProcessCaceMap(){
  const std::string output_map_name = g_save_map_dir + "/" + g_map_name;
  if (map_fragment_paths_.empty()) {
    LOG(ERROR) << RED << " ---> No map fragments are available for final merge." << RESET;
    return false;
  }

  LOG(INFO) << YELLOW
            << " ---> Incrementally merging map fragments. count="
            << map_fragment_paths_.size()
            << " work_dir=" << map_fragment_dir_.string() << RESET;

  CloudPtr filtered_map(new PointCloudType());
  CloudPtr fragment_batch(new PointCloudType());
  const std::size_t batch_limit =
      g_map_max_points_in_memory > 0
      ? static_cast<std::size_t>(g_map_max_points_in_memory)
      : 2000000U;
  auto flush_fragment_batch = [&]() {
    if (fragment_batch->empty()) {
      return true;
    }
    if (!merge_cloud_incrementally(
            filtered_map,
            fragment_batch,
            g_map_ds_size,
            "bounded final merge")) {
      return false;
    }
    fragment_batch.reset(new PointCloudType());
    return true;
  };
  std::size_t merged_count = 0;
  for (const auto& path : map_fragment_paths_) {
    CloudPtr fragment(new PointCloudType());
    if (pcl::io::loadPCDFile<PointType>(path.string(), *fragment) != 0) {
      LOG(ERROR) << RED << " ---> Failed to load map fragment: "
                 << path.string() << RESET;
      return false;
    }
    *fragment_batch += *fragment;
    if (fragment_batch->size() >= batch_limit &&
        !flush_fragment_batch()) {
      return false;
    }
    ++merged_count;
    if ((merged_count % 10) == 0 ||
        merged_count == map_fragment_paths_.size()) {
      LOG(INFO) << GREEN
                << " ---> [SuperLIO]: bounded final merge progress="
                << merged_count << "/" << map_fragment_paths_.size()
                << " retained_points=" << filtered_map->size()
                << " pending_points=" << fragment_batch->size() << RESET;
    }
  }
  if (!flush_fragment_batch()) {
    return false;
  }

  if (!save_pcd_binary_safe(output_map_name, *filtered_map)) {
    return false;
  }
  LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << RESET;
  LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map->size() << RESET;
  return true;
}


CloudPtr SuperLIO::loadLoopKeyFrameCloud(const std::size_t index) const
{
  CloudPtr cloud(new PointCloudType());
  if (index >= loop_keyframes_.size()) {
    return cloud;
  }
  const auto& keyframe = loop_keyframes_[index];
  if (keyframe.cloud_body && !keyframe.cloud_body->empty()) {
    return keyframe.cloud_body;
  }
  if (keyframe.cloud_path.empty()) {
    return cloud;
  }
  if (pcl::io::loadPCDFile<PointType>(
          keyframe.cloud_path.string(), *cloud) != 0) {
    LOG(ERROR) << RED << " ---> Failed to load loop keyframe: "
               << keyframe.cloud_path.string() << RESET;
    cloud->clear();
  }
  return cloud;
}


void SuperLIO::cleanupMapPersistenceFiles()
{
  if (!g_map_cleanup_work_files || map_work_dir_.empty()) {
    return;
  }
  std::error_code ec;
  std::filesystem::remove_all(map_work_dir_, ec);
  if (ec) {
    LOG(WARNING) << YELLOW << " ---> Failed to clean mapping work directory: "
                 << map_work_dir_.string()
                 << " error: " << ec.message() << RESET;
  } else {
    LOG(INFO) << GREEN << " ---> Mapping work files cleaned: "
              << map_work_dir_.string() << RESET;
  }
}


bool SuperLIO::saveFrontendKeyFrameTrajectory() const
{
  const std::filesystem::path output_path =
      std::filesystem::path(g_save_map_dir) /
      "frontend_keyframe_trajectory.txt";
  const std::filesystem::path temporary_path =
      output_path.string() + ".tmp";
  try {
    std::ofstream output(
        temporary_path,
        std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
      return false;
    }

    output << "NAV_LIO_FRONTEND_KEYFRAME_TRAJECTORY_V2\n";
    output << loop_keyframes_.size() << "\n";
    output << "# keyframe_index scan_index timestamp cumulative_path_m "
              "raw_tx raw_ty raw_tz raw_qx raw_qy raw_qz raw_qw "
              "rot_eig_min rot_eig_mid rot_eig_max rot_min_max_ratio "
              "trans_eig_min trans_eig_mid trans_eig_max "
              "trans_min_max_ratio raw_yaw_information_ratio "
              "conditional_yaw_information_ratio "
              "translation_information_ratio "
              "effective_matches points\n";
    output << std::setprecision(17);

    double cumulative_path = 0.0;
    for (std::size_t index = 0; index < loop_keyframes_.size(); ++index) {
      const auto& keyframe = loop_keyframes_[index];
      if (index > 0) {
        cumulative_path += static_cast<double>(
            (keyframe.pose.t_ - loop_keyframes_[index - 1].pose.t_).norm());
      }

      Eigen::Quaternionf quaternion(keyframe.pose.R_);
      quaternion.normalize();
      const auto information_ratio = [](const Eigen::Vector3d& eigenvalues) {
        if (!eigenvalues.allFinite() || eigenvalues.z() <= 1.0e-12) {
          return 0.0;
        }
        return std::clamp(
            std::max(0.0, eigenvalues.x()) / eigenvalues.z(),
            0.0,
            1.0);
      };

      output << index << " "
             << keyframe.scan_index << " "
             << keyframe.timestamp << " "
             << cumulative_path << " "
             << keyframe.pose.t_.x() << " "
             << keyframe.pose.t_.y() << " "
             << keyframe.pose.t_.z() << " "
             << quaternion.x() << " "
             << quaternion.y() << " "
             << quaternion.z() << " "
             << quaternion.w() << " "
             << keyframe.lidar_rotation_information_eigenvalues.x() << " "
             << keyframe.lidar_rotation_information_eigenvalues.y() << " "
             << keyframe.lidar_rotation_information_eigenvalues.z() << " "
             << information_ratio(
                    keyframe.lidar_rotation_information_eigenvalues) << " "
             << keyframe.lidar_translation_information_eigenvalues.x() << " "
             << keyframe.lidar_translation_information_eigenvalues.y() << " "
             << keyframe.lidar_translation_information_eigenvalues.z() << " "
             << information_ratio(
                    keyframe.lidar_translation_information_eigenvalues) << " "
             << keyframe.lidar_yaw_information_ratio << " "
             << keyframe.lidar_conditional_yaw_information_ratio << " "
             << keyframe.lidar_translation_information_ratio << " "
             << keyframe.effective_match_count << " "
             << keyframe.point_count << "\n";
    }

    output.flush();
    if (!output.good()) {
      output.close();
      std::filesystem::remove(temporary_path);
      return false;
    }
    output.close();

    std::error_code rename_error;
    std::filesystem::rename(
        temporary_path, output_path, rename_error);
    if (rename_error) {
      std::filesystem::remove(temporary_path);
      return false;
    }
    return true;
  } catch (...) {
    std::error_code cleanup_error;
    std::filesystem::remove(temporary_path, cleanup_error);
    return false;
  }
}

void SuperLIO::maybeCacheLoopKeyFrame(const SE3& pose, double timestamp)
{
  if (!g_loop_closure_enable || !ds_undistort_ || ds_undistort_->empty()) {
    return;
  }

  const V3 current_pos = pose.t_;
  if (std::isfinite(last_loop_keyframe_pos_.x())) {
    const auto distance = (current_pos - last_loop_keyframe_pos_).norm();
    if (distance < g_loop_keyframe_min_distance) {
      return;
    }
  }

  LoopKeyFrame keyframe;
  keyframe.pose = pose;
  keyframe.timestamp = timestamp;
  keyframe.scan_index = processed_scan_index_;
  keyframe.effective_match_count = effective_match_count_;
  keyframe.lidar_rotation_information_eigenvalues =
      latest_lidar_rotation_information_eigenvalues_;
  keyframe.lidar_translation_information_eigenvalues =
      latest_lidar_translation_information_eigenvalues_;
  keyframe.lidar_yaw_information_ratio =
      latest_lidar_yaw_information_ratio_;
  keyframe.lidar_conditional_yaw_information_ratio =
      latest_lidar_conditional_yaw_information_ratio_;
  keyframe.lidar_translation_information_ratio =
      latest_lidar_translation_information_ratio_;
  keyframe.cloud_body.reset(new PointCloudType());
  *keyframe.cloud_body = *ds_undistort_;
  normalize_cloud_layout(*keyframe.cloud_body);
  keyframe.point_count = keyframe.cloud_body->size();
  if (g_loop_online_enable) {
    keyframe.online_cloud_body.reset(new PointCloudType());
    pcl::VoxelGrid<PointType> online_filter;
    online_filter.setInputCloud(keyframe.cloud_body);
    online_filter.setLeafSize(
        g_loop_online_voxel_size,
        g_loop_online_voxel_size,
        g_loop_online_voxel_size);
    online_filter.filter(*keyframe.online_cloud_body);
    normalize_cloud_layout(*keyframe.online_cloud_body);
    // A pathological dense scan must not turn an online background task into
    // unbounded Jetson memory. Keep spatially uniform voxel output and apply a
    // deterministic stride only above the generous per-frame ceiling.
    constexpr std::size_t kMaximumOnlinePointsPerFrame = 4000;
    if (keyframe.online_cloud_body->size() >
        kMaximumOnlinePointsPerFrame) {
      CloudPtr limited(new PointCloudType());
      limited->reserve(kMaximumOnlinePointsPerFrame);
      const std::size_t stride =
          (keyframe.online_cloud_body->size() +
           kMaximumOnlinePointsPerFrame - 1) /
          kMaximumOnlinePointsPerFrame;
      for (std::size_t i = 0;
           i < keyframe.online_cloud_body->size() &&
           limited->size() < kMaximumOnlinePointsPerFrame;
           i += stride) {
        limited->push_back(keyframe.online_cloud_body->points[i]);
      }
      normalize_cloud_layout(*limited);
      keyframe.online_cloud_body = limited;
    }
  }
  if (g_loop_persist_keyframes && map_persistence_enabled_) {
    std::ostringstream filename;
    filename << "keyframe_" << std::setw(6) << std::setfill('0')
             << loop_keyframes_.size() << ".pcd";
    const auto path = loop_keyframe_dir_ / filename.str();
    const auto enqueue_status = enqueueCloudWrite(
            path,
            keyframe.cloud_body,
            false,
            "loop keyframe",
            false);
    if (enqueue_status == CloudWriteEnqueueStatus::Queued) {
      keyframe.cloud_path = path;
      keyframe.cloud_body.reset();
    } else if (enqueue_status == CloudWriteEnqueueStatus::Unavailable &&
               !map_writer_failure_logged_) {
      map_writer_failure_logged_ = true;
      LOG(ERROR) << RED
                 << " ---> [SuperLIO]: unable to persist loop keyframe; "
                 << "retaining subsequent keyframes in RAM."
                 << RESET;
    }
  }
  std::size_t keyframe_index = 0;
  {
    std::lock_guard<std::mutex> lock(loop_keyframes_mutex_);
    keyframe_index = loop_keyframes_.size();
    loop_keyframes_.push_back(keyframe);
  }
  last_loop_keyframe_pos_ = current_pos;
  scheduleOnlineLoopTask(keyframe_index);
}

bool SuperLIO::initializeOnlineLoopWorker()
{
  if (!g_loop_online_enable) {
    return true;
  }
  if (!g_loop_closure_enable || !g_save_map) {
    LOG(ERROR) << RED
               << " ---> [OnlineLoop]: online closure requires mapping and loop closure."
               << RESET;
    return false;
  }
  online_loop_stopping_ = false;
  online_loop_tasks_.clear();
  online_loop_results_.clear();
  online_loop_edges_.clear();
  online_loop_pending_ = OnlineLoopPending{};
  online_loop_last_scheduled_index_ = 0;
  online_loop_dropped_tasks_ = 0;
  online_loop_accepted_count_ = 0;
  online_loop_rejected_count_ = 0;
  online_loop_map_to_odom_ = SE3();
  online_loop_target_map_to_odom_ = SE3();
  online_loop_thread_ = std::thread(&SuperLIO::onlineLoopWorker, this);
  LOG(INFO) << GREEN
            << " ---> [OnlineLoop]: background worker started. interval_keyframes="
            << g_loop_online_interval_keyframes
            << " queue_capacity=" << g_loop_online_queue_capacity
            << " candidate_limit=" << g_loop_online_candidate_limit
            << " task_budget=" << g_loop_online_max_task_seconds << "s"
            << RESET;
  return true;
}

void SuperLIO::scheduleOnlineLoopTask(const std::size_t later_index)
{
  if (!g_loop_online_enable || !online_loop_thread_.joinable()) {
    return;
  }
  const std::size_t minimum_index = static_cast<std::size_t>(
      std::max(g_loop_keyframe_min_gap,
               2 * g_loop_online_local_window_size + 5));
  if (later_index < minimum_index ||
      later_index < online_loop_last_scheduled_index_ +
          static_cast<std::size_t>(g_loop_online_interval_keyframes)) {
    return;
  }

  std::lock_guard<std::mutex> lock(online_loop_mutex_);
  if (online_loop_stopping_) {
    return;
  }
  if (online_loop_tasks_.size() >=
      static_cast<std::size_t>(g_loop_online_queue_capacity)) {
    // Keep the broad history already queued and coalesce only the newest
    // pending request. All keyframes remain available to the save-time pass.
    online_loop_tasks_.back() = later_index;
    ++online_loop_dropped_tasks_;
    if ((online_loop_dropped_tasks_ % 10U) == 1U) {
      LOG(WARNING) << YELLOW
                   << " ---> [OnlineLoop]: background queue full; coalesced newest task."
                   << " capacity=" << g_loop_online_queue_capacity
                   << " coalesced=" << online_loop_dropped_tasks_
                   << " latest=" << later_index << RESET;
    }
  } else {
    online_loop_tasks_.push_back(later_index);
  }
  online_loop_last_scheduled_index_ = later_index;
  online_loop_cv_.notify_one();
}

void SuperLIO::stopOnlineLoopWorker()
{
  if (!online_loop_thread_.joinable()) {
    return;
  }
  std::size_t skipped = 0;
  std::size_t final_index = 0;
  bool enqueue_final = false;
  {
    std::lock_guard<std::mutex> lock(loop_keyframes_mutex_);
    if (!loop_keyframes_.empty()) {
      final_index = loop_keyframes_.size() - 1U;
      const std::size_t minimum_index = static_cast<std::size_t>(
          std::max(g_loop_keyframe_min_gap,
                   2 * g_loop_online_local_window_size + 5));
      enqueue_final = final_index >= minimum_index;
    }
  }
  {
    std::lock_guard<std::mutex> lock(online_loop_mutex_);
    online_loop_stopping_ = true;
    skipped = online_loop_tasks_.size();
    online_loop_tasks_.clear();
    // Do not lose a closure just because the route ended between periodic
    // checks. Drain one final newest task; the worker's per-task deadline
    // remains the generous hard bound.
    if (enqueue_final) {
      online_loop_tasks_.push_back(final_index);
    }
  }
  online_loop_cv_.notify_all();
  online_loop_thread_.join();
  LOG(INFO) << GREEN
            << " ---> [OnlineLoop]: worker stopped. accepted="
            << online_loop_accepted_count_
            << " rejected=" << online_loop_rejected_count_
            << " coalesced=" << online_loop_dropped_tasks_
            << " stale_tasks_skipped_at_stop=" << skipped
            << " final_task_drained=" << enqueue_final
            << " final_index=" << final_index << RESET;
}

void SuperLIO::onlineLoopWorker()
{
  struct Candidate {
    int earlier = -1;
    float xy_distance = std::numeric_limits<float>::max();
    float path_length = 0.0f;
    float score = std::numeric_limits<float>::max();
  };
  struct StrictCandidateHypothesis {
    int earlier = -1;
    SE3 correction;
    LoopGeometryVerification geometry;
    float fitness = std::numeric_limits<float>::max();
  };

  const auto yaw_of = [](const M3& rotation) {
    return std::atan2(rotation(1, 0), rotation(0, 0));
  };
  const auto wrap_angle = [](float angle) {
    while (angle > static_cast<float>(M_PI)) {
      angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle < -static_cast<float>(M_PI)) {
      angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
  };

  while (true) {
    std::size_t requested_later = 0;
    {
      std::unique_lock<std::mutex> lock(online_loop_mutex_);
      online_loop_cv_.wait(lock, [this]() {
        return online_loop_stopping_ || !online_loop_tasks_.empty();
      });
      if (online_loop_stopping_ && online_loop_tasks_.empty()) {
        break;
      }
      requested_later = online_loop_tasks_.front();
      online_loop_tasks_.pop_front();
    }

    std::vector<OnlineLoopFrame> frames;
    {
      std::lock_guard<std::mutex> lock(loop_keyframes_mutex_);
      if (requested_later >= loop_keyframes_.size()) {
        continue;
      }
      frames.reserve(requested_later + 1);
      for (std::size_t index = 0;
           index <= requested_later;
           ++index) {
        OnlineLoopFrame frame;
        frame.raw_pose = loop_keyframes_[index].pose;
        frame.timestamp = loop_keyframes_[index].timestamp;
        frame.cloud_body = loop_keyframes_[index].online_cloud_body;
        frames.push_back(std::move(frame));
      }
    }
    if (frames.empty()) {
      continue;
    }
    const int later = static_cast<int>(frames.size()) - 1;
    const int window = g_loop_online_local_window_size;
    const int minimum_gap = std::max(
        g_loop_keyframe_min_gap, 2 * window + 5);
    if (later <= minimum_gap) {
      continue;
    }

    std::vector<float> cumulative_path(frames.size(), 0.0f);
    for (std::size_t i = 1; i < frames.size(); ++i) {
      cumulative_path[i] = cumulative_path[i - 1] +
          (frames[i].raw_pose.t_ - frames[i - 1].raw_pose.t_).norm();
    }

    const V3 later_position = frames[later].raw_pose.t_;
    std::vector<Candidate> candidates;
    candidates.reserve(32);
    for (int earlier = 0;
         earlier <= later - minimum_gap - window;
         ++earlier) {
      if (!frames[earlier].cloud_body ||
          frames[earlier].cloud_body->empty()) {
        continue;
      }
      const double sensor_time =
          frames[later].timestamp - frames[earlier].timestamp;
      if (!std::isfinite(sensor_time) ||
          sensor_time < g_loop_internal_min_sensor_time_seconds) {
        continue;
      }
      const V3 delta = later_position - frames[earlier].raw_pose.t_;
      const float xy_distance = std::hypot(delta.x(), delta.y());
      if (xy_distance > g_loop_online_search_radius) {
        continue;
      }
      const float path_length =
          cumulative_path[later] - cumulative_path[earlier];
      const float adaptive_vertical_allowance = std::min(
          3.0f, 0.5f + 0.01f * std::max(0.0f, path_length));
      if (std::abs(delta.z()) > adaptive_vertical_allowance) {
        continue;
      }
      bool duplicate_of_accepted = false;
      for (const auto& edge : online_loop_edges_) {
        if (std::abs(edge.from - earlier) <= 2 * window &&
            std::abs(edge.to - later) <= 2 * window) {
          duplicate_of_accepted = true;
          break;
        }
      }
      if (duplicate_of_accepted) {
        continue;
      }
      const float yaw_difference = std::abs(wrap_angle(
          yaw_of(frames[later].raw_pose.R_) -
          yaw_of(frames[earlier].raw_pose.R_)));
      Candidate candidate;
      candidate.earlier = earlier;
      candidate.xy_distance = xy_distance;
      candidate.path_length = path_length;
      // Distance remains primary, but a same-direction corridor is evaluated
      // before a perpendicular crossing when both revisit the same area.
      candidate.score = xy_distance + 0.5f * yaw_difference;
      candidates.push_back(candidate);
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& lhs, const Candidate& rhs) {
                return lhs.score < rhs.score;
              });
    if (candidates.size() >
        static_cast<std::size_t>(g_loop_online_candidate_limit)) {
      candidates.resize(
          static_cast<std::size_t>(g_loop_online_candidate_limit));
    }

    const auto build_window = [&frames, later](
        const int begin,
        const int end,
        const V3& anchor) {
      CloudPtr merged(new PointCloudType());
      constexpr float kMaximumRadius = 32.0f;
      const float radius_squared = kMaximumRadius * kMaximumRadius;
      for (int index = std::max(0, begin);
           index <= std::min(later, end);
           ++index) {
        if (!frames[index].cloud_body ||
            frames[index].cloud_body->empty()) {
          continue;
        }
        CloudPtr world(new PointCloudType());
        pcl::transformPointCloud(
            *frames[index].cloud_body, *world,
            se3_to_matrix4f(frames[index].raw_pose));
        for (const auto& point : world->points) {
          const float dx = point.x - static_cast<float>(anchor.x());
          const float dy = point.y - static_cast<float>(anchor.y());
          if (dx * dx + dy * dy <= radius_squared &&
              std::abs(point.z - static_cast<float>(anchor.z())) <= 8.0f) {
            merged->push_back(point);
          }
        }
      }
      normalize_cloud_layout(*merged);
      if (merged->empty()) {
        return merged;
      }
      CloudPtr filtered(new PointCloudType());
      pcl::VoxelGrid<PointType> voxel;
      voxel.setInputCloud(merged);
      voxel.setLeafSize(
          g_loop_online_voxel_size,
          g_loop_online_voxel_size,
          g_loop_online_voxel_size);
      voxel.filter(*filtered);
      normalize_cloud_layout(*filtered);
      constexpr std::size_t kMaximumWindowPoints = 50000;
      if (filtered->size() <= kMaximumWindowPoints) {
        return filtered;
      }
      CloudPtr limited(new PointCloudType());
      limited->reserve(kMaximumWindowPoints);
      const std::size_t stride =
          (filtered->size() + kMaximumWindowPoints - 1) /
          kMaximumWindowPoints;
      for (std::size_t i = 0;
           i < filtered->size() && limited->size() < kMaximumWindowPoints;
           i += stride) {
        limited->push_back(filtered->points[i]);
      }
      normalize_cloud_layout(*limited);
      return limited;
    };

    const auto task_started = std::chrono::steady_clock::now();
    bool found = false;
    Candidate best_candidate;
    SE3 best_correction;
    LoopGeometryVerification best_geometry;
    float best_fitness = std::numeric_limits<float>::max();
    std::size_t valid_target_count = 0;
    std::size_t seed_attempt_count = 0;
    std::size_t converged_count = 0;
    std::size_t fitness_pass_count = 0;
    std::size_t rotation_pass_count = 0;
    std::size_t geometry_pass_count = 0;
    std::size_t translation_pass_count = 0;
    std::size_t support_pass_count = 0;
    float diagnostic_best_overlap = 0.0f;
    float diagnostic_best_rmse = std::numeric_limits<float>::max();
    float diagnostic_best_anchor_translation = 0.0f;
    bool diagnostic_main_valid = false;
    bool diagnostic_structural_available = false;
    bool diagnostic_structural_valid = false;
    bool diagnostic_large_correction = false;
    int diagnostic_supported_blocks = 0;
    float diagnostic_block_ratio = 0.0f;
    float diagnostic_span = 0.0f;
    float diagnostic_minor_span = 0.0f;
    float diagnostic_structural_overlap = 0.0f;
    std::vector<StrictCandidateHypothesis> strict_candidate_hypotheses;
    strict_candidate_hypotheses.reserve(candidates.size());
    // The later window is identical for every earlier candidate. Build it
    // once so a generous candidate/time budget does not multiply frontend-
    // independent CPU and memory traffic on Jetson.
    const CloudPtr source = candidates.empty()
        ? CloudPtr(new PointCloudType())
        : build_window(later - 2 * window, later, later_position);
    for (const auto& candidate : candidates) {
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - task_started).count();
      if (elapsed >= g_loop_online_max_task_seconds) {
        break;
      }
      const V3 earlier_position =
          frames[candidate.earlier].raw_pose.t_;
      const int target_begin = std::max(0, candidate.earlier - window);
      const int target_end = std::min(
          later,
          std::max(candidate.earlier + window,
                   target_begin + 2 * window));
      const CloudPtr target = build_window(
          target_begin,
          target_end,
          earlier_position);
      if (!source || !target || source->size() < 500 ||
          target->size() < 500) {
        continue;
      }
      ++valid_target_count;
      bool candidate_found = false;
      StrictCandidateHypothesis candidate_hypothesis;
      candidate_hypothesis.earlier = candidate.earlier;

      std::array<Eigen::Matrix4f, 3> seeds;
      seeds[0] = Eigen::Matrix4f::Identity();
      seeds[1] = Eigen::Matrix4f::Identity();
      seeds[1].block<3, 1>(0, 3) =
          (earlier_position - later_position).cast<float>();
      seeds[2] = Eigen::Matrix4f::Identity();
      seeds[2](0, 3) = static_cast<float>(
          earlier_position.x() - later_position.x());
      seeds[2](1, 3) = static_cast<float>(
          earlier_position.y() - later_position.y());

      for (const auto& seed : seeds) {
        const double seed_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - task_started).count();
        if (seed_elapsed >= g_loop_online_max_task_seconds) {
          break;
        }
        ++seed_attempt_count;
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        gicp.setMaximumIterations(32);
        gicp.setMaxCorrespondenceDistance(g_loop_icp_max_distance);
        gicp.setTransformationEpsilon(1.0e-4);
        gicp.setEuclideanFitnessEpsilon(1.0e-4);
        gicp.setInputSource(source);
        gicp.setInputTarget(target);
        PointCloudType aligned;
        gicp.align(aligned, seed);
        if (!gicp.hasConverged()) {
          continue;
        }
        ++converged_count;
        Eigen::Matrix4f correction = gicp.getFinalTransformation();
        const float fitness = static_cast<float>(
            gicp.getFitnessScore(g_loop_icp_max_distance));
        if (!correction.allFinite() || !std::isfinite(fitness) ||
            fitness > g_loop_icp_score_threshold) {
          continue;
        }
        ++fitness_pass_count;
        const LoopRotationMetrics full_rotation =
            evaluate_loop_rotation(correction.block<3, 3>(0, 0));
        if (!loop_rotation_is_plausible(
                full_rotation,
                adaptive_loop_yaw_limit_deg(candidate.path_length))) {
          continue;
        }
        ++rotation_pass_count;
        SE3 projected = project_gravity_aligned_loop_correction(
            correction, later_position);
        correction = se3_to_matrix4f(projected);
        if (g_loop_ground_z_refinement_enable) {
          const CloudPtr source_ground =
              extract_loop_ground_envelope(source, later_position);
          const CloudPtr target_ground =
              extract_loop_ground_envelope(target, earlier_position);
          refine_loop_ground_z(source_ground, target_ground, correction);
          projected = SE3(correction);
        }
        const LoopGeometryVerification geometry = verify_loop_geometry(
            source, target, correction, later_position);
        if (geometry.symmetric_overlap > diagnostic_best_overlap ||
            (std::abs(geometry.symmetric_overlap -
                      diagnostic_best_overlap) < 1.0e-4f &&
             geometry.symmetric_trimmed_rmse < diagnostic_best_rmse)) {
          diagnostic_best_overlap = geometry.symmetric_overlap;
          diagnostic_best_rmse = geometry.symmetric_trimmed_rmse;
          diagnostic_best_anchor_translation =
              geometry.anchor_translation;
          diagnostic_main_valid = geometry.main_valid;
          diagnostic_structural_available =
              geometry.structural_evidence_available;
          diagnostic_structural_valid = geometry.structural_valid;
          diagnostic_large_correction = geometry.large_correction;
          diagnostic_supported_blocks = std::min(
              geometry.source_to_target.supported_blocks,
              geometry.target_to_source.supported_blocks);
          diagnostic_block_ratio = std::min(
              geometry.source_to_target.supported_block_ratio,
              geometry.target_to_source.supported_block_ratio);
          diagnostic_span = std::min(
              geometry.source_to_target.support_span,
              geometry.target_to_source.support_span);
          diagnostic_minor_span = std::min(
              geometry.source_to_target.support_minor_span,
              geometry.target_to_source.support_minor_span);
          diagnostic_structural_overlap =
              geometry.structural_symmetric_overlap;
        }
        if (!geometry.valid) {
          continue;
        }
        ++geometry_pass_count;
        // A long route may require more than a fixed 3 m correction. Scale
        // the allowance with accumulated path while retaining an independent
        // 3 m raw-Z discovery cap above, so real multi-floor revisits do not
        // enter merely because the route is long.
        const float adaptive_translation_allowance = std::min(
            g_loop_online_search_radius,
            1.0f + 0.01f * std::max(0.0f, candidate.path_length));
        if (geometry.anchor_translation > adaptive_translation_allowance) {
          continue;
        }
        ++translation_pass_count;
        const InternalReciprocalMatchVector reciprocal =
            collect_internal_reciprocal_matches(
                source, target, correction);
        const InternalSupportVoxelSummary support =
            analyze_internal_support_voxels(reciprocal);
        const InternalResidualModeAnalysis modes =
            analyze_internal_residual_modes(reciprocal);
        if (!support.strict_valid || modes.spatially_overlapping) {
          continue;
        }
        ++support_pass_count;
        if (!candidate_found ||
            geometry.confidence >
                candidate_hypothesis.geometry.confidence ||
            (std::abs(geometry.confidence -
                      candidate_hypothesis.geometry.confidence) <
                 1.0e-4f &&
             fitness < candidate_hypothesis.fitness)) {
          candidate_found = true;
          candidate_hypothesis.correction = projected;
          candidate_hypothesis.geometry = geometry;
          candidate_hypothesis.fitness = fitness;
        }
        if (!found || geometry.confidence > best_geometry.confidence ||
            (std::abs(geometry.confidence - best_geometry.confidence) <
                 1.0e-4f && fitness < best_fitness)) {
          found = true;
          best_candidate = candidate;
          best_correction = projected;
          best_geometry = geometry;
          best_fitness = fitness;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (candidate_found) {
        strict_candidate_hypotheses.push_back(
            std::move(candidate_hypothesis));
      }
    }

    if (!found) {
      ++online_loop_rejected_count_;
      const double task_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - task_started).count();
      if (!candidates.empty() ||
          (online_loop_rejected_count_ % 25U) == 1U) {
        LOG(INFO) << GREEN
                  << " ---> [OnlineLoop]: task finished without a strict loop."
                  << " later=" << later
                  << " candidates=" << candidates.size()
                  << " task_seconds=" << task_seconds
                  << " deadline_reached="
                  << (task_seconds >= g_loop_online_max_task_seconds)
                  << " valid_targets=" << valid_target_count
                  << " seed_attempts=" << seed_attempt_count
                  << " converged=" << converged_count
                  << " fitness_pass=" << fitness_pass_count
                  << " rotation_pass=" << rotation_pass_count
                  << " geometry_pass=" << geometry_pass_count
                  << " translation_pass=" << translation_pass_count
                  << " support_pass=" << support_pass_count
                  << " best_overlap=" << diagnostic_best_overlap
                  << " best_rmse=" << diagnostic_best_rmse
                  << " best_anchor_translation="
                  << diagnostic_best_anchor_translation
                  << " main_valid=" << diagnostic_main_valid
                  << " supported_blocks="
                  << diagnostic_supported_blocks
                  << " block_ratio=" << diagnostic_block_ratio
                  << " span=" << diagnostic_span
                  << " minor_span=" << diagnostic_minor_span
                  << " structural_available="
                  << diagnostic_structural_available
                  << " structural_valid="
                  << diagnostic_structural_valid
                  << " structural_overlap="
                  << diagnostic_structural_overlap
                  << " large_correction="
                  << diagnostic_large_correction
                  << " queue_capacity=" << g_loop_online_queue_capacity
                  << RESET;
      }
      if (online_loop_pending_.valid &&
          later - online_loop_pending_.later >
              4 * g_loop_online_interval_keyframes) {
        online_loop_pending_ = OnlineLoopPending{};
      }
      continue;
    }

    int same_task_consensus_count = 0;
    for (const auto& hypothesis : strict_candidate_hypotheses) {
      if (std::abs(hypothesis.earlier - best_candidate.earlier) >
          2 * window) {
        continue;
      }
      const V3 best_anchor = best_correction * later_position;
      const V3 hypothesis_anchor =
          hypothesis.correction * later_position;
      const float translation_difference =
          (best_anchor - hypothesis_anchor).norm();
      const float yaw_difference_deg = std::abs(wrap_angle(
          yaw_of(best_correction.R_) -
          yaw_of(hypothesis.correction.R_))) *
          180.0f / static_cast<float>(M_PI);
      if (translation_difference <=
              g_loop_online_confirmation_translation &&
          yaw_difference_deg <= g_loop_online_confirmation_yaw_deg) {
        ++same_task_consensus_count;
      }
    }
    constexpr int kSameTaskStrictCandidateQuorum = 3;
    const bool same_task_three_vote =
        same_task_consensus_count >= kSameTaskStrictCandidateQuorum;

    bool confirmation_agrees = false;
    if (online_loop_pending_.valid &&
        std::abs(online_loop_pending_.earlier -
                 best_candidate.earlier) <= 2 * window &&
        later > online_loop_pending_.later) {
      const V3 pending_anchor =
          online_loop_pending_.correction * later_position;
      const V3 current_anchor = best_correction * later_position;
      const float translation_difference =
          (pending_anchor - current_anchor).norm();
      const float yaw_difference_deg = std::abs(wrap_angle(
          yaw_of(online_loop_pending_.correction.R_) -
          yaw_of(best_correction.R_))) *
          180.0f / static_cast<float>(M_PI);
      confirmation_agrees =
          translation_difference <=
              g_loop_online_confirmation_translation &&
          yaw_difference_deg <= g_loop_online_confirmation_yaw_deg;
    }
    if (same_task_three_vote) {
      online_loop_pending_.valid = true;
      online_loop_pending_.earlier = best_candidate.earlier;
      online_loop_pending_.later = later;
      online_loop_pending_.correction = best_correction;
      online_loop_pending_.confirmations =
          g_loop_online_min_confirmations;
    } else if (confirmation_agrees) {
      ++online_loop_pending_.confirmations;
      online_loop_pending_.earlier = best_candidate.earlier;
      online_loop_pending_.later = later;
      online_loop_pending_.correction = best_correction;
    } else {
      online_loop_pending_.valid = true;
      online_loop_pending_.earlier = best_candidate.earlier;
      online_loop_pending_.later = later;
      online_loop_pending_.correction = best_correction;
      online_loop_pending_.confirmations = 1;
    }
    if (online_loop_pending_.confirmations <
        g_loop_online_min_confirmations) {
      LOG(INFO) << GREEN
                << " ---> [OnlineLoop]: strict candidate pending confirmation."
                << " earlier=" << best_candidate.earlier
                << " later=" << later
                << " confirmations="
                << online_loop_pending_.confirmations << "/"
                << g_loop_online_min_confirmations
                << " same_task_strict_candidates="
                << strict_candidate_hypotheses.size()
                << " same_task_consensus="
                << same_task_consensus_count << "/"
                << kSameTaskStrictCandidateQuorum
                << " overlap=" << best_geometry.symmetric_overlap
                << " rmse=" << best_geometry.symmetric_trimmed_rmse
                << RESET;
      continue;
    }

    OnlineLoopEdge accepted_edge;
    accepted_edge.from = best_candidate.earlier;
    accepted_edge.to = later;
    accepted_edge.measurement = best_correction;
    accepted_edge.weight = std::clamp(
        0.5f + 0.5f * best_geometry.confidence, 0.5f, 1.0f);
    online_loop_edges_.push_back(accepted_edge);

    PoseGraphEdgeVector graph_edges;
    graph_edges.reserve(frames.size() + online_loop_edges_.size());
    for (int index = 0; index < later; ++index) {
      PoseGraphEdge edge;
      edge.from = index;
      edge.to = index + 1;
      edge.measurement = SE3();
      edge.weight = 1.0f;
      graph_edges.push_back(edge);
    }
    for (const auto& online_edge : online_loop_edges_) {
      if (online_edge.from < 0 || online_edge.to > later ||
          online_edge.to <= online_edge.from) {
        continue;
      }
      PoseGraphEdge edge;
      edge.from = online_edge.from;
      edge.to = online_edge.to;
      edge.measurement = online_edge.measurement;
      edge.weight = online_edge.weight;
      edge.loop = true;
      graph_edges.push_back(edge);
    }
    PoseVector corrections(frames.size(), SE3());
    float max_adjacent_delta = 0.0f;
    float max_adjacent_rotation_deg = 0.0f;
    float max_window_delta = 0.0f;
    float max_window_strain = 0.0f;
    const auto optimize_and_check_graph = [&]() {
      corrections.assign(frames.size(), SE3());
      max_adjacent_delta = 0.0f;
      max_adjacent_rotation_deg = 0.0f;
      max_window_delta = 0.0f;
      max_window_strain = 0.0f;
      if (!optimize_pose_graph(corrections, graph_edges, 6)) {
        return false;
      }
      for (int index = 1; index <= later; ++index) {
        const V3 raw_delta =
            frames[index].raw_pose.t_ - frames[index - 1].raw_pose.t_;
        const V3 optimized_delta =
            corrections[index] * frames[index].raw_pose.t_ -
            corrections[index - 1] * frames[index - 1].raw_pose.t_;
        max_adjacent_delta = std::max(
            max_adjacent_delta, (optimized_delta - raw_delta).norm());
        const M3 relative_rotation =
            corrections[index - 1].R_.transpose() *
            corrections[index].R_;
        max_adjacent_rotation_deg = std::max(
            max_adjacent_rotation_deg,
            evaluate_loop_rotation(relative_rotation).total_deg);
      }
      int window_end = 0;
      for (int begin = 0; begin < later; ++begin) {
        window_end = std::max(window_end, begin + 1);
        while (window_end <= later &&
               cumulative_path[window_end] - cumulative_path[begin] <
                   30.0f) {
          ++window_end;
        }
        if (window_end > later) {
          break;
        }
        const float path =
            cumulative_path[window_end] - cumulative_path[begin];
        const V3 raw_delta = frames[window_end].raw_pose.t_ -
            frames[begin].raw_pose.t_;
        const V3 optimized_delta =
            corrections[window_end] * frames[window_end].raw_pose.t_ -
            corrections[begin] * frames[begin].raw_pose.t_;
        const float delta = (optimized_delta - raw_delta).norm();
        max_window_delta = std::max(max_window_delta, delta);
        max_window_strain = std::max(
            max_window_strain, delta / std::max(path, 1.0f));
      }
      const float adjacent_limit = std::max(
          0.10f, 0.25f * g_loop_keyframe_min_distance);
      return
          max_adjacent_delta <= adjacent_limit &&
          max_adjacent_rotation_deg <= 0.50f &&
          max_window_delta <= g_loop_max_local_translation_delta &&
          max_window_strain <= g_loop_max_local_translation_strain;
    };
    float accepted_measurement_alpha = 1.0f;
    bool graph_valid = optimize_and_check_graph();
    if (!graph_valid) {
      constexpr std::array<float, 2> kOnlineMeasurementAlphas = {
          0.50f, 0.25f};
      for (const float trial_alpha : kOnlineMeasurementAlphas) {
        const V6 scaled_measurement =
            trial_alpha * best_correction.log_vee();
        const SE3 trial_measurement(scaled_measurement);
        online_loop_edges_.back().measurement = trial_measurement;
        bool graph_edge_updated = false;
        for (auto edge_iterator = graph_edges.rbegin();
             edge_iterator != graph_edges.rend();
             ++edge_iterator) {
          if (edge_iterator->loop &&
              edge_iterator->from == accepted_edge.from &&
              edge_iterator->to == accepted_edge.to) {
            edge_iterator->measurement = trial_measurement;
            graph_edge_updated = true;
            break;
          }
        }
        if (!graph_edge_updated) {
          continue;
        }
        const bool trial_valid = optimize_and_check_graph();
        LOG(INFO) << GREEN
                  << " ---> [OnlineLoop]: safe measurement line search."
                  << " earlier=" << best_candidate.earlier
                  << " later=" << later
                  << " alpha=" << trial_alpha
                  << " adjacent_delta=" << max_adjacent_delta
                  << " window_delta=" << max_window_delta
                  << " window_strain=" << max_window_strain
                  << " safe=" << trial_valid << RESET;
        if (trial_valid) {
          accepted_measurement_alpha = trial_alpha;
          graph_valid = true;
          break;
        }
      }
    }
    if (!graph_valid) {
      online_loop_edges_.pop_back();
      online_loop_pending_ = OnlineLoopPending{};
      ++online_loop_rejected_count_;
      LOG(WARNING) << YELLOW
                   << " ---> [OnlineLoop]: graph safety rejected strict loop."
                   << " earlier=" << best_candidate.earlier
                   << " later=" << later
                   << " adjacent_delta=" << max_adjacent_delta
                   << " adjacent_rotation_deg="
                   << max_adjacent_rotation_deg
                   << " window_delta=" << max_window_delta
                   << " window_strain=" << max_window_strain << RESET;
      continue;
    }
    if (accepted_measurement_alpha < 1.0f) {
      LOG(WARNING) << YELLOW
                   << " ---> [OnlineLoop]: retained maximum safe partial correction."
                   << " earlier=" << best_candidate.earlier
                   << " later=" << later
                   << " alpha=" << accepted_measurement_alpha
                   << " full_measurement_preserved_for_final_backend=true"
                   << RESET;
    }

    VV3 raw_path;
    VV3 corrected_path;
    raw_path.reserve(frames.size());
    corrected_path.reserve(frames.size());
    for (std::size_t index = 0; index < frames.size(); ++index) {
      raw_path.push_back(frames[index].raw_pose.t_);
      corrected_path.push_back(
          (corrections[index] * frames[index].raw_pose).t_);
    }

    OnlineLoopResult result;
    result.accepted = true;
    result.earlier = best_candidate.earlier;
    result.later = later;
    result.latest_correction = corrections.back();
    result.raw_path = std::move(raw_path);
    result.corrected_path = std::move(corrected_path);
    result.overlap = best_geometry.symmetric_overlap;
    result.rmse = best_geometry.symmetric_trimmed_rmse;
    {
      std::lock_guard<std::mutex> lock(online_loop_mutex_);
      online_loop_results_.push_back(std::move(result));
      while (online_loop_results_.size() > 4U) {
        online_loop_results_.pop_front();
      }
    }
    ++online_loop_accepted_count_;
    online_loop_pending_ = OnlineLoopPending{};
    LOG(INFO) << GREEN
              << " ---> [OnlineLoop]: accepted and optimized asynchronously."
              << " earlier=" << best_candidate.earlier
              << " later=" << later
              << " overlap=" << best_geometry.symmetric_overlap
              << " rmse=" << best_geometry.symmetric_trimmed_rmse
              << " loops=" << online_loop_edges_.size()
              << " confirmation_mode="
              << (same_task_three_vote
                      ? "same_task_three_vote"
                      : "different_later_windows")
              << " same_task_consensus="
              << same_task_consensus_count
              << " measurement_alpha="
              << accepted_measurement_alpha
              << " adjacent_delta=" << max_adjacent_delta
              << " window_delta=" << max_window_delta
              << " task_seconds=" << std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - task_started).count()
              << RESET;
  }
}

void SuperLIO::pollOnlineLoopResults(
    const SE3& current_raw_pose, const double timestamp)
{
  if (!g_loop_online_enable) {
    return;
  }
  std::optional<OnlineLoopResult> latest;
  {
    std::lock_guard<std::mutex> lock(online_loop_mutex_);
    if (!online_loop_results_.empty()) {
      latest = std::move(online_loop_results_.back());
      online_loop_results_.clear();
    }
  }
  if (latest && latest->accepted) {
    online_loop_target_map_to_odom_ = latest->latest_correction;
    std::size_t path_suffix_frames = 0;
    {
      std::lock_guard<std::mutex> lock(loop_keyframes_mutex_);
      for (std::size_t index = latest->raw_path.size();
           index < loop_keyframes_.size(); ++index) {
        const SE3 raw_pose = loop_keyframes_[index].pose;
        latest->raw_path.push_back(raw_pose.t_);
        latest->corrected_path.push_back(
            (latest->latest_correction * raw_pose).t_);
        ++path_suffix_frames;
      }
    }
    if (!latest->raw_path.empty() &&
        latest->raw_path.size() == latest->corrected_path.size()) {
      data_wrapper_->pub_online_loop_paths(
          latest->raw_path, latest->corrected_path, timestamp);
    }
    const V3 current_position =
        online_loop_map_to_odom_ * current_raw_pose.t_;
    const V3 target_position =
        online_loop_target_map_to_odom_ * current_raw_pose.t_;
    LOG(INFO) << GREEN
              << " ---> [OnlineLoop]: new global correction target."
              << " earlier=" << latest->earlier
              << " later=" << latest->later
              << " path_suffix_frames=" << path_suffix_frames
              << " robot_translation="
              << (target_position - current_position).norm()
              << " overlap=" << latest->overlap
              << " rmse=" << latest->rmse << RESET;
  }
  advanceOnlineLoopCorrection(current_raw_pose);
}

void SuperLIO::advanceOnlineLoopCorrection(const SE3& current_raw_pose)
{
  if (!g_loop_online_enable) {
    return;
  }
  const auto yaw_of = [](const M3& rotation) {
    return std::atan2(rotation(1, 0), rotation(0, 0));
  };
  auto wrap_angle = [](float angle) {
    while (angle > static_cast<float>(M_PI)) {
      angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle < -static_cast<float>(M_PI)) {
      angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
  };
  const float current_yaw = yaw_of(online_loop_map_to_odom_.R_);
  const float target_yaw = yaw_of(online_loop_target_map_to_odom_.R_);
  const float maximum_yaw_step =
      g_loop_online_max_yaw_step_deg *
      static_cast<float>(M_PI) / 180.0f;
  const float yaw_step = std::clamp(
      wrap_angle(target_yaw - current_yaw),
      -maximum_yaw_step,
      maximum_yaw_step);
  const M3 next_rotation = Eigen::AngleAxisf(
      current_yaw + yaw_step, V3::UnitZ()).toRotationMatrix();

  const V3 raw_anchor = current_raw_pose.t_;
  const V3 current_anchor = online_loop_map_to_odom_ * raw_anchor;
  const V3 target_anchor =
      online_loop_target_map_to_odom_ * raw_anchor;
  V3 translation_step = target_anchor - current_anchor;
  const float translation_norm = translation_step.norm();
  if (translation_norm > g_loop_online_max_translation_step) {
    translation_step *=
        g_loop_online_max_translation_step / translation_norm;
  }
  const V3 next_anchor = current_anchor + translation_step;
  const V3 next_translation = next_anchor - next_rotation * raw_anchor;
  online_loop_map_to_odom_ = SE3(next_rotation, next_translation);
}

void SuperLIO::saveLoopClosedMap()
{
  if (!g_loop_closure_enable) {
    return;
  }

  const std::string loop_map_path = g_save_map_dir + "/" + g_loop_map_name;
  const std::string correction_path =
      g_save_map_dir + "/loop_correction.txt";
  const std::string trajectory_path =
      g_save_map_dir + "/loop_pose_graph.txt";
  const double finalize_budget_seconds =
      std::min(
          g_loop_max_finalize_seconds,
          g_loop_finalize_base_seconds +
              g_loop_finalize_seconds_per_keyframe *
                  static_cast<double>(loop_keyframes_.size()));
  const auto finalize_started = std::chrono::steady_clock::now();
  const auto finalize_deadline = finalize_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(finalize_budget_seconds));
  const auto endpoint_deadline = finalize_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
              0.35 * finalize_budget_seconds));
  // Do not let yaw expansion consume the complete endpoint budget. The tail
  // is reserved for an independent micro-window verification of provisional
  // partial-overlap hypotheses.
  const auto endpoint_seed_deadline = finalize_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
              0.28 * finalize_budget_seconds));
  const auto registration_deadline = finalize_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
              0.75 * finalize_budget_seconds));
  bool endpoint_timeout_logged = false;
  bool endpoint_seed_timeout_logged = false;
  bool registration_timeout_logged = false;
  auto endpoint_timed_out = [&]() {
    if (std::chrono::steady_clock::now() <= endpoint_deadline) {
      return false;
    }
    if (!endpoint_timeout_logged) {
      endpoint_timeout_logged = true;
      LOG(WARNING) << YELLOW
                   << " ---> 终点回环搜索阶段预算已用完，"
                   << "保留已有假设并转入中途回环。"
                   << " endpoint_budget_seconds: "
                   << 0.35 * finalize_budget_seconds << RESET;
    }
    return true;
  };
  auto endpoint_seed_timed_out = [&]() {
    if (std::chrono::steady_clock::now() <= endpoint_seed_deadline) {
      return false;
    }
    if (!endpoint_seed_timeout_logged) {
      endpoint_seed_timeout_logged = true;
      LOG(WARNING) << YELLOW
                   << " ---> 终点回环 seed 搜索预算已用完，"
                   << "保留尾部预算给 partial 独立小窗口复核。"
                   << " seed_budget_seconds: "
                   << 0.28 * finalize_budget_seconds << RESET;
    }
    return true;
  };
  auto registration_timed_out = [&]() {
    if (std::chrono::steady_clock::now() <= registration_deadline) {
      return false;
    }
    if (!registration_timeout_logged) {
      registration_timeout_logged = true;
      LOG(WARNING) << YELLOW
                   << " ---> 回环配准阶段预算已用完，"
                   << "保留已有回环边并转入位姿图和地图提交。"
                   << " registration_budget_seconds: "
                   << 0.75 * finalize_budget_seconds << RESET;
    }
    return true;
  };
  auto finalize_timed_out = [&]() {
    if (std::chrono::steady_clock::now() <= finalize_deadline) {
      return false;
    }
    LOG(WARNING) << YELLOW
                 << " ---> 闭环后端超过保存时间预算，保留已原子提交的原始 map.pcd。"
                 << " budget_seconds: " << finalize_budget_seconds
                 << RESET;
    return true;
  };
  LOG(INFO) << GREEN
            << " ---> 回环保存分阶段预算。total_seconds: "
            << finalize_budget_seconds
            << " keyframes: " << loop_keyframes_.size()
            << " base_seconds: " << g_loop_finalize_base_seconds
            << " seconds_per_keyframe: "
            << g_loop_finalize_seconds_per_keyframe
            << " max_seconds: " << g_loop_max_finalize_seconds
            << " endpoint_seconds: "
            << 0.35 * finalize_budget_seconds
            << " endpoint_seed_seconds: "
            << 0.28 * finalize_budget_seconds
            << " registration_seconds: "
            << 0.75 * finalize_budget_seconds << RESET;
  {
    // A repeated save into the same scene must never reuse an older accepted
    // closure when the current run cannot validate one.
    std::error_code ec;
    std::filesystem::remove(loop_map_path, ec);
    ec.clear();
    std::filesystem::remove(correction_path, ec);
    ec.clear();
    std::filesystem::remove(trajectory_path, ec);
  }

  // Endpoint closure keeps its conservative historical gap. Internal closure
  // only needs enough index separation for the two +/-W local submaps not to
  // overlap; sensor-time and travelled-path gates below provide independent
  // evidence that this is a revisit rather than adjacent trajectory.
  const int internal_min_index_gap =
      std::max(2 * g_loop_local_window_size + 5, 30);
  const std::size_t endpoint_min_keyframes =
      static_cast<std::size_t>(g_loop_keyframe_min_gap + 3);
  const std::size_t internal_min_keyframes =
      static_cast<std::size_t>(
          internal_min_index_gap + g_loop_local_window_size + 2);
  const std::size_t minimum_keyframes_for_any_loop =
      std::min(endpoint_min_keyframes, internal_min_keyframes);
  if (loop_keyframes_.size() < minimum_keyframes_for_any_loop) {
    LOG(WARNING) << YELLOW << " ---> 闭环关键帧数量不足，跳过 map_loop.pcd 生成。"
                 << " keyframes: " << loop_keyframes_.size()
                 << " endpoint_min_keyframes: "
                 << endpoint_min_keyframes
                 << " internal_min_keyframes: "
                 << internal_min_keyframes << RESET;
    return;
  }

  const int end_idx = static_cast<int>(loop_keyframes_.size()) - 1;
  const auto& end_keyframe = loop_keyframes_[end_idx];
  PoseVector raw_poses;
  raw_poses.reserve(loop_keyframes_.size());
  std::vector<float> cumulative_route_length(
      loop_keyframes_.size(), 0.0f);
  for (std::size_t i = 0; i < loop_keyframes_.size(); ++i) {
    raw_poses.push_back(loop_keyframes_[i].pose);
    if (i > 0) {
      cumulative_route_length[i] =
          cumulative_route_length[i - 1] +
          (raw_poses[i].t_ - raw_poses[i - 1].t_).norm();
    }
  }
  const float raw_route_length = cumulative_route_length.back();
  struct LoopCandidate {
    int index = -1;
    float horizontal_distance = std::numeric_limits<float>::max();
  };
  std::vector<LoopCandidate> candidates;

  for (int i = 0; i <= end_idx - g_loop_keyframe_min_gap; ++i) {
    const V3 delta = end_keyframe.pose.t_ - loop_keyframes_[i].pose.t_;
    const float horizontal_distance =
        static_cast<float>(std::hypot(delta.x(), delta.y()));
    if (horizontal_distance < g_loop_search_radius) {
      candidates.push_back({i, horizontal_distance});
    }
  }

  if (candidates.empty()) {
    LOG(INFO) << GREEN
              << " ---> 终点未进入任何历史关键帧的保守搜索半径；"
              << "不强制回起点，继续检查中途回环。"
              << " horizontal_search_radius: "
              << g_loop_search_radius << RESET;
  }

  // Schedule endpoint candidates by temporal place-cluster rather than letting
  // one dense revisit monopolize the registration budget. Within every cluster
  // farthest-index sampling covers its temporal extent; the outer round-robin
  // then gives each distinct place one candidate before taking another from the
  // same place. The earliest cluster is retained as a useful start-return hint,
  // but receives no special acceptance rule.
  const int endpoint_cluster_gap = 2 * g_loop_local_window_size;
  constexpr int kMinimumEndpointConsensus = 3;
  struct EndpointCandidateCluster
  {
    std::vector<LoopCandidate> members;
    std::vector<LoopCandidate> coverage_order;
    float minimum_horizontal_distance =
        std::numeric_limits<float>::max();
    int first_index = -1;
  };
  std::vector<EndpointCandidateCluster> endpoint_candidate_clusters;
  for (const auto& candidate : candidates) {
    if (endpoint_candidate_clusters.empty() ||
        candidate.index -
                endpoint_candidate_clusters.back().members.front().index >
            endpoint_cluster_gap) {
      endpoint_candidate_clusters.emplace_back();
      endpoint_candidate_clusters.back().first_index = candidate.index;
    }
    auto& cluster = endpoint_candidate_clusters.back();
    cluster.members.push_back(candidate);
    cluster.minimum_horizontal_distance = std::min(
        cluster.minimum_horizontal_distance,
        candidate.horizontal_distance);
  }

  for (auto& cluster : endpoint_candidate_clusters) {
    std::vector<bool> selected(cluster.members.size(), false);
    while (cluster.coverage_order.size() < cluster.members.size()) {
      int best_index = -1;
      float best_coverage = -1.0f;
      float best_horizontal = std::numeric_limits<float>::max();
      for (std::size_t i = 0; i < cluster.members.size(); ++i) {
        if (selected[i]) {
          continue;
        }
        float coverage = 0.0f;
        if (!cluster.coverage_order.empty()) {
          coverage = std::numeric_limits<float>::max();
          for (const auto& already_selected : cluster.coverage_order) {
            coverage = std::min(
                coverage,
                static_cast<float>(std::abs(
                    cluster.members[i].index - already_selected.index)));
          }
        }
        if (best_index < 0 || coverage > best_coverage ||
            (coverage == best_coverage &&
             cluster.members[i].horizontal_distance < best_horizontal)) {
          best_index = static_cast<int>(i);
          best_coverage = coverage;
          best_horizontal = cluster.members[i].horizontal_distance;
        }
      }
      if (best_index < 0) {
        break;
      }
      selected[best_index] = true;
      cluster.coverage_order.push_back(cluster.members[best_index]);
    }
  }
  std::stable_sort(
      endpoint_candidate_clusters.begin(),
      endpoint_candidate_clusters.end(),
      [](const EndpointCandidateCluster& lhs,
         const EndpointCandidateCluster& rhs) {
        const bool lhs_contains_start = lhs.first_index == 0;
        const bool rhs_contains_start = rhs.first_index == 0;
        if (lhs_contains_start != rhs_contains_start) {
          return lhs_contains_start;
        }
        return lhs.minimum_horizontal_distance <
            rhs.minimum_horizontal_distance;
      });

  const std::size_t discovered_endpoint_cluster_count =
      endpoint_candidate_clusters.size();
  endpoint_candidate_clusters.erase(
      std::remove_if(
          endpoint_candidate_clusters.begin(),
          endpoint_candidate_clusters.end(),
          [](const EndpointCandidateCluster& cluster) {
            return cluster.coverage_order.size() <
                static_cast<std::size_t>(kMinimumEndpointConsensus);
          }),
      endpoint_candidate_clusters.end());
  const int maximum_schedulable_endpoint_clusters =
      g_loop_candidate_limit >= kMinimumEndpointConsensus
      ? std::max(
            1, g_loop_candidate_limit /
                   kMinimumEndpointConsensus)
      : 0;
  if (static_cast<int>(endpoint_candidate_clusters.size()) >
      maximum_schedulable_endpoint_clusters) {
    endpoint_candidate_clusters.resize(
        static_cast<std::size_t>(
            maximum_schedulable_endpoint_clusters));
  }

  std::vector<LoopCandidate> selected_candidates;
  // Materialize the three votes needed by consensus as a bounded batch per
  // selected temporal cluster. A false place can consume only this fixed
  // quorum before the next cluster, while the start-return cluster can still
  // become usable if the deadline expires before every cluster is explored.
  for (const auto& cluster : endpoint_candidate_clusters) {
    for (int depth = 0;
         depth < kMinimumEndpointConsensus;
         ++depth) {
      if (static_cast<int>(selected_candidates.size()) >=
              g_loop_candidate_limit ||
          static_cast<std::size_t>(depth) >=
              cluster.coverage_order.size()) {
        break;
      }
      selected_candidates.push_back(
          cluster.coverage_order[static_cast<std::size_t>(depth)]);
    }
  }
  // Spend any remaining budget in cross-cluster rounds for extra medoid
  // stability; these are optional and never displace the minimum quorum.
  for (std::size_t depth = kMinimumEndpointConsensus;
       static_cast<int>(selected_candidates.size()) <
           g_loop_candidate_limit;
       ++depth) {
    bool added_at_depth = false;
    for (const auto& cluster : endpoint_candidate_clusters) {
      if (depth >= cluster.coverage_order.size()) {
        continue;
      }
      selected_candidates.push_back(cluster.coverage_order[depth]);
      added_at_depth = true;
      if (static_cast<int>(selected_candidates.size()) >=
          g_loop_candidate_limit) {
        break;
      }
    }
    if (!added_at_depth) {
      break;
    }
  }
  LOG(INFO) << GREEN
            << " ---> 终点候选按时间簇轮转。clusters: "
            << discovered_endpoint_cluster_count
            << " consensus_eligible_clusters: "
            << endpoint_candidate_clusters.size()
            << " discovered: " << candidates.size()
            << " scheduled: " << selected_candidates.size()
            << " cluster_gap: " << endpoint_cluster_gap << RESET;

  auto build_local_world_cloud_with_window = [&] (
      int center_idx, int window_size) {
    CloudPtr cloud(new PointCloudType());
    const int start_idx =
        std::max(0, center_idx - window_size);
    const int stop_idx =
        std::min(end_idx, center_idx + window_size);
    for (int i = start_idx; i <= stop_idx; ++i) {
      CloudPtr keyframe_cloud =
          loadLoopKeyFrameCloud(static_cast<std::size_t>(i));
      if (!keyframe_cloud || keyframe_cloud->empty()) {
        continue;
      }
      CloudPtr transformed(new PointCloudType());
      pcl::transformPointCloud(
          *keyframe_cloud,
          *transformed,
          se3_to_matrix4f(loop_keyframes_[i].pose));
      *cloud += *transformed;
    }
    normalize_cloud_layout(*cloud);
    CloudPtr filtered(new PointCloudType());
    make_map_pcd_cloud(cloud, *filtered, g_loop_map_ds_size, "loop local");
    return filtered;
  };
  auto build_local_world_cloud = [&](int center_idx) {
    return build_local_world_cloud_with_window(
        center_idx, g_loop_local_window_size);
  };

  CloudPtr source = build_local_world_cloud(end_idx);
  if (source->empty()) {
    LOG(WARNING) << YELLOW << " ---> 闭环 ICP 源点云为空，跳过 map_loop.pcd 生成。" << RESET;
    return;
  }
  CloudPtr source_ground =
      g_loop_ground_z_refinement_enable
      ? extract_loop_ground_envelope(
          source, raw_poses[end_idx].t_)
      : CloudPtr(new PointCloudType());

  struct EndpointHypothesis
  {
    int index = -1;
    float horizontal_distance = std::numeric_limits<float>::max();
    double score = std::numeric_limits<double>::max();
    float overlap_ratio = 0.0f;
    float rotation_deg = std::numeric_limits<float>::max();
    float yaw_deg = std::numeric_limits<float>::max();
    float tilt_deg = std::numeric_limits<float>::max();
    float yaw_limit_deg = 0.0f;
    int seed_yaw_deg = 0;
    int seed_translation_mode = -1;
    unsigned int seed_translation_mode_mask = 0U;
    unsigned int yaw_zero_translation_mode_mask = 0U;
    int basin_seed_count = 0;
    float post_anchor_distance =
        std::numeric_limits<float>::max();
    Eigen::Matrix4f full_correction = Eigen::Matrix4f::Identity();
    PoseGraphEdge edge;
    LoopConsistency consistency;
    LoopGeometryVerification geometry;
    LoopGeometryVerification micro_geometry;
    double selection_cost = std::numeric_limits<double>::max();
    int consensus_count = 0;
    float consensus_translation_spread =
        std::numeric_limits<float>::max();
    float consensus_yaw_spread_deg =
        std::numeric_limits<float>::max();
    bool partial_geometry = false;
    bool raw_consistent = false;
    bool micro_window_verified = false;
    // A two-vote endpoint is never a normal consensus. It is retained only
    // as a provisional measurement so two independent strict internal groups
    // can later predict and authorize it. Local evidence-scale fallback is
    // explicitly forbidden for this case.
    bool two_vote_strict_provisional = false;
    bool two_vote_all_strong = false;
    // A short ordered terminal path can disambiguate which part of an
    // otherwise repetitive corridor was revisited.  This never upgrades the
    // observation to full SE(3): along-corridor translation stays free.
    bool corridor_sequence_verified = false;
    int corridor_sequence_matches = 0;
    float corridor_sequence_span = 0.0f;
    float corridor_sequence_rmse =
        std::numeric_limits<float>::max();
    float corridor_sequence_density = 0.0f;
    float corridor_tangent_mismatch_deg =
        std::numeric_limits<float>::max();
    float corridor_axis_x = 0.0f;
    float corridor_axis_y = 0.0f;
    bool strong = false;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using EndpointHypothesisVector = std::vector<
      EndpointHypothesis,
      Eigen::aligned_allocator<EndpointHypothesis>>;
  EndpointHypothesisVector endpoint_hypotheses;
  struct EndpointRejectedBasin
  {
    float delta_x = 0.0f;
    float delta_y = 0.0f;
    float delta_z = 0.0f;
    float signed_yaw_deg = 0.0f;
    unsigned int seed_translation_mode_mask = 0U;
    unsigned int yaw_zero_translation_mode_mask = 0U;
    int seed_count = 0;
  };
  struct EndpointCandidateState
  {
    LoopCandidate candidate;
    float segment_path_length = 0.0f;
    float yaw_limit_deg = 0.0f;
    CloudPtr target;
    CloudPtr target_ground;
    CloudPtr micro_target;
    bool target_ground_ready = false;
    bool target_unavailable = false;
    bool converged = false;
    bool best_coarse_accepted = false;
    bool best_rotation_plausible = false;
    double best_score = std::numeric_limits<double>::max();
    float best_overlap_ratio = 0.0f;
    LoopRotationMetrics best_rotation;
    int best_seed_yaw_deg = 0;
    int best_seed_translation_mode = -1;
    int attempts = 0;
    int duplicate_basin_hits = 0;
    int duplicate_rejected_basin_hits = 0;
    bool mandatory_complete = false;
    bool expansion_saturated = false;
    EndpointHypothesisVector basins;
    std::vector<EndpointRejectedBasin> rejected_basins;
  };
  std::vector<EndpointCandidateState> endpoint_states;
  endpoint_states.reserve(selected_candidates.size());
  for (const auto& candidate : selected_candidates) {
    EndpointCandidateState state;
    state.candidate = candidate;
    state.segment_path_length =
        cumulative_route_length[end_idx] -
        cumulative_route_length[candidate.index];
    state.yaw_limit_deg =
        adaptive_loop_yaw_limit_deg(state.segment_path_length);
    state.target_ground.reset(new PointCloudType());
    state.target_ground_ready = !g_loop_ground_z_refinement_enable;
    endpoint_states.push_back(std::move(state));
  }

  const Eigen::Vector3f endpoint_source_anchor =
      end_keyframe.pose.t_.cast<float>();
  constexpr unsigned int kEndpointRawRelativeModeMask = 1U << 0U;
  constexpr unsigned int kEndpointAnchorCoincidentModeMask = 1U << 2U;
  const unsigned int endpoint_mandatory_mode_mask =
      kEndpointRawRelativeModeMask |
      kEndpointAnchorCoincidentModeMask;
  const auto endpoint_correction_anchor_delta = [&] (
      const EndpointHypothesis& hypothesis) -> Eigen::Vector3f {
    return (hypothesis.full_correction.block<3, 3>(0, 0) *
                endpoint_source_anchor +
            hypothesis.full_correction.block<3, 1>(0, 3) -
            endpoint_source_anchor).eval();
  };
  const auto endpoint_correction_signed_yaw = [] (
      const EndpointHypothesis& hypothesis) {
    return std::atan2(
               hypothesis.full_correction(1, 0),
               hypothesis.full_correction(0, 0)) *
        180.0f / static_cast<float>(M_PI);
  };
  const auto endpoint_yaw_difference = [] (
      const float lhs, const float rhs) {
    float difference = std::fmod(std::abs(lhs - rhs), 360.0f);
    return difference > 180.0f ? 360.0f - difference : difference;
  };
  const auto endpoint_hypothesis_is_better = [] (
      const EndpointHypothesis& candidate,
      const EndpointHypothesis& current) {
    if (candidate.strong != current.strong) {
      return candidate.strong;
    }
    if (candidate.raw_consistent != current.raw_consistent) {
      return candidate.raw_consistent;
    }
    if (candidate.geometry.valid != current.geometry.valid) {
      return candidate.geometry.valid;
    }
    if (candidate.micro_window_verified !=
        current.micro_window_verified) {
      return candidate.micro_window_verified;
    }
    if (std::abs(candidate.geometry.confidence -
                 current.geometry.confidence) > 1.0e-6f) {
      return candidate.geometry.confidence > current.geometry.confidence;
    }
    if (std::abs(candidate.consistency.weight -
                 current.consistency.weight) > 1.0e-6f) {
      return candidate.consistency.weight > current.consistency.weight;
    }
    if (std::abs(candidate.selection_cost -
                 current.selection_cost) > 1.0e-6) {
      return candidate.selection_cost < current.selection_cost;
    }
    if (candidate.edge.ground_z_valid != current.edge.ground_z_valid) {
      return candidate.edge.ground_z_valid;
    }
    return candidate.score < current.score;
  };
  const auto state_has_raw_consistent_hypothesis = [] (
      const EndpointCandidateState& state) {
    return std::any_of(
        state.basins.begin(), state.basins.end(),
        [](const EndpointHypothesis& hypothesis) {
          return hypothesis.raw_consistent;
        });
  };
  const auto state_has_strict_hypothesis = [] (
      const EndpointCandidateState& state) {
    return std::any_of(
        state.basins.begin(), state.basins.end(),
        [](const EndpointHypothesis& hypothesis) {
          return hypothesis.raw_consistent && hypothesis.geometry.valid;
        });
  };
  const auto record_rejected_endpoint_basin = [&] (
      EndpointCandidateState& state,
      const Eigen::Matrix4f& rejected_correction,
      const int seed_yaw_deg,
      const int translation_mode) {
    const Eigen::Vector3f delta =
        rejected_correction.block<3, 3>(0, 0) *
            endpoint_source_anchor +
        rejected_correction.block<3, 1>(0, 3) -
        endpoint_source_anchor;
    const float signed_yaw_deg = std::atan2(
        rejected_correction(1, 0), rejected_correction(0, 0)) *
        180.0f / static_cast<float>(M_PI);
    const unsigned int mode_mask =
        translation_mode >= 0 && translation_mode < 32
        ? (1U << static_cast<unsigned int>(translation_mode)) : 0U;
    for (auto& existing : state.rejected_basins) {
      const float translation_difference = std::sqrt(
          std::pow(existing.delta_x - delta.x(), 2.0f) +
          std::pow(existing.delta_y - delta.y(), 2.0f) +
          std::pow(existing.delta_z - delta.z(), 2.0f));
      const float yaw_difference = endpoint_yaw_difference(
          existing.signed_yaw_deg, signed_yaw_deg);
      if (translation_difference > 0.20f || yaw_difference > 0.25f) {
        continue;
      }
      existing.seed_translation_mode_mask |= mode_mask;
      if (seed_yaw_deg == 0) {
        existing.yaw_zero_translation_mode_mask |= mode_mask;
      }
      ++existing.seed_count;
      ++state.duplicate_rejected_basin_hits;
      // Do not stop after one failed seed. Only a rejected basin reproduced
      // by both mandatory initializations and a third independent attempt can
      // suppress further yaw expansion for this candidate.
      const bool independently_repeated =
          (existing.yaw_zero_translation_mode_mask &
           endpoint_mandatory_mode_mask) ==
              endpoint_mandatory_mode_mask &&
          existing.seed_count >= 3;
      state.expansion_saturated =
          state.expansion_saturated || independently_repeated;
      LOG(INFO) << GREEN
                << " ---> 终点回环 rejected seed 收敛到已有盆地。"
                << " candidate_idx: " << state.candidate.index
                << " translation_difference: "
                << translation_difference
                << " yaw_difference_deg: " << yaw_difference
                << " mode_mask: "
                << existing.seed_translation_mode_mask
                << " yaw_zero_mode_mask: "
                << existing.yaw_zero_translation_mode_mask
                << " seed_count: " << existing.seed_count
                << " expansion_saturated: "
                << state.expansion_saturated << RESET;
      return;
    }
    EndpointRejectedBasin rejected;
    rejected.delta_x = delta.x();
    rejected.delta_y = delta.y();
    rejected.delta_z = delta.z();
    rejected.signed_yaw_deg = signed_yaw_deg;
    rejected.seed_translation_mode_mask = mode_mask;
    rejected.yaw_zero_translation_mode_mask =
        seed_yaw_deg == 0 ? mode_mask : 0U;
    rejected.seed_count = 1;
    state.rejected_basins.push_back(rejected);
  };

  const auto evaluate_endpoint_seed = [&] (
      EndpointCandidateState& state,
      const int seed_yaw_deg,
      const int translation_mode) -> bool {
    if (endpoint_seed_timed_out()) {
      return false;
    }
    ++state.attempts;
    if (!state.target && !state.target_unavailable) {
      state.target = build_local_world_cloud(state.candidate.index);
      state.target_unavailable = !state.target || state.target->empty();
    }
    if (endpoint_seed_timed_out()) {
      return false;
    }
    if (state.target_unavailable) {
      return true;
    }

    const Eigen::Vector3f target_anchor =
        loop_keyframes_[state.candidate.index].pose.t_.cast<float>();
    const float seed_yaw_rad =
        static_cast<float>(seed_yaw_deg) *
        static_cast<float>(M_PI) / 180.0f;
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    initial_guess.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(seed_yaw_rad, Eigen::Vector3f::UnitZ())
            .toRotationMatrix();
    Eigen::Vector3f seeded_anchor = endpoint_source_anchor;
    if (translation_mode >= 1) {
      seeded_anchor.z() = target_anchor.z();
    }
    if (translation_mode == 2) {
      seeded_anchor = target_anchor;
    }
    initial_guess.block<3, 1>(0, 3) =
        seeded_anchor -
        initial_guess.block<3, 3>(0, 0) * endpoint_source_anchor;

    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(source);
    icp.setInputTarget(state.target);
    icp.setMaxCorrespondenceDistance(g_loop_icp_max_distance);
    icp.setMaximumIterations(80);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-5);
    PointCloudType aligned;
    icp.align(aligned, initial_guess);
    const Eigen::Matrix4f attempt_correction =
        icp.getFinalTransformation();
    if (endpoint_seed_timed_out()) {
      return false;
    }
    if (!icp.hasConverged() || !attempt_correction.allFinite() ||
        aligned.empty()) {
      return true;
    }

    pcl::search::KdTree<PointType> target_tree;
    target_tree.setInputCloud(state.target);
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_squared_distance(1);
    std::size_t overlap_count = 0;
    double squared_distance_sum = 0.0;
    const float max_squared_distance =
        g_loop_icp_max_distance * g_loop_icp_max_distance;
    for (const auto& point : aligned.points) {
      if (target_tree.nearestKSearch(
              point, 1, nearest_index,
              nearest_squared_distance) > 0 &&
          nearest_squared_distance[0] <= max_squared_distance) {
        ++overlap_count;
        squared_distance_sum += nearest_squared_distance[0];
      }
    }
    if (endpoint_seed_timed_out()) {
      return false;
    }
    if (overlap_count == 0) {
      return true;
    }
    const double score =
        squared_distance_sum / static_cast<double>(overlap_count);
    const float overlap_ratio =
        static_cast<float>(overlap_count) /
        static_cast<float>(aligned.size());
    const LoopRotationMetrics rotation = evaluate_loop_rotation(
        attempt_correction.block<3, 3>(0, 0));
    const bool rotation_plausible = loop_rotation_is_plausible(
        rotation, state.yaw_limit_deg);
    const bool coarse_accepted =
        score <= g_loop_icp_score_threshold &&
        overlap_ratio >= g_loop_min_overlap_ratio &&
        rotation_plausible;
    const bool better_coarse =
        (!state.best_coarse_accepted && coarse_accepted) ||
        (state.best_coarse_accepted == coarse_accepted &&
         ((!state.best_rotation_plausible && rotation_plausible) ||
          (state.best_rotation_plausible == rotation_plausible &&
           score < state.best_score)));
    state.converged = true;
    if (better_coarse) {
      state.best_coarse_accepted = coarse_accepted;
      state.best_rotation_plausible = rotation_plausible;
      state.best_score = score;
      state.best_overlap_ratio = overlap_ratio;
      state.best_rotation = rotation;
      state.best_seed_yaw_deg = seed_yaw_deg;
      state.best_seed_translation_mode = translation_mode;
    }
    if (!coarse_accepted) {
      const Eigen::Matrix4f rejected_correction = se3_to_matrix4f(
          project_gravity_aligned_loop_correction(
              attempt_correction, raw_poses[end_idx].t_));
      record_rejected_endpoint_basin(
          state, rejected_correction, seed_yaw_deg,
          translation_mode);
      return true;
    }

    EndpointHypothesis attempt;
    attempt.index = state.candidate.index;
    attempt.horizontal_distance = state.candidate.horizontal_distance;
    attempt.score = score;
    attempt.overlap_ratio = overlap_ratio;
    attempt.rotation_deg = rotation.total_deg;
    attempt.yaw_deg = rotation.yaw_deg;
    attempt.tilt_deg = rotation.tilt_deg;
    attempt.yaw_limit_deg = state.yaw_limit_deg;
    attempt.seed_yaw_deg = seed_yaw_deg;
    attempt.seed_translation_mode = translation_mode;
    attempt.seed_translation_mode_mask =
        translation_mode >= 0 && translation_mode < 32
        ? (1U << static_cast<unsigned int>(translation_mode)) : 0U;
    attempt.yaw_zero_translation_mode_mask =
        seed_yaw_deg == 0
        ? attempt.seed_translation_mode_mask : 0U;
    attempt.basin_seed_count = 1;
    attempt.edge.from = state.candidate.index;
    attempt.edge.to = end_idx;
    Eigen::Matrix4f graph_correction = se3_to_matrix4f(
        project_gravity_aligned_loop_correction(
            attempt_correction, raw_poses[end_idx].t_));
    if (endpoint_seed_timed_out()) {
      return false;
    }
    if (!state.target_ground_ready) {
      state.target_ground = extract_loop_ground_envelope(
          state.target, raw_poses[state.candidate.index].t_);
      state.target_ground_ready = true;
    }
    const GroundZRefinement ground_z = refine_loop_ground_z(
        source_ground, state.target_ground, graph_correction);
    if (endpoint_seed_timed_out()) {
      return false;
    }
    attempt.geometry = verify_loop_geometry(
        source, state.target, graph_correction, raw_poses[end_idx].t_);
    if (endpoint_seed_timed_out()) {
      return false;
    }
    const bool partial_geometry_valid =
        endpoint_partial_geometry_is_valid(attempt.geometry);
    const bool endpoint_geometry_valid =
        attempt.geometry.valid || partial_geometry_valid;
    const Eigen::Vector3f corrected_source_anchor =
        graph_correction.block<3, 3>(0, 0) * endpoint_source_anchor +
        graph_correction.block<3, 1>(0, 3);
    attempt.post_anchor_distance =
        (corrected_source_anchor - target_anchor).norm();
    LOG(INFO) << GREEN
              << " ---> 终点回环 seed 精验。candidate_idx: "
              << state.candidate.index
              << " seed_yaw_deg: " << seed_yaw_deg
              << " seed_translation_mode: " << translation_mode
              << " coarse_score: " << score
              << " coarse_overlap_ratio: " << overlap_ratio
              << " ground_pairs: " << ground_z.pair_count
              << " ground_inlier_ratio: " << ground_z.inlier_ratio
              << " ground_residual_mad: " << ground_z.residual_mad
              << " ground_z_adjustment: " << ground_z.z_adjustment
              << " ground_applied: " << ground_z.valid
              << " tight_symmetric_overlap: "
              << attempt.geometry.symmetric_overlap
              << " tight_trimmed_rmse: "
              << attempt.geometry.symmetric_trimmed_rmse
              << " blocks: "
              << std::min(
                     attempt.geometry.source_to_target.supported_blocks,
                     attempt.geometry.target_to_source.supported_blocks)
              << " block_ratio: "
              << std::min(
                     attempt.geometry.source_to_target.supported_block_ratio,
                     attempt.geometry.target_to_source.supported_block_ratio)
              << " support_span: "
              << std::min(
                     attempt.geometry.source_to_target.support_span,
                     attempt.geometry.target_to_source.support_span)
              << " support_minor_span: "
              << std::min(
                     attempt.geometry.source_to_target.support_minor_span,
                     attempt.geometry.target_to_source.support_minor_span)
              << " structural_symmetric_overlap: "
              << attempt.geometry.structural_symmetric_overlap
              << " structural_trimmed_rmse: "
              << attempt.geometry.structural_symmetric_trimmed_rmse
              << " anchor_translation: "
              << attempt.geometry.anchor_translation
              << " post_anchor_distance: "
              << attempt.post_anchor_distance
              << " yaw_deg: " << attempt.geometry.yaw_deg
              << " confidence: " << attempt.geometry.confidence
              << " accepted_geometry: "
              << (attempt.geometry.valid
                      ? "strict"
                      : (partial_geometry_valid ? "partial" : "rejected"))
              << RESET;
    if (!endpoint_geometry_valid) {
      record_rejected_endpoint_basin(
          state, graph_correction, seed_yaw_deg,
          translation_mode);
      return true;
    }

    attempt.partial_geometry =
        partial_geometry_valid && !attempt.geometry.valid;
    attempt.full_correction = graph_correction;
    attempt.edge.measurement = SE3(graph_correction);
    attempt.edge.weight = 1.0f + 0.5f * attempt.geometry.confidence;
    attempt.edge.loop = true;
    attempt.edge.endpoint = true;
    attempt.edge.ground_z_valid = ground_z.valid;
    PoseVector identity_reference(loop_keyframes_.size(), SE3());
    attempt.consistency = evaluate_loop_consistency(
        attempt.edge, identity_reference, state.segment_path_length,
        raw_poses[end_idx].t_);
    attempt.selection_cost =
        attempt.score + (1.0 - attempt.consistency.weight) +
        0.1 * (1.0 - attempt.overlap_ratio) +
        0.5 * (1.0 - attempt.geometry.confidence);
    const float consistency_limit = attempt.partial_geometry
        ? std::max(0.60f, g_loop_min_consistency_weight)
        : g_loop_min_consistency_weight;
    attempt.raw_consistent =
        attempt.consistency.weight >= consistency_limit;
    LOG(INFO) << GREEN
              << " ---> 终点回环 seed 原始图一致性。candidate_idx: "
              << state.candidate.index
              << " seed_yaw_deg: " << seed_yaw_deg
              << " seed_translation_mode: " << translation_mode
              << " path_length: " << attempt.consistency.path_length
              << " translation_residual: "
              << attempt.consistency.translation_residual
              << " rotation_residual_deg: "
              << attempt.consistency.rotation_residual_deg
              << " consistency_weight: " << attempt.consistency.weight
              << " consistency_limit: " << consistency_limit
              << " post_anchor_distance: "
              << attempt.post_anchor_distance
              << " accepted: " << attempt.raw_consistent << RESET;

    for (auto& existing : state.basins) {
      const float translation_difference =
          (endpoint_correction_anchor_delta(existing) -
           endpoint_correction_anchor_delta(attempt)).norm();
      const float yaw_difference = endpoint_yaw_difference(
          endpoint_correction_signed_yaw(existing),
          endpoint_correction_signed_yaw(attempt));
      if (translation_difference > 0.20f || yaw_difference > 0.25f) {
        continue;
      }
      const unsigned int merged_mode_mask =
          existing.seed_translation_mode_mask |
          attempt.seed_translation_mode_mask;
      const unsigned int merged_yaw_zero_mode_mask =
          existing.yaw_zero_translation_mode_mask |
          attempt.yaw_zero_translation_mode_mask;
      const int merged_seed_count =
          existing.basin_seed_count + attempt.basin_seed_count;
      ++state.duplicate_basin_hits;
      if (!existing.raw_consistent && !attempt.raw_consistent) {
        ++state.duplicate_rejected_basin_hits;
      }
      if (endpoint_hypothesis_is_better(attempt, existing)) {
        existing = attempt;
      }
      existing.seed_translation_mode_mask = merged_mode_mask;
      existing.yaw_zero_translation_mode_mask =
          merged_yaw_zero_mode_mask;
      existing.basin_seed_count = merged_seed_count;
      const bool has_raw_hypothesis =
          state_has_raw_consistent_hypothesis(state);
      const bool has_strict_hypothesis =
          state_has_strict_hypothesis(state);
      const bool mandatory_seed_consensus =
          (existing.yaw_zero_translation_mode_mask &
           endpoint_mandatory_mode_mask) ==
          endpoint_mandatory_mode_mask;
      state.expansion_saturated =
          (!has_raw_hypothesis &&
           mandatory_seed_consensus) ||
          (has_raw_hypothesis && !has_strict_hypothesis &&
           state.basins.size() == 1 &&
           mandatory_seed_consensus);
      LOG(INFO) << GREEN
                << " ---> 终点回环 seed 收敛到已有盆地。candidate_idx: "
                << state.candidate.index
                << " translation_difference: "
                << translation_difference
                << " yaw_difference_deg: " << yaw_difference
                << " mode_mask: "
                << existing.seed_translation_mode_mask
                << " yaw_zero_mode_mask: "
                << existing.yaw_zero_translation_mode_mask
                << " basin_seed_count: " << existing.basin_seed_count
                << " duplicate_hits: " << state.duplicate_basin_hits
                << " expansion_saturated: "
                << state.expansion_saturated << RESET;
      return true;
    }
    state.basins.push_back(attempt);
    return true;
  };

  // The mandatory phase is bounded to exactly two yaw-zero registrations per
  // candidate. Once a candidate starts, run both raw-relative and
  // anchor-coincident initializations before consulting the seed deadline;
  // this prevents an unlucky deadline boundary from observing mode 0 only.
  // The candidate order itself is the temporal-cluster round-robin above, so
  // a false historical place can consume at most two registrations before a
  // different place gets the same opportunity.
  int endpoint_mandatory_candidates_done = 0;
  bool endpoint_seed_phase_active = true;
  for (auto& state : endpoint_states) {
    if (endpoint_seed_timed_out()) {
      endpoint_seed_phase_active = false;
      break;
    }
    if (!evaluate_endpoint_seed(state, 0, 0) ||
        endpoint_seed_timed_out() ||
        !evaluate_endpoint_seed(state, 0, 2)) {
      endpoint_seed_phase_active = false;
      break;
    }
    state.mandatory_complete = true;
    ++endpoint_mandatory_candidates_done;
  }
  const bool endpoint_mandatory_phase_complete =
      endpoint_mandatory_candidates_done ==
          static_cast<int>(endpoint_states.size());
  LOG(INFO) << GREEN
            << " ---> 终点回环 mandatory seed 公平比较完成。done: "
            << endpoint_mandatory_candidates_done
            << " total: " << endpoint_states.size()
            << " complete: " << endpoint_mandatory_phase_complete
            << RESET;

  // Z-aligned yaw=0 is a supplemental basin for candidates not already proven
  // by strict geometry. It is still evaluated one candidate at a time across
  // the complete schedule, never as a private inner loop.
  if (endpoint_seed_phase_active &&
      !endpoint_seed_timed_out()) {
    for (auto& state : endpoint_states) {
      if (endpoint_seed_timed_out()) {
        break;
      }
      if (state.mandatory_complete &&
          !state_has_strict_hypothesis(state) &&
          !state.expansion_saturated) {
        if (!evaluate_endpoint_seed(state, 0, 1)) {
          endpoint_seed_phase_active = false;
          break;
        }
      }
    }
  }

  // Expand yaw in fair global rounds. Duplicate rejected/partial basins stop
  // further work on that candidate, preventing one false place from consuming
  // all 21 historical seed combinations.
  const std::array<int, 6> expansion_yaw_degrees{
      -15, 15, -30, 30, -45, 45};
  const std::array<int, 3> expansion_translation_modes{0, 2, 1};
  constexpr int kMaximumEndpointAttemptsPerCandidate = 15;
  // Cover the complete yaw range in mode 0, then mode 2, before spending any
  // optional budget on mode 1. Thus the hard per-candidate cap removes the
  // 21-attempt monopoly without losing the +/-45 degree search envelope.
  for (const int translation_mode : expansion_translation_modes) {
    if (!endpoint_seed_phase_active) {
      break;
    }
    for (const int seed_yaw_deg : expansion_yaw_degrees) {
      for (auto& state : endpoint_states) {
        if (endpoint_seed_timed_out()) {
          break;
        }
        if (state.attempts >=
            kMaximumEndpointAttemptsPerCandidate) {
          if (!state.expansion_saturated) {
            state.expansion_saturated = true;
            LOG(INFO) << GREEN
                      << " ---> 终点回环候选达到公平尝试上限。"
                      << " candidate_idx: " << state.candidate.index
                      << " attempts: " << state.attempts
                      << " limit: "
                      << kMaximumEndpointAttemptsPerCandidate
                      << RESET;
          }
          continue;
        }
        if (!state.mandatory_complete ||
            state.expansion_saturated ||
            state_has_strict_hypothesis(state)) {
          continue;
        }
        if (!evaluate_endpoint_seed(
                state, seed_yaw_deg, translation_mode)) {
          endpoint_seed_phase_active = false;
          break;
        }
      }
      if (!endpoint_seed_phase_active || endpoint_seed_timed_out()) {
        break;
      }
    }
    if (!endpoint_seed_phase_active || endpoint_seed_timed_out()) {
      break;
    }
  }

  // A partial full-window match is provisional even when several temporal
  // neighbours agree: those neighbours share most of their points.  Before
  // allowing such a basin into endpoint consensus, require two independent
  // pieces of evidence:
  //   1. yaw-zero raw-relative and anchor-coincident seeds reached the same
  //      correction basin; and
  //   2. a new GICP estimate on a smaller local submap remains in that basin
  //      and passes the normal strict structural verifier.
  // The corrected endpoint-to-history distance is diagnostic only. Different
  // runs may stop at different physical positions, so it is never a gate.
  const int endpoint_micro_window_size = std::max(
      1, g_loop_local_window_size / 2);
  std::vector<std::vector<std::size_t>> endpoint_micro_basin_indices(
      endpoint_states.size());
  for (std::size_t state_index = 0;
       state_index < endpoint_states.size(); ++state_index) {
    auto& state = endpoint_states[state_index];
    auto& eligible = endpoint_micro_basin_indices[state_index];
    for (std::size_t basin_index = 0;
         basin_index < state.basins.size(); ++basin_index) {
      const auto& basin = state.basins[basin_index];
      if (!basin.raw_consistent || !basin.partial_geometry) {
        continue;
      }
      const bool mandatory_seed_consensus =
          (basin.yaw_zero_translation_mode_mask &
           endpoint_mandatory_mode_mask) ==
          endpoint_mandatory_mode_mask;
      LOG(INFO) << GREEN
                << " ---> 终点 partial 盆地 mandatory seed 共识。"
                << " candidate_idx: " << state.candidate.index
                << " mode_mask: " << basin.seed_translation_mode_mask
                << " yaw_zero_mode_mask: "
                << basin.yaw_zero_translation_mode_mask
                << " basin_seed_count: " << basin.basin_seed_count
                << " accepted: " << mandatory_seed_consensus
                << RESET;
      if (mandatory_seed_consensus) {
        eligible.push_back(basin_index);
      }
    }
    std::stable_sort(
        eligible.begin(), eligible.end(),
        [&](const std::size_t lhs, const std::size_t rhs) {
          return endpoint_hypothesis_is_better(
              state.basins[lhs], state.basins[rhs]);
        });
  }

  std::vector<std::pair<std::size_t, std::size_t>>
      endpoint_micro_schedule;
  for (std::size_t depth = 0;; ++depth) {
    bool added = false;
    for (std::size_t state_index = 0;
         state_index < endpoint_states.size(); ++state_index) {
      const auto& eligible = endpoint_micro_basin_indices[state_index];
      if (depth >= eligible.size()) {
        continue;
      }
      endpoint_micro_schedule.emplace_back(
          state_index, eligible[depth]);
      added = true;
    }
    if (!added) {
      break;
    }
  }

  CloudPtr endpoint_micro_source;
  const auto evaluate_alignment_quality = [] (
      const PointCloudType::ConstPtr& quality_source,
      const PointCloudType::ConstPtr& quality_target,
      const Eigen::Matrix4f& quality_correction,
      double& score,
      float& overlap_ratio) {
    score = std::numeric_limits<double>::max();
    overlap_ratio = 0.0f;
    if (!quality_source || !quality_target ||
        quality_source->empty() || quality_target->empty() ||
        !quality_correction.allFinite()) {
      return false;
    }
    CloudPtr aligned_source(new PointCloudType());
    pcl::transformPointCloud(
        *quality_source, *aligned_source, quality_correction);
    if (aligned_source->empty()) {
      return false;
    }
    pcl::search::KdTree<PointType> target_tree;
    target_tree.setInputCloud(quality_target);
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_squared_distance(1);
    std::size_t overlap_count = 0;
    double squared_distance_sum = 0.0;
    const float maximum_squared_distance =
        g_loop_icp_max_distance * g_loop_icp_max_distance;
    for (const auto& point : aligned_source->points) {
      if (target_tree.nearestKSearch(
              point, 1, nearest_index,
              nearest_squared_distance) > 0 &&
          nearest_squared_distance[0] <= maximum_squared_distance) {
        ++overlap_count;
        squared_distance_sum += nearest_squared_distance[0];
      }
    }
    if (overlap_count == 0) {
      return false;
    }
    score = squared_distance_sum /
        static_cast<double>(overlap_count);
    overlap_ratio = static_cast<float>(overlap_count) /
        static_cast<float>(aligned_source->size());
    return std::isfinite(score) && std::isfinite(overlap_ratio);
  };

  const auto verify_endpoint_partial_micro = [&] (
      EndpointCandidateState& state,
      EndpointHypothesis& basin) {
    if (endpoint_timed_out()) {
      return false;
    }
    if (!endpoint_micro_source) {
      endpoint_micro_source = build_local_world_cloud_with_window(
          end_idx, endpoint_micro_window_size);
    }
    if (!state.micro_target) {
      state.micro_target = build_local_world_cloud_with_window(
          state.candidate.index, endpoint_micro_window_size);
    }
    const CloudPtr& micro_target = state.micro_target;
    if (!endpoint_micro_source || endpoint_micro_source->empty() ||
        !micro_target || micro_target->empty() ||
        !state.target || state.target->empty()) {
      LOG(WARNING) << YELLOW
                   << " ---> 终点 partial 独立小窗口点云为空，拒绝盆地。"
                   << " candidate_idx: " << state.candidate.index
                   << RESET;
      return false;
    }
    if (endpoint_timed_out()) {
      return false;
    }

    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> micro_icp;
    micro_icp.setInputSource(endpoint_micro_source);
    micro_icp.setInputTarget(micro_target);
    micro_icp.setMaxCorrespondenceDistance(
        std::min(
            g_loop_icp_max_distance,
            std::max(0.50f, 2.0f * g_loop_verification_max_distance)));
    micro_icp.setMaximumIterations(40);
    micro_icp.setTransformationEpsilon(1e-6);
    micro_icp.setEuclideanFitnessEpsilon(1e-5);
    PointCloudType micro_aligned;
    micro_icp.align(micro_aligned, basin.full_correction);
    const Eigen::Matrix4f micro_result =
        micro_icp.getFinalTransformation();
    if (!micro_icp.hasConverged() || !micro_result.allFinite() ||
        micro_aligned.empty()) {
      LOG(INFO) << GREEN
                << " ---> 终点 partial 独立小窗口复核。candidate_idx: "
                << state.candidate.index
                << " converged: " << micro_icp.hasConverged()
                << " accepted_geometry: rejected"
                << " reason: micro_gicp_failed" << RESET;
      return false;
    }
    if (endpoint_timed_out()) {
      return false;
    }

    Eigen::Matrix4f final_correction = se3_to_matrix4f(
        project_gravity_aligned_loop_correction(
            micro_result, raw_poses[end_idx].t_));
    if (!state.target_ground_ready) {
      state.target_ground = extract_loop_ground_envelope(
          state.target, raw_poses[state.candidate.index].t_);
      state.target_ground_ready = true;
    }
    const GroundZRefinement ground_z = refine_loop_ground_z(
        source_ground, state.target_ground, final_correction);
    if (endpoint_timed_out()) {
      return false;
    }

    const Eigen::Vector3f original_delta =
        endpoint_correction_anchor_delta(basin);
    const Eigen::Vector3f final_delta =
        final_correction.block<3, 3>(0, 0) * endpoint_source_anchor +
        final_correction.block<3, 1>(0, 3) - endpoint_source_anchor;
    const float correction_translation_difference =
        (final_delta - original_delta).norm();
    const float final_signed_yaw = std::atan2(
        final_correction(1, 0), final_correction(0, 0)) *
        180.0f / static_cast<float>(M_PI);
    const float correction_yaw_difference = endpoint_yaw_difference(
        final_signed_yaw, endpoint_correction_signed_yaw(basin));
    const bool same_correction_basin =
        correction_translation_difference <= 0.30f &&
        correction_yaw_difference <= 0.50f;

    const LoopGeometryVerification micro_geometry =
        verify_loop_geometry(
            endpoint_micro_source, micro_target,
            final_correction, raw_poses[end_idx].t_);
    if (endpoint_timed_out()) {
      return false;
    }
    const bool micro_static_geometry_valid =
        micro_geometry.valid &&
        micro_geometry.structural_evidence_available &&
        micro_geometry.structural_valid;
    const LoopGeometryVerification full_geometry =
        verify_loop_geometry(
            source, state.target, final_correction,
            raw_poses[end_idx].t_);
    if (endpoint_timed_out()) {
      return false;
    }
    const bool full_partial_geometry_valid =
        endpoint_partial_geometry_is_valid(full_geometry);
    const bool full_geometry_valid =
        full_geometry.valid || full_partial_geometry_valid;
    const LoopRotationMetrics rotation = evaluate_loop_rotation(
        final_correction.block<3, 3>(0, 0));
    const bool rotation_plausible = loop_rotation_is_plausible(
        rotation, state.yaw_limit_deg);

    double final_score = std::numeric_limits<double>::max();
    float final_overlap_ratio = 0.0f;
    const bool full_quality_available = evaluate_alignment_quality(
        source, state.target, final_correction,
        final_score, final_overlap_ratio);
    if (endpoint_timed_out()) {
      return false;
    }
    const bool coarse_quality_valid =
        full_quality_available &&
        final_score <= g_loop_icp_score_threshold &&
        final_overlap_ratio >= g_loop_min_overlap_ratio;

    EndpointHypothesis verified = basin;
    verified.score = final_score;
    verified.overlap_ratio = final_overlap_ratio;
    verified.rotation_deg = rotation.total_deg;
    verified.yaw_deg = rotation.yaw_deg;
    verified.tilt_deg = rotation.tilt_deg;
    verified.full_correction = final_correction;
    verified.edge.measurement = SE3(final_correction);
    verified.edge.weight = 1.0f + 0.25f *
        (full_geometry.confidence + micro_geometry.confidence);
    verified.edge.ground_z_valid = ground_z.valid;
    verified.geometry = full_geometry;
    verified.micro_geometry = micro_geometry;
    verified.partial_geometry =
        !full_geometry.valid && full_partial_geometry_valid;
    const Eigen::Vector3f target_anchor =
        raw_poses[state.candidate.index].t_.cast<float>();
    verified.post_anchor_distance =
        (final_correction.block<3, 3>(0, 0) *
             endpoint_source_anchor +
         final_correction.block<3, 1>(0, 3) - target_anchor).norm();
    PoseVector identity_reference(loop_keyframes_.size(), SE3());
    verified.consistency = evaluate_loop_consistency(
        verified.edge, identity_reference,
        state.segment_path_length, raw_poses[end_idx].t_);
    const float consistency_limit = verified.partial_geometry
        ? std::max(0.60f, g_loop_min_consistency_weight)
        : g_loop_min_consistency_weight;
    verified.raw_consistent =
        verified.consistency.weight >= consistency_limit;
    verified.selection_cost =
        verified.score + (1.0 - verified.consistency.weight) +
        0.1 * (1.0 - verified.overlap_ratio) +
        0.5 * (1.0 - verified.geometry.confidence) +
        0.25 * (1.0 - verified.micro_geometry.confidence);
    verified.micro_window_verified =
        same_correction_basin && micro_static_geometry_valid &&
        full_geometry_valid && rotation_plausible &&
        coarse_quality_valid && verified.raw_consistent;

    LOG(INFO) << GREEN
              << " ---> 终点 partial 独立小窗口复核。candidate_idx: "
              << state.candidate.index
              << " micro_window_size: " << endpoint_micro_window_size
              << " correction_translation_difference: "
              << correction_translation_difference
              << " correction_yaw_difference_deg: "
              << correction_yaw_difference
              << " micro_symmetric_overlap: "
              << micro_geometry.symmetric_overlap
              << " micro_trimmed_rmse: "
              << micro_geometry.symmetric_trimmed_rmse
              << " micro_structural_overlap: "
              << micro_geometry.structural_symmetric_overlap
              << " micro_accepted_geometry: "
              << (micro_static_geometry_valid ? "strict" : "rejected")
              << " full_accepted_geometry: "
              << (full_geometry.valid
                      ? "strict"
                      : (full_partial_geometry_valid
                             ? "partial" : "rejected"))
              << " full_score: " << final_score
              << " full_overlap_ratio: " << final_overlap_ratio
              << " raw_consistency_weight: "
              << verified.consistency.weight
              << " post_anchor_distance: "
              << verified.post_anchor_distance
              << " post_anchor_distance_is_gate: false"
              << " accepted: " << verified.micro_window_verified
              << RESET;
    if (!verified.micro_window_verified) {
      return false;
    }
    basin = verified;
    return true;
  };

  int endpoint_micro_verified_count = 0;
  for (const auto& [state_index, basin_index] :
       endpoint_micro_schedule) {
    if (endpoint_timed_out()) {
      break;
    }
    auto& state = endpoint_states[state_index];
    auto& basin = state.basins[basin_index];
    if (verify_endpoint_partial_micro(state, basin)) {
      ++endpoint_micro_verified_count;
    }
  }
  LOG(INFO) << GREEN
            << " ---> 终点 partial 独立小窗口复核汇总。scheduled: "
            << endpoint_micro_schedule.size()
            << " verified: " << endpoint_micro_verified_count
            << " micro_window_size: " << endpoint_micro_window_size
            << RESET;

  for (auto& state : endpoint_states) {
    std::optional<EndpointHypothesis> best_hypothesis;
    for (const auto& hypothesis : state.basins) {
      if (!hypothesis.raw_consistent ||
          (!hypothesis.geometry.valid &&
           !hypothesis.micro_window_verified)) {
        continue;
      }
      if (!best_hypothesis.has_value() ||
          endpoint_hypothesis_is_better(
              hypothesis, *best_hypothesis)) {
        best_hypothesis = hypothesis;
      }
    }
    LOG(INFO) << GREEN
              << " ---> 闭环候选配准结果。candidate_idx: "
              << state.candidate.index
              << " horizontal_distance: "
              << state.candidate.horizontal_distance
              << " converged: " << state.converged
              << " best_score: " << state.best_score
              << " overlap_ratio: " << state.best_overlap_ratio
              << " rotation_deg: " << state.best_rotation.total_deg
              << " yaw_deg: " << state.best_rotation.yaw_deg
              << " tilt_deg: " << state.best_rotation.tilt_deg
              << " yaw_limit_deg: " << state.yaw_limit_deg
              << " seed_yaw_deg: " << state.best_seed_yaw_deg
              << " seed_translation_mode: "
              << state.best_seed_translation_mode
              << " attempts: " << state.attempts
              << " distinct_basins: " << state.basins.size()
              << " duplicate_basins: " << state.duplicate_basin_hits
              << " accepted_geometry: "
              << (!best_hypothesis.has_value()
                      ? "none"
                      : (best_hypothesis->geometry.valid
                             ? "strict" : "partial"))
              << " post_anchor_distance: "
              << (best_hypothesis.has_value()
                      ? best_hypothesis->post_anchor_distance
                      : std::numeric_limits<float>::quiet_NaN())
              << RESET;
    if (!best_hypothesis.has_value()) {
      continue;
    }
    endpoint_hypotheses.push_back(*best_hypothesis);

    const double strong_score_threshold =
        0.6 * static_cast<double>(g_loop_icp_score_threshold);
    const float strong_overlap_threshold =
        std::min(
            0.95f,
            std::max(0.80f, g_loop_min_overlap_ratio + 0.35f));
    const float strong_consistency_threshold =
        std::max(0.50f, g_loop_min_consistency_weight);
    const bool strong_endpoint =
        best_hypothesis->score <= strong_score_threshold &&
        best_hypothesis->overlap_ratio >= strong_overlap_threshold &&
        best_hypothesis->consistency.weight >=
            strong_consistency_threshold &&
        best_hypothesis->geometry.valid &&
        best_hypothesis->geometry.structural_valid &&
        best_hypothesis->geometry.symmetric_overlap >= 0.60f &&
        best_hypothesis->geometry.confidence >= 0.60f;
    endpoint_hypotheses.back().strong = strong_endpoint;
    if (strong_endpoint) {
      LOG(INFO) << GREEN
                << " ---> 强终点回环已通过完整验证，"
                << "继续枚举独立候选，不赋予图优化保护特权。"
                << " candidate_idx: " << best_hypothesis->index
                << " score: " << best_hypothesis->score
                << " overlap_ratio: " << best_hypothesis->overlap_ratio
                << " consistency_weight: "
                << best_hypothesis->consistency.weight << RESET;
    }
  }

  // The final keyframe has no special semantic relationship with the first
  // keyframe. Temporally adjacent target windows reuse almost all points and
  // therefore form one observation, not several independent loop edges. A
  // real revisit normally must nevertheless produce the same corrected
  // endpoint from at least three of those windows. A much tighter pair of
  // strict registrations is retained only as a deferred hypothesis; it is
  // not accepted unless the independent internal-only graph predicts it.
  // Keep exactly one transform medoid from every retained cluster.
  std::sort(
      endpoint_hypotheses.begin(), endpoint_hypotheses.end(),
      [](const EndpointHypothesis& lhs, const EndpointHypothesis& rhs) {
        return lhs.index < rhs.index;
      });
  EndpointHypothesisVector distinct_endpoint_hypotheses;
  constexpr float kEndpointConsensusTranslation = 0.50f;
  constexpr float kEndpointConsensusYawDeg = 0.75f;
  constexpr float kTwoVoteStrictTranslation = 0.05f;
  constexpr float kTwoVoteStrictYawDeg = 0.15f;
  const Eigen::Vector3f endpoint_raw_anchor =
      raw_poses[end_idx].t_.cast<float>();
  const auto endpoint_anchor_delta = [&] (
      const EndpointHypothesis& hypothesis) -> Eigen::Vector3f {
    const Eigen::Matrix3f rotation =
        hypothesis.full_correction.block<3, 3>(0, 0);
    const Eigen::Vector3f translation =
        hypothesis.full_correction.block<3, 1>(0, 3);
    return rotation * endpoint_raw_anchor + translation -
        endpoint_raw_anchor;
  };
  const auto endpoint_signed_yaw_deg = [] (
      const EndpointHypothesis& hypothesis) {
    return std::atan2(
               hypothesis.full_correction(1, 0),
               hypothesis.full_correction(0, 0)) *
        180.0f / static_cast<float>(M_PI);
  };
  const auto yaw_distance_deg = [] (
      const float lhs, const float rhs) {
    float difference = std::fmod(std::abs(lhs - rhs), 360.0f);
    if (difference > 180.0f) {
      difference = 360.0f - difference;
    }
    return difference;
  };
  const auto endpoint_hypothesis_is_strict = [] (
      const EndpointHypothesis& hypothesis) {
    return !hypothesis.partial_geometry &&
        hypothesis.geometry.valid && hypothesis.raw_consistent;
  };

  struct EndpointCorridorSequenceEvidence
  {
    bool valid = false;
    int matches = 0;
    float span = 0.0f;
    float rmse = std::numeric_limits<float>::max();
    float density = 0.0f;
    float source_linearity = 0.0f;
    float target_linearity = 0.0f;
    float tangent_mismatch_deg =
        std::numeric_limits<float>::max();
    Eigen::Vector2f target_axis = Eigen::Vector2f::Zero();
  };
  const auto evaluate_endpoint_corridor_sequence = [&] (
      const EndpointHypothesis& hypothesis) {
    EndpointCorridorSequenceEvidence result;
    if (!g_loop_endpoint_corridor_partial_enable ||
        !endpoint_hypothesis_is_strict(hypothesis)) {
      return result;
    }

    const int half_window = std::max(8, 2 * g_loop_local_window_size);
    const int source_begin = std::max(0, end_idx - half_window);
    const int target_begin = std::max(0, hypothesis.index - half_window);
    const int target_end = std::min(
        end_idx - g_loop_keyframe_min_gap,
        hypothesis.index + half_window);
    if (source_begin >= end_idx || target_begin >= target_end) {
      return result;
    }

    struct SequencePoint
    {
      int index = -1;
      Eigen::Vector2f xy = Eigen::Vector2f::Zero();
    };
    std::vector<SequencePoint> source_points;
    std::vector<SequencePoint> target_points;
    source_points.reserve(
        static_cast<std::size_t>(end_idx - source_begin + 1));
    target_points.reserve(
        static_cast<std::size_t>(target_end - target_begin + 1));
    const Eigen::Matrix3f correction_rotation =
        hypothesis.full_correction.block<3, 3>(0, 0);
    const Eigen::Vector3f correction_translation =
        hypothesis.full_correction.block<3, 1>(0, 3);
    for (int index = source_begin; index <= end_idx; ++index) {
      const Eigen::Vector3f corrected =
          correction_rotation * raw_poses[index].t_.cast<float>() +
          correction_translation;
      source_points.push_back({
          index, Eigen::Vector2f(corrected.x(), corrected.y())});
    }
    for (int index = target_begin; index <= target_end; ++index) {
      const auto& translation = raw_poses[index].t_;
      target_points.push_back({
          index,
          Eigen::Vector2f(
              static_cast<float>(translation.x()),
              static_cast<float>(translation.y()))});
    }

    const float maximum_pair_distance = std::max(
        0.75f, 1.5f * g_loop_keyframe_min_distance);
    const float maximum_pair_distance_squared =
        maximum_pair_distance * maximum_pair_distance;
    std::vector<int> source_nearest_target(
        source_points.size(), -1);
    std::vector<float> source_nearest_distance_squared(
        source_points.size(), std::numeric_limits<float>::max());
    std::vector<int> target_nearest_source(
        target_points.size(), -1);
    std::vector<float> target_nearest_distance_squared(
        target_points.size(), std::numeric_limits<float>::max());
    for (std::size_t source_index = 0;
         source_index < source_points.size();
         ++source_index) {
      for (std::size_t target_index = 0;
           target_index < target_points.size();
           ++target_index) {
        const float squared_distance =
            (source_points[source_index].xy -
             target_points[target_index].xy).squaredNorm();
        if (squared_distance <
            source_nearest_distance_squared[source_index]) {
          source_nearest_distance_squared[source_index] = squared_distance;
          source_nearest_target[source_index] =
              static_cast<int>(target_index);
        }
        if (squared_distance <
            target_nearest_distance_squared[target_index]) {
          target_nearest_distance_squared[target_index] = squared_distance;
          target_nearest_source[target_index] =
              static_cast<int>(source_index);
        }
      }
    }

    struct SequencePair
    {
      int source_offset = -1;
      int target_offset = -1;
      float squared_distance = std::numeric_limits<float>::max();
    };
    std::vector<SequencePair> pairs;
    for (std::size_t source_index = 0;
         source_index < source_points.size();
         ++source_index) {
      const int target_index = source_nearest_target[source_index];
      if (target_index < 0 ||
          source_nearest_distance_squared[source_index] >
              maximum_pair_distance_squared ||
          target_nearest_source[static_cast<std::size_t>(target_index)] !=
              static_cast<int>(source_index)) {
        continue;
      }
      pairs.push_back({
          static_cast<int>(source_index), target_index,
          source_nearest_distance_squared[source_index]});
    }
    if (pairs.size() < 5U) {
      return result;
    }

    // Find the longest ordered mutual-nearest chain in either traversal
    // direction.  Reversing through the same corridor is therefore valid,
    // while independently good registrations that slide to different parts
    // of a repetitive wall do not become extra votes.
    std::vector<int> best_chain;
    const float maximum_step_path = std::max(
        2.0f, 4.0f * g_loop_keyframe_min_distance);
    for (const int direction : {-1, 1}) {
      std::vector<int> length(pairs.size(), 1);
      std::vector<int> parent(pairs.size(), -1);
      int best_end = 0;
      for (std::size_t i = 0; i < pairs.size(); ++i) {
        const int source_pose_i =
            source_points[static_cast<std::size_t>(
                pairs[i].source_offset)].index;
        const int target_pose_i =
            target_points[static_cast<std::size_t>(
                pairs[i].target_offset)].index;
        for (std::size_t j = 0; j < i; ++j) {
          const int source_pose_j =
              source_points[static_cast<std::size_t>(
                  pairs[j].source_offset)].index;
          const int target_pose_j =
              target_points[static_cast<std::size_t>(
                  pairs[j].target_offset)].index;
          if (direction * (target_pose_i - target_pose_j) <= 0) {
            continue;
          }
          const float source_step =
              cumulative_route_length[source_pose_i] -
              cumulative_route_length[source_pose_j];
          const float target_step = std::abs(
              cumulative_route_length[target_pose_i] -
              cumulative_route_length[target_pose_j]);
          if (source_step <= 0.0f ||
              source_step > maximum_step_path ||
              target_step > maximum_step_path) {
            continue;
          }
          if (length[j] + 1 > length[i]) {
            length[i] = length[j] + 1;
            parent[i] = static_cast<int>(j);
          }
        }
        if (length[i] > length[static_cast<std::size_t>(best_end)]) {
          best_end = static_cast<int>(i);
        }
      }
      std::vector<int> chain;
      for (int cursor = best_end; cursor >= 0;
           cursor = parent[static_cast<std::size_t>(cursor)]) {
        chain.push_back(cursor);
      }
      std::reverse(chain.begin(), chain.end());
      if (chain.size() > best_chain.size()) {
        best_chain = std::move(chain);
      }
    }
    if (best_chain.size() < 5U) {
      return result;
    }

    std::vector<Eigen::Vector2f> matched_source;
    std::vector<Eigen::Vector2f> matched_target;
    matched_source.reserve(best_chain.size());
    matched_target.reserve(best_chain.size());
    double squared_error_sum = 0.0;
    for (const int pair_index : best_chain) {
      const auto& pair = pairs[static_cast<std::size_t>(pair_index)];
      matched_source.push_back(
          source_points[static_cast<std::size_t>(
              pair.source_offset)].xy);
      matched_target.push_back(
          target_points[static_cast<std::size_t>(
              pair.target_offset)].xy);
      squared_error_sum += pair.squared_distance;
    }
    const auto line_metrics = [] (
        const std::vector<Eigen::Vector2f>& points,
        Eigen::Vector2f& axis) {
      Eigen::Vector2f center = Eigen::Vector2f::Zero();
      for (const auto& point : points) {
        center += point;
      }
      center /= static_cast<float>(points.size());
      Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
      for (const auto& point : points) {
        const Eigen::Vector2f delta = point - center;
        covariance.noalias() += delta * delta.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
      if (solver.info() != Eigen::Success) {
        axis.setZero();
        return 0.0f;
      }
      const Eigen::Vector2f eigenvalues = solver.eigenvalues();
      axis = solver.eigenvectors().col(1).normalized();
      return eigenvalues(1) /
          std::max(eigenvalues.sum(), 1.0e-6f);
    };
    Eigen::Vector2f source_axis;
    Eigen::Vector2f target_axis;
    result.source_linearity = line_metrics(matched_source, source_axis);
    result.target_linearity = line_metrics(matched_target, target_axis);
    const float absolute_axis_dot = std::clamp(
        std::abs(source_axis.dot(target_axis)), 0.0f, 1.0f);
    result.tangent_mismatch_deg =
        std::acos(absolute_axis_dot) *
        180.0f / static_cast<float>(M_PI);
    result.target_axis = target_axis;
    result.matches = static_cast<int>(best_chain.size());
    result.rmse = static_cast<float>(std::sqrt(
        squared_error_sum / static_cast<double>(best_chain.size())));

    const auto& first_pair =
        pairs[static_cast<std::size_t>(best_chain.front())];
    const auto& last_pair =
        pairs[static_cast<std::size_t>(best_chain.back())];
    const int first_source_pose =
        source_points[static_cast<std::size_t>(
            first_pair.source_offset)].index;
    const int last_source_pose =
        source_points[static_cast<std::size_t>(
            last_pair.source_offset)].index;
    const int first_target_pose =
        target_points[static_cast<std::size_t>(
            first_pair.target_offset)].index;
    const int last_target_pose =
        target_points[static_cast<std::size_t>(
            last_pair.target_offset)].index;
    const float source_span = std::abs(
        cumulative_route_length[last_source_pose] -
        cumulative_route_length[first_source_pose]);
    const float target_span = std::abs(
        cumulative_route_length[last_target_pose] -
        cumulative_route_length[first_target_pose]);
    result.span = std::min(source_span, target_span);
    const float expected_samples =
        result.span / std::max(g_loop_keyframe_min_distance, 0.1f) + 1.0f;
    result.density = static_cast<float>(result.matches) /
        std::max(expected_samples, 1.0f);

    const float minimum_span = std::max(
        2.0f, 4.0f * g_loop_keyframe_min_distance);
    const float maximum_rmse = std::max(
        0.35f, 0.70f * g_loop_keyframe_min_distance);
    result.valid = result.matches >= 5 &&
        result.span >= minimum_span && result.rmse <= maximum_rmse &&
        result.density >= 0.60f &&
        result.source_linearity >= 0.94f &&
        result.target_linearity >= 0.94f &&
        result.tangent_mismatch_deg <= 6.0f &&
        result.target_axis.allFinite() &&
        result.target_axis.norm() >= 0.9f;
    return result;
  };

  for (auto& hypothesis : endpoint_hypotheses) {
    const auto sequence =
        evaluate_endpoint_corridor_sequence(hypothesis);
    hypothesis.corridor_sequence_verified = sequence.valid;
    hypothesis.corridor_sequence_matches = sequence.matches;
    hypothesis.corridor_sequence_span = sequence.span;
    hypothesis.corridor_sequence_rmse = sequence.rmse;
    hypothesis.corridor_sequence_density = sequence.density;
    hypothesis.corridor_tangent_mismatch_deg =
        sequence.tangent_mismatch_deg;
    hypothesis.corridor_axis_x = sequence.target_axis.x();
    hypothesis.corridor_axis_y = sequence.target_axis.y();
    LOG(INFO) << GREEN
              << " ---> 终点走廊有序轨迹复核。candidate_idx: "
              << hypothesis.index
              << " matches: " << sequence.matches
              << " matched_span: " << sequence.span
              << " rmse: " << sequence.rmse
              << " density: " << sequence.density
              << " source_linearity: " << sequence.source_linearity
              << " target_linearity: " << sequence.target_linearity
              << " tangent_mismatch_deg: "
              << sequence.tangent_mismatch_deg
              << " corridor_axis_xy: "
              << sequence.target_axis.x() << " "
              << sequence.target_axis.y()
              << " accepted: " << sequence.valid << RESET;
  }
  for (std::size_t begin = 0;
       begin < endpoint_hypotheses.size();) {
    std::size_t end = begin + 1;
    while (end < endpoint_hypotheses.size() &&
           endpoint_hypotheses[end].index -
               endpoint_hypotheses[begin].index <=
               endpoint_cluster_gap) {
      ++end;
    }

    std::size_t medoid = begin;
    int medoid_consensus_count = 0;
    float medoid_consensus_cost =
        std::numeric_limits<float>::max();
    for (std::size_t i = begin; i < end; ++i) {
      const Eigen::Vector3f candidate_delta =
          endpoint_anchor_delta(endpoint_hypotheses[i]);
      const float candidate_yaw =
          endpoint_signed_yaw_deg(endpoint_hypotheses[i]);
      int consensus_count = 0;
      float consensus_cost = 0.0f;
      for (std::size_t j = begin; j < end; ++j) {
        const float translation_difference =
            (candidate_delta -
             endpoint_anchor_delta(endpoint_hypotheses[j])).norm();
        const float yaw_difference = yaw_distance_deg(
            candidate_yaw,
            endpoint_signed_yaw_deg(endpoint_hypotheses[j]));
        if (translation_difference > kEndpointConsensusTranslation ||
            yaw_difference > kEndpointConsensusYawDeg) {
          continue;
        }
        ++consensus_count;
        consensus_cost +=
            translation_difference / kEndpointConsensusTranslation +
            yaw_difference / kEndpointConsensusYawDeg;
      }
      if (consensus_count > medoid_consensus_count ||
          (consensus_count == medoid_consensus_count &&
           (consensus_cost < medoid_consensus_cost - 1.0e-6f ||
            (std::abs(consensus_cost - medoid_consensus_cost) <=
                 1.0e-6f &&
             endpoint_hypothesis_is_better(
                 endpoint_hypotheses[i],
                 endpoint_hypotheses[medoid]))))) {
        medoid = i;
        medoid_consensus_count = consensus_count;
        medoid_consensus_cost = consensus_cost;
      }
    }

    // Preserve the normal three-window quorum. A pair of independently
    // registered strict windows may only survive as a provisional hypothesis
    // when their corrections are much tighter than the normal consensus
    // tolerance. The internal-only graph must still authorize it later.
    std::size_t strict_medoid = begin;
    int strict_medoid_consensus_count = 0;
    float strict_medoid_consensus_cost =
        std::numeric_limits<float>::max();
    for (std::size_t i = begin; i < end; ++i) {
      if (!endpoint_hypothesis_is_strict(endpoint_hypotheses[i])) {
        continue;
      }
      const Eigen::Vector3f candidate_delta =
          endpoint_anchor_delta(endpoint_hypotheses[i]);
      const float candidate_yaw =
          endpoint_signed_yaw_deg(endpoint_hypotheses[i]);
      int consensus_count = 0;
      float consensus_cost = 0.0f;
      for (std::size_t j = begin; j < end; ++j) {
        if (!endpoint_hypothesis_is_strict(endpoint_hypotheses[j])) {
          continue;
        }
        const float translation_difference =
            (candidate_delta -
             endpoint_anchor_delta(endpoint_hypotheses[j])).norm();
        const float yaw_difference = yaw_distance_deg(
            candidate_yaw,
            endpoint_signed_yaw_deg(endpoint_hypotheses[j]));
        if (translation_difference > kTwoVoteStrictTranslation ||
            yaw_difference > kTwoVoteStrictYawDeg) {
          continue;
        }
        ++consensus_count;
        consensus_cost +=
            translation_difference / kTwoVoteStrictTranslation +
            yaw_difference / kTwoVoteStrictYawDeg;
      }
      if (consensus_count > strict_medoid_consensus_count ||
          (consensus_count == strict_medoid_consensus_count &&
           (consensus_cost < strict_medoid_consensus_cost - 1.0e-6f ||
            (std::abs(consensus_cost - strict_medoid_consensus_cost) <=
                 1.0e-6f &&
             endpoint_hypothesis_is_better(
                 endpoint_hypotheses[i],
                 endpoint_hypotheses[strict_medoid]))))) {
        strict_medoid = i;
        strict_medoid_consensus_count = consensus_count;
        strict_medoid_consensus_cost = consensus_cost;
      }
    }

    const bool standard_cluster_consistent =
        medoid_consensus_count >= kMinimumEndpointConsensus;
    const bool two_vote_strict_provisional =
        !standard_cluster_consistent &&
        strict_medoid_consensus_count >= 2;
    int strict_place_candidates = 0;
    int corridor_sequence_candidates = 0;
    std::size_t corridor_sequence_medoid = begin;
    double best_corridor_sequence_cost =
        std::numeric_limits<double>::max();
    for (std::size_t i = begin; i < end; ++i) {
      const auto& hypothesis = endpoint_hypotheses[i];
      if (!endpoint_hypothesis_is_strict(hypothesis)) {
        continue;
      }
      ++strict_place_candidates;
      const float minimum_corridor_overlap = std::max(
          0.70f, g_loop_min_symmetric_overlap + 0.25f);
      const float maximum_corridor_rmse = std::min(
          0.18f, g_loop_max_trimmed_rmse);
      const float minimum_corridor_structural_overlap = std::max(
          0.65f, g_loop_min_structural_overlap + 0.30f);
      const float maximum_corridor_post_anchor = std::max(
          0.75f, 1.5f * g_loop_keyframe_min_distance);
      const bool geometry_uniquely_supports_normal =
          hypothesis.corridor_sequence_verified &&
          hypothesis.geometry.structural_valid &&
          hypothesis.geometry.symmetric_overlap >=
              minimum_corridor_overlap &&
          hypothesis.geometry.symmetric_trimmed_rmse <=
              maximum_corridor_rmse &&
          hypothesis.geometry.structural_symmetric_overlap >=
              minimum_corridor_structural_overlap &&
          hypothesis.geometry.structural_symmetric_trimmed_rmse <=
              maximum_corridor_rmse &&
          hypothesis.post_anchor_distance <=
              maximum_corridor_post_anchor;
      if (!geometry_uniquely_supports_normal) {
        continue;
      }
      ++corridor_sequence_candidates;
      const double corridor_cost =
          static_cast<double>(hypothesis.corridor_sequence_rmse) +
          0.02 * static_cast<double>(
              hypothesis.corridor_tangent_mismatch_deg) +
          static_cast<double>(
              hypothesis.geometry.symmetric_trimmed_rmse) +
          0.5 * static_cast<double>(
              hypothesis.geometry.structural_symmetric_trimmed_rmse) -
          0.1 * static_cast<double>(
              hypothesis.geometry.symmetric_overlap);
      if (corridor_cost < best_corridor_sequence_cost) {
        best_corridor_sequence_cost = corridor_cost;
        corridor_sequence_medoid = i;
      }
    }
    // Two strict registrations confirm the physical place.  Exactly one must
    // also align an ordered straight trajectory and pass the stronger
    // structural test; otherwise the repeated corridor is still ambiguous.
    const bool corridor_sequence_partial =
        g_loop_endpoint_corridor_partial_enable &&
        !standard_cluster_consistent &&
        !two_vote_strict_provisional &&
        strict_place_candidates >= 2 &&
        corridor_sequence_candidates == 1;
    const std::size_t representative_medoid =
        corridor_sequence_partial
        ? corridor_sequence_medoid
        : (two_vote_strict_provisional ? strict_medoid : medoid);
    const int representative_consensus_count =
        corridor_sequence_partial
        ? strict_place_candidates
        : (two_vote_strict_provisional
               ? strict_medoid_consensus_count : medoid_consensus_count);
    const float representative_translation_limit =
        two_vote_strict_provisional
        ? kTwoVoteStrictTranslation : kEndpointConsensusTranslation;
    const float representative_yaw_limit =
        two_vote_strict_provisional
        ? kTwoVoteStrictYawDeg : kEndpointConsensusYawDeg;
    float translation_spread = 0.0f;
    float yaw_spread = 0.0f;
    const Eigen::Vector3f medoid_delta =
        endpoint_anchor_delta(endpoint_hypotheses[representative_medoid]);
    const float medoid_yaw =
        endpoint_signed_yaw_deg(
            endpoint_hypotheses[representative_medoid]);
    int representative_strong_strict_count = 0;
    for (std::size_t i = begin; i < end; ++i) {
      if (two_vote_strict_provisional &&
          !endpoint_hypothesis_is_strict(endpoint_hypotheses[i])) {
        continue;
      }
      const float translation_difference =
          (medoid_delta -
           endpoint_anchor_delta(endpoint_hypotheses[i])).norm();
      const float yaw_difference = yaw_distance_deg(
          medoid_yaw,
          endpoint_signed_yaw_deg(endpoint_hypotheses[i]));
      if (translation_difference <= representative_translation_limit &&
          yaw_difference <= representative_yaw_limit) {
        if (endpoint_hypothesis_is_strict(endpoint_hypotheses[i]) &&
            endpoint_hypotheses[i].strong) {
          ++representative_strong_strict_count;
        }
        translation_spread = std::max(
            translation_spread, translation_difference);
        yaw_spread = std::max(yaw_spread, yaw_difference);
      }
    }
    const bool cluster_retained = standard_cluster_consistent ||
        two_vote_strict_provisional || corridor_sequence_partial;
    LOG(INFO) << GREEN
              << " ---> 终点回环相邻候选修正共识。cluster_first_idx: "
              << endpoint_hypotheses[begin].index
              << " cluster_last_idx: "
              << endpoint_hypotheses[end - 1].index
              << " members: " << end - begin
              << " medoid_idx: "
              << endpoint_hypotheses[representative_medoid].index
              << " consensus_count: " << medoid_consensus_count
              << " strict_tight_consensus_count: "
              << strict_medoid_consensus_count
              << " strong_strict_consensus_count: "
              << representative_strong_strict_count
              << " translation_spread: " << translation_spread
              << " yaw_spread_deg: " << yaw_spread
              << " standard_three_vote_accepted: "
              << standard_cluster_consistent
              << " two_vote_strict_provisional: "
              << two_vote_strict_provisional
              << " strict_place_candidates: "
              << strict_place_candidates
              << " corridor_sequence_candidates: "
              << corridor_sequence_candidates
              << " corridor_sequence_partial: "
              << corridor_sequence_partial
              << " retained_for_deferred_gate: "
              << cluster_retained << RESET;
    if (cluster_retained) {
      EndpointHypothesis representative =
          endpoint_hypotheses[representative_medoid];
      representative.consensus_count = representative_consensus_count;
      representative.consensus_translation_spread =
          translation_spread;
      representative.consensus_yaw_spread_deg = yaw_spread;
      representative.two_vote_strict_provisional =
          two_vote_strict_provisional;
      representative.two_vote_all_strong =
          two_vote_strict_provisional &&
          representative_strong_strict_count >= 2;
      if (corridor_sequence_partial) {
        representative.edge.corridor_partial = true;
        representative.edge.corridor_axis_x =
            representative.corridor_axis_x;
        representative.edge.corridor_axis_y =
            representative.corridor_axis_y;
      }
      distinct_endpoint_hypotheses.push_back(representative);
    }
    begin = end;
  }

  // Endpoint registration clouds are no longer needed once compact
  // hypotheses have been materialized. Release them before internal-loop
  // submaps are allocated; retaining both sets can create a large transient
  // peak on Jetson for long bags.
  for (auto& state : endpoint_states) {
    state.target.reset();
    state.target_ground.reset();
    state.micro_target.reset();
    state.basins.clear();
    state.rejected_basins.clear();
  }
  endpoint_states.clear();
  endpoint_micro_source.reset();
  source_ground.reset();
  source.reset();
  const auto two_vote_provisional_clusters = std::count_if(
      distinct_endpoint_hypotheses.begin(),
      distinct_endpoint_hypotheses.end(),
      [](const EndpointHypothesis& hypothesis) {
        return hypothesis.two_vote_strict_provisional;
      });
  const auto corridor_sequence_partial_clusters = std::count_if(
      distinct_endpoint_hypotheses.begin(),
      distinct_endpoint_hypotheses.end(),
      [](const EndpointHypothesis& hypothesis) {
        return hypothesis.edge.corridor_partial;
      });
  LOG(INFO) << GREEN
            << " ---> 终点回环子图缓存已释放，进入中途回环。"
            << " endpoint_hypotheses: "
            << endpoint_hypotheses.size()
            << " consensus_clusters: "
            << distinct_endpoint_hypotheses.size()
            << " two_vote_strict_provisional_clusters: "
            << two_vote_provisional_clusters
            << " corridor_sequence_partial_clusters: "
            << corridor_sequence_partial_clusters << RESET;

  int candidate_idx = -1;
  float best_horizontal_distance = std::numeric_limits<float>::max();
  double best_score = std::numeric_limits<double>::max();
  double best_selection_cost = std::numeric_limits<double>::max();
  float best_consistency_weight = 0.0f;
  float best_post_anchor_distance =
      std::numeric_limits<float>::quiet_NaN();
  bool best_partial_geometry = false;
  bool best_micro_window_verified = false;
  int best_endpoint_consensus_count = 0;
  bool best_two_vote_strict_provisional = false;
  bool best_corridor_sequence_partial = false;
  bool best_guarded_endpoint_proposal = false;
  SE3 guarded_endpoint_prediction_measurement;
  SE3 guarded_endpoint_full_measurement;
  float guarded_endpoint_alpha = 1.0f;
  Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
  std::optional<PoseGraphEdge> endpoint_edge;
  bool endpoint_edge_strong = false;
  bool endpoint_distributed_anchor_group_active = false;
  // Endpoint hypotheses stay provisional here. Internal loops are validated
  // against the raw trajectory and optimized first; only that independent
  // field may authorize a large endpoint deformation below.
  // Optimize a deformation graph of world-frame correction poses C_i rather
  // than the large absolute poses X_i directly:
  //
  //   X_i = C_i * T_i(raw)
  //
  // Consecutive C_i are connected by identity edges, so the correction is
  // spread smoothly over the route.  If G maps a later raw world cloud onto
  // an earlier raw world cloud, a loop edge measures C_earlier^-1*C_later=G.
  // This is the same global pose-graph idea, but avoids numerical
  // cancellation on 100 m absolute coordinates and makes the intended
  // deformation explicit.
  PoseVector correction_poses(
      loop_keyframes_.size(), SE3());
  PoseGraphEdgeVector pose_graph_edges;
  pose_graph_edges.reserve(loop_keyframes_.size() + 32);
  for (int i = 0; i < end_idx; ++i) {
    PoseGraphEdge edge;
    edge.from = i;
    edge.to = i + 1;
    edge.measurement = SE3();
    edge.weight = 1.0f;
    edge.loop = false;
    pose_graph_edges.push_back(edge);
  }

  struct RevisitCandidate
  {
    int earlier = -1;
    int later = -1;
    int index_gap = 0;
    double sensor_time_separation = 0.0;
    float horizontal_distance =
        std::numeric_limits<float>::max();
    float vertical_distance =
        std::numeric_limits<float>::max();
    float segment_search_radius = 0.0f;
    float discovery_score =
        std::numeric_limits<float>::max();
    float path_length = 0.0f;
    float place_center_x = 0.0f;
    float place_center_y = 0.0f;
    float corridor_tangent_x = 0.0f;
    float corridor_tangent_y = 0.0f;
    float tangent_mismatch_deg =
        std::numeric_limits<float>::max();
    bool corridor_tangent_consistent = false;
    int evidence_group_id = -1;
  };
  std::vector<RevisitCandidate> revisit_candidates;
  // Candidate discovery grows continuously with route length instead of
  // switching at a hard map-size boundary. Final acceptance is still decided
  // by full 3-D registration and graph consistency below.
  const float internal_search_radius =
      std::min(
          g_loop_search_radius,
          g_loop_internal_search_radius +
              g_loop_translation_drift_ratio * raw_route_length);
  LOG(INFO) << GREEN
            << " ---> 中途回环搜索范围。raw_route_length: "
            << raw_route_length
            << " internal_search_radius: "
            << internal_search_radius << RESET;
  constexpr int kEarlierCandidatesPerLater = 3;
  const int revisit_index_separation =
      std::max(30, 3 * g_loop_local_window_size);
  const int earlier_candidate_nms_gap =
      revisit_index_separation;
  // Pair-index proximity alone is not physical duplication.  On a long
  // corridor two revisits can be only a few keyframes apart at both passes,
  // yet observe places many metres apart.  Estimate an unoriented route
  // tangent from a short local baseline so a turn/crossing match cannot hide
  // a same-direction corridor continuation behind index-space NMS.
  const int corridor_tangent_half_window = std::max(
      2, g_loop_local_window_size / 5);
  constexpr float kCorridorTangentMismatchDeg = 15.0f;
  const float corridor_anchor_separation = std::max(
      3.0f,
      g_loop_keyframe_min_distance *
          static_cast<float>(std::max(6, g_loop_local_window_size)));
  const float corridor_cross_track_limit = std::max(
      2.0f, 2.0f * g_loop_internal_search_radius);
  const auto local_route_tangent = [&] (const int index) {
    Eigen::Vector2f tangent = Eigen::Vector2f::Zero();
    const int maximum_half_window = std::max(
        corridor_tangent_half_window, g_loop_local_window_size);
    for (int half_window = corridor_tangent_half_window;
         half_window <= maximum_half_window;
         half_window += corridor_tangent_half_window) {
      const int first = std::max(0, index - half_window);
      const int last = std::min(end_idx, index + half_window);
      const V3 delta = raw_poses[last].t_ - raw_poses[first].t_;
      tangent = Eigen::Vector2f(
          static_cast<float>(delta.x()),
          static_cast<float>(delta.y()));
      if (tangent.norm() >= std::max(
              0.5f, g_loop_keyframe_min_distance)) {
        tangent.normalize();
        return std::make_pair(true, tangent);
      }
    }
    return std::make_pair(false, tangent);
  };
  const auto populate_corridor_descriptor = [&] (
      RevisitCandidate& candidate) {
    const V3 center = 0.5 * (
        raw_poses[candidate.earlier].t_ +
        raw_poses[candidate.later].t_);
    candidate.place_center_x = static_cast<float>(center.x());
    candidate.place_center_y = static_cast<float>(center.y());
    const auto earlier_tangent = local_route_tangent(candidate.earlier);
    const auto later_tangent = local_route_tangent(candidate.later);
    if (!earlier_tangent.first || !later_tangent.first) {
      return;
    }
    Eigen::Vector2f aligned_later = later_tangent.second;
    float tangent_dot = earlier_tangent.second.dot(aligned_later);
    if (tangent_dot < 0.0f) {
      aligned_later = -aligned_later;
      tangent_dot = -tangent_dot;
    }
    tangent_dot = std::clamp(tangent_dot, -1.0f, 1.0f);
    candidate.tangent_mismatch_deg =
        std::acos(tangent_dot) *
        180.0f / static_cast<float>(M_PI);
    Eigen::Vector2f consensus =
        earlier_tangent.second + aligned_later;
    if (consensus.norm() <= 1.0e-3f) {
      return;
    }
    consensus.normalize();
    candidate.corridor_tangent_x = consensus.x();
    candidate.corridor_tangent_y = consensus.y();
    candidate.corridor_tangent_consistent =
        candidate.tangent_mismatch_deg <=
            kCorridorTangentMismatchDeg;
  };
  // This is a travelled-distance gate, not an XYZ displacement gate. It
  // scales with the configured keyframe density and the combined nominal
  // support of both local submaps. Requiring more than the bare center-index
  // gap makes this independent evidence instead of a mathematical duplicate
  // of keyframe sampling. It neither assumes a fixed height nor flattens ramps.
  const int combined_local_support_keyframes =
      2 * (2 * g_loop_local_window_size + 1);
  const float internal_min_path_length =
      g_loop_keyframe_min_distance *
      static_cast<float>(std::max(
          internal_min_index_gap,
          combined_local_support_keyframes));
  struct RevisitPreGateSample
  {
    int earlier = -1;
    int later = -1;
    int index_gap = 0;
    double sensor_time_separation =
        std::numeric_limits<double>::quiet_NaN();
    float path_length = std::numeric_limits<float>::quiet_NaN();
    float horizontal_distance =
        std::numeric_limits<float>::quiet_NaN();
    float segment_search_radius =
        std::numeric_limits<float>::quiet_NaN();
  };
  std::uint64_t pre_gate_total_pairs = 0;
  std::uint64_t pre_gate_rejected_min_gap = 0;
  std::uint64_t pre_gate_rejected_time = 0;
  std::uint64_t pre_gate_rejected_path = 0;
  std::uint64_t pre_gate_rejected_radius = 0;
  std::uint64_t pre_gate_passed = 0;
  std::uint64_t pre_gate_passed_below_endpoint_gap = 0;
  RevisitPreGateSample min_gap_sample;
  RevisitPreGateSample time_sample;
  RevisitPreGateSample path_sample;
  RevisitPreGateSample radius_sample;
  const auto capture_pre_gate_sample =
      [&](RevisitPreGateSample& sample, int earlier, int later) {
        if (sample.earlier >= 0) {
          return;
        }
        sample.earlier = earlier;
        sample.later = later;
        sample.index_gap = later - earlier;
        sample.sensor_time_separation =
            loop_keyframes_[later].timestamp -
            loop_keyframes_[earlier].timestamp;
        sample.path_length =
            cumulative_route_length[later] -
            cumulative_route_length[earlier];
        const V3 delta =
            loop_keyframes_[later].pose.t_ -
            loop_keyframes_[earlier].pose.t_;
        sample.horizontal_distance =
            static_cast<float>(std::hypot(delta.x(), delta.y()));
        sample.segment_search_radius =
            std::min(
                internal_search_radius,
                g_loop_internal_search_radius +
                    g_loop_translation_drift_ratio * sample.path_length);
      };
  int later_frames_with_multiple_earlier = 0;
  for (int later = 1;
       later < end_idx - g_loop_local_window_size;
       ++later) {
    pre_gate_total_pairs += static_cast<std::uint64_t>(later);
    const int min_gap_rejections_for_later =
        std::min(later, internal_min_index_gap - 1);
    pre_gate_rejected_min_gap +=
        static_cast<std::uint64_t>(min_gap_rejections_for_later);
    if (min_gap_sample.earlier < 0 &&
        later >= internal_min_index_gap - 1) {
      capture_pre_gate_sample(
          min_gap_sample,
          later - (internal_min_index_gap - 1), later);
    }
    std::vector<RevisitCandidate> candidates_for_later;
    const int latest_earlier =
        later - internal_min_index_gap;
    if (latest_earlier < 0) {
      continue;
    }
    for (int earlier = 0; earlier <= latest_earlier; ++earlier) {
      const double sensor_time_separation =
          loop_keyframes_[later].timestamp -
          loop_keyframes_[earlier].timestamp;
      if (!std::isfinite(sensor_time_separation) ||
          sensor_time_separation <
              g_loop_internal_min_sensor_time_seconds) {
        ++pre_gate_rejected_time;
        capture_pre_gate_sample(time_sample, earlier, later);
        continue;
      }
      const float segment_path_length =
          cumulative_route_length[later] -
          cumulative_route_length[earlier];
      if (segment_path_length < internal_min_path_length) {
        ++pre_gate_rejected_path;
        capture_pre_gate_sample(path_sample, earlier, later);
        continue;
      }
      const V3 delta =
          loop_keyframes_[later].pose.t_ -
          loop_keyframes_[earlier].pose.t_;
      const float horizontal_distance =
          static_cast<float>(std::hypot(delta.x(), delta.y()));
      // A short local segment must not inherit the full-map search radius.
      // Grow its discovery radius only with the travelled distance between
      // the two observations. Long-return loops can still use the complete
      // adaptive radius, while nearby parallel aisles stay out of the pool.
      const float segment_search_radius =
          std::min(
              internal_search_radius,
              g_loop_internal_search_radius +
                  g_loop_translation_drift_ratio * segment_path_length);
      if (horizontal_distance >= segment_search_radius) {
        ++pre_gate_rejected_radius;
        capture_pre_gate_sample(radius_sample, earlier, later);
        continue;
      }
      ++pre_gate_passed;
      const int index_gap = later - earlier;
      if (index_gap < g_loop_keyframe_min_gap) {
        ++pre_gate_passed_below_endpoint_gap;
      }
      const float vertical_distance =
          static_cast<float>(std::abs(delta.z()));
      // Z is only a soft ordering hint. It helps a same-level alternative
      // precede an XY-identical different-floor alias, but never rejects a
      // candidate; ramps and real floor changes still reach full 3-D ICP.
      const float discovery_score =
          horizontal_distance +
          0.15f * std::min(vertical_distance, internal_search_radius);
      RevisitCandidate candidate;
      candidate.earlier = earlier;
      candidate.later = later;
      candidate.index_gap = index_gap;
      candidate.sensor_time_separation = sensor_time_separation;
      candidate.horizontal_distance = horizontal_distance;
      candidate.vertical_distance = vertical_distance;
      candidate.segment_search_radius = segment_search_radius;
      candidate.discovery_score = discovery_score;
      candidate.path_length = segment_path_length;
      populate_corridor_descriptor(candidate);
      candidates_for_later.push_back(candidate);
    }

    std::sort(
        candidates_for_later.begin(), candidates_for_later.end(),
        [](const RevisitCandidate& lhs,
           const RevisitCandidate& rhs) {
          if (lhs.discovery_score != rhs.discovery_score) {
            return lhs.discovery_score < rhs.discovery_score;
          }
          if (lhs.horizontal_distance != rhs.horizontal_distance) {
            return lhs.horizontal_distance < rhs.horizontal_distance;
          }
          return lhs.earlier < rhs.earlier;
        });
    std::vector<int> selected_earlier_indices;
    selected_earlier_indices.reserve(kEarlierCandidatesPerLater);
    for (const auto& candidate : candidates_for_later) {
      bool suppressed = false;
      for (const int selected_earlier : selected_earlier_indices) {
        if (std::abs(candidate.earlier - selected_earlier) <=
            earlier_candidate_nms_gap) {
          suppressed = true;
          break;
        }
      }
      if (suppressed) {
        continue;
      }
      revisit_candidates.push_back(candidate);
      selected_earlier_indices.push_back(candidate.earlier);
      if (static_cast<int>(selected_earlier_indices.size()) >=
          kEarlierCandidatesPerLater) {
        break;
      }
    }
    if (selected_earlier_indices.size() > 1) {
      ++later_frames_with_multiple_earlier;
    }
  }
  LOG(INFO) << GREEN
            << " ---> 中途回环 pre-gate 统计。total_pairs: "
            << pre_gate_total_pairs
            << " internal_min_index_gap: "
            << internal_min_index_gap
            << " min_sensor_time_seconds: "
            << g_loop_internal_min_sensor_time_seconds
            << " min_path_length: " << internal_min_path_length
            << " rejected_min_gap: " << pre_gate_rejected_min_gap
            << " rejected_time: " << pre_gate_rejected_time
            << " rejected_path: " << pre_gate_rejected_path
            << " rejected_radius: " << pre_gate_rejected_radius
            << " passed: " << pre_gate_passed
            << " passed_below_endpoint_gap: "
            << pre_gate_passed_below_endpoint_gap << RESET;
  const auto log_pre_gate_sample =
      [](const char* reason, const RevisitPreGateSample& sample) {
        if (sample.earlier < 0) {
          return;
        }
        LOG(INFO) << GREEN
                  << " ---> 中途回环 pre-gate 拒绝样例。reason: "
                  << reason
                  << " earlier_idx: " << sample.earlier
                  << " later_idx: " << sample.later
                  << " index_gap: " << sample.index_gap
                  << " sensor_dt: "
                  << sample.sensor_time_separation
                  << " path_length: " << sample.path_length
                  << " horizontal_distance: "
                  << sample.horizontal_distance
                  << " segment_search_radius: "
                  << sample.segment_search_radius << RESET;
      };
  log_pre_gate_sample("min_gap", min_gap_sample);
  log_pre_gate_sample("sensor_time", time_sample);
  log_pre_gate_sample("path_length", path_sample);
  log_pre_gate_sample("search_radius", radius_sample);
  std::sort(
      revisit_candidates.begin(),
      revisit_candidates.end(),
      [](const RevisitCandidate& lhs, const RevisitCandidate& rhs) {
        if (lhs.discovery_score != rhs.discovery_score) {
          return lhs.discovery_score < rhs.discovery_score;
        }
        return lhs.horizontal_distance < rhs.horizontal_distance;
      });
  std::vector<RevisitCandidate> selected_revisits;
  const int max_selected_revisits =
      std::min(30, g_loop_candidate_limit);
  const int corridor_coverage_quota = std::min(
      6, (max_selected_revisits + 4) / 5);
  const int closest_revisit_quota =
      std::min(
          12,
          std::max(
              0, max_selected_revisits - corridor_coverage_quota));
  const auto pair_windows_overlap = [&] (
      const RevisitCandidate& lhs,
      const RevisitCandidate& rhs) {
    return std::abs(lhs.later - rhs.later) <
               revisit_index_separation &&
        std::abs(lhs.earlier - rhs.earlier) <
               revisit_index_separation;
  };
  const auto corridor_anchor_is_distinct = [&] (
      const RevisitCandidate& candidate,
      const RevisitCandidate& selected,
      float* along_track,
      float* cross_track,
      float* pair_tangent_difference_deg) {
    if (!candidate.corridor_tangent_consistent) {
      return false;
    }
    const Eigen::Vector2f tangent(
        candidate.corridor_tangent_x,
        candidate.corridor_tangent_y);
    const Eigen::Vector2f normal(-tangent.y(), tangent.x());
    const Eigen::Vector2f place_delta(
        candidate.place_center_x - selected.place_center_x,
        candidate.place_center_y - selected.place_center_y);
    const float along = std::abs(place_delta.dot(tangent));
    const float cross = std::abs(place_delta.dot(normal));
    float tangent_difference = 0.0f;
    if (selected.corridor_tangent_consistent) {
      const Eigen::Vector2f selected_tangent(
          selected.corridor_tangent_x,
          selected.corridor_tangent_y);
      tangent_difference = std::acos(std::clamp(
          std::abs(tangent.dot(selected_tangent)), 0.0f, 1.0f)) *
          180.0f / static_cast<float>(M_PI);
    }
    if (along_track) {
      *along_track = along;
    }
    if (cross_track) {
      *cross_track = cross;
    }
    if (pair_tangent_difference_deg) {
      *pair_tangent_difference_deg = tangent_difference;
    }
    return along >= corridor_anchor_separation &&
        cross <= corridor_cross_track_limit &&
        (!selected.corridor_tangent_consistent ||
         tangent_difference <= kCorridorTangentMismatchDeg);
  };
  const auto pair_is_strictly_separated =
      [&](const RevisitCandidate& candidate) {
        for (const auto& selected : selected_revisits) {
          if (pair_windows_overlap(candidate, selected)) {
            return false;
          }
        }
        return true;
      };
  const auto corridor_candidate_is_separated =
      [&](const RevisitCandidate& candidate) {
        for (const auto& selected : selected_revisits) {
          if (!pair_windows_overlap(candidate, selected)) {
            continue;
          }
          // Two index-overlapping windows remain separate anchors only when
          // the new pair itself follows the same unoriented route tangent and
          // has advanced far enough along that physical corridor.  A crossing
          // or turn pair may therefore be tried, but cannot suppress the
          // straight-corridor evidence that follows it.
          if (corridor_anchor_is_distinct(
                  candidate, selected, nullptr, nullptr, nullptr)) {
            continue;
          }
          return false;
        }
        return true;
      };

  // Keep the strongest distance-ranked candidates first. These are cheap to
  // register and preserve the strict pair-index NMS behaviour that worked for
  // compact maps. Only the explicitly reserved corridor quota below may use
  // a physical-continuation exception.
  for (const auto& candidate : revisit_candidates) {
    if (!pair_is_strictly_separated(candidate)) {
      continue;
    }
    selected_revisits.push_back(candidate);
    if (static_cast<int>(selected_revisits.size()) >=
        closest_revisit_quota) {
      break;
    }
  }

  struct CorridorCoverageSelection
  {
    int earlier = -1;
    int later = -1;
    int related_earlier = -1;
    int related_later = -1;
    float tangent_mismatch_deg = 0.0f;
    float related_tangent_mismatch_deg = 0.0f;
    float along_track = 0.0f;
    float cross_track = 0.0f;
    float related_tangent_difference_deg = 0.0f;
  };
  std::vector<CorridorCoverageSelection> corridor_coverage_selections;
  const auto fill_pair_index_coverage = [&] (const int target_size) {
    // Distance-only ranking over-selects stops and small local circuits. Fill
    // by farthest-point sampling on pair index, but stop before the complete
    // budget so physical-corridor continuation has reserved slots.
    while (static_cast<int>(selected_revisits.size()) < target_size) {
      int best_index = -1;
      float best_coverage_score = -1.0f;
      for (std::size_t i = 0; i < revisit_candidates.size(); ++i) {
        const auto& candidate = revisit_candidates[i];
        if (!pair_is_strictly_separated(candidate)) {
          continue;
        }
        float nearest_selected_gap =
            static_cast<float>(end_idx);
        for (const auto& selected : selected_revisits) {
          const float earlier_gap = static_cast<float>(
              std::abs(candidate.earlier - selected.earlier));
          const float later_gap = static_cast<float>(
              std::abs(candidate.later - selected.later));
          nearest_selected_gap = std::min(
              nearest_selected_gap,
              std::hypot(earlier_gap, later_gap));
        }
        const float distance_quality =
            1.0f /
            (1.0f + candidate.discovery_score /
                std::max(internal_search_radius, 0.1f));
        const float coverage_score =
            nearest_selected_gap * distance_quality;
        if (coverage_score > best_coverage_score) {
          best_coverage_score = coverage_score;
          best_index = static_cast<int>(i);
        }
      }
      if (best_index < 0) {
        break;
      }
      selected_revisits.push_back(revisit_candidates[best_index]);
    }
  };
  const auto fill_corridor_coverage = [&] (const int target_size) {
    while (static_cast<int>(selected_revisits.size()) < target_size) {
      int best_index = -1;
      int best_related_index = -1;
      float best_score = -1.0f;
      float best_along = 0.0f;
      float best_cross = 0.0f;
      float best_tangent_difference = 0.0f;
      for (std::size_t i = 0; i < revisit_candidates.size(); ++i) {
        const auto& candidate = revisit_candidates[i];
        if (!candidate.corridor_tangent_consistent ||
            !corridor_candidate_is_separated(candidate)) {
          continue;
        }
        for (std::size_t selected_index = 0;
             selected_index < selected_revisits.size();
             ++selected_index) {
          const auto& selected = selected_revisits[selected_index];
          if (!pair_windows_overlap(candidate, selected)) {
            continue;
          }
          float along = 0.0f;
          float cross = 0.0f;
          float tangent_difference = 0.0f;
          if (!corridor_anchor_is_distinct(
                  candidate, selected, &along, &cross,
                  &tangent_difference)) {
            continue;
          }
          const float distance_quality =
              1.0f /
              (1.0f + candidate.discovery_score /
                  std::max(internal_search_radius, 0.1f));
          const float tangent_quality = std::clamp(
              1.0f - candidate.tangent_mismatch_deg /
                  kCorridorTangentMismatchDeg,
              0.0f, 1.0f);
          // An index-selected turn/crossing pair is a poor representative of
          // the shared corridor.  Prefer one aligned continuation for that
          // evidence neighbourhood before adding second/third continuations
          // to an already well represented straight segment.
          const float crossing_repair_priority =
              selected.corridor_tangent_consistent
              ? 1.0f
              : 1.0f + std::min(
                    1.5f,
                    selected.tangent_mismatch_deg /
                        kCorridorTangentMismatchDeg);
          const float coverage_score =
              std::min(
                  along, 3.0f * corridor_anchor_separation) *
              distance_quality * (0.5f + 0.5f * tangent_quality) *
              crossing_repair_priority;
          if (coverage_score > best_score) {
            best_score = coverage_score;
            best_index = static_cast<int>(i);
            best_related_index = static_cast<int>(selected_index);
            best_along = along;
            best_cross = cross;
            best_tangent_difference = tangent_difference;
          }
        }
      }
      if (best_index < 0 || best_related_index < 0) {
        break;
      }
      const auto selected = revisit_candidates[
          static_cast<std::size_t>(best_index)];
      const auto& related = selected_revisits[
          static_cast<std::size_t>(best_related_index)];
      CorridorCoverageSelection selection;
      selection.earlier = selected.earlier;
      selection.later = selected.later;
      selection.related_earlier = related.earlier;
      selection.related_later = related.later;
      selection.tangent_mismatch_deg =
          selected.tangent_mismatch_deg;
      selection.related_tangent_mismatch_deg =
          related.tangent_mismatch_deg;
      selection.along_track = best_along;
      selection.cross_track = best_cross;
      selection.related_tangent_difference_deg =
          best_tangent_difference;
      corridor_coverage_selections.push_back(selection);
      selected_revisits.push_back(selected);
    }
  };
  // First establish broad pair-index coverage, then spend the reserved part of
  // the same <=30 budget on corridor continuations relative to those anchors.
  // If no valid corridor continuation exists, return the unused slots to the
  // generic farthest-point selector.
  const int pair_index_coverage_target = std::max(
      closest_revisit_quota,
      max_selected_revisits - corridor_coverage_quota);
  fill_pair_index_coverage(pair_index_coverage_target);
  const std::size_t pair_index_selected_count =
      selected_revisits.size();
  fill_corridor_coverage(max_selected_revisits);
  const std::size_t corridor_selected_end =
      selected_revisits.size();
  if (corridor_selected_end > pair_index_selected_count &&
      pair_index_selected_count >
          static_cast<std::size_t>(closest_revisit_quota)) {
    // Registration is deadline bounded. Keep the strongest closest set first,
    // then process the specifically reserved corridor anchors before generic
    // pair-index coverage so the new evidence cannot be truncated at the tail.
    std::rotate(
        selected_revisits.begin() + closest_revisit_quota,
        selected_revisits.begin() +
            static_cast<std::ptrdiff_t>(pair_index_selected_count),
        selected_revisits.begin() +
            static_cast<std::ptrdiff_t>(corridor_selected_end));
  }
  fill_pair_index_coverage(max_selected_revisits);
  // A standard strong endpoint confirms that the terminal window revisits a
  // historical place, but a single medoid edge only constrains one correction
  // value. Reserve up to two extra registrations of the exact accepted
  // endpoint pairs so the existing internal local-anchor pipeline can verify
  // several positions along that same segment. These candidates remain in the
  // same correlated evidence group and can never become independent votes for
  // authorizing the endpoint itself.
  int endpoint_segment_refinement_candidates = 0;
  constexpr int kMaximumEndpointSegmentRefinementCandidates = 2;
  const std::size_t endpoint_segment_refinement_begin =
      selected_revisits.size();
  for (const auto& hypothesis : distinct_endpoint_hypotheses) {
    if (endpoint_segment_refinement_candidates >=
            kMaximumEndpointSegmentRefinementCandidates ||
        hypothesis.consensus_count < 3 || !hypothesis.strong ||
        hypothesis.partial_geometry ||
        hypothesis.two_vote_strict_provisional ||
        hypothesis.edge.corridor_partial ||
        hypothesis.index < 0 || hypothesis.index >= end_idx) {
      continue;
    }
    const bool duplicate_pair = std::any_of(
        selected_revisits.begin(), selected_revisits.end(),
        [&](const RevisitCandidate& candidate) {
          return candidate.earlier == hypothesis.index &&
              candidate.later == end_idx;
        });
    if (duplicate_pair) {
      continue;
    }
    RevisitCandidate candidate;
    candidate.earlier = hypothesis.index;
    candidate.later = end_idx;
    candidate.index_gap = end_idx - hypothesis.index;
    candidate.sensor_time_separation =
        loop_keyframes_[end_idx].timestamp -
        loop_keyframes_[hypothesis.index].timestamp;
    const V3 raw_delta =
        raw_poses[end_idx].t_ - raw_poses[hypothesis.index].t_;
    candidate.horizontal_distance = static_cast<float>(
        std::hypot(raw_delta.x(), raw_delta.y()));
    candidate.vertical_distance =
        static_cast<float>(std::abs(raw_delta.z()));
    candidate.segment_search_radius = internal_search_radius;
    candidate.discovery_score = candidate.horizontal_distance +
        0.15f * std::min(
            candidate.vertical_distance, internal_search_radius);
    candidate.path_length =
        cumulative_route_length[end_idx] -
        cumulative_route_length[hypothesis.index];
    populate_corridor_descriptor(candidate);
    selected_revisits.push_back(candidate);
    ++endpoint_segment_refinement_candidates;
    LOG(INFO) << GREEN
              << " ---> 强终点回环追加整段局部锚精验。earlier_idx: "
              << candidate.earlier
              << " later_idx: " << candidate.later
              << " consensus_count: " << hypothesis.consensus_count
              << " raw_xy_distance: " << candidate.horizontal_distance
              << " raw_z_distance: " << candidate.vertical_distance
              << " path_length: " << candidate.path_length
              << " extra_candidates: "
              << endpoint_segment_refinement_candidates << "/"
              << kMaximumEndpointSegmentRefinementCandidates
              << " endpoint_support_eligible: false" << RESET;
  }
  if (endpoint_segment_refinement_candidates > 0 &&
      endpoint_segment_refinement_begin >
          static_cast<std::size_t>(closest_revisit_quota)) {
    // Registration has a save-time deadline. Process these confirmed segment
    // refinements immediately after the strongest closest candidates instead
    // of leaving them behind generic coverage candidates at the tail.
    std::rotate(
        selected_revisits.begin() + closest_revisit_quota,
        selected_revisits.begin() +
            static_cast<std::ptrdiff_t>(
                endpoint_segment_refinement_begin),
        selected_revisits.end());
  }
  // Index-overlapping physical corridor anchors reuse part of the same two
  // temporal windows.  Keep them as one evidence group so they can improve
  // spatial placement without becoming independent votes or gaining repeated
  // information weight in the graph.
  std::vector<int> evidence_parent(selected_revisits.size());
  std::iota(evidence_parent.begin(), evidence_parent.end(), 0);
  const auto find_evidence_root = [&] (int index) {
    while (evidence_parent[index] != index) {
      evidence_parent[index] = evidence_parent[
          evidence_parent[index]];
      index = evidence_parent[index];
    }
    return index;
  };
  for (std::size_t i = 0; i < selected_revisits.size(); ++i) {
    for (std::size_t j = 0; j < i; ++j) {
      if (!pair_windows_overlap(
              selected_revisits[i], selected_revisits[j])) {
        continue;
      }
      const int root_i = find_evidence_root(static_cast<int>(i));
      const int root_j = find_evidence_root(static_cast<int>(j));
      if (root_i != root_j) {
        evidence_parent[root_i] = root_j;
      }
    }
  }
  std::map<int, int> dense_evidence_group_ids;
  for (std::size_t i = 0; i < selected_revisits.size(); ++i) {
    const int root = find_evidence_root(static_cast<int>(i));
    auto insertion = dense_evidence_group_ids.emplace(
        root, static_cast<int>(dense_evidence_group_ids.size()));
    selected_revisits[i].evidence_group_id = insertion.first->second;
  }
  // Preserve selection priority for registration. Re-sorting by `later`
  // would make a deadline truncate the map tail and discard the pair-space
  // coverage deliberately chosen above.
  LOG(INFO) << GREEN
            << " ---> 中途回环候选覆盖选择。discovered: "
            << revisit_candidates.size()
            << " selected: " << selected_revisits.size()
            << " top_k_earlier: " << kEarlierCandidatesPerLater
            << " multi_earlier_later_frames: "
            << later_frames_with_multiple_earlier
            << " closest_quota: " << closest_revisit_quota
            << " corridor_quota: " << corridor_coverage_quota
            << " corridor_selected: "
            << corridor_coverage_selections.size()
            << " index_separation: " << revisit_index_separation
            << " corridor_anchor_separation: "
            << corridor_anchor_separation
            << " corridor_cross_track_limit: "
            << corridor_cross_track_limit
            << " tangent_limit_deg: "
            << kCorridorTangentMismatchDeg
            << " evidence_groups: "
            << dense_evidence_group_ids.size()
            << " endpoint_segment_refinement_candidates: "
            << endpoint_segment_refinement_candidates
            << RESET;
  for (const auto& selection : corridor_coverage_selections) {
    LOG(INFO) << GREEN
              << " ---> 中途回环物理走廊补位。earlier_idx: "
              << selection.earlier
              << " later_idx: " << selection.later
              << " related_earlier_idx: "
              << selection.related_earlier
              << " related_later_idx: "
              << selection.related_later
              << " tangent_mismatch_deg: "
              << selection.tangent_mismatch_deg
              << " related_tangent_mismatch_deg: "
              << selection.related_tangent_mismatch_deg
              << " along_track: " << selection.along_track
              << " cross_track: " << selection.cross_track
              << " related_tangent_difference_deg: "
              << selection.related_tangent_difference_deg
              << " total_budget_unchanged: "
              << max_selected_revisits << RESET;
  }

  // Internal loops must stand on the raw trajectory by themselves. Using an
  // endpoint-only deformation here lets one false endpoint manufacture the
  // consistency of every later edge (circular confirmation).
  PoseVector raw_consistency_reference_poses(
      loop_keyframes_.size(), SE3());

  int accepted_internal_loops = 0;
  PoseGraphEdgeVector accepted_internal_edges;
  accepted_internal_edges.reserve(selected_revisits.size());
  struct InternalRegistrationAttempt
  {
    Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
    double score = std::numeric_limits<double>::max();
    float overlap_ratio = 0.0f;
    LoopRotationMetrics rotation;
    int seed_translation_mode = -1;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using InternalRegistrationAttemptVector = std::vector<
      InternalRegistrationAttempt,
      Eigen::aligned_allocator<InternalRegistrationAttempt>>;
  struct InternalFrameSlice
  {
    int index = -1;
    double timestamp = 0.0;
    float route_length = 0.0f;
    CloudPtr world_cloud;
  };
  struct InternalAdaptiveWindow
  {
    bool valid = false;
    int start_index = -1;
    int end_index = -1;
    int representative_index = -1;
    int frame_count = 0;
    int contributing_frames = 0;
    int supported_voxels = 0;
    int post_match_voxel_target = 0;
    int point_count = 0;
    double sensor_time_span = 0.0;
    float route_span = 0.0f;
    float support_span = 0.0f;
    float support_minor_span = 0.0f;
    bool post_match_support_ready = false;
    CloudPtr cloud;
  };
  struct InternalLocalAnchorResult
  {
    PoseGraphEdge edge;
    Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    LoopConsistency consistency;
    LoopGeometryVerification geometry;
    GroundZRefinement ground_z;
    InternalResidualModeAnalysis residual_modes;
    InternalAdaptiveWindow source_window;
    InternalAdaptiveWindow target_window;
    double score = std::numeric_limits<double>::max();
    double selection_cost = std::numeric_limits<double>::max();
    float overlap_ratio = 0.0f;
    float crop_radius = 0.0f;
    float raw_weight = 0.0f;
    int reciprocal_matches = 0;
    int support_voxels = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using InternalLocalAnchorResultVector = std::vector<
      InternalLocalAnchorResult,
      Eigen::aligned_allocator<InternalLocalAnchorResult>>;
  struct InternalGroundZOnlyProposal
  {
    PoseGraphEdge edge;
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    GroundZOnlyEvidence evidence;
    LoopConsistency consistency;
    int evidence_group_id = -1;
    int original_earlier = -1;
    int original_later = -1;
    int seed_translation_mode = -1;
    int reciprocal_matches = 0;
    int support_voxels = 0;
    float support_span = 0.0f;
    float support_minor_span = 0.0f;
    float tight_overlap = 0.0f;
    float tight_rmse = std::numeric_limits<float>::max();
    float raw_weight = 0.0f;
    bool full_ground_footprint = false;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using InternalGroundZOnlyProposalVector = std::vector<
      InternalGroundZOnlyProposal,
      Eigen::aligned_allocator<InternalGroundZOnlyProposal>>;
  InternalGroundZOnlyProposalVector ground_z_only_proposals;
  ground_z_only_proposals.reserve(selected_revisits.size() * 2);

  const auto load_internal_frame_slices = [&] (const int center_index) {
    std::vector<InternalFrameSlice> frames;
    const int start_index =
        std::max(0, center_index - g_loop_local_window_size);
    const int stop_index =
        std::min(end_idx, center_index + g_loop_local_window_size);
    frames.reserve(static_cast<std::size_t>(stop_index - start_index + 1));
    for (int index = start_index; index <= stop_index; ++index) {
      CloudPtr body_cloud =
          loadLoopKeyFrameCloud(static_cast<std::size_t>(index));
      if (!body_cloud || body_cloud->empty()) {
        continue;
      }
      InternalFrameSlice frame;
      frame.index = index;
      frame.timestamp = loop_keyframes_[index].timestamp;
      frame.route_length = cumulative_route_length[index];
      frame.world_cloud.reset(new PointCloudType());
      pcl::transformPointCloud(
          *body_cloud, *frame.world_cloud,
          se3_to_matrix4f(loop_keyframes_[index].pose));
      normalize_cloud_layout(*frame.world_cloud);
      frames.push_back(std::move(frame));
    }
    return frames;
  };

  const auto merge_internal_frame_slices = [&] (
      const std::vector<InternalFrameSlice>& frames,
      const char* context) {
    CloudPtr accumulated(new PointCloudType());
    for (const auto& frame : frames) {
      if (frame.world_cloud && !frame.world_cloud->empty()) {
        *accumulated += *frame.world_cloud;
      }
    }
    normalize_cloud_layout(*accumulated);
    CloudPtr filtered(new PointCloudType());
    make_map_pcd_cloud(
        accumulated, *filtered, g_loop_map_ds_size, context);
    return filtered;
  };

  const auto build_internal_anchor_window = [&] (
      const std::vector<InternalFrameSlice>& frames,
      const InternalAnchorCenterVector& anchor_centers,
      const int anchor_id,
      const float crop_radius,
      const Eigen::Matrix4f& evidence_correction,
      const bool transform_for_evidence) {
    InternalAdaptiveWindow result;
    result.cloud.reset(new PointCloudType());
    if (frames.empty() || anchor_id < 0 ||
        static_cast<std::size_t>(anchor_id) >= anchor_centers.size()) {
      return result;
    }

    struct CroppedFrame
    {
      int index = -1;
      double timestamp = 0.0;
      float route_length = 0.0f;
      CloudPtr registration_cloud;
      CloudPtr evidence_cloud;
      std::set<InternalVoxelKey> occupied_voxels;
    };
    std::vector<CroppedFrame> cropped_frames;
    cropped_frames.reserve(frames.size());
    const float radius_squared = crop_radius * crop_radius;
    const Eigen::Matrix3f evidence_rotation =
        evidence_correction.block<3, 3>(0, 0);
    const Eigen::Vector3f evidence_translation =
        evidence_correction.block<3, 1>(0, 3);
    for (const auto& frame : frames) {
      CroppedFrame cropped;
      cropped.index = frame.index;
      cropped.timestamp = frame.timestamp;
      cropped.route_length = frame.route_length;
      cropped.registration_cloud.reset(new PointCloudType());
      cropped.evidence_cloud.reset(new PointCloudType());
      if (!frame.world_cloud) {
        cropped_frames.push_back(std::move(cropped));
        continue;
      }
      cropped.registration_cloud->reserve(frame.world_cloud->size());
      cropped.evidence_cloud->reserve(frame.world_cloud->size());
      for (const auto& point : frame.world_cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
          continue;
        }
        const Eigen::Vector3f raw(point.x, point.y, point.z);
        const Eigen::Vector3f evidence = transform_for_evidence
            ? evidence_rotation * raw + evidence_translation
            : raw;
        if ((evidence - anchor_centers[anchor_id]).squaredNorm() >
            radius_squared) {
          continue;
        }
        int nearest_anchor = anchor_id;
        float nearest_squared =
            (evidence - anchor_centers[anchor_id]).squaredNorm();
        for (std::size_t other = 0;
             other < anchor_centers.size();
             ++other) {
          const float squared =
              (evidence - anchor_centers[other]).squaredNorm();
          if (squared < nearest_squared - 1.0e-6f ||
              (std::abs(squared - nearest_squared) <= 1.0e-6f &&
               static_cast<int>(other) < nearest_anchor)) {
            nearest_squared = squared;
            nearest_anchor = static_cast<int>(other);
          }
        }
        if (nearest_anchor != anchor_id) {
          continue;
        }
        cropped.registration_cloud->push_back(point);
        PointType evidence_point = point;
        evidence_point.x = evidence.x();
        evidence_point.y = evidence.y();
        evidence_point.z = evidence.z();
        cropped.evidence_cloud->push_back(evidence_point);
        cropped.occupied_voxels.insert(internal_voxel_key(
            evidence, g_loop_verification_block_size));
      }
      normalize_cloud_layout(*cropped.registration_cloud);
      normalize_cloud_layout(*cropped.evidence_cloud);
      cropped_frames.push_back(std::move(cropped));
    }

    struct WindowEvidence
    {
      bool valid = false;
      bool base_valid = false;
      int contributing_frames = 0;
      int supported_voxels = 0;
      int point_count = 0;
      float support_span = 0.0f;
      float support_minor_span = 0.0f;
    };
    if (cropped_frames.empty()) {
      return result;
    }

    // Start at the frame that contributes the broadest local evidence and
    // grow one temporal neighbour at a time.  Every point enters the running
    // statistics once, instead of rescanning all O(W^2) begin/end windows.
    int peak_slot = 0;
    for (int slot = 1;
         slot < static_cast<int>(cropped_frames.size());
         ++slot) {
      const auto& candidate = cropped_frames[slot];
      const auto& peak = cropped_frames[peak_slot];
      if (candidate.occupied_voxels.size() >
              peak.occupied_voxels.size() ||
          (candidate.occupied_voxels.size() ==
               peak.occupied_voxels.size() &&
           candidate.evidence_cloud->size() >
               peak.evidence_cloud->size())) {
        peak_slot = slot;
      }
    }
    int best_begin = peak_slot;
    int best_end = peak_slot;
    std::map<InternalVoxelKey, int> accumulated_voxel_counts;
    std::set<InternalVoxelKey> accumulated_voxels;
    WindowEvidence best_evidence;
    bool has_point = false;
    Eigen::Vector3f minimum = Eigen::Vector3f::Zero();
    Eigen::Vector3f maximum = Eigen::Vector3f::Zero();
    constexpr int kMinimumWindowPointsPerVoxel = 10;
    // Keep growing after the old six-voxel minimum so downsampling and
    // reciprocal-NN filtering have spatial reserve. If the bounded local
    // frame range cannot reach this target, preserve the baseline-valid
    // window and let the final post-match verifier decide it.
    const int post_match_voxel_target = std::max(
        g_loop_min_verification_blocks,
        (3 * g_loop_min_verification_blocks + 1) / 2);
    const auto update_window_validity = [&] () {
      if (has_point) {
        std::array<float, 3> extents = {
            maximum.x() - minimum.x(),
            maximum.y() - minimum.y(),
            maximum.z() - minimum.z()};
        std::sort(extents.begin(), extents.end(), std::greater<float>());
        best_evidence.support_span = extents[0];
        best_evidence.support_minor_span = extents[1];
      }
      const int minimum_points =
          10 * g_loop_min_verification_blocks;
      best_evidence.base_valid =
          best_evidence.contributing_frames >= 2 &&
          best_evidence.point_count >= minimum_points &&
          best_evidence.supported_voxels >=
              g_loop_min_verification_blocks &&
          best_evidence.support_span >=
              g_loop_min_verification_span &&
          best_evidence.support_minor_span >=
              g_loop_verification_block_size;
      best_evidence.valid =
          best_evidence.base_valid &&
          best_evidence.supported_voxels >=
              post_match_voxel_target &&
          best_evidence.point_count >=
              kMinimumWindowPointsPerVoxel *
                  post_match_voxel_target;
    };
    const auto add_frame_to_window = [&] (const int slot) {
      const auto& cloud = cropped_frames[slot].evidence_cloud;
      if (cloud && !cloud->empty()) {
        ++best_evidence.contributing_frames;
      }
      if (cloud) {
        for (const auto& point : cloud->points) {
          const Eigen::Vector3f xyz(point.x, point.y, point.z);
          ++best_evidence.point_count;
          const InternalVoxelKey key = internal_voxel_key(
              xyz, g_loop_verification_block_size);
          int& voxel_count = accumulated_voxel_counts[key];
          ++voxel_count;
          if (voxel_count == kMinimumWindowPointsPerVoxel) {
            ++best_evidence.supported_voxels;
          }
          accumulated_voxels.insert(key);
          if (!has_point) {
            minimum = maximum = xyz;
            has_point = true;
          } else {
            minimum = minimum.cwiseMin(xyz);
            maximum = maximum.cwiseMax(xyz);
          }
        }
      }
      update_window_validity();
    };
    const auto marginal_voxel_count = [&] (const int slot) {
      int count = 0;
      for (const auto& key : cropped_frames[slot].occupied_voxels) {
        if (accumulated_voxels.count(key) == 0U) {
          ++count;
        }
      }
      return count;
    };

    add_frame_to_window(peak_slot);
    while (!best_evidence.valid &&
           (best_begin > 0 ||
            best_end + 1 < static_cast<int>(cropped_frames.size()))) {
      const int left_slot = best_begin > 0 ? best_begin - 1 : -1;
      const int right_slot =
          best_end + 1 < static_cast<int>(cropped_frames.size())
          ? best_end + 1 : -1;
      int selected_slot = -1;
      if (left_slot < 0) {
        selected_slot = right_slot;
      } else if (right_slot < 0) {
        selected_slot = left_slot;
      } else {
        const int left_new_voxels = marginal_voxel_count(left_slot);
        const int right_new_voxels = marginal_voxel_count(right_slot);
        const double left_dt = std::max(
            1.0e-3,
            std::abs(cropped_frames[best_begin].timestamp -
                     cropped_frames[left_slot].timestamp));
        const double right_dt = std::max(
            1.0e-3,
            std::abs(cropped_frames[right_slot].timestamp -
                     cropped_frames[best_end].timestamp));
        const double left_gain =
            static_cast<double>(left_new_voxels) / left_dt;
        const double right_gain =
            static_cast<double>(right_new_voxels) / right_dt;
        if (left_gain > right_gain + 1.0e-9 ||
            (std::abs(left_gain - right_gain) <= 1.0e-9 &&
             cropped_frames[left_slot].evidence_cloud->size() >
                 cropped_frames[right_slot].evidence_cloud->size())) {
          selected_slot = left_slot;
        } else {
          selected_slot = right_slot;
        }
      }
      if (selected_slot < 0) {
        break;
      }
      best_begin = std::min(best_begin, selected_slot);
      best_end = std::max(best_end, selected_slot);
      add_frame_to_window(selected_slot);
    }
    if (!best_evidence.base_valid) {
      return result;
    }
    const double best_time_span = std::max(
        0.0,
        cropped_frames[best_end].timestamp -
            cropped_frames[best_begin].timestamp);
    const int best_frame_count = best_end - best_begin + 1;

    CloudPtr accumulated(new PointCloudType());
    int representative_slot = best_begin;
    std::size_t representative_points = 0;
    for (int slot = best_begin; slot <= best_end; ++slot) {
      const auto& cloud = cropped_frames[slot].registration_cloud;
      if (!cloud) {
        continue;
      }
      *accumulated += *cloud;
      if (cloud->size() > representative_points) {
        representative_points = cloud->size();
        representative_slot = slot;
      }
    }
    normalize_cloud_layout(*accumulated);
    make_map_pcd_cloud(
        accumulated, *result.cloud, g_loop_map_ds_size,
        "internal local anchor");
    if (result.cloud->empty()) {
      return result;
    }
    result.valid = true;
    result.start_index = cropped_frames[best_begin].index;
    result.end_index = cropped_frames[best_end].index;
    result.representative_index =
        cropped_frames[representative_slot].index;
    result.frame_count = best_frame_count;
    result.contributing_frames = best_evidence.contributing_frames;
    result.supported_voxels = best_evidence.supported_voxels;
    result.post_match_voxel_target = post_match_voxel_target;
    result.point_count = static_cast<int>(result.cloud->size());
    result.sensor_time_span = best_time_span;
    result.route_span = std::abs(
        cropped_frames[best_end].route_length -
        cropped_frames[best_begin].route_length);
    result.support_span = best_evidence.support_span;
    result.support_minor_span = best_evidence.support_minor_span;
    result.post_match_support_ready = best_evidence.valid;
    return result;
  };

  int accepted_internal_groups = 0;
  std::map<int, int> next_internal_anchor_id_by_group;
  std::map<int, float> internal_evidence_group_weight_budget;
  for (const auto& revisit : selected_revisits) {
    if (registration_timed_out()) {
      break;
    }
    std::vector<InternalFrameSlice> revisit_source_frames =
        load_internal_frame_slices(revisit.later);
    std::vector<InternalFrameSlice> revisit_target_frames =
        load_internal_frame_slices(revisit.earlier);
    CloudPtr revisit_source = merge_internal_frame_slices(
        revisit_source_frames, "internal coarse source");
    CloudPtr revisit_target = merge_internal_frame_slices(
        revisit_target_frames, "internal coarse target");
    if (revisit_source->empty() || revisit_target->empty()) {
      continue;
    }

    const float segment_path_length =
        cumulative_route_length[revisit.later] -
        cumulative_route_length[revisit.earlier];
    const float revisit_yaw_limit_deg =
        adaptive_loop_yaw_limit_deg(segment_path_length);
    pcl::search::KdTree<PointType> target_tree;
    target_tree.setInputCloud(revisit_target);
    InternalRegistrationAttemptVector coarse_attempts;
    constexpr int kMaximumCoarseAttemptsPerPair = 3;
    InternalRegistrationAttempt best_coarse_attempt;
    bool has_coarse_result = false;
    const Eigen::Vector3f revisit_source_anchor =
        loop_keyframes_[revisit.later].pose.t_.cast<float>();
    const Eigen::Vector3f revisit_target_anchor =
        loop_keyframes_[revisit.earlier].pose.t_.cast<float>();
    for (int translation_mode = 0;
         translation_mode < 3;
         ++translation_mode) {
      if (registration_timed_out()) {
        break;
      }
      Eigen::Vector3f seeded_anchor = revisit_source_anchor;
      if (translation_mode >= 1) {
        seeded_anchor.z() = revisit_target_anchor.z();
      }
      if (translation_mode == 2) {
        seeded_anchor = revisit_target_anchor;
      }
      Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
      initial_guess.block<3, 1>(0, 3) =
          seeded_anchor - revisit_source_anchor;
      pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
      icp.setInputSource(revisit_source);
      icp.setInputTarget(revisit_target);
      icp.setMaxCorrespondenceDistance(g_loop_icp_max_distance);
      icp.setMaximumIterations(80);
      icp.setTransformationEpsilon(1e-6);
      icp.setEuclideanFitnessEpsilon(1e-5);
      PointCloudType aligned;
      icp.align(aligned, initial_guess);
      const Eigen::Matrix4f attempt_correction =
          icp.getFinalTransformation();
      if (!icp.hasConverged() ||
          !attempt_correction.allFinite() ||
          aligned.empty()) {
        continue;
      }
      std::vector<int> nearest_index(1);
      std::vector<float> nearest_squared_distance(1);
      std::size_t overlap_count = 0;
      double squared_distance_sum = 0.0;
      const float max_squared_distance =
          g_loop_icp_max_distance * g_loop_icp_max_distance;
      for (const auto& point : aligned.points) {
        if (target_tree.nearestKSearch(
                point, 1, nearest_index,
                nearest_squared_distance) > 0 &&
            nearest_squared_distance[0] <= max_squared_distance) {
          ++overlap_count;
          squared_distance_sum += nearest_squared_distance[0];
        }
      }
      if (overlap_count == 0) {
        continue;
      }
      const double attempt_score =
          squared_distance_sum /
          static_cast<double>(overlap_count);
      const float attempt_overlap_ratio =
          static_cast<float>(overlap_count) /
          static_cast<float>(aligned.size());
      const LoopRotationMetrics attempt_rotation =
          evaluate_loop_rotation(
              attempt_correction.block<3, 3>(0, 0));
      const bool attempt_accepted =
          attempt_score <= g_loop_icp_score_threshold &&
          attempt_overlap_ratio >= g_loop_min_overlap_ratio &&
          loop_rotation_is_plausible(
              attempt_rotation, revisit_yaw_limit_deg);
      InternalRegistrationAttempt attempt;
      attempt.correction = attempt_correction;
      attempt.score = attempt_score;
      attempt.overlap_ratio = attempt_overlap_ratio;
      attempt.rotation = attempt_rotation;
      attempt.seed_translation_mode = translation_mode;
      if (!has_coarse_result ||
          attempt.score < best_coarse_attempt.score) {
        best_coarse_attempt = attempt;
        has_coarse_result = true;
      }
      LOG(INFO) << GREEN
                << " ---> 中途回环粗配准 seed。earlier_idx: "
                << revisit.earlier
                << " later_idx: " << revisit.later
                << " seed_translation_mode: " << translation_mode
                << " score: " << attempt_score
                << " overlap_ratio: " << attempt_overlap_ratio
                << " rotation_deg: " << attempt_rotation.total_deg
                << " yaw_deg: " << attempt_rotation.yaw_deg
                << " tilt_deg: " << attempt_rotation.tilt_deg
                << " coarse_accepted: " << attempt_accepted << RESET;
      if (!attempt_accepted) {
        continue;
      }

      // Translation seeds often converge to the same basin. Keep one copy so
      // tight bidirectional verification is spent on distinct corrections.
      bool duplicate_basin = false;
      for (auto& existing : coarse_attempts) {
        const Eigen::Vector3f existing_anchor =
            existing.correction.block<3, 3>(0, 0) *
                revisit_source_anchor +
            existing.correction.block<3, 1>(0, 3);
        const Eigen::Vector3f attempt_anchor =
            attempt.correction.block<3, 3>(0, 0) *
                revisit_source_anchor +
            attempt.correction.block<3, 1>(0, 3);
        const float basin_translation =
            (existing_anchor - attempt_anchor).norm();
        const float basin_rotation_deg =
            std::abs(Eigen::AngleAxisf(
                existing.correction.block<3, 3>(0, 0).transpose() *
                attempt.correction.block<3, 3>(0, 0)).angle()) *
            180.0f / static_cast<float>(M_PI);
        if (basin_translation > 0.20f ||
            basin_rotation_deg > 0.25f) {
          continue;
        }
        duplicate_basin = true;
        if (attempt.score < existing.score) {
          existing = attempt;
        }
        break;
      }
      if (!duplicate_basin &&
          static_cast<int>(coarse_attempts.size()) <
              kMaximumCoarseAttemptsPerPair) {
        coarse_attempts.push_back(attempt);
      }
    }
    std::sort(
        coarse_attempts.begin(), coarse_attempts.end(),
        [](const InternalRegistrationAttempt& lhs,
           const InternalRegistrationAttempt& rhs) {
          if (lhs.score != rhs.score) {
            return lhs.score < rhs.score;
          }
          return lhs.overlap_ratio > rhs.overlap_ratio;
        });
    LOG(INFO) << GREEN
              << " ---> 中途回环候选。earlier_idx: "
              << revisit.earlier
              << " later_idx: " << revisit.later
              << " earlier_xyz: "
              << loop_keyframes_[revisit.earlier].pose.t_.transpose()
              << " later_xyz: "
              << loop_keyframes_[revisit.later].pose.t_.transpose()
              << " index_gap: " << revisit.index_gap
              << " sensor_dt: "
              << revisit.sensor_time_separation
              << " path_length: " << revisit.path_length
              << " horizontal_distance: "
              << revisit.horizontal_distance
              << " vertical_distance: "
              << revisit.vertical_distance
              << " segment_search_radius: "
              << revisit.segment_search_radius
              << " discovery_score: " << revisit.discovery_score
              << " best_score: " << best_coarse_attempt.score
              << " best_overlap_ratio: "
              << best_coarse_attempt.overlap_ratio
              << " best_rotation_deg: "
              << best_coarse_attempt.rotation.total_deg
              << " best_yaw_deg: "
              << best_coarse_attempt.rotation.yaw_deg
              << " best_tilt_deg: "
              << best_coarse_attempt.rotation.tilt_deg
              << " yaw_limit_deg: " << revisit_yaw_limit_deg
              << " coarse_attempts: " << coarse_attempts.size()
              << " coarse_accepted: "
              << !coarse_attempts.empty() << RESET;
    if (coarse_attempts.empty()) {
      continue;
    }

    // A fixed +/-W full-FOV submap is intentionally only a basin seed.  A
    // broad view can obtain an excellent aggregate score while averaging two
    // locally inconsistent layers.  Independently cropped local anchors are
    // always preferred.  If none survives, an aggregate correction can enter
    // only through the explicit 0.10/0.20 soft tier after strict geometry,
    // raw consistency and non-overlapping 3D residual checks.
    InternalLocalAnchorResultVector best_group_results;
    double best_group_cost = std::numeric_limits<double>::max();
    int best_group_seed_mode = -1;
    std::optional<InternalLocalAnchorResult> best_soft_fallback;
    double best_soft_fallback_cost =
        std::numeric_limits<double>::max();
    int best_soft_fallback_seed_mode = -1;
    for (const auto& attempt : coarse_attempts) {
      if (registration_timed_out()) {
        break;
      }
      const Eigen::Matrix4f seed_correction = se3_to_matrix4f(
          project_gravity_aligned_loop_correction(
              attempt.correction,
              raw_poses[revisit.later].t_));
      const InternalReciprocalMatchVector coarse_matches =
          collect_internal_reciprocal_matches(
              revisit_source, revisit_target, seed_correction);
      const InternalSupportVoxelSummary coarse_support =
          analyze_internal_support_voxels(coarse_matches);
      const auto& support_voxels = coarse_support.voxels;
      const InternalResidualModeAnalysis coarse_modes =
          analyze_internal_residual_modes(coarse_matches);
      LOG(INFO) << GREEN
                << " ---> 中途回环全窗口仅作 seed。earlier_idx: "
                << revisit.earlier
                << " later_idx: " << revisit.later
                << " seed_translation_mode: "
                << attempt.seed_translation_mode
                << " source_points: " << revisit_source->size()
                << " target_points: " << revisit_target->size()
                << " reciprocal_matches: " << coarse_matches.size()
                << " support_voxels_3d: " << support_voxels.size()
                << " support_pairs_per_voxel_threshold: "
                << coarse_support.minimum_matches_per_voxel
                << " support_span: " << coarse_support.support_span
                << " support_minor_span: "
                << coarse_support.support_minor_span
                << " residual_axis: " << coarse_modes.principal_axis
                << " residual_mode0_points: "
                << coarse_modes.mode0_points
                << " residual_mode1_points: "
                << coarse_modes.mode1_points
                << " residual_mode_separation: "
                << coarse_modes.separation
                << " residual_within_mad: "
                << coarse_modes.within_mode_mad
                << " residual_separation_limit: "
                << coarse_modes.separation_threshold
                << " residual_spatial_overlap: "
                << coarse_modes.spatial_overlap_ratio
                << " residual_bimodal: "
                << coarse_modes.significant
                << " residual_ambiguous: "
                << coarse_modes.spatially_overlapping
                << " inserted_as_edge: false" << RESET;
      if (coarse_modes.spatially_overlapping) {
        LOG(WARNING) << YELLOW
                     << " ---> 中途回环整体 seed 无法形成安全局部锚。"
                     << " earlier_idx: " << revisit.earlier
                     << " later_idx: " << revisit.later
                     << " reason: "
                     << "overlapping_residual_modes"
                     << RESET;
        continue;
      }

      const int minimum_soft_reciprocal_matches =
          2 * std::max(30, 5 * g_loop_min_verification_blocks);
      const bool soft_3d_support_valid =
          support_voxels.size() >= 2 &&
          static_cast<int>(coarse_matches.size()) >=
              minimum_soft_reciprocal_matches;
      const bool strict_local_support_available =
          static_cast<int>(support_voxels.size()) >=
              g_loop_min_verification_blocks;

      PoseGraphEdge fallback_edge;
      fallback_edge.from = revisit.earlier;
      fallback_edge.to = revisit.later;
      fallback_edge.measurement = SE3(seed_correction);
      fallback_edge.loop = true;
      fallback_edge.soft_fallback = true;
      const float fallback_weight_scale =
          (coarse_modes.significant ||
           !strict_local_support_available) ? 0.10f : 0.20f;
      const bool evaluate_fallback =
          !best_soft_fallback.has_value() &&
          soft_3d_support_valid;
      LoopGeometryVerification fallback_geometry;
      LoopConsistency fallback_consistency;
      bool fallback_raw_consistent = false;
      bool fallback_valid = false;
      double fallback_cost = std::numeric_limits<double>::max();
      if (evaluate_fallback) {
        fallback_geometry = verify_loop_geometry(
            revisit_source, revisit_target, seed_correction,
            raw_poses[revisit.later].t_);
        fallback_consistency = evaluate_loop_consistency(
            fallback_edge, raw_consistency_reference_poses,
            segment_path_length, raw_poses[revisit.later].t_);
        fallback_raw_consistent =
            fallback_consistency.weight >=
                g_loop_min_consistency_weight;
        fallback_valid =
            fallback_geometry.valid && fallback_raw_consistent;
        fallback_cost =
            attempt.score +
            (1.0 - static_cast<double>(fallback_consistency.weight)) +
            0.1 * (1.0 - static_cast<double>(attempt.overlap_ratio)) +
            0.5 * (1.0 -
                static_cast<double>(fallback_geometry.confidence));
      }
      LOG(INFO) << GREEN
                << " ---> 中途回环 soft fallback 预验。earlier_idx: "
                << revisit.earlier
                << " later_idx: " << revisit.later
                << " seed_translation_mode: "
                << attempt.seed_translation_mode
                << " evaluated: " << evaluate_fallback
                << " geometry_accepted: "
                << fallback_geometry.valid
                << " tight_symmetric_overlap: "
                << fallback_geometry.symmetric_overlap
                << " tight_trimmed_rmse: "
                << fallback_geometry.symmetric_trimmed_rmse
                << " raw_consistency_weight: "
                << fallback_consistency.weight
                << " residual_bimodal: "
                << coarse_modes.significant
                << " residual_spatial_overlap: "
                << coarse_modes.spatial_overlap_ratio
                << " reciprocal_matches: " << coarse_matches.size()
                << " support_voxels_3d: " << support_voxels.size()
                << " soft_3d_support_valid: "
                << soft_3d_support_valid
                << " strict_local_support_available: "
                << strict_local_support_available
                << " fallback_weight_scale: "
                << fallback_weight_scale
                << " eligible_if_no_local_anchor: "
                << fallback_valid << RESET;
      if (fallback_valid &&
          fallback_cost < best_soft_fallback_cost) {
        InternalLocalAnchorResult fallback;
        fallback.edge = fallback_edge;
        fallback.correction = seed_correction;
        fallback.center = 0.5f * (
            raw_poses[revisit.earlier].t_.cast<float>() +
            (seed_correction.block<3, 3>(0, 0) *
                 raw_poses[revisit.later].t_.cast<float>() +
             seed_correction.block<3, 1>(0, 3)));
        fallback.consistency = fallback_consistency;
        fallback.geometry = fallback_geometry;
        fallback.residual_modes = coarse_modes;
        fallback.score = attempt.score;
        fallback.selection_cost = fallback_cost;
        fallback.overlap_ratio = attempt.overlap_ratio;
        fallback.raw_weight =
            fallback_weight_scale * 1.5f *
            fallback_consistency.weight *
            std::max(0.25f, fallback_geometry.confidence);
        fallback.reciprocal_matches =
            static_cast<int>(coarse_matches.size());
        fallback.support_voxels =
            static_cast<int>(support_voxels.size());
        best_soft_fallback = std::move(fallback);
        best_soft_fallback_cost = fallback_cost;
        best_soft_fallback_seed_mode = attempt.seed_translation_mode;
      }

      if (!strict_local_support_available) {
        LOG(WARNING) << YELLOW
                     << " ---> 中途回环3D支持不足以生成strict局部锚，"
                     << "仅保留通过预验的soft候选。earlier_idx: "
                     << revisit.earlier
                     << " later_idx: " << revisit.later
                     << " reason: insufficient_3d_support_soft_only"
                     << " reciprocal_matches: "
                     << coarse_matches.size()
                     << " support_voxels_3d: "
                     << support_voxels.size()
                     << " soft_eligible: " << fallback_valid
                     << RESET;
        continue;
      }

      const float minimum_crop_radius = std::max(
          2.0f * g_loop_verification_block_size,
          0.5f * g_loop_min_verification_span +
              g_loop_verification_max_distance);
      const float maximum_crop_radius = std::max(
          minimum_crop_radius,
          2.0f * g_loop_min_verification_span +
              g_loop_verification_max_distance);
      const Eigen::Vector3f source_sensor =
          raw_poses[revisit.later].t_.cast<float>();
      const Eigen::Vector3f corrected_source_sensor =
          seed_correction.block<3, 3>(0, 0) * source_sensor +
          seed_correction.block<3, 1>(0, 3);
      const Eigen::Vector3f target_sensor =
          raw_poses[revisit.earlier].t_.cast<float>();
      const Eigen::Vector3f desired_primary_center =
          0.5f * (corrected_source_sensor + target_sensor);
      std::size_t primary_voxel = 0;
      float primary_distance = std::numeric_limits<float>::max();
      for (std::size_t voxel = 0;
           voxel < support_voxels.size();
           ++voxel) {
        const float distance =
            (support_voxels[voxel].center -
             desired_primary_center).squaredNorm();
        if (distance < primary_distance) {
          primary_distance = distance;
          primary_voxel = voxel;
        }
      }
      InternalAnchorCenterVector anchor_centers;
      anchor_centers.push_back(support_voxels[primary_voxel].center);
      constexpr int kMaximumLocalAnchorsPerRevisit = 3;
      const float minimum_anchor_separation =
          2.0f * minimum_crop_radius;
      while (static_cast<int>(anchor_centers.size()) <
             kMaximumLocalAnchorsPerRevisit) {
        std::size_t best_voxel = support_voxels.size();
        float best_separation = -1.0f;
        for (std::size_t voxel = 0;
             voxel < support_voxels.size();
             ++voxel) {
          float nearest_center = std::numeric_limits<float>::max();
          for (const auto& center : anchor_centers) {
            nearest_center = std::min(
                nearest_center,
                (support_voxels[voxel].center - center).norm());
          }
          if (nearest_center > best_separation) {
            best_separation = nearest_center;
            best_voxel = voxel;
          }
        }
        if (best_voxel >= support_voxels.size() ||
            best_separation < minimum_anchor_separation) {
          break;
        }
        anchor_centers.push_back(
            support_voxels[best_voxel].center);
      }

      InternalLocalAnchorResultVector local_results;
      local_results.reserve(anchor_centers.size());
      for (std::size_t anchor = 0;
           anchor < anchor_centers.size();
           ++anchor) {
        if (registration_timed_out()) {
          break;
        }
        InternalAdaptiveWindow source_window;
        InternalAdaptiveWindow target_window;
        std::vector<float> assigned_voxel_distances;
        assigned_voxel_distances.reserve(support_voxels.size());
        for (const auto& voxel : support_voxels) {
          int nearest_anchor = 0;
          float nearest_squared =
              (voxel.center - anchor_centers.front()).squaredNorm();
          for (std::size_t other = 1;
               other < anchor_centers.size();
               ++other) {
            const float squared =
                (voxel.center - anchor_centers[other]).squaredNorm();
            if (squared < nearest_squared) {
              nearest_squared = squared;
              nearest_anchor = static_cast<int>(other);
            }
          }
          if (nearest_anchor == static_cast<int>(anchor)) {
            assigned_voxel_distances.push_back(
                std::sqrt(nearest_squared));
          }
        }
        if (static_cast<int>(assigned_voxel_distances.size()) <
            g_loop_min_verification_blocks) {
          continue;
        }
        std::sort(
            assigned_voxel_distances.begin(),
            assigned_voxel_distances.end());
        const std::size_t evidence_rank = static_cast<std::size_t>(
            std::max(0, g_loop_min_verification_blocks - 1));
        float selected_radius = std::clamp(
            assigned_voxel_distances[evidence_rank] +
                g_loop_verification_max_distance,
            minimum_crop_radius, maximum_crop_radius);
        source_window = build_internal_anchor_window(
            revisit_source_frames, anchor_centers,
            static_cast<int>(anchor), selected_radius,
            seed_correction, true);
        target_window = build_internal_anchor_window(
            revisit_target_frames, anchor_centers,
            static_cast<int>(anchor), selected_radius,
            Eigen::Matrix4f::Identity(), false);
        // One bounded retry covers the case where reciprocal support is broad
        // enough but no two individual frames meet the temporal evidence at
        // the minimal radius.  Do not scan every intermediate radius.
        if ((!source_window.valid || !target_window.valid) &&
            selected_radius < maximum_crop_radius - 1.0e-4f) {
          selected_radius = maximum_crop_radius;
          source_window = build_internal_anchor_window(
              revisit_source_frames, anchor_centers,
              static_cast<int>(anchor), selected_radius,
              seed_correction, true);
          target_window = build_internal_anchor_window(
              revisit_target_frames, anchor_centers,
              static_cast<int>(anchor), selected_radius,
              Eigen::Matrix4f::Identity(), false);
        }
        LOG(INFO) << GREEN
                  << " ---> 中途回环自适应局部窗。earlier_idx: "
                  << revisit.earlier
                  << " later_idx: " << revisit.later
                  << " seed_translation_mode: "
                  << attempt.seed_translation_mode
                  << " anchor_id: " << anchor
                  << " anchor_xyz: "
                  << anchor_centers[anchor].transpose()
                  << " crop_radius: " << selected_radius
                  << " source_window: "
                  << source_window.start_index << ".."
                  << source_window.end_index
                  << " source_representative: "
                  << source_window.representative_index
                  << " source_frames: " << source_window.frame_count
                  << " source_contributing_frames: "
                  << source_window.contributing_frames
                  << " source_sensor_dt: "
                  << source_window.sensor_time_span
                  << " source_route_span: "
                  << source_window.route_span
                  << " source_voxels_3d: "
                  << source_window.supported_voxels
                  << " source_post_match_voxel_target: "
                  << source_window.post_match_voxel_target
                  << " source_post_match_support_ready: "
                  << source_window.post_match_support_ready
                  << " target_window: "
                  << target_window.start_index << ".."
                  << target_window.end_index
                  << " target_representative: "
                  << target_window.representative_index
                  << " target_frames: " << target_window.frame_count
                  << " target_contributing_frames: "
                  << target_window.contributing_frames
                  << " target_sensor_dt: "
                  << target_window.sensor_time_span
                  << " target_route_span: "
                  << target_window.route_span
                  << " target_voxels_3d: "
                  << target_window.supported_voxels
                  << " target_post_match_voxel_target: "
                  << target_window.post_match_voxel_target
                  << " target_post_match_support_ready: "
                  << target_window.post_match_support_ready
                  << " accepted: "
                  << (source_window.valid && target_window.valid)
                  << RESET;
        if (!source_window.valid || !target_window.valid ||
            selected_radius <= 0.0f ||
            !source_window.cloud || !target_window.cloud ||
            source_window.cloud->empty() || target_window.cloud->empty()) {
          continue;
        }

        const float local_max_correspondence_distance = std::min(
            g_loop_icp_max_distance,
            std::max(
                2.0f * g_loop_verification_max_distance,
                3.0f * std::max(
                    coarse_modes.within_mode_mad,
                    g_loop_map_ds_size)));
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> local_icp;
        local_icp.setInputSource(source_window.cloud);
        local_icp.setInputTarget(target_window.cloud);
        local_icp.setMaxCorrespondenceDistance(
            local_max_correspondence_distance);
        local_icp.setMaximumIterations(60);
        local_icp.setTransformationEpsilon(1e-6);
        local_icp.setEuclideanFitnessEpsilon(1e-5);
        PointCloudType locally_aligned;
        local_icp.align(locally_aligned, seed_correction);
        Eigen::Matrix4f local_correction =
            local_icp.getFinalTransformation();
        if (!local_icp.hasConverged() ||
            !local_correction.allFinite() ||
            locally_aligned.empty()) {
          continue;
        }

        const double pre_projection_fitness =
            local_icp.getFitnessScore(
                local_max_correspondence_distance);
        const int local_from = target_window.representative_index;
        const int local_to = source_window.representative_index;
        if (local_from < 0 || local_to <= local_from) {
          continue;
        }
        const float local_path_length =
            cumulative_route_length[local_to] -
            cumulative_route_length[local_from];
        local_correction = se3_to_matrix4f(
            project_gravity_aligned_loop_correction(
                local_correction, raw_poses[local_to].t_));
        const CloudPtr local_source_ground =
            g_loop_ground_z_refinement_enable
            ? extract_loop_ground_envelope(
                source_window.cloud, raw_poses[local_to].t_)
            : CloudPtr(new PointCloudType());
        const CloudPtr local_target_ground =
            g_loop_ground_z_refinement_enable
            ? extract_loop_ground_envelope(
                target_window.cloud, raw_poses[local_from].t_)
            : CloudPtr(new PointCloudType());
        const GroundZRefinement ground_z = refine_loop_ground_z(
            local_source_ground, local_target_ground, local_correction);
        // Gravity projection and ground refinement change the transform that
        // enters the graph.  Recompute score/overlap with that exact final
        // correction; pre-projection GICP fitness is diagnostic only.
        CloudPtr final_local_aligned(new PointCloudType());
        pcl::transformPointCloud(
            *source_window.cloud, *final_local_aligned,
            local_correction);
        normalize_cloud_layout(*final_local_aligned);
        pcl::search::KdTree<PointType> local_target_tree;
        local_target_tree.setInputCloud(target_window.cloud);
        std::vector<int> nearest_index(1);
        std::vector<float> nearest_squared_distance(1);
        const float local_max_squared =
            local_max_correspondence_distance *
            local_max_correspondence_distance;
        std::size_t local_overlap_count = 0;
        double local_squared_distance_sum = 0.0;
        for (const auto& point : final_local_aligned->points) {
          if (local_target_tree.nearestKSearch(
                  point, 1, nearest_index,
                  nearest_squared_distance) > 0 &&
              nearest_squared_distance[0] <= local_max_squared) {
            ++local_overlap_count;
            local_squared_distance_sum += nearest_squared_distance[0];
          }
        }
        if (local_overlap_count == 0) {
          continue;
        }
        const double local_score =
            local_squared_distance_sum /
            static_cast<double>(local_overlap_count);
        const float local_overlap =
            static_cast<float>(local_overlap_count) /
            static_cast<float>(final_local_aligned->size());
        const LoopRotationMetrics local_rotation =
            evaluate_loop_rotation(
                local_correction.block<3, 3>(0, 0));
        const bool local_coarse_valid =
            local_score <= g_loop_icp_score_threshold &&
            local_overlap >= g_loop_min_overlap_ratio &&
            loop_rotation_is_plausible(
                local_rotation,
                adaptive_loop_yaw_limit_deg(local_path_length));
        const LoopGeometryVerification geometry = verify_loop_geometry(
            source_window.cloud, target_window.cloud,
            local_correction, raw_poses[local_to].t_);
        const InternalReciprocalMatchVector local_matches =
            collect_internal_reciprocal_matches(
                source_window.cloud, target_window.cloud,
                local_correction);
        const InternalSupportVoxelSummary local_support =
            analyze_internal_support_voxels(local_matches);
        const InternalResidualModeAnalysis local_modes =
            analyze_internal_residual_modes(local_matches);
        const GroundZOnlyEvidence ground_z_only =
            estimate_ground_z_only(
                local_source_ground, local_target_ground);
        PoseGraphEdge edge;
        edge.from = local_from;
        edge.to = local_to;
        edge.measurement = SE3(local_correction);
        const LoopConsistency consistency = evaluate_loop_consistency(
            edge, raw_consistency_reference_poses,
            local_path_length, raw_poses[local_to].t_);
        const bool graph_consistent =
            consistency.weight >= g_loop_min_consistency_weight;
        const bool local_support_valid = local_support.strict_valid;
        // A local crop is the final unit of evidence.  Any remaining resolved
        // two-mode residual means it is still averaging structures and must be
        // rejected rather than recursively split or promoted as a graph edge.
        const bool local_mode_valid = !local_modes.significant;
        const bool local_valid =
            local_coarse_valid && geometry.valid &&
            graph_consistent && local_support_valid &&
            local_mode_valid;
        V6 ground_z_only_measurement = V6::Zero();
        ground_z_only_measurement(5) = ground_z_only.z_adjustment;
        PoseGraphEdge ground_z_only_edge;
        ground_z_only_edge.from = local_from;
        ground_z_only_edge.to = local_to;
        ground_z_only_edge.measurement = SE3(ground_z_only_measurement);
        ground_z_only_edge.loop = true;
        ground_z_only_edge.ground_z_valid =
            ground_z_only.valid || ground_z_only.distributed_valid;
        ground_z_only_edge.proactive_ground_z_only = true;
        const LoopConsistency ground_z_only_consistency =
            evaluate_loop_consistency(
                ground_z_only_edge, raw_consistency_reference_poses,
                local_path_length, raw_poses[local_to].t_);
        constexpr float kMinimumZOnlyTightOverlap = 0.65f;
        constexpr float kMaximumZOnlyTightRmse = 0.18f;
        constexpr int kMinimumZOnlyReciprocalMatches = 80;
        constexpr int kMinimumZOnlySupportVoxels = 12;
        const float minimum_z_only_consistency =
            std::max(0.60f, g_loop_min_consistency_weight);
        const bool ground_z_only_scene_valid =
            local_coarse_valid &&
            local_overlap >= kMinimumZOnlyTightOverlap &&
            geometry.symmetric_overlap >= kMinimumZOnlyTightOverlap &&
            geometry.symmetric_trimmed_rmse <= kMaximumZOnlyTightRmse &&
            static_cast<int>(local_matches.size()) >=
                std::max(kMinimumZOnlyReciprocalMatches,
                         local_support.minimum_total_matches) &&
            static_cast<int>(local_support.voxels.size()) >=
                kMinimumZOnlySupportVoxels &&
            local_support.support_span >=
                g_loop_min_verification_span &&
            local_support.support_minor_span >=
                g_loop_verification_block_size &&
            local_mode_valid;
        const bool ground_z_only_consistent =
            ground_z_only_consistency.weight >=
                minimum_z_only_consistency;
        const bool ground_z_only_proposal_valid =
            ground_z_only.distributed_valid &&
            ground_z_only_scene_valid &&
            ground_z_only_consistent;
        const double selection_cost =
            local_score +
            (1.0 - static_cast<double>(consistency.weight)) +
            0.1 * (1.0 - static_cast<double>(local_overlap)) +
            0.5 * (1.0 - static_cast<double>(geometry.confidence));
        LOG(INFO) << GREEN
                  << " ---> 中途回环局部锚精验。earlier_idx: "
                  << revisit.earlier
                  << " later_idx: " << revisit.later
                  << " seed_translation_mode: "
                  << attempt.seed_translation_mode
                  << " anchor_id: " << anchor
                  << " anchor_xyz: "
                  << anchor_centers[anchor].transpose()
                  << " edge_from: " << local_from
                  << " edge_to: " << local_to
                  << " crop_radius: " << selected_radius
                  << " pre_projection_fitness: "
                  << pre_projection_fitness
                  << " score: " << local_score
                  << " overlap_ratio: " << local_overlap
                  << " reciprocal_matches: " << local_matches.size()
                  << " reciprocal_matches_required: "
                  << local_support.minimum_total_matches
                  << " support_voxels_3d: "
                  << local_support.voxels.size()
                  << " support_voxels_required: "
                  << g_loop_min_verification_blocks
                  << " support_pairs_per_voxel_threshold: "
                  << local_support.minimum_matches_per_voxel
                  << " support_span: " << local_support.support_span
                  << " support_span_required: "
                  << g_loop_min_verification_span
                  << " support_minor_span: "
                  << local_support.support_minor_span
                  << " support_minor_span_required: "
                  << g_loop_verification_block_size
                  << " support_valid: " << local_support_valid
                  << " coarse_valid: " << local_coarse_valid
                  << " geometry_valid: " << geometry.valid
                  << " graph_consistent: " << graph_consistent
                  << " residual_mode_valid: " << local_mode_valid
                  << " residual_mode_separation: "
                  << local_modes.separation
                  << " residual_within_mad: "
                  << local_modes.within_mode_mad
                  << " residual_separation_limit: "
                  << local_modes.separation_threshold
                  << " residual_bimodal: "
                  << local_modes.significant
                  << " tight_symmetric_overlap: "
                  << geometry.symmetric_overlap
                  << " tight_trimmed_rmse: "
                  << geometry.symmetric_trimmed_rmse
                  << " structural_symmetric_overlap: "
                  << geometry.structural_symmetric_overlap
                  << " anchor_translation: "
                  << geometry.anchor_translation
                  << " yaw_deg: " << geometry.yaw_deg
                  << " ground_pairs: " << ground_z.pair_count
                  << " ground_residual_mad: "
                  << ground_z.residual_mad
                  << " ground_z_adjustment: "
                  << ground_z.z_adjustment
                  << " proactive_z_valid: " << ground_z_only.valid
                  << " proactive_z_distributed_valid: "
                  << ground_z_only.distributed_valid
                  << " proactive_z_adjustment: "
                  << ground_z_only.z_adjustment
                  << " proactive_z_pairs: "
                  << ground_z_only.pair_count
                  << " proactive_z_inlier_ratio: "
                  << ground_z_only.inlier_ratio
                  << " proactive_z_mad: "
                  << ground_z_only.residual_mad
                  << " proactive_z_block_mad: "
                  << ground_z_only.block_residual_mad
                  << " proactive_z_blocks: "
                  << ground_z_only.supported_blocks
                  << " proactive_z_span: "
                  << ground_z_only.support_span
                  << " proactive_z_minor_span: "
                  << ground_z_only.support_minor_span
                  << " proactive_z_source_slope_deg: "
                  << ground_z_only.source_ground_slope_deg
                  << " proactive_z_target_slope_deg: "
                  << ground_z_only.target_ground_slope_deg
                  << " proactive_z_pair_xy_p90: "
                  << ground_z_only.pair_xy_distance_p90
                  << " proactive_z_slope_height_ambiguity: "
                  << ground_z_only.slope_height_ambiguity
                  << " proactive_z_slope_height_limit: "
                  << ground_z_only.maximum_slope_height_ambiguity
                  << " proactive_z_independent_regions: "
                  << ground_z_only.independent_regions
                  << " proactive_z_region_separation: "
                  << ground_z_only.independent_region_separation
                  << " proactive_z_region_z_difference: "
                  << ground_z_only.independent_region_z_difference
                  << " proactive_z_scene_valid: "
                  << ground_z_only_scene_valid
                  << " proactive_z_consistency_weight: "
                  << ground_z_only_consistency.weight
                  << " proactive_z_proposal_valid: "
                  << ground_z_only_proposal_valid
                  << " consistency_weight: "
                  << consistency.weight
                  << " selection_cost: " << selection_cost
                  << " accepted: " << local_valid << RESET;
        if (ground_z_only_proposal_valid && !local_valid) {
          InternalGroundZOnlyProposal proposal;
          proposal.edge = ground_z_only_edge;
          proposal.center = Eigen::Vector3f(
              ground_z_only.support_center_x,
              ground_z_only.support_center_y,
              0.5f * (raw_poses[local_from].t_.z() +
                      raw_poses[local_to].t_.z()));
          proposal.evidence = ground_z_only;
          proposal.consistency = ground_z_only_consistency;
          proposal.evidence_group_id = revisit.evidence_group_id;
          proposal.original_earlier = revisit.earlier;
          proposal.original_later = revisit.later;
          proposal.seed_translation_mode =
              attempt.seed_translation_mode;
          proposal.reciprocal_matches =
              static_cast<int>(local_matches.size());
          proposal.support_voxels =
              static_cast<int>(local_support.voxels.size());
          proposal.support_span = local_support.support_span;
          proposal.support_minor_span =
              local_support.support_minor_span;
          proposal.tight_overlap = geometry.symmetric_overlap;
          proposal.tight_rmse = geometry.symmetric_trimmed_rmse;
          proposal.full_ground_footprint = ground_z_only.valid;
          const float pair_quality = std::min(
              1.0f, static_cast<float>(ground_z_only.inlier_count) /
                        160.0f);
          const float block_quality = std::min(
              1.0f, static_cast<float>(ground_z_only.supported_blocks) /
                        12.0f);
          const float footprint_weight =
              proposal.full_ground_footprint ? 1.0f : 0.55f;
          proposal.raw_weight = 0.75f * footprint_weight *
              ground_z_only_consistency.weight *
              std::max(0.25f, pair_quality * block_quality);
          ground_z_only_proposals.push_back(std::move(proposal));
          LOG(INFO) << GREEN
                    << " ---> 中途回环独立纯Z局部提案。group_id: "
                    << revisit.evidence_group_id
                    << " original_pair: " << revisit.earlier
                    << " -> " << revisit.later
                    << " edge_pair: " << local_from
                    << " -> " << local_to
                    << " seed_translation_mode: "
                    << attempt.seed_translation_mode
                    << " anchor_id: " << anchor
                    << " support_center_xy: "
                    << ground_z_only.support_center_x << " "
                    << ground_z_only.support_center_y
                    << " z_adjustment: "
                    << ground_z_only.z_adjustment
                    << " pairs: " << ground_z_only.pair_count
                    << " blocks: " << ground_z_only.supported_blocks
                    << " ground_span: "
                    << ground_z_only.support_span
                    << " ground_minor_span: "
                    << ground_z_only.support_minor_span
                    << " full_ground_footprint: "
                    << ground_z_only_proposals.back()
                           .full_ground_footprint
                    << " consistency_weight: "
                    << ground_z_only_consistency.weight
                    << " raw_weight: "
                    << ground_z_only_proposals.back().raw_weight
                    << " full_se3_accepted: false" << RESET;
        }
        if (!local_valid) {
          continue;
        }

        InternalLocalAnchorResult result;
        result.edge = edge;
        result.correction = local_correction;
        result.center = anchor_centers[anchor];
        result.consistency = consistency;
        result.geometry = geometry;
        result.ground_z = ground_z;
        result.residual_modes = local_modes;
        result.source_window = source_window;
        result.target_window = target_window;
        result.score = local_score;
        result.selection_cost = selection_cost;
        result.overlap_ratio = local_overlap;
        result.crop_radius = selected_radius;
        result.raw_weight =
            1.5f * consistency.weight *
            std::max(0.25f, geometry.confidence);
        result.reciprocal_matches =
            static_cast<int>(local_matches.size());
        result.support_voxels =
            static_cast<int>(local_support.voxels.size());
        local_results.push_back(std::move(result));
      }
      if (local_results.empty()) {
        continue;
      }

      // Choose the largest mutually consistent set of local anchors.  Compare
      // corrections at a shared physical location; matrix translation alone
      // is not the sensor displacement when yaw rotates around world origin.
      const auto local_anchors_are_consistent = [&] (
          const InternalLocalAnchorResult& lhs,
          const InternalLocalAnchorResult& rhs) {
        if (lhs.edge.from == rhs.edge.from &&
            lhs.edge.to == rhs.edge.to) {
          return false;
        }
        const Eigen::Vector3f common_anchor =
            0.5f * (lhs.center + rhs.center);
        const auto correction_delta = [&] (
            const Eigen::Matrix4f& correction) {
          return correction.block<3, 3>(0, 0) * common_anchor +
              correction.block<3, 1>(0, 3) - common_anchor;
        };
        const float translation_difference =
            (correction_delta(lhs.correction) -
             correction_delta(rhs.correction)).norm();
        const float place_separation =
            (lhs.center - rhs.center).norm();
        const float local_noise = std::max(
            lhs.geometry.symmetric_trimmed_rmse,
            rhs.geometry.symmetric_trimmed_rmse);
        const float translation_limit = std::max(
            2.0f * g_loop_map_ds_size,
            g_loop_translation_drift_ratio *
                std::max(1.0f, place_separation) +
                3.0f * local_noise);
        const float lhs_yaw = std::atan2(
            lhs.correction(1, 0), lhs.correction(0, 0));
        const float rhs_yaw = std::atan2(
            rhs.correction(1, 0), rhs.correction(0, 0));
        const float yaw_difference =
            std::abs(std::atan2(
                std::sin(lhs_yaw - rhs_yaw),
                std::cos(lhs_yaw - rhs_yaw))) *
            180.0f / static_cast<float>(M_PI);
        const float yaw_limit = std::max(
            0.25f,
            g_loop_rotation_drift_deg_per_m *
                std::max(1.0f, place_separation));
        return translation_difference <= translation_limit &&
            yaw_difference <= yaw_limit;
      };

      std::size_t medoid = 0;
      int medoid_count = -1;
      double medoid_cost = std::numeric_limits<double>::max();
      for (std::size_t pivot = 0;
           pivot < local_results.size();
           ++pivot) {
        int count = 1;
        double cost = local_results[pivot].selection_cost;
        for (std::size_t other = 0;
             other < local_results.size();
             ++other) {
          if (other != pivot && local_anchors_are_consistent(
                                    local_results[pivot],
                                    local_results[other])) {
            ++count;
            cost += local_results[other].selection_cost;
          }
        }
        if (count > medoid_count ||
            (count == medoid_count && cost < medoid_cost)) {
          medoid = pivot;
          medoid_count = count;
          medoid_cost = cost;
        }
      }
      InternalLocalAnchorResultVector consistent_results;
      consistent_results.reserve(local_results.size());
      consistent_results.push_back(local_results[medoid]);
      for (std::size_t other = 0;
           other < local_results.size();
           ++other) {
        if (other == medoid ||
            !local_anchors_are_consistent(
                local_results[medoid], local_results[other])) {
          continue;
        }
        bool duplicate_pair = false;
        bool compatible_with_group = true;
        for (const auto& selected : consistent_results) {
          if (selected.edge.from == local_results[other].edge.from &&
              selected.edge.to == local_results[other].edge.to) {
            duplicate_pair = true;
            break;
          }
          if (!local_anchors_are_consistent(
                  selected, local_results[other])) {
            compatible_with_group = false;
            break;
          }
        }
        if (!duplicate_pair && compatible_with_group) {
          consistent_results.push_back(local_results[other]);
        }
      }
      const double average_cost = std::accumulate(
          consistent_results.begin(), consistent_results.end(), 0.0,
          [](const double sum,
             const InternalLocalAnchorResult& result) {
            return sum + result.selection_cost;
          }) / static_cast<double>(consistent_results.size());
      if (consistent_results.size() > best_group_results.size() ||
          (consistent_results.size() == best_group_results.size() &&
           average_cost < best_group_cost)) {
        best_group_results = std::move(consistent_results);
        best_group_cost = average_cost;
        best_group_seed_mode = attempt.seed_translation_mode;
      }
      if (best_group_results.size() >=
          static_cast<std::size_t>(kMaximumLocalAnchorsPerRevisit)) {
        break;
      }
    }

    // A local anchor crop can be too small to supply two spatial votes even
    // though the complete revisit contains a long, clean common ground
    // footprint.  Evaluate that footprint once at raw-world XY.  The full
    // GICP result is used only to prove place identity; its translation is
    // never reused as the Z measurement.  A wide proposal must contain two
    // independently separated ground regions and can create only a pure-Z
    // edge, so it cannot authorize endpoint, XY, or yaw correction.
    if (best_soft_fallback.has_value()) {
      const CloudPtr full_source_ground =
          extract_loop_ground_envelope(
              revisit_source, raw_poses[revisit.later].t_);
      const CloudPtr full_target_ground =
          extract_loop_ground_envelope(
              revisit_target, raw_poses[revisit.earlier].t_);
      const GroundZOnlyEvidence full_ground_z_only =
          estimate_ground_z_only(
              full_source_ground, full_target_ground);
      V6 full_ground_z_measurement = V6::Zero();
      full_ground_z_measurement(5) =
          full_ground_z_only.z_adjustment;
      PoseGraphEdge full_ground_z_edge;
      full_ground_z_edge.from = revisit.earlier;
      full_ground_z_edge.to = revisit.later;
      full_ground_z_edge.measurement = SE3(full_ground_z_measurement);
      full_ground_z_edge.loop = true;
      full_ground_z_edge.ground_z_valid =
          full_ground_z_only.wide_valid;
      full_ground_z_edge.proactive_ground_z_only = true;
      const LoopConsistency full_ground_z_consistency =
          evaluate_loop_consistency(
              full_ground_z_edge, raw_consistency_reference_poses,
              segment_path_length, raw_poses[revisit.later].t_);
      constexpr float kWideGroundMaximumRawXyDistance = 0.75f;
      constexpr float kWideGroundMinimumTightOverlap = 0.80f;
      constexpr float kWideGroundMaximumTightRmse = 0.15f;
      const bool full_ground_scene_valid =
          !best_soft_fallback->residual_modes.significant &&
          best_soft_fallback->geometry.valid &&
          best_soft_fallback->geometry.symmetric_overlap >=
              kWideGroundMinimumTightOverlap &&
          best_soft_fallback->geometry.symmetric_trimmed_rmse <=
              kWideGroundMaximumTightRmse &&
          best_soft_fallback->consistency.weight >=
              std::max(0.80f, g_loop_min_consistency_weight) &&
          revisit.horizontal_distance <=
              kWideGroundMaximumRawXyDistance &&
          best_soft_fallback->reciprocal_matches >=
              2 * g_loop_ground_z_min_pairs &&
          best_soft_fallback->support_voxels >=
              2 * g_loop_min_verification_blocks;
      const bool full_ground_proposal_valid =
          full_ground_z_only.wide_valid &&
          full_ground_z_consistency.weight >=
              std::max(0.80f, g_loop_min_consistency_weight) &&
          full_ground_scene_valid;
      LOG(INFO) << (full_ground_proposal_valid ? GREEN : YELLOW)
                << " ---> 中途回环独立纯Z宽地面复核。group_id: "
                << revisit.evidence_group_id
                << " original_pair: " << revisit.earlier
                << " -> " << revisit.later
                << " raw_xy_distance: "
                << revisit.horizontal_distance
                << " z_adjustment: "
                << full_ground_z_only.z_adjustment
                << " pairs: " << full_ground_z_only.pair_count
                << " inlier_ratio: "
                << full_ground_z_only.inlier_ratio
                << " residual_mad: "
                << full_ground_z_only.residual_mad
                << " wide_valid: "
                << full_ground_z_only.wide_valid
                << " blocks: "
                << full_ground_z_only.supported_blocks
                << " ground_span: "
                << full_ground_z_only.support_span
                << " ground_minor_span: "
                << full_ground_z_only.support_minor_span
                << " independent_regions: "
                << full_ground_z_only.independent_regions
                << " region_separation: "
                << full_ground_z_only.independent_region_separation
                << " region_z_difference: "
                << full_ground_z_only.independent_region_z_difference
                << " source_slope_deg: "
                << full_ground_z_only.source_ground_slope_deg
                << " target_slope_deg: "
                << full_ground_z_only.target_ground_slope_deg
                << " pair_xy_p90: "
                << full_ground_z_only.pair_xy_distance_p90
                << " slope_height_ambiguity: "
                << full_ground_z_only.slope_height_ambiguity
                << " slope_height_limit: "
                << full_ground_z_only.maximum_slope_height_ambiguity
                << " tight_overlap: "
                << best_soft_fallback->geometry.symmetric_overlap
                << " tight_rmse: "
                << best_soft_fallback->geometry.symmetric_trimmed_rmse
                << " consistency_weight: "
                << full_ground_z_consistency.weight
                << " scene_valid: " << full_ground_scene_valid
                << " accepted: " << full_ground_proposal_valid
                << RESET;
      if (full_ground_proposal_valid) {
        InternalGroundZOnlyProposal proposal;
        proposal.edge = full_ground_z_edge;
        proposal.center = Eigen::Vector3f(
            full_ground_z_only.support_center_x,
            full_ground_z_only.support_center_y,
            0.5f * (raw_poses[revisit.earlier].t_.z() +
                    raw_poses[revisit.later].t_.z()));
        proposal.evidence = full_ground_z_only;
        proposal.consistency = full_ground_z_consistency;
        proposal.evidence_group_id = revisit.evidence_group_id;
        proposal.original_earlier = revisit.earlier;
        proposal.original_later = revisit.later;
        proposal.seed_translation_mode =
            best_soft_fallback_seed_mode;
        proposal.reciprocal_matches =
            best_soft_fallback->reciprocal_matches;
        proposal.support_voxels =
            best_soft_fallback->support_voxels;
        proposal.support_span = std::min(
            best_soft_fallback->geometry.source_to_target.support_span,
            best_soft_fallback->geometry.target_to_source.support_span);
        proposal.support_minor_span = std::min(
            best_soft_fallback->geometry.source_to_target
                .support_minor_span,
            best_soft_fallback->geometry.target_to_source
                .support_minor_span);
        proposal.tight_overlap =
            best_soft_fallback->geometry.symmetric_overlap;
        proposal.tight_rmse =
            best_soft_fallback->geometry.symmetric_trimmed_rmse;
        proposal.full_ground_footprint =
            full_ground_z_only.wide_valid;
        const float pair_quality = std::min(
            1.0f, static_cast<float>(full_ground_z_only.inlier_count) /
                      320.0f);
        const float block_quality = std::min(
            1.0f, static_cast<float>(full_ground_z_only.supported_blocks) /
                      24.0f);
        proposal.raw_weight = 0.75f *
            full_ground_z_consistency.weight *
            std::max(0.35f, pair_quality * block_quality);
        ground_z_only_proposals.push_back(std::move(proposal));
      }
    }
    if (best_group_results.empty() && best_soft_fallback.has_value()) {
      best_group_results.push_back(*best_soft_fallback);
      best_group_cost = best_soft_fallback_cost;
      best_group_seed_mode = best_soft_fallback_seed_mode;
      LOG(WARNING) << YELLOW
                   << " ---> 中途回环无 strict 局部锚，提交低权 soft fallback。"
                   << " earlier_idx: " << revisit.earlier
                   << " later_idx: " << revisit.later
                   << " fallback_weight: "
                   << best_soft_fallback->raw_weight
                   << " residual_bimodal: "
                   << best_soft_fallback->residual_modes.significant
                   << " endpoint_support_eligible: false"
                   << RESET;
    }
    if (best_group_results.empty()) {
      LOG(WARNING) << YELLOW
                   << " ---> 中途回环无局部锚且 soft fallback 不满足门限，硬拒绝。"
                   << " earlier_idx: " << revisit.earlier
                   << " later_idx: " << revisit.later
                   << " coarse_attempts: " << coarse_attempts.size()
                   << RESET;
      continue;
    }

    float raw_weight_square_sum = 0.0f;
    float group_weight_budget = 0.0f;
    for (const auto& result : best_group_results) {
      raw_weight_square_sum += result.raw_weight * result.raw_weight;
      group_weight_budget = std::max(
          group_weight_budget, result.raw_weight);
    }
    const float raw_weight_norm =
        std::sqrt(std::max(raw_weight_square_sum, 1.0e-12f));
    float normalized_weight_square_sum = 0.0f;
    const bool soft_group =
        best_group_results.front().edge.soft_fallback;
    const int group_id = revisit.evidence_group_id;
    internal_evidence_group_weight_budget[group_id] = std::max(
        internal_evidence_group_weight_budget[group_id],
        group_weight_budget);
    for (std::size_t anchor = 0;
         anchor < best_group_results.size();
         ++anchor) {
      auto& result = best_group_results[anchor];
      result.edge.loop_group_id = group_id;
      result.edge.anchor_id =
          next_internal_anchor_id_by_group[group_id]++;
      result.edge.weight =
          group_weight_budget * result.raw_weight /
          raw_weight_norm;
      result.edge.loop = true;
      result.edge.ground_z_valid = result.ground_z.valid;
      normalized_weight_square_sum +=
          result.edge.weight * result.edge.weight;
      accepted_internal_edges.push_back(result.edge);
      ++accepted_internal_loops;
      LOG(INFO) << GREEN
                << " ---> 中途回环提交局部锚边。group_id: "
                << group_id
                << " anchor_id: " << result.edge.anchor_id
                << " soft_fallback: " << result.edge.soft_fallback
                << " from: " << result.edge.from
                << " to: " << result.edge.to
                << " anchor_xyz: " << result.center.transpose()
                << " crop_radius: " << result.crop_radius
                << " raw_weight: " << result.raw_weight
                << " normalized_weight: " << result.edge.weight
                << " score: " << result.score
                << " overlap_ratio: " << result.overlap_ratio
                << " consistency_weight: "
                << result.consistency.weight
                << " geometry_confidence: "
                << result.geometry.confidence << RESET;
    }
    LOG(INFO) << GREEN
              << " ---> 中途回环组内信息归一。group_id: "
              << group_id
              << " original_pair: " << revisit.earlier
              << " -> " << revisit.later
              << " seed_translation_mode: "
              << best_group_seed_mode
              << " anchors: " << best_group_results.size()
              << " strict_group: " << !soft_group
              << " soft_fallback_group: " << soft_group
              << " group_weight_budget: " << group_weight_budget
              << " raw_weight_square_sum: "
              << raw_weight_square_sum
              << " normalized_weight_square_sum: "
              << normalized_weight_square_sum
              << " budget_square: "
              << group_weight_budget * group_weight_budget
              << RESET;
  }

  // A failed full-SE(3) registration may still contain a strong, directly
  // observed height correction.  Admit it only as a distributed pure-Z
  // evidence group: duplicate seed basins are collapsed, at least two
  // spatially separated ground anchors must agree, and the permitted height
  // change grows only with the actually verified corridor span.  This is the
  // safe path for long flat-corridor Z drift; ramps and true multi-floor jumps
  // have already failed the per-anchor slope/range gates above.
  std::map<int, InternalGroundZOnlyProposalVector>
      ground_z_only_proposals_by_group;
  for (const auto& proposal : ground_z_only_proposals) {
    ground_z_only_proposals_by_group[proposal.evidence_group_id]
        .push_back(proposal);
  }
  constexpr float kZOnlyDuplicateCenterDistance = 2.0f;
  constexpr float kZOnlyMinimumAnchorSeparation = 8.0f;
  constexpr float kZOnlyMaximumVerticalPathRatio = 0.01f;
  // Height drift may change gradually along a long revisit.  A fixed
  // all-anchor Z spread rejects exactly the distributed evidence we need,
  // while accepting each anchor independently can create ripples.  Require a
  // spatially Lipschitz correction field instead: every accepted pair of
  // anchors must agree within measurement noise plus 2 cm per metre.
  constexpr float kZOnlyPairNoiseAllowance = 0.05f;
  constexpr float kZOnlyMaximumSpatialGradient = 0.02f;
  for (auto& [group_id, proposals] :
       ground_z_only_proposals_by_group) {
    bool has_strict_full_edge = false;
    bool has_soft_full_edge = false;
    for (const auto& edge : accepted_internal_edges) {
      if (edge.loop_group_id != group_id) {
        continue;
      }
      if (edge.soft_fallback) {
        has_soft_full_edge = true;
      } else {
        has_strict_full_edge = true;
      }
    }

    std::sort(
        proposals.begin(), proposals.end(),
        [](const InternalGroundZOnlyProposal& lhs,
           const InternalGroundZOnlyProposal& rhs) {
          if (lhs.evidence.independent_regions !=
              rhs.evidence.independent_regions) {
            return lhs.evidence.independent_regions >
                rhs.evidence.independent_regions;
          }
          if (lhs.full_ground_footprint !=
              rhs.full_ground_footprint) {
            return lhs.full_ground_footprint >
                rhs.full_ground_footprint;
          }
          if (lhs.raw_weight != rhs.raw_weight) {
            return lhs.raw_weight > rhs.raw_weight;
          }
          if (lhs.edge.from != rhs.edge.from) {
            return lhs.edge.from < rhs.edge.from;
          }
          return lhs.edge.to < rhs.edge.to;
        });
    InternalGroundZOnlyProposalVector unique_proposals;
    unique_proposals.reserve(proposals.size());
    for (const auto& proposal : proposals) {
      bool duplicate = false;
      for (const auto& selected : unique_proposals) {
        const float center_distance = std::hypot(
            proposal.center.x() - selected.center.x(),
            proposal.center.y() - selected.center.y());
        const bool same_temporal_anchor =
            proposal.edge.from == selected.edge.from &&
            proposal.edge.to == selected.edge.to;
        if (center_distance < kZOnlyDuplicateCenterDistance ||
            (same_temporal_anchor &&
             center_distance < kZOnlyMinimumAnchorSeparation)) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        unique_proposals.push_back(proposal);
      }
    }

    // A transitive corridor evidence group can contain several temporal
    // revisit offsets.  Do not let a far, unrelated offset inflate physical
    // span or vote count: select a non-transitive offset window first, then a
    // mutually consistent spatial Z field inside it.  Narrow observations
    // remain provisional here; a group must still contain a full-footprint
    // anchor before it can enter the graph.
    std::sort(
        unique_proposals.begin(), unique_proposals.end(),
        [](const InternalGroundZOnlyProposal& lhs,
           const InternalGroundZOnlyProposal& rhs) {
          return (lhs.edge.to - lhs.edge.from) <
              (rhs.edge.to - rhs.edge.from);
        });
    const int maximum_temporal_offset_spread =
        2 * g_loop_local_window_size;
    std::size_t best_count = 0;
    int best_full_footprint_count = 0;
    float best_anchor_span = 0.0f;
    float best_weight_sum = 0.0f;
    InternalGroundZOnlyProposalVector consensus;
    for (std::size_t temporal_begin = 0;
         temporal_begin < unique_proposals.size();
         ++temporal_begin) {
      std::size_t temporal_end = temporal_begin;
      const int minimum_offset =
          unique_proposals[temporal_begin].edge.to -
          unique_proposals[temporal_begin].edge.from;
      while (temporal_end < unique_proposals.size() &&
             unique_proposals[temporal_end].edge.to -
                     unique_proposals[temporal_end].edge.from -
                     minimum_offset <=
                 maximum_temporal_offset_spread) {
        ++temporal_end;
      }
      InternalGroundZOnlyProposalVector temporal_window(
          unique_proposals.begin() +
              static_cast<std::ptrdiff_t>(temporal_begin),
          unique_proposals.begin() +
              static_cast<std::ptrdiff_t>(temporal_end));
      std::sort(
          temporal_window.begin(), temporal_window.end(),
          [](const InternalGroundZOnlyProposal& lhs,
             const InternalGroundZOnlyProposal& rhs) {
            if (lhs.raw_weight != rhs.raw_weight) {
              return lhs.raw_weight > rhs.raw_weight;
            }
            return lhs.full_ground_footprint >
                rhs.full_ground_footprint;
          });
      for (std::size_t seed = 0;
           seed < temporal_window.size();
           ++seed) {
        InternalGroundZOnlyProposalVector candidate;
        candidate.reserve(temporal_window.size());
        candidate.push_back(temporal_window[seed]);
        float weight_sum = 0.0f;
        for (std::size_t slot = 0;
             slot < temporal_window.size();
             ++slot) {
          if (slot == seed) {
            continue;
          }
          const auto& proposal = temporal_window[slot];
          bool mutually_consistent = true;
          for (const auto& selected : candidate) {
            const float distance = std::hypot(
                proposal.center.x() - selected.center.x(),
                proposal.center.y() - selected.center.y());
            const float maximum_difference =
                kZOnlyPairNoiseAllowance +
                kZOnlyMaximumSpatialGradient * distance;
            const bool same_direction =
                proposal.evidence.z_adjustment *
                    selected.evidence.z_adjustment > 0.0f;
            if (!same_direction ||
                std::abs(
                    proposal.evidence.z_adjustment -
                    selected.evidence.z_adjustment) >
                    maximum_difference) {
              mutually_consistent = false;
              break;
            }
          }
          if (mutually_consistent) {
            candidate.push_back(proposal);
          }
        }
        int full_footprint_count = 0;
        float anchor_span = 0.0f;
        for (std::size_t i = 0; i < candidate.size(); ++i) {
          weight_sum += candidate[i].raw_weight;
          full_footprint_count +=
              candidate[i].full_ground_footprint ? 1 : 0;
          for (std::size_t j = i + 1; j < candidate.size(); ++j) {
            anchor_span = std::max(
                anchor_span,
                std::hypot(
                    candidate[i].center.x() - candidate[j].center.x(),
                    candidate[i].center.y() - candidate[j].center.y()));
          }
        }
        const bool better =
            candidate.size() > best_count ||
            (candidate.size() == best_count &&
             full_footprint_count > best_full_footprint_count) ||
            (candidate.size() == best_count &&
             full_footprint_count == best_full_footprint_count &&
             anchor_span > best_anchor_span + 1.0e-4f) ||
            (candidate.size() == best_count &&
             full_footprint_count == best_full_footprint_count &&
             std::abs(anchor_span - best_anchor_span) <= 1.0e-4f &&
             weight_sum > best_weight_sum);
        if (better) {
          consensus = std::move(candidate);
          best_count = consensus.size();
          best_full_footprint_count = full_footprint_count;
          best_anchor_span = anchor_span;
          best_weight_sum = weight_sum;
        }
      }
    }
    float maximum_anchor_separation = 0.0f;
    for (std::size_t i = 0; i < consensus.size(); ++i) {
      for (std::size_t j = i + 1; j < consensus.size(); ++j) {
        maximum_anchor_separation = std::max(
            maximum_anchor_separation,
            std::hypot(
                consensus[i].center.x() - consensus[j].center.x(),
                consensus[i].center.y() - consensus[j].center.y()));
      }
    }
    std::vector<float> z_adjustments;
    z_adjustments.reserve(consensus.size());
    int full_footprint_anchors = 0;
    int total_ground_pairs = 0;
    int total_ground_blocks = 0;
    float maximum_observed_z_gradient = 0.0f;
    int minimum_selected_offset = std::numeric_limits<int>::max();
    int maximum_selected_offset = std::numeric_limits<int>::lowest();
    for (const auto& proposal : consensus) {
      z_adjustments.push_back(proposal.evidence.z_adjustment);
      full_footprint_anchors +=
          proposal.full_ground_footprint ? 1 : 0;
      total_ground_pairs += proposal.evidence.pair_count;
      total_ground_blocks += proposal.evidence.supported_blocks;
      const int temporal_offset =
          proposal.edge.to - proposal.edge.from;
      minimum_selected_offset = std::min(
          minimum_selected_offset, temporal_offset);
      maximum_selected_offset = std::max(
          maximum_selected_offset, temporal_offset);
    }
    for (std::size_t i = 0; i < consensus.size(); ++i) {
      for (std::size_t j = i + 1; j < consensus.size(); ++j) {
        const float distance = std::hypot(
            consensus[i].center.x() - consensus[j].center.x(),
            consensus[i].center.y() - consensus[j].center.y());
        if (distance > 1.0e-3f) {
          maximum_observed_z_gradient = std::max(
              maximum_observed_z_gradient,
              std::abs(
                  consensus[i].evidence.z_adjustment -
                  consensus[j].evidence.z_adjustment) /
                  distance);
        }
      }
    }
    const float group_z_adjustment = z_adjustments.empty()
        ? 0.0f : robust_median(z_adjustments);
    bool vertical_path_ratio_valid = true;
    for (const auto& proposal : consensus) {
      const float vertical_path_ratio =
          std::abs(proposal.evidence.z_adjustment) /
          std::max(proposal.consistency.path_length, 1.0f);
      vertical_path_ratio_valid = vertical_path_ratio_valid &&
          vertical_path_ratio <= kZOnlyMaximumVerticalPathRatio;
    }
    float minimum_group_z = std::numeric_limits<float>::max();
    float maximum_group_z_observed =
        std::numeric_limits<float>::lowest();
    for (const float z_adjustment : z_adjustments) {
      minimum_group_z = std::min(minimum_group_z, z_adjustment);
      maximum_group_z_observed = std::max(
          maximum_group_z_observed, z_adjustment);
    }
    const float group_z_spread = z_adjustments.empty()
        ? std::numeric_limits<float>::max()
        : maximum_group_z_observed - minimum_group_z;
    const bool single_wide_ground_valid =
        consensus.size() == 1 &&
        consensus.front().full_ground_footprint &&
        consensus.front().evidence.independent_regions >= 2 &&
        consensus.front().evidence.pair_count >=
            2 * g_loop_ground_z_min_pairs &&
        consensus.front().evidence.supported_blocks >=
            2 * g_loop_min_verification_blocks &&
        consensus.front().evidence.support_span >=
            kZOnlyMinimumAnchorSeparation &&
        consensus.front().evidence.support_minor_span >=
            2.0f * g_loop_verification_block_size &&
        consensus.front().tight_overlap >= 0.80f &&
        consensus.front().tight_rmse <= 0.15f &&
        consensus.front().consistency.weight >=
            std::max(0.80f, g_loop_min_consistency_weight);
    const float effective_anchor_span = single_wide_ground_valid
        ? std::max(
              maximum_anchor_separation,
              consensus.front().evidence.support_span)
        : maximum_anchor_separation;
    float minimum_evidence_path_length =
        std::numeric_limits<float>::max();
    for (const auto& proposal : consensus) {
      minimum_evidence_path_length = std::min(
          minimum_evidence_path_length,
          proposal.consistency.path_length);
    }
    if (!std::isfinite(minimum_evidence_path_length)) {
      minimum_evidence_path_length = 0.0f;
    }
    const float maximum_group_z = std::min(
        3.0f,
        std::max(
            0.15f * effective_anchor_span + 0.25f,
            kZOnlyMaximumVerticalPathRatio *
                minimum_evidence_path_length));
    const bool multi_anchor_valid =
        single_wide_ground_valid ||
        (consensus.size() >= 3 && full_footprint_anchors >= 1) ||
        (consensus.size() >= 2 &&
         full_footprint_anchors == static_cast<int>(consensus.size()));
    const bool anchor_span_valid =
        effective_anchor_span >= kZOnlyMinimumAnchorSeparation;
    const int evidence_multiplier = std::max(
        2, std::min(3, static_cast<int>(consensus.size())));
    const bool aggregate_support_valid =
        total_ground_pairs >=
            evidence_multiplier * g_loop_ground_z_min_pairs &&
        total_ground_blocks >=
            evidence_multiplier * g_loop_min_verification_blocks;
    float maximum_abs_group_z = 0.0f;
    for (const float z_adjustment : z_adjustments) {
      maximum_abs_group_z = std::max(
          maximum_abs_group_z, std::abs(z_adjustment));
    }
    const bool deformation_scale_valid =
        maximum_abs_group_z <= maximum_group_z;
    const bool group_valid =
        !has_strict_full_edge && multi_anchor_valid &&
        anchor_span_valid && aggregate_support_valid &&
        deformation_scale_valid &&
        vertical_path_ratio_valid;
    LOG(INFO) << (group_valid ? GREEN : YELLOW)
              << " ---> 中途回环独立纯Z证据组。group_id: "
              << group_id
              << " raw_proposals: " << proposals.size()
              << " unique_proposals: " << unique_proposals.size()
              << " consensus_anchors: " << consensus.size()
              << " z_adjustment: " << group_z_adjustment
              << " z_spread: " << group_z_spread
              << " z_spread_is_gate: false"
              << " maximum_observed_z_gradient: "
              << maximum_observed_z_gradient
              << " maximum_z_gradient: "
              << kZOnlyMaximumSpatialGradient
              << " pair_noise_allowance: "
              << kZOnlyPairNoiseAllowance
              << " temporal_offset_spread: "
              << (consensus.empty()
                      ? -1
                      : maximum_selected_offset - minimum_selected_offset)
              << " maximum_temporal_offset_spread: "
              << maximum_temporal_offset_spread
              << " anchor_span: " << maximum_anchor_separation
              << " effective_anchor_span: "
              << effective_anchor_span
              << " minimum_anchor_span: "
              << kZOnlyMinimumAnchorSeparation
              << " single_wide_ground_valid: "
              << single_wide_ground_valid
              << " independent_regions: "
              << (consensus.empty()
                      ? 0
                      : consensus.front().evidence.independent_regions)
              << " independent_region_separation: "
              << (consensus.empty()
                      ? 0.0f
                      : consensus.front().evidence
                            .independent_region_separation)
              << " full_footprint_anchors: "
              << full_footprint_anchors
              << " total_ground_pairs: " << total_ground_pairs
              << " total_ground_blocks: " << total_ground_blocks
              << " aggregate_support_valid: "
              << aggregate_support_valid
              << " maximum_group_z: " << maximum_group_z
              << " vertical_path_ratio_valid: "
              << vertical_path_ratio_valid
              << " strict_full_edge_present: "
              << has_strict_full_edge
              << " soft_full_edge_present: "
              << has_soft_full_edge
              << " accepted: " << group_valid << RESET;
    if (!group_valid) {
      continue;
    }

    int replaced_soft_edges = 0;
    std::optional<PoseGraphEdge> planar_companion;
    int planar_companion_group_id = -1;
    if (has_soft_full_edge) {
      // The full fallback contains useful corridor-normal XY/yaw evidence,
      // but its roll/pitch/Z are superseded by the independently validated
      // ground observation.  Preserve at most one tightly bounded planar
      // component at the same information level reached after two normal
      // soft-edge downweight steps.  Giving it a separate group lets graph
      // safety remove it without weakening the pure-Z observation.
      constexpr float kPlanarCompanionMaximumTranslation = 0.75f;
      constexpr float kPlanarCompanionMaximumYawDeg = 1.0f;
      constexpr float kPlanarCompanionWeightScale = 0.25f;
      constexpr int kPlanarCompanionGroupOffset = 1000000;
      for (const auto& edge : accepted_internal_edges) {
        if (edge.loop_group_id != group_id || !edge.soft_fallback) {
          continue;
        }
        V6 planar_measurement = edge.measurement.log_vee();
        const float planar_translation = std::hypot(
            planar_measurement(3), planar_measurement(4));
        const float yaw_deg = std::abs(planar_measurement(2)) *
            180.0f / static_cast<float>(M_PI);
        if (!std::isfinite(planar_translation) ||
            !std::isfinite(yaw_deg) ||
            planar_translation > kPlanarCompanionMaximumTranslation ||
            yaw_deg > kPlanarCompanionMaximumYawDeg) {
          continue;
        }
        if (planar_companion.has_value() &&
            planar_companion->weight >= edge.weight) {
          continue;
        }
        planar_measurement(0) = 0.0f;
        planar_measurement(1) = 0.0f;
        planar_measurement(5) = 0.0f;
        PoseGraphEdge companion = edge;
        companion.measurement = SE3(planar_measurement);
        companion.weight *= kPlanarCompanionWeightScale;
        companion.safety_downweight_level = 2;
        companion.ground_z_valid = false;
        companion.ground_z_planar_hold = false;
        companion.proactive_ground_z_only = false;
        companion.proactive_planar_companion = true;
        companion.prediction_consistent = false;
        planar_companion_group_id =
            kPlanarCompanionGroupOffset + group_id;
        companion.loop_group_id = planar_companion_group_id;
        companion.anchor_id = 0;
        planar_companion = std::move(companion);
      }
      const auto new_end = std::remove_if(
          accepted_internal_edges.begin(), accepted_internal_edges.end(),
          [&] (const PoseGraphEdge& edge) {
            const bool remove = edge.loop_group_id == group_id;
            if (remove) {
              ++replaced_soft_edges;
            }
            return remove;
          });
      accepted_internal_edges.erase(new_end, accepted_internal_edges.end());
      accepted_internal_loops = std::max(
          0, accepted_internal_loops - replaced_soft_edges);
      if (planar_companion.has_value()) {
        internal_evidence_group_weight_budget[
            planar_companion_group_id] = planar_companion->weight;
        accepted_internal_edges.push_back(*planar_companion);
        ++accepted_internal_loops;
        const V6 planar_measurement =
            planar_companion->measurement.log_vee();
        LOG(INFO) << GREEN
                  << " ---> 独立纯Z保留低权平面伴随边。source_group_id: "
                  << group_id
                  << " companion_group_id: "
                  << planar_companion_group_id
                  << " from: " << planar_companion->from
                  << " to: " << planar_companion->to
                  << " planar_translation: "
                  << std::hypot(
                         planar_measurement(3), planar_measurement(4))
                  << " yaw_deg: "
                  << std::abs(planar_measurement(2)) *
                         180.0f / static_cast<float>(M_PI)
                  << " retained_weight: "
                  << planar_companion->weight
                  << " constrained_axes: yaw_xy"
                  << " endpoint_support_eligible: false" << RESET;
      }
    }

    float raw_weight_square_sum = 0.0f;
    float group_weight_budget = 0.0f;
    for (const auto& proposal : consensus) {
      raw_weight_square_sum +=
          proposal.raw_weight * proposal.raw_weight;
      group_weight_budget = std::max(
          group_weight_budget, proposal.raw_weight);
    }
    const float raw_weight_norm = std::sqrt(
        std::max(raw_weight_square_sum, 1.0e-12f));
    internal_evidence_group_weight_budget[group_id] =
        group_weight_budget;
    float normalized_weight_square_sum = 0.0f;
    for (auto& proposal : consensus) {
      // Preserve the locally observed correction field after the group has
      // proved that it varies smoothly in space.  Forcing every anchor to the
      // same median creates a step at both ends of a long corridor; the pose
      // graph and the group-level information budget provide the smoothing.
      V6 local_measurement = V6::Zero();
      local_measurement(5) = proposal.evidence.z_adjustment;
      proposal.edge.measurement = SE3(local_measurement);
      proposal.edge.loop_group_id = group_id;
      proposal.edge.anchor_id =
          next_internal_anchor_id_by_group[group_id]++;
      proposal.edge.weight =
          group_weight_budget * proposal.raw_weight / raw_weight_norm;
      normalized_weight_square_sum +=
          proposal.edge.weight * proposal.edge.weight;
      accepted_internal_edges.push_back(proposal.edge);
      ++accepted_internal_loops;
      LOG(INFO) << GREEN
                << " ---> 中途回环提交独立纯Z锚边。group_id: "
                << group_id
                << " anchor_id: " << proposal.edge.anchor_id
                << " from: " << proposal.edge.from
                << " to: " << proposal.edge.to
                << " support_center_xy: "
                << proposal.center.x() << " " << proposal.center.y()
                << " z_adjustment: "
                << proposal.evidence.z_adjustment
                << " group_z_adjustment: "
                << group_z_adjustment
                << " raw_weight: " << proposal.raw_weight
                << " normalized_weight: " << proposal.edge.weight
                << " replaced_soft_edges: " << replaced_soft_edges
                << " endpoint_support_eligible: false" << RESET;
    }
    LOG(INFO) << GREEN
              << " ---> 中途回环独立纯Z组内信息归一。group_id: "
              << group_id
              << " anchors: " << consensus.size()
              << " group_weight_budget: " << group_weight_budget
              << " raw_weight_square_sum: "
              << raw_weight_square_sum
              << " normalized_weight_square_sum: "
              << normalized_weight_square_sum
              << " budget_square: "
              << group_weight_budget * group_weight_budget
              << RESET;
  }

  // A corridor continuation may contribute several spatial anchors from two
  // index-overlapping revisit pairs.  They intentionally share one evidence
  // group; normalize their combined information once more across candidates
  // so the extra spatial coverage does not become an extra independent vote.
  std::map<int, float> evidence_group_weight_square_sum;
  std::map<int, int> evidence_group_edge_count;
  for (const auto& edge : accepted_internal_edges) {
    evidence_group_weight_square_sum[edge.loop_group_id] +=
        edge.weight * edge.weight;
    ++evidence_group_edge_count[edge.loop_group_id];
  }
  for (const auto& [group_id, square_sum] :
       evidence_group_weight_square_sum) {
    const float combined_norm = std::sqrt(std::max(
        square_sum, 1.0e-12f));
    const float budget = std::max(
        internal_evidence_group_weight_budget[group_id], 1.0e-6f);
    const float scale = std::min(1.0f, budget / combined_norm);
    for (auto& edge : accepted_internal_edges) {
      if (edge.loop_group_id == group_id) {
        edge.weight *= scale;
      }
    }
    if (scale < 1.0f - 1.0e-5f) {
      LOG(INFO) << GREEN
                << " ---> 中途回环跨候选证据组归一。group_id: "
                << group_id
                << " edges: " << evidence_group_edge_count[group_id]
                << " combined_weight_norm: " << combined_norm
                << " group_weight_budget: " << budget
                << " applied_scale: " << scale << RESET;
    }
  }
  accepted_internal_groups = static_cast<int>(
      evidence_group_weight_square_sum.size());

  // First optimize a graph containing odometry and raw-consistent internal
  // loops only. Endpoint evidence is intentionally absent: this correction
  // field is the independent prediction used to gate endpoint deformation.
  if (finalize_timed_out()) {
    return;
  }
  int strict_internal_loops = 0;
  int soft_internal_loops = 0;
  int proactive_ground_z_loops = 0;
  std::set<int> strict_internal_group_ids;
  std::set<int> soft_internal_group_ids;
  std::set<int> proactive_ground_z_group_ids;
  for (const auto& edge : accepted_internal_edges) {
    if (edge.proactive_ground_z_only) {
      ++proactive_ground_z_loops;
      proactive_ground_z_group_ids.insert(edge.loop_group_id);
    } else if (edge.soft_fallback) {
      ++soft_internal_loops;
      soft_internal_group_ids.insert(edge.loop_group_id);
    } else {
      ++strict_internal_loops;
      strict_internal_group_ids.insert(edge.loop_group_id);
    }
    pose_graph_edges.push_back(edge);
  }
  int strict_internal_groups =
      static_cast<int>(strict_internal_group_ids.size());
  int soft_internal_groups =
      static_cast<int>(soft_internal_group_ids.size());
  int proactive_ground_z_groups =
      static_cast<int>(proactive_ground_z_group_ids.size());
  PoseVector internal_reference_poses(
      loop_keyframes_.size(), SE3());
  bool internal_graph_valid = true;
  float internal_initial_cost = 0.0f;
  float internal_optimized_cost = 0.0f;
  if (!accepted_internal_edges.empty()) {
    internal_initial_cost = pose_graph_robust_cost(
        internal_reference_poses, pose_graph_edges);
    internal_graph_valid = optimize_pose_graph(
        internal_reference_poses, pose_graph_edges);
    if (internal_graph_valid) {
      internal_optimized_cost = pose_graph_robust_cost(
          internal_reference_poses, pose_graph_edges);
      internal_graph_valid =
          std::isfinite(internal_optimized_cost) &&
          internal_optimized_cost < internal_initial_cost;
    }
    LOG(INFO) << GREEN
              << " ---> internal-only 位姿图。internal_loops: "
              << accepted_internal_loops
              << " internal_groups: " << accepted_internal_groups
              << " strict_internal_loops: " << strict_internal_loops
              << " strict_internal_groups: " << strict_internal_groups
              << " soft_internal_loops: " << soft_internal_loops
              << " soft_internal_groups: " << soft_internal_groups
              << " proactive_ground_z_loops: "
              << proactive_ground_z_loops
              << " proactive_ground_z_groups: "
              << proactive_ground_z_groups
              << " graph_cost: " << internal_initial_cost
              << " -> " << internal_optimized_cost
              << " valid: " << internal_graph_valid << RESET;
    if (!internal_graph_valid) {
      LOG(WARNING) << YELLOW
                   << " ---> internal-only 位姿图无效，"
                   << "不允许其确认终点并从最终图移除这些内部边。"
                   << RESET;
      pose_graph_edges.resize(static_cast<std::size_t>(end_idx));
      accepted_internal_edges.clear();
      accepted_internal_loops = 0;
      accepted_internal_groups = 0;
      strict_internal_loops = 0;
      strict_internal_groups = 0;
      soft_internal_loops = 0;
      soft_internal_groups = 0;
      proactive_ground_z_loops = 0;
      proactive_ground_z_groups = 0;
      internal_reference_poses.assign(
          loop_keyframes_.size(), SE3());
    }
  }

  // Endpoint prediction must not inherit even a small deformation from soft
  // fallback groups. Build a second correction field from odometry plus
  // strict internal groups only. The complete internal graph above remains
  // the map optimizer input, but only this strict-only field may authorize a
  // large endpoint measurement.
  PoseVector strict_internal_reference_poses(
      loop_keyframes_.size(), SE3());
  bool strict_internal_graph_valid = false;
  float strict_internal_initial_cost = 0.0f;
  float strict_internal_optimized_cost = 0.0f;
  if (internal_graph_valid && strict_internal_groups >= 2) {
    PoseGraphEdgeVector strict_prediction_edges;
    strict_prediction_edges.reserve(
        static_cast<std::size_t>(end_idx + strict_internal_loops));
    strict_prediction_edges.insert(
        strict_prediction_edges.end(),
        pose_graph_edges.begin(),
        pose_graph_edges.begin() + static_cast<std::ptrdiff_t>(end_idx));
    for (const auto& edge : accepted_internal_edges) {
      if (!edge.soft_fallback && !edge.proactive_ground_z_only) {
        strict_prediction_edges.push_back(edge);
      }
    }
    strict_internal_initial_cost = pose_graph_robust_cost(
        strict_internal_reference_poses, strict_prediction_edges);
    strict_internal_graph_valid = optimize_pose_graph(
        strict_internal_reference_poses, strict_prediction_edges);
    if (strict_internal_graph_valid) {
      strict_internal_optimized_cost = pose_graph_robust_cost(
          strict_internal_reference_poses, strict_prediction_edges);
      strict_internal_graph_valid =
          std::isfinite(strict_internal_optimized_cost) &&
          strict_internal_optimized_cost < strict_internal_initial_cost;
    }
  }
  LOG(INFO) << GREEN
            << " ---> strict-only 终点预测图。strict_internal_loops: "
            << strict_internal_loops
            << " strict_internal_groups: " << strict_internal_groups
            << " soft_edges_in_prediction: 0"
            << " proactive_z_edges_in_prediction: 0"
            << " graph_cost: " << strict_internal_initial_cost
            << " -> " << strict_internal_optimized_cost
            << " valid: " << strict_internal_graph_valid << RESET;

  const int independent_index_margin =
      2 * g_loop_local_window_size;
  constexpr float kIndependentLoopPlaceDistance = 5.0f;
  const auto loop_place_center = [&] (
      const PoseGraphEdge& edge) -> V3 {
    return 0.5 * (raw_poses[edge.from].t_ + raw_poses[edge.to].t_);
  };
  const auto internal_edges_are_independent = [&] (
      const PoseGraphEdge& lhs, const PoseGraphEdge& rhs) {
    // Child anchors from one revisit improve spatial placement but remain one
    // physical observation.  They must never satisfy the endpoint's demand
    // for two independent internal loop groups.
    if (lhs.loop_group_id >= 0 &&
        lhs.loop_group_id == rhs.loop_group_id) {
      return false;
    }
    if (std::abs(lhs.from - rhs.from) <= independent_index_margin ||
        std::abs(lhs.to - rhs.to) <= independent_index_margin) {
      return false;
    }
    const V3 center_delta =
        loop_place_center(lhs) - loop_place_center(rhs);
    return std::hypot(center_delta.x(), center_delta.y()) >
        kIndependentLoopPlaceDistance;
  };

  // Endpoint selection happens only now. Two spatially and temporally
  // independent internal loop clusters authorize comparison with the
  // internal-only prediction. With fewer anchors, extrapolation is permitted
  // only when the requested endpoint deformation fits inside the verified
  // local support footprint.
  for (const auto& hypothesis : distinct_endpoint_hypotheses) {
    PoseVector raw_reference_poses(loop_keyframes_.size(), SE3());
    const LoopConsistency endpoint_consistency =
        hypothesis.edge.corridor_partial
        ? evaluate_loop_consistency(
              hypothesis.edge, raw_reference_poses,
              hypothesis.consistency.path_length,
              raw_poses[end_idx].t_)
        : hypothesis.consistency;
    const float minimum_raw_consistency = hypothesis.partial_geometry
        ? std::max(0.60f, g_loop_min_consistency_weight)
        : g_loop_min_consistency_weight;
    const bool raw_graph_consistent =
        endpoint_consistency.weight >= minimum_raw_consistency;

    std::vector<std::size_t> eligible_internal_edges;
    eligible_internal_edges.reserve(accepted_internal_edges.size());
    std::set<int> eligible_strict_internal_group_ids;
    std::set<int> endpoint_overlapping_internal_group_ids;
    for (const auto& edge : accepted_internal_edges) {
      const bool overlaps_endpoint_windows =
          std::abs(edge.from - hypothesis.edge.from) <=
              independent_index_margin ||
          end_idx - edge.to <= independent_index_margin;
      if (overlaps_endpoint_windows && edge.loop_group_id >= 0) {
        endpoint_overlapping_internal_group_ids.insert(
            edge.loop_group_id);
      }
    }
    for (std::size_t i = 0;
         i < accepted_internal_edges.size();
         ++i) {
      const auto& edge = accepted_internal_edges[i];
      // A low-weight aggregate fallback may gently improve a raw trajectory,
      // but it is not independent local evidence and cannot authorize a large
      // endpoint deformation.
      if (edge.soft_fallback || edge.proactive_ground_z_only) {
        continue;
      }
      // Correlation is a property of the complete physical evidence group.
      // A corridor sibling just outside the numeric endpoint margin must not
      // authorize the endpoint when another edge in the same group overlaps.
      if (endpoint_overlapping_internal_group_ids.count(
              edge.loop_group_id) == 0U) {
        eligible_internal_edges.push_back(i);
        eligible_strict_internal_group_ids.insert(edge.loop_group_id);
      }
    }

    // Find the most separated independent pair first. Starting a greedy set
    // with the first edge can under-count when that edge overlaps two anchors
    // that are independent from each other.
    std::vector<std::size_t> independent_internal_edges;
    float best_pair_separation = -1.0f;
    for (std::size_t i = 0; i < eligible_internal_edges.size(); ++i) {
      for (std::size_t j = i + 1;
           j < eligible_internal_edges.size();
           ++j) {
        const auto lhs_index = eligible_internal_edges[i];
        const auto rhs_index = eligible_internal_edges[j];
        const auto& lhs = accepted_internal_edges[lhs_index];
        const auto& rhs = accepted_internal_edges[rhs_index];
        if (!internal_edges_are_independent(lhs, rhs)) {
          continue;
        }
        const V3 center_delta =
            loop_place_center(lhs) - loop_place_center(rhs);
        const float separation =
            static_cast<float>(std::abs(lhs.from - rhs.from) +
                               std::abs(lhs.to - rhs.to)) +
            static_cast<float>(
                std::hypot(center_delta.x(), center_delta.y()));
        if (separation > best_pair_separation) {
          best_pair_separation = separation;
          independent_internal_edges = {lhs_index, rhs_index};
        }
      }
    }
    if (independent_internal_edges.size() >= 2) {
      for (const auto candidate : eligible_internal_edges) {
        if (std::find(
                independent_internal_edges.begin(),
                independent_internal_edges.end(),
                candidate) != independent_internal_edges.end()) {
          continue;
        }
        bool independent = true;
        for (const auto selected : independent_internal_edges) {
          if (!internal_edges_are_independent(
                  accepted_internal_edges[candidate],
                  accepted_internal_edges[selected])) {
            independent = false;
            break;
          }
        }
        if (independent) {
          independent_internal_edges.push_back(candidate);
        }
      }
    }

    const bool has_global_internal_support =
        internal_graph_valid && strict_internal_graph_valid &&
        independent_internal_edges.size() >= 2;
    LoopConsistency predicted_consistency;
    float prediction_translation_limit = 0.0f;
    float prediction_yaw_limit_deg = 0.0f;
    float uncovered_tail_path = raw_route_length;
    bool prediction_consistent = false;
    float support_span = std::min(
        hypothesis.geometry.source_to_target.support_span,
        hypothesis.geometry.target_to_source.support_span);
    const float evidence_deformation =
        hypothesis.geometry.anchor_translation +
        0.5f * support_span * hypothesis.geometry.yaw_deg *
            static_cast<float>(M_PI) / 180.0f;
    const float evidence_deformation_limit = 0.15f * support_span;
    bool local_evidence_scale_valid = false;

    if (has_global_internal_support) {
      int latest_support_to = 0;
      for (const auto edge_index : independent_internal_edges) {
        latest_support_to = std::max(
            latest_support_to,
            accepted_internal_edges[edge_index].to);
      }
      uncovered_tail_path =
          cumulative_route_length[end_idx] -
          cumulative_route_length[latest_support_to];
      prediction_translation_limit = std::min(
          1.50f, 0.50f + 0.01f * uncovered_tail_path);
      prediction_yaw_limit_deg = std::min(
          2.0f, 0.50f + 0.01f * uncovered_tail_path);
      predicted_consistency = evaluate_loop_consistency(
          hypothesis.edge,
          strict_internal_reference_poses,
          endpoint_consistency.path_length,
          raw_poses[end_idx].t_);
      prediction_consistent =
          predicted_consistency.translation_residual <=
              prediction_translation_limit &&
          predicted_consistency.rotation_residual_deg <=
              prediction_yaw_limit_deg;
    } else if (!hypothesis.two_vote_strict_provisional) {
      local_evidence_scale_valid =
          support_span >= 8.0f &&
          evidence_deformation <= evidence_deformation_limit;
    }

    const bool regular_endpoint_gate_valid =
        raw_graph_consistent &&
        (hypothesis.two_vote_strict_provisional
             ? (has_global_internal_support && prediction_consistent)
             : (has_global_internal_support
                    ? prediction_consistent
                    : local_evidence_scale_valid));
    const float target_route_fraction = raw_route_length > 1.0e-3f
        ? cumulative_route_length[hypothesis.index] / raw_route_length
        : 1.0f;
    const bool two_vote_guarded_evidence_valid =
        hypothesis.two_vote_strict_provisional &&
        hypothesis.two_vote_all_strong &&
        target_route_fraction <= 0.02f &&
        std::isfinite(hypothesis.post_anchor_distance) &&
        hypothesis.post_anchor_distance <= 0.50f &&
        hypothesis.geometry.symmetric_overlap >= 0.70f &&
        hypothesis.geometry.symmetric_trimmed_rmse <= 0.15f &&
        hypothesis.geometry.structural_valid;
    const bool corridor_guarded_evidence_valid =
        hypothesis.edge.corridor_partial &&
        hypothesis.corridor_sequence_verified &&
        target_route_fraction <= 0.05f &&
        hypothesis.corridor_sequence_matches >= 5 &&
        hypothesis.corridor_sequence_span >= std::max(
            2.0f, 4.0f * g_loop_keyframe_min_distance) &&
        hypothesis.corridor_sequence_rmse <= std::max(
            0.35f, 0.70f * g_loop_keyframe_min_distance) &&
        hypothesis.corridor_sequence_density >= 0.60f &&
        hypothesis.corridor_tangent_mismatch_deg <= 6.0f &&
        hypothesis.geometry.symmetric_overlap >= 0.70f &&
        hypothesis.geometry.symmetric_trimmed_rmse <= 0.15f &&
        hypothesis.geometry.structural_valid &&
        hypothesis.geometry.structural_symmetric_overlap >= 0.65f &&
        hypothesis.post_anchor_distance <= std::max(
            0.75f, 1.5f * g_loop_keyframe_min_distance);
    const bool guarded_endpoint_evidence_valid =
        two_vote_guarded_evidence_valid ||
        corridor_guarded_evidence_valid;
    // This guarded path is authorized by the two strong, independent endpoint
    // registrations themselves, never by a soft internal group. The internal
    // graph contributes only the baseline measurement used by safety line
    // search below.
    const bool guarded_endpoint_proposal =
        !regular_endpoint_gate_valid && raw_graph_consistent &&
        guarded_endpoint_evidence_valid;
    const bool endpoint_gate_valid =
        regular_endpoint_gate_valid || guarded_endpoint_proposal;
    LOG(INFO) << GREEN
              << " ---> 终点回环多锚点门控。candidate_idx: "
              << hypothesis.index
              << " partial_geometry: " << hypothesis.partial_geometry
              << " two_vote_strict_provisional: "
              << hypothesis.two_vote_strict_provisional
              << " two_vote_all_strong: "
              << hypothesis.two_vote_all_strong
              << " consensus_count: " << hypothesis.consensus_count
              << " raw_path_length: "
              << endpoint_consistency.path_length
              << " raw_translation_residual: "
              << endpoint_consistency.translation_residual
              << " raw_rotation_residual_deg: "
              << endpoint_consistency.rotation_residual_deg
              << " raw_consistency_weight: "
              << endpoint_consistency.weight
              << " raw_consistency_limit: "
              << minimum_raw_consistency
              << " strict_internal_groups_total: "
              << strict_internal_groups
              << " soft_internal_groups_total: "
              << soft_internal_groups
              << " eligible_strict_internal_groups: "
              << eligible_strict_internal_group_ids.size()
              << " independent_internal_groups: "
              << independent_internal_edges.size()
              << " prediction_translation_residual: "
              << predicted_consistency.translation_residual
              << " prediction_translation_limit: "
              << prediction_translation_limit
              << " prediction_rotation_residual_deg: "
              << predicted_consistency.rotation_residual_deg
              << " prediction_yaw_limit_deg: "
              << prediction_yaw_limit_deg
              << " uncovered_tail_path: " << uncovered_tail_path
              << " support_span: " << support_span
              << " evidence_deformation: " << evidence_deformation
              << " evidence_deformation_limit: "
              << evidence_deformation_limit
              << " target_route_fraction: " << target_route_fraction
              << " post_anchor_distance: "
              << hypothesis.post_anchor_distance
              << " corridor_partial: "
              << hypothesis.edge.corridor_partial
              << " corridor_sequence_matches: "
              << hypothesis.corridor_sequence_matches
              << " corridor_sequence_span: "
              << hypothesis.corridor_sequence_span
              << " corridor_sequence_rmse: "
              << hypothesis.corridor_sequence_rmse
              << " corridor_tangent_mismatch_deg: "
              << hypothesis.corridor_tangent_mismatch_deg
              << " corridor_guarded_evidence_valid: "
              << corridor_guarded_evidence_valid
              << " guarded_evidence_valid: "
              << guarded_endpoint_evidence_valid
              << " guarded_endpoint_proposal: "
              << guarded_endpoint_proposal
              << " gate_mode: "
              << (hypothesis.edge.corridor_partial
                      ? "corridor_sequence_partial"
                      : (hypothesis.two_vote_strict_provisional
                      ? "two_vote_internal_prediction_required"
                      : (has_global_internal_support
                             ? "internal_prediction"
                             : "local_evidence_scale")))
              << " accepted: " << endpoint_gate_valid << RESET;
    if (!endpoint_gate_valid) {
      continue;
    }

    if (candidate_idx < 0 ||
        (g_loop_prefer_earliest_candidate &&
         hypothesis.index < candidate_idx) ||
        (!g_loop_prefer_earliest_candidate &&
         hypothesis.selection_cost < best_selection_cost)) {
      candidate_idx = hypothesis.index;
      best_horizontal_distance = hypothesis.horizontal_distance;
      best_score = hypothesis.score;
      best_selection_cost = hypothesis.selection_cost;
      best_consistency_weight = endpoint_consistency.weight;
      best_post_anchor_distance = hypothesis.post_anchor_distance;
      best_partial_geometry = hypothesis.partial_geometry;
      best_micro_window_verified =
          hypothesis.micro_window_verified;
      best_endpoint_consensus_count = hypothesis.consensus_count;
      best_two_vote_strict_provisional =
          hypothesis.two_vote_strict_provisional;
      best_corridor_sequence_partial =
          hypothesis.edge.corridor_partial;
      best_guarded_endpoint_proposal = guarded_endpoint_proposal;
      guarded_endpoint_prediction_measurement =
          strict_internal_graph_valid
          ? strict_internal_reference_poses[hypothesis.edge.from].inverse() *
                strict_internal_reference_poses[hypothesis.edge.to]
          : SE3();
      guarded_endpoint_full_measurement = hypothesis.edge.measurement;
      guarded_endpoint_alpha = 1.0f;
      correction = hypothesis.full_correction;
      endpoint_edge = hypothesis.edge;
      endpoint_edge->prediction_consistent =
          has_global_internal_support && prediction_consistent;
      endpoint_edge_strong = hypothesis.strong;
    }
  }

  if (endpoint_edge.has_value()) {
    // An internal edge using both endpoint windows is correlated with the
    // selected endpoint medoid. It may stay in the final graph, but never as
    // another full-strength vote for the same local point set.
    std::set<int> endpoint_overlapping_group_ids;
    for (std::size_t edge_index = static_cast<std::size_t>(end_idx);
         edge_index < pose_graph_edges.size();
         ++edge_index) {
      const auto& edge = pose_graph_edges[edge_index];
      const bool overlaps_endpoint_closure =
          std::abs(edge.from - candidate_idx) <=
              independent_index_margin &&
          end_idx - edge.to <= independent_index_margin;
      if (overlaps_endpoint_closure && edge.loop_group_id >= 0) {
        endpoint_overlapping_group_ids.insert(edge.loop_group_id);
      }
    }
    std::vector<std::size_t> endpoint_distributed_anchor_indices;
    std::set<std::pair<int, int>> endpoint_distributed_anchor_pairs;
    bool endpoint_overlapping_group_has_incompatible_edge = false;
    for (std::size_t edge_index = static_cast<std::size_t>(end_idx);
         edge_index < pose_graph_edges.size(); ++edge_index) {
      const auto& edge = pose_graph_edges[edge_index];
      if (endpoint_overlapping_group_ids.count(edge.loop_group_id) == 0U) {
        continue;
      }
      const bool strict_full_anchor =
          !edge.soft_fallback && !edge.proactive_ground_z_only &&
          !edge.proactive_planar_companion &&
          !edge.ground_z_planar_hold && !edge.corridor_partial;
      if (strict_full_anchor) {
        endpoint_distributed_anchor_indices.push_back(edge_index);
        endpoint_distributed_anchor_pairs.emplace(edge.from, edge.to);
      } else {
        endpoint_overlapping_group_has_incompatible_edge = true;
      }
    }
    endpoint_distributed_anchor_group_active =
        endpoint_distributed_anchor_pairs.size() >= 2U;
    float distributed_anchor_weight_scale = 0.25f;
    if (endpoint_distributed_anchor_group_active) {
      float distributed_anchor_weight_square_sum = 0.0f;
      for (const std::size_t edge_index :
           endpoint_distributed_anchor_indices) {
        const float weight = pose_graph_edges[edge_index].weight;
        distributed_anchor_weight_square_sum += weight * weight;
      }
      const float distributed_anchor_weight_norm = std::sqrt(
          std::max(distributed_anchor_weight_square_sum, 1.0e-12f));
      // Preserve exactly the original endpoint information budget while
      // moving 36% of its squared information to spatially distributed local
      // anchors: endpoint share=0.8, local share=0.6, and
      // sqrt(0.8^2+0.6^2)=1. The map gains segment coverage without receiving
      // another independent vote from the same point clouds.
      const float original_endpoint_weight = endpoint_edge->weight;
      endpoint_edge->weight = 0.80f * original_endpoint_weight;
      distributed_anchor_weight_scale =
          0.60f * original_endpoint_weight /
          distributed_anchor_weight_norm;
      LOG(INFO) << GREEN
                << " ---> 强终点回环启用沿路分布式局部锚。"
                << " endpoint_candidate_idx: " << candidate_idx
                << " anchors: "
                << endpoint_distributed_anchor_indices.size()
                << " correlated_non_strict_edges_present: "
                << endpoint_overlapping_group_has_incompatible_edge
                << " endpoint_weight: " << original_endpoint_weight
                << " -> " << endpoint_edge->weight
                << " local_weight_norm: "
                << distributed_anchor_weight_norm
                << " local_weight_scale: "
                << distributed_anchor_weight_scale
                << " combined_information_budget_unchanged: true"
                << RESET;
    }
    for (std::size_t edge_index = static_cast<std::size_t>(end_idx);
         edge_index < pose_graph_edges.size(); ++edge_index) {
      auto& edge = pose_graph_edges[edge_index];
      if (endpoint_overlapping_group_ids.count(edge.loop_group_id) == 0U) {
        continue;
      }
      const bool distributed_anchor = std::find(
          endpoint_distributed_anchor_indices.begin(),
          endpoint_distributed_anchor_indices.end(), edge_index) !=
          endpoint_distributed_anchor_indices.end();
      if (endpoint_distributed_anchor_group_active && distributed_anchor) {
        edge.weight *= distributed_anchor_weight_scale;
      } else {
        edge.weight *= 0.25f;
      }
      LOG(INFO) << GREEN
                << " ---> 内部回环证据组与已选终点窗口重叠，"
                << (endpoint_distributed_anchor_group_active &&
                            distributed_anchor
                        ? "作为分布式终点锚归一。"
                        : "按相关观测降权。")
                << " earlier_idx: " << edge.from
                << " later_idx: " << edge.to
                << " endpoint_candidate_idx: " << candidate_idx
                << " distributed_anchor: " << distributed_anchor
                << " edge_weight: " << edge.weight << RESET;
    }
    pose_graph_edges.push_back(*endpoint_edge);
    LOG(INFO) << GREEN << " ---> 找到终点回环候选。candidate_idx: "
              << candidate_idx
              << " end_idx: " << end_idx
              << " horizontal_pose_distance: " << best_horizontal_distance
              << " correction_translation: "
              << correction.block<3, 1>(0, 3).transpose()
              << " icp_score: " << best_score
              << " consistency_weight: " << best_consistency_weight
              << " selection_cost: " << best_selection_cost
              << " accepted_geometry: "
              << (best_partial_geometry ? "partial_micro" : "strict")
              << " micro_window_verified: "
              << best_micro_window_verified
              << " two_vote_strict_provisional: "
              << best_two_vote_strict_provisional
              << " corridor_sequence_partial: "
              << best_corridor_sequence_partial
              << " corridor_axis_xy: "
              << endpoint_edge->corridor_axis_x << " "
              << endpoint_edge->corridor_axis_y
              << " guarded_endpoint_proposal: "
              << best_guarded_endpoint_proposal
              << " guarded_endpoint_alpha: "
              << guarded_endpoint_alpha
              << " distributed_anchor_group_active: "
              << endpoint_distributed_anchor_group_active
              << " post_anchor_distance: "
              << best_post_anchor_distance
              << " post_anchor_distance_is_gate: false"
              << " strong: " << endpoint_edge_strong << RESET;
  } else {
    LOG(INFO) << GREEN << " ---> 终点回环未通过多锚点/证据尺度门控，"
                 << "继续使用独立中途回环。"
                 << " candidates_tested: " << selected_candidates.size()
                 << " consensus_clusters: "
                 << distinct_endpoint_hypotheses.size() << RESET;
  }

  if (candidate_idx < 0 && accepted_internal_loops == 0) {
    LOG(INFO) << GREEN
              << " ---> 没有通过完整验证的终点或中途回环；"
              << "保留原始 map.pcd，不生成 map_loop.pcd。" << RESET;
    return;
  }

  if (finalize_timed_out()) {
    return;
  }

  float initial_graph_cost =
      pose_graph_robust_cost(correction_poses, pose_graph_edges);
  if (!optimize_pose_graph(
          correction_poses, pose_graph_edges)) {
    LOG(ERROR) << RED
               << " ---> 多回环位姿图优化失败，拒绝生成闭环地图。"
               << RESET;
    return;
  }
  float optimized_graph_cost =
      pose_graph_robust_cost(correction_poses, pose_graph_edges);

  // A smooth deformation can still be globally wrong. Bound total XYZ
  // translation by a fraction of travelled route, never by a fixed Z or
  // fixed-metre ceiling. Corrections are gravity-projected before entering
  // this graph, so its rotation is yaw and uses the same path-aware cap as the
  // loop observations.
  const float max_total_translation =
      0.05f * raw_route_length;
  const float max_total_yaw_deg =
      adaptive_loop_yaw_limit_deg(raw_route_length);
  // A correction can stay below the adjacent-frame gate while accumulating
  // into a large bend over dozens of keyframes. Measure non-rigid deformation
  // in metres per metre travelled. First remove the window's midpoint rigid
  // correction: a valid global yaw rotates the displacement vector but does
  // not stretch the local map and must not be misclassified as strain.
  const float local_strain_window_length = std::max(
      20.0f, 3.0f * g_loop_search_radius);
  const float max_local_translation_strain =
      g_loop_max_local_translation_strain;
  const float max_local_translation_delta =
      g_loop_max_local_translation_delta;
  // Pure-Z evidence must correct slow accumulated drift, never sculpt a new
  // ramp.  Limit correction-field Z change to one percent over the same local
  // path window used by the general strain gate.
  const float max_local_vertical_delta =
      0.01f * local_strain_window_length;

  struct GraphSafetyMetrics
  {
    float max_correction_translation = 0.0f;
    float max_correction_rotation_deg = 0.0f;
    float max_adjacent_translation = 0.0f;
    float max_adjacent_rotation_deg = 0.0f;
    V3 max_adjacent_translation_vector = V3::Zero();
    int max_adjacent_translation_index = -1;
    float max_local_translation_strain = 0.0f;
    float max_local_translation_delta = 0.0f;
    float max_local_vertical_delta = 0.0f;
    int max_local_vertical_from = -1;
    int max_local_vertical_to = -1;
    float max_local_window_path_length = 0.0f;
    int max_local_window_from = -1;
    int max_local_window_to = -1;
  };
  const auto measure_graph_safety =
      [&](const PoseVector& poses) {
        GraphSafetyMetrics metrics;
        std::vector<V3> corrected_positions(
            poses.size(), V3::Zero());
        for (std::size_t i = 0; i < poses.size(); ++i) {
          corrected_positions[i] =
              (poses[i] * raw_poses[i]).t_;
          const float anchor_displacement =
              (corrected_positions[i] - raw_poses[i].t_).norm();
          metrics.max_correction_translation =
              std::max(
                  metrics.max_correction_translation,
                  anchor_displacement);
          metrics.max_correction_rotation_deg =
              std::max(
                  metrics.max_correction_rotation_deg,
                  poses[i].so3().log_vee().norm() *
                      180.0f / static_cast<float>(M_PI));
          if (i == 0) {
            continue;
          }
          const SE3 adjacent = poses[i - 1].inverse() * poses[i];
          const float adjacent_translation = adjacent.t_.norm();
          if (adjacent_translation >
              metrics.max_adjacent_translation) {
            metrics.max_adjacent_translation = adjacent_translation;
            metrics.max_adjacent_translation_vector = adjacent.t_;
            metrics.max_adjacent_translation_index =
                static_cast<int>(i);
          }
          metrics.max_adjacent_rotation_deg =
              std::max(
                  metrics.max_adjacent_rotation_deg,
                  adjacent.so3().log_vee().norm() *
                      180.0f / static_cast<float>(M_PI));
        }
        for (std::size_t from = 0;
             from + 1 < poses.size();
             ++from) {
          const float target_length =
              cumulative_route_length[from] +
              local_strain_window_length;
          const auto begin =
              cumulative_route_length.begin() +
              static_cast<std::ptrdiff_t>(from + 1);
          const auto to_iterator = std::lower_bound(
              begin, cumulative_route_length.end(), target_length);
          if (to_iterator == cumulative_route_length.end()) {
            break;
          }
          const std::size_t to = static_cast<std::size_t>(
              std::distance(
                  cumulative_route_length.begin(), to_iterator));
          const float window_path_length =
              cumulative_route_length[to] -
              cumulative_route_length[from];
          if (window_path_length <= 1.0e-3f) {
            continue;
          }
          const float midpoint_path_length =
              0.5f * (cumulative_route_length[from] +
                      cumulative_route_length[to]);
          const auto midpoint_iterator = std::lower_bound(
              cumulative_route_length.begin() +
                  static_cast<std::ptrdiff_t>(from),
              cumulative_route_length.begin() +
                  static_cast<std::ptrdiff_t>(to + 1),
              midpoint_path_length);
          const std::size_t midpoint = static_cast<std::size_t>(
              std::distance(
                  cumulative_route_length.begin(), midpoint_iterator));
          const V3 raw_delta =
              raw_poses[to].t_ - raw_poses[from].t_;
          const V3 expected_rigid_delta =
              poses[midpoint].R_ * raw_delta;
          const V3 corrected_delta =
              corrected_positions[to] - corrected_positions[from];
          const float correction_delta =
              (corrected_delta - expected_rigid_delta).norm();
          const float strain =
              correction_delta / window_path_length;
          const float vertical_delta = std::abs(
              corrected_delta.z() - expected_rigid_delta.z());
          if (vertical_delta > metrics.max_local_vertical_delta) {
            metrics.max_local_vertical_delta = vertical_delta;
            metrics.max_local_vertical_from = static_cast<int>(from);
            metrics.max_local_vertical_to = static_cast<int>(to);
          }
          if (strain > metrics.max_local_translation_strain) {
            metrics.max_local_translation_strain = strain;
            metrics.max_local_translation_delta = correction_delta;
            metrics.max_local_window_path_length = window_path_length;
            metrics.max_local_window_from = static_cast<int>(from);
            metrics.max_local_window_to = static_cast<int>(to);
          }
        }
        return metrics;
      };
  const auto safety_ratio = [&](const GraphSafetyMetrics& metrics) {
    return std::max(
        {metrics.max_correction_translation /
             std::max(max_total_translation, 1.0e-6f),
         metrics.max_correction_rotation_deg /
             std::max(max_total_yaw_deg, 1.0e-6f),
         metrics.max_adjacent_translation / 0.15f,
         metrics.max_adjacent_rotation_deg / 2.0f,
         metrics.max_local_translation_strain /
             std::max(max_local_translation_strain, 1.0e-6f),
         metrics.max_local_translation_delta /
             std::max(max_local_translation_delta, 1.0e-6f),
         metrics.max_local_vertical_delta /
             std::max(max_local_vertical_delta, 1.0e-6f)});
  };
  struct AdjacentCloudEvidence
  {
    bool valid = false;
    bool supports_correction = false;
    std::size_t raw_pairs = 0;
    std::size_t corrected_pairs = 0;
    float raw_overlap = 0.0f;
    float corrected_overlap = 0.0f;
    float raw_score = std::numeric_limits<float>::max();
    float corrected_score = std::numeric_limits<float>::max();
  };
  const auto evaluate_adjacent_cloud_evidence = [&] (
      const PoseVector& poses,
      const GraphSafetyMetrics& metrics) {
    AdjacentCloudEvidence evidence;
    const int current_index = metrics.max_adjacent_translation_index;
    if (metrics.max_adjacent_translation <= 0.15f ||
        current_index <= 0 ||
        current_index >= static_cast<int>(loop_keyframes_.size())) {
      return evidence;
    }
    CloudPtr previous_body = loadLoopKeyFrameCloud(
        static_cast<std::size_t>(current_index - 1));
    CloudPtr current_body = loadLoopKeyFrameCloud(
        static_cast<std::size_t>(current_index));
    if (!previous_body || previous_body->empty() ||
        !current_body || current_body->empty()) {
      return evidence;
    }

    auto align_pair = [&] (
        const SE3& previous_pose,
        const SE3& current_pose,
        std::size_t& pair_count,
        float& overlap,
        float& score) {
      CloudPtr previous_world(new PointCloudType());
      CloudPtr current_world(new PointCloudType());
      pcl::transformPointCloud(
          *previous_body, *previous_world,
          se3_to_matrix4f(previous_pose));
      pcl::transformPointCloud(
          *current_body, *current_world,
          se3_to_matrix4f(current_pose));
      pcl::search::KdTree<PointType> tree;
      tree.setInputCloud(previous_world);
      std::vector<int> nearest_index(1);
      std::vector<float> nearest_squared_distance(1);
      const float correspondence_distance =
          std::min(0.5f, g_loop_icp_max_distance);
      const float max_squared_distance =
          correspondence_distance * correspondence_distance;
      double squared_distance_sum = 0.0;
      pair_count = 0;
      for (const auto& point : current_world->points) {
        if (tree.nearestKSearch(
                point, 1, nearest_index,
                nearest_squared_distance) > 0 &&
            nearest_squared_distance[0] <= max_squared_distance) {
          ++pair_count;
          squared_distance_sum += nearest_squared_distance[0];
        }
      }
      overlap = current_world->empty()
          ? 0.0f
          : static_cast<float>(pair_count) /
              static_cast<float>(current_world->size());
      score = pair_count == 0
          ? std::numeric_limits<float>::max()
          : static_cast<float>(
              squared_distance_sum / static_cast<double>(pair_count));
    };

    align_pair(
        raw_poses[current_index - 1],
        raw_poses[current_index],
        evidence.raw_pairs,
        evidence.raw_overlap,
        evidence.raw_score);
    align_pair(
        poses[current_index - 1] * raw_poses[current_index - 1],
        poses[current_index] * raw_poses[current_index],
        evidence.corrected_pairs,
        evidence.corrected_overlap,
        evidence.corrected_score);
    evidence.valid =
        evidence.raw_pairs >= 100 &&
        evidence.corrected_pairs >= 100 &&
        std::isfinite(evidence.raw_score) &&
        std::isfinite(evidence.corrected_score);
    if (evidence.valid) {
      evidence.supports_correction =
          evidence.corrected_overlap + 0.02f >= evidence.raw_overlap &&
          evidence.corrected_score <=
              1.05f * evidence.raw_score + 1.0e-4f;
    }
    return evidence;
  };
  const auto graph_is_safe = [&] (
      const GraphSafetyMetrics& metrics,
      const float initial_cost,
      const float optimized_cost,
      const bool adjacent_correction_supported) {
    return std::isfinite(optimized_cost) &&
        optimized_cost < initial_cost &&
        metrics.max_correction_translation <= max_total_translation &&
        metrics.max_correction_rotation_deg <= max_total_yaw_deg &&
        (metrics.max_adjacent_translation <= 0.15f ||
         adjacent_correction_supported) &&
        metrics.max_adjacent_rotation_deg <= 2.0f &&
        metrics.max_local_translation_strain <=
            max_local_translation_strain &&
        metrics.max_local_translation_delta <=
            max_local_translation_delta &&
        metrics.max_local_vertical_delta <=
            max_local_vertical_delta;
  };

  GraphSafetyMetrics graph_metrics =
      measure_graph_safety(correction_poses);
  AdjacentCloudEvidence adjacent_cloud_evidence =
      evaluate_adjacent_cloud_evidence(
          correction_poses, graph_metrics);
  auto log_adjacent_cloud_evidence = [&] () {
    if (graph_metrics.max_adjacent_translation <= 0.15f) {
      return;
    }
    LOG(INFO) << GREEN
              << " ---> 大相邻校正局部点云复核。index: "
              << graph_metrics.max_adjacent_translation_index
              << " correction_xyz: "
              << graph_metrics.max_adjacent_translation_vector.transpose()
              << " raw_pairs: " << adjacent_cloud_evidence.raw_pairs
              << " corrected_pairs: "
              << adjacent_cloud_evidence.corrected_pairs
              << " raw_overlap: " << adjacent_cloud_evidence.raw_overlap
              << " corrected_overlap: "
              << adjacent_cloud_evidence.corrected_overlap
              << " raw_score: " << adjacent_cloud_evidence.raw_score
              << " corrected_score: "
              << adjacent_cloud_evidence.corrected_score
              << " evidence_valid: " << adjacent_cloud_evidence.valid
              << " supports_correction: "
              << adjacent_cloud_evidence.supports_correction << RESET;
  };
  log_adjacent_cloud_evidence();

  // More spatially distributed candidates can expose mutually inconsistent
  // loop edges. Do not relax the local-warp gate. Instead, use leave-one-out
  // graph tests to adjust the whole physical loop group whose change improves
  // the worst safety ratio, then re-optimize. Soft fallbacks are downweighted
  // in two stages before deletion. The final keyframe has no
  // special semantic relationship with the first one, so an endpoint edge
  // must survive the same graph-level evidence as every internal loop.
  const std::size_t first_loop_edge_index =
      static_cast<std::size_t>(end_idx);
  bool endpoint_edge_active = endpoint_edge.has_value();
  int pruned_internal_loops = 0;
  int pruned_internal_groups = 0;
  int pruned_proactive_ground_z_loops = 0;
  int pruned_proactive_ground_z_groups = 0;
  int pruned_endpoint_loops = 0;
  int downgraded_internal_z_loops = 0;
  int downgraded_endpoint_z_loops = 0;
  int downweighted_soft_fallback_loops = 0;
  int proactive_ground_z_line_search_steps = 0;
  int guarded_endpoint_line_search_steps = 0;
  int graph_loop_modifications = 0;
  const auto guarded_measurement_at_alpha = [&] (const float alpha) {
    const V6 full_increment =
        (guarded_endpoint_prediction_measurement.inverse() *
         guarded_endpoint_full_measurement).log_vee();
    const V6 scaled_increment = alpha * full_increment;
    return guarded_endpoint_prediction_measurement *
        SE3(scaled_increment);
  };
  while (!graph_is_safe(
             graph_metrics,
             initial_graph_cost,
             optimized_graph_cost,
             adjacent_cloud_evidence.supports_correction) &&
         graph_loop_modifications < 16 &&
         pose_graph_edges.size() > first_loop_edge_index &&
         !finalize_timed_out()) {
    const float previous_ratio = safety_ratio(graph_metrics);
    std::size_t best_removed_index = pose_graph_edges.size();
    int best_removed_group_id = -1;
    int best_group_edge_count = 1;
    bool best_downgrade_to_z = false;
    bool best_downweight_soft = false;
    float best_ratio = previous_ratio;
    float best_selection_score = previous_ratio + 0.05f;
    float best_initial_cost = initial_graph_cost;
    float best_optimized_cost = optimized_graph_cost;
    GraphSafetyMetrics best_metrics = graph_metrics;
    PoseVector best_poses;
    PoseGraphEdgeVector best_edges;

    std::set<int> tested_internal_groups;
    for (std::size_t edge_index = first_loop_edge_index;
         edge_index < pose_graph_edges.size();
         ++edge_index) {
      const auto& candidate_edge = pose_graph_edges[edge_index];
      const int candidate_group_id = candidate_edge.loop_group_id;
      if (!candidate_edge.endpoint && candidate_group_id >= 0 &&
          !tested_internal_groups.insert(candidate_group_id).second) {
        continue;
      }
      const auto belongs_to_candidate_group = [&] (
          const PoseGraphEdge& edge,
          const std::size_t index) {
        if (candidate_edge.endpoint || candidate_group_id < 0) {
          return index == edge_index;
        }
        return !edge.endpoint &&
            edge.loop_group_id == candidate_group_id;
      };
      int candidate_group_edge_count = 0;
      bool can_downgrade_to_z = true;
      bool can_downweight_soft = true;
      for (std::size_t index = first_loop_edge_index;
           index < pose_graph_edges.size();
           ++index) {
        if (!belongs_to_candidate_group(
                pose_graph_edges[index], index)) {
          continue;
        }
        ++candidate_group_edge_count;
        can_downgrade_to_z = can_downgrade_to_z &&
            pose_graph_edges[index].ground_z_valid &&
            pose_graph_edges[index].prediction_consistent &&
            !pose_graph_edges[index].ground_z_planar_hold &&
            !pose_graph_edges[index].proactive_ground_z_only &&
            !pose_graph_edges[index].corridor_partial;
        can_downweight_soft = can_downweight_soft &&
            pose_graph_edges[index].soft_fallback &&
            pose_graph_edges[index].safety_downweight_level < 2;
      }
      if (candidate_group_edge_count <= 0) {
        continue;
      }
      const int action_count =
          1 + (can_downweight_soft ? 1 : 0) +
          (can_downgrade_to_z ? 1 : 0);
      for (int action = 0; action < action_count; ++action) {
        int action_cursor = 0;
        const bool downweight_soft =
            can_downweight_soft && action == action_cursor++;
        const bool downgrade_to_z =
            can_downgrade_to_z && action == action_cursor++;
        PoseGraphEdgeVector trial_edges = pose_graph_edges;
        if (downweight_soft) {
          for (std::size_t index = first_loop_edge_index;
               index < trial_edges.size();
               ++index) {
            if (belongs_to_candidate_group(trial_edges[index], index)) {
              trial_edges[index].weight *= 0.5f;
              ++trial_edges[index].safety_downweight_level;
            }
          }
        } else if (downgrade_to_z) {
          for (std::size_t index = first_loop_edge_index;
               index < trial_edges.size();
               ++index) {
            if (belongs_to_candidate_group(trial_edges[index], index)) {
              trial_edges[index].ground_z_planar_hold = true;
            }
          }
        } else {
          for (std::size_t index = trial_edges.size();
               index-- > first_loop_edge_index;) {
            if (belongs_to_candidate_group(trial_edges[index], index)) {
              trial_edges.erase(trial_edges.begin() +
                  static_cast<std::ptrdiff_t>(index));
            }
          }
        }
        PoseVector trial_poses(loop_keyframes_.size(), SE3());
        const float trial_initial_cost =
            pose_graph_robust_cost(trial_poses, trial_edges);
        if (!optimize_pose_graph(trial_poses, trial_edges)) {
          continue;
        }
        const float trial_optimized_cost =
            pose_graph_robust_cost(trial_poses, trial_edges);
        const GraphSafetyMetrics trial_metrics =
            measure_graph_safety(trial_poses);
        const float trial_ratio = safety_ratio(trial_metrics);
        // Prefer retaining independently validated ground Z when doing so is
        // nearly as safe as deleting the complete loop. A materially safer
        // full deletion still wins.
        const float action_penalty = downweight_soft
            ? 0.0f : (downgrade_to_z ? 0.02f : 0.05f);
        const float selection_score =
            trial_ratio + action_penalty;
        if (!std::isfinite(trial_optimized_cost) ||
            trial_optimized_cost >= trial_initial_cost ||
            trial_ratio >= previous_ratio - 1.0e-4f ||
            selection_score >= best_selection_score - 1.0e-4f) {
          continue;
        }
        best_removed_index = edge_index;
        best_removed_group_id = candidate_group_id;
        best_group_edge_count = candidate_group_edge_count;
        best_downgrade_to_z = downgrade_to_z;
        best_downweight_soft = downweight_soft;
        best_ratio = trial_ratio;
        best_selection_score = selection_score;
        best_initial_cost = trial_initial_cost;
        best_optimized_cost = trial_optimized_cost;
        best_metrics = trial_metrics;
        best_poses = std::move(trial_poses);
        best_edges = std::move(trial_edges);
      }
    }

    if (best_removed_index >= pose_graph_edges.size()) {
      break;
    }
    const PoseGraphEdge removed_edge =
        pose_graph_edges[best_removed_index];
    // If deletion wins the leave-one-out comparison for a soft group, do not
    // jump straight from its original information to zero.  Commit one 0.5
    // information step and re-evaluate the complete graph; the same group
    // must pass through levels 1 and 2 before it becomes removable.  This
    // preserves useful weak closures without granting them any special graph
    // safety exemption.
    if (!best_downweight_soft && !best_downgrade_to_z &&
        !removed_edge.endpoint && removed_edge.soft_fallback &&
        removed_edge.safety_downweight_level < 2) {
      PoseGraphEdgeVector staged_edges = pose_graph_edges;
      int staged_group_edge_count = 0;
      for (std::size_t index = first_loop_edge_index;
           index < staged_edges.size(); ++index) {
        const bool belongs_to_group = best_removed_group_id >= 0
            ? (!staged_edges[index].endpoint &&
               staged_edges[index].loop_group_id ==
                   best_removed_group_id)
            : index == best_removed_index;
        if (!belongs_to_group) {
          continue;
        }
        staged_edges[index].weight *= 0.5f;
        ++staged_edges[index].safety_downweight_level;
        ++staged_group_edge_count;
      }
      PoseVector staged_poses(loop_keyframes_.size(), SE3());
      const float staged_initial_cost =
          pose_graph_robust_cost(staged_poses, staged_edges);
      if (staged_group_edge_count > 0 &&
          optimize_pose_graph(staged_poses, staged_edges)) {
        const float staged_optimized_cost =
            pose_graph_robust_cost(staged_poses, staged_edges);
        if (std::isfinite(staged_optimized_cost) &&
            staged_optimized_cost < staged_initial_cost) {
          const GraphSafetyMetrics staged_metrics =
              measure_graph_safety(staged_poses);
          LOG(WARNING) << YELLOW
                       << " ---> soft 回环删除前强制分级降权。group_id: "
                       << best_removed_group_id
                       << " group_edges: " << staged_group_edge_count
                       << " previous_level: "
                       << removed_edge.safety_downweight_level
                       << " new_level: "
                       << removed_edge.safety_downweight_level + 1
                       << " previous_weight: " << removed_edge.weight
                       << " new_weight: "
                       << 0.5f * removed_edge.weight
                       << " safety_ratio: " << previous_ratio
                       << " -> " << safety_ratio(staged_metrics)
                       << " graph_safe: "
                       << graph_is_safe(
                              staged_metrics, staged_initial_cost,
                              staged_optimized_cost, false)
                       << RESET;
          pose_graph_edges = std::move(staged_edges);
          correction_poses = std::move(staged_poses);
          initial_graph_cost = staged_initial_cost;
          optimized_graph_cost = staged_optimized_cost;
          graph_metrics = staged_metrics;
          adjacent_cloud_evidence = evaluate_adjacent_cloud_evidence(
              correction_poses, graph_metrics);
          downweighted_soft_fallback_loops +=
              staged_group_edge_count;
          ++graph_loop_modifications;
          log_adjacent_cloud_evidence();
          continue;
        }
      }
    }
    // A pure-Z observation can be geometrically valid yet ask the correction
    // field to change too quickly over a short local graph segment.  Before
    // deleting that evidence, line-search only its measurement amplitude and
    // keep the largest non-zero value that satisfies the unchanged global,
    // adjacent, strain and vertical-delta safety gates.
    if (!best_downweight_soft && !best_downgrade_to_z &&
        !removed_edge.endpoint &&
        removed_edge.proactive_ground_z_only) {
      constexpr std::array<float, 8> kGroundZOnlyAlphas = {
          0.75f, 0.70f, 0.65f, 0.60f,
          0.55f, 0.50f, 0.40f, 0.25f};
      bool retained_ground_z = false;
      for (const float trial_alpha : kGroundZOnlyAlphas) {
        PoseGraphEdgeVector trial_edges = pose_graph_edges;
        int trial_group_edge_count = 0;
        for (std::size_t index = first_loop_edge_index;
             index < trial_edges.size(); ++index) {
          const bool belongs_to_group = best_removed_group_id >= 0
              ? (!trial_edges[index].endpoint &&
                 trial_edges[index].loop_group_id ==
                     best_removed_group_id)
              : index == best_removed_index;
          if (!belongs_to_group ||
              !trial_edges[index].proactive_ground_z_only) {
            continue;
          }
          const V6 scaled_measurement = trial_alpha *
              trial_edges[index].measurement.log_vee();
          trial_edges[index].measurement = SE3(scaled_measurement);
          ++trial_group_edge_count;
        }
        if (trial_group_edge_count <= 0) {
          continue;
        }
        PoseVector trial_poses(loop_keyframes_.size(), SE3());
        const float trial_initial_cost =
            pose_graph_robust_cost(trial_poses, trial_edges);
        if (!optimize_pose_graph(trial_poses, trial_edges)) {
          continue;
        }
        const float trial_optimized_cost =
            pose_graph_robust_cost(trial_poses, trial_edges);
        const GraphSafetyMetrics trial_metrics =
            measure_graph_safety(trial_poses);
        const AdjacentCloudEvidence trial_adjacent_evidence =
            evaluate_adjacent_cloud_evidence(
                trial_poses, trial_metrics);
        const bool trial_safe = graph_is_safe(
            trial_metrics, trial_initial_cost, trial_optimized_cost,
            trial_adjacent_evidence.supports_correction);
        LOG(INFO) << (trial_safe ? GREEN : YELLOW)
                  << " ---> 独立纯Z测量幅度复核。group_id: "
                  << best_removed_group_id
                  << " group_edges: " << trial_group_edge_count
                  << " trial_alpha: " << trial_alpha
                  << " graph_cost: " << trial_initial_cost
                  << " -> " << trial_optimized_cost
                  << " safety_ratio: "
                  << safety_ratio(trial_metrics)
                  << " max_local_delta: "
                  << trial_metrics.max_local_translation_delta
                  << " max_local_vertical_delta: "
                  << trial_metrics.max_local_vertical_delta
                  << " safe: " << trial_safe << RESET;
        if (!trial_safe) {
          continue;
        }
        pose_graph_edges = std::move(trial_edges);
        correction_poses = std::move(trial_poses);
        initial_graph_cost = trial_initial_cost;
        optimized_graph_cost = trial_optimized_cost;
        graph_metrics = trial_metrics;
        adjacent_cloud_evidence = trial_adjacent_evidence;
        ++proactive_ground_z_line_search_steps;
        ++graph_loop_modifications;
        retained_ground_z = true;
        LOG(WARNING) << YELLOW
                     << " ---> 独立纯Z以最大安全非零幅度保留。group_id: "
                     << best_removed_group_id
                     << " alpha: " << trial_alpha
                     << " endpoint_support_eligible: false" << RESET;
        break;
      }
      if (retained_ground_z) {
        log_adjacent_cloud_evidence();
        continue;
      }
    }
    // A guarded endpoint is not protected from graph safety. However, before
    // deleting it outright, reduce the measurement itself along the SE(3)
    // geodesic from the internal-only prediction P to the full registration G.
    // Weight-only reduction can still leave a long-loop constraint dominant.
    if (!best_downweight_soft && !best_downgrade_to_z &&
        removed_edge.endpoint && best_guarded_endpoint_proposal) {
      constexpr std::array<float, 2> kGuardedEndpointAlphas = {
          0.50f, 0.25f};
      bool retained_guarded_endpoint = false;
      for (const float trial_alpha : kGuardedEndpointAlphas) {
        if (trial_alpha >= guarded_endpoint_alpha - 1.0e-6f) {
          continue;
        }
        PoseGraphEdgeVector trial_edges = pose_graph_edges;
        trial_edges[best_removed_index].measurement =
            guarded_measurement_at_alpha(trial_alpha);
        PoseVector trial_poses(loop_keyframes_.size(), SE3());
        const float trial_initial_cost =
            pose_graph_robust_cost(trial_poses, trial_edges);
        if (!optimize_pose_graph(trial_poses, trial_edges)) {
          continue;
        }
        const float trial_optimized_cost =
            pose_graph_robust_cost(trial_poses, trial_edges);
        const GraphSafetyMetrics trial_metrics =
            measure_graph_safety(trial_poses);
        const AdjacentCloudEvidence trial_adjacent_evidence =
            evaluate_adjacent_cloud_evidence(trial_poses, trial_metrics);
        const bool trial_safe = graph_is_safe(
            trial_metrics, trial_initial_cost, trial_optimized_cost,
            trial_adjacent_evidence.supports_correction);
        LOG(INFO) << GREEN
                  << " ---> guarded endpoint 测量幅度复核。candidate_idx: "
                  << candidate_idx
                  << " previous_alpha: " << guarded_endpoint_alpha
                  << " trial_alpha: " << trial_alpha
                  << " graph_cost: " << trial_initial_cost
                  << " -> " << trial_optimized_cost
                  << " safety_ratio: " << safety_ratio(trial_metrics)
                  << " max_local_strain: "
                  << trial_metrics.max_local_translation_strain
                  << " max_local_delta: "
                  << trial_metrics.max_local_translation_delta
                  << " max_local_vertical_delta: "
                  << trial_metrics.max_local_vertical_delta
                  << " safe: " << trial_safe << RESET;
        if (!trial_safe) {
          continue;
        }
        pose_graph_edges = std::move(trial_edges);
        correction_poses = std::move(trial_poses);
        initial_graph_cost = trial_initial_cost;
        optimized_graph_cost = trial_optimized_cost;
        graph_metrics = trial_metrics;
        adjacent_cloud_evidence = trial_adjacent_evidence;
        guarded_endpoint_alpha = trial_alpha;
        ++guarded_endpoint_line_search_steps;
        ++graph_loop_modifications;
        retained_guarded_endpoint = true;
        LOG(WARNING) << YELLOW
                     << " ---> guarded endpoint 以最大安全非零幅度保留。"
                     << " candidate_idx: " << candidate_idx
                     << " alpha: " << guarded_endpoint_alpha
                     << " soft_groups_authorized: false" << RESET;
        break;
      }
      if (retained_guarded_endpoint) {
        log_adjacent_cloud_evidence();
        continue;
      }
    }
    LOG(WARNING) << YELLOW
                 << " ---> 调整冲突回环并重新优化。action: "
                 << (best_downweight_soft
                     ? "downweight_soft_fallback"
                     : (best_downgrade_to_z
                         ? "keep_ground_z_and_planar_hold" : "remove"))
                 << " type: "
                 << (removed_edge.endpoint ? "endpoint" : "internal")
                 << " from: "
                 << removed_edge.from
                 << " to: " << removed_edge.to
                 << " group_id: " << best_removed_group_id
                 << " group_edges: " << best_group_edge_count
                 << " soft_fallback: " << removed_edge.soft_fallback
                 << " proactive_ground_z_only: "
                 << removed_edge.proactive_ground_z_only
                 << " previous_downweight_level: "
                 << removed_edge.safety_downweight_level
                 << " edge_weight: " << removed_edge.weight
                 << " safety_ratio: " << previous_ratio
                 << " -> " << best_ratio << RESET;
    pose_graph_edges = std::move(best_edges);
    correction_poses = std::move(best_poses);
    initial_graph_cost = best_initial_cost;
    optimized_graph_cost = best_optimized_cost;
    graph_metrics = best_metrics;
    adjacent_cloud_evidence = evaluate_adjacent_cloud_evidence(
        correction_poses, graph_metrics);
    log_adjacent_cloud_evidence();
    if (best_downweight_soft) {
      downweighted_soft_fallback_loops += best_group_edge_count;
    } else if (best_downgrade_to_z) {
      if (removed_edge.endpoint) {
        ++downgraded_endpoint_z_loops;
      } else {
        downgraded_internal_z_loops += best_group_edge_count;
      }
    } else {
      if (removed_edge.endpoint) {
        endpoint_edge_active = false;
        ++pruned_endpoint_loops;
      } else {
        pruned_internal_loops += best_group_edge_count;
        accepted_internal_loops = std::max(
            0, accepted_internal_loops - best_group_edge_count);
        if (best_removed_group_id >= 0) {
          ++pruned_internal_groups;
          accepted_internal_groups = std::max(
              0, accepted_internal_groups - 1);
        }
        if (removed_edge.proactive_ground_z_only) {
          pruned_proactive_ground_z_loops += best_group_edge_count;
          proactive_ground_z_loops = std::max(
              0, proactive_ground_z_loops - best_group_edge_count);
          ++pruned_proactive_ground_z_groups;
          proactive_ground_z_groups = std::max(
              0, proactive_ground_z_groups - 1);
        }
      }
    }
    ++graph_loop_modifications;
  }

  // A loop can be geometrically correct and survive graph safety while its
  // soft graph residual is still large enough to leave a visible double wall
  // or double ground surface.  Revisit admission and residual completion are
  // deliberately separate: this stage never creates a new loop.  It may only
  // strengthen an already accepted strict multi-anchor group, or a strong
  // standard (>=3 vote) endpoint.  Every trial is solved from scratch and must
  // pass the unchanged graph-safety gates; otherwise the first-pass graph is
  // retained byte-for-byte.
  int post_residual_refinement_groups = 0;
  int post_residual_refinement_edges = 0;
  int post_residual_unresolved_groups = 0;
  float post_residual_refinement_scale = 1.0f;
  float post_residual_before_translation = 0.0f;
  float post_residual_after_translation = 0.0f;
  float post_residual_before_rotation_deg = 0.0f;
  float post_residual_after_rotation_deg = 0.0f;
  if (g_loop_post_residual_refinement_enable &&
      graph_is_safe(
          graph_metrics, initial_graph_cost, optimized_graph_cost,
          adjacent_cloud_evidence.supports_correction) &&
      !finalize_timed_out()) {
    struct PostResidualGroup
    {
      int group_id = -1;
      bool endpoint = false;
      bool z_only = false;
      std::vector<std::size_t> edge_indices;
    };
    struct PostResidualStats
    {
      float max_translation = 0.0f;
      float max_z = 0.0f;
      float max_rotation_deg = 0.0f;
      float rms_translation = 0.0f;
      float rms_rotation_deg = 0.0f;
    };
    struct InternalPostResidualGroup
    {
      std::vector<std::size_t> full_edges;
      std::vector<std::size_t> z_edges;
      bool incompatible = false;
    };

    const auto edge_post_residual = [&] (
        const PoseGraphEdge& edge,
        const PoseVector& poses) {
      PostResidualStats stats;
      const SE3 predicted =
          poses[edge.from].inverse() * poses[edge.to];
      if (edge.proactive_ground_z_only) {
        const float residual_z = std::abs(
            predicted.log_vee()(5) -
            pose_graph_edge_measurement(edge)(5));
        stats.max_translation = residual_z;
        stats.max_z = residual_z;
        stats.rms_translation = residual_z;
        return stats;
      }
      const V3 raw_anchor = raw_poses[edge.to].t_;
      const V3 anchor_residual =
          (predicted * raw_anchor) - (edge.measurement * raw_anchor);
      stats.max_translation = anchor_residual.norm();
      stats.max_z = std::abs(anchor_residual.z());
      stats.max_rotation_deg =
          pose_graph_residual(
              poses[edge.from], poses[edge.to], edge.measurement)
              .head<3>().norm() * 180.0f / static_cast<float>(M_PI);
      stats.rms_translation = stats.max_translation;
      stats.rms_rotation_deg = stats.max_rotation_deg;
      return stats;
    };
    const auto group_post_residual = [&] (
        const PostResidualGroup& group,
        const PoseGraphEdgeVector& edges,
        const PoseVector& poses) {
      PostResidualStats stats;
      double translation_square_sum = 0.0;
      double rotation_square_sum = 0.0;
      int count = 0;
      for (const std::size_t edge_index : group.edge_indices) {
        if (edge_index >= edges.size()) {
          continue;
        }
        const PostResidualStats edge_stats =
            edge_post_residual(edges[edge_index], poses);
        stats.max_translation = std::max(
            stats.max_translation, edge_stats.max_translation);
        stats.max_z = std::max(stats.max_z, edge_stats.max_z);
        stats.max_rotation_deg = std::max(
            stats.max_rotation_deg, edge_stats.max_rotation_deg);
        translation_square_sum +=
            edge_stats.max_translation * edge_stats.max_translation;
        rotation_square_sum +=
            edge_stats.max_rotation_deg * edge_stats.max_rotation_deg;
        ++count;
      }
      if (count > 0) {
        stats.rms_translation = static_cast<float>(std::sqrt(
            translation_square_sum / static_cast<double>(count)));
        stats.rms_rotation_deg = static_cast<float>(std::sqrt(
            rotation_square_sum / static_cast<double>(count)));
      }
      return stats;
    };

    std::map<int, InternalPostResidualGroup> internal_groups;
    for (std::size_t edge_index = first_loop_edge_index;
         edge_index < pose_graph_edges.size(); ++edge_index) {
      const auto& edge = pose_graph_edges[edge_index];
      if (edge.endpoint || edge.loop_group_id < 0) {
        continue;
      }
      auto& group = internal_groups[edge.loop_group_id];
      if (edge.proactive_ground_z_only) {
        group.z_edges.push_back(edge_index);
      } else if (edge.proactive_planar_companion) {
        // A planar companion is correlated with the pure-Z anchors but is not
        // itself strengthened by the residual-completion pass.
        continue;
      } else if (!edge.soft_fallback &&
                 !edge.ground_z_planar_hold &&
                 !edge.corridor_partial) {
        group.full_edges.push_back(edge_index);
      } else {
        group.incompatible = true;
      }
    }

    std::vector<PostResidualGroup> refinement_groups;
    for (const auto& [group_id, candidate] : internal_groups) {
      if (candidate.incompatible ||
          (!candidate.full_edges.empty() && !candidate.z_edges.empty())) {
        continue;
      }
      const auto& candidate_edges = candidate.z_edges.empty()
          ? candidate.full_edges : candidate.z_edges;
      std::set<std::pair<int, int>> unique_pairs;
      std::set<int> unique_anchor_ids;
      float maximum_later_path_separation = 0.0f;
      float maximum_later_xy_separation = 0.0f;
      for (const std::size_t edge_index : candidate_edges) {
        const auto& edge = pose_graph_edges[edge_index];
        unique_pairs.emplace(edge.from, edge.to);
        if (edge.anchor_id >= 0) {
          unique_anchor_ids.insert(edge.anchor_id);
        }
      }
      for (std::size_t i = 0; i < candidate_edges.size(); ++i) {
        const auto& lhs = pose_graph_edges[candidate_edges[i]];
        for (std::size_t j = i + 1; j < candidate_edges.size(); ++j) {
          const auto& rhs = pose_graph_edges[candidate_edges[j]];
          maximum_later_path_separation = std::max(
              maximum_later_path_separation,
              std::abs(cumulative_route_length[lhs.to] -
                       cumulative_route_length[rhs.to]));
          const V3 delta = raw_poses[lhs.to].t_ - raw_poses[rhs.to].t_;
          maximum_later_xy_separation = std::max(
              maximum_later_xy_separation,
              std::hypot(delta.x(), delta.y()));
        }
      }
      const int independent_anchor_count = static_cast<int>(std::max(
          unique_pairs.size(), unique_anchor_ids.size()));
      if (independent_anchor_count <
              g_loop_post_residual_refinement_min_anchors ||
          maximum_later_path_separation < 3.0f ||
          maximum_later_xy_separation < 2.0f) {
        continue;
      }
      // When a standard endpoint is also present, a correlated internal group
      // must not receive a second increase from the same endpoint windows.
      bool overlaps_selected_endpoint = false;
      if (endpoint_edge_active && candidate_idx >= 0) {
        for (const std::size_t edge_index : candidate_edges) {
          const auto& edge = pose_graph_edges[edge_index];
          if (std::abs(edge.from - candidate_idx) <=
                  independent_index_margin &&
              end_idx - edge.to <= independent_index_margin) {
            overlaps_selected_endpoint = true;
            break;
          }
        }
      }
      if (overlaps_selected_endpoint) {
        continue;
      }
      PostResidualGroup group;
      group.group_id = group_id;
      group.z_only = !candidate.z_edges.empty();
      group.edge_indices = candidate_edges;
      refinement_groups.push_back(std::move(group));
    }

    const bool standard_strong_endpoint =
        endpoint_edge_active && endpoint_edge_strong &&
        best_endpoint_consensus_count >= 3 &&
        !best_partial_geometry &&
        !best_two_vote_strict_provisional &&
        !best_corridor_sequence_partial &&
        !best_guarded_endpoint_proposal &&
        !endpoint_distributed_anchor_group_active &&
        downgraded_endpoint_z_loops == 0;
    if (standard_strong_endpoint) {
      for (std::size_t edge_index = first_loop_edge_index;
           edge_index < pose_graph_edges.size(); ++edge_index) {
        const auto& edge = pose_graph_edges[edge_index];
        if (!edge.endpoint || edge.soft_fallback ||
            edge.ground_z_planar_hold ||
            edge.proactive_ground_z_only || edge.corridor_partial) {
          continue;
        }
        PostResidualGroup group;
        group.group_id = -1;
        group.endpoint = true;
        group.edge_indices.push_back(edge_index);
        refinement_groups.push_back(std::move(group));
        break;
      }
    }

    std::vector<PostResidualGroup> active_refinement_groups;
    std::vector<PostResidualStats> baseline_group_stats;
    for (const auto& group : refinement_groups) {
      const PostResidualStats stats = group_post_residual(
          group, pose_graph_edges, correction_poses);
      LOG(INFO) << GREEN
                << " ---> 已确认回环优化后残差复验。type: "
                << (group.endpoint
                        ? "endpoint" : (group.z_only ? "pure_z" : "full"))
                << " group_id: " << group.group_id
                << " anchors: " << group.edge_indices.size()
                << " max_translation: " << stats.max_translation
                << " max_z: " << stats.max_z
                << " max_rotation_deg: " << stats.max_rotation_deg
                << " rms_translation: " << stats.rms_translation
                << " rms_rotation_deg: " << stats.rms_rotation_deg
                << " target_translation: "
                << g_loop_post_residual_refinement_target_translation
                << " target_rotation_deg: "
                << g_loop_post_residual_refinement_target_rotation_deg
                << RESET;
      if (stats.max_translation >
              g_loop_post_residual_refinement_target_translation ||
          stats.max_rotation_deg >
              g_loop_post_residual_refinement_target_rotation_deg) {
        active_refinement_groups.push_back(group);
        baseline_group_stats.push_back(stats);
      }
    }

    if (!active_refinement_groups.empty() &&
        g_loop_post_residual_refinement_max_weight_scale > 1.0f + 1.0e-5f) {
      std::vector<float> trial_scales;
      for (const float scale : {1.5f, 2.0f, 3.0f, 4.0f, 6.0f, 8.0f}) {
        if (scale <=
            g_loop_post_residual_refinement_max_weight_scale + 1.0e-5f) {
          trial_scales.push_back(scale);
        }
      }
      if (trial_scales.empty() ||
          std::abs(trial_scales.back() -
                   g_loop_post_residual_refinement_max_weight_scale) >
              1.0e-5f) {
        trial_scales.push_back(
            g_loop_post_residual_refinement_max_weight_scale);
      }
      std::sort(trial_scales.begin(), trial_scales.end());
      trial_scales.erase(
          std::unique(trial_scales.begin(), trial_scales.end()),
          trial_scales.end());

      auto normalized_residual_score = [&] (
          const PostResidualStats& stats) {
        return std::max(
            stats.max_translation /
                std::max(
                    g_loop_post_residual_refinement_target_translation,
                    1.0e-6f),
            stats.max_rotation_deg /
                std::max(
                    g_loop_post_residual_refinement_target_rotation_deg,
                    1.0e-6f));
      };
      float baseline_worst_score = 0.0f;
      for (const auto& stats : baseline_group_stats) {
        baseline_worst_score = std::max(
            baseline_worst_score, normalized_residual_score(stats));
        post_residual_before_translation = std::max(
            post_residual_before_translation, stats.max_translation);
        post_residual_before_rotation_deg = std::max(
            post_residual_before_rotation_deg,
            stats.max_rotation_deg);
      }
      float best_trial_score = baseline_worst_score;
      PoseGraphEdgeVector best_trial_edges;
      PoseVector best_trial_poses;
      GraphSafetyMetrics best_trial_metrics;
      AdjacentCloudEvidence best_trial_adjacent_evidence;
      float best_trial_initial_cost = initial_graph_cost;
      float best_trial_optimized_cost = optimized_graph_cost;
      std::vector<PostResidualStats> best_trial_group_stats;

      for (const float trial_scale : trial_scales) {
        if (finalize_timed_out()) {
          break;
        }
        PoseGraphEdgeVector trial_edges = pose_graph_edges;
        int scaled_edges = 0;
        for (const auto& group : active_refinement_groups) {
          for (const std::size_t edge_index : group.edge_indices) {
            if (edge_index < trial_edges.size()) {
              trial_edges[edge_index].weight *= trial_scale;
              ++scaled_edges;
            }
          }
        }
        PoseVector trial_poses(loop_keyframes_.size(), SE3());
        const float trial_initial_cost =
            pose_graph_robust_cost(trial_poses, trial_edges);
        if (scaled_edges <= 0 ||
            !optimize_pose_graph(trial_poses, trial_edges)) {
          continue;
        }
        const float trial_optimized_cost =
            pose_graph_robust_cost(trial_poses, trial_edges);
        const GraphSafetyMetrics trial_metrics =
            measure_graph_safety(trial_poses);
        const AdjacentCloudEvidence trial_adjacent_evidence =
            evaluate_adjacent_cloud_evidence(trial_poses, trial_metrics);
        const bool trial_safe = graph_is_safe(
            trial_metrics, trial_initial_cost, trial_optimized_cost,
            trial_adjacent_evidence.supports_correction);
        float trial_worst_score = 0.0f;
        bool individual_residuals_safe = true;
        std::vector<PostResidualStats> trial_group_stats;
        trial_group_stats.reserve(active_refinement_groups.size());
        for (std::size_t group_index = 0;
             group_index < active_refinement_groups.size(); ++group_index) {
          const PostResidualStats stats = group_post_residual(
              active_refinement_groups[group_index],
              trial_edges, trial_poses);
          trial_group_stats.push_back(stats);
          trial_worst_score = std::max(
              trial_worst_score, normalized_residual_score(stats));
          const auto& baseline = baseline_group_stats[group_index];
          individual_residuals_safe = individual_residuals_safe &&
              stats.max_translation <=
                  baseline.max_translation + 0.01f &&
              stats.max_rotation_deg <=
                  baseline.max_rotation_deg + 0.02f;
        }
        LOG(INFO) << (trial_safe && individual_residuals_safe
                          ? GREEN : YELLOW)
                  << " ---> 已确认回环残差二次优化试算。scale: "
                  << trial_scale
                  << " groups: " << active_refinement_groups.size()
                  << " edges: " << scaled_edges
                  << " worst_normalized_residual: "
                  << baseline_worst_score << " -> " << trial_worst_score
                  << " graph_cost: " << trial_initial_cost
                  << " -> " << trial_optimized_cost
                  << " max_local_delta: "
                  << trial_metrics.max_local_translation_delta
                  << " max_local_vertical_delta: "
                  << trial_metrics.max_local_vertical_delta
                  << " individual_residuals_safe: "
                  << individual_residuals_safe
                  << " graph_safe: " << trial_safe << RESET;
        const bool materially_better =
            trial_worst_score <= 1.0f ||
            trial_worst_score <= 0.90f * baseline_worst_score;
        if (!trial_safe || !individual_residuals_safe ||
            !materially_better ||
            trial_worst_score >= best_trial_score - 1.0e-3f) {
          continue;
        }
        best_trial_score = trial_worst_score;
        best_trial_edges = std::move(trial_edges);
        best_trial_poses = std::move(trial_poses);
        best_trial_metrics = trial_metrics;
        best_trial_adjacent_evidence = trial_adjacent_evidence;
        best_trial_initial_cost = trial_initial_cost;
        best_trial_optimized_cost = trial_optimized_cost;
        best_trial_group_stats = std::move(trial_group_stats);
        post_residual_refinement_scale = trial_scale;
      }

      if (!best_trial_edges.empty()) {
        pose_graph_edges = std::move(best_trial_edges);
        correction_poses = std::move(best_trial_poses);
        graph_metrics = best_trial_metrics;
        adjacent_cloud_evidence = best_trial_adjacent_evidence;
        initial_graph_cost = best_trial_initial_cost;
        optimized_graph_cost = best_trial_optimized_cost;
        post_residual_refinement_groups = static_cast<int>(
            active_refinement_groups.size());
        for (std::size_t group_index = 0;
             group_index < active_refinement_groups.size(); ++group_index) {
          const auto& group = active_refinement_groups[group_index];
          const auto& before = baseline_group_stats[group_index];
          const auto& after = best_trial_group_stats[group_index];
          post_residual_refinement_edges += static_cast<int>(
              group.edge_indices.size());
          post_residual_after_translation = std::max(
              post_residual_after_translation, after.max_translation);
          post_residual_after_rotation_deg = std::max(
              post_residual_after_rotation_deg,
              after.max_rotation_deg);
          const bool resolved =
              after.max_translation <=
                  g_loop_post_residual_refinement_target_translation &&
              after.max_rotation_deg <=
                  g_loop_post_residual_refinement_target_rotation_deg;
          if (!resolved) {
            ++post_residual_unresolved_groups;
          }
          LOG(INFO) << (resolved ? GREEN : YELLOW)
                    << " ---> 已确认回环残差二次优化结果。type: "
                    << (group.endpoint
                            ? "endpoint" :
                              (group.z_only ? "pure_z" : "full"))
                    << " group_id: " << group.group_id
                    << " anchors: " << group.edge_indices.size()
                    << " scale: " << post_residual_refinement_scale
                    << " translation: " << before.max_translation
                    << " -> " << after.max_translation
                    << " z: " << before.max_z
                    << " -> " << after.max_z
                    << " rotation_deg: " << before.max_rotation_deg
                    << " -> " << after.max_rotation_deg
                    << " target_reached: " << resolved << RESET;
        }
        ++graph_loop_modifications;
      } else {
        post_residual_unresolved_groups = static_cast<int>(
            active_refinement_groups.size());
        post_residual_after_translation =
            post_residual_before_translation;
        post_residual_after_rotation_deg =
            post_residual_before_rotation_deg;
        post_residual_refinement_scale = 1.0f;
        LOG(WARNING) << YELLOW
                     << " ---> 已确认回环残差二次优化没有安全收益，"
                     << "完整保留第一次优化结果。groups: "
                     << active_refinement_groups.size()
                     << " max_translation: "
                     << post_residual_before_translation
                     << " max_rotation_deg: "
                     << post_residual_before_rotation_deg << RESET;
      }
    }
  }

  if (!graph_is_safe(
          graph_metrics,
          initial_graph_cost,
          optimized_graph_cost,
          adjacent_cloud_evidence.supports_correction)) {
    LOG(ERROR) << RED
               << " ---> 位姿图修正不可信，拒绝生成闭环地图。"
               << " graph_cost: " << initial_graph_cost
               << " -> " << optimized_graph_cost
               << " raw_route_length: " << raw_route_length
               << " max_correction_translation: "
               << graph_metrics.max_correction_translation
               << " max_total_translation: "
               << max_total_translation
               << " max_correction_rotation_deg: "
               << graph_metrics.max_correction_rotation_deg
               << " max_total_rotation_deg: "
               << max_total_yaw_deg
               << " max_adjacent_translation: "
               << graph_metrics.max_adjacent_translation
               << " max_adjacent_translation_vector: "
               << graph_metrics.max_adjacent_translation_vector.transpose()
               << " max_adjacent_translation_index: "
               << graph_metrics.max_adjacent_translation_index
               << " max_adjacent_rotation_deg: "
               << graph_metrics.max_adjacent_rotation_deg
               << " adjacent_cloud_evidence_valid: "
               << adjacent_cloud_evidence.valid
               << " adjacent_correction_supported: "
               << adjacent_cloud_evidence.supports_correction
               << " max_local_translation_strain: "
               << graph_metrics.max_local_translation_strain
               << " max_local_translation_strain_limit: "
               << max_local_translation_strain
               << " max_local_translation_delta: "
               << graph_metrics.max_local_translation_delta
               << " max_local_translation_delta_limit: "
               << max_local_translation_delta
               << " max_local_vertical_delta: "
               << graph_metrics.max_local_vertical_delta
               << " max_local_vertical_delta_limit: "
               << max_local_vertical_delta
               << " max_local_vertical_from: "
               << graph_metrics.max_local_vertical_from
               << " max_local_vertical_to: "
               << graph_metrics.max_local_vertical_to
               << " max_local_window_path_length: "
               << graph_metrics.max_local_window_path_length
               << " max_local_window_from: "
               << graph_metrics.max_local_window_from
               << " max_local_window_to: "
               << graph_metrics.max_local_window_to
               << " pruned_internal_loops: "
               << pruned_internal_loops
               << " pruned_internal_groups: "
               << pruned_internal_groups
               << " proactive_ground_z_loops: "
               << proactive_ground_z_loops
               << " proactive_ground_z_groups: "
               << proactive_ground_z_groups
               << " pruned_proactive_ground_z_loops: "
               << pruned_proactive_ground_z_loops
               << " pruned_proactive_ground_z_groups: "
               << pruned_proactive_ground_z_groups
               << " pruned_endpoint_loops: "
               << pruned_endpoint_loops
               << " downgraded_internal_z_loops: "
               << downgraded_internal_z_loops
               << " downgraded_endpoint_z_loops: "
               << downgraded_endpoint_z_loops
               << " downweighted_soft_fallback_loops: "
               << downweighted_soft_fallback_loops
               << " guarded_endpoint_proposal: "
               << best_guarded_endpoint_proposal
               << " corridor_sequence_partial: "
               << best_corridor_sequence_partial
               << " endpoint_distributed_anchor_group_active: "
               << endpoint_distributed_anchor_group_active
               << " guarded_endpoint_alpha: "
               << guarded_endpoint_alpha
               << " guarded_endpoint_line_search_steps: "
               << guarded_endpoint_line_search_steps
               << " proactive_ground_z_line_search_steps: "
               << proactive_ground_z_line_search_steps
               << " post_residual_refinement_groups: "
               << post_residual_refinement_groups
               << " post_residual_refinement_edges: "
               << post_residual_refinement_edges
               << " post_residual_unresolved_groups: "
               << post_residual_unresolved_groups
               << " post_residual_refinement_scale: "
               << post_residual_refinement_scale
               << " post_residual_translation: "
               << post_residual_before_translation << " -> "
               << post_residual_after_translation
               << " post_residual_rotation_deg: "
               << post_residual_before_rotation_deg << " -> "
               << post_residual_after_rotation_deg << RESET;
    return;
  }
  LOG(INFO) << GREEN
            << " ---> 多回环位姿图优化完成。nodes: "
            << correction_poses.size()
            << " internal_loops: " << accepted_internal_loops
            << " internal_groups: " << accepted_internal_groups
            << " pruned_internal_loops: " << pruned_internal_loops
            << " pruned_internal_groups: " << pruned_internal_groups
            << " proactive_ground_z_loops: "
            << proactive_ground_z_loops
            << " proactive_ground_z_groups: "
            << proactive_ground_z_groups
            << " pruned_proactive_ground_z_loops: "
            << pruned_proactive_ground_z_loops
            << " pruned_proactive_ground_z_groups: "
            << pruned_proactive_ground_z_groups
            << " endpoint_loop_active: " << endpoint_edge_active
            << " pruned_endpoint_loops: " << pruned_endpoint_loops
            << " downgraded_internal_z_loops: "
            << downgraded_internal_z_loops
            << " downgraded_endpoint_z_loops: "
            << downgraded_endpoint_z_loops
            << " downweighted_soft_fallback_loops: "
            << downweighted_soft_fallback_loops
            << " guarded_endpoint_proposal: "
            << best_guarded_endpoint_proposal
            << " corridor_sequence_partial: "
            << best_corridor_sequence_partial
            << " endpoint_distributed_anchor_group_active: "
            << endpoint_distributed_anchor_group_active
            << " guarded_endpoint_alpha: "
            << guarded_endpoint_alpha
            << " guarded_endpoint_line_search_steps: "
            << guarded_endpoint_line_search_steps
            << " proactive_ground_z_line_search_steps: "
            << proactive_ground_z_line_search_steps
            << " post_residual_refinement_groups: "
            << post_residual_refinement_groups
            << " post_residual_refinement_edges: "
            << post_residual_refinement_edges
            << " post_residual_unresolved_groups: "
            << post_residual_unresolved_groups
            << " post_residual_refinement_scale: "
            << post_residual_refinement_scale
            << " post_residual_translation: "
            << post_residual_before_translation << " -> "
            << post_residual_after_translation
            << " post_residual_rotation_deg: "
            << post_residual_before_rotation_deg << " -> "
            << post_residual_after_rotation_deg
            << " graph_cost: " << initial_graph_cost
            << " -> " << optimized_graph_cost
            << " raw_route_length: " << raw_route_length
            << " max_correction_translation: "
            << graph_metrics.max_correction_translation
            << " max_correction_rotation_deg: "
            << graph_metrics.max_correction_rotation_deg
            << " max_adjacent_translation: "
            << graph_metrics.max_adjacent_translation
            << " max_adjacent_translation_vector: "
            << graph_metrics.max_adjacent_translation_vector.transpose()
            << " max_adjacent_translation_index: "
            << graph_metrics.max_adjacent_translation_index
            << " max_adjacent_rotation_deg: "
            << graph_metrics.max_adjacent_rotation_deg
            << " max_local_translation_strain: "
            << graph_metrics.max_local_translation_strain
            << " max_local_translation_delta: "
            << graph_metrics.max_local_translation_delta
            << " max_local_translation_delta_limit: "
            << max_local_translation_delta
            << " max_local_vertical_delta: "
            << graph_metrics.max_local_vertical_delta
            << " max_local_vertical_delta_limit: "
            << max_local_vertical_delta
            << " max_local_vertical_from: "
            << graph_metrics.max_local_vertical_from
            << " max_local_vertical_to: "
            << graph_metrics.max_local_vertical_to
            << " max_local_window_path_length: "
            << graph_metrics.max_local_window_path_length
            << " max_local_window_from: "
            << graph_metrics.max_local_window_from
            << " max_local_window_to: "
            << graph_metrics.max_local_window_to
            << " adjacent_correction_supported: "
            << adjacent_cloud_evidence.supports_correction << RESET;

  CloudPtr filtered_loop_map(new PointCloudType());
  CloudPtr loop_batch(new PointCloudType());
  const std::size_t loop_batch_limit =
      g_map_max_points_in_memory > 0
      ? static_cast<std::size_t>(g_map_max_points_in_memory)
      : 2000000U;
  auto flush_loop_batch = [&]() {
    if (loop_batch->empty()) {
      return true;
    }
    if (!merge_cloud_incrementally(
            filtered_loop_map,
            loop_batch,
            g_loop_map_ds_size,
            "bounded loop merge")) {
      return false;
    }
    loop_batch.reset(new PointCloudType());
    return true;
  };
  TimedPoseCorrectionVector pose_corrections;
  pose_corrections.reserve(loop_keyframes_.size());
  for (int i = 0; i <= end_idx; ++i) {
    if ((i % 64) == 0 && finalize_timed_out()) {
      return;
    }
    const SE3 optimized_pose =
        correction_poses[i] * raw_poses[i];
    CloudPtr keyframe_cloud =
        loadLoopKeyFrameCloud(static_cast<std::size_t>(i));
    if (!keyframe_cloud || keyframe_cloud->empty()) {
      LOG(ERROR) << RED << " ---> 闭环地图缺少关键帧点云。index="
                 << i << RESET;
      return;
    }
    CloudPtr transformed(new PointCloudType());
    pcl::transformPointCloud(
        *keyframe_cloud,
        *transformed,
        optimized_pose.T());
    *loop_batch += *transformed;
    if (loop_batch->size() >= loop_batch_limit &&
        !flush_loop_batch()) {
      return;
    }

    TimedPoseCorrection sample;
    sample.timestamp = loop_keyframes_[i].timestamp;
    sample.correction = correction_poses[i].T();
    pose_corrections.push_back(sample);
  }

  if (finalize_timed_out()) {
    return;
  }
  if (!flush_loop_batch()) {
    return;
  }

  if (finalize_timed_out()) {
    return;
  }

  if (save_pcd_binary_safe(loop_map_path, *filtered_loop_map)) {
    LOG(INFO) << GREEN << " ---> 闭环地图已保存: " << loop_map_path << RESET;
    LOG(INFO) << GREEN << " ---> 闭环地图点数: " << filtered_loop_map->size() << RESET;

    const std::filesystem::path canonical_map_path =
        std::filesystem::path(g_save_map_dir) / g_map_name;
    const std::filesystem::path raw_map_path =
        canonical_map_path.parent_path() /
        (canonical_map_path.stem().string() + "_raw" +
         canonical_map_path.extension().string());
    std::error_code copy_error;
    if (std::filesystem::exists(canonical_map_path)) {
      std::filesystem::copy_file(
          canonical_map_path,
          raw_map_path,
          std::filesystem::copy_options::overwrite_existing,
          copy_error);
      if (copy_error) {
        LOG(WARNING) << YELLOW << " ---> 原始未闭环地图备份失败: "
                     << raw_map_path.string()
                     << " error: " << copy_error.message() << RESET;
      }
    }

    if (!copy_file_atomic(loop_map_path, canonical_map_path)) {
      LOG(ERROR) << RED << " ---> 闭环地图无法提交为正式地图: "
                 << canonical_map_path.string() << RESET;
      return;
    }
    if (!save_loop_trajectory_safe(
            trajectory_path, pose_corrections)) {
      LOG(ERROR) << RED
                 << " ---> 正式地图已闭环，但位姿图轨迹保存失败。"
                 << RESET;
      return;
    }
    LOG(INFO) << GREEN << " ---> 闭环地图已提交为正式 map.pcd；"
              << "terrain 将按同一条优化轨迹重建（不锁定 Z）。"
              << RESET;
  }
}


void SuperLIO::saveMap(){
  if(!g_save_map) return;
  // The online worker may still be reading persisted keyframes. Stop it
  // before closing the writer; all unprocessed tasks remain represented by
  // the immutable keyframe set consumed by the complete final backend below.
  stopOnlineLoopWorker();
  const bool fragments_enabled =
      g_map_max_points_in_memory > 0 || g_pcd_save_interval > 0;
  bool raw_map_saved = false;

  if (fragments_enabled) {
    flushMapFragment(true);
  }
  const bool writer_ok = stopMapWriter();
  LOG(INFO) << GREEN
            << " ---> [SuperLIO]: background map writer summary."
            << " queue_capacity=" << g_map_max_pending_writes
            << " max_outstanding=" << map_writer_max_outstanding_
            << " nonblocking_busy_events=" << map_writer_queue_full_count_
            << RESET;
  if (!writer_ok) {
    LOG(ERROR) << RED
               << " ---> Map persistence writer did not flush all queued data; "
               << "keeping work files for recovery." << RESET;
    return;
  }

  if (fragments_enabled && point_map_ && !point_map_->empty()) {
    // A background write may have failed before shutdown and then recovered
    // synchronously. Preserve the still-active tail as one final fragment.
    std::ostringstream filename;
    filename << "fragment_" << std::setw(6) << std::setfill('0')
             << (pcd_index_ + 1) << ".pcd";
    const auto path = map_fragment_dir_ / filename.str();
    PointCloudType filtered_tail;
    make_map_pcd_cloud(
        point_map_, filtered_tail, g_map_ds_size, "shutdown tail");
    if (!filtered_tail.empty() &&
        save_pcd_binary_safe(path.string(), filtered_tail)) {
      ++pcd_index_;
      map_fragment_paths_.push_back(path);
      point_map_->clear();
    } else {
      LOG(ERROR) << RED << " ---> Failed to persist final in-memory map tail."
                 << RESET;
      return;
    }
  }

  if (fragments_enabled) {
    LOG(INFO) << YELLOW << " ---> Processing bounded map fragments..." << RESET;
    raw_map_saved = ProcessCaceMap();
  } else if(point_map_ && !point_map_->empty()){
    LOG(INFO) << YELLOW << " ---> Saving in-memory map..... " << RESET;
    std::string map_name = g_save_map_dir + "/" + g_map_name;
    LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
    PointCloudType latst_map;
    make_map_pcd_cloud(point_map_, latst_map, g_map_ds_size, "final");
    if (save_pcd_binary_safe(map_name, latst_map)) {
      LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << RESET;
      LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
      raw_map_saved = true;
    }
  }

  if (!raw_map_saved) {
    LOG(ERROR) << RED
               << " ---> Raw map was not saved; keeping mapping work files."
               << RESET;
    return;
  }
  if (!saveFrontendKeyFrameTrajectory()) {
    LOG(ERROR) << RED
               << " ---> Failed to save frontend keyframe trajectory: "
               << (std::filesystem::path(g_save_map_dir) /
                   "frontend_keyframe_trajectory.txt").string()
               << RESET;
  } else {
    LOG(INFO) << GREEN
              << " ---> Frontend keyframe trajectory saved. keyframes="
              << loop_keyframes_.size()
              << " path="
              << (std::filesystem::path(g_save_map_dir) /
                  "frontend_keyframe_trajectory.txt").string()
              << RESET;
  }
  saveLoopClosedMap();
  cleanupMapPersistenceFiles();
}


inline double get_cpu_time_seconds() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}


bool SuperLIO::Propagation_Undistort(){
  auto reject_undistortion = [this](const std::string& reason) {
    ++rejected_undistortion_count_;
    const auto now = std::chrono::steady_clock::now();
    if (last_undistortion_warning_time_ == std::chrono::steady_clock::time_point{} ||
        now - last_undistortion_warning_time_ > std::chrono::seconds(1)) {
      last_undistortion_warning_time_ = now;
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: reject LiDAR undistortion. " << reason
                   << " rejected=" << rejected_undistortion_count_ << RESET;
    }
    scan_undistort_full_->clear();
    ds_undistort_->clear();
    return false;
  };

  auto& raw_pc = measures_.lidar.pc;
  if (!raw_pc || raw_pc->empty()) {
    return reject_undistortion("empty point cloud");
  }

  constexpr double time_epsilon = 1e-6;
  const double coverage_tolerance =
      std::max(g_scan_boundary_tolerance, time_epsilon);
  double minimum_query_time = std::numeric_limits<double>::infinity();
  double maximum_query_time = -std::numeric_limits<double>::infinity();
  for (const auto& point : raw_pc->points) {
    if (!std::isfinite(point.offset_time) || point.offset_time < 0.0) {
      return reject_undistortion("invalid point offset time");
    }
    const double query_time = measures_.lidar.start_time + point.offset_time;
    minimum_query_time = std::min(minimum_query_time, query_time);
    maximum_query_time = std::max(maximum_query_time, query_time);
  }

  if (minimum_query_time < measures_.lidar.start_time - time_epsilon ||
      maximum_query_time > measures_.lidar.end_time + time_epsilon) {
    std::ostringstream reason;
    reason << "point time outside scan range: point=["
           << minimum_query_time << ", " << maximum_query_time
           << "] scan=[" << measures_.lidar.start_time
           << ", " << measures_.lidar.end_time << "]";
    return reject_undistortion(reason.str());
  }

  propagate_states_.clear();
  propagate_states_.emplace_back(kf_->GetDynamicState());
  kf_->SetObsTime(measures_.lidar.end_time);
  for (auto &imu : measures_.imu) {
    kf_->Predict(imu);
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }

  // Keep a short gravity-direction average for validating local plane normals.
  // It is deliberately not used as a direct attitude correction: walking
  // acceleration can tilt an accelerometer-only estimate by several degrees.
  updateGravityReference();

  static const M3 TLI_R = g_lidar_imu.R_;
  static const V3 TLI_t = g_lidar_imu.t_;
  const SE3 T_end = kf_->GetSE3();
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;
  const double start_time = measures_.lidar.start_time;

  if (propagate_states_.size() < 2 ||
      propagate_states_.front().time > minimum_query_time + coverage_tolerance ||
      propagate_states_.back().time < maximum_query_time - coverage_tolerance) {
    std::ostringstream reason;
    reason << "state coverage incomplete: state=["
           << propagate_states_.front().time << ", "
           << propagate_states_.back().time << "] point=["
           << minimum_query_time << ", " << maximum_query_time << "]";
    return reject_undistortion(reason.str());
  }

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full_->resize(ptsize); 

  tbb::parallel_for(
  tbb::blocked_range<size_t>(0, ptsize),
  [&](const tbb::blocked_range<size_t>& r) {
    M3 R_h, R_t; V3 p_h, v_h, acc_t;
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {  
      auto& pt_full = scan_undistort_full_->points[idx];
      const auto& pt = raw_pc->points[idx];
      pt_full.intensity = pt.intensity;
      double query_time = start_time + pt.offset_time;

      auto match_iter_n = std::upper_bound(
        propagate_states_.begin(), propagate_states_.end(), query_time,
        [](double time, const DynamicState& state) { return time < state.time; });
      if (match_iter_n == propagate_states_.begin()) {
        match_iter_n = std::next(propagate_states_.begin());
      } else if (match_iter_n == propagate_states_.end()) {
        match_iter_n = std::prev(propagate_states_.end());
      }
      auto match_iter = std::prev(match_iter_n);
      double imu_dt = match_iter_n->time - match_iter->time;
      const double query_dt = std::clamp(
        query_time - match_iter->time, 0.0, std::max(imu_dt, 0.0));
      const double s = imu_dt > 1e-9 ? query_dt / imu_dt : 0.0;
      R_h = match_iter->R;
      R_t = match_iter_n->R;
      p_h = match_iter->p;
      v_h = match_iter->v;
      acc_t = match_iter_n->a;
      M3 R_i = Quat(R_h).slerp(s, Quat(R_t)).toRotationMatrix();
      const double trans_dt = g_use_query_time_undistort ? query_dt : imu_dt;
      V3 t_ei(p_h + v_h * trans_dt + 0.5 * acc_t * trans_dt * trans_dt - T_end_t);
      V3 raw(pt.x, pt.y, pt.z);
      V3 eigen_point = R_inv * (R_i * (TLI_R * raw + TLI_t) + t_ei);
      pt_full.x = eigen_point[0];
      pt_full.y = eigen_point[1];
      pt_full.z = eigen_point[2];
    }
  });
  return true;
}


void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
}


void SuperLIO::updateGravityReference(){
  if (!g_level_constraint_enable || imu_reference_accel_norm_ <= 1e-6 ||
      measures_.imu.empty() || propagate_states_.size() < 2) {
    gravity_reference_valid_ = false;
    return;
  }

  const std::size_t sample_count = std::min(
      measures_.imu.size(), propagate_states_.size() - 1);
  for (std::size_t i = 0; i < sample_count; ++i) {
    const auto& imu = measures_.imu[i];
    if (imu.secs <= last_gravity_sample_time_ + 1e-9) {
      continue;
    }
    last_gravity_sample_time_ = imu.secs;

    const double accel_norm = imu.acc.norm();
    if (!std::isfinite(accel_norm) || accel_norm <= 1e-6) {
      continue;
    }
    const double norm_error = std::abs(
        accel_norm / imu_reference_accel_norm_ - 1.0);
    if (norm_error > g_level_max_accel_norm_ratio) {
      continue;
    }

    V3 up_world = propagate_states_[i + 1].R * (imu.acc / accel_norm);
    const scalar up_norm = up_world.norm();
    if (!up_world.allFinite() || up_norm <= 1e-6) {
      continue;
    }
    gravity_direction_window_.push_back(
        GravityDirectionSample{imu.secs, up_world / up_norm});
  }

  const double newest_time = measures_.imu.back().secs;
  while (!gravity_direction_window_.empty() &&
         newest_time - gravity_direction_window_.front().timestamp >
             g_level_gravity_window_sec) {
    gravity_direction_window_.pop_front();
  }

  const double minimum_duration = std::min(
      0.5, 0.5 * g_level_gravity_window_sec);
  if (gravity_direction_window_.size() < 20 ||
      gravity_direction_window_.back().timestamp -
              gravity_direction_window_.front().timestamp < minimum_duration) {
    gravity_reference_valid_ = false;
    return;
  }

  V3 mean_up = V3::Zero();
  for (const auto& sample : gravity_direction_window_) {
    mean_up += sample.up_world;
  }
  const scalar mean_norm = mean_up.norm();
  gravity_reference_valid_ = mean_up.allFinite() && mean_norm > 1e-6;
  if (gravity_reference_valid_) {
    gravity_reference_world_ = mean_up / mean_norm;
  }
}


SuperLIO::LevelPlaneObservation SuperLIO::estimateLevelPlane() const {
  LevelPlaneObservation observation;
  if (!g_level_constraint_enable || !gravity_reference_valid_ ||
      !ds_undistort_ || ds_undistort_->empty()) {
    return observation;
  }

  const SE3 predicted_pose = kf_->GetSE3();
  V3 up_body = predicted_pose.R_.transpose() * gravity_reference_world_;
  const scalar up_body_norm = up_body.norm();
  if (!up_body.allFinite() || up_body_norm <= 1e-6) {
    return observation;
  }
  up_body /= up_body_norm;

  std::vector<V3> candidates;
  candidates.reserve(ds_undistort_->size());
  const double max_range_squared =
      g_level_max_point_range * g_level_max_point_range;
  for (const auto& point : ds_undistort_->points) {
    const V3 p(point.x, point.y, point.z);
    if (!p.allFinite()) {
      continue;
    }
    const double range_squared = p.squaredNorm();
    if (range_squared < 0.25 || range_squared > max_range_squared) {
      continue;
    }
    const double down_projection = -up_body.dot(p);
    if (down_projection < g_level_min_down_distance ||
        down_projection > g_level_max_down_distance) {
      continue;
    }
    candidates.push_back(p);
  }

  observation.candidate_count = static_cast<int>(candidates.size());
  const int required_inliers = std::max(
      g_level_min_plane_inliers,
      static_cast<int>(std::ceil(
          g_level_min_plane_inlier_ratio * candidates.size())));
  if (static_cast<int>(candidates.size()) < required_inliers ||
      candidates.size() < 3) {
    return observation;
  }

  // The RANSAC seed is frame-derived, making bag replays deterministic while
  // avoiding a geometry bias from repeatedly choosing the same point triples.
  std::mt19937 generator(
      static_cast<std::uint32_t>(frame_num_) * 2654435761u + 0x9e3779b9u);
  std::uniform_int_distribution<std::size_t> sample_index(
      0, candidates.size() - 1);
  const double candidate_angle_deg = std::max(
      10.0, 2.0 * g_level_max_plane_gravity_angle_deg);
  const double candidate_cosine = std::cos(
      candidate_angle_deg * M_PI / 180.0);

  int best_count = 0;
  V3 best_normal = V3::UnitZ();
  double best_offset = 0.0;
  for (int iteration = 0; iteration < g_level_ransac_iterations; ++iteration) {
    const std::size_t i0 = sample_index(generator);
    const std::size_t i1 = sample_index(generator);
    const std::size_t i2 = sample_index(generator);
    if (i0 == i1 || i0 == i2 || i1 == i2) {
      continue;
    }

    V3 normal = (candidates[i1] - candidates[i0]).cross(
        candidates[i2] - candidates[i0]);
    const scalar normal_norm = normal.norm();
    if (!normal.allFinite() || normal_norm <= 1e-5) {
      continue;
    }
    normal /= normal_norm;
    if (normal.dot(up_body) < 0.0) {
      normal = -normal;
    }
    if (normal.dot(up_body) < candidate_cosine) {
      continue;
    }

    const double offset = -normal.dot(candidates[i0]);
    int inlier_count = 0;
    for (const auto& candidate : candidates) {
      if (std::abs(normal.dot(candidate) + offset) <=
          g_level_plane_distance_threshold) {
        ++inlier_count;
      }
    }
    if (inlier_count > best_count) {
      best_count = inlier_count;
      best_normal = normal;
      best_offset = offset;
    }
  }

  if (best_count < required_inliers) {
    return observation;
  }

  V3 centroid = V3::Zero();
  std::vector<const V3*> inliers;
  inliers.reserve(best_count);
  for (const auto& candidate : candidates) {
    if (std::abs(best_normal.dot(candidate) + best_offset) <=
        g_level_plane_distance_threshold) {
      inliers.push_back(&candidate);
      centroid += candidate;
    }
  }
  if (static_cast<int>(inliers.size()) < required_inliers) {
    return observation;
  }
  centroid /= static_cast<scalar>(inliers.size());

  M3 covariance = M3::Zero();
  for (const V3* point : inliers) {
    const V3 centered = *point - centroid;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<scalar>(inliers.size());
  Eigen::SelfAdjointEigenSolver<M3> eigen_solver(covariance);
  if (eigen_solver.info() != Eigen::Success) {
    return observation;
  }
  const V3 eigenvalues = eigen_solver.eigenvalues();
  if (!eigenvalues.allFinite() || eigenvalues[1] < 0.04) {
    return observation;
  }

  V3 refined_normal = eigen_solver.eigenvectors().col(0);
  if (refined_normal.dot(up_body) < 0.0) {
    refined_normal = -refined_normal;
  }
  const double refined_offset = -refined_normal.dot(centroid);
  double squared_error_sum = 0.0;
  int refined_inlier_count = 0;
  for (const auto& candidate : candidates) {
    const double error = refined_normal.dot(candidate) + refined_offset;
    if (std::abs(error) <= g_level_plane_distance_threshold) {
      squared_error_sum += error * error;
      ++refined_inlier_count;
    }
  }
  if (refined_inlier_count < required_inliers) {
    return observation;
  }

  const double rms = std::sqrt(
      squared_error_sum / static_cast<double>(refined_inlier_count));
  if (!std::isfinite(rms) ||
      rms > 0.8 * g_level_plane_distance_threshold) {
    return observation;
  }

  const auto angle_deg = [](const V3& a, const V3& b) {
    return std::acos(std::clamp(
        static_cast<double>(a.dot(b)), -1.0, 1.0)) * 180.0 / M_PI;
  };
  const double gravity_angle_deg = angle_deg(refined_normal, up_body);
  observation.normal_body = refined_normal;
  observation.plane_offset_body = refined_offset;
  observation.inlier_count = refined_inlier_count;
  observation.inlier_ratio = static_cast<double>(refined_inlier_count) /
      static_cast<double>(candidates.size());
  observation.rms = rms;
  observation.gravity_angle_deg = gravity_angle_deg;
  observation.slope_protection_active =
      level_slope_protection_active_;

  V3 plane_up_world = predicted_pose.R_ * refined_normal;
  const scalar plane_up_world_norm = plane_up_world.norm();
  if (!plane_up_world.allFinite() || plane_up_world_norm <= 1e-6) {
    return observation;
  }
  plane_up_world /= plane_up_world_norm;
  const double innovation_deg = angle_deg(plane_up_world, V3::UnitZ());
  observation.innovation_deg = innovation_deg;
  observation.plane_valid = true;
  observation.slope_enter_evidence =
      gravity_angle_deg >= g_level_max_plane_gravity_angle_deg &&
      innovation_deg >= g_level_max_plane_gravity_angle_deg;
  observation.slope_exit_evidence =
      std::min(gravity_angle_deg, innovation_deg) <=
          g_level_slope_exit_angle_deg &&
      innovation_deg <= g_level_max_attitude_innovation_deg;
  observation.dynamic_gravity_mismatch =
      gravity_angle_deg >= g_level_max_plane_gravity_angle_deg &&
      innovation_deg <= g_level_slope_soft_start_angle_deg;

  if (level_slope_protection_active_) {
    // A latched ramp remains protected through short noisy dips. The state is
    // advanced only after the complete LiDAR observation is committed.
    observation.slope_rejected = true;
    return observation;
  }
  if (innovation_deg > g_level_max_attitude_innovation_deg) {
    return observation;
  }

  observation.valid = true;
  return observation;
}


SuperLIO::WallYawObservation SuperLIO::estimateWallYaw() const {
  WallYawObservation observation;
  if (!g_wall_yaw_constraint_enable || !gravity_reference_valid_ ||
      !ds_undistort_ || ds_undistort_->empty()) {
    return observation;
  }
  if (processed_scan_index_ %
          static_cast<std::uint64_t>(g_wall_yaw_extraction_interval_frames) !=
      0U) {
    return observation;
  }
  observation.extraction_attempted = true;

  const SE3 predicted_pose = kf_->GetSE3();
  V3 up_body = predicted_pose.R_.transpose() * gravity_reference_world_;
  const scalar up_body_norm = up_body.norm();
  if (!up_body.allFinite() || up_body_norm <= 1e-6) {
    return observation;
  }
  up_body /= up_body_norm;

  std::vector<V3> candidates;
  candidates.reserve(ds_undistort_->size());
  const double min_range_squared = 1.0;
  const double max_range_squared =
      g_wall_yaw_max_point_range * g_wall_yaw_max_point_range;
  for (const auto& point : ds_undistort_->points) {
    const V3 candidate(point.x, point.y, point.z);
    if (!candidate.allFinite()) {
      continue;
    }
    const double range_squared = candidate.squaredNorm();
    if (range_squared < min_range_squared ||
        range_squared > max_range_squared) {
      continue;
    }
    candidates.push_back(candidate);
  }

  observation.candidate_count = static_cast<int>(candidates.size());
  const int required_inliers = std::max(
      g_wall_yaw_min_plane_inliers,
      static_cast<int>(std::ceil(
          g_wall_yaw_min_plane_inlier_ratio * candidates.size())));
  if (static_cast<int>(candidates.size()) < required_inliers ||
      candidates.size() < 3) {
    return observation;
  }

  // 固定随机种子保证同一 rosbag 的墙面提取结果可重复。
  std::mt19937 generator(
      static_cast<std::uint32_t>(processed_scan_index_) * 2654435761u +
      0x6a09e667u);
  std::uniform_int_distribution<std::size_t> sample_index(
      0, candidates.size() - 1);
  const double candidate_vertical_angle_deg = std::max(
      15.0, 2.0 * g_wall_yaw_max_vertical_angle_deg);
  const double candidate_vertical_sine = std::sin(
      candidate_vertical_angle_deg * M_PI / 180.0);

  int best_count = 0;
  V3 best_normal = V3::UnitX();
  double best_offset = 0.0;
  for (int iteration = 0;
       iteration < g_wall_yaw_ransac_iterations;
       ++iteration) {
    const std::size_t i0 = sample_index(generator);
    const std::size_t i1 = sample_index(generator);
    const std::size_t i2 = sample_index(generator);
    if (i0 == i1 || i0 == i2 || i1 == i2) {
      continue;
    }

    V3 normal = (candidates[i1] - candidates[i0]).cross(
        candidates[i2] - candidates[i0]);
    const scalar normal_norm = normal.norm();
    if (!normal.allFinite() || normal_norm <= 1e-5) {
      continue;
    }
    normal /= normal_norm;
    if (std::abs(normal.dot(up_body)) > candidate_vertical_sine) {
      continue;
    }

    const double offset = -normal.dot(candidates[i0]);
    int inlier_count = 0;
    for (const auto& candidate : candidates) {
      if (std::abs(normal.dot(candidate) + offset) <=
          g_wall_yaw_plane_distance_threshold) {
        ++inlier_count;
      }
    }
    if (inlier_count > best_count) {
      best_count = inlier_count;
      best_normal = normal;
      best_offset = offset;
    }
  }

  if (best_count < required_inliers) {
    return observation;
  }

  V3 centroid = V3::Zero();
  std::vector<const V3*> inliers;
  inliers.reserve(best_count);
  for (const auto& candidate : candidates) {
    if (std::abs(best_normal.dot(candidate) + best_offset) <=
        g_wall_yaw_plane_distance_threshold) {
      inliers.push_back(&candidate);
      centroid += candidate;
    }
  }
  if (static_cast<int>(inliers.size()) < required_inliers) {
    return observation;
  }
  centroid /= static_cast<scalar>(inliers.size());

  M3 covariance = M3::Zero();
  for (const V3* point : inliers) {
    const V3 centered = *point - centroid;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<scalar>(inliers.size());
  Eigen::SelfAdjointEigenSolver<M3> eigen_solver(covariance);
  if (eigen_solver.info() != Eigen::Success) {
    return observation;
  }
  const V3 eigenvalues = eigen_solver.eigenvalues();
  if (!eigenvalues.allFinite() || eigenvalues[1] < 0.04) {
    return observation;
  }

  V3 refined_normal = eigen_solver.eigenvectors().col(0);
  if (refined_normal.dot(best_normal) < 0.0) {
    refined_normal = -refined_normal;
  }
  const double vertical_angle_deg = std::asin(std::clamp(
      std::abs(static_cast<double>(refined_normal.dot(up_body))),
      0.0, 1.0)) * 180.0 / M_PI;
  if (vertical_angle_deg > g_wall_yaw_max_vertical_angle_deg) {
    return observation;
  }

  const double refined_offset = -refined_normal.dot(centroid);
  std::vector<const V3*> refined_inliers;
  refined_inliers.reserve(inliers.size());
  double squared_error_sum = 0.0;
  for (const auto& candidate : candidates) {
    const double error = refined_normal.dot(candidate) + refined_offset;
    if (std::abs(error) <= g_wall_yaw_plane_distance_threshold) {
      refined_inliers.push_back(&candidate);
      squared_error_sum += error * error;
    }
  }
  if (static_cast<int>(refined_inliers.size()) < required_inliers) {
    return observation;
  }

  const double rms = std::sqrt(
      squared_error_sum / static_cast<double>(refined_inliers.size()));
  if (!std::isfinite(rms) ||
      rms > 0.8 * g_wall_yaw_plane_distance_threshold) {
    return observation;
  }

  V3 horizontal_tangent = up_body.cross(refined_normal);
  const scalar tangent_norm = horizontal_tangent.norm();
  if (!horizontal_tangent.allFinite() || tangent_norm <= 1e-6) {
    return observation;
  }
  horizontal_tangent /= tangent_norm;

  std::vector<double> vertical_coordinates;
  std::vector<double> horizontal_coordinates;
  vertical_coordinates.reserve(refined_inliers.size());
  horizontal_coordinates.reserve(refined_inliers.size());
  for (const V3* point : refined_inliers) {
    const V3 centered = *point - centroid;
    vertical_coordinates.push_back(centered.dot(up_body));
    horizontal_coordinates.push_back(centered.dot(horizontal_tangent));
  }
  std::sort(vertical_coordinates.begin(), vertical_coordinates.end());
  std::sort(horizontal_coordinates.begin(), horizontal_coordinates.end());
  const std::size_t lower_index = static_cast<std::size_t>(std::floor(
      0.05 * static_cast<double>(refined_inliers.size() - 1)));
  const std::size_t upper_index = static_cast<std::size_t>(std::ceil(
      0.95 * static_cast<double>(refined_inliers.size() - 1)));
  const double vertical_span =
      vertical_coordinates[upper_index] - vertical_coordinates[lower_index];
  const double horizontal_span =
      horizontal_coordinates[upper_index] - horizontal_coordinates[lower_index];
  if (!std::isfinite(vertical_span) || !std::isfinite(horizontal_span) ||
      vertical_span < g_wall_yaw_min_vertical_span ||
      horizontal_span < g_wall_yaw_min_horizontal_span) {
    return observation;
  }

  observation.plane_valid = true;
  observation.normal_body = refined_normal;
  observation.inlier_count = static_cast<int>(refined_inliers.size());
  observation.inlier_ratio = static_cast<double>(refined_inliers.size()) /
      static_cast<double>(candidates.size());
  observation.rms = rms;
  observation.vertical_angle_deg = vertical_angle_deg;
  observation.vertical_span = vertical_span;
  observation.horizontal_span = horizontal_span;

  return observation;
}


void SuperLIO::prepareWallYawConstraint(
    WallYawObservation& observation,
    const SE3& pose,
    const double lidar_raw_yaw_information_ratio,
    const double lidar_conditional_yaw_information_ratio,
    const double lidar_translation_information_ratio) {
  observation.lidar_raw_yaw_information_ratio = std::clamp(
      lidar_raw_yaw_information_ratio, 0.0, 1.0);
  observation.lidar_conditional_yaw_information_ratio = std::clamp(
      lidar_conditional_yaw_information_ratio, 0.0, 1.0);
  observation.lidar_translation_information_ratio = std::clamp(
      lidar_translation_information_ratio, 0.0, 1.0);
  observation.reference_valid = !wall_yaw_references_.empty();

  if (observation.plane_valid) {
    const auto unit_ratio = [](const double value, const double scale) {
      return std::clamp(value / std::max(scale, 1e-9), 0.0, 1.0);
    };
    const double inlier_quality = unit_ratio(
        static_cast<double>(observation.inlier_count),
        2.0 * static_cast<double>(g_wall_yaw_min_plane_inliers));
    const double ratio_quality = unit_ratio(
        observation.inlier_ratio,
        2.0 * g_wall_yaw_min_plane_inlier_ratio);
    const double rms_quality = std::clamp(
        1.0 - observation.rms /
            std::max(g_wall_yaw_plane_distance_threshold, 1e-9),
        0.0,
        1.0);
    const double vertical_span_quality = unit_ratio(
        observation.vertical_span,
        2.0 * g_wall_yaw_min_vertical_span);
    const double horizontal_span_quality = unit_ratio(
        observation.horizontal_span,
        2.0 * g_wall_yaw_min_horizontal_span);
    const double verticality_quality = std::clamp(
        1.0 - observation.vertical_angle_deg /
            std::max(g_wall_yaw_max_vertical_angle_deg, 1e-9),
        0.0,
        1.0);
    observation.recapture_scene_quality = (1.0 / 6.0) * (
        inlier_quality + ratio_quality + rms_quality +
        vertical_span_quality + horizontal_span_quality +
        verticality_quality);
  }

  const double weak_ratio = g_wall_yaw_information_weak_ratio;
  const double strong_ratio = g_wall_yaw_information_strong_ratio;
  if (observation.lidar_conditional_yaw_information_ratio <= weak_ratio) {
    observation.observability_gate = 1.0;
  } else if (observation.lidar_conditional_yaw_information_ratio >=
             strong_ratio) {
    observation.observability_gate = 0.0;
  } else {
    const double linear_gate =
        (strong_ratio -
         observation.lidar_conditional_yaw_information_ratio) /
        (strong_ratio - weak_ratio);
    // Smoothstep avoids a discontinuous correction when geometry changes.
    observation.observability_gate =
        linear_gate * linear_gate * (3.0 - 2.0 * linear_gate);
  }

  const std::uint64_t current_scan = processed_scan_index_;
  const std::uint64_t stale_scan_gap = static_cast<std::uint64_t>(
      20 * g_wall_yaw_extraction_interval_frames);
  auto reset_recapture = [this]() {
    wall_yaw_recapture_state_ = WallYawRecaptureState{};
  };

  if (!observation.plane_valid) {
    observation.recapture_gate_reason = observation.extraction_attempted
        ? "plane_invalid"
        : "not_extracted";
    if (observation.extraction_attempted) {
      reset_recapture();
    } else if (!wall_yaw_recapture_state_.signed_innovations_rad.empty() &&
               current_scan > wall_yaw_recapture_state_.last_frame +
                                  stale_scan_gap) {
      reset_recapture();
    }
    if (!wall_yaw_reference_samples_.empty() &&
        current_scan >
            wall_yaw_reference_samples_.back().frame + stale_scan_gap) {
      wall_yaw_reference_samples_.clear();
    }
    return;
  }

  V3 normal_world = pose.R_ * observation.normal_body;
  normal_world.z() = 0.0;
  const scalar horizontal_normal_norm = normal_world.norm();
  if (!normal_world.allFinite() || horizontal_normal_norm <= 1e-6) {
    return;
  }
  normal_world /= horizontal_normal_norm;
  const double observed_angle = std::atan2(normal_world.y(), normal_world.x());

  auto target_for_axis = [&](const double axis_rad) {
    double best_dot = -std::numeric_limits<double>::infinity();
    V3 best_target = V3::UnitX();
    for (int axis_index = 0; axis_index < 4; ++axis_index) {
      const double target_angle = axis_rad + axis_index * M_PI_2;
      const V3 target(
          std::cos(target_angle), std::sin(target_angle), 0.0);
      const double dot = normal_world.dot(target);
      if (dot > best_dot) {
        best_dot = dot;
        best_target = target;
      }
    }
    return best_target;
  };
  auto signed_innovation_to = [&](const V3& target) {
    return std::atan2(
        normal_world.x() * target.y() - normal_world.y() * target.x(),
        std::clamp(static_cast<double>(normal_world.dot(target)), -1.0, 1.0));
  };

  // Every stored reference has already passed a stable multi-frame quorum, or
  // is copied from such a mature parent. Mature history is never evicted: at
  // capacity we stop extending instead of silently forgetting the start area.
  auto append_mature_reference = [&](const WallYawReference& candidate) {
    for (std::size_t index = 0; index < wall_yaw_references_.size(); ++index) {
      const auto& existing = wall_yaw_references_[index];
      const double axis_distance_deg = std::abs(wrap_period_signed(
          candidate.axis_rad - existing.axis_rad, M_PI_2)) * 180.0 / M_PI;
      const double center_distance =
          (candidate.center - existing.center).norm();
      if (axis_distance_deg <= g_wall_yaw_max_innovation_deg &&
          center_distance <= 0.25 * g_wall_yaw_reference_radius_m) {
        return static_cast<int>(index);
      }
    }
    if (wall_yaw_references_.size() >=
        static_cast<std::size_t>(g_wall_yaw_max_references)) {
      if (!wall_yaw_reference_capacity_warning_logged_) {
        wall_yaw_reference_capacity_warning_logged_ = true;
        LOG(WARNING) << YELLOW
                     << " ---> [SuperLIO]: 墙面航向成熟锚已达容量，"
                     << "保留历史锚并停止新增。capacity="
                     << g_wall_yaw_max_references << RESET;
      }
      return -1;
    }
    wall_yaw_references_.push_back(candidate);
    return static_cast<int>(wall_yaw_references_.size() - 1);
  };

  struct ReferenceMatch {
    int index = -1;
    double score = std::numeric_limits<double>::infinity();
    double distance = 0.0;
    double innovation_rad = 0.0;
    V3 target = V3::UnitX();
  };

  ReferenceMatch strict_match;
  ReferenceMatch historical_conflict;
  bool nearby_mature_conflict = false;
  int nearest_mature_reference_index = -1;
  double nearest_mature_reference_distance =
      std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < wall_yaw_references_.size(); ++index) {
    const auto& reference = wall_yaw_references_[index];
    const double distance = (pose.t_ - reference.center).norm();
    if (!std::isfinite(distance) || distance > g_wall_yaw_reference_radius_m) {
      continue;
    }
    if (reference.mature && distance < nearest_mature_reference_distance) {
      nearest_mature_reference_index = static_cast<int>(index);
      nearest_mature_reference_distance = distance;
    }
    const V3 target = target_for_axis(reference.axis_rad);
    const double innovation_rad = signed_innovation_to(target);
    const double innovation_deg = std::abs(innovation_rad) * 180.0 / M_PI;
    const double score =
        innovation_deg /
            std::max(g_wall_yaw_recapture_max_innovation_deg, 1e-6) +
        0.25 * distance / g_wall_yaw_reference_radius_m;
    if (innovation_deg <= g_wall_yaw_max_innovation_deg) {
      if (score < strict_match.score) {
        strict_match = ReferenceMatch{
            static_cast<int>(index), score, distance, innovation_rad, target};
      }
      continue;
    }
    if (!reference.mature) {
      continue;
    }
    nearby_mature_conflict = true;
    if (innovation_deg > g_wall_yaw_recapture_max_innovation_deg) {
      continue;
    }
    const bool older_lineage = historical_conflict.index < 0 ||
        reference.created_frame < wall_yaw_references_[
            static_cast<std::size_t>(historical_conflict.index)].created_frame;
    const bool same_lineage_age = historical_conflict.index >= 0 &&
        reference.created_frame == wall_yaw_references_[
            static_cast<std::size_t>(historical_conflict.index)].created_frame;
    if (older_lineage || (same_lineage_age && score < historical_conflict.score)) {
      historical_conflict = ReferenceMatch{
          static_cast<int>(index), score, distance, innovation_rad, target};
    }
  }

  // A newer drift-derived chain can otherwise shadow the protected start
  // anchor with a strict match on return. Let a substantially closer, older
  // mature lineage enter the conservative re-capture path instead.
  const bool prefer_historical_conflict =
      historical_conflict.index >= 0 &&
      (strict_match.index < 0 ||
       (historical_conflict.distance <=
            0.5 * g_wall_yaw_reference_radius_m &&
        wall_yaw_references_[static_cast<std::size_t>(
            historical_conflict.index)].created_frame <
            wall_yaw_references_[static_cast<std::size_t>(
                strict_match.index)].created_frame));

  if (strict_match.index >= 0 && !prefer_historical_conflict) {
    reset_recapture();
    observation.recapture_gate_reason = "strict_reference_match";
    auto& reference =
        wall_yaw_references_[static_cast<std::size_t>(strict_match.index)];
    reference.last_used_frame = current_scan;
    ++reference.accepted_count;
    wall_yaw_reference_samples_.clear();
    observation.reference_index = strict_match.index;
    observation.reference_valid = true;
    observation.target_normal_world = strict_match.target;
    observation.innovation_deg =
        std::abs(strict_match.innovation_rad) * 180.0 / M_PI;
    observation.valid = true;

    // A fixed local radius is safe around turns and independently oriented
    // buildings, but by itself leaves a long degenerate corridor uncovered
    // after the robot walks out of the first radius. Once the same wall axis
    // is observed near the edge, copy the already-verified axis to a new
    // local anchor. Never derive the copied axis from the weak current pose.
    const double extension_distance =
        g_wall_yaw_reference_extension_ratio * g_wall_yaw_reference_radius_m;
    if (strict_match.distance >= extension_distance) {
      WallYawReference extension;
      extension.axis_rad = reference.axis_rad;
      extension.center = pose.t_;
      extension.created_frame = reference.created_frame;
      extension.last_used_frame = current_scan;
      extension.accepted_count = 1;
      extension.mature = reference.mature;
      const std::size_t previous_size = wall_yaw_references_.size();
      const int extension_index = append_mature_reference(extension);
      if (extension_index >= 0) {
        observation.reference_index = extension_index;
        if (wall_yaw_references_.size() > previous_size) {
          LOG(INFO) << GREEN
                    << " ---> [SuperLIO]: 续接长走廊墙面航向成熟锚点。index="
                    << observation.reference_index
                    << " axis=" << extension.axis_rad * 180.0 / M_PI
                    << "deg center=" << extension.center.transpose()
                    << " source_distance=" << strict_match.distance
                    << "m total_references=" << wall_yaw_references_.size()
                    << RESET;
        }
      }
    }
    return;
  }

  if (historical_conflict.index >= 0) {
    wall_yaw_reference_samples_.clear();
    observation.reference_index = historical_conflict.index;
    observation.reference_valid = true;
    observation.target_normal_world = historical_conflict.target;
    observation.innovation_deg =
        std::abs(historical_conflict.innovation_rad) * 180.0 / M_PI;

    // Re-capture is an independent revisit channel. The conditional Hessian
    // still controls ordinary wall assistance, but it only describes local
    // incremental observability and cannot disprove accumulated global yaw
    // drift after a long route. Strong local yaw may therefore contribute
    // evidence here, provided the revisit is tightly tied to one old mature
    // anchor and the wall scene remains high quality and temporally stable.
    const auto& conflict_reference = wall_yaw_references_[
        static_cast<std::size_t>(historical_conflict.index)];
    const double recapture_core_radius =
        g_wall_yaw_recapture_core_radius_ratio *
        g_wall_yaw_reference_radius_m;
    const std::uint64_t reference_age = current_scan >=
            conflict_reference.created_frame
        ? current_scan - conflict_reference.created_frame
        : 0U;
    observation.recapture_reference_distance_m =
        historical_conflict.distance;
    observation.recapture_core_radius_m = recapture_core_radius;
    observation.recapture_reference_age_frames = reference_age;
    if (historical_conflict.distance > recapture_core_radius) {
      reset_recapture();
      observation.recapture_gate_reason = "outside_anchor_core";
      return;
    }
    if (nearest_mature_reference_index != historical_conflict.index) {
      reset_recapture();
      observation.recapture_gate_reason = "not_nearest_mature_anchor";
      return;
    }
    if (reference_age < static_cast<std::uint64_t>(
            g_wall_yaw_recapture_min_reference_age_frames)) {
      reset_recapture();
      observation.recapture_gate_reason = "reference_too_young";
      return;
    }
    if (observation.recapture_scene_quality <
        g_wall_yaw_recapture_min_scene_quality) {
      reset_recapture();
      observation.recapture_gate_reason = "scene_quality_low";
      return;
    }
    observation.recapture_constraint_gate = std::clamp(
        observation.recapture_scene_quality, 0.0, 1.0);

    const std::uint64_t max_candidate_gap = static_cast<std::uint64_t>(
        2 * g_wall_yaw_extraction_interval_frames);
    const bool candidate_gap =
        !wall_yaw_recapture_state_.signed_innovations_rad.empty() &&
        current_scan >
            wall_yaw_recapture_state_.last_frame + max_candidate_gap;
    const int previous_recapture_reference =
        wall_yaw_recapture_state_.reference_index;
    std::string pending_reason = "quorum_pending";
    if (candidate_gap) {
      reset_recapture();
      pending_reason = "candidate_gap_restart";
    }
    if (wall_yaw_recapture_state_.reference_index !=
        historical_conflict.index) {
      const bool switched_candidate =
          previous_recapture_reference >= 0 &&
          previous_recapture_reference != historical_conflict.index;
      reset_recapture();
      wall_yaw_recapture_state_.reference_index = historical_conflict.index;
      if (!candidate_gap) {
        pending_reason = switched_candidate
            ? "anchor_switch_restart"
            : "quorum_started";
      }
    }
    if (!wall_yaw_recapture_state_.signed_innovations_rad.empty()) {
      const double mean_innovation = std::accumulate(
          wall_yaw_recapture_state_.signed_innovations_rad.begin(),
          wall_yaw_recapture_state_.signed_innovations_rad.end(),
          0.0) /
          static_cast<double>(
              wall_yaw_recapture_state_.signed_innovations_rad.size());
      const double deviation_deg = std::abs(
          historical_conflict.innovation_rad - mean_innovation) *
          180.0 / M_PI;
      const bool initial_convergence =
          wall_yaw_recapture_state_.signed_innovations_rad.size() <
          static_cast<std::size_t>(
              std::max(1, g_wall_yaw_recapture_min_frames / 2));
      const double allowed_deviation_deg = initial_convergence
          ? g_wall_yaw_recapture_initial_max_deviation_deg
          : g_wall_yaw_reference_max_deviation_deg;
      const double sign_guard_rad = 0.25 * M_PI / 180.0;
      const bool sign_flip =
          std::abs(mean_innovation) > sign_guard_rad &&
          std::abs(historical_conflict.innovation_rad) > sign_guard_rad &&
          mean_innovation * historical_conflict.innovation_rad < 0.0;
      if (sign_flip || deviation_deg > allowed_deviation_deg) {
        reset_recapture();
        wall_yaw_recapture_state_.reference_index = historical_conflict.index;
        pending_reason = sign_flip
            ? "innovation_sign_restart"
            : "innovation_jump_restart";
      }
    }
    wall_yaw_recapture_state_.signed_innovations_rad.push_back(
        historical_conflict.innovation_rad);
    wall_yaw_recapture_state_.last_frame = current_scan;
    while (wall_yaw_recapture_state_.signed_innovations_rad.size() >
           static_cast<std::size_t>(g_wall_yaw_recapture_min_frames)) {
      wall_yaw_recapture_state_.signed_innovations_rad.pop_front();
    }
    observation.recapture_sample_count = static_cast<int>(
        wall_yaw_recapture_state_.signed_innovations_rad.size());
    if (observation.recapture_sample_count <
        g_wall_yaw_recapture_min_frames) {
      observation.recapture_gate_reason = pending_reason;
      return;
    }

    observation.valid = true;
    observation.recapture = true;
    observation.recapture_gate_reason = "active";
    auto& reference = wall_yaw_references_[static_cast<std::size_t>(
        historical_conflict.index)];
    reference.last_used_frame = current_scan;
    ++reference.accepted_count;
    return;
  }

  // Any nearby mature reference outside the re-capture angle is an ambiguous
  // protected region. Fail closed: neither pull nor learn a replacement from
  // the current pose until the robot leaves that local support.
  if (nearby_mature_conflict) {
    wall_yaw_reference_samples_.clear();
    reset_recapture();
    observation.recapture_gate_reason = "innovation_outside_recapture_band";
    return;
  }
  reset_recapture();
  observation.recapture_gate_reason = "no_mature_conflict";

  // A new local reference may only be learned while normal LiDAR matching has
  // sufficient conditional yaw information. In a degenerate corridor we use
  // an existing reference, but never turn the drifting pose into a new truth.
  if (observation.lidar_conditional_yaw_information_ratio <
      g_wall_yaw_reference_min_yaw_information_ratio) {
    if (!wall_yaw_reference_samples_.empty() &&
        current_scan >
            wall_yaw_reference_samples_.back().frame + stale_scan_gap) {
      wall_yaw_reference_samples_.clear();
    }
    return;
  }

  if (!wall_yaw_reference_samples_.empty()) {
    std::deque<double> sample_angles;
    V3 sample_center = V3::Zero();
    for (const auto& sample : wall_yaw_reference_samples_) {
      sample_angles.push_back(sample.axis_rad);
      sample_center += sample.position;
    }
    sample_center /= static_cast<scalar>(wall_yaw_reference_samples_.size());
    const double candidate_axis = mean_manhattan_axis(sample_angles);
    const double deviation_deg = std::abs(wrap_period_signed(
        observed_angle - candidate_axis, M_PI_2)) * 180.0 / M_PI;
    const double spatial_deviation = (pose.t_ - sample_center).norm();
    const bool stale =
        current_scan >
        wall_yaw_reference_samples_.back().frame + stale_scan_gap;
    if (stale ||
        spatial_deviation > 0.5 * g_wall_yaw_reference_radius_m) {
      wall_yaw_reference_samples_.clear();
    } else if (deviation_deg >
               g_wall_yaw_reference_max_deviation_deg) {
      // The largest plane can briefly switch from a building wall to a
      // cabinet, escalator side or vehicle. Ignore that observation instead
      // of letting one outlier erase an otherwise stable local direction.
      return;
    }
  }

  wall_yaw_reference_samples_.push_back(
      WallYawReferenceSample{observed_angle, pose.t_, current_scan});
  if (wall_yaw_reference_samples_.size() <
      static_cast<std::size_t>(g_wall_yaw_reference_min_frames)) {
    return;
  }

  std::deque<double> sample_angles;
  V3 reference_center = V3::Zero();
  for (const auto& sample : wall_yaw_reference_samples_) {
    sample_angles.push_back(sample.axis_rad);
    reference_center += sample.position;
  }
  reference_center /= static_cast<scalar>(wall_yaw_reference_samples_.size());
  WallYawReference reference;
  reference.axis_rad = mean_manhattan_axis(sample_angles);
  reference.center = reference_center;
  reference.created_frame = current_scan;
  reference.last_used_frame = current_scan;
  reference.accepted_count = 1;
  reference.mature = true;
  const int reference_index = append_mature_reference(reference);
  wall_yaw_reference_samples_.clear();
  if (reference_index < 0) {
    return;
  }

  observation.reference_index = reference_index;
  observation.reference_valid = true;
  observation.target_normal_world = target_for_axis(
      wall_yaw_references_[static_cast<std::size_t>(reference_index)].axis_rad);
  const double innovation_rad =
      signed_innovation_to(observation.target_normal_world);
  observation.innovation_deg = std::abs(innovation_rad) * 180.0 / M_PI;
  observation.valid =
      observation.innovation_deg <= g_wall_yaw_max_innovation_deg;

  LOG(INFO) << GREEN
            << " ---> [SuperLIO]: 新建局部墙面航向参考。index="
            << observation.reference_index
            << " axis=" << reference.axis_rad * 180.0 / M_PI
            << "deg center=" << reference.center.transpose()
            << " raw_yaw_ratio="
            << observation.lidar_raw_yaw_information_ratio
            << " conditional_yaw_ratio="
            << observation.lidar_conditional_yaw_information_ratio
            << " translation_ratio="
            << observation.lidar_translation_information_ratio
            << " total_references=" << wall_yaw_references_.size()
            << RESET;
}


struct ThreadACC{
  M6d HTVH = M6d::Zero();
  V6d HTVr = V6d::Zero();
  ThreadACC(): HTVH(M6d::Zero()), HTVr(V6d::Zero()) {}
};


void SuperLIO::Observe(){
  // 迭代观测会原地修改名义状态，因此保留 IMU 预测状态，避免错误匹配污染后续帧。
  const SysState predicted_state = kf_->GetSysState();
  const ESKF::COV predicted_covariance = kf_->GetCov();
  latest_prediction_pose_ = predicted_state.GetSE3();
  latest_prediction_pose_valid_ =
      latest_prediction_pose_.t_.allFinite() &&
      latest_prediction_pose_.R_.allFinite();
  const LevelPlaneObservation level_observation = estimateLevelPlane();
  WallYawObservation wall_yaw_observation = estimateWallYaw();
  // Wall-reference learning happens while UpdateObserve iterates, before the
  // frame-level match/motion checks below. Preserve the small wall state so a
  // rejected scan cannot advance a quorum, create/extend an anchor, or advance
  // a re-capture sequence whose pose is subsequently rolled back.
  auto wall_yaw_reference_samples_before = wall_yaw_reference_samples_;
  auto wall_yaw_references_before = wall_yaw_references_;
  auto wall_yaw_recapture_state_before = wall_yaw_recapture_state_;
  const bool wall_yaw_reference_capacity_warning_before =
      wall_yaw_reference_capacity_warning_logged_;

  const double inlier_confidence = level_observation.valid
      ? std::clamp(
          static_cast<double>(level_observation.inlier_count) /
              static_cast<double>(2 * g_level_min_plane_inliers),
          0.25, 1.0)
      : 0.0;
  const double ratio_confidence = level_observation.valid
      ? std::clamp(
          level_observation.inlier_ratio /
              (2.0 * g_level_min_plane_inlier_ratio),
          0.25, 1.0)
      : 0.0;
  const double rms_confidence = level_observation.valid
      ? std::clamp(
          1.0 - level_observation.rms /
              g_level_plane_distance_threshold,
          0.25, 1.0)
      : 0.0;
  const double dual_slope_evidence_angle_deg = level_observation.plane_valid
      ? std::min(
          level_observation.gravity_angle_deg,
          level_observation.innovation_deg)
      : 0.0;
  const double gravity_confidence = level_observation.valid
      ? std::clamp(
          1.0 - dual_slope_evidence_angle_deg /
              g_level_max_plane_gravity_angle_deg,
          0.25, 1.0)
      : 0.0;
  double level_slope_gate = 0.0;
  if (level_observation.valid) {
    // A flat plane under linear acceleration can disagree with the short IMU
    // gravity window while already being nearly horizontal in the world. Use
    // the weaker of the two independent slope signals: only dual evidence may
    // fade the level prior or advance the latched slope-protection state.
    if (dual_slope_evidence_angle_deg <=
        g_level_slope_soft_start_angle_deg) {
      level_slope_gate = 1.0;
    } else {
      const double linear_gate = std::clamp(
          (g_level_max_plane_gravity_angle_deg -
           dual_slope_evidence_angle_deg) /
              std::max(
                  g_level_max_plane_gravity_angle_deg -
                      g_level_slope_soft_start_angle_deg,
                  1e-6),
          0.0,
          1.0);
      level_slope_gate =
          linear_gate * linear_gate * (3.0 - 2.0 * linear_gate);
    }
  }
  // The first three committed planes after leaving slope protection restore
  // the strong 0.015-degree prior gradually. Include this frame tentatively in
  // its gate; a frame rejected below is rolled back and does not advance the
  // committed recovery counter.
  double level_slope_recovery_gate = 1.0;
  if (level_slope_recovery_active_) {
    level_slope_recovery_gate = level_observation.valid
        ? std::clamp(
            static_cast<double>(level_slope_recovery_count_ + 1) /
                static_cast<double>(g_level_slope_recovery_min_frames),
            0.0,
            1.0)
        : 0.0;
  }
  const double level_effective_gate =
      level_slope_gate * level_slope_recovery_gate;
  const double level_confidence = level_observation.valid
      ? std::clamp(
          0.25 * (inlier_confidence + ratio_confidence +
                  rms_confidence + gravity_confidence),
          0.25, 1.0)
      : 0.0;
  const double level_sigma_rad =
      g_level_attitude_stddev_deg * M_PI / 180.0;
  const double level_information = level_observation.valid
      ? level_confidence * level_effective_gate /
            (level_sigma_rad * level_sigma_rad)
      : 0.0;

  // Build one fixed, bounded world-Z correction from the previous committed
  // ground plane.  The reference plane predicts the ground elevation at the
  // current XY position, while the current plane supplies the instantaneous
  // sensor-to-ground height.  Body bob therefore cancels and a real ramp is
  // followed instead of being flattened. It is deliberately applied after
  // the joint ESKF update, where changing state.p.z() cannot leak through
  // cross-covariance into XY, attitude, velocity or IMU biases.
  bool ground_height_continuity_valid = false;
  std::uint64_t ground_height_reference_gap = 0;
  double ground_height_horizontal_step = 0.0;
  double ground_height_normal_difference_deg = 0.0;
  double ground_height_predicted_ground_z = 0.0;
  double ground_height_expected_ground_z = 0.0;
  double ground_height_innovation = 0.0;
  double ground_height_bounded_correction = 0.0;
  double ground_height_information = 0.0;
  double ground_height_direct_correction = 0.0;
  bool ground_height_budget_limited = false;
  const bool ground_height_reference_available =
      ground_height_continuity_reference_.valid;
  if (g_ground_height_continuity_enable &&
      level_observation.plane_valid &&
      ground_height_reference_available &&
      latest_prediction_pose_valid_ &&
      processed_scan_index_ >
          ground_height_continuity_reference_.scan_index) {
    ground_height_reference_gap = processed_scan_index_ -
        ground_height_continuity_reference_.scan_index;
    const SE3 height_prediction_pose = latest_prediction_pose_;
    V3 current_ground_normal_world =
        height_prediction_pose.R_ * level_observation.normal_body;
    double current_plane_offset_body =
        level_observation.plane_offset_body;
    const scalar current_normal_norm =
        current_ground_normal_world.norm();
    if (current_ground_normal_world.allFinite() &&
        std::isfinite(static_cast<double>(current_normal_norm)) &&
        current_normal_norm > 1.0e-6) {
      current_ground_normal_world /= current_normal_norm;
      if (current_ground_normal_world.z() < 0.0) {
        current_ground_normal_world = -current_ground_normal_world;
        current_plane_offset_body = -current_plane_offset_body;
      }
      const V3 frame_delta = height_prediction_pose.t_ -
          ground_height_continuity_reference_.sensor_position_world;
      ground_height_horizontal_step = frame_delta.head<2>().norm();
      ground_height_normal_difference_deg = std::acos(std::clamp(
          static_cast<double>(current_ground_normal_world.dot(
              ground_height_continuity_reference_.normal_world)),
          -1.0, 1.0)) * 180.0 / M_PI;
      const double previous_normal_z =
          ground_height_continuity_reference_.normal_world.z();
      const double current_normal_z = current_ground_normal_world.z();
      if (ground_height_reference_gap <=
              static_cast<std::uint64_t>(
                  g_ground_height_continuity_max_frame_gap) &&
          ground_height_horizontal_step <=
              g_ground_height_continuity_max_horizontal_step_m &&
          ground_height_normal_difference_deg <=
              g_ground_height_continuity_max_normal_difference_deg &&
          previous_normal_z >= 0.5 && current_normal_z >= 0.5) {
        ground_height_predicted_ground_z =
            height_prediction_pose.t_.z() -
            current_plane_offset_body / current_normal_z;
        ground_height_expected_ground_z =
            ground_height_continuity_reference_.ground_height_world -
            (ground_height_continuity_reference_.normal_world.x() *
                 frame_delta.x() +
             ground_height_continuity_reference_.normal_world.y() *
                 frame_delta.y()) /
                previous_normal_z;
        ground_height_innovation =
            ground_height_expected_ground_z -
            ground_height_predicted_ground_z;
        if (std::isfinite(ground_height_innovation) &&
            std::abs(ground_height_innovation) <=
                g_ground_height_continuity_max_innovation_m) {
          ground_height_bounded_correction = std::clamp(
              ground_height_innovation,
              -g_ground_height_continuity_max_correction_per_frame_m,
              g_ground_height_continuity_max_correction_per_frame_m);
          const double height_inlier_confidence = std::clamp(
              static_cast<double>(level_observation.inlier_count) /
                  static_cast<double>(2 * g_level_min_plane_inliers),
              0.25, 1.0);
          const double height_ratio_confidence = std::clamp(
              level_observation.inlier_ratio /
                  (2.0 * g_level_min_plane_inlier_ratio),
              0.25, 1.0);
          const double height_rms_confidence = std::clamp(
              1.0 - level_observation.rms /
                  g_level_plane_distance_threshold,
              0.25, 1.0);
          const double height_confidence = std::clamp(
              (height_inlier_confidence + height_ratio_confidence +
               height_rms_confidence) / 3.0,
              0.25, 1.0);
          ground_height_information = height_confidence /
              (g_ground_height_continuity_stddev_m *
               g_ground_height_continuity_stddev_m);
          ground_height_continuity_valid =
              std::isfinite(ground_height_bounded_correction) &&
              std::isfinite(ground_height_information) &&
              ground_height_information > 0.0;
        }
      }
    }
  }
  const double wall_inlier_confidence = wall_yaw_observation.plane_valid
      ? std::clamp(
          static_cast<double>(wall_yaw_observation.inlier_count) /
              static_cast<double>(2 * g_wall_yaw_min_plane_inliers),
          0.25, 1.0)
      : 0.0;
  const double wall_ratio_confidence = wall_yaw_observation.plane_valid
      ? std::clamp(
          wall_yaw_observation.inlier_ratio /
              (2.0 * g_wall_yaw_min_plane_inlier_ratio),
          0.25, 1.0)
      : 0.0;
  const double wall_rms_confidence = wall_yaw_observation.plane_valid
      ? std::clamp(
          1.0 - wall_yaw_observation.rms /
              g_wall_yaw_plane_distance_threshold,
          0.25, 1.0)
      : 0.0;
  const double wall_vertical_confidence = wall_yaw_observation.plane_valid
      ? std::clamp(
          1.0 - wall_yaw_observation.vertical_angle_deg /
              g_wall_yaw_max_vertical_angle_deg,
          0.25, 1.0)
      : 0.0;
  const double wall_span_confidence = wall_yaw_observation.plane_valid
      ? 0.5 * (
          std::clamp(
              wall_yaw_observation.vertical_span /
                  (2.0 * g_wall_yaw_min_vertical_span),
              0.25, 1.0) +
          std::clamp(
              wall_yaw_observation.horizontal_span /
                  (2.0 * g_wall_yaw_min_horizontal_span),
              0.25, 1.0))
      : 0.0;
  const double wall_yaw_confidence = wall_yaw_observation.plane_valid
      ? std::clamp(
          0.2 * (wall_inlier_confidence + wall_ratio_confidence +
                 wall_rms_confidence + wall_vertical_confidence +
                 wall_span_confidence),
          0.25, 1.0)
      : 0.0;
  double wall_yaw_information = 0.0;
  double wall_yaw_effective_constraint_gate = 0.0;
  bool wall_yaw_prepared = false;
  bool wall_yaw_bounded_target_prepared = false;
  V3d wall_yaw_bounded_target_world = V3d::UnitX();
  size_t ptsize = ds_undistort_->size();
  
  static std::vector<float> _lengths;
  points_body_v3_.resize(ptsize);
  _lengths.resize(ptsize);
  if (effect_knn_idxs_.size() < ptsize) {
    effect_knn_idxs_.resize(ptsize);
  }
  if (abcd_vec_.size() < ptsize) {
    abcd_vec_.resize(ptsize);
  }
  if (effect_mask_.size() < ptsize) {
    effect_mask_.resize(ptsize, false);
  }
  if (effect_knn_mask_.size() < ptsize) {
    effect_knn_mask_.resize(ptsize, false);
  }

  effect_knn_num_ = ptsize;
  std::iota(effect_knn_idxs_.begin(), effect_knn_idxs_.begin() + ptsize, 0);

  for(size_t i = 0; i < ptsize; ++i){
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    _lengths[i] = points_body_v3_[i].norm();
  }

  ivox_->reset_max_group();
  int iter_num = 0;
  V3d lidar_rotation_information_eigenvalues = V3d::Zero();
  V3d lidar_translation_information_eigenvalues = V3d::Zero();
  double lidar_raw_yaw_information_ratio = 0.0;
  double lidar_conditional_yaw_information_ratio = 0.0;
  double lidar_translation_information_ratio = 0.0;

  kf_->UpdateObserve([&, this](const ESKF::KFState &kf_state, M6 &HTVH, V6 &HTVr) {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();

    tbb::enumerable_thread_specific<ThreadACC> tls_acc;

    tbb::parallel_for(
      tbb::blocked_range<size_t>(0, effect_knn_num_),
      [&](const tbb::blocked_range<size_t>& r) {
        KNNHeapType top_K;
        auto& local_acc = tls_acc.local();
        for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
          int idx = effect_knn_idxs_[r_s];
          V3& point_body = points_body_v3_[idx];
          V3 point_world = pose * point_body;

          if(!need_converge){
            top_K.reset();
            ivox_->getTopK(point_world, top_K);
            if(top_K.count < 4){
              effect_mask_[idx] = false;
              effect_knn_mask_[idx] = false;
              continue;
            }
            effect_knn_mask_[idx] = true;
            effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
          }

          if(!effect_mask_[idx]) continue;

          auto& abcd = abcd_vec_[idx];
          scalar error;
          effect_mask_[idx] = compute_error(abcd, point_world, _lengths[idx], error);
          if(!effect_mask_[idx]) continue;
          
          {
            V3d normvec(abcd[0], abcd[1], abcd[2]);
            V3d nb = R_transpose * normvec;
            V3d point_body_d = point_body.cast<double>();
            V6d J;
            J.head<3>() = point_body_d.cross(nb);
            J.tail<3>() = normvec;

            local_acc.HTVH += J * 1000 * J.transpose();
            local_acc.HTVr -= J * 1000 * error;
          }
        }
    });

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    for(const auto& local_acc : tls_acc){
      sum_HTVH += local_acc.HTVH;
      sum_HTVr += local_acc.HTVr;
    }

    Eigen::SelfAdjointEigenSolver<M3d> lidar_information_solver(
        sum_HTVH.block<3, 3>(0, 0));
    if (lidar_information_solver.info() == Eigen::Success) {
      lidar_rotation_information_eigenvalues =
          lidar_information_solver.eigenvalues();
    }
    Eigen::SelfAdjointEigenSolver<M3d> lidar_translation_information_solver(
        sum_HTVH.block<3, 3>(3, 3));
    if (lidar_translation_information_solver.info() == Eigen::Success) {
      lidar_translation_information_eigenvalues =
          lidar_translation_information_solver.eigenvalues();
    }

    const M3d rotation = pose.R_.cast<double>();
    V3d yaw_axis_body = rotation.transpose() * V3d::UnitZ();
    yaw_axis_body.normalize();
    // Evaluate the pure LiDAR 6-D Hessian before level/wall priors are added.
    // A large H(yaw,yaw) alone is not independent yaw evidence when it can be
    // cancelled by an unobservable translation or roll/pitch direction.
    const LidarPoseObservability lidar_observability =
        evaluate_lidar_pose_observability(
            sum_HTVH,
            yaw_axis_body,
            lidar_rotation_information_eigenvalues,
            lidar_translation_information_eigenvalues);
    lidar_raw_yaw_information_ratio = lidar_observability.raw_yaw_ratio;
    lidar_conditional_yaw_information_ratio =
        lidar_observability.conditional_yaw_ratio;
    lidar_translation_information_ratio =
        lidar_observability.translation_ratio;
    if (!wall_yaw_prepared) {
      prepareWallYawConstraint(
          wall_yaw_observation,
          pose,
          lidar_raw_yaw_information_ratio,
          lidar_conditional_yaw_information_ratio,
          lidar_translation_information_ratio);
      wall_yaw_prepared = true;
    }
    const double wall_yaw_sigma_deg = wall_yaw_observation.recapture
        ? g_wall_yaw_recapture_stddev_deg
        : g_wall_yaw_stddev_deg;
    const double wall_yaw_sigma_rad =
        std::max(wall_yaw_sigma_deg, 1e-6) * M_PI / 180.0;
    wall_yaw_effective_constraint_gate = wall_yaw_observation.recapture
        ? wall_yaw_observation.recapture_constraint_gate
        : wall_yaw_observation.observability_gate;
    wall_yaw_information = wall_yaw_observation.valid
        ? wall_yaw_confidence /
              (wall_yaw_sigma_rad * wall_yaw_sigma_rad) *
              wall_yaw_effective_constraint_gate
        : 0.0;

    if (wall_yaw_observation.valid &&
        !wall_yaw_bounded_target_prepared) {
      V3d frame_start_normal_world =
          rotation * wall_yaw_observation.normal_body.cast<double>();
      frame_start_normal_world.z() = 0.0;
      const double frame_start_normal_norm =
          frame_start_normal_world.norm();
      if (frame_start_normal_world.allFinite() &&
          std::isfinite(frame_start_normal_norm) &&
          frame_start_normal_norm > 1e-6) {
        frame_start_normal_world /= frame_start_normal_norm;
        const V3d full_target_normal_world =
            wall_yaw_observation.target_normal_world.cast<double>();
        const double full_target_residual = std::atan2(
            frame_start_normal_world.x() * full_target_normal_world.y() -
                frame_start_normal_world.y() * full_target_normal_world.x(),
            std::clamp(
                frame_start_normal_world.dot(full_target_normal_world),
                -1.0,
                1.0));
        const double frame_limit_deg = wall_yaw_observation.recapture
            ? g_wall_yaw_recapture_max_correction_per_frame_deg
            : g_wall_yaw_max_correction_per_frame_deg;
        const double bounded_target_delta = std::clamp(
            full_target_residual,
            -frame_limit_deg * M_PI / 180.0,
            frame_limit_deg * M_PI / 180.0);
        wall_yaw_bounded_target_world =
            Eigen::AngleAxisd(bounded_target_delta, V3d::UnitZ()) *
            frame_start_normal_world;
        wall_yaw_observation.frame_target_delta_deg =
            bounded_target_delta * 180.0 / M_PI;
        wall_yaw_bounded_target_prepared = true;
      }
    }

    if (level_observation.valid) {
      // z 为世界竖直方向，h(R)=R*n_body。右乘姿态误差下
      // dh/d(delta_theta)=-R*hat(n_body)。该雅可比秩为二，只增加
      // roll/pitch 信息，不观测 yaw、位置或高度。
      const V3d normal_body = level_observation.normal_body.cast<double>();
      const V3d predicted_up = rotation * normal_body;
      const V3d residual = V3d::UnitZ() - predicted_up;
      const M3d H = -rotation *
          SO3::hat(level_observation.normal_body).cast<double>();
      sum_HTVH.block<3, 3>(0, 0).noalias() +=
          level_information * H.transpose() * H;
      sum_HTVr.head<3>().noalias() +=
          level_information * H.transpose() * residual;
    }
    if (wall_yaw_observation.valid && wall_yaw_information > 0.0 &&
        wall_yaw_bounded_target_prepared) {
      V3d normal_world =
          rotation * wall_yaw_observation.normal_body.cast<double>();
      normal_world.z() = 0.0;
      const double normal_norm = normal_world.norm();
      if (std::isfinite(normal_norm) && normal_norm > 1e-6) {
        normal_world /= normal_norm;
        // The first iteration fixes one bounded world-frame target. Later ESKF
        // iterations converge to that same target; they do not consume the
        // per-frame yaw allowance repeatedly.
        const double residual = std::atan2(
            normal_world.x() * wall_yaw_bounded_target_world.y() -
                normal_world.y() * wall_yaw_bounded_target_world.x(),
            std::clamp(
                normal_world.dot(wall_yaw_bounded_target_world), -1.0, 1.0));

        // 右乘误差中 R^T*z_world 对应绕世界竖直轴的纯旋转。
        // 该 rank-1 观测不会向 XY、Z、roll 或 pitch 注入信息。
        sum_HTVH.block<3, 3>(0, 0).noalias() +=
            wall_yaw_information *
            yaw_axis_body * yaw_axis_body.transpose();
        sum_HTVr.head<3>().noalias() +=
            wall_yaw_information * yaw_axis_body * residual;
      }
    }
    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();

    if(need_converge) return;

    int _effect_knn_num = 0;
    for(size_t i = 0; i < effect_knn_num_; ++i){
      int idx = effect_knn_idxs_[i];
      if(!effect_knn_mask_[idx]) continue;
      effect_knn_idxs_[_effect_knn_num] = idx;
      _effect_knn_num++;
    }

    // LOG(INFO) << "effect_knn_num_: " << effect_knn_num_ << ", _effect_knn_num: " << _effect_knn_num;
    effect_knn_num_ = _effect_knn_num;

    iter_num++;
  });

  // Apply the optional height aid outside the joint Kalman normal equations.
  // Only p.z is changed; covariance and every other state component remain
  // exactly as produced by the normal LiDAR/IMU update. The signed session
  // budget prevents a biased local plane chain from creating metres of drift.
  if (ground_height_continuity_valid) {
    const double remaining_lower =
        -g_ground_height_continuity_max_total_correction_m -
        ground_height_continuity_applied_offset_m_;
    const double remaining_upper =
        g_ground_height_continuity_max_total_correction_m -
        ground_height_continuity_applied_offset_m_;
    ground_height_direct_correction = std::clamp(
        ground_height_bounded_correction,
        remaining_lower,
        remaining_upper);
    ground_height_budget_limited =
        std::abs(ground_height_direct_correction -
                 ground_height_bounded_correction) > 1.0e-9;
    if (std::abs(ground_height_direct_correction) > 1.0e-9) {
      SysState height_adjusted_state = kf_->GetSysState();
      height_adjusted_state.p.z() += ground_height_direct_correction;
      kf_->SetX(height_adjusted_state);
    }
  }

  effective_match_count_ = 0;
  for (std::size_t i = 0; i < effect_knn_num_; ++i) {
    const int index = effect_knn_idxs_[i];
    if (index >= 0 && static_cast<std::size_t>(index) < effect_mask_.size() &&
        effect_mask_[index]) {
      effective_match_count_++;
    }
  }
  latest_lidar_rotation_information_eigenvalues_ =
      lidar_rotation_information_eigenvalues;
  latest_lidar_translation_information_eigenvalues_ =
      lidar_translation_information_eigenvalues;
  latest_lidar_yaw_information_ratio_ =
      lidar_raw_yaw_information_ratio;
  latest_lidar_conditional_yaw_information_ratio_ =
      lidar_conditional_yaw_information_ratio;
  latest_lidar_translation_information_ratio_ =
      lidar_translation_information_ratio;

  const auto level_log_time = std::chrono::steady_clock::now();
  if (last_level_constraint_log_time_ ==
          std::chrono::steady_clock::time_point{} ||
      level_log_time - last_level_constraint_log_time_ >
          std::chrono::seconds(5)) {
    last_level_constraint_log_time_ = level_log_time;
    const SE3 diagnostic_pose = kf_->GetSE3();
    double post_update_innovation_deg = 0.0;
    if (level_observation.valid) {
      V3 post_update_plane_up =
          diagnostic_pose.R_ * level_observation.normal_body;
      post_update_plane_up.normalize();
      post_update_innovation_deg = std::acos(std::clamp(
          static_cast<double>(post_update_plane_up.dot(V3::UnitZ())),
          -1.0, 1.0)) * 180.0 / M_PI;
    }
    double wall_post_update_innovation_deg = 0.0;
    if (wall_yaw_observation.valid) {
      V3 wall_normal_world =
          diagnostic_pose.R_ * wall_yaw_observation.normal_body;
      wall_normal_world.z() = 0.0;
      const scalar wall_normal_norm = wall_normal_world.norm();
      if (wall_normal_world.allFinite() && wall_normal_norm > 1e-6) {
        wall_normal_world /= wall_normal_norm;
        wall_post_update_innovation_deg = std::abs(std::atan2(
            wall_normal_world.x() *
                    wall_yaw_observation.target_normal_world.y() -
                wall_normal_world.y() *
                    wall_yaw_observation.target_normal_world.x(),
            std::clamp(
                static_cast<double>(wall_normal_world.dot(
                    wall_yaw_observation.target_normal_world)),
                -1.0, 1.0))) * 180.0 / M_PI;
      }
    }
    LOG(INFO) << GREEN
              << " ---> [SuperLIO]: level constraint health. current_valid="
              << level_observation.valid
              << " plane_valid=" << level_observation.plane_valid
              << " accepted=" << level_constraint_accepted_count_
              << " rejected=" << level_constraint_rejected_count_
              << " gated=" << level_constraint_gated_count_
              << " candidates=" << level_observation.candidate_count
              << " inliers=" << level_observation.inlier_count
              << " ratio=" << level_observation.inlier_ratio
              << " rms=" << level_observation.rms
              << " plane_gravity_angle="
              << level_observation.gravity_angle_deg
              << "deg slope_rejected="
              << level_observation.slope_rejected
              << " slope_state_active="
              << level_slope_protection_active_
              << " slope_enter_evidence="
              << level_observation.slope_enter_evidence
              << " slope_exit_evidence="
              << level_observation.slope_exit_evidence
              << " dynamic_gravity_mismatch="
              << level_observation.dynamic_gravity_mismatch
              << " slope_enter_count=" << level_slope_enter_count_
              << "/" << g_level_slope_enter_min_frames
              << " slope_exit_count=" << level_slope_exit_count_
              << "/" << g_level_slope_exit_min_frames
              << " slope_pending_invalid_count="
              << level_slope_pending_invalid_count_
              << "/" << g_level_slope_pending_max_invalid_frames
              << " slope_recovery_active="
              << level_slope_recovery_active_
              << " slope_recovery_count=" << level_slope_recovery_count_
              << "/" << g_level_slope_recovery_min_frames
              << " slope_spatial_path="
              << level_slope_spatial_path_m_
              << "/" << g_level_slope_spatial_window_m
              << "m slope_spatial_support_ratio="
              << level_slope_spatial_last_support_ratio_
              << " slope_spatial_observed_grade="
              << level_slope_spatial_last_observed_grade_deg_
              << "deg slope_spatial_plane_grade="
              << level_slope_spatial_last_expected_grade_deg_
              << "deg slope_spatial_error="
              << level_slope_spatial_last_error_deg_
              << "deg slope_spatial_mismatch_windows="
              << level_slope_spatial_mismatch_count_
              << "/" << g_level_slope_spatial_max_mismatch_windows
              << " slope_spatial_reentry_blocked="
              << level_slope_spatial_reentry_blocked_
              << " slope_spatial_reentry_consistent_windows="
              << level_slope_spatial_reentry_consistent_count_
              << "/"
              << g_level_slope_spatial_reentry_consistent_windows
              << " slope_gate=" << level_slope_gate
              << " recovery_gate=" << level_slope_recovery_gate
              << " effective_gate=" << level_effective_gate
              << " attitude_innovation="
              << level_observation.innovation_deg
              << "deg post_update_innovation="
              << post_update_innovation_deg
              << "deg information=" << level_information
              << " ground_height_valid="
              << ground_height_continuity_valid
              << " ground_height_reference="
              << ground_height_reference_available
              << " ground_height_accepted="
              << ground_height_continuity_accepted_count_
              << " ground_height_rejected="
              << ground_height_continuity_rejected_count_
              << " ground_height_gated="
              << ground_height_continuity_gated_count_
              << " ground_height_budget_gated="
              << ground_height_continuity_budget_gated_count_
              << " ground_height_gap="
              << ground_height_reference_gap
              << " ground_height_horizontal_step="
              << ground_height_horizontal_step
              << "m ground_height_normal_difference="
              << ground_height_normal_difference_deg
              << "deg ground_height_predicted="
              << ground_height_predicted_ground_z
              << "m ground_height_expected="
              << ground_height_expected_ground_z
              << "m ground_height_innovation="
              << ground_height_innovation
              << "m ground_height_bounded_correction="
              << ground_height_bounded_correction
              << "m ground_height_direct_correction="
              << ground_height_direct_correction
              << "m ground_height_applied_offset="
              << ground_height_continuity_applied_offset_m_
              << "/"
              << g_ground_height_continuity_max_total_correction_m
              << "m ground_height_budget_limited="
              << ground_height_budget_limited
              << " ground_height_information="
              << ground_height_information
              << " lidar_rotation_information_eigenvalues="
              << lidar_rotation_information_eigenvalues.transpose()
              << " position=" << diagnostic_pose.t_.transpose() << RESET;
    LOG(INFO) << GREEN
              << " ---> [SuperLIO]: 墙面航向约束健康状态。当前有效="
              << wall_yaw_observation.valid
              << " 平面有效=" << wall_yaw_observation.plane_valid
              << " 参考有效=" << wall_yaw_observation.reference_valid
              << " 接受=" << wall_yaw_constraint_accepted_count_
              << " 拒绝=" << wall_yaw_constraint_rejected_count_
              << " 可观测性门控=" << wall_yaw_constraint_gated_count_
              << " 降频跳过=" << wall_yaw_extraction_skipped_count_
              << " 参考样本=" << wall_yaw_reference_samples_.size()
              << " 局部参考=" << wall_yaw_references_.size()
              << " 当前参考=" << wall_yaw_observation.reference_index
              << " 候选点=" << wall_yaw_observation.candidate_count
              << " 内点=" << wall_yaw_observation.inlier_count
              << " 内点比例=" << wall_yaw_observation.inlier_ratio
              << " RMS=" << wall_yaw_observation.rms
              << " 垂直角=" << wall_yaw_observation.vertical_angle_deg
              << "deg 竖直跨度=" << wall_yaw_observation.vertical_span
              << "m 水平跨度=" << wall_yaw_observation.horizontal_span
              << "m 创新=" << wall_yaw_observation.innovation_deg
              << "deg 更新后创新=" << wall_post_update_innovation_deg
              << "deg 帧目标修正="
              << wall_yaw_observation.frame_target_delta_deg
              << "deg 重捕=" << wall_yaw_observation.recapture
              << " 重捕样本="
              << wall_yaw_observation.recapture_sample_count
              << " 重捕场景质量="
              << wall_yaw_observation.recapture_scene_quality
              << " 重捕锚距="
              << wall_yaw_observation.recapture_reference_distance_m
              << "/" << wall_yaw_observation.recapture_core_radius_m
              << "m 重捕锚龄="
              << wall_yaw_observation.recapture_reference_age_frames
              << " 重捕证据门控="
              << wall_yaw_observation.recapture_constraint_gate
              << " 重捕判定="
              << wall_yaw_observation.recapture_gate_reason
              << " raw_yaw相对信息强度="
              << wall_yaw_observation.lidar_raw_yaw_information_ratio
              << " conditional_yaw相对信息强度="
              << wall_yaw_observation.lidar_conditional_yaw_information_ratio
              << " 平移最弱最强比="
              << wall_yaw_observation.lidar_translation_information_ratio
              << " 普通门控权重="
              << wall_yaw_observation.observability_gate
              << " 有效门控权重="
              << wall_yaw_effective_constraint_gate
              << " 信息量=" << wall_yaw_information << RESET;
  }

  const SE3 current_pose = kf_->GetSE3();
  const SE3 predicted_pose = latest_prediction_pose_;
  const bool pose_finite = current_pose.t_.allFinite() && current_pose.R_.allFinite();
  double frame_translation = 0.0;
  double frame_rotation_deg = 0.0;
  double predicted_translation = 0.0;
  double predicted_rotation_deg = 0.0;
  bool prediction_valid =
      predicted_pose.t_.allFinite() && predicted_pose.R_.allFinite();
  bool motion_valid = pose_finite;
  if (pose_finite && has_last_accepted_pose_) {
    frame_translation = (current_pose.t_ - last_pose_.t_).norm();
    const M3 relative_rotation = last_pose_.R_.transpose() * current_pose.R_;
    frame_rotation_deg = std::abs(
      Eigen::AngleAxis<scalar>(relative_rotation).angle() * 180.0 / M_PI);
    motion_valid = frame_translation <= g_max_frame_translation &&
                   frame_rotation_deg <= g_max_frame_rotation_deg;
  }
  if (prediction_valid && has_last_accepted_pose_) {
    predicted_translation = (predicted_pose.t_ - last_pose_.t_).norm();
    const M3 predicted_relative_rotation =
        last_pose_.R_.transpose() * predicted_pose.R_;
    predicted_rotation_deg = std::abs(
      Eigen::AngleAxis<scalar>(predicted_relative_rotation).angle() * 180.0 / M_PI);
    prediction_valid =
        predicted_translation <= g_max_frame_translation &&
        predicted_rotation_deg <= g_max_frame_rotation_deg;
  }

  observation_valid_ =
    effective_match_count_ >= static_cast<std::size_t>(g_min_effective_points) &&
    motion_valid;
  if (!observation_valid_) {
    wall_yaw_reference_samples_ =
        std::move(wall_yaw_reference_samples_before);
    wall_yaw_references_ = std::move(wall_yaw_references_before);
    wall_yaw_recapture_state_ =
        std::move(wall_yaw_recapture_state_before);
    wall_yaw_reference_capacity_warning_logged_ =
        wall_yaw_reference_capacity_warning_before;
    const char* rollback_mode = "imu_prediction";
    if (prediction_valid || !has_last_accepted_state_) {
      kf_->SetX(predicted_state);
      kf_->SetCov(predicted_covariance);
    } else {
      SysState recovered_state = last_accepted_state_;
      recovered_state.timestamp = predicted_state.timestamp;
      recovered_state.v.setZero();
      kf_->SetX(recovered_state);
      kf_->SetCov(last_accepted_covariance_);
      rollback_mode = "last_accepted_state";
    }
    consecutive_invalid_observations_++;
    if (consecutive_invalid_observations_ == 1 ||
        consecutive_invalid_observations_ % 10 == 0) {
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: reject unsafe map update. effective_matches="
                   << effective_match_count_ << " min=" << g_min_effective_points
                   << " frame_translation=" << frame_translation
                   << "m frame_rotation=" << frame_rotation_deg
                   << "deg predicted_translation=" << predicted_translation
                   << "m predicted_rotation=" << predicted_rotation_deg
                   << "deg finite=" << pose_finite
                   << " state_rollback=" << rollback_mode
                   << " consecutive=" << consecutive_invalid_observations_ << RESET;
    }
  } else if (consecutive_invalid_observations_ > 0) {
    LOG(INFO) << GREEN << " ---> [SuperLIO]: observation recovered after "
              << consecutive_invalid_observations_ << " rejected frames." << RESET;
    consecutive_invalid_observations_ = 0;
  }

  // Slope hysteresis is committed only together with a safe LiDAR frame. A
  // rejected pose update never changes this state. The angular latch protects
  // the start of a real ramp immediately, while a spatial lease below prevents
  // one early plane/attitude error from disabling level stabilization for an
  // arbitrarily long route.
  if (observation_valid_) {
    const bool protection_was_active =
        level_slope_protection_active_;
    const bool recovery_was_active =
        level_slope_recovery_active_;
    const char* slope_transition_reason = "angle_hysteresis";
    if (level_observation.plane_valid) {
      level_slope_pending_invalid_count_ = 0;
      if (level_slope_protection_active_) {
        level_slope_enter_count_ = 0;
        level_slope_recovery_active_ = false;
        level_slope_recovery_count_ = 0;
        if (level_observation.slope_exit_evidence) {
          level_slope_exit_count_ = std::min(
              level_slope_exit_count_ + 1,
              g_level_slope_exit_min_frames);
        } else {
          level_slope_exit_count_ = 0;
        }
        if (level_slope_exit_count_ >=
            g_level_slope_exit_min_frames) {
          level_slope_protection_active_ = false;
          level_slope_exit_count_ = 0;
          level_slope_recovery_active_ = true;
          level_slope_recovery_count_ = 0;
        }
      } else {
        level_slope_exit_count_ = 0;
        if (level_slope_recovery_active_ && level_observation.valid) {
          level_slope_recovery_count_ = std::min(
              level_slope_recovery_count_ + 1,
              g_level_slope_recovery_min_frames);
          if (level_slope_recovery_count_ >=
              g_level_slope_recovery_min_frames) {
            level_slope_recovery_active_ = false;
          }
        }
        if (level_observation.slope_enter_evidence &&
            !level_slope_spatial_reentry_blocked_) {
          level_slope_enter_count_ = std::min(
              level_slope_enter_count_ + 1,
              g_level_slope_enter_min_frames);
        } else {
          level_slope_enter_count_ = 0;
        }
        if (level_slope_enter_count_ >=
            g_level_slope_enter_min_frames) {
          level_slope_protection_active_ = true;
          level_slope_enter_count_ = 0;
          level_slope_recovery_active_ = false;
          level_slope_recovery_count_ = 0;
        }
      }
    } else if (level_slope_enter_count_ > 0 ||
               level_slope_exit_count_ > 0) {
      ++level_slope_pending_invalid_count_;
      if (level_slope_pending_invalid_count_ >
          g_level_slope_pending_max_invalid_frames) {
        LOG(INFO) << GREEN
                  << " ---> [SuperLIO]: clear stale level slope quorum after "
                  << level_slope_pending_invalid_count_
                  << " committed frames without a valid plane. active="
                  << level_slope_protection_active_
                  << " enter_count=" << level_slope_enter_count_
                  << " exit_count=" << level_slope_exit_count_
                  << " committed_scan=" << processed_scan_index_ << RESET;
        level_slope_enter_count_ = 0;
        level_slope_exit_count_ = 0;
        level_slope_pending_invalid_count_ = 0;
      }
    }

    const auto clear_spatial_window = [&] (const bool clear_mismatch) {
      level_slope_spatial_path_m_ = 0.0;
      level_slope_spatial_supported_path_m_ = 0.0;
      level_slope_spatial_observed_dz_m_ = 0.0;
      level_slope_spatial_expected_dz_m_ = 0.0;
      if (clear_mismatch) {
        level_slope_spatial_mismatch_count_ = 0;
      }
    };

    if (!protection_was_active && level_slope_protection_active_) {
      // Do not charge the displacement preceding the latch to its first lease.
      clear_spatial_window(true);
    } else if (((protection_was_active &&
                 level_slope_protection_active_) ||
                (!level_slope_protection_active_ &&
                 level_slope_spatial_reentry_blocked_)) &&
               has_last_accepted_pose_) {
      const V3 frame_delta = current_pose.t_ - last_pose_.t_;
      const double horizontal_step = frame_delta.head<2>().norm();
      if (std::isfinite(horizontal_step) && horizontal_step > 1.0e-4) {
        level_slope_spatial_path_m_ += horizontal_step;
        if (level_observation.plane_valid) {
          V3 plane_up_world =
              current_pose.R_ * level_observation.normal_body;
          const scalar plane_norm = plane_up_world.norm();
          if (plane_up_world.allFinite() &&
              std::isfinite(static_cast<double>(plane_norm)) &&
              plane_norm > 1.0e-6) {
            plane_up_world /= plane_norm;
            if (plane_up_world.z() < 0.0) {
              plane_up_world = -plane_up_world;
            }
            // Ground steeper than 60 degrees is not a usable height lease.
            // Treat it as unsupported instead of allowing an unstable 1/nz.
            if (plane_up_world.z() >= 0.5) {
              const double expected_dz = -(
                  static_cast<double>(plane_up_world.x()) * frame_delta.x() +
                  static_cast<double>(plane_up_world.y()) * frame_delta.y()) /
                  static_cast<double>(plane_up_world.z());
              if (std::isfinite(expected_dz)) {
                level_slope_spatial_supported_path_m_ += horizontal_step;
                level_slope_spatial_observed_dz_m_ += frame_delta.z();
                level_slope_spatial_expected_dz_m_ += expected_dz;
              }
            }
          }
        }
      }

      if (level_slope_spatial_path_m_ >=
          g_level_slope_spatial_window_m) {
        level_slope_spatial_last_support_ratio_ =
            level_slope_spatial_supported_path_m_ /
            std::max(level_slope_spatial_path_m_, 1.0e-6);
        const bool spatial_support_valid =
            level_slope_spatial_last_support_ratio_ >=
                g_level_slope_spatial_min_support_ratio &&
            level_slope_spatial_supported_path_m_ > 1.0e-3;
        if (spatial_support_valid) {
          level_slope_spatial_last_observed_grade_deg_ = std::atan2(
              level_slope_spatial_observed_dz_m_,
              level_slope_spatial_supported_path_m_) * 180.0 / M_PI;
          level_slope_spatial_last_expected_grade_deg_ = std::atan2(
              level_slope_spatial_expected_dz_m_,
              level_slope_spatial_supported_path_m_) * 180.0 / M_PI;
          level_slope_spatial_last_error_deg_ = std::abs(
              level_slope_spatial_last_observed_grade_deg_ -
              level_slope_spatial_last_expected_grade_deg_);
        } else {
          level_slope_spatial_last_observed_grade_deg_ = 0.0;
          level_slope_spatial_last_expected_grade_deg_ = 0.0;
          level_slope_spatial_last_error_deg_ =
              std::numeric_limits<double>::infinity();
        }
        const bool spatial_lease_consistent =
            spatial_support_valid &&
            level_slope_spatial_last_error_deg_ <=
                g_level_slope_spatial_max_grade_error_deg;
        if (spatial_lease_consistent) {
          level_slope_spatial_mismatch_count_ = 0;
          if (level_slope_spatial_reentry_blocked_) {
            level_slope_spatial_reentry_consistent_count_ = std::min(
                level_slope_spatial_reentry_consistent_count_ + 1,
                g_level_slope_spatial_reentry_consistent_windows);
          }
        } else {
          level_slope_spatial_reentry_consistent_count_ = 0;
          if (level_slope_protection_active_) {
            level_slope_spatial_mismatch_count_ = std::min(
                level_slope_spatial_mismatch_count_ + 1,
                g_level_slope_spatial_max_mismatch_windows);
          }
        }
        LOG_IF(WARNING, !spatial_lease_consistent)
            << YELLOW
            << " ---> [SuperLIO]: level slope spatial lease. consistent="
            << spatial_lease_consistent
            << " path=" << level_slope_spatial_path_m_
            << "m supported_path="
            << level_slope_spatial_supported_path_m_
            << "m support_ratio="
            << level_slope_spatial_last_support_ratio_
            << " observed_grade="
            << level_slope_spatial_last_observed_grade_deg_
            << "deg plane_grade="
            << level_slope_spatial_last_expected_grade_deg_
            << "deg error=" << level_slope_spatial_last_error_deg_
            << "deg limit="
            << g_level_slope_spatial_max_grade_error_deg
            << "deg mismatch_windows="
            << level_slope_spatial_mismatch_count_
            << "/" << g_level_slope_spatial_max_mismatch_windows
            << " reentry_blocked="
            << level_slope_spatial_reentry_blocked_
            << " reentry_consistent_windows="
            << level_slope_spatial_reentry_consistent_count_
            << "/" << g_level_slope_spatial_reentry_consistent_windows
            << " committed_scan=" << processed_scan_index_ << RESET;
        if (level_slope_spatial_mismatch_count_ >=
                g_level_slope_spatial_max_mismatch_windows &&
            level_slope_protection_active_) {
          level_slope_protection_active_ = false;
          level_slope_exit_count_ = 0;
          level_slope_recovery_active_ = true;
          level_slope_recovery_count_ = 0;
          level_slope_spatial_reentry_blocked_ = true;
          level_slope_spatial_reentry_consistent_count_ = 0;
          slope_transition_reason = spatial_support_valid
              ? "spatial_grade_mismatch"
              : "spatial_evidence_expired";
        } else if (level_slope_spatial_reentry_blocked_ &&
                   level_slope_spatial_reentry_consistent_count_ >=
                       g_level_slope_spatial_reentry_consistent_windows) {
          level_slope_spatial_reentry_blocked_ = false;
          level_slope_spatial_reentry_consistent_count_ = 0;
          LOG(INFO) << GREEN
                    << " ---> [SuperLIO]: level slope spatial re-entry enabled"
                    << " after "
                    << g_level_slope_spatial_reentry_consistent_windows
                    << " consistent " << g_level_slope_spatial_window_m
                    << "m windows. committed_scan="
                    << processed_scan_index_ << RESET;
        }
        clear_spatial_window(
            !level_slope_protection_active_);
      }
    }

    if (protection_was_active && !level_slope_protection_active_) {
      clear_spatial_window(true);
    }
    if (level_slope_protection_active_ != protection_was_active) {
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: level slope protection transition. "
                   << "active=" << level_slope_protection_active_
                   << " reason=" << slope_transition_reason
                   << " gravity_angle="
                   << level_observation.gravity_angle_deg
                   << "deg world_plane_tilt="
                   << level_observation.innovation_deg
                   << "deg enter_threshold="
                   << g_level_max_plane_gravity_angle_deg
                   << "deg exit_threshold="
                   << g_level_slope_exit_angle_deg
                   << "deg spatial_window="
                   << g_level_slope_spatial_window_m
                   << "m spatial_reentry_blocked="
                   << level_slope_spatial_reentry_blocked_
                   << " recovery_active="
                   << level_slope_recovery_active_
                   << " recovery_frames="
                   << g_level_slope_recovery_min_frames
                   << " committed_scan=" << processed_scan_index_
                   << RESET;
    }
    if (recovery_was_active && !level_slope_recovery_active_ &&
        !level_slope_protection_active_) {
      LOG(INFO) << GREEN
                << " ---> [SuperLIO]: level slope recovery complete after "
                << level_slope_recovery_count_
                << " committed valid planes. recovery_gate=1"
                << " committed_scan=" << processed_scan_index_ << RESET;
    }
  }

  // Commit the relative-height chain only with a safe LiDAR pose.  A rejected
  // scan cannot move the reference.  A valid discontinuity is intentionally
  // adopted as the new reference after being gated, so stairs/curbs do not
  // get pulled toward the previous floor on subsequent frames.
  if (observation_valid_ && g_ground_height_continuity_enable) {
    if (ground_height_continuity_valid) {
      if (std::abs(ground_height_direct_correction) > 1.0e-9) {
        ++ground_height_continuity_accepted_count_;
        ground_height_continuity_applied_offset_m_ +=
            ground_height_direct_correction;
        ground_height_continuity_applied_offset_m_ = std::clamp(
            ground_height_continuity_applied_offset_m_,
            -g_ground_height_continuity_max_total_correction_m,
            g_ground_height_continuity_max_total_correction_m);
      } else {
        ++ground_height_continuity_gated_count_;
      }
      if (ground_height_budget_limited) {
        ++ground_height_continuity_budget_gated_count_;
      }
    } else if (level_observation.plane_valid &&
               ground_height_reference_available) {
      ++ground_height_continuity_gated_count_;
    } else {
      ++ground_height_continuity_rejected_count_;
    }

    if (level_observation.plane_valid) {
      const SE3 committed_pose = kf_->GetSE3();
      V3 committed_normal_world =
          committed_pose.R_ * level_observation.normal_body;
      double committed_plane_offset_body =
          level_observation.plane_offset_body;
      const scalar committed_normal_norm =
          committed_normal_world.norm();
      if (committed_normal_world.allFinite() &&
          std::isfinite(static_cast<double>(committed_normal_norm)) &&
          committed_normal_norm > 1.0e-6) {
        committed_normal_world /= committed_normal_norm;
        if (committed_normal_world.z() < 0.0) {
          committed_normal_world = -committed_normal_world;
          committed_plane_offset_body =
              -committed_plane_offset_body;
        }
        if (committed_normal_world.z() >= 0.5) {
          const double committed_ground_height =
              committed_pose.t_.z() -
              committed_plane_offset_body /
                  committed_normal_world.z();
          if (std::isfinite(committed_ground_height)) {
            ground_height_continuity_reference_.valid = true;
            ground_height_continuity_reference_.normal_world =
                committed_normal_world;
            ground_height_continuity_reference_.sensor_position_world =
                committed_pose.t_;
            ground_height_continuity_reference_.ground_height_world =
                committed_ground_height;
            ground_height_continuity_reference_.scan_index =
                processed_scan_index_;
          }
        }
      }
    }
  }

  // Level health counters, like wall counters below, describe only frontend
  // frames whose pose was committed. A valid plane suppressed by either the
  // instantaneous dual-slope gate or the latch is reported separately.
  if (observation_valid_) {
    if (level_observation.plane_valid &&
        (level_observation.slope_rejected ||
         level_effective_gate <= 1e-6)) {
      ++level_constraint_gated_count_;
    } else if (!level_observation.valid) {
      ++level_constraint_rejected_count_;
    } else {
      ++level_constraint_accepted_count_;
    }
  }

  // Wall health counters describe committed frontend frames. A scan whose
  // pose and wall state were rolled back must not look like accepted history
  // in diagnostics.
  if (observation_valid_) {
    if (!wall_yaw_observation.extraction_attempted) {
      ++wall_yaw_extraction_skipped_count_;
    } else if (!wall_yaw_observation.valid) {
      ++wall_yaw_constraint_rejected_count_;
    } else if (wall_yaw_effective_constraint_gate <= 1e-6) {
      ++wall_yaw_constraint_gated_count_;
    } else {
      ++wall_yaw_constraint_accepted_count_;
    }
  }

  frame_num_++;
}

void SuperLIO::UpdateMap() {
  if (!observation_valid_) return;
  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0) return;
  
  last_pose_ = kf_->GetSE3();
  has_last_accepted_pose_ = true;
  last_accepted_state_ = kf_->GetSysState();
  last_accepted_covariance_ = kf_->GetCov();
  has_last_accepted_state_ = true;
  points_world_v3_.resize(ptsize);
  
  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;
  
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body_v3_[i];
    points_world_v3_[i] = R * pt + t;
  }
  
  ivox_->insert(points_world_v3_);

}


void SuperLIO::Output(){
  auto state = kf_->GetNavState();
  if (g_loop_online_enable && g_save_map) {
    const SE3 raw_pose(state.R.R_, state.p);
    pollOnlineLoopResults(raw_pose, state.timestamp);
    const SE3 corrected_pose = online_loop_map_to_odom_ * raw_pose;
    NavState corrected_state = state;
    corrected_state.R = SO3(corrected_pose.R_);
    corrected_state.p = corrected_pose.t_;
    corrected_state.v = online_loop_map_to_odom_.R_ * state.v;
    data_wrapper_->pub_relocation_odom(
        corrected_state,
        raw_pose,
        online_loop_map_to_odom_,
        observation_valid_);
    state = corrected_state;
  } else {
    data_wrapper_->pub_odom(state);
  }

  if (!observation_valid_) return;

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  CloudPtr world_pc(new PointCloudType());
  
  if(g_visual_map){
    static int count = -1;
    count++;
    if(count % g_pub_step != 0){
      return;
    }
    count = 0;
    if(g_visual_dense){
      pcl::transformPointCloud(*scan_undistort_full_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }else{
      pcl::transformPointCloud(*ds_undistort_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }
  }
}

void SuperLIO::printTimeRecord(){
  if(!g_time_eva) return;
  time_record_.PrintAll();
}

} // namespace END.
