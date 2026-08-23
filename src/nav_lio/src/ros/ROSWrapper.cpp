
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>


using namespace BASIC;

namespace LI2Sup{

namespace {

constexpr double kSensorTimeEpsilon = 1e-6;
IMUData interpolateImu(const IMUData& before, const IMUData& after, double time)
{
  const double interval = after.secs - before.secs;
  const double ratio = interval > kSensorTimeEpsilon
    ? std::clamp((time - before.secs) / interval, 0.0, 1.0)
    : 0.0;

  IMUData result;
  result.secs = time;
  result.acc = before.acc + ratio * (after.acc - before.acc);
  result.gyr = before.gyr + ratio * (after.gyr - before.gyr);
  return result;
}

}  // namespace

void LoadParamFromRos(rclcpp::Node& node)
{
  node.declare_parameter<bool>("lio.map.save_map", false);
  node.get_parameter("lio.map.save_map", g_save_map);

  LOG(INFO) << GREEN << " ---> [Param] map/save_map: "
            << (g_save_map ? "true" : "false") << RESET;

  node.declare_parameter<bool>("lio.eva.timer", false);
  node.get_parameter("lio.eva.timer", g_time_eva);

  node.declare_parameter<bool>("lio.map.if_filter", false);
  node.get_parameter("lio.map.if_filter", g_if_filter);

  node.declare_parameter<std::string>("lio.map.save_map_dir", "");
  node.get_parameter("lio.map.save_map_dir", g_save_map_dir);
  if (!g_save_map_dir.empty() && g_save_map_dir.front() != '/') {
    g_save_map_dir = g_root_dir + g_save_map_dir;
  }

  node.declare_parameter<std::string>("lio.map.map_name", "default");
  node.get_parameter("lio.map.map_name", g_map_name);

  node.declare_parameter<double>("lio.map.ds_size", 0.5);
  node.get_parameter("lio.map.ds_size", g_map_ds_size);

  node.declare_parameter<int>("lio.map.save_interval", 1);
  node.get_parameter("lio.map.save_interval", g_pcd_save_interval);

  node.declare_parameter<std::int64_t>(
      "lio.map.max_points_in_memory", 0);
  node.get_parameter(
      "lio.map.max_points_in_memory",
      g_map_max_points_in_memory);
  if (g_map_max_points_in_memory < 0) {
    throw std::invalid_argument(
        "lio.map.max_points_in_memory must be non-negative");
  }
  if (g_map_max_points_in_memory > 0 && !g_if_filter) {
    throw std::invalid_argument(
        "bounded map persistence requires lio.map.if_filter=true");
  }

  node.declare_parameter<int>("lio.map.max_pending_writes", 64);
  node.get_parameter(
      "lio.map.max_pending_writes",
      g_map_max_pending_writes);
  if (g_map_max_pending_writes < 1 ||
      g_map_max_pending_writes > 256) {
    throw std::invalid_argument(
        "lio.map.max_pending_writes must be in [1, 256]");
  }

  node.declare_parameter<bool>("lio.map.cleanup_work_files", true);
  node.get_parameter(
      "lio.map.cleanup_work_files",
      g_map_cleanup_work_files);

  node.declare_parameter<bool>("lio.loop.enable", false);
  node.get_parameter("lio.loop.enable", g_loop_closure_enable);
  node.declare_parameter<bool>(
      "lio.loop.endpoint_corridor_partial.enable", true);
  node.get_parameter(
      "lio.loop.endpoint_corridor_partial.enable",
      g_loop_endpoint_corridor_partial_enable);

  node.declare_parameter<bool>("lio.loop.persist_keyframes", false);
  node.get_parameter(
      "lio.loop.persist_keyframes",
      g_loop_persist_keyframes);

  node.declare_parameter<std::string>("lio.loop.map_name", "map_loop.pcd");
  node.get_parameter("lio.loop.map_name", g_loop_map_name);

  node.declare_parameter<double>("lio.loop.keyframe_min_distance", 0.5);
  double loop_keyframe_min_distance = 0.5;
  node.get_parameter("lio.loop.keyframe_min_distance", loop_keyframe_min_distance);
  g_loop_keyframe_min_distance = static_cast<float>(loop_keyframe_min_distance);
  if (!std::isfinite(g_loop_keyframe_min_distance) ||
      g_loop_keyframe_min_distance <= 0.0f) {
    throw std::invalid_argument(
        "lio.loop.keyframe_min_distance must be finite and positive");
  }

  node.declare_parameter<int>("lio.loop.keyframe_min_gap", 80);
  node.get_parameter("lio.loop.keyframe_min_gap", g_loop_keyframe_min_gap);
  if (g_loop_keyframe_min_gap < 1) {
    throw std::invalid_argument(
        "lio.loop.keyframe_min_gap must be positive");
  }

  node.declare_parameter<double>(
      "lio.loop.internal_min_sensor_time_seconds", 30.0);
  node.get_parameter(
      "lio.loop.internal_min_sensor_time_seconds",
      g_loop_internal_min_sensor_time_seconds);
  if (!std::isfinite(g_loop_internal_min_sensor_time_seconds) ||
      g_loop_internal_min_sensor_time_seconds <= 0.0) {
    throw std::invalid_argument(
        "lio.loop.internal_min_sensor_time_seconds must be finite and positive");
  }

  node.declare_parameter<double>("lio.loop.search_radius", 5.0);
  double loop_search_radius = 5.0;
  node.get_parameter("lio.loop.search_radius", loop_search_radius);
  g_loop_search_radius = static_cast<float>(loop_search_radius);
  if (g_loop_search_radius <= 0.0f) {
    throw std::invalid_argument("lio.loop.search_radius must be positive");
  }

  node.declare_parameter<double>("lio.loop.internal_search_radius", 1.5);
  double loop_internal_search_radius = 1.5;
  node.get_parameter(
      "lio.loop.internal_search_radius",
      loop_internal_search_radius);
  g_loop_internal_search_radius =
      static_cast<float>(loop_internal_search_radius);
  if (g_loop_internal_search_radius <= 0.0f ||
      g_loop_internal_search_radius > g_loop_search_radius) {
    throw std::invalid_argument(
        "lio.loop.internal_search_radius must be positive and no greater than lio.loop.search_radius");
  }

  node.declare_parameter<double>("lio.loop.icp_max_distance", 2.0);
  double loop_icp_max_distance = 2.0;
  node.get_parameter("lio.loop.icp_max_distance", loop_icp_max_distance);
  g_loop_icp_max_distance = static_cast<float>(loop_icp_max_distance);
  if (!std::isfinite(g_loop_icp_max_distance) ||
      g_loop_icp_max_distance <= 0.0f) {
    throw std::invalid_argument(
        "lio.loop.icp_max_distance must be finite and positive");
  }

  node.declare_parameter<double>("lio.loop.icp_score_threshold", 1.0);
  double loop_icp_score_threshold = 1.0;
  node.get_parameter("lio.loop.icp_score_threshold", loop_icp_score_threshold);
  g_loop_icp_score_threshold = static_cast<float>(loop_icp_score_threshold);

  node.declare_parameter<double>("lio.loop.map_ds_size", 0.1);
  double loop_map_ds_size = 0.1;
  node.get_parameter("lio.loop.map_ds_size", loop_map_ds_size);
  g_loop_map_ds_size = static_cast<float>(loop_map_ds_size);

  node.declare_parameter<int>("lio.loop.candidate_limit", 30);
  node.get_parameter("lio.loop.candidate_limit", g_loop_candidate_limit);
  if (g_loop_candidate_limit < 1) {
    throw std::invalid_argument("lio.loop.candidate_limit must be positive");
  }

  node.declare_parameter<int>("lio.loop.local_window_size", 10);
  node.get_parameter("lio.loop.local_window_size", g_loop_local_window_size);
  if (g_loop_local_window_size < 2) {
    throw std::invalid_argument("lio.loop.local_window_size must be at least 2");
  }

  node.declare_parameter<double>("lio.loop.max_correction_rotation_deg", 5.0);
  double loop_max_correction_rotation_deg = 5.0;
  node.get_parameter(
      "lio.loop.max_correction_rotation_deg",
      loop_max_correction_rotation_deg);
  g_loop_max_correction_rotation_deg =
      static_cast<float>(loop_max_correction_rotation_deg);
  if (g_loop_max_correction_rotation_deg <= 0.0f ||
      g_loop_max_correction_rotation_deg > 180.0f) {
    throw std::invalid_argument(
        "lio.loop.max_correction_rotation_deg must be in (0, 180]");
  }

  node.declare_parameter<double>(
      "lio.loop.max_correction_tilt_deg", 2.0);
  double loop_max_correction_tilt_deg = 2.0;
  node.get_parameter(
      "lio.loop.max_correction_tilt_deg",
      loop_max_correction_tilt_deg);
  g_loop_max_correction_tilt_deg =
      static_cast<float>(loop_max_correction_tilt_deg);
  if (g_loop_max_correction_tilt_deg <= 0.0f ||
      g_loop_max_correction_tilt_deg > 45.0f) {
    throw std::invalid_argument(
        "lio.loop.max_correction_tilt_deg must be in (0, 45]");
  }

  node.declare_parameter<double>(
      "lio.loop.max_adaptive_yaw_deg", 15.0);
  double loop_max_adaptive_yaw_deg = 15.0;
  node.get_parameter(
      "lio.loop.max_adaptive_yaw_deg",
      loop_max_adaptive_yaw_deg);
  g_loop_max_adaptive_yaw_deg =
      static_cast<float>(loop_max_adaptive_yaw_deg);
  if (g_loop_max_adaptive_yaw_deg <
          g_loop_max_correction_rotation_deg ||
      g_loop_max_adaptive_yaw_deg > 90.0f) {
    throw std::invalid_argument(
        "lio.loop.max_adaptive_yaw_deg must be no smaller than "
        "lio.loop.max_correction_rotation_deg and no greater than 90");
  }

  node.declare_parameter<double>("lio.loop.min_overlap_ratio", 0.35);
  double loop_min_overlap_ratio = 0.35;
  node.get_parameter("lio.loop.min_overlap_ratio", loop_min_overlap_ratio);
  g_loop_min_overlap_ratio = static_cast<float>(loop_min_overlap_ratio);
  if (g_loop_min_overlap_ratio <= 0.0f ||
      g_loop_min_overlap_ratio > 1.0f) {
    throw std::invalid_argument(
        "lio.loop.min_overlap_ratio must be in (0, 1]");
  }

  node.declare_parameter<double>(
      "lio.loop.translation_drift_ratio", 0.02);
  double loop_translation_drift_ratio = 0.02;
  node.get_parameter(
      "lio.loop.translation_drift_ratio",
      loop_translation_drift_ratio);
  g_loop_translation_drift_ratio =
      static_cast<float>(loop_translation_drift_ratio);
  if (g_loop_translation_drift_ratio <= 0.0f) {
    throw std::invalid_argument(
        "lio.loop.translation_drift_ratio must be positive");
  }

  node.declare_parameter<double>(
      "lio.loop.rotation_drift_deg_per_m", 0.02);
  double loop_rotation_drift_deg_per_m = 0.02;
  node.get_parameter(
      "lio.loop.rotation_drift_deg_per_m",
      loop_rotation_drift_deg_per_m);
  g_loop_rotation_drift_deg_per_m =
      static_cast<float>(loop_rotation_drift_deg_per_m);
  if (g_loop_rotation_drift_deg_per_m <= 0.0f) {
    throw std::invalid_argument(
        "lio.loop.rotation_drift_deg_per_m must be positive");
  }

  node.declare_parameter<double>(
      "lio.loop.min_consistency_weight", 0.25);
  double loop_min_consistency_weight = 0.25;
  node.get_parameter(
      "lio.loop.min_consistency_weight",
      loop_min_consistency_weight);
  g_loop_min_consistency_weight =
      static_cast<float>(loop_min_consistency_weight);
  if (g_loop_min_consistency_weight <= 0.0f ||
      g_loop_min_consistency_weight > 1.0f) {
    throw std::invalid_argument(
        "lio.loop.min_consistency_weight must be in (0, 1]");
  }

  node.declare_parameter<double>(
      "lio.loop.verification.max_distance", 0.6);
  double loop_verification_max_distance = 0.6;
  node.get_parameter(
      "lio.loop.verification.max_distance",
      loop_verification_max_distance);
  g_loop_verification_max_distance =
      static_cast<float>(loop_verification_max_distance);
  if (g_loop_verification_max_distance <= 0.0f ||
      g_loop_verification_max_distance > g_loop_icp_max_distance) {
    throw std::invalid_argument(
        "lio.loop.verification.max_distance must be positive and no greater than lio.loop.icp_max_distance");
  }

  node.declare_parameter<double>(
      "lio.loop.verification.min_symmetric_overlap", 0.45);
  double loop_min_symmetric_overlap = 0.45;
  node.get_parameter(
      "lio.loop.verification.min_symmetric_overlap",
      loop_min_symmetric_overlap);
  g_loop_min_symmetric_overlap =
      static_cast<float>(loop_min_symmetric_overlap);
  if (g_loop_min_symmetric_overlap <= 0.0f ||
      g_loop_min_symmetric_overlap > 1.0f) {
    throw std::invalid_argument(
        "lio.loop.verification.min_symmetric_overlap must be in (0, 1]");
  }

  node.declare_parameter<double>(
      "lio.loop.verification.max_trimmed_rmse", 0.30);
  double loop_max_trimmed_rmse = 0.30;
  node.get_parameter(
      "lio.loop.verification.max_trimmed_rmse",
      loop_max_trimmed_rmse);
  g_loop_max_trimmed_rmse =
      static_cast<float>(loop_max_trimmed_rmse);
  if (g_loop_max_trimmed_rmse <= 0.0f ||
      g_loop_max_trimmed_rmse > g_loop_verification_max_distance) {
    throw std::invalid_argument(
        "lio.loop.verification.max_trimmed_rmse must be positive and no greater than verification.max_distance");
  }

  node.declare_parameter<double>(
      "lio.loop.verification.block_size", 1.0);
  double loop_verification_block_size = 1.0;
  node.get_parameter(
      "lio.loop.verification.block_size",
      loop_verification_block_size);
  g_loop_verification_block_size =
      static_cast<float>(loop_verification_block_size);
  if (g_loop_verification_block_size <= 0.0f) {
    throw std::invalid_argument(
        "lio.loop.verification.block_size must be positive");
  }

  node.declare_parameter<int>(
      "lio.loop.verification.min_blocks", 6);
  node.get_parameter(
      "lio.loop.verification.min_blocks",
      g_loop_min_verification_blocks);
  if (g_loop_min_verification_blocks < 1) {
    throw std::invalid_argument(
        "lio.loop.verification.min_blocks must be positive");
  }

  node.declare_parameter<double>(
      "lio.loop.verification.min_block_ratio", 0.50);
  double loop_min_verification_block_ratio = 0.50;
  node.get_parameter(
      "lio.loop.verification.min_block_ratio",
      loop_min_verification_block_ratio);
  g_loop_min_verification_block_ratio =
      static_cast<float>(loop_min_verification_block_ratio);
  if (g_loop_min_verification_block_ratio <= 0.0f ||
      g_loop_min_verification_block_ratio > 1.0f) {
    throw std::invalid_argument(
        "lio.loop.verification.min_block_ratio must be in (0, 1]");
  }

  node.declare_parameter<double>(
      "lio.loop.verification.min_span", 3.0);
  double loop_min_verification_span = 3.0;
  node.get_parameter(
      "lio.loop.verification.min_span",
      loop_min_verification_span);
  g_loop_min_verification_span =
      static_cast<float>(loop_min_verification_span);
  if (g_loop_min_verification_span <= 0.0f) {
    throw std::invalid_argument(
        "lio.loop.verification.min_span must be positive");
  }

  node.declare_parameter<double>(
      "lio.loop.verification.min_structural_overlap", 0.35);
  double loop_min_structural_overlap = 0.35;
  node.get_parameter(
      "lio.loop.verification.min_structural_overlap",
      loop_min_structural_overlap);
  g_loop_min_structural_overlap =
      static_cast<float>(loop_min_structural_overlap);
  if (g_loop_min_structural_overlap <= 0.0f ||
      g_loop_min_structural_overlap > 1.0f) {
    throw std::invalid_argument(
        "lio.loop.verification.min_structural_overlap must be in (0, 1]");
  }

  node.declare_parameter<double>(
      "lio.loop.max_local_translation_strain", 0.015);
  double loop_max_local_translation_strain = 0.015;
  node.get_parameter(
      "lio.loop.max_local_translation_strain",
      loop_max_local_translation_strain);
  g_loop_max_local_translation_strain =
      static_cast<float>(loop_max_local_translation_strain);
  if (g_loop_max_local_translation_strain <= 0.0f ||
      g_loop_max_local_translation_strain > 0.10f) {
    throw std::invalid_argument(
        "lio.loop.max_local_translation_strain must be in (0, 0.10]");
  }

  node.declare_parameter<double>(
      "lio.loop.max_local_translation_delta", 0.50);
  double loop_max_local_translation_delta = 0.50;
  node.get_parameter(
      "lio.loop.max_local_translation_delta",
      loop_max_local_translation_delta);
  g_loop_max_local_translation_delta =
      static_cast<float>(loop_max_local_translation_delta);
  if (g_loop_max_local_translation_delta <= 0.0f) {
    throw std::invalid_argument(
        "lio.loop.max_local_translation_delta must be positive");
  }

  node.declare_parameter<bool>(
      "lio.loop.ground_z_refinement.enable", true);
  node.get_parameter(
      "lio.loop.ground_z_refinement.enable",
      g_loop_ground_z_refinement_enable);

  node.declare_parameter<double>(
      "lio.loop.ground_z_refinement.cell_size", 0.25);
  double loop_ground_z_cell_size = 0.25;
  node.get_parameter(
      "lio.loop.ground_z_refinement.cell_size",
      loop_ground_z_cell_size);
  g_loop_ground_z_cell_size =
      static_cast<float>(loop_ground_z_cell_size);
  if (g_loop_ground_z_cell_size <= 0.0f) {
    throw std::invalid_argument(
        "lio.loop.ground_z_refinement.cell_size must be positive");
  }

  node.declare_parameter<double>(
      "lio.loop.ground_z_refinement.pair_xy_distance", 0.35);
  double loop_ground_z_pair_xy_distance = 0.35;
  node.get_parameter(
      "lio.loop.ground_z_refinement.pair_xy_distance",
      loop_ground_z_pair_xy_distance);
  g_loop_ground_z_pair_xy_distance =
      static_cast<float>(loop_ground_z_pair_xy_distance);
  if (g_loop_ground_z_pair_xy_distance <= 0.0f) {
    throw std::invalid_argument(
        "lio.loop.ground_z_refinement.pair_xy_distance must be positive");
  }

  node.declare_parameter<int>(
      "lio.loop.ground_z_refinement.min_pairs", 80);
  node.get_parameter(
      "lio.loop.ground_z_refinement.min_pairs",
      g_loop_ground_z_min_pairs);
  if (g_loop_ground_z_min_pairs < 10) {
    throw std::invalid_argument(
        "lio.loop.ground_z_refinement.min_pairs must be at least 10");
  }

  node.declare_parameter<double>(
      "lio.loop.ground_z_refinement.min_inlier_ratio", 0.65);
  double loop_ground_z_min_inlier_ratio = 0.65;
  node.get_parameter(
      "lio.loop.ground_z_refinement.min_inlier_ratio",
      loop_ground_z_min_inlier_ratio);
  g_loop_ground_z_min_inlier_ratio =
      static_cast<float>(loop_ground_z_min_inlier_ratio);
  if (g_loop_ground_z_min_inlier_ratio <= 0.0f ||
      g_loop_ground_z_min_inlier_ratio > 1.0f) {
    throw std::invalid_argument(
        "lio.loop.ground_z_refinement.min_inlier_ratio must be in (0, 1]");
  }

  node.declare_parameter<double>(
      "lio.loop.ground_z_refinement.max_mad", 0.08);
  double loop_ground_z_max_mad = 0.08;
  node.get_parameter(
      "lio.loop.ground_z_refinement.max_mad",
      loop_ground_z_max_mad);
  g_loop_ground_z_max_mad =
      static_cast<float>(loop_ground_z_max_mad);
  if (g_loop_ground_z_max_mad <= 0.0f) {
    throw std::invalid_argument(
        "lio.loop.ground_z_refinement.max_mad must be positive");
  }

  node.declare_parameter<double>(
      "lio.loop.ground_z_refinement.max_adjustment", 0.5);
  double loop_ground_z_max_adjustment = 0.5;
  node.get_parameter(
      "lio.loop.ground_z_refinement.max_adjustment",
      loop_ground_z_max_adjustment);
  g_loop_ground_z_max_adjustment =
      static_cast<float>(loop_ground_z_max_adjustment);
  if (g_loop_ground_z_max_adjustment <= 0.0f) {
    throw std::invalid_argument(
        "lio.loop.ground_z_refinement.max_adjustment must be positive");
  }

  node.declare_parameter<double>(
      "lio.loop.ground_z_refinement.planar_hold_weight", 0.25);
  double loop_ground_z_planar_hold_weight = 0.25;
  node.get_parameter(
      "lio.loop.ground_z_refinement.planar_hold_weight",
      loop_ground_z_planar_hold_weight);
  g_loop_ground_z_planar_hold_weight =
      static_cast<float>(loop_ground_z_planar_hold_weight);
  if (g_loop_ground_z_planar_hold_weight <= 0.0f ||
      g_loop_ground_z_planar_hold_weight > 1.0f) {
    throw std::invalid_argument(
        "lio.loop.ground_z_refinement.planar_hold_weight must be in (0, 1]");
  }

  node.declare_parameter<bool>(
      "lio.loop.post_residual_refinement.enable", true);
  node.get_parameter(
      "lio.loop.post_residual_refinement.enable",
      g_loop_post_residual_refinement_enable);

  node.declare_parameter<int>(
      "lio.loop.post_residual_refinement.min_anchors", 2);
  node.get_parameter(
      "lio.loop.post_residual_refinement.min_anchors",
      g_loop_post_residual_refinement_min_anchors);
  if (g_loop_post_residual_refinement_min_anchors < 2 ||
      g_loop_post_residual_refinement_min_anchors > 8) {
    throw std::invalid_argument(
        "lio.loop.post_residual_refinement.min_anchors must be in [2, 8]");
  }

  node.declare_parameter<double>(
      "lio.loop.post_residual_refinement.target_translation", 0.10);
  double loop_post_residual_target_translation = 0.10;
  node.get_parameter(
      "lio.loop.post_residual_refinement.target_translation",
      loop_post_residual_target_translation);
  g_loop_post_residual_refinement_target_translation =
      static_cast<float>(loop_post_residual_target_translation);
  if (!std::isfinite(g_loop_post_residual_refinement_target_translation) ||
      g_loop_post_residual_refinement_target_translation < 0.03f ||
      g_loop_post_residual_refinement_target_translation > 0.30f) {
    throw std::invalid_argument(
        "lio.loop.post_residual_refinement.target_translation must be in [0.03, 0.30]");
  }

  node.declare_parameter<double>(
      "lio.loop.post_residual_refinement.target_rotation_deg", 0.20);
  double loop_post_residual_target_rotation_deg = 0.20;
  node.get_parameter(
      "lio.loop.post_residual_refinement.target_rotation_deg",
      loop_post_residual_target_rotation_deg);
  g_loop_post_residual_refinement_target_rotation_deg =
      static_cast<float>(loop_post_residual_target_rotation_deg);
  if (!std::isfinite(g_loop_post_residual_refinement_target_rotation_deg) ||
      g_loop_post_residual_refinement_target_rotation_deg < 0.05f ||
      g_loop_post_residual_refinement_target_rotation_deg > 1.0f) {
    throw std::invalid_argument(
        "lio.loop.post_residual_refinement.target_rotation_deg must be in [0.05, 1.0]");
  }

  node.declare_parameter<double>(
      "lio.loop.post_residual_refinement.max_weight_scale", 4.0);
  double loop_post_residual_max_weight_scale = 4.0;
  node.get_parameter(
      "lio.loop.post_residual_refinement.max_weight_scale",
      loop_post_residual_max_weight_scale);
  g_loop_post_residual_refinement_max_weight_scale =
      static_cast<float>(loop_post_residual_max_weight_scale);
  if (!std::isfinite(g_loop_post_residual_refinement_max_weight_scale) ||
      g_loop_post_residual_refinement_max_weight_scale < 1.0f ||
      g_loop_post_residual_refinement_max_weight_scale > 8.0f) {
    throw std::invalid_argument(
        "lio.loop.post_residual_refinement.max_weight_scale must be in [1, 8]");
  }

  node.declare_parameter<double>(
      "lio.loop.finalize_base_seconds", 120.0);
  node.get_parameter(
      "lio.loop.finalize_base_seconds",
      g_loop_finalize_base_seconds);
  if (g_loop_finalize_base_seconds <= 0.0) {
    throw std::invalid_argument(
        "lio.loop.finalize_base_seconds must be positive");
  }

  node.declare_parameter<double>(
      "lio.loop.finalize_seconds_per_keyframe", 0.04);
  node.get_parameter(
      "lio.loop.finalize_seconds_per_keyframe",
      g_loop_finalize_seconds_per_keyframe);
  if (g_loop_finalize_seconds_per_keyframe < 0.0) {
    throw std::invalid_argument(
        "lio.loop.finalize_seconds_per_keyframe must be non-negative");
  }

  node.declare_parameter<double>(
      "lio.loop.max_finalize_seconds", 600.0);
  node.get_parameter(
      "lio.loop.max_finalize_seconds",
      g_loop_max_finalize_seconds);
  if (g_loop_max_finalize_seconds < g_loop_finalize_base_seconds) {
    throw std::invalid_argument(
        "lio.loop.max_finalize_seconds must be no smaller than "
        "lio.loop.finalize_base_seconds");
  }

  node.declare_parameter<bool>(
      "lio.loop.prefer_earliest_candidate", false);
  node.get_parameter(
      "lio.loop.prefer_earliest_candidate",
      g_loop_prefer_earliest_candidate);

  node.declare_parameter<bool>("lio.loop.online.enable", false);
  node.get_parameter("lio.loop.online.enable", g_loop_online_enable);

  node.declare_parameter<int>("lio.loop.online.interval_keyframes", 5);
  node.get_parameter(
      "lio.loop.online.interval_keyframes",
      g_loop_online_interval_keyframes);
  if (g_loop_online_interval_keyframes < 1) {
    throw std::invalid_argument(
        "lio.loop.online.interval_keyframes must be positive");
  }

  node.declare_parameter<int>("lio.loop.online.queue_capacity", 32);
  node.get_parameter(
      "lio.loop.online.queue_capacity", g_loop_online_queue_capacity);
  if (g_loop_online_queue_capacity < 4 ||
      g_loop_online_queue_capacity > 64) {
    throw std::invalid_argument(
        "lio.loop.online.queue_capacity must be in [4, 64]");
  }

  node.declare_parameter<int>("lio.loop.online.candidate_limit", 4);
  node.get_parameter(
      "lio.loop.online.candidate_limit", g_loop_online_candidate_limit);
  if (g_loop_online_candidate_limit < 1 ||
      g_loop_online_candidate_limit > 12) {
    throw std::invalid_argument(
        "lio.loop.online.candidate_limit must be in [1, 12]");
  }

  node.declare_parameter<int>("lio.loop.online.local_window_size", 8);
  node.get_parameter(
      "lio.loop.online.local_window_size",
      g_loop_online_local_window_size);
  if (g_loop_online_local_window_size < 3 ||
      g_loop_online_local_window_size > 20) {
    throw std::invalid_argument(
        "lio.loop.online.local_window_size must be in [3, 20]");
  }

  const auto read_online_loop_double = [&node](
      const char* name,
      const double default_value,
      float& value,
      const double minimum,
      const double maximum) {
    double parameter = default_value;
    node.declare_parameter<double>(name, default_value);
    node.get_parameter(name, parameter);
    if (!std::isfinite(parameter) ||
        parameter < minimum || parameter > maximum) {
      throw std::invalid_argument(
          std::string(name) + " must be finite and in [" +
          std::to_string(minimum) + ", " +
          std::to_string(maximum) + "]");
    }
    value = static_cast<float>(parameter);
  };
  read_online_loop_double(
      "lio.loop.online.search_radius", 8.0,
      g_loop_online_search_radius, 2.0, 20.0);
  read_online_loop_double(
      "lio.loop.online.voxel_size", 0.30,
      g_loop_online_voxel_size, 0.3, 2.0);

  node.declare_parameter<double>(
      "lio.loop.online.max_task_seconds", 60.0);
  node.get_parameter(
      "lio.loop.online.max_task_seconds",
      g_loop_online_max_task_seconds);
  if (!std::isfinite(g_loop_online_max_task_seconds) ||
      g_loop_online_max_task_seconds < 5.0 ||
      g_loop_online_max_task_seconds > 120.0) {
    throw std::invalid_argument(
        "lio.loop.online.max_task_seconds must be in [5, 120]");
  }

  node.declare_parameter<int>(
      "lio.loop.online.min_confirmations", 2);
  node.get_parameter(
      "lio.loop.online.min_confirmations",
      g_loop_online_min_confirmations);
  if (g_loop_online_min_confirmations < 2 ||
      g_loop_online_min_confirmations > 4) {
    throw std::invalid_argument(
        "lio.loop.online.min_confirmations must be in [2, 4]");
  }
  read_online_loop_double(
      "lio.loop.online.confirmation_translation", 0.35,
      g_loop_online_confirmation_translation, 0.05, 1.0);
  read_online_loop_double(
      "lio.loop.online.confirmation_yaw_deg", 0.50,
      g_loop_online_confirmation_yaw_deg, 0.05, 2.0);
  read_online_loop_double(
      "lio.loop.online.max_translation_step", 0.05,
      g_loop_online_max_translation_step, 0.005, 0.5);
  read_online_loop_double(
      "lio.loop.online.max_yaw_step_deg", 0.10,
      g_loop_online_max_yaw_step_deg, 0.01, 1.0);

  if (g_loop_online_enable && !g_loop_closure_enable) {
    throw std::invalid_argument(
        "lio.loop.online.enable requires lio.loop.enable=true");
  }

  node.declare_parameter<std::string>("lio.ros.lidar_topic", "/lidar");
  node.get_parameter("lio.ros.lidar_topic", g_lidar_topic);

  node.declare_parameter<std::string>("lio.ros.imu_topic", "/imu");
  node.get_parameter("lio.ros.imu_topic", g_imu_topic);

  node.declare_parameter<int>("lio.sensor.lidar_type", 0);
  node.get_parameter("lio.sensor.lidar_type", g_lidar_type);

  double temp_range_dis;
  node.declare_parameter<double>("lio.sensor.blind", 0.0);
  node.get_parameter("lio.sensor.blind", temp_range_dis);
  g_blind2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<double>("lio.sensor.maxrange", 100.0);
  node.get_parameter("lio.sensor.maxrange", temp_range_dis);
  g_maxrange2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<int>("lio.sensor.filter_rate", 1);
  node.get_parameter("lio.sensor.filter_rate", g_filter_rate);

  node.declare_parameter<bool>("lio.sensor.enable_downsample", false);
  node.get_parameter("lio.sensor.enable_downsample", g_enable_downsample);

  node.declare_parameter<double>("lio.sensor.voxel_fliter_size", 0.2);
  node.get_parameter("lio.sensor.voxel_fliter_size", g_voxel_fliter_size);

  node.declare_parameter<double>("lio.sensor.lidar_time_offset", 0.0);
  node.get_parameter("lio.sensor.lidar_time_offset", g_lidar_time_offset);

  node.declare_parameter<bool>("lio.sensor.use_query_time_undistort", true);
  node.get_parameter("lio.sensor.use_query_time_undistort", g_use_query_time_undistort);

  node.declare_parameter<double>("lio.sensor.gravity_norm", 9.81);
  node.get_parameter("lio.sensor.gravity_norm", g_gravity_norm);

  node.declare_parameter<int>("lio.sensor.imu_type", 0);
  node.get_parameter("lio.sensor.imu_type", g_imu_type);

  node.declare_parameter<double>("lio.sensor.imu_na", 0.0);
  node.get_parameter("lio.sensor.imu_na", g_imu_na);

  node.declare_parameter<double>("lio.sensor.imu_ng", 0.0);
  node.get_parameter("lio.sensor.imu_ng", g_imu_ng);

  node.declare_parameter<double>("lio.sensor.imu_nba", 0.0);
  node.get_parameter("lio.sensor.imu_nba", g_imu_nba);

  node.declare_parameter<double>("lio.sensor.imu_nbg", 0.0);
  node.get_parameter("lio.sensor.imu_nbg", g_imu_nbg);

  node.declare_parameter<int>("lio.sensor.imu_init_samples", 400);
  node.get_parameter("lio.sensor.imu_init_samples", g_imu_init_samples);
  g_imu_init_samples = std::max(100, g_imu_init_samples);

  node.declare_parameter<double>("lio.sensor.imu_init_max_gyro_norm", 0.05);
  node.get_parameter(
      "lio.sensor.imu_init_max_gyro_norm", g_imu_init_max_gyro_norm);
  g_imu_init_max_gyro_norm = std::max(0.001, g_imu_init_max_gyro_norm);

  node.declare_parameter<double>("lio.sensor.imu_init_max_gyro_stddev", 0.03);
  node.get_parameter(
      "lio.sensor.imu_init_max_gyro_stddev", g_imu_init_max_gyro_stddev);
  g_imu_init_max_gyro_stddev = std::max(0.001, g_imu_init_max_gyro_stddev);

  node.declare_parameter<double>(
      "lio.sensor.imu_init_max_accel_stddev_ratio", 0.05);
  node.get_parameter(
      "lio.sensor.imu_init_max_accel_stddev_ratio",
      g_imu_init_max_accel_stddev_ratio);
  g_imu_init_max_accel_stddev_ratio = std::clamp(
      g_imu_init_max_accel_stddev_ratio, 0.001, 1.0);

  node.declare_parameter<double>("lio.sensor.max_imu_integration_dt", 0.05);
  node.get_parameter(
      "lio.sensor.max_imu_integration_dt", g_max_imu_integration_dt);
  g_max_imu_integration_dt = std::clamp(g_max_imu_integration_dt, 0.005, 0.2);

  node.declare_parameter<double>("lio.sensor.scan_boundary_tolerance", 0.001);
  node.get_parameter(
      "lio.sensor.scan_boundary_tolerance", g_scan_boundary_tolerance);
  g_scan_boundary_tolerance = std::clamp(
      g_scan_boundary_tolerance, kSensorTimeEpsilon, 0.01);

  // ================= extrinsic =================
  std::vector<double> extrinsic_lidar_imu;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.lidar_imu", std::vector<double>(12, 0.0));
  node.get_parameter("lio.extrinsic.lidar_imu", extrinsic_lidar_imu);

  V3 __t(extrinsic_lidar_imu[0],
         extrinsic_lidar_imu[1],
         extrinsic_lidar_imu[2]);
  std::vector<scalar> r_data(9);
  for (int i = 0; i < 9; ++i) {
    r_data[i] = static_cast<scalar>(extrinsic_lidar_imu[3 + i]);
  }
  M3 __R(r_data.data());
  double lidar_imu_yaw_deg = 0.0;
  node.declare_parameter<double>("lio.extrinsic.lidar_imu_yaw_deg", 0.0);
  node.get_parameter("lio.extrinsic.lidar_imu_yaw_deg", lidar_imu_yaw_deg);
  const auto lidar_imu_correction_R =
      Eigen::AngleAxisd(lidar_imu_yaw_deg * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ());
  __R = __R * lidar_imu_correction_R.toRotationMatrix().cast<scalar>();
  g_lidar_imu = SE3(__R, __t);

  std::vector<double> extrinsic_odom_robo;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.odom_robo", std::vector<double>(6, 0.0));
  node.get_parameter("lio.extrinsic.odom_robo", extrinsic_odom_robo);

  if (extrinsic_odom_robo.size() != 6) {
    throw std::runtime_error("lio.extrinsic.odom_robo must contain exactly 6 values");
  }

  const bool odom_robo_override = node.declare_parameter<bool>(
      "lio.extrinsic.odom_robo_override", false);
  const std::array<const char *, 6> odom_robo_override_names = {
      "lio.extrinsic.odom_robo_x_m",
      "lio.extrinsic.odom_robo_y_m",
      "lio.extrinsic.odom_robo_z_m",
      "lio.extrinsic.odom_robo_roll_deg",
      "lio.extrinsic.odom_robo_pitch_deg",
      "lio.extrinsic.odom_robo_yaw_deg",
  };
  for (std::size_t index = 0; index < odom_robo_override_names.size(); ++index) {
    const double override_value = node.declare_parameter<double>(
        odom_robo_override_names[index], extrinsic_odom_robo[index]);
    if (odom_robo_override) {
      extrinsic_odom_robo[index] = override_value;
    }
  }
  if (odom_robo_override) {
    LOG(INFO) << "[SuperLIO] Applying lidar mount calibration odom_robo: ["
              << extrinsic_odom_robo[0] << ", " << extrinsic_odom_robo[1]
              << ", " << extrinsic_odom_robo[2] << ", "
              << extrinsic_odom_robo[3] << ", " << extrinsic_odom_robo[4]
              << ", " << extrinsic_odom_robo[5] << "]";
  }

  __t = V3(extrinsic_odom_robo[0],
           extrinsic_odom_robo[1],
           extrinsic_odom_robo[2]);

  auto temp_R =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                          Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[4] * M_PI / 180.0,
                          Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[3] * M_PI / 180.0,
                          Eigen::Vector3d::UnitX());

  g_odom_robo.R_ = temp_R.cast<scalar>();
  g_odom_robo.R_ = g_odom_robo.R_.transpose().eval();
  g_odom_robo = SE3(g_odom_robo.R_, __t);

  auto temp_R_yaw =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  g_lidar_robo_yaw = temp_R_yaw.cast<scalar>();

  // ================= hash map =================
  node.declare_parameter<int>("lio.hash_map.hash_capacity", 1000000);
  node.get_parameter("lio.hash_map.hash_capacity", g_ivox_capacity);

  node.declare_parameter<double>("lio.hash_map.vox_resolution", 0.5);
  node.get_parameter("lio.hash_map.vox_resolution", g_ivox_resolution);

  // kf
  node.declare_parameter<int>("lio.kf.kf_type", 0);
  node.get_parameter("lio.kf.kf_type", g_kf_type);

  node.declare_parameter<int>("lio.kf.kf_max_iterations", 0);
  node.get_parameter("lio.kf.kf_max_iterations", g_kf_max_iterations);

  node.declare_parameter<bool>("lio.kf.kf_align_gravity", false);
  node.get_parameter("lio.kf.kf_align_gravity", g_kf_align_gravity);

  node.declare_parameter<bool>("lio.kf.estimate_gravity", false);
  node.get_parameter("lio.kf.estimate_gravity", g_kf_estimate_gravity);

  node.declare_parameter<double>("lio.kf.kf_quit_eps", 0.0);
  node.get_parameter("lio.kf.kf_quit_eps", g_kf_quit_eps);

  node.declare_parameter<bool>("lio.kf.level_constraint.enable", true);
  node.get_parameter(
      "lio.kf.level_constraint.enable", g_level_constraint_enable);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.gravity_window_sec", 1.0);
  node.get_parameter(
      "lio.kf.level_constraint.gravity_window_sec", g_level_gravity_window_sec);
  g_level_gravity_window_sec = std::clamp(
      g_level_gravity_window_sec, 0.2, 5.0);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.max_accel_norm_ratio", 0.12);
  node.get_parameter(
      "lio.kf.level_constraint.max_accel_norm_ratio",
      g_level_max_accel_norm_ratio);
  g_level_max_accel_norm_ratio = std::clamp(
      g_level_max_accel_norm_ratio, 0.01, 0.5);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.max_point_range", 18.0);
  node.get_parameter(
      "lio.kf.level_constraint.max_point_range", g_level_max_point_range);
  g_level_max_point_range = std::clamp(g_level_max_point_range, 2.0, 60.0);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.min_down_distance", 0.05);
  node.get_parameter(
      "lio.kf.level_constraint.min_down_distance", g_level_min_down_distance);
  g_level_min_down_distance = std::clamp(
      g_level_min_down_distance, 0.0, 2.0);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.max_down_distance", 3.0);
  node.get_parameter(
      "lio.kf.level_constraint.max_down_distance", g_level_max_down_distance);
  g_level_max_down_distance = std::clamp(
      g_level_max_down_distance,
      g_level_min_down_distance + 0.1, 10.0);

  node.declare_parameter<int>(
      "lio.kf.level_constraint.ransac_iterations", 96);
  node.get_parameter(
      "lio.kf.level_constraint.ransac_iterations", g_level_ransac_iterations);
  g_level_ransac_iterations = std::clamp(g_level_ransac_iterations, 16, 256);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.plane_distance_threshold", 0.06);
  node.get_parameter(
      "lio.kf.level_constraint.plane_distance_threshold",
      g_level_plane_distance_threshold);
  g_level_plane_distance_threshold = std::clamp(
      g_level_plane_distance_threshold, 0.01, 0.2);

  node.declare_parameter<int>(
      "lio.kf.level_constraint.min_plane_inliers", 40);
  node.get_parameter(
      "lio.kf.level_constraint.min_plane_inliers", g_level_min_plane_inliers);
  g_level_min_plane_inliers = std::clamp(g_level_min_plane_inliers, 10, 1000);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.min_plane_inlier_ratio", 0.12);
  node.get_parameter(
      "lio.kf.level_constraint.min_plane_inlier_ratio",
      g_level_min_plane_inlier_ratio);
  g_level_min_plane_inlier_ratio = std::clamp(
      g_level_min_plane_inlier_ratio, 0.01, 0.9);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.slope_soft_start_angle_deg", 1.0);
  node.get_parameter(
      "lio.kf.level_constraint.slope_soft_start_angle_deg",
      g_level_slope_soft_start_angle_deg);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.max_plane_gravity_angle_deg", 1.5);
  node.get_parameter(
      "lio.kf.level_constraint.max_plane_gravity_angle_deg",
      g_level_max_plane_gravity_angle_deg);
  g_level_max_plane_gravity_angle_deg = std::clamp(
      g_level_max_plane_gravity_angle_deg, 0.5, 15.0);
  g_level_slope_soft_start_angle_deg = std::clamp(
      g_level_slope_soft_start_angle_deg,
      0.1,
      g_level_max_plane_gravity_angle_deg - 0.1);

  node.declare_parameter<int>(
      "lio.kf.level_constraint.slope_enter_min_frames", 5);
  node.get_parameter(
      "lio.kf.level_constraint.slope_enter_min_frames",
      g_level_slope_enter_min_frames);
  g_level_slope_enter_min_frames = std::clamp(
      g_level_slope_enter_min_frames, 2, 100);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.slope_exit_angle_deg", 0.75);
  node.get_parameter(
      "lio.kf.level_constraint.slope_exit_angle_deg",
      g_level_slope_exit_angle_deg);
  g_level_slope_exit_angle_deg = std::clamp(
      g_level_slope_exit_angle_deg,
      0.05,
      g_level_slope_soft_start_angle_deg);

  node.declare_parameter<int>(
      "lio.kf.level_constraint.slope_exit_min_frames", 10);
  node.get_parameter(
      "lio.kf.level_constraint.slope_exit_min_frames",
      g_level_slope_exit_min_frames);
  g_level_slope_exit_min_frames = std::clamp(
      g_level_slope_exit_min_frames, 2, 200);

  node.declare_parameter<int>(
      "lio.kf.level_constraint.slope_pending_max_invalid_frames", 2);
  node.get_parameter(
      "lio.kf.level_constraint.slope_pending_max_invalid_frames",
      g_level_slope_pending_max_invalid_frames);
  g_level_slope_pending_max_invalid_frames = std::clamp(
      g_level_slope_pending_max_invalid_frames, 0, 20);

  node.declare_parameter<int>(
      "lio.kf.level_constraint.slope_recovery_min_frames", 3);
  node.get_parameter(
      "lio.kf.level_constraint.slope_recovery_min_frames",
      g_level_slope_recovery_min_frames);
  g_level_slope_recovery_min_frames = std::clamp(
      g_level_slope_recovery_min_frames, 1, 20);

  node.declare_parameter<bool>(
      "lio.kf.level_constraint.slope_bounded_lease_enable", true);
  node.get_parameter(
      "lio.kf.level_constraint.slope_bounded_lease_enable",
      g_level_slope_bounded_lease_enable);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.slope_bounded_lease_max_path_m", 5.0);
  node.get_parameter(
      "lio.kf.level_constraint.slope_bounded_lease_max_path_m",
      g_level_slope_bounded_lease_max_path_m);
  g_level_slope_bounded_lease_max_path_m = std::clamp(
      g_level_slope_bounded_lease_max_path_m, 1.0, 30.0);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.slope_bounded_lease_reentry_path_m", 15.0);
  node.get_parameter(
      "lio.kf.level_constraint.slope_bounded_lease_reentry_path_m",
      g_level_slope_bounded_lease_reentry_path_m);
  g_level_slope_bounded_lease_reentry_path_m = std::clamp(
      g_level_slope_bounded_lease_reentry_path_m,
      g_level_slope_bounded_lease_max_path_m,
      100.0);

  node.declare_parameter<bool>(
      "lio.kf.level_constraint.slope_spatial_enable", false);
  node.get_parameter(
      "lio.kf.level_constraint.slope_spatial_enable",
      g_level_slope_spatial_enable);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.slope_spatial_window_m", 5.0);
  node.get_parameter(
      "lio.kf.level_constraint.slope_spatial_window_m",
      g_level_slope_spatial_window_m);
  g_level_slope_spatial_window_m = std::clamp(
      g_level_slope_spatial_window_m, 2.0, 30.0);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.slope_spatial_min_support_ratio", 0.40);
  node.get_parameter(
      "lio.kf.level_constraint.slope_spatial_min_support_ratio",
      g_level_slope_spatial_min_support_ratio);
  g_level_slope_spatial_min_support_ratio = std::clamp(
      g_level_slope_spatial_min_support_ratio, 0.10, 1.0);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.slope_spatial_max_grade_error_deg", 0.75);
  node.get_parameter(
      "lio.kf.level_constraint.slope_spatial_max_grade_error_deg",
      g_level_slope_spatial_max_grade_error_deg);
  g_level_slope_spatial_max_grade_error_deg = std::clamp(
      g_level_slope_spatial_max_grade_error_deg, 0.20, 5.0);

  node.declare_parameter<int>(
      "lio.kf.level_constraint.slope_spatial_max_mismatch_windows", 2);
  node.get_parameter(
      "lio.kf.level_constraint.slope_spatial_max_mismatch_windows",
      g_level_slope_spatial_max_mismatch_windows);
  g_level_slope_spatial_max_mismatch_windows = std::clamp(
      g_level_slope_spatial_max_mismatch_windows, 1, 10);

  node.declare_parameter<int>(
      "lio.kf.level_constraint.slope_spatial_reentry_consistent_windows", 2);
  node.get_parameter(
      "lio.kf.level_constraint.slope_spatial_reentry_consistent_windows",
      g_level_slope_spatial_reentry_consistent_windows);
  g_level_slope_spatial_reentry_consistent_windows = std::clamp(
      g_level_slope_spatial_reentry_consistent_windows, 1, 10);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.max_attitude_innovation_deg", 6.0);
  node.get_parameter(
      "lio.kf.level_constraint.max_attitude_innovation_deg",
      g_level_max_attitude_innovation_deg);
  g_level_max_attitude_innovation_deg = std::clamp(
      g_level_max_attitude_innovation_deg, 0.5, 20.0);

  node.declare_parameter<double>(
      "lio.kf.level_constraint.attitude_stddev_deg", 0.015);
  node.get_parameter(
      "lio.kf.level_constraint.attitude_stddev_deg",
      g_level_attitude_stddev_deg);
  g_level_attitude_stddev_deg = std::clamp(
      g_level_attitude_stddev_deg, 0.005, 10.0);

  node.declare_parameter<bool>(
      "lio.kf.ground_height_continuity.enable", false);
  node.get_parameter(
      "lio.kf.ground_height_continuity.enable",
      g_ground_height_continuity_enable);

  node.declare_parameter<int>(
      "lio.kf.ground_height_continuity.max_frame_gap", 5);
  node.get_parameter(
      "lio.kf.ground_height_continuity.max_frame_gap",
      g_ground_height_continuity_max_frame_gap);
  g_ground_height_continuity_max_frame_gap = std::clamp(
      g_ground_height_continuity_max_frame_gap, 1, 50);

  node.declare_parameter<double>(
      "lio.kf.ground_height_continuity.max_horizontal_step_m", 1.0);
  node.get_parameter(
      "lio.kf.ground_height_continuity.max_horizontal_step_m",
      g_ground_height_continuity_max_horizontal_step_m);
  g_ground_height_continuity_max_horizontal_step_m = std::clamp(
      g_ground_height_continuity_max_horizontal_step_m, 0.1, 5.0);

  node.declare_parameter<double>(
      "lio.kf.ground_height_continuity.max_normal_difference_deg", 5.0);
  node.get_parameter(
      "lio.kf.ground_height_continuity.max_normal_difference_deg",
      g_ground_height_continuity_max_normal_difference_deg);
  g_ground_height_continuity_max_normal_difference_deg = std::clamp(
      g_ground_height_continuity_max_normal_difference_deg, 0.5, 30.0);

  node.declare_parameter<double>(
      "lio.kf.ground_height_continuity.max_innovation_m", 0.05);
  node.get_parameter(
      "lio.kf.ground_height_continuity.max_innovation_m",
      g_ground_height_continuity_max_innovation_m);
  g_ground_height_continuity_max_innovation_m = std::clamp(
      g_ground_height_continuity_max_innovation_m, 0.01, 0.5);

  node.declare_parameter<double>(
      "lio.kf.ground_height_continuity.max_correction_per_frame_m", 0.01);
  node.get_parameter(
      "lio.kf.ground_height_continuity.max_correction_per_frame_m",
      g_ground_height_continuity_max_correction_per_frame_m);
  g_ground_height_continuity_max_correction_per_frame_m = std::clamp(
      g_ground_height_continuity_max_correction_per_frame_m,
      0.001,
      g_ground_height_continuity_max_innovation_m);

  node.declare_parameter<double>(
      "lio.kf.ground_height_continuity.max_total_correction_m", 0.30);
  node.get_parameter(
      "lio.kf.ground_height_continuity.max_total_correction_m",
      g_ground_height_continuity_max_total_correction_m);
  g_ground_height_continuity_max_total_correction_m = std::clamp(
      g_ground_height_continuity_max_total_correction_m, 0.05, 1.0);

  node.declare_parameter<double>(
      "lio.kf.ground_height_continuity.stddev_m", 0.02);
  node.get_parameter(
      "lio.kf.ground_height_continuity.stddev_m",
      g_ground_height_continuity_stddev_m);
  g_ground_height_continuity_stddev_m = std::clamp(
      g_ground_height_continuity_stddev_m, 0.002, 0.2);

  node.declare_parameter<bool>(
      "lio.kf.wall_yaw_constraint.enable", false);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.enable", g_wall_yaw_constraint_enable);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.max_point_range", 20.0);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.max_point_range",
      g_wall_yaw_max_point_range);
  g_wall_yaw_max_point_range = std::clamp(
      g_wall_yaw_max_point_range, 2.0, 100.0);

  node.declare_parameter<int>(
      "lio.kf.wall_yaw_constraint.ransac_iterations", 128);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.ransac_iterations",
      g_wall_yaw_ransac_iterations);
  g_wall_yaw_ransac_iterations = std::clamp(
      g_wall_yaw_ransac_iterations, 16, 512);

  node.declare_parameter<int>(
      "lio.kf.wall_yaw_constraint.extraction_interval_frames", 3);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.extraction_interval_frames",
      g_wall_yaw_extraction_interval_frames);
  g_wall_yaw_extraction_interval_frames = std::clamp(
      g_wall_yaw_extraction_interval_frames, 1, 30);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.plane_distance_threshold", 0.08);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.plane_distance_threshold",
      g_wall_yaw_plane_distance_threshold);
  g_wall_yaw_plane_distance_threshold = std::clamp(
      g_wall_yaw_plane_distance_threshold, 0.02, 0.3);

  node.declare_parameter<int>(
      "lio.kf.wall_yaw_constraint.min_plane_inliers", 40);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.min_plane_inliers",
      g_wall_yaw_min_plane_inliers);
  g_wall_yaw_min_plane_inliers = std::clamp(
      g_wall_yaw_min_plane_inliers, 10, 5000);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.min_plane_inlier_ratio", 0.05);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.min_plane_inlier_ratio",
      g_wall_yaw_min_plane_inlier_ratio);
  g_wall_yaw_min_plane_inlier_ratio = std::clamp(
      g_wall_yaw_min_plane_inlier_ratio, 0.005, 0.9);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.max_vertical_angle_deg", 8.0);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.max_vertical_angle_deg",
      g_wall_yaw_max_vertical_angle_deg);
  g_wall_yaw_max_vertical_angle_deg = std::clamp(
      g_wall_yaw_max_vertical_angle_deg, 1.0, 30.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.min_vertical_span", 0.8);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.min_vertical_span",
      g_wall_yaw_min_vertical_span);
  g_wall_yaw_min_vertical_span = std::clamp(
      g_wall_yaw_min_vertical_span, 0.2, 10.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.min_horizontal_span", 1.5);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.min_horizontal_span",
      g_wall_yaw_min_horizontal_span);
  g_wall_yaw_min_horizontal_span = std::clamp(
      g_wall_yaw_min_horizontal_span, 0.2, 30.0);

  node.declare_parameter<int>(
      "lio.kf.wall_yaw_constraint.reference_min_frames", 15);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.reference_min_frames",
      g_wall_yaw_reference_min_frames);
  g_wall_yaw_reference_min_frames = std::clamp(
      g_wall_yaw_reference_min_frames, 3, 500);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.reference_max_deviation_deg", 1.0);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.reference_max_deviation_deg",
      g_wall_yaw_reference_max_deviation_deg);
  g_wall_yaw_reference_max_deviation_deg = std::clamp(
      g_wall_yaw_reference_max_deviation_deg, 0.2, 15.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.reference_radius_m", 30.0);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.reference_radius_m",
      g_wall_yaw_reference_radius_m);
  g_wall_yaw_reference_radius_m = std::clamp(
      g_wall_yaw_reference_radius_m, 5.0, 200.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.reference_extension_ratio", 0.75);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.reference_extension_ratio",
      g_wall_yaw_reference_extension_ratio);
  g_wall_yaw_reference_extension_ratio = std::clamp(
      g_wall_yaw_reference_extension_ratio, 0.25, 0.95);

  node.declare_parameter<int>(
      "lio.kf.wall_yaw_constraint.max_references", 256);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.max_references",
      g_wall_yaw_max_references);
  g_wall_yaw_max_references = std::clamp(
      g_wall_yaw_max_references, 1, 4096);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.reference_min_yaw_information_ratio", 0.45);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.reference_min_yaw_information_ratio",
      g_wall_yaw_reference_min_yaw_information_ratio);
  g_wall_yaw_reference_min_yaw_information_ratio = std::clamp(
      g_wall_yaw_reference_min_yaw_information_ratio, 0.0, 1.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.yaw_information_weak_ratio", 0.40);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.yaw_information_weak_ratio",
      g_wall_yaw_information_weak_ratio);
  g_wall_yaw_information_weak_ratio = std::clamp(
      g_wall_yaw_information_weak_ratio, 0.0, 1.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.yaw_information_strong_ratio", 0.70);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.yaw_information_strong_ratio",
      g_wall_yaw_information_strong_ratio);
  g_wall_yaw_information_strong_ratio = std::clamp(
      g_wall_yaw_information_strong_ratio,
      g_wall_yaw_information_weak_ratio + 1e-4, 1.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.max_innovation_deg", 2.0);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.max_innovation_deg",
      g_wall_yaw_max_innovation_deg);
  g_wall_yaw_max_innovation_deg = std::clamp(
      g_wall_yaw_max_innovation_deg, 0.2, 30.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.stddev_deg", 0.3);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.stddev_deg", g_wall_yaw_stddev_deg);
  g_wall_yaw_stddev_deg = std::clamp(
      g_wall_yaw_stddev_deg, 0.02, 10.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.max_correction_per_frame_deg", 0.03);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.max_correction_per_frame_deg",
      g_wall_yaw_max_correction_per_frame_deg);
  g_wall_yaw_max_correction_per_frame_deg = std::clamp(
      g_wall_yaw_max_correction_per_frame_deg, 0.001, 1.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.recapture_max_innovation_deg", 8.0);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.recapture_max_innovation_deg",
      g_wall_yaw_recapture_max_innovation_deg);
  g_wall_yaw_recapture_max_innovation_deg = std::clamp(
      g_wall_yaw_recapture_max_innovation_deg,
      g_wall_yaw_max_innovation_deg + 0.1, 45.0);

  node.declare_parameter<int>(
      "lio.kf.wall_yaw_constraint.recapture_min_frames", 30);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.recapture_min_frames",
      g_wall_yaw_recapture_min_frames);
  g_wall_yaw_recapture_min_frames = std::clamp(
      g_wall_yaw_recapture_min_frames, 3, 1000);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.recapture_core_radius_ratio", 0.25);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.recapture_core_radius_ratio",
      g_wall_yaw_recapture_core_radius_ratio);
  g_wall_yaw_recapture_core_radius_ratio = std::clamp(
      g_wall_yaw_recapture_core_radius_ratio, 0.10, 0.50);

  node.declare_parameter<int>(
      "lio.kf.wall_yaw_constraint.recapture_min_reference_age_frames", 300);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.recapture_min_reference_age_frames",
      g_wall_yaw_recapture_min_reference_age_frames);
  g_wall_yaw_recapture_min_reference_age_frames = std::clamp(
      g_wall_yaw_recapture_min_reference_age_frames, 0, 100000);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.recapture_min_scene_quality", 0.80);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.recapture_min_scene_quality",
      g_wall_yaw_recapture_min_scene_quality);
  g_wall_yaw_recapture_min_scene_quality = std::clamp(
      g_wall_yaw_recapture_min_scene_quality, 0.50, 1.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.recapture_initial_max_deviation_deg", 1.5);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.recapture_initial_max_deviation_deg",
      g_wall_yaw_recapture_initial_max_deviation_deg);
  g_wall_yaw_recapture_initial_max_deviation_deg = std::clamp(
      g_wall_yaw_recapture_initial_max_deviation_deg,
      g_wall_yaw_reference_max_deviation_deg,
      std::min(g_wall_yaw_recapture_max_innovation_deg, 5.0));

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.recapture_stddev_deg", 1.5);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.recapture_stddev_deg",
      g_wall_yaw_recapture_stddev_deg);
  g_wall_yaw_recapture_stddev_deg = std::clamp(
      g_wall_yaw_recapture_stddev_deg,
      g_wall_yaw_stddev_deg, 20.0);

  node.declare_parameter<double>(
      "lio.kf.wall_yaw_constraint.recapture_max_correction_per_frame_deg",
      0.01);
  node.get_parameter(
      "lio.kf.wall_yaw_constraint.recapture_max_correction_per_frame_deg",
      g_wall_yaw_recapture_max_correction_per_frame_deg);
  g_wall_yaw_recapture_max_correction_per_frame_deg = std::clamp(
      g_wall_yaw_recapture_max_correction_per_frame_deg,
      0.0005, g_wall_yaw_max_correction_per_frame_deg);

  // submaps
  node.declare_parameter<double>("lio.submap.submap_resolution", 0.0);
  node.get_parameter("lio.submap.submap_resolution", g_submap_resolution);

  node.declare_parameter<int>("lio.submap.submap_capacity", 0);
  node.get_parameter("lio.submap.submap_capacity", g_submap_capacity);

  // visual
  node.declare_parameter<bool>("lio.output.robot", false);
  node.get_parameter("lio.output.robot", g_2_robot);

  node.declare_parameter<bool>("lio.output.planner", false);
  node.get_parameter("lio.output.planner", g_planner_enable);

  node.declare_parameter<bool>("lio.output.plan_env_world", false);
  node.get_parameter("lio.output.plan_env_world", g_2_plan_env_world);

  node.declare_parameter<bool>("lio.output.plan_env_body", false);
  node.get_parameter("lio.output.plan_env_body", g_2_plan_env_body);

  node.declare_parameter<bool>("lio.output.ml_map", false);
  node.get_parameter("lio.output.ml_map", g_2_ml_map);

  node.declare_parameter<bool>("lio.output.map", false);
  node.get_parameter("lio.output.map", g_visual_map);

  node.declare_parameter<bool>("lio.output.dense", false);
  node.get_parameter("lio.output.dense", g_visual_dense);

  node.declare_parameter<int>("lio.output.pub_step", 0);
  node.get_parameter("lio.output.pub_step", g_pub_step);

  node.declare_parameter<double>("lio.output.map_preview_ds_size", 0.2);
  double map_preview_ds_size = 0.2;
  node.get_parameter("lio.output.map_preview_ds_size", map_preview_ds_size);
  g_map_preview_ds_size = static_cast<float>(map_preview_ds_size);

  node.declare_parameter<int>("lio.output.map_preview_publish_interval", 20);
  node.get_parameter(
      "lio.output.map_preview_publish_interval", g_map_preview_publish_interval);
  g_map_preview_publish_interval = std::max(1, g_map_preview_publish_interval);

  node.declare_parameter<int>("lio.output.map_preview_max_points", 150000);
  node.get_parameter("lio.output.map_preview_max_points", g_map_preview_max_points);
  g_map_preview_max_points = std::max(1000, g_map_preview_max_points);

  node.declare_parameter<int>("lio.safety.min_effective_points", 20);
  node.get_parameter("lio.safety.min_effective_points", g_min_effective_points);
  g_min_effective_points = std::max(1, g_min_effective_points);

  node.declare_parameter<double>("lio.safety.max_frame_translation", 2.0);
  node.get_parameter("lio.safety.max_frame_translation", g_max_frame_translation);
  g_max_frame_translation = std::max(0.1, g_max_frame_translation);

  node.declare_parameter<double>("lio.safety.max_frame_rotation_deg", 45.0);
  node.get_parameter("lio.safety.max_frame_rotation_deg", g_max_frame_rotation_deg);
  g_max_frame_rotation_deg = std::clamp(g_max_frame_rotation_deg, 1.0, 180.0);

  // ================= relocation =================
  node.declare_parameter<bool>("lio.relocation.update_map", false);
  node.get_parameter("lio.relocation.update_map", g_update_map);

  std::vector<double> init_pose;
  node.declare_parameter<std::vector<double>>(
      "lio.relocation.init_pose", std::vector<double>(6, 0.0));
  node.get_parameter("lio.relocation.init_pose", init_pose);

  g_init_px    = init_pose[0];
  g_init_py    = init_pose[1];
  g_init_pz    = init_pose[2];
  g_init_roll  = init_pose[3];
  g_init_pitch = init_pose[4];
  g_init_yaw   = init_pose[5];

  node.declare_parameter<bool>("lio.relocation.anchor.enable", true);
  node.get_parameter(
      "lio.relocation.anchor.enable", g_relocation_anchor_enable);

  node.declare_parameter<int>(
      "lio.relocation.anchor.interval_frames", 10);
  node.get_parameter(
      "lio.relocation.anchor.interval_frames",
      g_relocation_anchor_interval_frames);
  g_relocation_anchor_interval_frames =
      std::max(1, g_relocation_anchor_interval_frames);

  node.declare_parameter<int>(
      "lio.relocation.anchor.window_frames", 20);
  node.get_parameter(
      "lio.relocation.anchor.window_frames",
      g_relocation_anchor_window_frames);
  g_relocation_anchor_window_frames =
      std::max(4, g_relocation_anchor_window_frames);

  node.declare_parameter<int>(
      "lio.relocation.anchor.min_frames", 8);
  node.get_parameter(
      "lio.relocation.anchor.min_frames",
      g_relocation_anchor_min_frames);
  g_relocation_anchor_min_frames = std::clamp(
      g_relocation_anchor_min_frames, 4,
      g_relocation_anchor_window_frames);

  node.declare_parameter<int>(
      "lio.relocation.anchor.max_failures", 5);
  node.get_parameter(
      "lio.relocation.anchor.max_failures",
      g_relocation_anchor_max_failures);
  g_relocation_anchor_max_failures =
      std::max(1, g_relocation_anchor_max_failures);

  const auto read_positive_relocation_double = [&node](
      const char* name, double default_value, double& value,
      double minimum, double maximum) {
    node.declare_parameter<double>(name, default_value);
    node.get_parameter(name, value);
    if (!std::isfinite(value)) {
      value = default_value;
    }
    value = std::clamp(value, minimum, maximum);
  };
  read_positive_relocation_double(
      "lio.relocation.anchor.map_radius", 35.0,
      g_relocation_anchor_map_radius, 5.0, 100.0);
  read_positive_relocation_double(
      "lio.relocation.anchor.voxel_size", 0.5,
      g_relocation_anchor_voxel_size, 0.1, 2.0);
  read_positive_relocation_double(
      "lio.relocation.anchor.max_correspondence_distance", 1.5,
      g_relocation_anchor_max_correspondence_distance, 0.2, 5.0);
  read_positive_relocation_double(
      "lio.relocation.anchor.verification_distance", 0.6,
      g_relocation_anchor_verification_distance, 0.1,
      g_relocation_anchor_max_correspondence_distance);
  read_positive_relocation_double(
      "lio.relocation.anchor.min_overlap", 0.55,
      g_relocation_anchor_min_overlap, 0.1, 0.95);
  read_positive_relocation_double(
      "lio.relocation.anchor.max_rmse", 0.35,
      g_relocation_anchor_max_rmse, 0.05, 1.0);
  read_positive_relocation_double(
      "lio.relocation.anchor.max_translation", 2.0,
      g_relocation_anchor_max_translation, 0.1, 5.0);
  read_positive_relocation_double(
      "lio.relocation.anchor.max_yaw_deg", 5.0,
      g_relocation_anchor_max_yaw_deg, 0.1, 15.0);
  read_positive_relocation_double(
      "lio.relocation.anchor.max_tilt_deg", 1.5,
      g_relocation_anchor_max_tilt_deg, 0.1, 5.0);
  read_positive_relocation_double(
      "lio.relocation.anchor.max_translation_step", 0.35,
      g_relocation_anchor_max_translation_step, 0.02,
      g_relocation_anchor_max_translation);
  read_positive_relocation_double(
      "lio.relocation.anchor.max_yaw_step_deg", 0.75,
      g_relocation_anchor_max_yaw_step_deg, 0.05,
      g_relocation_anchor_max_yaw_deg);
  read_positive_relocation_double(
      "lio.relocation.anchor.min_motion", 1.0,
      g_relocation_anchor_min_motion, 0.1, 10.0);
  read_positive_relocation_double(
      "lio.relocation.anchor.min_support_major", 4.0,
      g_relocation_anchor_min_support_major, 1.0, 20.0);
  read_positive_relocation_double(
      "lio.relocation.anchor.min_support_minor", 2.0,
      g_relocation_anchor_min_support_minor, 0.5,
      g_relocation_anchor_min_support_major);

  node.declare_parameter<int>(
      "lio.relocation.anchor.min_structural_points", 80);
  node.get_parameter(
      "lio.relocation.anchor.min_structural_points",
      g_relocation_anchor_min_structural_points);
  g_relocation_anchor_min_structural_points =
      std::max(20, g_relocation_anchor_min_structural_points);

  LOG(INFO) << GREEN << " ---> [Params]: Load from ROS2 parameter server."
            << RESET;
}


void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, CloudPtr& point_cloud){
  point_cloud->clear();
  if (msg->point_num < 2) {
    return;
  }
  CloudPtr cloud_full(new PointCloudType());
  int plsize = msg->point_num;
  cloud_full->resize(plsize);
  point_cloud->reserve(plsize);
  std::vector<bool> is_valid_pt(plsize, false);
  std::vector<std::size_t> index(plsize - 1);
  std::iota(std::begin(index), std::end(index), 1);

  std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
    if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)
    {
      // if (i % g_filter_rate == 0) 
      {
        cloud_full->at(i).x = msg->points[i].x;
        cloud_full->at(i).y = msg->points[i].y;
        cloud_full->at(i).z = msg->points[i].z;
        cloud_full->at(i).intensity = msg->points[i].reflectivity;

        if ((abs(cloud_full->at(i).x - cloud_full->at(i - 1).x) > 1e-7) ||
            (abs(cloud_full->at(i).y - cloud_full->at(i - 1).y) > 1e-7) ||
            (abs(cloud_full->at(i).z - cloud_full->at(i - 1).z) > 1e-7))
        {
          double normal_dis = cloud_full->at(i).x * cloud_full->at(i).x + 
                              cloud_full->at(i).y * cloud_full->at(i).y +
                              cloud_full->at(i).z * cloud_full->at(i).z;
          if(normal_dis > g_blind2 and normal_dis < g_maxrange2){
            is_valid_pt[i] = true;
          }
        }
      }
    }
  });

  for (int i = 1; i < plsize; i++) {
    if (is_valid_pt[i]) {
      point_cloud->points.push_back(cloud_full->at(i));
    }
  }
}


std::string lidarTypeToString(int type) {
  if (type <= 0 || type >= static_cast<int>(LID_TYPE_NAMES.size())) return "UNKNOWN";
  return LID_TYPE_NAMES[type];
}


inline bool validPoint(double x, double y, double z)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return false;

  double d2 = x * x + y * y + z * z;
  return (d2 > g_blind2 && d2 < g_maxrange2);
}


inline double stampToSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) +
         static_cast<double>(t.nanosec) * 1e-9;
}


inline builtin_interfaces::msg::Time toRosTime(double t_sec)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_sec));
  t.nanosec = static_cast<uint32_t>((t_sec - t.sec) * 1e9);
  return t;
}


constexpr double kMaxSensorClockJumpSeconds = 60.0;


ROSWrapper::ROSWrapper(const rclcpp::NodeOptions& options)
: rclcpp::Node("super_lio", options)
{
  LoadParamFromRos(*this);
  LOG(INFO) << GREEN << " ---> Using Lidar type: "
            << lidarTypeToString(g_lidar_type) << RESET;

  msg2uav_.header.frame_id = "map";
  path_.header.frame_id = "map";

  setupIO();
}


void ROSWrapper::setupIO(){
  //// input ======================================
  cb_imu_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_lidar_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_processing_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions imu_sub_opt;
  imu_sub_opt.callback_group = cb_imu_;
  rclcpp::SubscriptionOptions lidar_sub_opt;
  lidar_sub_opt.callback_group = cb_lidar_;

  const bool offline_reliable_qos =
      this->declare_parameter<bool>("lio.ros.offline_reliable_qos", false);
  pause_drain_timeout_seconds_ = this->declare_parameter<double>(
      "lio.ros.pause_drain_timeout_seconds", 5.0);
  if (pause_drain_timeout_seconds_ <= 0.0) {
    throw std::invalid_argument(
        "lio.ros.pause_drain_timeout_seconds must be positive");
  }
  imu_buffer_history_seconds_ = this->declare_parameter<double>(
      "lio.ros.imu_buffer_history_seconds", 60.0);
  if (!std::isfinite(imu_buffer_history_seconds_) ||
      imu_buffer_history_seconds_ < 5.0 ||
      imu_buffer_history_seconds_ > 600.0) {
    throw std::invalid_argument(
        "lio.ros.imu_buffer_history_seconds must be in [5, 600]");
  }
  const int imu_qos_depth =
      this->declare_parameter<int>(
          "lio.ros.imu_qos_depth", 16384);
  const int lidar_qos_depth =
      this->declare_parameter<int>(
          "lio.ros.lidar_qos_depth", offline_reliable_qos ? 1024 : 256);
  if (imu_qos_depth < 2 || lidar_qos_depth < 2) {
    throw std::invalid_argument("sensor QoS depth must be at least 2");
  }

  // Live sensors favor freshness; offline rosbag replay favors completeness.
  auto imu_qos = rclcpp::SensorDataQoS();
  imu_qos.keep_last(static_cast<std::size_t>(imu_qos_depth));

  auto lidar_qos = rclcpp::SensorDataQoS();
  lidar_qos.keep_last(static_cast<std::size_t>(lidar_qos_depth));
  if (offline_reliable_qos) {
    imu_qos.reliable();
    lidar_qos.reliable();
  }

  LOG(INFO) << GREEN
            << " ---> [SuperLIO]: sensor QoS mode="
            << (offline_reliable_qos ? "offline-reliable" : "live-best-effort")
            << " imu_depth=" << imu_qos_depth
            << " lidar_depth=" << lidar_qos_depth
            << " imu_history_seconds=" << imu_buffer_history_seconds_
            << RESET;

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      g_imu_topic,
      imu_qos,
      std::bind(&ROSWrapper::imuHandler, this, std::placeholders::_1),
      imu_sub_opt);

  if (g_lidar_type == LID_TYPE::LIVOX) {
    sub_lidar_ =
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::livoxHandler, this, std::placeholders::_1),
            lidar_sub_opt);
  } else {
    sub_lidar_std_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::stdMsgHandler, this, std::placeholders::_1),
            lidar_sub_opt);
  }

  pause_mapping_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/lio/pause_mapping",
      std::bind(
          &ROSWrapper::pauseMapping, this,
          std::placeholders::_1, std::placeholders::_2));
  mapping_status_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/lio/mapping_status",
      [this](
          const std_srvs::srv::Trigger::Request::SharedPtr,
          std_srvs::srv::Trigger::Response::SharedPtr response) {
        std::size_t pending_lidar = 0;
        bool lidar_pushed = false;
        std::uint64_t missing_imu_start = 0;
        std::uint64_t imu_gap = 0;
        std::uint64_t invalid_time = 0;
        std::uint64_t out_of_order = 0;
        std::uint64_t pruned_imu = 0;
        {
          std::lock_guard<std::mutex> lock(sensor_buffer_mutex_);
          pending_lidar = lidar_buffer_.size();
          lidar_pushed = lidar_pushed_;
          missing_imu_start = lidar_missing_imu_start_count_;
          imu_gap = lidar_imu_gap_count_;
          invalid_time = lidar_invalid_time_count_;
          out_of_order = lidar_out_of_order_count_;
          pruned_imu = pruned_imu_buffer_count_;
        }
        response->success = true;
        response->message =
            "received_lidar=" +
            std::to_string(received_lidar_count_.load()) +
            " admitted_lidar=" +
            std::to_string(admitted_lidar_count_.load()) +
            " completed_lidar=" +
            std::to_string(completed_lidar_count_.load()) +
            " dropped_lidar=" +
            std::to_string(dropped_lidar_count_.load()) +
            " missing_imu_start=" +
            std::to_string(missing_imu_start) +
            " imu_gap=" + std::to_string(imu_gap) +
            " invalid_time=" + std::to_string(invalid_time) +
            " out_of_order=" + std::to_string(out_of_order) +
            " pruned_imu=" + std::to_string(pruned_imu) +
            " pending_lidar=" + std::to_string(pending_lidar) +
            " lidar_pushed=" +
            std::string(lidar_pushed ? "true" : "false") +
            " processing=" +
            std::string(processing_measure_.load() ? "true" : "false") +
            " mapping_paused=" +
            std::string(mapping_paused_.load() ? "true" : "false") +
            " input_frozen=" +
            std::string(input_frozen_.load() ? "true" : "false");
      });

  /// output ======================================
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odom", 100);

  // Relocation publishes a continuous local odometry stream separately from
  // the existing map-frame pose topic.  Keeping /lio/odom unchanged avoids
  // breaking terrain/SCAN consumers that currently require map coordinates.
  pub_local_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/local_odom", 100);
  pub_localization_valid_ = this->create_publisher<std_msgs::msg::Bool>(
      "/lio/localization_valid", 10);

  pub_imu_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/imu/odom", 10);

  pub_robo_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/robo/odom", 10);

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
      "/lio/path", 10);
  pub_raw_compare_path_ = this->create_publisher<nav_msgs::msg::Path>(
      "/lio/path_raw_compare", 10);
  pub_online_compare_path_ = this->create_publisher<nav_msgs::msg::Path>(
      "/lio/path_online_compare", 10);

  // Both clouds are visualization/terrain inputs. Best effort with one sample
  // prevents a slow RViz or terrain reader from retaining stale large clouds
  // while the sensor and processing executors continue with current data.
  const auto realtime_cloud_qos = rclcpp::SensorDataQoS().keep_last(1);
  const auto preview_cloud_qos = rclcpp::SensorDataQoS().keep_last(1);

  pub_cloud_world_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/cloud_world", realtime_cloud_qos);

  pub_map_accumulated_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/map_accumulated", preview_cloud_qos);

  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);
}


void ROSWrapper::pauseMapping(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  // Freeze new LiDAR admission first. Frames already admitted before this
  // point define the requested sensor-time cutoff. IMU is allowed to continue
  // only until it brackets that last scan, so the cutoff frame can be
  // synchronized and processed instead of being silently discarded.
  input_frozen_.store(true);
  std::size_t pending_lidar_at_freeze = 0;
  double target_cutoff = -1.0;
  {
    std::lock_guard<std::mutex> lock(sensor_buffer_mutex_);
    if (!lidar_buffer_.empty()) {
      target_cutoff = lidar_buffer_.back().end_time;
    }
    target_cutoff = std::max(target_cutoff, last_timestamp_lidar_);
    snapshot_target_cutoff_ = target_cutoff;
    pending_lidar_at_freeze = lidar_buffer_.size();
  }

  const auto drain_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(pause_drain_timeout_seconds_));
  bool drained = false;
  std::size_t dropped_at_timeout = 0;
  {
    std::unique_lock<std::mutex> lock(sensor_buffer_mutex_);
    auto snapshot_drained = [this]() {
      return lidar_buffer_.empty() &&
             !lidar_pushed_ &&
             !processing_measure_.load();
    };
    while (!(drained = snapshot_drained()) &&
           std::chrono::steady_clock::now() < drain_deadline) {
      sensor_buffer_cv_.wait_until(lock, drain_deadline);
    }

    // Recheck and transition under the same queue mutex used by sync_measure.
    // This prevents a timer callback from claiming another frame after the
    // snapshot has been declared immutable.
    drained = snapshot_drained();
    mapping_paused_.store(true);
    if (!drained) {
      dropped_at_timeout = lidar_buffer_.size();
      dropped_lidar_count_.fetch_add(dropped_at_timeout);
      lidar_buffer_.clear();
      lidar_pushed_ = false;
    }
  }

  const double committed_cutoff = last_timestamp_lidar_;
  response->success = drained;
  response->message =
      "SuperLIO snapshot frozen; target_cutoff=" +
      std::to_string(target_cutoff) +
      " committed_cutoff=" + std::to_string(committed_cutoff) +
      " drained=" + std::string(drained ? "true" : "false") +
      " dropped_at_timeout=" + std::to_string(dropped_at_timeout) +
      " received_lidar=" +
      std::to_string(received_lidar_count_.load()) +
      " admitted_lidar=" +
      std::to_string(admitted_lidar_count_.load()) +
      " completed_lidar=" +
      std::to_string(completed_lidar_count_.load()) +
      " dropped_lidar=" +
      std::to_string(dropped_lidar_count_.load()) +
      " missing_imu_start=" +
      std::to_string(lidar_missing_imu_start_count_) +
      " imu_gap=" + std::to_string(lidar_imu_gap_count_) +
      " invalid_time=" + std::to_string(lidar_invalid_time_count_) +
      " out_of_order=" + std::to_string(lidar_out_of_order_count_) +
      " pruned_imu=" + std::to_string(pruned_imu_buffer_count_) +
      " source_gaps=" + std::to_string(lidar_source_gap_count_) +
      " estimated_missing_lidar=" +
      std::to_string(lidar_estimated_missing_count_);
  LOG(INFO) << (drained ? GREEN : YELLOW)
            << " ---> [SuperLIO]: mapping snapshot frozen. "
            << "target_cutoff=" << target_cutoff
            << " committed_cutoff=" << committed_cutoff
            << " pending_lidar_at_freeze=" << pending_lidar_at_freeze
            << " drained=" << drained
            << " dropped_at_timeout=" << dropped_at_timeout
            << " received_lidar=" << received_lidar_count_.load()
            << " admitted_lidar=" << admitted_lidar_count_.load()
            << " completed_lidar=" << completed_lidar_count_.load()
            << " dropped_lidar=" << dropped_lidar_count_.load()
            << " missing_imu_start=" << lidar_missing_imu_start_count_
            << " imu_gap=" << lidar_imu_gap_count_
            << " invalid_time=" << lidar_invalid_time_count_
            << " out_of_order=" << lidar_out_of_order_count_
            << " pruned_imu=" << pruned_imu_buffer_count_
            << " source_gaps=" << lidar_source_gap_count_
            << " estimated_missing_lidar=" << lidar_estimated_missing_count_
            << RESET;
}


void ROSWrapper::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg){
  if (mapping_paused_.load()) return;

  IMUData data;
  data.secs = stampToSec(msg->header.stamp);
  data.acc  = V3(msg->linear_acceleration.x,
                 msg->linear_acceleration.y,
                 msg->linear_acceleration.z);
  data.gyr  = V3(msg->angular_velocity.x,
                 msg->angular_velocity.y,
                 msg->angular_velocity.z);

  // Keep this callback intentionally lightweight. The ESKF is owned by the
  // LIO processing callback; running forward prediction here would create a
  // data race once sensor ingestion and scan matching use separate threads.
  std::lock_guard<std::mutex> lock(sensor_buffer_mutex_);

  if (mapping_paused_.load() ||
      (input_frozen_.load() && snapshot_target_cutoff_ >= 0.0 &&
       last_timestamp_imu_ + kSensorTimeEpsilon >=
           snapshot_target_cutoff_)) {
    return;
  }

  if (last_timestamp_imu_ >= 0.0) {
    const double timestamp_delta = data.secs - last_timestamp_imu_;
    if (timestamp_delta < 0.0 || timestamp_delta > kMaxSensorClockJumpSeconds) {
      rejected_imu_clock_count_++;
      const auto warning_time = std::chrono::steady_clock::now();
      if (last_clock_domain_warning_time_ == std::chrono::steady_clock::time_point{} ||
          warning_time - last_clock_domain_warning_time_ > std::chrono::seconds(1)) {
        last_clock_domain_warning_time_ = warning_time;
        LOG(WARNING) << YELLOW
                     << " ---> [SuperLIO]: reject IMU from inconsistent clock. dt="
                     << timestamp_delta << "s rejected="
                     << rejected_imu_clock_count_ << RESET;
      }
      return;
    }
  }

  const double imu_stamp_lag = this->now().seconds() - data.secs;
  const auto warning_time = std::chrono::steady_clock::now();
  if (imu_stamp_lag > 0.1 &&
      (last_imu_lag_warning_time_ == std::chrono::steady_clock::time_point{} ||
       warning_time - last_imu_lag_warning_time_ > std::chrono::seconds(1))) {
    last_imu_lag_warning_time_ = warning_time;
    LOG(WARNING) << YELLOW
                 << " ---> [SuperLIO]: IMU timestamp is stale by "
                 << imu_stamp_lag
                 << "s; LiDAR synchronization will wait. imu_buffer="
                 << imu_buffer_.size() << RESET;
  }

  imu_buffer_.push_back(data);
  last_timestamp_imu_ = data.secs;
  sensor_buffer_cv_.notify_all();

  // Keep a generous sensor-time history, and never prune past the oldest
  // LiDAR frame already waiting for scan matching. A slow disk write or map
  // preview must not turn frontend backlog into an unrecoverable IMU gap.
  double prune_before = data.secs - imu_buffer_history_seconds_;
  if (!lidar_buffer_.empty() &&
      std::isfinite(lidar_buffer_.front().start_time)) {
    prune_before = std::min(prune_before, lidar_buffer_.front().start_time);
  }
  // Retain the last sample at/before prune_before so scan-start interpolation
  // always has a historical anchor.
  while (imu_buffer_.size() > 2 &&
         imu_buffer_[1].secs <= prune_before) {
    imu_buffer_.pop_front();
    ++pruned_imu_buffer_count_;
  }
}


void ROSWrapper::livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
  if (mapping_paused_.load() || input_frozen_.load()) return;
  if(msg->point_num < 10) return;

  ++received_lidar_count_;
  const auto arrival_time = std::chrono::steady_clock::now();
  const double source_time = stampToSec(msg->header.stamp);
  if (last_lidar_source_time_ > 0.0) {
    const double timestamp_delta = source_time - last_lidar_source_time_;
    if (timestamp_delta < 0.0 || timestamp_delta > kMaxSensorClockJumpSeconds) {
      rejected_lidar_clock_count_++;
      const auto warning_time = std::chrono::steady_clock::now();
      if (last_clock_domain_warning_time_ == std::chrono::steady_clock::time_point{} ||
          warning_time - last_clock_domain_warning_time_ > std::chrono::seconds(1)) {
        last_clock_domain_warning_time_ = warning_time;
        LOG(WARNING) << YELLOW
                     << " ---> [SuperLIO]: reject LiDAR from inconsistent clock. dt="
                     << timestamp_delta << "s rejected="
                     << rejected_lidar_clock_count_ << RESET;
      }
      return;
    }
  }
  if (last_lidar_source_time_ > 0.0 &&
      last_lidar_arrival_time_ != std::chrono::steady_clock::time_point{}) {
    const double source_interval = source_time - last_lidar_source_time_;
    const double arrival_interval = std::chrono::duration<double>(
        arrival_time - last_lidar_arrival_time_).count();
    if (source_interval >= 0.08 && source_interval <= 0.12 &&
        arrival_interval > 0.2) {
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: lidar callback delayed. source_dt="
                   << source_interval << "s arrival_dt=" << arrival_interval << "s"
                   << RESET;
    } else if (source_interval < 0.08 || source_interval > 0.12) {
      ++lidar_source_gap_count_;
      if (source_interval > 0.12) {
        const auto expected_frames =
            static_cast<std::uint64_t>(std::llround(source_interval / 0.1));
        if (expected_frames > 1) {
          lidar_estimated_missing_count_ += expected_frames - 1;
        }
      }
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: lidar source timestamp gap. source_dt="
                   << source_interval << "s arrival_dt=" << arrival_interval << "s"
                   << " estimated_missing_total=" << lidar_estimated_missing_count_
                   << RESET;
    }
  }
  last_lidar_source_time_ = source_time;
  last_lidar_arrival_time_ = arrival_time;

  LidarData lidar_data;
  std::size_t ptsize = msg->point_num;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(ptsize / g_filter_rate + 1);

  double offset_time = 0.0;
  for(std::size_t _i = 0; _i < ptsize; _i += g_filter_rate){
    auto& pt = msg->points[_i];
    auto tag = pt.tag & 0x30;
    if (tag == 0x10 || tag == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2 && dis < g_maxrange2){
        const double point_offset_time = pt.offset_time * 1e-9;
        if (!std::isfinite(point_offset_time) || point_offset_time < 0.0) {
          continue;
        }
        offset_time = std::max(offset_time, point_offset_time);
        lidar_data.pc->emplace_back(
            pt.x, pt.y, pt.z, pt.reflectivity, point_offset_time);
      }
    }
  }
  if (lidar_data.pc->empty()) {
    return;
  }
  lidar_data.start_time = source_time + g_lidar_time_offset;
  lidar_data.end_time   = lidar_data.start_time + offset_time;
  {
    std::lock_guard<std::mutex> lock(sensor_buffer_mutex_);
    if (mapping_paused_.load() || input_frozen_.load()) {
      return;
    }
    lidar_buffer_.push_back(std::move(lidar_data));
    admitted_lidar_count_.fetch_add(1);
    sensor_buffer_cv_.notify_all();
  }
}


void ROSWrapper::stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if (mapping_paused_.load() || input_frozen_.load()) return;
  if(msg->data.size() < 10) return;

  received_lidar_count_.fetch_add(1);
  
  LidarData lidar_data;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());

  double offset_time = 0.0;
  double dis = 0.0;

  switch (g_lidar_type) {

  case LID_TYPE::HESAI16:
  {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    const double time_begin = pl_orig.points[0].timestamp;
    lidar_data.start_time = time_begin + g_lidar_time_offset;
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate)
    {
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.timestamp - time_begin;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::VEL_NCLT:
  {
    pcl::PointCloud<NCLT::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp) + g_lidar_time_offset;
    
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.time * 1e-6;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, 1.0, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::VELO16:
  case LID_TYPE::VELO32:
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp) + g_lidar_time_offset;

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, pt.time);
    }
    lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time;
    break;
  }
  case OUSTER:
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp) + g_lidar_time_offset;

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.t * 1e-9;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  default:
    return;
  }
  
  {
    std::lock_guard<std::mutex> lock(sensor_buffer_mutex_);
    if (mapping_paused_.load() || input_frozen_.load()) {
      return;
    }
    lidar_buffer_.push_back(std::move(lidar_data));
    admitted_lidar_count_.fetch_add(1);
    sensor_buffer_cv_.notify_all();
  }
}


bool ROSWrapper::sync_measure(MeasureGroup& meas){
  if (mapping_paused_.load()) {
    return false;
  }

  // Only queue inspection/copying is protected. sync_measure returns before
  // expensive undistortion and scan matching, so sensor callbacks remain
  // responsive while the LIO thread processes the copied MeasureGroup.
  std::lock_guard<std::mutex> lock(sensor_buffer_mutex_);

  if (mapping_paused_.load()) {
    return false;
  }

  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  auto record_sync_result = [this](bool dropped) {
    const auto now = std::chrono::steady_clock::now();
    if (sync_health_window_start_ == std::chrono::steady_clock::time_point{}) {
      sync_health_window_start_ = now;
    }
    ++sync_window_completed_count_;
    if (dropped) {
      ++sync_window_dropped_count_;
    }

    const double window_seconds = std::chrono::duration<double>(
        now - sync_health_window_start_).count();
    if (window_seconds < 2.0 || sync_window_completed_count_ < 10) {
      return;
    }

    const double drop_ratio = static_cast<double>(sync_window_dropped_count_) /
        static_cast<double>(sync_window_completed_count_);
    const bool unhealthy = drop_ratio > 0.05;
    if (unhealthy) {
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: LiDAR/IMU synchronization unhealthy. "
                   << "drop_ratio=" << drop_ratio * 100.0 << "% ("
                   << sync_window_dropped_count_ << "/"
                   << sync_window_completed_count_
                   << "); mapping continues without automatic pause."
                   << RESET;
    } else if (sync_health_unhealthy_) {
      LOG(INFO) << GREEN
                << " ---> [SuperLIO]: LiDAR/IMU synchronization recovered. "
                << "drop_ratio=" << drop_ratio * 100.0 << "%"
                << RESET;
    }
    sync_health_unhealthy_ = unhealthy;
    sync_window_completed_count_ = 0;
    sync_window_dropped_count_ = 0;
    sync_health_window_start_ = now;
  };

  if (!lidar_pushed_) {
    meas.lidar = lidar_buffer_.front();
    lidar_pushed_ = true;
  }

  if(last_timestamp_lidar_ > meas.lidar.end_time){
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    ++lidar_out_of_order_count_;
    dropped_lidar_count_.fetch_add(1);
    sensor_buffer_cv_.notify_all();
    return false;
  }

  auto drop_lidar_for_imu =
    [this, &record_sync_result](std::uint64_t& reason_count, const char* reason,
           const std::string& details) {
      ++lidar_without_imu_count_;
      ++reason_count;
      const auto warning_time = std::chrono::steady_clock::now();
      if (last_missing_imu_warning_time_ == std::chrono::steady_clock::time_point{} ||
          warning_time - last_missing_imu_warning_time_ > std::chrono::seconds(1)) {
        last_missing_imu_warning_time_ = warning_time;
        LOG(WARNING) << YELLOW
                     << " ---> [SuperLIO]: drop LiDAR frame: " << reason
                     << ". " << details
                     << " dropped_total=" << lidar_without_imu_count_
                     << " reason_count=" << reason_count << RESET;
      }
      lidar_buffer_.pop_front();
      lidar_pushed_ = false;
      dropped_lidar_count_.fetch_add(1);
      record_sync_result(true);
      sensor_buffer_cv_.notify_all();
      return false;
    };

  const double lidar_start = meas.lidar.start_time;
  const double lidar_end = meas.lidar.end_time;
  if (!std::isfinite(lidar_start) || !std::isfinite(lidar_end) ||
      lidar_end + kSensorTimeEpsilon < lidar_start) {
    std::ostringstream details;
    details << std::fixed << std::setprecision(6)
            << "lidar_start=" << lidar_start << " lidar_end=" << lidar_end;
    return drop_lidar_for_imu(
      lidar_invalid_time_count_, "invalid LiDAR time range", details.str());
  }

  // A future IMU sample is required so the state can be interpolated exactly
  // at the scan end. Waiting here is safe: this is not an error or a drop.
  if (last_timestamp_imu_ + kSensorTimeEpsilon < lidar_end) {
    return false;
  }

  // Find one IMU sample at/before the scan start. Livox frame construction can
  // make adjacent scans overlap by a few tenths of a millisecond; accept that
  // bounded overlap instead of discarding almost every other frame.
  auto start_upper = std::lower_bound(
    imu_buffer_.begin(), imu_buffer_.end(), lidar_start,
    [](const IMUData& imu, double time) { return imu.secs < time; });

  if (start_upper == imu_buffer_.begin() &&
      start_upper->secs > lidar_start + g_scan_boundary_tolerance) {
    std::ostringstream details;
    details << std::fixed << std::setprecision(6)
            << "lidar_start=" << lidar_start
            << " oldest_imu=" << start_upper->secs
            << " delta=" << start_upper->secs - lidar_start
            << " tolerance=" << g_scan_boundary_tolerance;
    return drop_lidar_for_imu(
      lidar_missing_imu_start_count_, "missing IMU at scan start", details.str());
  }

  auto scan_start_anchor = start_upper;
  if (start_upper == imu_buffer_.end()) {
    scan_start_anchor = std::prev(start_upper);
  } else if (start_upper != imu_buffer_.begin() &&
             start_upper->secs > lidar_start + kSensorTimeEpsilon) {
    scan_start_anchor = std::prev(start_upper);
  }

  // Once mapping has accepted a scan, the queue front is the exact synthetic
  // IMU boundary retained for that scan end. Integrate from there so a missing
  // LiDAR frame does not also discard the intervening (valid) IMU motion.
  const auto integration_anchor = last_timestamp_lidar_ >= 0.0
    ? imu_buffer_.begin()
    : scan_start_anchor;

  auto end_upper = std::lower_bound(
    integration_anchor, imu_buffer_.end(), lidar_end,
    [](const IMUData& imu, double time) { return imu.secs < time; });
  if (end_upper == imu_buffer_.end()) {
    // last_timestamp_imu_ may already be newer while its callback has not yet
    // populated this buffer in an unusual executor schedule. Wait, do not drop.
    return false;
  }

  // Complete coverage is not enough: every integration interval must also be
  // continuous. A single large hole would otherwise distort the scan even
  // though samples exist at both ends.
  for (auto previous = integration_anchor, current = std::next(integration_anchor);
       current != std::next(end_upper); ++previous, ++current) {
    const double dt = current->secs - previous->secs;
    if (!std::isfinite(dt) || dt <= 0.0 ||
        dt > g_max_imu_integration_dt + kSensorTimeEpsilon) {
      std::ostringstream details;
      details << std::fixed << std::setprecision(6)
              << "gap_start=" << previous->secs
              << " gap_end=" << current->secs
              << " dt=" << dt
              << " limit=" << g_max_imu_integration_dt;

      // The samples before this discontinuity can never provide continuous
      // coverage for a later scan. Leaving them at the queue front makes every
      // subsequent LiDAR frame rediscover the same historical gap and be
      // dropped until the generic three-second queue retention finally evicts
      // it. During that interval map->base_footprint is stale; when processing
      // resumes SCAN sees the accumulated correction as a pose jump.
      //
      // Keep the first sample after the gap as the new integration anchor. The
      // current LiDAR frame is still rejected, and ESKF::Predict independently
      // refuses to integrate the missing interval. A following scan can then
      // resume from real post-gap IMU samples instead of replaying the defect.
      const std::size_t discarded_imu_samples =
          static_cast<std::size_t>(
              std::distance(imu_buffer_.begin(), current));
      const double recovery_anchor_time = current->secs;
      imu_buffer_.erase(imu_buffer_.begin(), current);
      details << " recovery_anchor=" << recovery_anchor_time
              << " discarded_imu=" << discarded_imu_samples;
      return drop_lidar_for_imu(
        lidar_imu_gap_count_, "discontinuous IMU coverage", details.str());
    }
  }

  meas.imu.clear();
  for (auto current = integration_anchor; current != end_upper; ++current) {
    meas.imu.push_back(*current);
  }

  const bool exact_end =
    std::abs(end_upper->secs - lidar_end) <= kSensorTimeEpsilon;
  IMUData end_sample;
  if (exact_end) {
    end_sample = *end_upper;
    end_sample.secs = lidar_end;
  } else {
    if (end_upper == integration_anchor) {
      std::ostringstream details;
      details << std::fixed << std::setprecision(6)
              << "lidar_start=" << lidar_start
              << " lidar_end=" << lidar_end
              << " first_imu=" << end_upper->secs;
      return drop_lidar_for_imu(
        lidar_missing_imu_start_count_, "cannot interpolate scan end", details.str());
    }
    end_sample = interpolateImu(*std::prev(end_upper), *end_upper, lidar_end);
  }
  meas.imu.push_back(end_sample);

  // Retain an exact boundary sample at the front of the queue. The next scan
  // can then prove beginning coverage instead of starting after this frame.
  imu_buffer_.erase(imu_buffer_.begin(), end_upper);
  if (exact_end) {
    imu_buffer_.front().secs = lidar_end;
  } else {
    imu_buffer_.push_front(end_sample);
  }

  last_timestamp_lidar_ = lidar_end;
  lidar_buffer_.pop_front();
  lidar_pushed_ = false;
  record_sync_result(false);
  processing_measure_.store(true);
  sensor_buffer_cv_.notify_all();

  const double processing_lag = this->now().seconds() - meas.lidar.end_time;
  const auto warning_time = std::chrono::steady_clock::now();
  if (processing_lag > 0.5 &&
      (last_lag_warning_time_ == std::chrono::steady_clock::time_point{} ||
       warning_time - last_lag_warning_time_ > std::chrono::seconds(1))) {
    last_lag_warning_time_ = warning_time;
    LOG(WARNING) << YELLOW
                 << " ---> [SuperLIO]: processing backlog. lag=" << processing_lag
                 << "s lidar_buffer=" << lidar_buffer_.size()
                 << " imu_buffer=" << imu_buffer_.size() << RESET;
  }
  return true;
}


void ROSWrapper::finish_measure()
{
  {
    std::lock_guard<std::mutex> lock(sensor_buffer_mutex_);
    completed_lidar_count_.fetch_add(1);
    processing_measure_.store(false);
  }
  sensor_buffer_cv_.notify_all();
}


void ROSWrapper::pub_odom(const NavState& state){
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";

  odom.header.stamp = toRosTime(state.timestamp);
  odom.pose.pose.position.x = state.p[0];
  odom.pose.pose.position.y = state.p[1];
  odom.pose.pose.position.z = state.p[2];

  V4 temp_q = state.R.coeffs();
  odom.pose.pose.orientation.x = temp_q[0];
  odom.pose.pose.orientation.y = temp_q[1];
  odom.pose.pose.orientation.z = temp_q[2];
  odom.pose.pose.orientation.w = temp_q[3];

  odom.twist.twist.linear.x = state.v[0];
  odom.twist.twist.linear.y = state.v[1];
  odom.twist.twist.linear.z = state.v[2];

  pub_odom_->publish(odom);    // imu frame -> lidar frequency

  V3 robo_position = state.R.R_ * ( - g_odom_robo.R_ * g_odom_robo.t_) + state.p;

  if(g_2_robot){
    static auto pub_msg2uav_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    M3 robo_rotation = state.R.R_ * g_odom_robo.R_;
    msg2uav_.header.stamp = odom.header.stamp;
    msg2uav_.pose.position.x = robo_position[0];
    msg2uav_.pose.position.y = robo_position[1];
    msg2uav_.pose.position.z = robo_position[2];
    Quat robo_quat(robo_rotation);
    msg2uav_.pose.orientation.w = robo_quat.w();
    msg2uav_.pose.orientation.x = robo_quat.x();
    msg2uav_.pose.orientation.y = robo_quat.y();
    msg2uav_.pose.orientation.z = robo_quat.z();
    pub_msg2uav_->publish(msg2uav_);
  }

  if((last_path_point_ - robo_position).norm() > 0.1)
  {
    path_.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped point;
    point.pose = odom.pose.pose;
    path_.poses.push_back(point);
    pub_path_->publish(path_);
    last_path_point_ = robo_position;
  }

  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = odom.header.stamp;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";

  tf_msg.transform.translation.x = state.p[0];
  tf_msg.transform.translation.y = state.p[1];
  tf_msg.transform.translation.z = state.p[2];

  tf_msg.transform.rotation.x = temp_q.x();
  tf_msg.transform.rotation.y = temp_q.y();
  tf_msg.transform.rotation.z = temp_q.z();
  tf_msg.transform.rotation.w = temp_q.w();

  tf_broadcaster_->sendTransform(tf_msg);

  // tf_msg.child_frame_id = "god";
  // tf_msg.transform.rotation.x = 0.0;
  // tf_msg.transform.rotation.y = 0.0;
  // tf_msg.transform.rotation.z = 0.0;
  // tf_msg.transform.rotation.w = 1.0;
  // tf_broadcaster_->sendTransform(tf_msg);

}


void ROSWrapper::pub_relocation_odom(
    const NavState& map_state,
    const SE3& odom_pose,
    const SE3& map_to_odom,
    const bool localization_valid)
{
  const auto stamp = toRosTime(map_state.timestamp);

  // Backward-compatible global pose.  Terrain analysis and SCAN currently
  // consume /lio/odom together with /lio/cloud_world, both in map.
  nav_msgs::msg::Odometry map_odom;
  map_odom.header.stamp = stamp;
  map_odom.header.frame_id = "map";
  map_odom.child_frame_id = "base_link";
  map_odom.pose.pose.position.x = map_state.p.x();
  map_odom.pose.pose.position.y = map_state.p.y();
  map_odom.pose.pose.position.z = map_state.p.z();
  const Quat map_base_quaternion =
      Quat(map_state.R.R_).normalized();
  map_odom.pose.pose.orientation.x = map_base_quaternion.x();
  map_odom.pose.pose.orientation.y = map_base_quaternion.y();
  map_odom.pose.pose.orientation.z = map_base_quaternion.z();
  map_odom.pose.pose.orientation.w = map_base_quaternion.w();
  map_odom.twist.twist.linear.x = map_state.v.x();
  map_odom.twist.twist.linear.y = map_state.v.y();
  map_odom.twist.twist.linear.z = map_state.v.z();
  pub_odom_->publish(map_odom);

  // Standards-compliant continuous local odometry.  Pose is odom->base_link;
  // twist is expressed in the child/base frame as required by Odometry.
  nav_msgs::msg::Odometry local_odom;
  local_odom.header.stamp = stamp;
  local_odom.header.frame_id = "odom";
  local_odom.child_frame_id = "base_link";
  local_odom.pose.pose.position.x = odom_pose.t_.x();
  local_odom.pose.pose.position.y = odom_pose.t_.y();
  local_odom.pose.pose.position.z = odom_pose.t_.z();
  const Quat odom_base_quaternion =
      Quat(odom_pose.R_).normalized();
  local_odom.pose.pose.orientation.x = odom_base_quaternion.x();
  local_odom.pose.pose.orientation.y = odom_base_quaternion.y();
  local_odom.pose.pose.orientation.z = odom_base_quaternion.z();
  local_odom.pose.pose.orientation.w = odom_base_quaternion.w();
  const V3 base_velocity = map_state.R.R_.transpose() * map_state.v;
  local_odom.twist.twist.linear.x = base_velocity.x();
  local_odom.twist.twist.linear.y = base_velocity.y();
  local_odom.twist.twist.linear.z = base_velocity.z();
  pub_local_odom_->publish(local_odom);

  std_msgs::msg::Bool health;
  health.data = localization_valid;
  pub_localization_valid_->publish(health);

  V3 robo_position =
      map_state.R.R_ * (-g_odom_robo.R_ * g_odom_robo.t_) + map_state.p;
  if (g_2_robot) {
    static auto pub_msg2uav_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10);
    const M3 robo_rotation = map_state.R.R_ * g_odom_robo.R_;
    const Quat robo_quaternion = Quat(robo_rotation).normalized();
    msg2uav_.header.stamp = stamp;
    msg2uav_.pose.position.x = robo_position.x();
    msg2uav_.pose.position.y = robo_position.y();
    msg2uav_.pose.position.z = robo_position.z();
    msg2uav_.pose.orientation.x = robo_quaternion.x();
    msg2uav_.pose.orientation.y = robo_quaternion.y();
    msg2uav_.pose.orientation.z = robo_quaternion.z();
    msg2uav_.pose.orientation.w = robo_quaternion.w();
    pub_msg2uav_->publish(msg2uav_);
  }

  if ((last_path_point_ - robo_position).norm() > 0.1) {
    path_.header.stamp = stamp;
    geometry_msgs::msg::PoseStamped point;
    point.header = map_odom.header;
    point.pose = map_odom.pose.pose;
    path_.poses.push_back(point);
    pub_path_->publish(path_);
    last_path_point_ = robo_position;
  }

  // Lightweight visualization-only comparison.  Both messages deliberately
  // use map as their frame: red keeps untouched LIO coordinates, while green
  // is replaced by the optimized keyframe path whenever an online loop is
  // accepted.  A one-metre spacing keeps RViz cheap on Jetson.
  const V3 raw_compare_position = odom_pose.t_;
  if ((last_raw_compare_path_point_ - raw_compare_position).norm() > 1.0) {
    raw_compare_path_.header = map_odom.header;
    geometry_msgs::msg::PoseStamped point;
    point.header = map_odom.header;
    point.pose = local_odom.pose.pose;
    raw_compare_path_.poses.push_back(point);
    pub_raw_compare_path_->publish(raw_compare_path_);
    last_raw_compare_path_point_ = raw_compare_position;
  }
  const V3 online_compare_position = map_state.p;
  if ((last_online_compare_path_point_ - online_compare_position).norm() > 1.0) {
    online_compare_path_.header = map_odom.header;
    geometry_msgs::msg::PoseStamped point;
    point.header = map_odom.header;
    point.pose = map_odom.pose.pose;
    online_compare_path_.poses.push_back(point);
    pub_online_compare_path_->publish(online_compare_path_);
    last_online_compare_path_point_ = online_compare_position;
  }

  geometry_msgs::msg::TransformStamped map_to_odom_tf;
  map_to_odom_tf.header.stamp = stamp;
  map_to_odom_tf.header.frame_id = "map";
  map_to_odom_tf.child_frame_id = "odom";
  map_to_odom_tf.transform.translation.x = map_to_odom.t_.x();
  map_to_odom_tf.transform.translation.y = map_to_odom.t_.y();
  map_to_odom_tf.transform.translation.z = map_to_odom.t_.z();
  const Quat map_odom_quaternion =
      Quat(map_to_odom.R_).normalized();
  map_to_odom_tf.transform.rotation.x = map_odom_quaternion.x();
  map_to_odom_tf.transform.rotation.y = map_odom_quaternion.y();
  map_to_odom_tf.transform.rotation.z = map_odom_quaternion.z();
  map_to_odom_tf.transform.rotation.w = map_odom_quaternion.w();

  geometry_msgs::msg::TransformStamped odom_to_base_tf;
  odom_to_base_tf.header.stamp = stamp;
  odom_to_base_tf.header.frame_id = "odom";
  odom_to_base_tf.child_frame_id = "base_link";
  odom_to_base_tf.transform.translation.x = odom_pose.t_.x();
  odom_to_base_tf.transform.translation.y = odom_pose.t_.y();
  odom_to_base_tf.transform.translation.z = odom_pose.t_.z();
  odom_to_base_tf.transform.rotation.x = odom_base_quaternion.x();
  odom_to_base_tf.transform.rotation.y = odom_base_quaternion.y();
  odom_to_base_tf.transform.rotation.z = odom_base_quaternion.z();
  odom_to_base_tf.transform.rotation.w = odom_base_quaternion.w();

  tf_broadcaster_->sendTransform({map_to_odom_tf, odom_to_base_tf});
}


void ROSWrapper::pub_cloud_world(const CloudPtr& pc, double time){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "map";
  cloud.header.stamp = toRosTime(time);
  pub_cloud_world_->publish(cloud);
}


void ROSWrapper::pub_map_accumulated(const CloudPtr& pc, double time){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "map";
  cloud.header.stamp = toRosTime(time);
  pub_map_accumulated_->publish(cloud);
}


void ROSWrapper::pub_online_loop_paths(
    const VV3& raw_path,
    const VV3& corrected_path,
    const double time){
  const auto stamp = toRosTime(time);
  const auto build_display_path = [&](const VV3& positions) {
    nav_msgs::msg::Path message;
    message.header.frame_id = "map";
    message.header.stamp = stamp;
    V3 last = V3::Constant(std::numeric_limits<scalar>::infinity());
    for (std::size_t index = 0; index < positions.size(); ++index) {
      const bool endpoint = index + 1U == positions.size();
      if (!message.poses.empty() && !endpoint &&
          (positions[index] - last).norm() < 1.0) {
        continue;
      }
      geometry_msgs::msg::PoseStamped point;
      point.header = message.header;
      point.pose.position.x = positions[index].x();
      point.pose.position.y = positions[index].y();
      point.pose.position.z = positions[index].z();
      point.pose.orientation.w = 1.0;
      message.poses.push_back(point);
      last = positions[index];
    }
    return message;
  };

  raw_compare_path_ = build_display_path(raw_path);
  online_compare_path_ = build_display_path(corrected_path);
  if (!raw_path.empty()) {
    last_raw_compare_path_point_ = raw_path.back();
  }
  if (!corrected_path.empty()) {
    last_online_compare_path_point_ = corrected_path.back();
  }
  pub_raw_compare_path_->publish(raw_compare_path_);
  pub_online_compare_path_->publish(online_compare_path_);
}


void ROSWrapper::pub_cloud2planner(const CloudPtr& pc, double time){
  static auto pub_cloud2robot_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/robo/cloud_world", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "map";
  cloud.header.stamp = toRosTime(time);
  pub_cloud2robot_->publish(cloud);
}


void ROSWrapper::pub_cloud_body_pose(const CloudPtr& pc, 
  const NavState& state)
{
  static auto pub_cloud_body_pose_ =
    this->create_publisher<nav_lio::msg::CloudPose>(
        "/lio/body/cloud_pose", 10);
  nav_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp); 
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];

  pub_cloud_body_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_cloud_world_pose(const CloudPtr& pc, 
   const NavState& state)
{
  static auto pub_cloud_world_pose_ =
    this->create_publisher<nav_lio::msg::CloudPose>(
        "/lio/world/cloud_pose", 10);
  nav_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp);  
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];
  pub_cloud_world_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_processing_time(double time, 
  double current_time, double mean_time, double std_time)
{
  static auto pub_processing_time_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lio/processing_time", 10);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = toRosTime(time);
  msg.pose.position.x = current_time;
  msg.pose.position.y = mean_time;
  msg.pose.position.z = std_time;
  pub_processing_time_->publish(msg);
}


void ROSWrapper::set_global_map(const BASIC::CloudPtr& global_map){
  pcl::toROSMsg(*global_map, global_map_msg_);
  global_map_msg_.header.frame_id = "map";

  static auto global_map_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/lio/global_map", 10);

  static auto global_map_timer =
    this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        static int count = -1;
        static int publish_interval = 1;

        count++;
        if (count % publish_interval != 0) {
          return;
        }

        count = 0;
        publish_interval++;
        if (publish_interval > 10) {
          publish_interval = 10;
        }
        global_map_msg_.header.stamp = this->now();
        global_map_pub->publish(global_map_msg_);
      });
}


void ROSWrapper::set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init)
{
  static auto init_pose_sub =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        [this, &init_pose, &flg_get_init_guess](
          const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
        {
          LOG(INFO) << YELLOW << " ---> [DEBUG] /initialpose callback triggered!" << RESET;
          LOG(INFO) << YELLOW << " ---> [DEBUG] Received pose: pos(" 
                    << msg->pose.pose.position.x << ", "
                    << msg->pose.pose.position.y << ", "
                    << msg->pose.pose.position.z << ") "
                    << "quat("
                    << msg->pose.pose.orientation.x << ", "
                    << msg->pose.pose.orientation.y << ", "
                    << msg->pose.pose.orientation.z << ", "
                    << msg->pose.pose.orientation.w << ")"
                    << RESET;

          V3 init_translation;
          init_translation << msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              msg->pose.pose.position.z;

          double x = msg->pose.pose.orientation.x;
          double y = msg->pose.pose.orientation.y;
          double z = msg->pose.pose.orientation.z;
          double w = msg->pose.pose.orientation.w;

          Quat init_rotation(w, x, y, z);

          init_pose = BASIC::SE3(SO3(init_rotation.toRotationMatrix()), init_translation);

          flg_get_init_guess = true;

          LOG(INFO) << YELLOW
                  << " ---> GET Initial guess from /initialpose: "
                  << init_translation.transpose()
                  << " yaw: "
                  << init_rotation.toRotationMatrix()
                          .eulerAngles(0, 1, 2)
                          .transpose()
                  << RESET;
        });

  static auto init_pose_rpy_sub =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initial_pose", 1,
        [this, &init_pose, &flg_get_init_guess](
          const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
        {
          LOG(INFO) << YELLOW << " ---> [DEBUG] /initial_pose callback triggered!" << RESET;
          LOG(INFO) << YELLOW << " ---> [DEBUG] Received pose: pos(" 
                    << msg->pose.pose.position.x << ", "
                    << msg->pose.pose.position.y << ", "
                    << msg->pose.pose.position.z << ") "
                    << "rpy("
                    << msg->pose.pose.orientation.x << ", "
                    << msg->pose.pose.orientation.y << ", "
                    << msg->pose.pose.orientation.z << ")"
                    << RESET;

          // 从话题消息中获取 xyz 位置
          V3 init_translation;
          init_translation << msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              msg->pose.pose.position.z;

          // 从话题消息中获取 rpy 欧拉角（角度制）
          double roll  = msg->pose.pose.orientation.x;
          double pitch = msg->pose.pose.orientation.y;
          double yaw   = msg->pose.pose.orientation.z;

          // 将角度转换为弧度，构建旋转矩阵
          Eigen::Matrix3d init_R = 
              (Eigen::AngleAxisd(yaw   / 180.0 * M_PI, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(pitch / 180.0 * M_PI, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(roll  / 180.0 * M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();

          init_pose = BASIC::SE3(BASIC::SO3(init_R.cast<BASIC::scalar>()), init_translation);

          flg_get_init_guess = true;

          LOG(INFO) << YELLOW
                  << " ---> GET Initial guess from /initial_pose: "
                  << init_translation.transpose()
                  << " rpy: "
                  << roll << " " << pitch << " " << yaw
                  << RESET;
        });

  if (flg_finish_init) {
    LOG(INFO) << YELLOW << " ---> [DEBUG] set_initial_data: flg_finish_init=true, unsubscribing /initialpose and /initial_pose" << RESET;
    init_pose_sub.reset();
    init_pose_rpy_sub.reset();
  } else {
    LOG(INFO) << YELLOW << " ---> [DEBUG] set_initial_data: flg_finish_init=false, subscriptions active" << RESET;
  }
}


} // namespace END.
