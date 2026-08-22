

#include "lio/params.h"

using namespace std;
using namespace BASIC;

namespace LI2Sup{

  const std::string g_root_dir = std::string(ROOT);
  std::atomic<bool> g_flag_run = true; 
  bool g_flg_map_init = true;

  /// evaluation
  bool g_time_eva = false;

  bool   g_save_map;
  bool   g_if_filter; 
  string g_save_map_dir;
  string g_map_name;
  float  g_map_ds_size;
  int    g_pcd_save_interval;
  std::int64_t g_map_max_points_in_memory = 0;
  int    g_map_max_pending_writes = 64;
  bool   g_map_cleanup_work_files = true;
  bool   g_loop_closure_enable = false;
  bool   g_loop_endpoint_corridor_partial_enable = true;
  bool   g_loop_persist_keyframes = false;
  string g_loop_map_name = "map_loop.pcd";
  float  g_loop_keyframe_min_distance = 0.5f;
  int    g_loop_keyframe_min_gap = 80;
  double g_loop_internal_min_sensor_time_seconds = 30.0;
  float  g_loop_search_radius = 5.0f;
  float  g_loop_internal_search_radius = 1.5f;
  float  g_loop_icp_max_distance = 2.0f;
  float  g_loop_icp_score_threshold = 1.0f;
  float  g_loop_map_ds_size = 0.1f;
  int    g_loop_candidate_limit = 30;
  int    g_loop_local_window_size = 10;
  float  g_loop_max_correction_rotation_deg = 5.0f;
  float  g_loop_max_correction_tilt_deg = 2.0f;
  float  g_loop_max_adaptive_yaw_deg = 15.0f;
  float  g_loop_min_overlap_ratio = 0.35f;
  float  g_loop_translation_drift_ratio = 0.02f;
  float  g_loop_rotation_drift_deg_per_m = 0.02f;
  float  g_loop_min_consistency_weight = 0.25f;
  float  g_loop_verification_max_distance = 0.6f;
  float  g_loop_min_symmetric_overlap = 0.45f;
  float  g_loop_max_trimmed_rmse = 0.30f;
  float  g_loop_verification_block_size = 1.0f;
  int    g_loop_min_verification_blocks = 6;
  float  g_loop_min_verification_block_ratio = 0.50f;
  float  g_loop_min_verification_span = 3.0f;
  float  g_loop_min_structural_overlap = 0.35f;
  float  g_loop_max_local_translation_strain = 0.015f;
  float  g_loop_max_local_translation_delta = 0.50f;
  bool   g_loop_ground_z_refinement_enable = true;
  float  g_loop_ground_z_cell_size = 0.25f;
  float  g_loop_ground_z_pair_xy_distance = 0.35f;
  int    g_loop_ground_z_min_pairs = 80;
  float  g_loop_ground_z_min_inlier_ratio = 0.65f;
  float  g_loop_ground_z_max_mad = 0.08f;
  float  g_loop_ground_z_max_adjustment = 0.5f;
  float  g_loop_ground_z_planar_hold_weight = 0.25f;
  bool   g_loop_post_residual_refinement_enable = true;
  int    g_loop_post_residual_refinement_min_anchors = 2;
  float  g_loop_post_residual_refinement_target_translation = 0.10f;
  float  g_loop_post_residual_refinement_target_rotation_deg = 0.20f;
  float  g_loop_post_residual_refinement_max_weight_scale = 4.0f;
  double g_loop_finalize_base_seconds = 120.0;
  double g_loop_finalize_seconds_per_keyframe = 0.04;
  double g_loop_max_finalize_seconds = 600.0;
  bool   g_loop_prefer_earliest_candidate = false;
  bool   g_loop_online_enable = false;
  int    g_loop_online_interval_keyframes = 5;
  int    g_loop_online_queue_capacity = 32;
  int    g_loop_online_candidate_limit = 4;
  int    g_loop_online_local_window_size = 8;
  float  g_loop_online_search_radius = 8.0f;
  float  g_loop_online_voxel_size = 0.30f;
  double g_loop_online_max_task_seconds = 60.0;
  int    g_loop_online_min_confirmations = 2;
  float  g_loop_online_confirmation_translation = 0.35f;
  float  g_loop_online_confirmation_yaw_deg = 0.50f;
  float  g_loop_online_max_translation_step = 0.05f;
  float  g_loop_online_max_yaw_step_deg = 0.10f;
  
  string g_imu_topic;
  string g_lidar_topic;

  int    g_lidar_type;
  float  g_blind2;
  float  g_maxrange2;
  int    g_filter_rate;
  bool   g_enable_downsample;
  float  g_voxel_fliter_size;
  double g_lidar_time_offset = 0.0;
  bool   g_use_query_time_undistort = true;

  int    g_imu_type;
  double g_gravity_norm = 9.7946;
  double g_imu_na;
  double g_imu_ng;
  double g_imu_nba;
  double g_imu_nbg;
  int g_imu_init_samples = 400;
  double g_imu_init_max_gyro_norm = 0.05;
  double g_imu_init_max_gyro_stddev = 0.03;
  double g_imu_init_max_accel_stddev_ratio = 0.05;
  double g_max_imu_integration_dt = 0.05;
  double g_scan_boundary_tolerance = 0.001;

  SE3 g_lidar_imu;
  SE3 g_odom_robo;
  M3  g_lidar_robo_yaw;

  /// hash_map
  std::size_t g_ivox_capacity = 100000;
  float       g_ivox_resolution = 0.5;

  /// kf
  int g_kf_type = 1;                // 1: ESKF, 2: InESKF
  int g_kf_max_iterations = 4;
  bool g_kf_align_gravity = true;
  bool g_kf_estimate_gravity = false;
  double g_kf_quit_eps;

  bool g_level_constraint_enable = true;
  double g_level_gravity_window_sec = 1.0;
  double g_level_max_accel_norm_ratio = 0.12;
  double g_level_max_point_range = 18.0;
  double g_level_min_down_distance = 0.05;
  double g_level_max_down_distance = 3.0;
  int g_level_ransac_iterations = 96;
  double g_level_plane_distance_threshold = 0.06;
  int g_level_min_plane_inliers = 40;
  double g_level_min_plane_inlier_ratio = 0.12;
  double g_level_slope_soft_start_angle_deg = 1.0;
  double g_level_max_plane_gravity_angle_deg = 1.5;
  int g_level_slope_enter_min_frames = 5;
  double g_level_slope_exit_angle_deg = 0.75;
  int g_level_slope_exit_min_frames = 10;
  int g_level_slope_pending_max_invalid_frames = 2;
  int g_level_slope_recovery_min_frames = 3;
  double g_level_slope_spatial_window_m = 5.0;
  double g_level_slope_spatial_min_support_ratio = 0.40;
  double g_level_slope_spatial_max_grade_error_deg = 0.75;
  int g_level_slope_spatial_max_mismatch_windows = 2;
  int g_level_slope_spatial_reentry_consistent_windows = 2;
  double g_level_max_attitude_innovation_deg = 6.0;
  // 这是聚合 RANSAC/PCA 平面法向的不确定度，不是单点噪声。
  // 双坡度证据门会在 1.0--1.5 度间渐弱，持续证据再锁存坡面保护；
  // 通过该门的平地法向仍需要足够强的先验来抑制长距离姿态累积漂移。
  double g_level_attitude_stddev_deg = 0.015;

  // Experimental only: a frame-to-frame ground-height chain can integrate a
  // small plane-normal bias over a long route.  Keep it available for offline
  // A/B tests, but fail safe in production until a non-accumulating estimator
  // replaces it.
  bool g_ground_height_continuity_enable = false;
  int g_ground_height_continuity_max_frame_gap = 5;
  double g_ground_height_continuity_max_horizontal_step_m = 1.0;
  double g_ground_height_continuity_max_normal_difference_deg = 5.0;
  double g_ground_height_continuity_max_innovation_m = 0.05;
  double g_ground_height_continuity_max_correction_per_frame_m = 0.01;
  double g_ground_height_continuity_max_total_correction_m = 0.30;
  double g_ground_height_continuity_stddev_m = 0.02;

  bool g_wall_yaw_constraint_enable = false;
  double g_wall_yaw_max_point_range = 20.0;
  int g_wall_yaw_ransac_iterations = 128;
  int g_wall_yaw_extraction_interval_frames = 3;
  double g_wall_yaw_plane_distance_threshold = 0.08;
  int g_wall_yaw_min_plane_inliers = 40;
  double g_wall_yaw_min_plane_inlier_ratio = 0.05;
  double g_wall_yaw_max_vertical_angle_deg = 8.0;
  double g_wall_yaw_min_vertical_span = 0.8;
  double g_wall_yaw_min_horizontal_span = 1.5;
  int g_wall_yaw_reference_min_frames = 15;
  double g_wall_yaw_reference_max_deviation_deg = 1.0;
  double g_wall_yaw_reference_radius_m = 30.0;
  double g_wall_yaw_reference_extension_ratio = 0.75;
  int g_wall_yaw_max_references = 256;
  double g_wall_yaw_reference_min_yaw_information_ratio = 0.45;
  double g_wall_yaw_information_weak_ratio = 0.40;
  double g_wall_yaw_information_strong_ratio = 0.70;
  double g_wall_yaw_max_innovation_deg = 2.0;
  double g_wall_yaw_stddev_deg = 0.3;
  double g_wall_yaw_max_correction_per_frame_deg = 0.03;
  double g_wall_yaw_recapture_max_innovation_deg = 8.0;
  int g_wall_yaw_recapture_min_frames = 30;
  double g_wall_yaw_recapture_core_radius_ratio = 0.25;
  int g_wall_yaw_recapture_min_reference_age_frames = 300;
  double g_wall_yaw_recapture_min_scene_quality = 0.80;
  double g_wall_yaw_recapture_initial_max_deviation_deg = 1.5;
  double g_wall_yaw_recapture_stddev_deg = 1.5;
  double g_wall_yaw_recapture_max_correction_per_frame_deg = 0.01;

  /// submap 
  double g_submap_resolution;
  int    g_submap_capacity;

  /// output
  bool g_2_robot    = false;
  bool g_2_plan_env_world = false; 
  bool g_2_plan_env_body  = false;
  bool g_2_ml_map = false;
  bool g_visual_map = true;
  bool g_visual_dense = false;
  int  g_pub_step;
  float g_map_preview_ds_size = 0.2f;
  int g_map_preview_publish_interval = 20;
  int g_map_preview_max_points = 150000;

  int g_min_effective_points = 20;
  double g_max_frame_translation = 2.0;
  double g_max_frame_rotation_deg = 45.0;

  /// for planner
  bool g_planner_enable;

  ResidualType g_residual_type = PROB;

  /// for relocation
  bool g_update_map = false;
  double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;
  bool g_relocation_anchor_enable = true;
  int g_relocation_anchor_interval_frames = 10;
  int g_relocation_anchor_window_frames = 20;
  int g_relocation_anchor_min_frames = 8;
  int g_relocation_anchor_max_failures = 5;
  double g_relocation_anchor_map_radius = 35.0;
  double g_relocation_anchor_voxel_size = 0.5;
  double g_relocation_anchor_max_correspondence_distance = 1.5;
  double g_relocation_anchor_verification_distance = 0.6;
  double g_relocation_anchor_min_overlap = 0.55;
  double g_relocation_anchor_max_rmse = 0.35;
  double g_relocation_anchor_max_translation = 2.0;
  double g_relocation_anchor_max_yaw_deg = 5.0;
  double g_relocation_anchor_max_tilt_deg = 1.5;
  double g_relocation_anchor_max_translation_step = 0.35;
  double g_relocation_anchor_max_yaw_step_deg = 0.75;
  double g_relocation_anchor_min_motion = 1.0;
  double g_relocation_anchor_min_support_major = 4.0;
  double g_relocation_anchor_min_support_minor = 2.0;
  int g_relocation_anchor_min_structural_points = 80;

}
