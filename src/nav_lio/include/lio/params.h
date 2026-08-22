/**
 * @file parameters.hpp
 * @author WangLiansheng (lswang@mail.ecust.edu.cn)
 * @date 2023-03-14
 * @copyright Copyright (c) 2023
 */


#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_


#include <atomic>
#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "basic/alias.h"
#include "basic/Manifold.h"


namespace LI2Sup{
  
  extern const std::string g_root_dir;
  extern std::atomic<bool> g_flag_run;
  extern bool g_flg_map_init;

  /// evaluation
  extern bool g_time_eva;

  extern bool g_save_map;
  extern bool g_if_filter;
  extern std::string g_map_name;
  extern std::string g_save_map_dir;
  extern float g_map_ds_size;
  extern int   g_pcd_save_interval;
  extern std::int64_t g_map_max_points_in_memory;
  extern int   g_map_max_pending_writes;
  extern bool  g_map_cleanup_work_files;
  extern bool  g_loop_closure_enable;
  // Safe fallback for a terminal revisit observed in a short, repetitive
  // corridor.  It verifies ordered trajectory overlap and leaves translation
  // along the corridor unconstrained instead of accepting an ambiguous full
  // SE(3) closure.
  extern bool  g_loop_endpoint_corridor_partial_enable;
  extern bool  g_loop_persist_keyframes;
  extern std::string g_loop_map_name;
  extern float g_loop_keyframe_min_distance;
  extern int   g_loop_keyframe_min_gap;
  extern double g_loop_internal_min_sensor_time_seconds;
  extern float g_loop_search_radius;
  extern float g_loop_internal_search_radius;
  extern float g_loop_icp_max_distance;
  extern float g_loop_icp_score_threshold;
  extern float g_loop_map_ds_size;
  extern int   g_loop_candidate_limit;
  extern int   g_loop_local_window_size;
  extern float g_loop_max_correction_rotation_deg;
  extern float g_loop_max_correction_tilt_deg;
  extern float g_loop_max_adaptive_yaw_deg;
  extern float g_loop_min_overlap_ratio;
  extern float g_loop_translation_drift_ratio;
  extern float g_loop_rotation_drift_deg_per_m;
  extern float g_loop_min_consistency_weight;
  extern float g_loop_verification_max_distance;
  extern float g_loop_min_symmetric_overlap;
  extern float g_loop_max_trimmed_rmse;
  extern float g_loop_verification_block_size;
  extern int   g_loop_min_verification_blocks;
  extern float g_loop_min_verification_block_ratio;
  extern float g_loop_min_verification_span;
  extern float g_loop_min_structural_overlap;
  extern float g_loop_max_local_translation_strain;
  extern float g_loop_max_local_translation_delta;
  extern bool  g_loop_ground_z_refinement_enable;
  extern float g_loop_ground_z_cell_size;
  extern float g_loop_ground_z_pair_xy_distance;
  extern int   g_loop_ground_z_min_pairs;
  extern float g_loop_ground_z_min_inlier_ratio;
  extern float g_loop_ground_z_max_mad;
  extern float g_loop_ground_z_max_adjustment;
  extern float g_loop_ground_z_planar_hold_weight;
  // Re-optimize only already accepted, independently supported loop groups
  // when their post-graph residual is still visible. The trial is committed
  // only if every original graph-safety gate continues to pass.
  extern bool  g_loop_post_residual_refinement_enable;
  extern int   g_loop_post_residual_refinement_min_anchors;
  extern float g_loop_post_residual_refinement_target_translation;
  extern float g_loop_post_residual_refinement_target_rotation_deg;
  extern float g_loop_post_residual_refinement_max_weight_scale;
  extern double g_loop_finalize_base_seconds;
  extern double g_loop_finalize_seconds_per_keyframe;
  extern double g_loop_max_finalize_seconds;
  extern bool  g_loop_prefer_earliest_candidate;
  // Low-frequency loop closure that runs while mapping. It never executes
  // registration or graph optimization on the LiDAR processing thread.
  extern bool   g_loop_online_enable;
  extern int    g_loop_online_interval_keyframes;
  extern int    g_loop_online_queue_capacity;
  extern int    g_loop_online_candidate_limit;
  extern int    g_loop_online_local_window_size;
  extern float  g_loop_online_search_radius;
  extern float  g_loop_online_voxel_size;
  extern double g_loop_online_max_task_seconds;
  extern int    g_loop_online_min_confirmations;
  extern float  g_loop_online_confirmation_translation;
  extern float  g_loop_online_confirmation_yaw_deg;
  extern float  g_loop_online_max_translation_step;
  extern float  g_loop_online_max_yaw_step_deg;
  
  extern std::string g_imu_topic;
  extern std::string g_lidar_topic;

  extern int   g_lidar_type;       // 1: mid360, 2: hesai16, 3: velo16, 4: velo32, 5: vel_nclt, 6: ls16 
  extern float g_blind2;
  extern float g_maxrange2;
  extern int   g_filter_rate;
  extern bool  g_enable_downsample;
  extern float g_voxel_fliter_size;
  extern double g_lidar_time_offset;
  extern bool  g_use_query_time_undistort;

  extern int    g_imu_type;
  extern double g_gravity_norm;
  extern double g_imu_na;
  extern double g_imu_ng;
  extern double g_imu_nba;
  extern double g_imu_nbg;
  extern int g_imu_init_samples;
  extern double g_imu_init_max_gyro_norm;
  extern double g_imu_init_max_gyro_stddev;
  extern double g_imu_init_max_accel_stddev_ratio;
  extern double g_max_imu_integration_dt;
  extern double g_scan_boundary_tolerance;

  extern BASIC::SE3 g_lidar_imu;      // lidar in imu frame
  extern BASIC::SE3 g_odom_robo;      // lidar in robot frame
  extern BASIC::M3  g_lidar_robo_yaw; // lidar in robot frame rotation only yaw

  /// hash_map
  extern std::size_t g_ivox_capacity;
  extern float       g_ivox_resolution;
  
  /// kf
  extern int g_kf_type;            // 1: ESKF, 2: InESKF.
  extern int g_kf_max_iterations;
  extern bool g_kf_align_gravity;
  extern bool g_kf_estimate_gravity;
  extern double g_kf_quit_eps;

  /// 由重力一致局部平面提供的绝对横滚/俯仰参考。
  /// 该约束只观测姿态，不限制位置或竖直速度。
  extern bool g_level_constraint_enable;
  extern double g_level_gravity_window_sec;
  extern double g_level_max_accel_norm_ratio;
  extern double g_level_max_point_range;
  extern double g_level_min_down_distance;
  extern double g_level_max_down_distance;
  extern int g_level_ransac_iterations;
  extern double g_level_plane_distance_threshold;
  extern int g_level_min_plane_inliers;
  extern double g_level_min_plane_inlier_ratio;
  extern double g_level_slope_soft_start_angle_deg;
  extern double g_level_max_plane_gravity_angle_deg;
  extern int g_level_slope_enter_min_frames;
  extern double g_level_slope_exit_angle_deg;
  extern int g_level_slope_exit_min_frames;
  extern int g_level_slope_pending_max_invalid_frames;
  extern int g_level_slope_recovery_min_frames;
  extern double g_level_slope_spatial_window_m;
  extern double g_level_slope_spatial_min_support_ratio;
  extern double g_level_slope_spatial_max_grade_error_deg;
  extern int g_level_slope_spatial_max_mismatch_windows;
  extern int g_level_slope_spatial_reentry_consistent_windows;
  extern double g_level_max_attitude_innovation_deg;
  extern double g_level_attitude_stddev_deg;

  /// Consecutive committed ground planes provide a slope-aware relative
  /// height aid. It runs after the joint ESKF update and directly adjusts
  /// world Z only; covariance, XY, attitude, velocity and biases are untouched.
  /// Discontinuities automatically break the short reference chain.
  extern bool g_ground_height_continuity_enable;
  extern int g_ground_height_continuity_max_frame_gap;
  extern double g_ground_height_continuity_max_horizontal_step_m;
  extern double g_ground_height_continuity_max_normal_difference_deg;
  extern double g_ground_height_continuity_max_innovation_m;
  extern double g_ground_height_continuity_max_correction_per_frame_m;
  /// Maximum signed world-Z offset that this auxiliary feature may inject
  /// during one mapping session. This bounds estimator influence; it is not a
  /// terrain-height or map-elevation limit.
  extern double g_ground_height_continuity_max_total_correction_m;
  extern double g_ground_height_continuity_stddev_m;

  /// 由稳定垂直墙面提供的 Manhattan 航向参考。
  /// 该约束只观测绕世界竖直轴的旋转，不限制位置、横滚或俯仰。
  extern bool g_wall_yaw_constraint_enable;
  extern double g_wall_yaw_max_point_range;
  extern int g_wall_yaw_ransac_iterations;
  extern int g_wall_yaw_extraction_interval_frames;
  extern double g_wall_yaw_plane_distance_threshold;
  extern int g_wall_yaw_min_plane_inliers;
  extern double g_wall_yaw_min_plane_inlier_ratio;
  extern double g_wall_yaw_max_vertical_angle_deg;
  extern double g_wall_yaw_min_vertical_span;
  extern double g_wall_yaw_min_horizontal_span;
  extern int g_wall_yaw_reference_min_frames;
  extern double g_wall_yaw_reference_max_deviation_deg;
  extern double g_wall_yaw_reference_radius_m;
  extern double g_wall_yaw_reference_extension_ratio;
  extern int g_wall_yaw_max_references;
  extern double g_wall_yaw_reference_min_yaw_information_ratio;
  extern double g_wall_yaw_information_weak_ratio;
  extern double g_wall_yaw_information_strong_ratio;
  extern double g_wall_yaw_max_innovation_deg;
  extern double g_wall_yaw_stddev_deg;
  extern double g_wall_yaw_max_correction_per_frame_deg;
  extern double g_wall_yaw_recapture_max_innovation_deg;
  extern int g_wall_yaw_recapture_min_frames;
  extern double g_wall_yaw_recapture_core_radius_ratio;
  extern int g_wall_yaw_recapture_min_reference_age_frames;
  extern double g_wall_yaw_recapture_min_scene_quality;
  extern double g_wall_yaw_recapture_initial_max_deviation_deg;
  extern double g_wall_yaw_recapture_stddev_deg;
  extern double g_wall_yaw_recapture_max_correction_per_frame_deg;

  /// submaps
  extern double g_submap_resolution;
  extern int    g_submap_capacity;

  /// output  
  extern bool g_2_robot;
  extern bool g_2_plan_env_world;
  extern bool g_2_plan_env_body;
  extern bool g_2_ml_map;
  extern bool g_visual_map;
  extern bool g_visual_dense;
  extern int  g_pub_step;
  extern float g_map_preview_ds_size;
  extern int g_map_preview_publish_interval;
  extern int g_map_preview_max_points;

  /// mapping safety
  extern int g_min_effective_points;
  extern double g_max_frame_translation;
  extern double g_max_frame_rotation_deg;

  /// for planner
  extern bool g_planner_enable;

  /// Define the hybrid residual formulation.
  enum ResidualType{
    PROB = 1,     // Probabilistic residual
    P2P  = 2,     // Point-to-plane residual
    MIX  = 3      // Hybrid residual (probabilistic + point-to-plane)
  };
  extern ResidualType g_residual_type;


  /// for relocation
  extern bool g_update_map;
  extern double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;
  extern bool g_relocation_anchor_enable;
  extern int g_relocation_anchor_interval_frames;
  extern int g_relocation_anchor_window_frames;
  extern int g_relocation_anchor_min_frames;
  extern int g_relocation_anchor_max_failures;
  extern double g_relocation_anchor_map_radius;
  extern double g_relocation_anchor_voxel_size;
  extern double g_relocation_anchor_max_correspondence_distance;
  extern double g_relocation_anchor_verification_distance;
  extern double g_relocation_anchor_min_overlap;
  extern double g_relocation_anchor_max_rmse;
  extern double g_relocation_anchor_max_translation;
  extern double g_relocation_anchor_max_yaw_deg;
  extern double g_relocation_anchor_max_tilt_deg;
  extern double g_relocation_anchor_max_translation_step;
  extern double g_relocation_anchor_max_yaw_step_deg;
  extern double g_relocation_anchor_min_motion;
  extern double g_relocation_anchor_min_support_major;
  extern double g_relocation_anchor_min_support_minor;
  extern int g_relocation_anchor_min_structural_points;

}

#endif
