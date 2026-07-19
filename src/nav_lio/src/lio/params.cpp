

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
  bool   g_loop_closure_enable = false;
  string g_loop_map_name = "map_loop.pcd";
  float  g_loop_keyframe_min_distance = 0.5f;
  int    g_loop_keyframe_min_gap = 80;
  float  g_loop_search_radius = 5.0f;
  float  g_loop_icp_max_distance = 2.0f;
  float  g_loop_icp_score_threshold = 1.0f;
  float  g_loop_map_ds_size = 0.1f;
  
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
  double g_level_max_plane_gravity_angle_deg = 3.0;
  double g_level_max_attitude_innovation_deg = 6.0;
  // 这是聚合 RANSAC/PCA 平面法向的不确定度，不是单点噪声。
  // Scene37 表明该取值可让绝对水平观测与点云匹配处于相近的信息量范围。
  double g_level_attitude_stddev_deg = 0.015;

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
  int g_wall_yaw_max_references = 8;
  double g_wall_yaw_reference_min_yaw_information_ratio = 0.45;
  double g_wall_yaw_information_weak_ratio = 0.40;
  double g_wall_yaw_information_strong_ratio = 0.70;
  double g_wall_yaw_max_innovation_deg = 2.0;
  double g_wall_yaw_stddev_deg = 0.3;
  double g_wall_yaw_max_correction_per_frame_deg = 0.03;

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

}
