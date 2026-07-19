/**
 * @file parameters.hpp
 * @author WangLiansheng (lswang@mail.ecust.edu.cn)
 * @date 2023-03-14
 * @copyright Copyright (c) 2023
 */


#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_


#include <atomic>
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
  extern bool  g_loop_closure_enable;
  extern std::string g_loop_map_name;
  extern float g_loop_keyframe_min_distance;
  extern int   g_loop_keyframe_min_gap;
  extern float g_loop_search_radius;
  extern float g_loop_icp_max_distance;
  extern float g_loop_icp_score_threshold;
  extern float g_loop_map_ds_size;
  
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
  extern double g_level_max_plane_gravity_angle_deg;
  extern double g_level_max_attitude_innovation_deg;
  extern double g_level_attitude_stddev_deg;

  /// 由稳定垂直墙面提供的 Manhattan 航向参考。
  /// 该约束只观测绕世界竖直轴的旋转，不限制位置、横滚或俯仰。
  extern bool g_wall_yaw_constraint_enable;
  extern double g_wall_yaw_max_point_range;
  extern int g_wall_yaw_ransac_iterations;
  extern double g_wall_yaw_plane_distance_threshold;
  extern int g_wall_yaw_min_plane_inliers;
  extern double g_wall_yaw_min_plane_inlier_ratio;
  extern double g_wall_yaw_max_vertical_angle_deg;
  extern double g_wall_yaw_min_vertical_span;
  extern double g_wall_yaw_min_horizontal_span;
  extern int g_wall_yaw_reference_min_frames;
  extern double g_wall_yaw_reference_max_deviation_deg;
  extern double g_wall_yaw_max_innovation_deg;
  extern double g_wall_yaw_stddev_deg;

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

}

#endif
