
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>


using namespace BASIC;

namespace LI2Sup{

namespace {

constexpr double kSensorTimeEpsilon = 1e-6;
constexpr double kMaxImuBufferAgeSeconds = 3.0;

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

  node.declare_parameter<bool>("lio.loop.enable", false);
  node.get_parameter("lio.loop.enable", g_loop_closure_enable);

  node.declare_parameter<std::string>("lio.loop.map_name", "map_loop.pcd");
  node.get_parameter("lio.loop.map_name", g_loop_map_name);

  node.declare_parameter<double>("lio.loop.keyframe_min_distance", 0.5);
  double loop_keyframe_min_distance = 0.5;
  node.get_parameter("lio.loop.keyframe_min_distance", loop_keyframe_min_distance);
  g_loop_keyframe_min_distance = static_cast<float>(loop_keyframe_min_distance);

  node.declare_parameter<int>("lio.loop.keyframe_min_gap", 80);
  node.get_parameter("lio.loop.keyframe_min_gap", g_loop_keyframe_min_gap);

  node.declare_parameter<double>("lio.loop.search_radius", 5.0);
  double loop_search_radius = 5.0;
  node.get_parameter("lio.loop.search_radius", loop_search_radius);
  g_loop_search_radius = static_cast<float>(loop_search_radius);

  node.declare_parameter<double>("lio.loop.icp_max_distance", 2.0);
  double loop_icp_max_distance = 2.0;
  node.get_parameter("lio.loop.icp_max_distance", loop_icp_max_distance);
  g_loop_icp_max_distance = static_cast<float>(loop_icp_max_distance);

  node.declare_parameter<double>("lio.loop.icp_score_threshold", 1.0);
  double loop_icp_score_threshold = 1.0;
  node.get_parameter("lio.loop.icp_score_threshold", loop_icp_score_threshold);
  g_loop_icp_score_threshold = static_cast<float>(loop_icp_score_threshold);

  node.declare_parameter<double>("lio.loop.map_ds_size", 0.1);
  double loop_map_ds_size = 0.1;
  node.get_parameter("lio.loop.map_ds_size", loop_map_ds_size);
  g_loop_map_ds_size = static_cast<float>(loop_map_ds_size);

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
      "lio.kf.level_constraint.max_plane_gravity_angle_deg", 3.0);
  node.get_parameter(
      "lio.kf.level_constraint.max_plane_gravity_angle_deg",
      g_level_max_plane_gravity_angle_deg);
  g_level_max_plane_gravity_angle_deg = std::clamp(
      g_level_max_plane_gravity_angle_deg, 0.5, 15.0);

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
  cb_sensor_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_processing_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = cb_sensor_;

  // Sensor streams must never back-pressure the Livox publishing threads.
  // Small best-effort histories preserve the newest measurements instead of
  // replaying seconds of stale IMU/LiDAR data after a temporary overload.
  auto imu_qos = rclcpp::SensorDataQoS();
  // MID-360 IMU is normally 200 Hz. Keep enough IMU history to cover the
  // complete five-frame LiDAR history plus scheduling jitter.
  imu_qos.keep_last(512);

  auto lidar_qos = rclcpp::SensorDataQoS();
  lidar_qos.keep_last(5);

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      g_imu_topic,
      imu_qos,
      std::bind(&ROSWrapper::imuHandler, this, std::placeholders::_1),
      sub_opt);

  if (g_lidar_type == LID_TYPE::LIVOX) {
    sub_lidar_ =
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::livoxHandler, this, std::placeholders::_1),
            sub_opt);
  } else {
    sub_lidar_std_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::stdMsgHandler, this, std::placeholders::_1),
            sub_opt);
  }

  pause_mapping_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/lio/pause_mapping",
      std::bind(
          &ROSWrapper::pauseMapping, this,
          std::placeholders::_1, std::placeholders::_2));

  /// output ======================================
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odom", 100);

  pub_imu_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/imu/odom", 10);

  pub_robo_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/robo/odom", 10);

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
      "/lio/path", 10);

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
  mapping_paused_.store(true);
  {
    std::lock_guard<std::mutex> lock(sensor_buffer_mutex_);
    lidar_buffer_.clear();
    imu_buffer_.clear();
    lidar_pushed_ = false;
  }

  response->success = true;
  response->message = "SuperLIO input frozen; final map state is ready to save.";
  LOG(INFO) << GREEN
            << " ---> [SuperLIO]: mapping input frozen, pending sensor buffers cleared."
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

  // Bound application memory by sensor time rather than by an arbitrary point
  // count. Three seconds is well above a normal scan-matching spike while
  // preventing unbounded growth when LiDAR is disconnected.
  while (imu_buffer_.size() > 2 &&
         data.secs - imu_buffer_.front().secs > kMaxImuBufferAgeSeconds) {
    imu_buffer_.pop_front();
    ++pruned_imu_buffer_count_;
  }
}


void ROSWrapper::livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
  if (mapping_paused_.load()) return;
  if(msg->point_num < 10) return;

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
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: lidar source timestamp gap. source_dt="
                   << source_interval << "s arrival_dt=" << arrival_interval << "s"
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
    lidar_buffer_.push_back(std::move(lidar_data));
  }
}


void ROSWrapper::stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if (mapping_paused_.load()) return;
  if(msg->data.size() < 10) return;
  
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
    lidar_buffer_.push_back(std::move(lidar_data));
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
      record_sync_result(true);
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
                              0.2;

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
