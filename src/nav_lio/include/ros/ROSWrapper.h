
#ifndef ROSWRAPPER_HPP_
#define ROSWRAPPER_HPP_

#include <map>
#include <tuple>
#include <deque>
#include <vector>
#include <execution>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>

/// msgs
#include "nav_lio/msg/cloud_pose.hpp"
#include "nav_lio/msg/cloud_pose2.hpp"


#include "lio/params.h"
#include "basic/alias.h"
#include "basic/logs.h"
#include "basic/Manifold.h"
#include "common/ds.h"

#include "lio/ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"


namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node);

void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, BASIC::CloudPtr& point_cloud);

class ROSWrapper : public rclcpp::Node {
public:
  explicit ROSWrapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ROSWrapper(){};
  using Ptr = std::shared_ptr<ROSWrapper>;
  bool sync_measure(MeasureGroup&);
  bool isMappingPaused() const { return mapping_paused_.load(); }

  void setESKF(ESKF::Ptr& eskf) { eskf_ = eskf;}

  void clear(){
    std::lock_guard<std::mutex> lock(sensor_buffer_mutex_);
    lidar_buffer_.clear();
    imu_buffer_.clear();
    lidar_pushed_ = false;
    last_timestamp_imu_ = -1.0;
    last_timestamp_lidar_ = -1.0;
    sync_window_completed_count_ = 0;
    sync_window_dropped_count_ = 0;
    sync_health_unhealthy_ = false;
    sync_health_window_start_ = {};
  }

  void pub_odom(const NavState&);
  void pub_cloud_world(const BASIC::CloudPtr& pc, double time);
  void pub_map_accumulated(const BASIC::CloudPtr& pc, double time);
  void pub_cloud2planner(const BASIC::CloudPtr& pc, double time);
  void pub_cloud_world_pose(const BASIC::CloudPtr& pc, 
                            const NavState& state);
  void pub_cloud_body_pose(const BASIC::CloudPtr& pc, 
                           const NavState& state);
  void pub_cloud_body_pose( const BASIC::VV3& pc_body,
                            const NavState& state);  
  void pub_processing_time(double time, double current_time, double mean_time, double std_time);

  void set_global_map(const BASIC::CloudPtr& global_map);

  void set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init = false);

  rclcpp::CallbackGroup::SharedPtr getSensorCallbackGroup() {
    return cb_sensor_;
  }

  rclcpp::CallbackGroup::SharedPtr getProcessingCallbackGroup() {
    return cb_processing_;
  }

private:
  void pauseMapping(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg);
  void livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void setupParams();
  void setupIO();

private:
  rclcpp::CallbackGroup::SharedPtr cb_sensor_;
  rclcpp::CallbackGroup::SharedPtr cb_processing_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_mapping_service_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_std_;

  std::deque<IMUData>   imu_buffer_;
  std::deque<LidarData> lidar_buffer_;
  std::mutex sensor_buffer_mutex_;
  bool lidar_pushed_ = false;
  double last_timestamp_imu_ = -1.0;
  double last_timestamp_lidar_ = -1.0;
  std::atomic<bool> mapping_paused_{false};
  std::chrono::steady_clock::time_point last_lidar_arrival_time_{};
  double last_lidar_source_time_ = -1.0;
  std::chrono::steady_clock::time_point last_imu_lag_warning_time_{};
  std::chrono::steady_clock::time_point last_lag_warning_time_{};
  std::chrono::steady_clock::time_point last_missing_imu_warning_time_{};
  std::chrono::steady_clock::time_point last_clock_domain_warning_time_{};
  std::uint64_t lidar_without_imu_count_ = 0;
  std::uint64_t lidar_missing_imu_start_count_ = 0;
  std::uint64_t lidar_imu_gap_count_ = 0;
  std::uint64_t lidar_invalid_time_count_ = 0;
  std::uint64_t rejected_imu_clock_count_ = 0;
  std::uint64_t rejected_lidar_clock_count_ = 0;
  std::uint64_t pruned_imu_buffer_count_ = 0;
  std::uint64_t sync_window_completed_count_ = 0;
  std::uint64_t sync_window_dropped_count_ = 0;
  bool sync_health_unhealthy_ = false;
  std::chrono::steady_clock::time_point sync_health_window_start_{};

  ESKF::Ptr eskf_{nullptr};

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped msg2uav_;
  sensor_msgs::msg::PointCloud2 global_map_msg_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  BASIC::V3 last_path_point_ = BASIC::V3(0, 0, -100);

/// output.
private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;       /// lidar fre --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odom_;   /// IMU fre   --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_robo_odom_;  /// IMU fre   --> Robot frame
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_world_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_accumulated_;
};

} // namespace END.

#endif
