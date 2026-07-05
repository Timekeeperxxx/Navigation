// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <chrono>
#include <csignal>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <thread>
#include <map>
#include <utility>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/pcd_io.h"

class SaveTerrainMapNode : public rclcpp::Node
{
public:
  SaveTerrainMapNode()
  : Node("save_terrain_map")
  {
    // map_dir 为空时使用本地临时目录，避免兜底路径依赖外部工程。
    this->declare_parameter<std::string>("save_directory", "/tmp/navigation_terrain_map");
    this->declare_parameter<std::string>("map_dir", "");
    this->declare_parameter<std::string>("file_prefix", "terrain_map_");
    this->declare_parameter<bool>("accumulate", true);
    this->declare_parameter<bool>("save_map_cloud", false);
    this->declare_parameter<float>("intensity_threshold", 0.05);
    this->declare_parameter<float>("ground_z_range", 0.1);
    this->declare_parameter<bool>("ground_xy_dedup", true);
    this->declare_parameter<float>("ground_xy_leaf_size", 0.01);
    this->declare_parameter<float>("ground_z_layer_size", 0.15);

    // Get parameters
    this->get_parameter("save_directory", save_directory_);
    this->get_parameter("map_dir", map_dir_);
    this->get_parameter("file_prefix", file_prefix_);
    this->get_parameter("accumulate", accumulate_);
    this->get_parameter("save_map_cloud", save_map_cloud_);
    this->get_parameter("intensity_threshold", intensity_threshold_);
    this->get_parameter("ground_z_range", ground_z_range_);
    this->get_parameter("ground_xy_dedup", ground_xy_dedup_);
    this->get_parameter("ground_xy_leaf_size", ground_xy_leaf_size_);
    this->get_parameter("ground_z_layer_size", ground_z_layer_size_);

    if (ground_xy_leaf_size_ <= 0.0f) {
      RCLCPP_WARN(
        this->get_logger(),
        "ground_xy_leaf_size must be positive, reset to 0.2m.");
      ground_xy_leaf_size_ = 0.2f;
    }
    if (ground_z_layer_size_ <= 0.0f) {
      RCLCPP_WARN(
        this->get_logger(),
        "ground_z_layer_size 必须为正数，重置为 0.15m。");
      ground_z_layer_size_ = 0.15f;
    }

    // If map_dir is specified, use it as the save directory
    if (!map_dir_.empty()) {
      save_directory_ = map_dir_;
    }

    // Ensure directory ends with '/'
    if (!save_directory_.empty() && save_directory_.back() != '/') {
      save_directory_ += '/';
    }

    ensureSaveDirectory();

    // Initialize accumulated clouds
    accumulated_map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    accumulated_ground_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    accumulated_base_footprint_fill_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    // Subscription to terrain_map topic
    terrain_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "terrain_map", 10,
      std::bind(&SaveTerrainMapNode::terrainCallback, this, std::placeholders::_1));

    // Subscription to base_footprint_fill_cloud topic
    base_footprint_fill_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "base_footprint_fill_cloud", 10,
      std::bind(&SaveTerrainMapNode::baseFootprintFillCallback, this, std::placeholders::_1));

    // Service to trigger saving
    save_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_terrain_map",
      std::bind(&SaveTerrainMapNode::saveServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "Save terrain map node initialized. Save directory: %s, accumulate: %s, intensity_threshold: %.3f, ground_xy_dedup: %s, ground_xy_leaf_size: %.3f, ground_z_layer_size: %.3f",
      save_directory_.c_str(), accumulate_ ? "true" : "false", intensity_threshold_,
      ground_xy_dedup_ ? "true" : "false", ground_xy_leaf_size_, ground_z_layer_size_);
  }

  ~SaveTerrainMapNode()
  {
    // 如果 main() 中已经保存并清空了点云，这里不再重复保存
    if (!saved_in_main_) {
      saveMapToFile();
    }
  }

  // 标记 main() 中已保存
  void markSavedInMain()
  {
    saved_in_main_ = true;
  }

  // 清空累积点云，用于避免析构时重复保存
  void clearAccumulatedClouds()
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    accumulated_map_cloud_->clear();
    accumulated_ground_cloud_->clear();
    accumulated_base_footprint_fill_cloud_->clear();
    accumulated_ground_grid_index_.clear();
    accumulated_base_footprint_fill_grid_index_.clear();
  }

  bool ensureSaveDirectory()
  {
    if (save_directory_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "保存目录为空。");
      return false;
    }

    std::error_code ec;
    std::filesystem::create_directories(save_directory_, ec);
    if (ec) {
      RCLCPP_ERROR(
        this->get_logger(), "创建保存目录失败 %s: %s",
        save_directory_.c_str(), ec.message().c_str());
      return false;
    }
    return true;
  }

  template<typename CloudT>
  bool savePcdBinary(const std::string & filename, const CloudT & cloud)
  {
    try {
      if (pcl::io::savePCDFileBinary(filename, cloud) == 0) {
        return true;
      }
      RCLCPP_ERROR(this->get_logger(), "保存 PCD 文件失败: %s", filename.c_str());
      return false;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "保存 PCD 文件失败: %s: %s",
        filename.c_str(), e.what());
      return false;
    }
  }

  // 保存地图到文件（不滤波，快速保存）
  void saveMapToFile()
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    if (!ensureSaveDirectory()) {
      return;
    }

    // Generate filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);
    char timestamp[20];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &now_tm);
    std::string base_filename = save_directory_ + file_prefix_ + timestamp;

    if (accumulate_) {
      if (
        accumulated_map_cloud_->empty() && accumulated_ground_cloud_->empty() &&
        accumulated_base_footprint_fill_cloud_->empty()) {
        RCLCPP_WARN(this->get_logger(), "No terrain map accumulated, nothing to save.");
        return;
      }

      bool has_error = false;

      if (save_map_cloud_ && !accumulated_map_cloud_->empty()) {
        std::string map_filename = base_filename + "_map.pcd";
        if (savePcdBinary(map_filename, *accumulated_map_cloud_)) {
          RCLCPP_INFO(this->get_logger(), "Saved map cloud to %s (points: %zu)",
                       map_filename.c_str(), accumulated_map_cloud_->size());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to save map cloud to %s", map_filename.c_str());
          has_error = true;
        }
      }

      if (!accumulated_ground_cloud_->empty()) {
        std::string ground_filename = base_filename + "_ground.pcd";
        if (savePcdBinary(ground_filename, *accumulated_ground_cloud_)) {
          RCLCPP_INFO(this->get_logger(), "Saved ground cloud to %s (points: %zu)",
                       ground_filename.c_str(), accumulated_ground_cloud_->size());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to save ground cloud to %s", ground_filename.c_str());
          has_error = true;
        }
      }

      // 保存 base_footprint 附近填充点云
      if (!accumulated_base_footprint_fill_cloud_->empty()) {
        std::string fill_filename = base_filename + "_base_footprint_fill.pcd";
        if (savePcdBinary(fill_filename, *accumulated_base_footprint_fill_cloud_)) {
          RCLCPP_INFO(this->get_logger(), "Saved base_footprint fill cloud to %s (points: %zu)",
                       fill_filename.c_str(), accumulated_base_footprint_fill_cloud_->size());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to save base_footprint fill cloud to %s", fill_filename.c_str());
          has_error = true;
        }
      }

      if (!has_error) {
        RCLCPP_INFO(this->get_logger(), "Save complete: %s_{map,ground,base_footprint_fill}.pcd", base_filename.c_str());
      }
    } else {
      // 非 accumulate 模式：保存最新一帧
      if (!latest_cloud_) {
        RCLCPP_WARN(this->get_logger(), "No terrain map received, nothing to save.");
        return;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*latest_cloud_, *full_cloud);

      pcl::PointCloud<pcl::PointXYZI> map_cloud, ground_cloud;
      splitCloudByIntensityAndZRange(full_cloud, map_cloud, ground_cloud);

      bool has_error = false;

      if (save_map_cloud_ && !map_cloud.empty()) {
        std::string map_filename = base_filename + "_map.pcd";
        if (savePcdBinary(map_filename, map_cloud)) {
          RCLCPP_INFO(this->get_logger(), "Saved map cloud to %s (points: %zu)",
                       map_filename.c_str(), map_cloud.size());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to save map cloud to %s", map_filename.c_str());
          has_error = true;
        }
      }

      if (!ground_cloud.empty()) {
        std::string ground_filename = base_filename + "_ground.pcd";
        if (savePcdBinary(ground_filename, ground_cloud)) {
          RCLCPP_INFO(this->get_logger(), "Saved ground cloud to %s (points: %zu)",
                       ground_filename.c_str(), ground_cloud.size());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to save ground cloud to %s", ground_filename.c_str());
          has_error = true;
        }
      }

      if (!has_error) {
        RCLCPP_INFO(this->get_logger(), "Save complete: %s_{map,ground}.pcd", base_filename.c_str());
      }
    }
  }

  // 根据强度阈值和空间Z值范围分割点云
  // 对于intensity低的点，如果其所在空间体素内Z值变化大（有墙壁），则归为map
  void splitCloudByIntensityAndZRange(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    pcl::PointCloud<pcl::PointXYZI> & map_cloud,
    pcl::PointCloud<pcl::PointXYZI> & ground_cloud)
  {
    // 第一步：按intensity初步分割
    pcl::PointCloud<pcl::PointXYZI> low_intensity_cloud;
    for (const auto & point : cloud->points) {
      if (point.intensity > intensity_threshold_) {
        map_cloud.push_back(point);
      } else {
        low_intensity_cloud.push_back(point);
      }
    }

    // 第二步：对低intensity的点，用空间体素检查Z值范围
    // 使用0.2m体素大小（与terrainAnalysis中的planarVoxelSize一致）
    const float voxel_size = 0.2f;
    // 使用哈希表存储每个体素内点的Z值范围
    struct VoxelZInfo {
      float min_z;
      float max_z;
    };
    std::map<std::pair<int64_t, int64_t>, VoxelZInfo> voxel_z_map;

    for (const auto & point : low_intensity_cloud.points) {
      int64_t gx = static_cast<int64_t>(std::floor(point.x / voxel_size));
      int64_t gy = static_cast<int64_t>(std::floor(point.y / voxel_size));
      auto key = std::make_pair(gx, gy);
      auto it = voxel_z_map.find(key);
      if (it == voxel_z_map.end()) {
        voxel_z_map[key] = {point.z, point.z};
      } else {
        if (point.z < it->second.min_z) it->second.min_z = point.z;
        if (point.z > it->second.max_z) it->second.max_z = point.z;
      }
    }

    // 第三步：根据体素内Z值范围重新分类低intensity点
    for (const auto & point : low_intensity_cloud.points) {
      int64_t gx = static_cast<int64_t>(std::floor(point.x / voxel_size));
      int64_t gy = static_cast<int64_t>(std::floor(point.y / voxel_size));
      auto key = std::make_pair(gx, gy);
      auto it = voxel_z_map.find(key);
      if (it != voxel_z_map.end()) {
        float z_range = it->second.max_z - it->second.min_z;
        if (z_range > ground_z_range_) {
          // Z值变化大，说明有墙壁/障碍物，归为map
          map_cloud.push_back(point);
        } else {
          ground_cloud.push_back(point);
        }
      } else {
        ground_cloud.push_back(point);
      }
    }
  }

private:
  void terrainCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_ = msg;

    if (accumulate_) {
      // Convert to PCL and split by intensity threshold with Z-range check
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*msg, *cloud);

      pcl::PointCloud<pcl::PointXYZI> map_cloud, ground_cloud;
      splitCloudByIntensityAndZRange(cloud, map_cloud, ground_cloud);

      if (save_map_cloud_) {
        *accumulated_map_cloud_ += map_cloud;
      }
      accumulateCloudWithGroundXyDedup(
        ground_cloud, accumulated_ground_cloud_, accumulated_ground_grid_index_);
    }
  }

  void baseFootprintFillCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulate_) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*msg, *cloud);
      accumulateCloudWithGroundXyDedup(
        *cloud, accumulated_base_footprint_fill_cloud_,
        accumulated_base_footprint_fill_grid_index_);
    }
  }

  struct GridKey
  {
    int64_t x;
    int64_t y;
    int64_t z;

    bool operator<(const GridKey & other) const
    {
      if (x != other.x) return x < other.x;
      if (y != other.y) return y < other.y;
      return z < other.z;
    }
  };

  GridKey groundGridKey(const pcl::PointXYZI & point) const
  {
    return GridKey{
      static_cast<int64_t>(std::floor(point.x / ground_xy_leaf_size_)),
      static_cast<int64_t>(std::floor(point.y / ground_xy_leaf_size_)),
      static_cast<int64_t>(std::floor(point.z / ground_z_layer_size_))};
  }

  void accumulateCloudWithGroundXyDedup(
    const pcl::PointCloud<pcl::PointXYZI> & src,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & dst,
    std::map<GridKey, std::size_t> & grid_index)
  {
    if (!ground_xy_dedup_) {
      *dst += src;
      return;
    }

    for (const auto & point : src.points) {
      const auto key = groundGridKey(point);
      auto it = grid_index.find(key);
      if (it == grid_index.end()) {
        grid_index[key] = dst->size();
        dst->push_back(point);
      } else {
        dst->points[it->second] = point;
      }
    }
  }

  void saveServiceCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    (void)request;  // unused
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    if (!ensureSaveDirectory()) {
      response->success = false;
      response->message = "创建保存目录失败。";
      return;
    }

    // Generate filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);
    char timestamp[20];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &now_tm);
    std::string base_filename = save_directory_ + file_prefix_ + timestamp;

    bool success = false;
    std::string message;

    if (accumulate_) {
      // Save accumulated clouds
      if (
        accumulated_map_cloud_->empty() && accumulated_ground_cloud_->empty() &&
        accumulated_base_footprint_fill_cloud_->empty()) {
        response->success = false;
        response->message = "No terrain map accumulated yet.";
        RCLCPP_WARN(this->get_logger(), "Save failed: no terrain map accumulated.");
        return;
      }

      bool has_error = false;
      std::string saved_files;

      if (save_map_cloud_ && !accumulated_map_cloud_->empty()) {
        std::string map_filename = base_filename + "_map.pcd";
        if (savePcdBinary(map_filename, *accumulated_map_cloud_)) {
          saved_files += map_filename + " ";
          RCLCPP_INFO(this->get_logger(), "Saved map cloud to %s (points: %zu)",
                       map_filename.c_str(), accumulated_map_cloud_->size());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to save map cloud to %s", map_filename.c_str());
          has_error = true;
        }
      }

      if (!accumulated_ground_cloud_->empty()) {
        std::string ground_filename = base_filename + "_ground.pcd";
        if (savePcdBinary(ground_filename, *accumulated_ground_cloud_)) {
          saved_files += ground_filename + " ";
          RCLCPP_INFO(this->get_logger(), "Saved ground cloud to %s (points: %zu)",
                       ground_filename.c_str(), accumulated_ground_cloud_->size());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to save ground cloud to %s", ground_filename.c_str());
          has_error = true;
        }
      }

      // 保存 base_footprint 附近填充点云
      if (!accumulated_base_footprint_fill_cloud_->empty()) {
        std::string fill_filename = base_filename + "_base_footprint_fill.pcd";
        if (savePcdBinary(fill_filename, *accumulated_base_footprint_fill_cloud_)) {
          saved_files += fill_filename + " ";
          RCLCPP_INFO(this->get_logger(), "Saved base_footprint fill cloud to %s (points: %zu)",
                       fill_filename.c_str(), accumulated_base_footprint_fill_cloud_->size());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to save base_footprint fill cloud to %s", fill_filename.c_str());
          has_error = true;
        }
      }

      success = !has_error;
      message = success ? ("Saved: " + saved_files) : "Failed to write some PCD files.";
    } else {
      // Save latest cloud split by intensity
      if (!latest_cloud_) {
        response->success = false;
        response->message = "No terrain map received yet.";
        RCLCPP_WARN(this->get_logger(), "Save failed: no terrain map available.");
        return;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*latest_cloud_, *full_cloud);

      // Split by intensity threshold with Z-range check
      pcl::PointCloud<pcl::PointXYZI> map_cloud, ground_cloud;
      splitCloudByIntensityAndZRange(full_cloud, map_cloud, ground_cloud);

      bool has_error = false;
      std::string saved_files;

      if (save_map_cloud_ && !map_cloud.empty()) {
        std::string map_filename = base_filename + "_map.pcd";
        if (savePcdBinary(map_filename, map_cloud)) {
          saved_files += map_filename + " ";
          RCLCPP_INFO(this->get_logger(), "Saved map cloud to %s (points: %zu)",
                       map_filename.c_str(), map_cloud.size());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to save map cloud to %s", map_filename.c_str());
          has_error = true;
        }
      }

      if (!ground_cloud.empty()) {
        std::string ground_filename = base_filename + "_ground.pcd";
        if (savePcdBinary(ground_filename, ground_cloud)) {
          saved_files += ground_filename + " ";
          RCLCPP_INFO(this->get_logger(), "Saved ground cloud to %s (points: %zu)",
                       ground_filename.c_str(), ground_cloud.size());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to save ground cloud to %s", ground_filename.c_str());
          has_error = true;
        }
      }

      success = !has_error;
      message = success ? ("Saved: " + saved_files) : "Failed to write some PCD files.";
    }

    response->success = success;
    response->message = message;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr base_footprint_fill_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;

  std::string save_directory_;
  std::string map_dir_;
  std::string file_prefix_;
  bool accumulate_;
  bool save_map_cloud_;
  float intensity_threshold_;
  float ground_z_range_;
  bool ground_xy_dedup_;
  float ground_xy_leaf_size_;
  float ground_z_layer_size_;

  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_map_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_ground_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_base_footprint_fill_cloud_;
  std::map<GridKey, std::size_t> accumulated_ground_grid_index_;
  std::map<GridKey, std::size_t> accumulated_base_footprint_fill_grid_index_;

  // 标记 main() 中是否已经保存过地图，避免析构函数重复保存
  bool saved_in_main_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SaveTerrainMapNode>();

  // 使用 SingleThreadedExecutor 手动控制执行循环
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // 注册 shutdown 回调，在 ROS 上下文仍然有效时保存地图
  // 当 SIGINT 触发时，rclcpp::shutdown() 被调用，此回调执行
  rclcpp::on_shutdown(
    [node]()
    {
      RCLCPP_INFO(rclcpp::get_logger("save_terrain_map"),
                  "Received shutdown signal, saving map before exit...");
      node->saveMapToFile();
      node->markSavedInMain();
      node->clearAccumulatedClouds();
    });

  // 手动执行循环，每次迭代检查 rclcpp::ok()
  // 这样当 shutdown 触发时能立即退出，避免 spin 卡住的问题
  while (rclcpp::ok()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 移除节点并关闭 executor
  executor.remove_node(node);

  // 显式重置节点 shared_ptr，触发析构
  // 析构函数中由于 saved_in_main_ 为 true，不会再次保存
  node.reset();

  // 调用 rclcpp::shutdown() 确保 ROS 上下文完全退出
  // 注意：在 ROS2 launch 系统中，每个节点是独立进程，shutdown 不会影响其他节点
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
