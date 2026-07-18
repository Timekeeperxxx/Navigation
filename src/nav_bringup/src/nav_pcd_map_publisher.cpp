#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{

using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

struct PublishedCloud
{
  std::string topic;
  PointCloud2 message;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher;
};

struct VoxelKey
{
  int64_t x;
  int64_t y;
  int64_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  size_t operator()(const VoxelKey & key) const
  {
    size_t seed = 0;
    combine(seed, key.x);
    combine(seed, key.y);
    combine(seed, key.z);
    return seed;
  }

private:
  static void combine(size_t & seed, int64_t value)
  {
    const auto hash = std::hash<int64_t>{}(value);
    seed ^= hash + static_cast<size_t>(0x9e3779b97f4a7c15ULL) + (seed << 6U) + (seed >> 2U);
  }
};

struct VoxelAccumulator
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double intensity{0.0};
  uint64_t count{0};
};

class NavPcdMapPublisher : public rclcpp::Node
{
public:
  NavPcdMapPublisher()
  : Node("nav_pcd_map_publisher")
  {
    const auto map_path = declare_parameter<std::string>("map_dir", "");
    const auto ground_path = declare_parameter<std::string>("ground_dir", "");
    const auto planground_path = declare_parameter<std::string>("planground_dir", "");
    frame_id_ = declare_parameter<std::string>("global_frame", "map");
    const auto publish_period = declare_parameter<double>("publish_period", 1.0);
    const auto map_leaf = declare_parameter<double>("map_down_sample", 0.0);
    const auto ground_leaf = declare_parameter<double>("ground_down_sample", 0.0);
    const auto planground_leaf = declare_parameter<double>("planground_down_sample", 0.0);

    if (!std::isfinite(publish_period) || publish_period <= 0.0) {
      throw std::invalid_argument("publish_period 必须是大于 0 的有限数值");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    load_entry("map_dir", map_path, "/mapcloud", map_leaf, qos);
    load_entry("ground_dir", ground_path, "/mapground", ground_leaf, qos);
    load_entry("planground_dir", planground_path, "/planground", planground_leaf, qos);

    if (entries_.empty()) {
      throw std::runtime_error("没有可发布的 PCD 文件");
    }

    publish_all();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(publish_period, 0.2)),
      [this]() {publish_all();});
  }

private:
  static void add_float_field(PointCloud2 & message, const std::string & name, uint32_t offset)
  {
    PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = PointField::FLOAT32;
    field.count = 1;
    message.fields.push_back(std::move(field));
  }

  PointCloud2 make_message(size_t point_count, bool is_dense) const
  {
    constexpr uint32_t kPointStep = sizeof(float) * 4U;
    if (point_count > std::numeric_limits<uint32_t>::max() / kPointStep) {
      throw std::runtime_error("点云数量超过 PointCloud2 单条消息容量上限");
    }
    PointCloud2 message;
    message.header.frame_id = frame_id_;
    message.height = 1;
    message.width = static_cast<uint32_t>(point_count);
    add_float_field(message, "x", 0);
    add_float_field(message, "y", sizeof(float));
    add_float_field(message, "z", sizeof(float) * 2U);
    add_float_field(message, "intensity", sizeof(float) * 3U);
    message.is_bigendian = false;
    message.point_step = kPointStep;
    message.row_step = kPointStep * message.width;
    message.is_dense = is_dense;
    message.data.resize(static_cast<size_t>(message.row_step));
    return message;
  }

  static void write_point(
    PointCloud2 & message, size_t offset, float x, float y, float z, float intensity)
  {
    std::memcpy(message.data.data() + offset, &x, sizeof(float));
    std::memcpy(message.data.data() + offset + sizeof(float), &y, sizeof(float));
    std::memcpy(message.data.data() + offset + sizeof(float) * 2U, &z, sizeof(float));
    std::memcpy(
      message.data.data() + offset + sizeof(float) * 3U, &intensity, sizeof(float));
  }

  PointCloud2 build_message(const PointCloud & cloud) const
  {
    PointCloud2 message = make_message(cloud.size(), cloud.is_dense);

    size_t offset = 0;
    for (const auto & point : cloud.points) {
      write_point(message, offset, point.x, point.y, point.z, point.intensity);
      offset += message.point_step;
    }
    return message;
  }

  static bool voxel_key_for_point(const Point & point, double inverse_leaf, VoxelKey & key)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
      !std::isfinite(point.intensity))
    {
      return false;
    }

    const double x = std::floor(static_cast<double>(point.x) * inverse_leaf);
    const double y = std::floor(static_cast<double>(point.y) * inverse_leaf);
    const double z = std::floor(static_cast<double>(point.z) * inverse_leaf);
    const double kMinIndex = static_cast<double>(std::numeric_limits<int64_t>::min());
    const double kMaxIndex = std::nextafter(
      static_cast<double>(std::numeric_limits<int64_t>::max()), 0.0);
    if (x < kMinIndex || x > kMaxIndex || y < kMinIndex || y > kMaxIndex ||
      z < kMinIndex || z > kMaxIndex)
    {
      return false;
    }

    key = {static_cast<int64_t>(x), static_cast<int64_t>(y), static_cast<int64_t>(z)};
    return true;
  }

  PointCloud2 build_voxel_message(
    const PointCloud & cloud, double leaf_size, size_t & output_count, size_t & skipped_count) const
  {
    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
    voxels.max_load_factor(0.8F);
    voxels.reserve(std::min<size_t>(cloud.size(), 2'000'000U));
    const double inverse_leaf = 1.0 / leaf_size;

    for (const auto & point : cloud.points) {
      VoxelKey key{};
      if (!voxel_key_for_point(point, inverse_leaf, key)) {
        ++skipped_count;
        continue;
      }
      auto & accumulator = voxels[key];
      accumulator.x += point.x;
      accumulator.y += point.y;
      accumulator.z += point.z;
      accumulator.intensity += point.intensity;
      ++accumulator.count;
    }

    output_count = voxels.size();
    PointCloud2 message = make_message(output_count, skipped_count == 0);
    size_t offset = 0;
    for (const auto & item : voxels) {
      const auto & accumulator = item.second;
      const double count = static_cast<double>(accumulator.count);
      write_point(
        message,
        offset,
        static_cast<float>(accumulator.x / count),
        static_cast<float>(accumulator.y / count),
        static_cast<float>(accumulator.z / count),
        static_cast<float>(accumulator.intensity / count));
      offset += message.point_step;
    }
    return message;
  }

  void load_entry(
    const std::string & parameter_name,
    const std::string & file_path,
    const std::string & topic,
    double leaf_size,
    const rclcpp::QoS & qos)
  {
    if (file_path.empty()) {
      RCLCPP_WARN(get_logger(), "%s 为空，跳过 %s", parameter_name.c_str(), topic.c_str());
      return;
    }
    if (!std::isfinite(leaf_size) || leaf_size < 0.0) {
      throw std::invalid_argument(parameter_name + " 的体素大小必须是非负有限数值");
    }

    // 每次只保留当前层的原始 PCL 点云；消息构造完成后立即释放，再加载下一层。
    auto raw_cloud = std::make_shared<PointCloud>();
    RCLCPP_INFO(get_logger(), "开始加载 %s：%s", parameter_name.c_str(), file_path.c_str());
    if (pcl::io::loadPCDFile<Point>(file_path, *raw_cloud) < 0) {
      throw std::runtime_error("PCD 加载失败：" + file_path);
    }
    const auto raw_count = raw_cloud->size();

    size_t output_count = raw_count;
    size_t skipped_count = 0;
    PointCloud2 message;
    if (leaf_size > 0.0 && raw_cloud->size() > 1) {
      // PCL VoxelGrid 使用 32 位线性体素索引，大范围地图可能溢出并退回原始点云。
      // 这里使用 64 位三维哈希键，避免 Scene35 这类地图失去降采样。
      message = build_voxel_message(*raw_cloud, leaf_size, output_count, skipped_count);
    } else {
      message = build_message(*raw_cloud);
    }

    PublishedCloud entry;
    entry.topic = topic;
    entry.message = std::move(message);
    entry.publisher = create_publisher<PointCloud2>(topic, qos);
    entries_.push_back(std::move(entry));

    RCLCPP_INFO(
      get_logger(),
      "完成 %s：%s -> %s, raw_points=%zu, points=%zu, down_sample=%.3f",
      parameter_name.c_str(), file_path.c_str(), topic.c_str(), raw_count, output_count, leaf_size);
    if (skipped_count > 0) {
      RCLCPP_WARN(
        get_logger(), "%s 跳过了 %zu 个包含非有限值或超范围坐标的点",
        parameter_name.c_str(), skipped_count);
    }
  }

  void publish_all()
  {
    const auto stamp = now();
    for (auto & entry : entries_) {
      entry.message.header.stamp = stamp;
      entry.publisher->publish(entry.message);
    }
  }

  std::string frame_id_{"map"};
  std::vector<PublishedCloud> entries_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<NavPcdMapPublisher>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & error) {
    std::cerr << "nav_pcd_map_publisher 启动失败：" << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
