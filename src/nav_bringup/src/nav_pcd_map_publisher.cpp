#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
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

class DisjointSet
{
public:
  explicit DisjointSet(size_t count)
  : parent_(count), rank_(count, 0)
  {
    std::iota(parent_.begin(), parent_.end(), 0);
  }

  size_t find(size_t index)
  {
    size_t root = index;
    while (parent_[root] != root) {
      root = parent_[root];
    }
    while (parent_[index] != index) {
      const auto next = parent_[index];
      parent_[index] = root;
      index = next;
    }
    return root;
  }

  bool unite(size_t first, size_t second)
  {
    auto first_root = find(first);
    auto second_root = find(second);
    if (first_root == second_root) {
      return false;
    }
    if (rank_[first_root] < rank_[second_root]) {
      std::swap(first_root, second_root);
    }
    parent_[second_root] = first_root;
    if (rank_[first_root] == rank_[second_root]) {
      ++rank_[first_root];
    }
    return true;
  }

private:
  std::vector<size_t> parent_;
  std::vector<uint8_t> rank_;
};

struct BridgeCandidate
{
  size_t component_a{0};
  size_t component_b{0};
  size_t point_a{0};
  size_t point_b{0};
  double distance_squared{0.0};
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
    planground_bridge_enabled_ =
      declare_parameter<bool>("planground_bridge_enabled", false);
    planground_bridge_neighbor_radius_ =
      declare_parameter<double>("planground_bridge_neighbor_radius", 0.2);
    planground_bridge_max_gap_ =
      declare_parameter<double>("planground_bridge_max_gap", 0.35);
    planground_bridge_point_spacing_ =
      declare_parameter<double>("planground_bridge_point_spacing", 0.05);
    planground_bridge_min_component_points_ =
      declare_parameter<int64_t>("planground_bridge_min_component_points", 20);
    planground_bridge_max_bridges_ =
      declare_parameter<int64_t>("planground_bridge_max_bridges", 64);

    if (!std::isfinite(publish_period) || publish_period <= 0.0) {
      throw std::invalid_argument("publish_period 必须是大于 0 的有限数值");
    }
    validate_bridge_parameters();

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

  void validate_bridge_parameters() const
  {
    if (!std::isfinite(planground_bridge_neighbor_radius_) ||
      planground_bridge_neighbor_radius_ <= 0.0)
    {
      throw std::invalid_argument("planground_bridge_neighbor_radius 必须是大于 0 的有限数值");
    }
    if (!std::isfinite(planground_bridge_max_gap_) ||
      planground_bridge_max_gap_ <= planground_bridge_neighbor_radius_)
    {
      throw std::invalid_argument(
              "planground_bridge_max_gap 必须大于 planground_bridge_neighbor_radius");
    }
    if (!std::isfinite(planground_bridge_point_spacing_) ||
      planground_bridge_point_spacing_ <= 0.0 ||
      planground_bridge_point_spacing_ > planground_bridge_neighbor_radius_)
    {
      throw std::invalid_argument(
              "planground_bridge_point_spacing 必须大于 0 且不大于连接半径");
    }
    if (planground_bridge_min_component_points_ <= 0) {
      throw std::invalid_argument("planground_bridge_min_component_points 必须大于 0");
    }
    if (planground_bridge_max_bridges_ < 0) {
      throw std::invalid_argument("planground_bridge_max_bridges 不能小于 0");
    }
  }

  size_t bridge_planground_gaps(PointCloud & cloud, const char * stage)
  {
    if (!planground_bridge_enabled_ || cloud.size() < 2 ||
      planground_bridge_max_bridges_ == 0)
    {
      return 0;
    }

    // KD 树始终引用未修改的原点云，所有补点在搜索结束后一次性追加。
    const auto search_cloud = cloud.makeShared();
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(search_cloud);

    DisjointSet components(search_cloud->size());
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_distances;
    for (size_t i = 0; i < search_cloud->size(); ++i) {
      neighbor_indices.clear();
      neighbor_distances.clear();
      kdtree.radiusSearch(
        search_cloud->points[i],
        planground_bridge_neighbor_radius_,
        neighbor_indices,
        neighbor_distances);
      for (const auto neighbor : neighbor_indices) {
        if (neighbor >= 0 && static_cast<size_t>(neighbor) != i) {
          components.unite(i, static_cast<size_t>(neighbor));
        }
      }
    }

    std::vector<size_t> component_sizes(search_cloud->size(), 0);
    for (size_t i = 0; i < search_cloud->size(); ++i) {
      ++component_sizes[components.find(i)];
    }

    size_t component_count = 0;
    for (const auto size : component_sizes) {
      if (size > 0) {
        ++component_count;
      }
    }
    if (component_count <= 1) {
      RCLCPP_INFO(
        get_logger(), "fill_footprint（%s）在 %.3fm 半径下已经连通，无需补点",
        stage, planground_bridge_neighbor_radius_);
      return 0;
    }

    // 每对分量只保留距离最近的端点，随后用 Kruskal 选择最少的连接边。
    std::map<std::pair<size_t, size_t>, BridgeCandidate> closest_candidates;
    const auto minimum_component_size =
      static_cast<size_t>(planground_bridge_min_component_points_);
    for (size_t i = 0; i < search_cloud->size(); ++i) {
      const auto component_i = components.find(i);
      if (component_sizes[component_i] < minimum_component_size) {
        continue;
      }
      neighbor_indices.clear();
      neighbor_distances.clear();
      kdtree.radiusSearch(
        search_cloud->points[i],
        planground_bridge_max_gap_,
        neighbor_indices,
        neighbor_distances);
      for (size_t neighbor_offset = 0; neighbor_offset < neighbor_indices.size();
        ++neighbor_offset)
      {
        const auto neighbor = neighbor_indices[neighbor_offset];
        if (neighbor < 0 || static_cast<size_t>(neighbor) <= i) {
          continue;
        }
        const auto neighbor_index = static_cast<size_t>(neighbor);
        const auto component_j = components.find(neighbor_index);
        if (component_i == component_j ||
          component_sizes[component_j] < minimum_component_size)
        {
          continue;
        }

        const auto component_pair = std::minmax(component_i, component_j);
        BridgeCandidate candidate;
        candidate.component_a = component_i;
        candidate.component_b = component_j;
        candidate.point_a = i;
        candidate.point_b = neighbor_index;
        candidate.distance_squared = static_cast<double>(neighbor_distances[neighbor_offset]);
        const auto existing = closest_candidates.find(component_pair);
        if (existing == closest_candidates.end() ||
          candidate.distance_squared < existing->second.distance_squared)
        {
          closest_candidates[component_pair] = candidate;
        }
      }
    }

    std::vector<BridgeCandidate> candidates;
    candidates.reserve(closest_candidates.size());
    for (const auto & item : closest_candidates) {
      candidates.push_back(item.second);
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const BridgeCandidate & first, const BridgeCandidate & second) {
        return first.distance_squared < second.distance_squared;
      });

    DisjointSet bridge_components(search_cloud->size());
    std::vector<Point> bridge_points;
    size_t bridge_count = 0;
    for (const auto & candidate : candidates) {
      if (bridge_count >= static_cast<size_t>(planground_bridge_max_bridges_) ||
        !bridge_components.unite(candidate.component_a, candidate.component_b))
      {
        continue;
      }

      const auto & start = search_cloud->points[candidate.point_a];
      const auto & end = search_cloud->points[candidate.point_b];
      const auto distance = std::sqrt(candidate.distance_squared);
      const auto segment_count = std::max<size_t>(
        2, static_cast<size_t>(std::ceil(distance / planground_bridge_point_spacing_)));
      for (size_t segment = 1; segment < segment_count; ++segment) {
        const auto ratio =
          static_cast<float>(segment) / static_cast<float>(segment_count);
        Point point;
        point.x = start.x + (end.x - start.x) * ratio;
        point.y = start.y + (end.y - start.y) * ratio;
        point.z = start.z + (end.z - start.z) * ratio;
        point.intensity = start.intensity + (end.intensity - start.intensity) * ratio;
        bridge_points.push_back(point);
      }
      ++bridge_count;
      RCLCPP_INFO(
        get_logger(),
        "fill_footprint（%s）自动连接 #%zu: gap=%.3fm, (%.3f, %.3f, %.3f) -> "
        "(%.3f, %.3f, %.3f)",
        stage, bridge_count, distance, start.x, start.y, start.z, end.x, end.y, end.z);
    }

    cloud.points.insert(cloud.points.end(), bridge_points.begin(), bridge_points.end());
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;
    RCLCPP_INFO(
      get_logger(),
      "fill_footprint（%s）连通修复完成: 原分量=%zu, 候选连接=%zu, 已连接=%zu, 补点=%zu",
      stage, component_count, candidates.size(), bridge_count, bridge_points.size());
    return bridge_points.size();
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

  PointCloud voxel_downsample(
    const PointCloud & cloud, double leaf_size, size_t & skipped_count) const
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

    PointCloud output;
    output.points.reserve(voxels.size());
    for (const auto & item : voxels) {
      const auto & accumulator = item.second;
      const double count = static_cast<double>(accumulator.count);
      Point point;
      point.x = static_cast<float>(accumulator.x / count);
      point.y = static_cast<float>(accumulator.y / count);
      point.z = static_cast<float>(accumulator.z / count);
      point.intensity = static_cast<float>(accumulator.intensity / count);
      output.points.push_back(point);
    }
    output.width = static_cast<uint32_t>(output.size());
    output.height = 1;
    output.is_dense = skipped_count == 0;
    return output;
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
    size_t bridge_point_count = 0;
    if (parameter_name == "planground_dir") {
      bridge_point_count = bridge_planground_gaps(*raw_cloud, "原始点云");
    }

    size_t output_count = raw_cloud->size();
    size_t skipped_count = 0;
    PointCloud2 message;
    if (leaf_size > 0.0 && raw_cloud->size() > 1) {
      // PCL VoxelGrid 使用 32 位线性体素索引，大范围地图可能溢出并退回原始点云。
      // 这里使用 64 位三维哈希键，避免 Scene35 这类地图失去降采样。
      auto downsampled_cloud = voxel_downsample(*raw_cloud, leaf_size, skipped_count);
      // 体素质心会轻微移动端点，因此必须针对真正发布的点云再做一次连通检查。
      if (parameter_name == "planground_dir") {
        bridge_point_count += bridge_planground_gaps(downsampled_cloud, "降采样后");
      }
      output_count = downsampled_cloud.size();
      message = build_message(downsampled_cloud);
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
      "完成 %s：%s -> %s, raw_points=%zu, bridge_points=%zu, points=%zu, down_sample=%.3f",
      parameter_name.c_str(), file_path.c_str(), topic.c_str(), raw_count, bridge_point_count,
      output_count, leaf_size);
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
  bool planground_bridge_enabled_{false};
  double planground_bridge_neighbor_radius_{0.2};
  double planground_bridge_max_gap_{0.35};
  double planground_bridge_point_spacing_{0.05};
  int64_t planground_bridge_min_component_points_{20};
  int64_t planground_bridge_max_bridges_{64};
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
