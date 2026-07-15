/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <global_planner/hybrid_a_star.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <algorithm>
#include <limits>

//=============================================================================
// v20 - Unified Map with Strong Planground Preference (规划主体在planground上)
//
// 核心设计:
// 将地面点云与planground点云融合为一张地图，但设置不同的代价值
// - Planground点: intensity = 0.0 (无额外代价，规划主体在planground上)
// - 地面点: intensity = 根据距离planground的距离 + 绕路比平衡因子 + 距离平衡因子计算
//
// 关键设计原则 (v20改进):
// 1. 规划的主体在planground上
//    -> planground点代价为0，地面点有显著正代价
//    -> 地面点的基础代价至少为 planground_bias_ * 1.5，确保planground优先
//    -> planground_bias_ 默认值从0.5提高到1.0，使地面点代价更高
//
// 2. 绕路比平衡 (detour_ratio_threshold_)
//    -> 使用平滑的指数函数，避免极端值
//    -> 当detour_ratio接近threshold时，平衡因子平滑过渡
//    -> 限制范围在 [0.8, 1.5] 之间，避免过度调整（v20: 下限从0.5提高到0.8）
//    -> 默认threshold从1.5提高到2.0，只有planground绕路超过2倍才允许走地面
//
// 3. 距离平衡 (distance_balance_threshold_)
//    -> 短距离 (straight_line_distance < distance_balance_threshold_):
//       地面点代价降低 (distance_balance_factor < 1.0)
//       允许更激进的抄近路，因为短距离即使走地面也不会绕太远
//    -> 长距离 (straight_line_distance >= distance_balance_threshold_):
//       地面点代价升高 (distance_balance_factor > 1.0)
//       避免长距离走地面导致绕远路，鼓励沿planground规划
//    -> v20: 限制范围从 [0.5, 1.5] 改为 [0.8, 1.3]，避免过度调整
//
// 4. 距离惩罚 - 避免走太远的地面路径
//    -> 当距离planground超过 max_ground_bridge_length_ 时，代价急剧上升
//    -> v20: max_ground_bridge_length_ 默认值从1.0降低到0.5
//    -> 这更严格地限制了地面路径的长度，避免绕远路
//
// 5. v20关键修复: 在planOnHybrid()中更新地面点intensity后，重建kdtree
//    -> 确保A*规划器使用最新的intensity值
//    -> 之前版本没有重建kdtree，导致A*使用的intensity是旧的占位值
//
// 参数调优建议 (v20):
// - planground_bias_: 1.0-2.0 (控制地面点的基础代价，越高越倾向于planground)
// - max_ground_bridge_length_: 0.3-0.5 (控制地面点的有效范围，越小越严格)
// - detour_ratio_threshold_: 1.8-2.5 (控制何时允许走地面抄近路)
// - distance_balance_threshold_: 3.0-8.0 (距离平衡阈值)
//   - 小于此距离：地面代价降低，允许抄近路
//   - 大于此距离：地面代价升高，避免绕远路
// - max_ground_cost_: 5.0-10.0 (限制最大代价，防止规划失败)
//=============================================================================

Hybrid_A_Star::Hybrid_A_Star(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_planground,
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground,
    std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros,
    double a_star_expanding_radius)
  : pc_planground_(new pcl::PointCloud<pcl::PointXYZI>)
  , pc_ground_(new pcl::PointCloud<pcl::PointXYZI>)
  , pc_hybrid_(new pcl::PointCloud<pcl::PointXYZI>)
  , kdtree_planground_(new pcl::KdTreeFLANN<pcl::PointXYZI>)
  , kdtree_ground_(new pcl::KdTreeFLANN<pcl::PointXYZI>)
  , kdtree_hybrid_(new pcl::KdTreeFLANN<pcl::PointXYZI>)
  , perception_ros_(perception_ros)
  , a_star_expanding_radius_(a_star_expanding_radius)
  , a_star_planner_(nullptr)
{
  // Copy the point clouds
  *pc_planground_ = *pc_planground;
  *pc_ground_ = *pc_ground;
  
  // Build KD-trees
  kdtree_planground_->setInputCloud(pc_planground_);
  kdtree_ground_->setInputCloud(pc_ground_);
  
  // Build the initial hybrid cloud
  rebuildHybridCloud();
}

Hybrid_A_Star::~Hybrid_A_Star()
{
  if (a_star_planner_) {
    delete a_star_planner_;
  }
}

void Hybrid_A_Star::updatePlanground(pcl::PointCloud<pcl::PointXYZI>::Ptr new_planground)
{
  *pc_planground_ = *new_planground;
  kdtree_planground_->setInputCloud(pc_planground_);
  rebuildHybridCloud();
}

void Hybrid_A_Star::updateGround(pcl::PointCloud<pcl::PointXYZI>::Ptr new_ground)
{
  *pc_ground_ = *new_ground;
  kdtree_ground_->setInputCloud(pc_ground_);
  rebuildHybridCloud();
}

double Hybrid_A_Star::straightLineDistance(const pcl::PointXYZI& a, const pcl::PointXYZI& b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void Hybrid_A_Star::rebuildHybridCloud()
{
  pc_hybrid_->clear();
  is_from_planground_.clear();
  
  // Phase 1: Add ALL planground points with zero cost
  for (size_t i = 0; i < pc_planground_->size(); ++i) {
    pcl::PointXYZI pt = pc_planground_->points[i];
    pt.intensity = 0.0f;
    pc_hybrid_->push_back(pt);
    is_from_planground_.push_back(true);
  }
  
  // Phase 2: Add ALL ground points with distance-to-planground placeholder
  for (size_t i = 0; i < pc_ground_->size(); ++i) {
    pcl::PointXYZI pt = pc_ground_->points[i];
    double dist_to_planground = distanceToNearestPlanground(pt);
    pt.intensity = static_cast<float>(dist_to_planground);
    pc_hybrid_->push_back(pt);
    is_from_planground_.push_back(false);
  }
  
  // Phase 3: Downsample ONLY the ground portion of the hybrid cloud
  // Preserve ALL planground points at their original density to maintain
  // the planground's advantage as the preferred planning surface.
  // Only downsample ground points to reduce computational cost.
  if (hybrid_downsample_leaf_size_ > 0.0 && pc_ground_->size() > 1000) {
    // CRITICAL FIX: Verify that is_from_planground_ is in sync with pc_hybrid_
    // Before downsampling, ensure the sizes match
    if (is_from_planground_.size() != pc_hybrid_->size()) {
      RCLCPP_WARN(perception_ros_->get_logger(),
        "[Hybrid] Size mismatch before downsampling: is_from_planground_=%lu, pc_hybrid_=%lu. Rebuilding source tracking.",
        is_from_planground_.size(), pc_hybrid_->size());
      // Rebuild is_from_planground_ from scratch
      is_from_planground_.clear();
      for (size_t i = 0; i < pc_planground_->size() && i < pc_hybrid_->size(); ++i) {
        is_from_planground_.push_back(true);
      }
      for (size_t i = pc_planground_->size(); i < pc_hybrid_->size(); ++i) {
        is_from_planground_.push_back(false);
      }
    }
    
    // Extract ground points from hybrid cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_only(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<size_t> ground_indices;
    for (size_t i = pc_planground_->size(); i < pc_hybrid_->size(); ++i) {
      ground_only->push_back(pc_hybrid_->points[i]);
      ground_indices.push_back(i);
    }
    
    // Downsample ground points
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setInputCloud(ground_only);
    voxel_grid.setLeafSize(hybrid_downsample_leaf_size_, hybrid_downsample_leaf_size_, hybrid_downsample_leaf_size_);
    voxel_grid.filter(*downsampled_ground);
    
    // Map downsampled ground points back to source tracking
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_ground_original;
    kdtree_ground_original.setInputCloud(ground_only);
    
    std::vector<bool> downsampled_source;
    for (size_t i = 0; i < downsampled_ground->size(); ++i) {
      std::vector<int> nearest_idx(1);
      std::vector<float> nearest_dist(1);
      kdtree_ground_original.nearestKSearch(downsampled_ground->points[i], 1, nearest_idx, nearest_dist);
      if (nearest_idx[0] >= 0 && static_cast<size_t>(nearest_idx[0]) < ground_indices.size()) {
        size_t original_idx = ground_indices[nearest_idx[0]];
        if (original_idx < is_from_planground_.size()) {
          downsampled_source.push_back(is_from_planground_[original_idx]);
        } else {
          downsampled_source.push_back(false);
        }
      } else {
        downsampled_source.push_back(false);
      }
    }
    
    // Rebuild hybrid cloud: planground points (preserved) + downsampled ground points
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_hybrid(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<bool> new_source;
    
    // Add all planground points (preserved at original density)
    for (size_t i = 0; i < pc_planground_->size(); ++i) {
      new_hybrid->push_back(pc_hybrid_->points[i]);
      new_source.push_back(true);
    }
    
    // Add downsampled ground points
    for (size_t i = 0; i < downsampled_ground->size(); ++i) {
      new_hybrid->push_back(downsampled_ground->points[i]);
      new_source.push_back(false);
    }
    
    *pc_hybrid_ = *new_hybrid;
    is_from_planground_ = new_source;
    
    RCLCPP_INFO(perception_ros_->get_logger(),
      "[Hybrid] Downsampled ground: %lu -> %lu points (leaf=%.3f), planground preserved: %lu points",
      ground_only->size(), downsampled_ground->size(), hybrid_downsample_leaf_size_, pc_planground_->size());
  }
  
  // CRITICAL FIX: Ensure is_from_planground_ is always in sync with pc_hybrid_
  if (is_from_planground_.size() != pc_hybrid_->size()) {
    RCLCPP_ERROR(perception_ros_->get_logger(),
      "[Hybrid] CRITICAL: Size mismatch after rebuild: is_from_planground_=%lu, pc_hybrid_=%lu. Rebuilding.",
      is_from_planground_.size(), pc_hybrid_->size());
    is_from_planground_.clear();
    for (size_t i = 0; i < pc_hybrid_->size(); ++i) {
      is_from_planground_.push_back(i < pc_planground_->size());
    }
  }
  
  kdtree_hybrid_->setInputCloud(pc_hybrid_);
  
  RCLCPP_INFO(perception_ros_->get_logger(),
    "[Hybrid] Rebuilt hybrid cloud: %lu points (%lu planground + %lu ground)",
    pc_hybrid_->size(), pc_planground_->size(), pc_ground_->size());
}

double Hybrid_A_Star::calculatePathLength(const std::vector<unsigned int>& path, 
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  if (path.size() < 2) return 0.0;
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += straightLineDistance(cloud->points[path[i-1]], cloud->points[path[i]]);
  }
  return length;
}

double Hybrid_A_Star::distanceToNearestPlanground(const pcl::PointXYZI& pt)
{
  if (pc_planground_->empty()) {
    return std::numeric_limits<double>::max();
  }
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  pcl::PointXYZI search_pt;
  search_pt.x = pt.x;
  search_pt.y = pt.y;
  search_pt.z = pt.z;
  search_pt.intensity = 0.0f;
  if (kdtree_planground_->nearestKSearch(search_pt, 1, indices, distances) > 0) {
    return std::sqrt(distances[0]);
  }
  return std::numeric_limits<double>::max();
}

double Hybrid_A_Star::calculateGroundCost(double dist_to_planground, 
                                           double planground_path_length,
                                           double straight_line_distance)
{
  //===========================================================================
  // v23 - Ultra-Strong Planground Preference with Maximum Anti-Detour (规划主体在planground上)
  //
  // 核心设计:
  // 将地面点云与planground点云融合为一张地图，但设置不同的代价值
  // - Planground点: intensity = 0.0 (无额外代价，规划主体在planground上)
  // - 地面点: intensity = 根据距离planground的距离 + 绕路比平衡因子 + 距离平衡因子计算
  //
  // 关键设计原则 (v23改进):
  // 1. 规划的主体在planground上
  //    -> planground点代价为0，地面点有显著正代价
  //    -> planground_bias_从5.0提高到10.0，大幅提高地面点基础代价
  //    -> min_ground_cost_从50.0提高到100.0，确保地面点始终比planground点贵很多
  //
  // 2. 绕路比平衡 (detour_ratio_threshold_)
  //    -> 使用平滑的指数函数，避免极端值
  //    -> 当detour_ratio接近threshold时，平衡因子平滑过渡
  //    -> 限制范围在 [detour_balance_factor_lower_bound_, detour_balance_factor_upper_bound_] 之间
  //       (v23: 下限从0.3提高到0.5，上限从2.0提高到3.0)
  //    -> 默认threshold从3.0提高到4.0，只有当planground绕路超过4.0倍时才允许走地面
  //    -> 避免轻微绕路就走地面导致更严重的绕路
  //
  // 3. 距离平衡 (distance_balance_threshold_)
  //    -> 短距离 (straight_line_distance < distance_balance_threshold_):
  //       地面点代价降低 (distance_balance_factor < 1.0)
  //       允许更激进的抄近路，因为短距离即使走地面也不会绕太远
  //    -> 长距离 (straight_line_distance >= distance_balance_threshold_):
  //       地面点代价升高 (distance_balance_factor > 1.0)
  //       避免长距离走地面导致绕远路，鼓励沿planground规划
  //    -> 范围 [0.5, 1.5]
  //    -> distance_balance_threshold_从2.0降低到1.5，更多路径被视为"长距离"避免绕路
  //
  // 4. 距离惩罚 - 避免走太远的地面路径
  //    -> 当距离planground超过 max_ground_bridge_length_ 时，代价急剧上升
  //    -> max_ground_bridge_length_从0.2降低到0.15，更严格限制地面路径长度
  //    -> 这更严格地限制了地面路径的长度，避免绕远路
  //
  // 5. v23增强: 地面路径长度惩罚 (ground_path_length_penalty_)
  //    -> 地面路径越长，代价越高
  //    -> 每走一步地面，代价乘以 (1 + ground_path_length_penalty_ * ground_steps / total_steps)
  //    -> ground_path_length_penalty_从2.0提高到5.0，确保长地面路径的代价远高于短地面路径
  //
  // 参数调优建议 (v23):
  // - planground_bias_: 10.0 (控制地面点的基础代价，越高越倾向于planground)
  // - max_ground_bridge_length_: 0.15 (控制地面点的有效范围，越小越严格)
  // - detour_ratio_threshold_: 4.0 (控制何时允许走地面抄近路)
  // - distance_balance_threshold_: 1.5 (距离平衡阈值)
  //   - 小于此距离：地面代价降低，允许抄近路
  //   - 大于此距离：地面代价升高，避免绕远路
  // - max_ground_cost_: 200.0 (限制最大代价，防止规划失败)
  // - min_ground_cost_: 100.0 (最小地面代价，确保planground优先)
  // - ground_path_length_penalty_: 5.0 (地面路径长度惩罚系数)
  // - detour_balance_factor_lower_bound_: 0.5 (绕路比平衡因子下限)
  // - detour_balance_factor_upper_bound_: 3.0 (绕路比平衡因子上限)
  //===========================================================================
  
  // Step 1: 计算基础代价 - 基于距离planground的距离
  // 使用二次函数，使靠近planground的地面点代价低，远离的代价高
  // v24: planground_bias_从10.0提高到15.0，进一步提高地面点基础代价
  // 确保规划主体在planground上，只有planground确实严重绕路时才考虑走地面
  double ratio = dist_to_planground / max_ground_bridge_length_;
  double base_cost = planground_bias_ * (1.0 + ratio * ratio);
  
  // Step 2: 计算绕路比平衡因子
  // detour_ratio = planground路径长度 / 直线距离
  // 使用平滑的指数函数，避免极端值
  // 核心思想：
  // - 当 detour_ratio < detour_ratio_threshold_ (planground路径较直):
  //   平衡因子 > 1.0，地面代价升高，规划器不会离开planground
  // - 当 detour_ratio > detour_ratio_threshold_ (planground绕路):
  //   平衡因子 < 1.0，地面代价降低，允许规划器走地面抄近路
  // - 当 detour_ratio == detour_ratio_threshold_:
  //   平衡因子 = 1.0，地面代价不变
  //
  // v23: detour_ratio_threshold_从3.0提高到4.0
  // 只有当planground绕路超过4.0倍时才允许走地面
  // 避免轻微绕路就走地面导致更严重的绕路
  // v23: 下限从0.3提高到0.5，上限从2.0提高到3.0
  // 即使planground严重绕路，地面代价也不应降得太低
  // 当planground路径很直时，地面代价大幅升高
  double detour_ratio = planground_path_length / (straight_line_distance + 0.0001);
  
  // 使用平滑的指数函数
  // k = 0.5 使过渡平滑，避免极端调整
  double k = 0.5;
  double detour_balance_factor = std::exp(k * (detour_ratio_threshold_ - detour_ratio));
  detour_balance_factor = std::max(detour_balance_factor_lower_bound_, std::min(detour_balance_factor_upper_bound_, detour_balance_factor));
  
  // Step 3: 计算距离平衡因子
  // 核心思想：根据起点到终点的直线距离调整地面代价
  // - 短距离 (straight_line_distance < distance_balance_threshold_):
  //   地面代价降低，允许抄近路。因为短距离即使走地面也不会绕太远
  // - 长距离 (straight_line_distance >= distance_balance_threshold_):
  //   地面代价升高，避免绕远路。长距离走地面可能导致显著绕路
  //
  // v23: distance_balance_threshold_从2.0降低到1.5
  // 更多路径被视为"长距离"，避免绕路
  double distance_balance_factor = 1.0 + 0.5 * std::tanh((straight_line_distance - distance_balance_threshold_) / distance_balance_threshold_);
  distance_balance_factor = std::max(0.5, std::min(1.5, distance_balance_factor));
  
  // Step 4: 距离惩罚 - 当距离planground超过阈值时，代价急剧上升
  // 这确保规划器不会走太远的地面路径
  // 使用指数函数，使超过阈值后代价急剧上升
  // v23: max_ground_bridge_length_从0.2降低到0.15
  // 更严格限制地面路径长度，避免绕路
  double distance_penalty = 1.0;
  if (dist_to_planground > max_ground_bridge_length_) {
    // 超过阈值后，代价呈指数增长
    double excess_ratio = (dist_to_planground - max_ground_bridge_length_) / max_ground_bridge_length_;
    distance_penalty = std::exp(excess_ratio * 2.0);
  }
  
  // Step 5: 计算最终代价
  // 最终代价 = 基础代价 * 绕路比平衡因子 * 距离平衡因子 * 距离惩罚
  // 其中：
  // - 基础代价：确保靠近planground的地面点代价低，远离的代价高
  // - 绕路比平衡因子：当planground绕路时降低地面代价，允许抄近路
  // - 距离平衡因子：短距离降低地面代价允许抄近路，长距离升高地面代价避免绕远路
  // - 距离惩罚：当距离planground太远时代价急剧上升，避免绕远路
  double cost = base_cost * detour_balance_factor * distance_balance_factor * distance_penalty;

  // v23: 使用固定最小代价100.0，确保地面点始终比planground点贵很多
  // 即使距离planground很近，地面点也应该比planground点贵很多
  // 这样规划器会优先选择planground，除非走地面确实能显著缩短路径
  // v23: min_ground_cost_从50.0提高到100.0，确保地面点始终比planground点贵很多
  cost = std::max(cost, min_ground_cost_);

  // 限制最大代价，防止路径规划失败
  // v23: max_ground_cost_从100.0提高到200.0，提高上限以允许更高的最小代价
  return std::min(cost, max_ground_cost_);
}

double Hybrid_A_Star::calculateEdgePenalty(const pcl::PointXYZI& pt)
{
  // v25: Edge penalty - penalize ground points near cloud edges
  //
  // Core idea: Points near the edge of the ground point cloud have fewer neighbors
  // within a given radius. By counting the number of neighbors, we can estimate
  // how close a point is to the cloud boundary.
  //
  // Edge penalty formula:
  //   neighbor_count = number of ground points within edge_penalty_radius_
  //   density_ratio = neighbor_count / expected_density
  //   edge_penalty = edge_penalty_weight_ * exp(-edge_penalty_falloff_rate_ * density_ratio)
  //
  // Where expected_density is the typical number of points in a well-populated area
  // (estimated from the average density of the ground cloud).
  //
  // This produces:
  // - High penalty for points near edges (few neighbors, low density_ratio)
  // - Low penalty for points in dense areas (many neighbors, high density_ratio)
  // - Smooth transition controlled by edge_penalty_falloff_rate_
  
  if (edge_penalty_weight_ <= 0.0 || edge_penalty_radius_ <= 0.0) {
    return 0.0;
  }
  
  // Count neighbors within the search radius
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_distances;
  int num_neighbors = kdtree_ground_->radiusSearch(pt, edge_penalty_radius_, neighbor_indices, neighbor_distances);
  
  // Subtract 1 to exclude the point itself
  if (num_neighbors > 0) {
    num_neighbors--;
  }
  
  // Estimate expected density: for a well-populated area with leaf_size=0.15,
  // we expect roughly (radius/leaf_size)^3 points in 3D, or (radius/leaf_size)^2 in 2D
  // Use a conservative estimate: at least 10 neighbors means "not an edge"
  const double MIN_NEIGHBORS_FOR_SAFE = 10.0;
  
  // density_ratio: 0.0 = edge (no neighbors), 1.0+ = safe area (many neighbors)
  double density_ratio = static_cast<double>(num_neighbors) / MIN_NEIGHBORS_FOR_SAFE;
  
  // Compute edge penalty using exponential falloff
  // When density_ratio = 0 (edge): penalty = edge_penalty_weight_
  // When density_ratio = 1 (safe): penalty = edge_penalty_weight_ * exp(-falloff_rate)
  // When density_ratio >> 1 (dense): penalty ≈ 0
  double edge_penalty = edge_penalty_weight_ * std::exp(-edge_penalty_falloff_rate_ * density_ratio);
  
  return edge_penalty;

}

bool Hybrid_A_Star::planOnPlangroundOnly(const pcl::PointXYZI& start_pose,
                                           const pcl::PointXYZI& goal_pose,
                                           double& path_length,
                                           std::vector<unsigned int>* path_out)
{
  if (pc_planground_->size() < 2) {
    path_length = straightLineDistance(start_pose, goal_pose);
    return false;
  }
  
  std::vector<int> start_idx(1), goal_idx(1);
  std::vector<float> start_dist(1), goal_dist(1);
  
  pcl::PointXYZI search_pt;
  search_pt.x = start_pose.x;
  search_pt.y = start_pose.y;
  search_pt.z = start_pose.z;
  search_pt.intensity = 0.0f;
  
  if (kdtree_planground_->nearestKSearch(search_pt, 1, start_idx, start_dist) == 0) {
    path_length = straightLineDistance(start_pose, goal_pose);
    return false;
  }
  
  search_pt.x = goal_pose.x;
  search_pt.y = goal_pose.y;
  search_pt.z = goal_pose.z;
  
  if (kdtree_planground_->nearestKSearch(search_pt, 1, goal_idx, goal_dist) == 0) {
    path_length = straightLineDistance(start_pose, goal_pose);
    return false;
  }
  
  A_Star_on_Graph planground_planner(pc_planground_, perception_ros_, a_star_expanding_radius_);
  planground_planner.setupTurningWeight(turning_weight_);

  std::vector<unsigned int> planground_path;
  planground_planner.getPath(static_cast<unsigned int>(start_idx[0]),
                              static_cast<unsigned int>(goal_idx[0]),
                              planground_path);
  
  if (planground_path.empty()) {
    path_length = straightLineDistance(start_pose, goal_pose);
    return false;
  }

  if (path_out) {
    *path_out = planground_path;
  }
  
  path_length = calculatePathLength(planground_path, pc_planground_);
  
  RCLCPP_INFO(perception_ros_->get_logger(),
    "[Hybrid] Planground-only path: %lu nodes, length=%.2f, straight=%.2f, detour_ratio=%.2f",
    planground_path.size(), path_length, 
    straightLineDistance(start_pose, goal_pose),
    path_length / (straightLineDistance(start_pose, goal_pose) + 0.0001));
  
  return true;
}

bool Hybrid_A_Star::findNearestInHybrid(const pcl::PointXYZI& pt, unsigned int& hybrid_idx)
{
  if (pc_hybrid_->empty()) {
    return false;
  }
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  pcl::PointXYZI search_pt;
  search_pt.x = pt.x;
  search_pt.y = pt.y;
  search_pt.z = pt.z;
  search_pt.intensity = 0.0f;
  if (kdtree_hybrid_->nearestKSearch(search_pt, 1, indices, distances) > 0 && indices[0] >= 0) {
    hybrid_idx = static_cast<unsigned int>(indices[0]);
    return true;
  }
  return false;
}

unsigned int Hybrid_A_Star::addStartPoseToHybrid(const pcl::PointXYZI& start_pose)
{
  pcl::PointXYZI start_node = start_pose;
  start_node.intensity = 0.0f;
  pc_hybrid_->push_back(start_node);
  is_from_planground_.push_back(true);
  unsigned int start_idx = static_cast<unsigned int>(pc_hybrid_->size() - 1);
  kdtree_hybrid_->setInputCloud(pc_hybrid_);
  return start_idx;
}

unsigned int Hybrid_A_Star::ensureGoalInHybrid(const pcl::PointXYZI& goal_pose, 
                                                 bool goal_on_ground,
                                                 unsigned int ground_goal_idx)
{
  unsigned int hybrid_goal_idx;
  if (!findNearestInHybrid(goal_pose, hybrid_goal_idx)) {
    return 0;
  }
  pc_hybrid_->points[hybrid_goal_idx].intensity = 0.0f;
  return hybrid_goal_idx;
}

bool Hybrid_A_Star::planOnHybrid(unsigned int hybrid_start_idx, unsigned int hybrid_goal_idx,
                                   std::vector<unsigned int>& path)
{
  path.clear();

  if (pc_hybrid_->size() < 2) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Hybrid cloud too small (%lu) for planning", pc_hybrid_->size());
    return false;
  }

  // Get start and goal positions
  pcl::PointXYZI start_pose = pc_hybrid_->points[hybrid_start_idx];
  pcl::PointXYZI goal_pose = pc_hybrid_->points[hybrid_goal_idx];

  // Compute straight-line distance
  double straight_line_distance = straightLineDistance(start_pose, goal_pose);

  // Plan on planground-only to get reference path length for detour ratio
  double planground_path_length = straight_line_distance;
  planOnPlangroundOnly(start_pose, goal_pose, planground_path_length);
  
  double detour_ratio = planground_path_length / (straight_line_distance + 0.0001);
  
  RCLCPP_INFO(perception_ros_->get_logger(),
    "[Hybrid] Detour analysis: straight=%.2f, planground_path=%.2f, ratio=%.2f, threshold=%.2f",
    straight_line_distance, planground_path_length, detour_ratio, detour_ratio_threshold_);
  
  // CRITICAL FIX: Ensure is_from_planground_ is in sync with pc_hybrid_ before iterating
  if (is_from_planground_.size() != pc_hybrid_->size()) {
    RCLCPP_ERROR(perception_ros_->get_logger(),
      "[Hybrid] CRITICAL: Size mismatch in planOnHybrid: is_from_planground_=%lu, pc_hybrid_=%lu. Rebuilding.",
      is_from_planground_.size(), pc_hybrid_->size());
    is_from_planground_.clear();
    for (size_t i = 0; i < pc_hybrid_->size(); ++i) {
      is_from_planground_.push_back(i < pc_planground_->size());
    }
  }
  
  // Update ground point intensities with actual cost based on detour ratio and edge penalty
  int ground_points_updated = 0;
  int edge_penalty_applied = 0;
  for (size_t i = 0; i < pc_hybrid_->size(); ++i) {
    if (i < is_from_planground_.size() && !is_from_planground_[i]) {
      double dist_to_planground = static_cast<double>(pc_hybrid_->points[i].intensity);
      double ground_cost = calculateGroundCost(dist_to_planground, planground_path_length, straight_line_distance);
      
      // v25: Apply edge penalty to discourage planning near cloud boundaries
      // Points near the edge of the ground cloud (low local density) get additional cost
      double edge_penalty = calculateEdgePenalty(pc_hybrid_->points[i]);
      if (edge_penalty > 0.0) {
        ground_cost += edge_penalty;
        edge_penalty_applied++;
      }
      
      pc_hybrid_->points[i].intensity = static_cast<float>(ground_cost);
      ground_points_updated++;
    }
  }
  
  // v20关键修复: 更新地面点intensity后，必须重建kdtree
  kdtree_hybrid_->setInputCloud(pc_hybrid_);
  
  RCLCPP_INFO(perception_ros_->get_logger(),
    "[Hybrid] Planning on hybrid cloud: %lu points (%d ground updated, %d with edge penalty), "
    "detour_ratio=%.2f, threshold=%.2f",
    pc_hybrid_->size(), ground_points_updated, edge_penalty_applied,
    detour_ratio, detour_ratio_threshold_);

  
  // Check if start pose has enough neighbors
  std::vector<int> start_neighbors;
  std::vector<float> start_neighbor_distances;
  kdtree_hybrid_->radiusSearch(pc_hybrid_->points[hybrid_start_idx], 
                                a_star_expanding_radius_, 
                                start_neighbors, start_neighbor_distances);
  
  start_neighbors.erase(
    std::remove(start_neighbors.begin(), start_neighbors.end(), static_cast<int>(hybrid_start_idx)),
    start_neighbors.end());
  
  unsigned int actual_start_idx = hybrid_start_idx;
  // A start node only needs one valid edge to enter the graph. Treating a
  // single neighbor as isolated discards the real robot pose unnecessarily.
  if (start_neighbors.empty()) {
    // Save goal position before any cloud modification
    pcl::PointXYZI goal_position = pc_hybrid_->points[hybrid_goal_idx];
    
    // If start pose was added (it's the last point), remove it first
    bool start_was_added = false;
    if (hybrid_start_idx == pc_hybrid_->size() - 1) {
      pc_hybrid_->erase(pc_hybrid_->end() - 1);
      is_from_planground_.pop_back();
      start_was_added = true;
      kdtree_hybrid_->setInputCloud(pc_hybrid_);
    }
    
    // CRITICAL FIX: After removing the start point, the hybrid_goal_idx may be invalid
    // because the point cloud size decreased by 1. Always re-find the goal position
    // in the modified cloud to ensure correct index.
    std::vector<int> goal_idx(1);
    std::vector<float> goal_dist(1);
    pcl::PointXYZI goal_search_pt;
    goal_search_pt.x = goal_position.x;
    goal_search_pt.y = goal_position.y;
    goal_search_pt.z = goal_position.z;
    goal_search_pt.intensity = 0.0f;
    if (kdtree_hybrid_->nearestKSearch(goal_search_pt, 1, goal_idx, goal_dist) > 0 && goal_idx[0] >= 0) {
      hybrid_goal_idx = static_cast<unsigned int>(goal_idx[0]);
    } else {
      RCLCPP_WARN(perception_ros_->get_logger(), 
        "[Hybrid] Cannot re-find goal in hybrid cloud after start removal");
      return false;
    }
    
    std::vector<int> nearest_start(1);
    std::vector<float> nearest_start_dist(1);
    pcl::PointXYZI search_pt;
    search_pt.x = start_pose.x;
    search_pt.y = start_pose.y;
    search_pt.z = start_pose.z;
    search_pt.intensity = 0.0f;
    kdtree_hybrid_->nearestKSearch(search_pt, 1, nearest_start, nearest_start_dist);
    
    if (nearest_start[0] >= 0) {
      actual_start_idx = static_cast<unsigned int>(nearest_start[0]);
      pc_hybrid_->points[actual_start_idx].intensity = 0.0f;
      RCLCPP_WARN(perception_ros_->get_logger(), 
        "[Hybrid] Start pose isolated (neighbors=%lu), using nearest cloud point idx=%u (dist=%.3f)",
        start_neighbors.size(), actual_start_idx, std::sqrt(nearest_start_dist[0]));
    } else {
      RCLCPP_WARN(perception_ros_->get_logger(), 
        "[Hybrid] Cannot find any nearest point in hybrid cloud for start pose");
      return false;
    }
  }
  
  // CRITICAL FIX: Validate indices before creating A* planner to prevent segfault
  if (actual_start_idx >= pc_hybrid_->size() || hybrid_goal_idx >= pc_hybrid_->size()) {
    RCLCPP_ERROR(perception_ros_->get_logger(),
      "[Hybrid] CRITICAL: Invalid indices before A* planning: start=%u (cloud=%lu), goal=%u (cloud=%lu)",
      actual_start_idx, pc_hybrid_->size(), hybrid_goal_idx, pc_hybrid_->size());
    return false;
  }
  
  // Plan on the hybrid cloud using A*
  if (a_star_planner_) {
    delete a_star_planner_;
  }
  
  a_star_planner_ = new A_Star_on_Graph(pc_hybrid_, perception_ros_, a_star_expanding_radius_);
  a_star_planner_->setupTurningWeight(turning_weight_);
  
  a_star_planner_->getPath(actual_start_idx, hybrid_goal_idx, path);

  if (path.empty()) {
    std::vector<unsigned int> planground_fallback;
    double fallback_length = straight_line_distance;
    const double start_to_planground = distanceToNearestPlanground(start_pose);
    const double goal_to_planground = distanceToNearestPlanground(goal_pose);
    if (start_to_planground <= 0.5 && goal_to_planground <= 0.5 &&
        planOnPlangroundOnly(
          start_pose, goal_pose, fallback_length, &planground_fallback) &&
        !planground_fallback.empty()) {
      path = planground_fallback;
      if (hybrid_start_idx < pc_hybrid_->size() &&
          path.front() != hybrid_start_idx) {
        path.insert(path.begin(), hybrid_start_idx);
      }
      RCLCPP_WARN(perception_ros_->get_logger(),
        "[Hybrid] Hybrid A* failed; using connected planground fallback: "
        "%lu nodes, start_offset=%.3f, goal_offset=%.3f",
        path.size(), start_to_planground, goal_to_planground);
      return true;
    }
    RCLCPP_WARN(perception_ros_->get_logger(), 
      "[Hybrid] No path found on hybrid cloud (expanding_radius=%.2f)",
      a_star_expanding_radius_);
    return false;
  }
  
  // Analyze the hybrid path
  int ground_nodes = 0;
  for (size_t i = 0; i < path.size(); ++i) {
    if (path[i] < is_from_planground_.size() && !is_from_planground_[path[i]]) {
      ground_nodes++;
    }
  }
  
  RCLCPP_INFO(perception_ros_->get_logger(), 
    "[Hybrid] Path found on hybrid cloud: %lu nodes (%d ground), path length=%.2f",
    path.size(), ground_nodes, calculatePathLength(path, pc_hybrid_));
  
  return true;
}

//=============================================================================
// Public planning functions
//=============================================================================

void Hybrid_A_Star::getPath(unsigned int start, unsigned int goal, std::vector<unsigned int>& path)
{
  path.clear();
  
  if (start >= pc_planground_->size() || goal >= pc_planground_->size()) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Invalid start (%u) or goal (%u) index (planground size=%lu)",
      start, goal, pc_planground_->size());
    return;
  }
  
  pcl::PointXYZI start_pose = pc_planground_->points[start];
  pcl::PointXYZI goal_pose = pc_planground_->points[goal];
  
  // Find corresponding indices in hybrid cloud
  unsigned int hybrid_start_idx, hybrid_goal_idx;
  if (!findNearestInHybrid(start_pose, hybrid_start_idx) ||
      !findNearestInHybrid(goal_pose, hybrid_goal_idx)) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Cannot find start/goal in hybrid cloud");
    return;
  }
  
  // Set start and goal intensities to zero (no cost)
  pc_hybrid_->points[hybrid_start_idx].intensity = 0.0f;
  pc_hybrid_->points[hybrid_goal_idx].intensity = 0.0f;
  
  // Plan on hybrid cloud
  if (!planOnHybrid(hybrid_start_idx, hybrid_goal_idx, path)) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] No path found on hybrid cloud");
    return;
  }
}

void Hybrid_A_Star::getPathWithGroundGoal(unsigned int start, unsigned int ground_goal, 
                                           std::vector<unsigned int>& path)
{
  path.clear();
  
  if (start >= pc_planground_->size()) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Invalid start index (%u) for planground (size=%lu)",
      start, pc_planground_->size());
    return;
  }
  
  if (ground_goal >= pc_ground_->size()) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Invalid ground goal index (%u) for ground (size=%lu)",
      ground_goal, pc_ground_->size());
    return;
  }
  
  pcl::PointXYZI start_pose = pc_planground_->points[start];
  pcl::PointXYZI goal_pose = pc_ground_->points[ground_goal];
  
  // Find start in hybrid cloud
  unsigned int hybrid_start_idx;
  if (!findNearestInHybrid(start_pose, hybrid_start_idx)) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Cannot find start in hybrid cloud");
    return;
  }
  
  // Ensure goal is in hybrid cloud with zero cost
  unsigned int hybrid_goal_idx = ensureGoalInHybrid(goal_pose, true, ground_goal);
  if (hybrid_goal_idx == 0) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Cannot find goal in hybrid cloud");
    return;
  }
  
  // Set start intensity to zero
  pc_hybrid_->points[hybrid_start_idx].intensity = 0.0f;
  
  // Plan on hybrid cloud
  if (!planOnHybrid(hybrid_start_idx, hybrid_goal_idx, path)) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] No path found on hybrid cloud (ground goal)");
    return;
  }
}

void Hybrid_A_Star::getPathWithGroundStartAndGoal(unsigned int ground_start, unsigned int ground_goal,
                                                    std::vector<unsigned int>& path)
{
  path.clear();
  
  if (ground_start >= pc_ground_->size() || ground_goal >= pc_ground_->size()) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Invalid ground start (%u) or goal (%u) index (ground size=%lu)",
      ground_start, ground_goal, pc_ground_->size());
    return;
  }
  
  pcl::PointXYZI start_pose = pc_ground_->points[ground_start];
  pcl::PointXYZI goal_pose = pc_ground_->points[ground_goal];
  
  // Find start and goal in hybrid cloud
  unsigned int hybrid_start_idx, hybrid_goal_idx;
  if (!findNearestInHybrid(start_pose, hybrid_start_idx) ||
      !findNearestInHybrid(goal_pose, hybrid_goal_idx)) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Cannot find start/goal in hybrid cloud");
    return;
  }
  
  // Set start and goal intensities to zero
  pc_hybrid_->points[hybrid_start_idx].intensity = 0.0f;
  pc_hybrid_->points[hybrid_goal_idx].intensity = 0.0f;
  
  // Plan on hybrid cloud
  if (!planOnHybrid(hybrid_start_idx, hybrid_goal_idx, path)) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] No path found on hybrid cloud (ground start and goal)");
    return;
  }
}

void Hybrid_A_Star::getPathWithStartPose(const geometry_msgs::msg::PoseStamped& start_pose,
                                           unsigned int goal,
                                           std::vector<unsigned int>& path)
{
  path.clear();

  if (goal >= pc_planground_->size()) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Invalid goal index (%u) for planground (size=%lu)",
      goal, pc_planground_->size());
    return;
  }

  // Convert start pose to point
  pcl::PointXYZI start_pt;
  start_pt.x = start_pose.pose.position.x;
  start_pt.y = start_pose.pose.position.y;
  start_pt.z = start_pose.pose.position.z;
  start_pt.intensity = 0.0f;

  // Get goal pose from planground
  pcl::PointXYZI goal_pt = pc_planground_->points[goal];

  // Add start pose to hybrid cloud
  unsigned int hybrid_start_idx = addStartPoseToHybrid(start_pt);

  // Find goal in hybrid cloud
  unsigned int hybrid_goal_idx;
  if (!findNearestInHybrid(goal_pt, hybrid_goal_idx)) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Cannot find goal in hybrid cloud");
    return;
  }

  // Set goal intensity to zero
  pc_hybrid_->points[hybrid_goal_idx].intensity = 0.0f;

  // Plan on hybrid cloud
  if (!planOnHybrid(hybrid_start_idx, hybrid_goal_idx, path)) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] No path found on hybrid cloud (start pose)");
    return;
  }
}

void Hybrid_A_Star::getPathWithStartPoseAndGroundGoal(const geometry_msgs::msg::PoseStamped& start_pose,
                                                        unsigned int ground_goal,
                                                        std::vector<unsigned int>& path)
{
  path.clear();
  
  if (ground_goal >= pc_ground_->size()) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Invalid ground goal index (%u) for ground (size=%lu)",
      ground_goal, pc_ground_->size());
    return;
  }
  
  // Convert start pose to point
  pcl::PointXYZI start_pt;
  start_pt.x = start_pose.pose.position.x;
  start_pt.y = start_pose.pose.position.y;
  start_pt.z = start_pose.pose.position.z;
  start_pt.intensity = 0.0f;
  
  // Get goal pose from ground
  pcl::PointXYZI goal_pt = pc_ground_->points[ground_goal];
  
  // Add start pose to hybrid cloud
  unsigned int hybrid_start_idx = addStartPoseToHybrid(start_pt);
  
  // Ensure goal is in hybrid cloud with zero cost
  unsigned int hybrid_goal_idx = ensureGoalInHybrid(goal_pt, true, ground_goal);
  if (hybrid_goal_idx == 0) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] Cannot find goal in hybrid cloud");
    return;
  }
  
  // Plan on hybrid cloud
  if (!planOnHybrid(hybrid_start_idx, hybrid_goal_idx, path)) {
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[Hybrid] No path found on hybrid cloud (start pose + ground goal)");
    return;
  }
}

void Hybrid_A_Star::simplifyPath(const std::vector<unsigned int>& path_indices,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                  std::vector<unsigned int>& simplified_indices)
{
  simplified_indices.clear();
  
  if (path_indices.size() < 3) {
    simplified_indices = path_indices;
    return;
  }
  
  // Line-of-sight shortcutting: remove intermediate nodes that can be shortcut
  simplified_indices.push_back(path_indices[0]);
  
  size_t current = 0;
  for (size_t i = 1; i < path_indices.size() - 1; i++) {
    // Try to shortcut from current to i+1
    const pcl::PointXYZI& p1 = cloud->points[path_indices[current]];
    const pcl::PointXYZI& p2 = cloud->points[path_indices[i + 1]];
    
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    
    // Check if the straight line between p1 and p2 stays close to the original path
    // Sample points along the straight line and check distance to original path
    bool can_shortcut = true;
    int num_samples = std::max(10, static_cast<int>(dist / 0.2));
    
    for (int s = 1; s < num_samples; s++) {
      double ratio = static_cast<double>(s) / num_samples;
      
      // Point on the straight line
      double sx = p1.x + dx * ratio;
      double sy = p1.y + dy * ratio;
      
      // Find the closest point on the original path segment (current to i+1)
      double min_path_dist = std::numeric_limits<double>::max();
      for (size_t j = current; j <= i + 1 && j < path_indices.size(); j++) {
        double px = cloud->points[path_indices[j]].x;
        double py = cloud->points[path_indices[j]].y;
        double d = std::sqrt((sx - px) * (sx - px) + (sy - py) * (sy - py));
        min_path_dist = std::min(min_path_dist, d);
      }
      
      if (min_path_dist > 0.15) {
        can_shortcut = false;
        break;
      }
    }
    
    if (!can_shortcut) {
      // Cannot shortcut, keep this node
      simplified_indices.push_back(path_indices[i]);
      current = i;
    }
    // else: skip this node (shortcut it)
  }
  
  // Always add the last node
  if (simplified_indices.back() != path_indices.back()) {
    simplified_indices.push_back(path_indices.back());
  }
  
  RCLCPP_DEBUG(perception_ros_->get_logger(),
    "[Hybrid] Path simplification: %lu -> %lu nodes",
    path_indices.size(), simplified_indices.size());
}

/**
 * @brief Detect sharp corners in the path and insert additional control points
 *        to ensure the smoothed path follows the original path more closely.
 * 
 * This function identifies sharp turns (angle < threshold) and inserts extra
 * control points near the corner to prevent the spline from cutting the corner too much.
 * 
 * @param path_indices The simplified path indices
 * @param cloud The point cloud
 * @param corner_enhanced_indices Output: path indices with corner enhancement
 * @param angle_threshold_deg Angle threshold in degrees below which a corner is considered "sharp"
 */
void Hybrid_A_Star::enhanceCorners(const std::vector<unsigned int>& path_indices,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                    std::vector<unsigned int>& corner_enhanced_indices,
                                    double angle_threshold_deg)
{
  corner_enhanced_indices.clear();
  
  if (path_indices.size() < 3) {
    corner_enhanced_indices = path_indices;
    return;
  }
  
  double angle_threshold_rad = angle_threshold_deg * M_PI / 180.0;
  
  // Always add the first point
  corner_enhanced_indices.push_back(path_indices[0]);
  
  for (size_t i = 1; i < path_indices.size() - 1; i++) {
    const pcl::PointXYZI& prev = cloud->points[path_indices[i - 1]];
    const pcl::PointXYZI& curr = cloud->points[path_indices[i]];
    const pcl::PointXYZI& next = cloud->points[path_indices[i + 1]];
    
    // Compute vectors
    double v1x = curr.x - prev.x;
    double v1y = curr.y - prev.y;
    double v2x = next.x - curr.x;
    double v2y = next.y - curr.y;
    
    double len1 = std::sqrt(v1x * v1x + v1y * v1y);
    double len2 = std::sqrt(v2x * v2x + v2y * v2y);
    
    if (len1 < 0.001 || len2 < 0.001) {
      corner_enhanced_indices.push_back(path_indices[i]);
      continue;
    }
    
    // Normalize
    v1x /= len1; v1y /= len1;
    v2x /= len2; v2y /= len2;
    
    // Compute angle between vectors (dot product)
    double dot = v1x * v2x + v1y * v2y;
    dot = std::max(-1.0, std::min(1.0, dot));
    double angle = std::acos(dot);
    
    if (angle < angle_threshold_rad) {
      // Sharp corner detected - always insert the original point to preserve the corner
      corner_enhanced_indices.push_back(path_indices[i]);
    } else {
      // Check distance from prev to next
      double dx = next.x - prev.x;
      double dy = next.y - prev.y;
      double dist_skip = std::sqrt(dx * dx + dy * dy);
      
      if (dist_skip > 2.0) {
        corner_enhanced_indices.push_back(path_indices[i]);
      }
      // else: skip this point for smoother path
    }
  }
  
  // Always add the last point
  corner_enhanced_indices.push_back(path_indices.back());
  
  RCLCPP_DEBUG(perception_ros_->get_logger(),
    "[Hybrid] Corner enhancement: %lu -> %lu nodes (angle_threshold=%.1f deg)",
    path_indices.size(), corner_enhanced_indices.size(), angle_threshold_deg);
}


void Hybrid_A_Star::smoothPath(const std::vector<unsigned int>& raw_path_indices,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                std::vector<std::tuple<double, double, double>>& smoothed_points,
                                double smoothing_resolution)
{
  smoothed_points.clear();
  
  if (raw_path_indices.size() < 2) {
    return;
  }
  
  // Step 1: Simplify the path first
  std::vector<unsigned int> simplified_indices;
  simplifyPath(raw_path_indices, cloud, simplified_indices);

  if (simplified_indices.size() < 2) {
    return;
  }

  // Step 2: Extract control points from the simplified path
  std::vector<double> ctrl_x, ctrl_y, ctrl_z;
  for (size_t i = 0; i < simplified_indices.size(); i++) {
    ctrl_x.push_back(cloud->points[simplified_indices[i]].x);
    ctrl_y.push_back(cloud->points[simplified_indices[i]].y);
    ctrl_z.push_back(cloud->points[simplified_indices[i]].z);
  }

  // Step 2.5: Insert auxiliary control points near sharp corners.
  // At a corner with turning angle larger than corner_angle_threshold (e.g. 30 deg),
  // we insert two extra points: one between (prev, corner) and one between (corner, next),
  // each at offset corner_anchor_distance along the incoming/outgoing edge.
  // This anchors the spline tightly to the corner so it cannot bow outward.
  {
    const double corner_angle_threshold_rad = 30.0 * M_PI / 180.0;
    const double corner_anchor_distance = 0.25;
    std::vector<double> new_x, new_y, new_z;
    new_x.reserve(ctrl_x.size() * 3);
    new_y.reserve(ctrl_y.size() * 3);
    new_z.reserve(ctrl_z.size() * 3);
    new_x.push_back(ctrl_x.front());
    new_y.push_back(ctrl_y.front());
    new_z.push_back(ctrl_z.front());
    for (size_t i = 1; i + 1 < ctrl_x.size(); i++) {
      double v1x = ctrl_x[i] - ctrl_x[i - 1];
      double v1y = ctrl_y[i] - ctrl_y[i - 1];
      double v2x = ctrl_x[i + 1] - ctrl_x[i];
      double v2y = ctrl_y[i + 1] - ctrl_y[i];
      double l1 = std::sqrt(v1x * v1x + v1y * v1y);
      double l2 = std::sqrt(v2x * v2x + v2y * v2y);
      bool inserted_pre = false;
      if (l1 > 1e-3 && l2 > 1e-3) {
        double dot = (v1x * v2x + v1y * v2y) / (l1 * l2);
        dot = std::max(-1.0, std::min(1.0, dot));
        double turning_angle = std::acos(dot); // 0 = straight, pi = U-turn
        if (turning_angle > corner_angle_threshold_rad) {
          double d_pre = std::min(corner_anchor_distance, l1 * 0.4);
          double d_post = std::min(corner_anchor_distance, l2 * 0.4);
          double t_pre = 1.0 - d_pre / l1;
          new_x.push_back(ctrl_x[i - 1] + v1x * t_pre);
          new_y.push_back(ctrl_y[i - 1] + v1y * t_pre);
          new_z.push_back(ctrl_z[i - 1] + (ctrl_z[i] - ctrl_z[i - 1]) * t_pre);
          new_x.push_back(ctrl_x[i]);
          new_y.push_back(ctrl_y[i]);
          new_z.push_back(ctrl_z[i]);
          double t_post = d_post / l2;
          new_x.push_back(ctrl_x[i] + v2x * t_post);
          new_y.push_back(ctrl_y[i] + v2y * t_post);
          new_z.push_back(ctrl_z[i] + (ctrl_z[i + 1] - ctrl_z[i]) * t_post);
          inserted_pre = true;
        }
      }
      if (!inserted_pre) {
        new_x.push_back(ctrl_x[i]);
        new_y.push_back(ctrl_y[i]);
        new_z.push_back(ctrl_z[i]);
      }
    }
    new_x.push_back(ctrl_x.back());
    new_y.push_back(ctrl_y.back());
    new_z.push_back(ctrl_z.back());
    ctrl_x.swap(new_x);
    ctrl_y.swap(new_y);
    ctrl_z.swap(new_z);
  }
  
  // Step 3: Apply Catmull-Rom spline interpolation for smooth path
  // Catmull-Rom is a cubic spline that passes through all control points
  // and produces a smooth, continuous curve.
  //
  // For each segment between control points i and i+1, we use 4 control points:
  // P0 = i-1, P1 = i, P2 = i+1, P3 = i+2
  // The spline passes through P1 and P2.
  //
  // For the first and last segments, we duplicate the endpoints.
  //
  // Z is interpolated using the same Catmull-Rom spline to ensure smooth
  // height transitions along the path.
  
  size_t n = ctrl_x.size();
  if (n == 2) {
    // Just two points: return a straight line
    double dx = ctrl_x[1] - ctrl_x[0];
    double dy = ctrl_y[1] - ctrl_y[0];
    double dz = ctrl_z[1] - ctrl_z[0];
    double total_dist = std::sqrt(dx * dx + dy * dy);
    int num_steps = std::max(2, static_cast<int>(total_dist / smoothing_resolution));
    
    for (int i = 0; i <= num_steps; i++) {
      double t = static_cast<double>(i) / num_steps;
      smoothed_points.push_back(std::make_tuple(
        ctrl_x[0] + dx * t,
        ctrl_y[0] + dy * t,
        ctrl_z[0] + dz * t
      ));
    }
    return;
  }
  
  // For Catmull-Rom, we need at least 3 control points

  // Calculate total path length for sampling
  double total_length = 0.0;
  for (size_t i = 1; i < n; i++) {
    double dx = ctrl_x[i] - ctrl_x[i-1];
    double dy = ctrl_y[i] - ctrl_y[i-1];
    total_length += std::sqrt(dx * dx + dy * dy);
  }

  // Number of samples based on total path length and resolution
  int total_samples = std::max(static_cast<int>(total_length / smoothing_resolution), static_cast<int>(n * 3));

  // Centripetal Catmull-Rom spline (alpha=0.5, Barry-Goldman form).
  // Unlike uniform Catmull-Rom, centripetal parameterization is guaranteed
  // NOT to form cusps or self-intersections at sharp corners.

  // Cumulative chord lengths for global parameterization (sampling)
  std::vector<double> chord_lengths(n, 0.0);
  for (size_t i = 1; i < n; i++) {
    double dx = ctrl_x[i] - ctrl_x[i-1];
    double dy = ctrl_y[i] - ctrl_y[i-1];
    chord_lengths[i] = chord_lengths[i-1] + std::sqrt(dx * dx + dy * dy);
  }
  double total_chord = chord_lengths[n-1];
  if (total_chord < 0.001) {
    smoothed_points.push_back(std::make_tuple(ctrl_x[0], ctrl_y[0], ctrl_z[0]));
    return;
  }

  // Sample along the spline uniformly by arc-length
  for (int s = 0; s <= total_samples; s++) {
    double t_global = static_cast<double>(s) / total_samples;
    double target_chord = t_global * total_chord;

    // Find segment
    size_t seg = 0;
    for (size_t i = 1; i < n; i++) {
      if (chord_lengths[i] >= target_chord) {
        seg = i - 1;
        break;
      }
    }
    if (seg >= n - 1) seg = n - 2;

    // Local parameter within segment [0, 1]
    double seg_length = chord_lengths[seg + 1] - chord_lengths[seg];
    double local_t = (seg_length > 0.001) ?
                     (target_chord - chord_lengths[seg]) / seg_length : 0.0;
    local_t = std::max(0.0, std::min(1.0, local_t));

    // Get 4 control points: P0, P1, P2, P3
    double p0x, p0y, p0z, p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z;
    p1x = ctrl_x[seg]; p1y = ctrl_y[seg]; p1z = ctrl_z[seg];
    p2x = ctrl_x[seg+1]; p2y = ctrl_y[seg+1]; p2z = ctrl_z[seg+1];
    if (seg == 0) {
      p0x = 2.0*p1x - p2x; p0y = 2.0*p1y - p2y; p0z = 2.0*p1z - p2z;
    } else {
      p0x = ctrl_x[seg-1]; p0y = ctrl_y[seg-1]; p0z = ctrl_z[seg-1];
    }
    if (seg + 2 >= n) {
      p3x = 2.0*p2x - p1x; p3y = 2.0*p2y - p1y; p3z = 2.0*p2z - p1z;
    } else {
      p3x = ctrl_x[seg+2]; p3y = ctrl_y[seg+2]; p3z = ctrl_z[seg+2];
    }

    // Centripetal knot intervals: dt_i = |P_{i+1} - P_i|^alpha, alpha=0.5
    auto dist2d = [](double ax, double ay, double az, double bx, double by, double bz) {
      double dx = bx - ax, dy = by - ay, dz = bz - az;
      return std::sqrt(dx*dx + dy*dy + dz*dz);
    };
    double d01 = std::pow(dist2d(p0x,p0y,p0z, p1x,p1y,p1z), 0.5);
    double d12 = std::pow(dist2d(p1x,p1y,p1z, p2x,p2y,p2z), 0.5);
    double d23 = std::pow(dist2d(p2x,p2y,p2z, p3x,p3y,p3z), 0.5);
    if (d01 < 1e-6) d01 = 1.0;
    if (d12 < 1e-6) d12 = 1.0;
    if (d23 < 1e-6) d23 = 1.0;

    // Knot values: t0=0, t1=d01, t2=d01+d12, t3=d01+d12+d23
    double t0 = 0.0;
    double t1 = d01;
    double t2 = t1 + d12;
    double t3 = t2 + d23;

    // Map local_t [0,1] to knot space [t1, t2]
    double tt = t1 + local_t * (t2 - t1);

    // Barry-Goldman pyramid (De Boor-like for Catmull-Rom)
    auto lerp = [](double a, double b, double ta, double tb, double t) {
      double f = (t - ta) / (tb - ta);
      return a + f * (b - a);
    };

    // Level 1
    double A1x = lerp(p0x, p1x, t0, t1, tt);
    double A1y = lerp(p0y, p1y, t0, t1, tt);
    double A1z = lerp(p0z, p1z, t0, t1, tt);
    double A2x = lerp(p1x, p2x, t1, t2, tt);
    double A2y = lerp(p1y, p2y, t1, t2, tt);
    double A2z = lerp(p1z, p2z, t1, t2, tt);
    double A3x = lerp(p2x, p3x, t2, t3, tt);
    double A3y = lerp(p2y, p3y, t2, t3, tt);
    double A3z = lerp(p2z, p3z, t2, t3, tt);

    // Level 2
    double B1x = lerp(A1x, A2x, t0, t2, tt);
    double B1y = lerp(A1y, A2y, t0, t2, tt);
    double B1z = lerp(A1z, A2z, t0, t2, tt);
    double B2x = lerp(A2x, A3x, t1, t3, tt);
    double B2y = lerp(A2y, A3y, t1, t3, tt);
    double B2z = lerp(A2z, A3z, t1, t3, tt);

    // Level 3 (final point)
    double Cx = lerp(B1x, B2x, t1, t2, tt);
    double Cy = lerp(B1y, B2y, t1, t2, tt);
    double Cz = lerp(B1z, B2z, t1, t2, tt);

    smoothed_points.push_back(std::make_tuple(Cx, Cy, Cz));
  }

  RCLCPP_DEBUG(perception_ros_->get_logger(),
    "[Hybrid] Path smoothing (centripetal): %lu control points -> %lu smoothed points (resolution=%.2f)",
    simplified_indices.size(), smoothed_points.size(), smoothing_resolution);
}

double Hybrid_A_Star::estimatePlangroundPathLength(const geometry_msgs::msg::PoseStamped& start_pose,
                                                     const geometry_msgs::msg::PoseStamped& goal_pose)
{
  pcl::PointXYZI start_pt, goal_pt;
  start_pt.x = start_pose.pose.position.x;
  start_pt.y = start_pose.pose.position.y;
  start_pt.z = start_pose.pose.position.z;
  start_pt.intensity = 0.0f;
  
  goal_pt.x = goal_pose.pose.position.x;
  goal_pt.y = goal_pose.pose.position.y;
  goal_pt.z = goal_pose.pose.position.z;
  goal_pt.intensity = 0.0f;
  
  double path_length;
  planOnPlangroundOnly(start_pt, goal_pt, path_length);
  return path_length;
}
void Hybrid_A_Star::smoothPathToRosPath(const std::vector<unsigned int>& raw_path_indices,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                         nav_msgs::msg::Path& ros_path,
                                         const geometry_msgs::msg::PoseStamped& goal_pose,
                                         const std::string& frame_id,
                                         double smoothing_resolution)
{
  // First, get the smoothed 3D points (x, y, z) from Catmull-Rom spline
  std::vector<std::tuple<double, double, double>> smoothed_points;
  smoothPath(raw_path_indices, cloud, smoothed_points, smoothing_resolution);
  
  ros_path.header.frame_id = frame_id;
  ros_path.header.stamp = rclcpp::Time(0); // placeholder, will be set by caller
  
  if (!smoothed_points.empty()) {
    // Use smoothed path with Catmull-Rom interpolated Z values
    for (size_t it = 0; it < smoothed_points.size(); it++) {
      geometry_msgs::msg::PoseStamped pst;
      pst.header = ros_path.header;
      pst.pose.position.x = std::get<0>(smoothed_points[it]);
      pst.pose.position.y = std::get<1>(smoothed_points[it]);
      pst.pose.position.z = std::get<2>(smoothed_points[it]);
      
      if (it < smoothed_points.size() - 1) {
        double vx = std::get<0>(smoothed_points[it+1]) - std::get<0>(smoothed_points[it]);
        double vy = std::get<1>(smoothed_points[it+1]) - std::get<1>(smoothed_points[it]);
        double yaw = atan2(vy, vx);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pst.pose.orientation.x = q.getX();
        pst.pose.orientation.y = q.getY();
        pst.pose.orientation.z = q.getZ();
        pst.pose.orientation.w = q.getW();
      }
      ros_path.poses.push_back(pst);
    }
  } else {
    // Fallback to raw path
    for (size_t it = 0; it < raw_path_indices.size(); it++) {
      geometry_msgs::msg::PoseStamped pst;
      pst.header = ros_path.header;
      pst.pose.position.x = cloud->points[raw_path_indices[it]].x;
      pst.pose.position.y = cloud->points[raw_path_indices[it]].y;
      pst.pose.position.z = cloud->points[raw_path_indices[it]].z;
      
      if (it < raw_path_indices.size() - 1) {
        double vx = cloud->points[raw_path_indices[it+1]].x - cloud->points[raw_path_indices[it]].x;
        double vy = cloud->points[raw_path_indices[it+1]].y - cloud->points[raw_path_indices[it]].y;
        double yaw = atan2(vy, vx);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pst.pose.orientation.x = q.getX();
        pst.pose.orientation.y = q.getY();
        pst.pose.orientation.z = q.getZ();
        pst.pose.orientation.w = q.getW();
      }
      ros_path.poses.push_back(pst);
    }
  }

  // A* searches on a point cloud, so its last node is the nearest traversable
  // cloud sample rather than necessarily the exact requested waypoint.  The
  // completion monitor, however, measures against the requested waypoint.
  // Restore a short, bounded goal connector so SCAN and the monitor use the
  // same XY endpoint instead of stopping on opposite sides of their tolerance.
  if (!ros_path.poses.empty()) {
    auto &path_goal = ros_path.poses.back();
    const double goal_dx = goal_pose.pose.position.x - path_goal.pose.position.x;
    const double goal_dy = goal_pose.pose.position.y - path_goal.pose.position.y;
    const double goal_offset = std::hypot(goal_dx, goal_dy);
    constexpr double kMaxExactGoalConnector = 0.25;

    if (goal_offset <= kMaxExactGoalConnector) {
      path_goal.pose.position = goal_pose.pose.position;
      if (ros_path.poses.size() >= 2) {
        auto &previous = ros_path.poses[ros_path.poses.size() - 2];
        const double segment_yaw = std::atan2(
            path_goal.pose.position.y - previous.pose.position.y,
            path_goal.pose.position.x - previous.pose.position.x);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, segment_yaw);
        previous.pose.orientation.x = q.getX();
        previous.pose.orientation.y = q.getY();
        previous.pose.orientation.z = q.getZ();
        previous.pose.orientation.w = q.getW();
        path_goal.pose.orientation = previous.pose.orientation;
      }
      RCLCPP_INFO(
          perception_ros_->get_logger(),
          "[Hybrid] Restored exact requested goal at path end (cloud snap offset=%.3fm).",
          goal_offset);
    } else {
      RCLCPP_WARN(
          perception_ros_->get_logger(),
          "[Hybrid] Requested goal is %.3fm from the planned cloud endpoint; keep the validated endpoint.",
          goal_offset);
    }
  }
}
