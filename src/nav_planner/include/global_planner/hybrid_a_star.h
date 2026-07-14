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

#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include <global_planner/a_star_on_pc.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/msg/path.hpp>

/**
 * @brief Hybrid A* planner that fuses planground and ground point clouds into a single map.
 * 
 * Core Design (v23 - Ultra-Strong Planground Preference with Maximum Anti-Detour):
 * 
 * The planner creates a single unified hybrid point cloud by fusing ALL planground points
 * with ALL ground points. Each point's intensity encodes its traversal cost:
 * 
 * - Planground points: intensity = 0.0 (no extra cost, preferred surface)
 * - Ground points: intensity = f(dist_to_planground, detour_ratio, straight_line_distance)
 *   where f is a cost function that considers:
 *   1. Distance from planground (closer = cheaper)
 *   2. Planground path detour ratio (more detour = cheaper ground)
 *   3. Straight-line distance (short trips = cheaper ground for shortcuts)
 * 
 * Key design principles:
 * 1. Planning主体在planground上 -> planground点代价为0，地面点有显著正代价
 * 2. 绕路比平衡 -> 当planground绕路时地面点代价降低，允许抄近路
 * 3. 距离平衡 -> 短距离允许抄近路，长距离避免绕远路
 * 4. 地面点用于抄近路，但避免绕远路
 * 
 * v23改进:
 * - planground_bias_从5.0提高到10.0，大幅提高地面点基础代价
 * - min_ground_cost_从50.0提高到100.0，确保地面点始终比planground点贵很多
 * - max_ground_bridge_length_从0.2降低到0.15，更严格限制地面路径长度
 * - detour_ratio_threshold_从3.0提高到4.0，只有当planground绕路超过4.0倍时才允许走地面
 * - distance_balance_threshold_从2.0降低到1.5，更多路径被视为"长距离"避免绕路
 * - max_ground_cost_从100.0提高到200.0，提高上限以允许更高的最小代价
 * - ground_path_length_penalty_从2.0提高到5.0，地面路径越长代价越高
 * - 新增detour_balance_factor_lower_bound_=0.5，即使planground严重绕路地面代价也不降太低
 * - 新增detour_balance_factor_upper_bound_=3.0，planground路径很直时地面代价大幅升高
 */
class Hybrid_A_Star {
public:
  /**
   * @brief Constructor
   * @param pc_planground The planground point cloud (primary planning surface)
   * @param pc_ground The ground point cloud (secondary/fallback surface)
   * @param perception_ros Perception3D ROS interface
   * @param a_star_expanding_radius Radius for neighbor search during A* expansion
   */
  Hybrid_A_Star(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_planground,
                pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground,
                std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros,
                double a_star_expanding_radius);
  
  ~Hybrid_A_Star();

  /**
   * @brief Set the turning weight penalty
   */
  void setupTurningWeight(double weight) { turning_weight_ = weight; }

  /**
   * @brief Set the planground bias weight (higher = stronger preference for planground)
   * 
   * This is the base cost multiplier for ground points. The actual cost for a ground point is:
   *   ground_cost = planground_bias_ * (1.0 + (dist_to_planground / max_ground_bridge_length_)^2)
   *                * detour_balance_factor
   * 
   * Recommended values:
   * - 5.0: Strong preference (ground only for significant shortcuts)
   * - 10.0: Very strong preference (ground only as last resort)
   * - 20.0: Extreme preference (ground almost never used)
   */
  void setupPlangroundBias(double bias) { planground_bias_ = bias; }

  /**
   * @brief Set the maximum allowed ground bridge length
   * Ground points farther than this from planground will have very high cost
   */
  void setMaxGroundBridgeLength(double length) { max_ground_bridge_length_ = length; }

  /**
   * @brief Set the voxel leaf size for downsampling the hybrid cloud
   * @param leaf_size The voxel grid leaf size in meters
   */
  void setDownsampleLeafSize(double leaf_size) { hybrid_downsample_leaf_size_ = leaf_size; }

  /**
   * @brief Set the detour ratio threshold for path-length-balanced cost
   * 
   * When the planground path length / straight-line distance ratio exceeds this threshold,
   * the ground cost starts to decrease, allowing the planner to use ground shortcuts
   * to cut off large detours.
   * 
   * For example, with threshold=4.0:
   * - Planground path is 2.0x straight line: ground is expensive, stay on planground
   * - Planground path is 5.0x straight line: ground becomes cheaper, use ground to cut detour
   * 
   * @param ratio Detour ratio threshold
   */
  void setDetourRatioThreshold(double ratio) { detour_ratio_threshold_ = ratio; }

  /**
   * @brief Set the ground search radius (kept for backward compatibility)
   * @param radius Search radius in meters
   */
  void setGroundSearchRadius(double radius) { (void)radius; /* No longer used in v16 unified map approach */ }

  /**
   * @brief Set the distance balance threshold
   * 
   * This threshold controls the trade-off between using ground shortcuts and avoiding detours:
   * - Short distances (straight_line_distance < threshold): ground cost is reduced
   *   allowing more aggressive shortcutting since short ground paths won't cause large detours
   * - Long distances (straight_line_distance >= threshold): ground cost is increased
   *   discouraging long ground paths that would cause significant detours
   * 
   * @param threshold Distance threshold in meters (recommended: 1.5-3.0)
   */
  void setDistanceBalanceThreshold(double threshold) { distance_balance_threshold_ = threshold; }

  /**
   * @brief Set the detour ratio (alias for setDetourRatioThreshold, kept for backward compatibility)
   * @param ratio Detour ratio threshold
   */
  void setDetourRatio(double ratio) { detour_ratio_threshold_ = ratio; }

  /**
   * @brief Set the maximum ground cost
   * 
   * This caps the maximum cost for ground points to prevent path finding failure.
   * 
   * @param cost Maximum ground cost (recommended: 100.0-200.0)
   */
  void setMaxGroundCost(double cost) { max_ground_cost_ = cost; }

  /**
   * @brief Set the minimum ground cost
   * 
   * This sets the minimum cost for ground points to prevent them from being too cheap.
   * 
   * @param cost Minimum ground cost (recommended: 50.0-100.0)
   */
  void setMinGroundCost(double cost) { min_ground_cost_ = cost; }

  /**
   * @brief Set the ground path length penalty coefficient
   * 
   * Higher values penalize longer ground paths more heavily.
   * 
   * @param penalty Penalty coefficient (recommended: 2.0-5.0)
   */
  void setGroundPathLengthPenalty(double penalty) { ground_path_length_penalty_ = penalty; }

  /**
   * @brief Set the detour balance factor lower bound
   * 
   * Even when planground has severe detours, ground cost won't drop below this factor.
   * 
   * @param bound Lower bound (recommended: 0.3-0.5)
   */
  void setDetourBalanceFactorLowerBound(double bound) { detour_balance_factor_lower_bound_ = bound; }

  /**
   * @brief Set the detour balance factor upper bound
   * 
   * When planground path is very straight, ground cost is multiplied by this factor.
   * 
   * @param bound Upper bound (recommended: 2.0-3.0)
   */
  void setDetourBalanceFactorUpperBound(double bound) { detour_balance_factor_upper_bound_ = bound; }

  /**
   * @brief Set the edge penalty parameters for ground points
   * 
   * Ground points near the edge of the point cloud (low local density) will have
   * additional cost to discourage the planner from planning near cloud edges.
   * This helps keep the planned path away from unsafe areas where the ground
   * point cloud is sparse or at its boundary.
   * 
   * @param radius Search radius for counting neighbors (meters)
   * @param weight Maximum edge penalty weight
   * @param falloff_rate Rate at which penalty decreases with density (higher = sharper falloff)
   */
  void setEdgePenaltyParams(double radius, double weight, double falloff_rate) {
    edge_penalty_radius_ = radius;
    edge_penalty_weight_ = weight;
    edge_penalty_falloff_rate_ = falloff_rate;
  }

  /**
   * @brief Update the planground point cloud and rebuild the hybrid map
   */

  void updatePlanground(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_planground);

  /**
   * @brief Update the ground point cloud and rebuild the hybrid map
   */
  void updateGround(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground);

  /**
   * @brief Rebuild the hybrid cloud from the current planground and ground clouds
   * 
   * This should be called after updating either cloud to regenerate the unified map.
   */
  void rebuildHybridCloud();

  /**
   * @brief Main planning function
   * 
   * Plans on the unified hybrid cloud where:
   * - Planground points have intensity=0 (no extra cost)
   * - Ground points have intensity based on distance to planground and path length balance
   * 
   * @param start Start node index in the planground cloud
   * @param goal Goal node index in the planground cloud
   * @param path Output path as node indices (indices into the hybrid cloud)
   */
  void getPath(unsigned int start, unsigned int goal, std::vector<unsigned int>& path);

  /**
   * @brief Plan with start on planground and goal on ground cloud
   * 
   * @param start Start node index in the planground cloud
   * @param ground_goal Goal node index in the ground cloud
   * @param path Output path as node indices (indices into the hybrid cloud)
   */
  void getPathWithGroundGoal(unsigned int start, unsigned int ground_goal, std::vector<unsigned int>& path);

  /**
   * @brief Plan with both start and goal on ground cloud
   * 
   * When the robot's position is not on the planground (e.g., after falling off the edge),
   * this method allows planning entirely on the ground cloud.
   * 
   * @param ground_start Start node index in the ground cloud
   * @param ground_goal Goal node index in the ground cloud
   * @param path Output path as node indices (indices into the hybrid cloud)
   */
  void getPathWithGroundStartAndGoal(unsigned int ground_start, unsigned int ground_goal,
                                      std::vector<unsigned int>& path);

  /**
   * @brief Plan with start pose (robot current position) and goal on planground
   * 
   * @param start_pose Robot's current position (from tf)
   * @param goal Goal node index in the planground cloud
   * @param path Output path as node indices (indices into the hybrid cloud)
   */
  void getPathWithStartPose(const geometry_msgs::msg::PoseStamped& start_pose,
                             unsigned int goal,
                             std::vector<unsigned int>& path);

  /**
   * @brief Plan with start pose (robot current position) and goal on ground cloud
   * 
   * @param start_pose Robot's current position (from tf)
   * @param ground_goal Goal node index in the ground cloud
   * @param path Output path as node indices (indices into the hybrid cloud)
   */
  void getPathWithStartPoseAndGroundGoal(const geometry_msgs::msg::PoseStamped& start_pose,
                                          unsigned int ground_goal,
                                          std::vector<unsigned int>& path);

  /**
   * @brief Get the hybrid point cloud (planground + ground fused)
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr getHybridCloud() { return pc_hybrid_; }

  /**
   * @brief Get mapping from hybrid cloud index to original cloud type
   * @return Vector where each element indicates if the point is from planground (true) or ground (false)
   */
  std::vector<bool> getPointSource() { return is_from_planground_; }

  /**
   * @brief Get the planground point cloud
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr getPlangroundCloud() { return pc_planground_; }

  /**
   * @brief Get the ground point cloud
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr getGroundCloud() { return pc_ground_; }

private:
  /**
   * @brief Calculate the straight-line distance between two points
   */
  double straightLineDistance(const pcl::PointXYZI& a, const pcl::PointXYZI& b);

  /**
   * @brief Calculate the path length
   */
  double calculatePathLength(const std::vector<unsigned int>& path, 
                             pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

  /**
   * @brief Calculate the distance from a point to the nearest planground point
   * @param pt The query point
   * @return Distance to nearest planground point
   */
  double distanceToNearestPlanground(const pcl::PointXYZI& pt);

  /**
   * @brief Calculate the edge penalty for a ground point based on local point density
   * 
   * Ground points near the edge of the point cloud (low local density) receive
   * additional cost to discourage the planner from planning near cloud edges.
   * This helps keep the planned path away from unsafe areas where the ground
   * point cloud is sparse or at its boundary.
   * 
   * The penalty is calculated as:
   *   penalty = edge_penalty_weight_ * exp(-falloff_rate_ * neighbor_count)
   * 
   * Where neighbor_count is the number of points within edge_penalty_radius_
   * of the query point. Points with few neighbors (edge points) get high penalty,
   * while points with many neighbors (interior points) get low penalty.
   * 
   * @param pt The query point
   * @return Edge penalty value (0.0 for interior points, up to edge_penalty_weight_ for edge points)
   */
  double calculateEdgePenalty(const pcl::PointXYZI& pt);

  /**
   * @brief Calculate the ground cost for a point based on its distance to planground

   * 
   * Cost function (v23 - Ultra-Strong Planground Preference with Maximum Anti-Detour):
   * 
   *   base_cost = planground_bias_ * (1.0 + ratio^2)
   *   where ratio = dist_to_planground / max_ground_bridge_length_
   * 
   *   detour_balance_factor = exp(k * (threshold - detour_ratio))
   *   where k = 0.5, clamped to [detour_balance_factor_lower_bound_, detour_balance_factor_upper_bound_]
   * 
   *   distance_balance_factor = 1.0 + 0.5 * tanh((dist - threshold) / threshold)
   *   clamped to [0.5, 1.5]
   * 
   *   distance_penalty = exp(excess_ratio * 2.0) if dist_to_planground > max_ground_bridge_length_
   * 
   *   final_cost = base_cost * detour_balance_factor * distance_balance_factor * distance_penalty
   *   clamped to [min_ground_cost, max_ground_cost_]
   * 
   * v23改进:
   * - planground_bias_从5.0提高到10.0，大幅提高地面点基础代价
   * - min_ground_cost_从50.0提高到100.0，确保地面点始终比planground点贵很多
   * - max_ground_bridge_length_从0.2降低到0.15，更严格限制地面路径长度
   * - detour_ratio_threshold_从3.0提高到4.0，只有当planground绕路超过4.0倍时才允许走地面
   * - distance_balance_threshold_从2.0降低到1.5，更多路径被视为"长距离"避免绕路
   * - max_ground_cost_从100.0提高到200.0，提高上限以允许更高的最小代价
   * - ground_path_length_penalty_从2.0提高到5.0，地面路径越长代价越高
   * - 新增detour_balance_factor_lower_bound_=0.5，即使planground严重绕路地面代价也不降太低
   * - 新增detour_balance_factor_upper_bound_=3.0，planground路径很直时地面代价大幅升高
   * 
   * @param dist_to_planground Distance from this point to the nearest planground point
   * @param planground_path_length Length of the planground-only path
   * @param straight_line_distance Straight-line distance between start and goal
   * @return The ground cost weight (set as intensity)
   */
  double calculateGroundCost(double dist_to_planground, 
                              double planground_path_length,
                              double straight_line_distance);

  /**
   * @brief Find the nearest point in the hybrid cloud to a given position
   * @param pt The query position
   * @param[out] hybrid_idx The index in the hybrid cloud
   * @return true if a nearest point was found
   */
  bool findNearestInHybrid(const pcl::PointXYZI& pt, unsigned int& hybrid_idx);

  /**
   * @brief Add a start pose point to the hybrid cloud
   * @param start_pose The robot's current position
   * @return The index of the added point in the hybrid cloud
   */
  unsigned int addStartPoseToHybrid(const pcl::PointXYZI& start_pose);

  /**
   * @brief Ensure the goal point is in the hybrid cloud with zero cost
   * @param goal_pose The goal position
   * @param goal_on_ground Whether the goal is from the ground cloud
   * @param ground_goal_idx If goal_on_ground, the index in the ground cloud
   * @return The index of the goal in the hybrid cloud
   */
  unsigned int ensureGoalInHybrid(const pcl::PointXYZI& goal_pose, 
                                   bool goal_on_ground = false,
                                   unsigned int ground_goal_idx = 0);

  /**
   * @brief Plan on the hybrid cloud between two indices
   * @param hybrid_start_idx Start index in the hybrid cloud
   * @param hybrid_goal_idx Goal index in the hybrid cloud
   * @param path Output path
   * @return true if a path was found
   */
  bool planOnHybrid(unsigned int hybrid_start_idx, unsigned int hybrid_goal_idx,
                     std::vector<unsigned int>& path);

  /**
   * @brief Plan on planground-only to get reference path length for detour ratio
   * @param start_pose Start position
   * @param goal_pose Goal position
   * @param path_length Output planground path length
   * @return true if a planground path was found
   */
  bool planOnPlangroundOnly(const pcl::PointXYZI& start_pose,
                             const pcl::PointXYZI& goal_pose,
                             double& path_length,
                             std::vector<unsigned int>* path_out = nullptr);

public:
  /**
   * @brief Smooth a path using cubic spline interpolation
   * 
   * This method takes a raw A* path (indices into the hybrid cloud) and produces
   * a smoothed path by:
   * 1. First simplifying the path using line-of-sight shortcutting (removing redundant nodes)
   * 2. Then applying cubic spline interpolation to generate smooth waypoints
   * 
   * The smoothed path is returned as a vector of 3D points (x, y, z) that can be
   * directly used for navigation.
   * 
   * @param raw_path_indices The raw A* path as indices into the hybrid cloud
   * @param cloud The point cloud (hybrid or planground) containing the point positions
   * @param smoothed_points Output: smoothed path as (x, y, z) points
   * @param smoothing_resolution Resolution of the smoothed path in meters (default: 0.2m)
   */
  void smoothPath(const std::vector<unsigned int>& raw_path_indices,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                  std::vector<std::tuple<double, double, double>>& smoothed_points,
                  double smoothing_resolution = 0.2);

  /**
   * @brief Simplify a path by removing redundant nodes using line-of-sight shortcutting
   * 
   * This is a pre-processing step before spline smoothing. It removes intermediate
   * nodes that can be shortcut by a straight line, reducing the number of control
   * points for the spline and making the final path smoother.
   * 
   * @param path_indices The raw path as indices into the cloud
   * @param cloud The point cloud
   * @param simplified_indices Output: simplified path indices
   */
  void simplifyPath(const std::vector<unsigned int>& path_indices,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    std::vector<unsigned int>& simplified_indices);

  /**
   * @brief Estimate planground path length between two poses
   * 
   * @param start_pose Start position (PoseStamped)
   * @param goal_pose Goal position (PoseStamped)
   * @return Estimated planground path length
   */
  double estimatePlangroundPathLength(const geometry_msgs::msg::PoseStamped& start_pose,
                                       const geometry_msgs::msg::PoseStamped& goal_pose);

  /**
   * @brief 平滑原始路径并直接生成ROS Path消息
   * 
   * 这是一个便捷方法，结合了smoothPath()和ROS路径构建，
   * 包括正确的朝向计算。
   * 
   * @param raw_path_indices 原始A*路径（云中的索引）
   * @param cloud 包含点位置的点云
   * @param ros_path 输出：平滑后的ROS路径
   * @param goal_pose 目标位姿（用于z和朝向参考）
   * @param frame_id ROS路径的坐标系ID
   * @param smoothing_resolution 平滑路径的分辨率（米，默认0.2m）
   */
  void smoothPathToRosPath(const std::vector<unsigned int>& raw_path_indices,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                           nav_msgs::msg::Path& ros_path,
                           const geometry_msgs::msg::PoseStamped& goal_pose,
                           const std::string& frame_id,
                           double smoothing_resolution = 0.2);

  /**
   * @brief 检测急转弯并在拐角处插入额外的控制点以保留拐角形状
   * 
   * 此函数识别急转弯（角度 < 阈值）并在拐角附近插入额外的
   * 控制点，防止样条曲线过度切割拐角。
   * 
   * @param path_indices 简化后的路径索引
   * @param cloud 点云
   * @param corner_enhanced_indices 输出：增强拐角后的路径索引
   * @param angle_threshold_deg 角度阈值（度，默认120度）
   */
  void enhanceCorners(const std::vector<unsigned int>& path_indices,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                      std::vector<unsigned int>& corner_enhanced_indices,
                      double angle_threshold_deg = 120.0);

private:
  // Point clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_planground_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_hybrid_;  // Fused cloud for planning

  // KD-trees
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_planground_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_ground_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_hybrid_;

  // Source tracking: true = planground point, false = ground point
  std::vector<bool> is_from_planground_;

  // Perception interface
  std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros_;

  // A* planner for the hybrid cloud
  A_Star_on_Graph* a_star_planner_;

  // Parameters
  double a_star_expanding_radius_;
  double turning_weight_;
  double planground_bias_ = 15.0;          // Base cost multiplier for ground points (v24: 从10.0提高到15.0)
                                           // 进一步提高地面点基础代价，确保规划主体在planground上
                                           // 只有planground确实严重绕路时才考虑走地面
  double max_ground_bridge_length_ = 0.12; // Max distance from planground for ground points to be useful (v24: 从0.15降低到0.12)
                                           // 更严格限制地面路径长度，避免绕路
                                           // 超过此距离的地面点代价急剧上升
  double hybrid_downsample_leaf_size_ = 0.15; // Voxel size for downsampling hybrid cloud
  
  // v24: Ultra-Strong Planground Preference with Maximum Anti-Detour (加强版)
  double detour_ratio_threshold_ = 4.0;    // Planground path length / straight-line ratio threshold (v23: 从3.0提高到4.0)
                                           // 只有当planground绕路超过4.0倍时才允许走地面
                                           // 避免轻微绕路就走地面导致更严重的绕路
  double max_ground_cost_ = 250.0;         // Maximum allowed ground cost (v24: 从200.0提高到250.0)
                                           // 提高上限以允许更高的最小代价
  double min_ground_cost_ = 150.0;         // Minimum ground cost (v24: 从100.0提高到150.0)
                                           // 进一步提高最小代价，确保地面点始终比planground点贵很多
                                           // 规划器只有在planground确实严重绕路时才考虑走地面
  double distance_balance_threshold_ = 1.5; // Distance balance threshold for short-vs-long trip trade-off (v23: 从2.0降低到1.5)
                                           // 短距离(<1.5m)允许轻微抄近路，长距离(>=1.5m)严格禁止走地面
                                           // 降低阈值使更多路径被视为"长距离"，避免绕路
  double ground_path_length_penalty_ = 5.0; // v23增强: 地面路径长度惩罚系数 (从2.0提高到5.0)
                                           // 地面路径越长，代价越高
                                           // 每走一步地面，代价乘以 (1 + ground_path_length_penalty_ * ground_steps / total_steps)
                                           // 确保长地面路径的代价远高于短地面路径
  double detour_balance_factor_lower_bound_ = 0.5; // v23新增: 绕路比平衡因子下限 (从0.3提高到0.5)
                                                   // 即使planground严重绕路，地面代价也不应降得太低
                                                   // 确保规划器不会因为planground绕路就轻易走地面
  double detour_balance_factor_upper_bound_ = 3.0; // v23新增: 绕路比平衡因子上限 (从2.0提高到3.0)
                                                   // 当planground路径很直时，地面代价大幅升高
                                                   // 确保规划器在planground路径合理时不会考虑走地面

  // v25 Edge penalty parameters - penalize ground points near cloud edges
  // These parameters add extra cost to ground points that are near the edge of the
  // ground point cloud (low local density), encouraging the planner to stay away
  // from cloud boundaries where the terrain data is less reliable.
  double edge_penalty_radius_ = 0.5;        // Search radius for counting neighbors (meters)
  double edge_penalty_weight_ = 200.0;      // Maximum edge penalty weight
  double edge_penalty_falloff_rate_ = 2.0;  // Rate at which penalty decreases with density
};

#endif // HYBRID_A_STAR_H
