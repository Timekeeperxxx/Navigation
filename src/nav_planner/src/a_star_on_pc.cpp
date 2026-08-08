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
#include <global_planner/a_star_on_pc.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

AstarList::AstarList(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_original_z_up){
  pc_original_z_up_ = pc_original_z_up;
  kdtree_ground_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  // A* only needs the complete neighbour set; sorting every radius result
  // adds work and does not affect costs or correctness.
  kdtree_ground_->setSortedResults(false);
  kdtree_ground_->setInputCloud(pc_original_z_up_);
}

void AstarList::Initial(){
  as_list_.assign(pc_original_z_up_->points.size(), Node_t{});
  node_revisions_.assign(pc_original_z_up_->points.size(), 0);
  for (size_t i = 0; i < as_list_.size(); ++i) {
    as_list_[i].self_index = static_cast<unsigned int>(i);
  }
  decltype(f_priority_queue_) empty_queue;
  f_priority_queue_.swap(empty_queue);
}

Node_t AstarList::getNode(unsigned int node_index){
  if (node_index >= as_list_.size()) {
    return Node_t{};
  }
  return as_list_[node_index];
}

float AstarList::getGVal(Node_t& a_node){
  if (a_node.self_index >= as_list_.size()) {
    return std::numeric_limits<float>::infinity();
  }
  return as_list_[a_node.self_index].g;
}

void AstarList::closeNode(Node_t& a_node){
  if (a_node.self_index >= as_list_.size()) {
    return;
  }
  as_list_[a_node.self_index].is_closed = true;
}

void AstarList::updateNode(Node_t& a_node){
  if (a_node.self_index >= as_list_.size()) {
    return;
  }
  as_list_[a_node.self_index] = a_node;
  const std::uint64_t revision = ++node_revisions_[a_node.self_index];
  f_priority_queue_.push(
    FrontierEntry{a_node.f, a_node.self_index, revision});
  //ROS_DEBUG("Add node ---> %u with g: %f, h: %f, f: %f",a_node.self_index, a_node.g, a_node.h, a_node.f);
}

bool AstarList::tryPopNodeWithMinimumF(Node_t& node){
  while (!f_priority_queue_.empty()) {
    const FrontierEntry entry = f_priority_queue_.top();
    f_priority_queue_.pop();

    if (entry.node_index >= as_list_.size() ||
        entry.node_index >= node_revisions_.size()) {
      continue;
    }

    const Node_t& candidate = as_list_[entry.node_index];
    if (candidate.is_closed || !candidate.is_opened ||
        node_revisions_[entry.node_index] != entry.revision) {
      continue;
    }

    node = candidate;
    return true;
  }

  node = Node_t{};
  return false;
}

Node_t AstarList::getNode_wi_MinimumF(){
  Node_t node;
  tryPopNodeWithMinimumF(node);
  return node;
}

bool AstarList::isClosed(unsigned int node_index){
  if (node_index >= as_list_.size()) {
    return true;
  }
  return as_list_[node_index].is_closed;
}

bool AstarList::isOpened(unsigned int node_index){
  if (node_index >= as_list_.size()) {
    return false;
  }
  return as_list_[node_index].is_opened;
}

bool AstarList::isFrontierEmpty(){
  while (!f_priority_queue_.empty()) {
    const FrontierEntry& entry = f_priority_queue_.top();
    if (entry.node_index < as_list_.size() &&
        entry.node_index < node_revisions_.size()) {
      const Node_t& candidate = as_list_[entry.node_index];
      if (!candidate.is_closed && candidate.is_opened &&
          node_revisions_[entry.node_index] == entry.revision) {
        return false;
      }
    }
    f_priority_queue_.pop();
  }
  return f_priority_queue_.empty();
}

//@----------------------------------------------------------------------------------------

A_Star_on_Graph::A_Star_on_Graph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up,
                                  std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros,
                                  double a_star_expanding_radius){

  perception_ros_ = perception_ros;
  pc_original_z_up_ = pc_original_z_up;
  a_star_expanding_radius_ = a_star_expanding_radius;
  ASLS_ = new AstarList(pc_original_z_up_);

  //@ Initialize ground line-of-sight kdtree
  kdtree_ground_los_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_ground_los_->setInputCloud(pc_original_z_up_);

  perception_ground_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  perception_ground_kdtree_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
}

A_Star_on_Graph::~A_Star_on_Graph(){
  if(ASLS_)
    delete ASLS_;
}

void A_Star_on_Graph::updateGraph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up){
  pc_original_z_up_ = pc_original_z_up;
  perception_cache_ready_ = false;
  planning_node_cache_.clear();
  ASLS_->pc_original_z_up_ = pc_original_z_up;
  ASLS_->kdtree_ground_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  ASLS_->kdtree_ground_->setSortedResults(false);
  ASLS_->kdtree_ground_->setInputCloud(pc_original_z_up);

  //@ Update ground line-of-sight kdtree
  kdtree_ground_los_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_ground_los_->setInputCloud(pc_original_z_up);
}

void A_Star_on_Graph::setMaxPlanningTime(double seconds){
  max_planning_time_seconds_ =
    std::isfinite(seconds) && seconds > 0.0 ? seconds : 0.0;
}

void A_Star_on_Graph::setCancelChecker(CancelChecker checker){
  cancel_checker_ = std::move(checker);
}

void A_Star_on_Graph::setEdgeValidator(EdgeValidator validator){
  edge_validator_ = std::move(validator);
}

void A_Star_on_Graph::setIndexEdgeValidator(IndexEdgeValidator validator){
  index_edge_validator_ = std::move(validator);
}

void A_Star_on_Graph::setHeuristicWeight(double weight){
  heuristic_weight_ =
    std::isfinite(weight) && weight >= 0.0 ? weight : 1.0;
}

void A_Star_on_Graph::setUsePerceptionCosts(bool enabled){
  if (use_perception_costs_ != enabled) {
    perception_cache_ready_ = false;
    planning_node_cache_.clear();
  }
  use_perception_costs_ = enabled;
}

double A_Star_on_Graph::getPitchFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding){
  //@ calculate vector: parent -> current
  float vx1, vy1, s1;
  vx1 = m_pcl_current.x - m_pcl_current_parent.x;
  vy1 = m_pcl_current.y - m_pcl_current_parent.y;
  s1 = sqrt(vx1*vx1 + vy1*vy1);
  //@ calculate vector: current -> expanding
  float vx2, vy2, s2;
  vx2 = m_pcl_expanding.x - m_pcl_current.x;
  vy2 = m_pcl_expanding.y - m_pcl_current.y;
  s2 = sqrt(vx2*vx2 + vy2*vy2);

  float pitch = fabs(m_pcl_current_parent.z - m_pcl_expanding.z)/(s1+s2);

  return pitch;
}

double A_Star_on_Graph::getThetaFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding){
  //@ calculate vector: parent -> current
  float vx1, vy1;
  vx1 = m_pcl_current.x - m_pcl_current_parent.x;
  vy1 = m_pcl_current.y - m_pcl_current_parent.y;
  //@ calculate vector: current -> expanding
  float vx2, vy2;
  vx2 = m_pcl_expanding.x - m_pcl_current.x;
  vy2 = m_pcl_expanding.y - m_pcl_current.y;
  float cos_theta = (vx1*vx2 + vy1*vy2)/(sqrt(vx1*vx1+vy1*vy1)*sqrt(vx2*vx2+vy2*vy2));
  if(fabs(cos_theta)>1)
    cos_theta = 1.0;
  double theta_of_vector = acos(cos_theta);
  if(vx1==0 && vy1==0)
    theta_of_vector = 0;
  else if(vx2==0 && vy2==0)
    theta_of_vector = 0;
  else if(fabs(fabs(vx1)-fabs(vx2))<=0.0001)
    theta_of_vector = 0;

  if(fabs(theta_of_vector)<=0.345)//cap
    theta_of_vector = 0.0;

  return theta_of_vector;
}

bool A_Star_on_Graph::isLineOfSightClear(
  const pcl::PointXYZI& pcl_current,
  const pcl::PointXYZI& pcl_expanding,
  double inscribed_radius){

  //@ generate line equation
  float dX =
      pcl_expanding.x - pcl_current.x;
  float dY =
      pcl_expanding.y - pcl_current.y;
  float dZ =
      pcl_expanding.z - pcl_current.z;

  float distance = sqrt(dX*dX + dY*dY + dZ*dZ);

  //@ CRITICAL FIX: Prevent division by zero when inscribed_radius or distance is zero
  if (inscribed_radius <= 0.0 || distance <= 0.0) {
    return true; // If no meaningful check can be done, assume line of sight is clear
  }

  distance = distance/inscribed_radius; //sample by every inscribed radius
  float dt = 1/distance;

  //@ Track the number of consecutive samples that have insufficient ground support
  //@ This helps detect gaps between disconnected point cloud patches
  int consecutive_no_ground_count = 0;
  const int max_consecutive_no_ground = 1; // Allow at most 1 consecutive sample without ground

  for(float t=0; t<=1.0+dt; t+=dt){
    float r = t;
    if(t>=1.0) //@ make sure we examine t=1.0
      r = 1.0;
    pcl::PointXYZI a_pt;
    a_pt.intensity = 0.0;
    a_pt.x = pcl_current.x + dX*r;
    a_pt.y = pcl_current.y + dY*r;
    a_pt.z = pcl_current.z + dZ*r;
    std::vector<int> pidx;
    std::vector<float> prsd;
    kdtree_lethal_->radiusSearch(a_pt, 2*inscribed_radius, pidx, prsd);
    if(pidx.size()>1){
      return false;
    }

    //@ Check ground connectivity: ensure the path stays on the ground
    //@ If there is no ground point nearby, the path is crossing a gap/blank area
    std::vector<int> ground_pidx;
    std::vector<float> ground_prsd;
    kdtree_ground_los_->radiusSearch(a_pt, inscribed_radius, ground_pidx, ground_prsd);
    if(ground_pidx.size()<1){
      consecutive_no_ground_count++;
      if(consecutive_no_ground_count > max_consecutive_no_ground){
        return false;
      }
    }
    else{
      //@ Reset counter when we find ground support
      consecutive_no_ground_count = 0;

      //@ Additional check: verify that the ground points found are not just a thin line
      //@ by checking if they are spread out in different directions
      //@ This helps detect cases where the interpolated point is near the edge of a point cloud patch
      //@ and the ground points are all clustered on one side (indicating a gap boundary)
      if(ground_pidx.size() < 3){
        //@ Too few ground points - likely at the edge of a point cloud patch
        //@ Check if the nearest ground point is significantly closer than the second nearest
        //@ If so, this suggests we're at the boundary of a point cloud patch
        if(ground_pidx.size() >= 2){
          float ratio = ground_prsd[1] / (ground_prsd[0] + 0.0001);
          if(ratio > 4.0){
            //@ The second nearest point is much farther than the nearest
            //@ This suggests we're at the edge of a point cloud patch
            consecutive_no_ground_count++;
            if(consecutive_no_ground_count > max_consecutive_no_ground){
              return false;
            }
          }
        }
      }
    }
  }
  return true;
}

bool A_Star_on_Graph::getPerceptionGroundIndex(
  unsigned int planning_index, unsigned int& ground_index){
  if (planning_index >= pc_original_z_up_->points.size() ||
      !perception_ground_cloud_ || perception_ground_cloud_->empty()) {
    return false;
  }

  std::vector<int> indices(1);
  std::vector<float> distances(1);
  if (perception_ground_kdtree_->nearestKSearch(
        pc_original_z_up_->points[planning_index], 1, indices, distances) <= 0 ||
      indices[0] < 0) {
    return false;
  }

  ground_index = static_cast<unsigned int>(indices[0]);
  return ground_index < perception_ground_cloud_->points.size();
}

void A_Star_on_Graph::getPath(
  unsigned int start, unsigned int goal,
  std::vector<unsigned int>& path){
  using SteadyClock = std::chrono::steady_clock;

  path.clear();
  planning_timed_out_ = false;
  planning_cancelled_ = false;
  const auto planning_started_at = SteadyClock::now();
  size_t neighbor_candidates = 0;
  size_t index_edge_rejections = 0;
  size_t point_edge_checks = 0;
  size_t point_edge_rejections = 0;
  double point_edge_check_seconds = 0.0;

  const auto should_stop = [&]() {
    if (cancel_checker_ && cancel_checker_()) {
      planning_cancelled_ = true;
      return true;
    }
    if (max_planning_time_seconds_ > 0.0 &&
        std::chrono::duration<double>(
          SteadyClock::now() - planning_started_at).count() >=
          max_planning_time_seconds_) {
      planning_timed_out_ = true;
      return true;
    }
    return false;
  };

  if(start >= pc_original_z_up_->points.size() || goal >= pc_original_z_up_->points.size()){
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[A*] Invalid start (%u) or goal (%u) for cloud size %lu",
      start, goal, pc_original_z_up_->points.size());
    return;
  }

  pcl::PointXYZI pcl_goal = pc_original_z_up_->points[goal];
  pcl::PointXYZI pcl_start = pc_original_z_up_->points[start];
  const float initial_h = static_cast<float>(
    heuristic_weight_ *
    std::sqrt(pcl::geometry::squaredDistance(pcl_start, pcl_goal)));
  Node_t current_node;
  current_node.self_index = start;
  current_node.g = 0.0f;
  current_node.h = initial_h;
  current_node.f = initial_h;
  current_node.parent_index = start;
  current_node.is_opened = true;

  double inscribed_radius =
    perception_ros_->getGlobalUtils()->getInscribedRadius();
  double inflation_descending_rate =
    perception_ros_->getGlobalUtils()->getInflationDescendingRate();

  auto shared_data = perception_ros_->getSharedDataPtr();
  if (use_perception_costs_ && !perception_cache_ready_) {
    if (!shared_data || !shared_data->pcl_ground_ ||
        shared_data->pcl_ground_->empty()) {
      RCLCPP_WARN(
        perception_ros_->get_logger(),
        "[A*] Perception ground is unavailable while perception costs are enabled");
      return;
    }
    // Build this translation tree lazily.  Reference-only searches explicitly
    // disable perception costs and therefore avoid copying/indexing the full
    // ground cloud before they can even start searching.
    *perception_ground_cloud_ = *shared_data->pcl_ground_;
    perception_ground_kdtree_->setInputCloud(perception_ground_cloud_);

    perception_ros_->getStackedPerception()->aggregateLethal();
    //@ generate kd-tree and handle no point cloud edge case
    kdtree_lethal_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());

    if(perception_ros_->getSharedDataPtr()->aggregate_lethal_->points.size()>0){
      kdtree_lethal_->setInputCloud(perception_ros_->getSharedDataPtr()->aggregate_lethal_);
    } else {
      //@ CRITICAL FIX: Even when lethal cloud is empty, we must set an empty input cloud
      //@ to prevent null pointer dereference in isLineOfSightClear() -> kdtree_lethal_->radiusSearch()
      pcl::PointCloud<pcl::PointXYZI>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      kdtree_lethal_->setInputCloud(empty_cloud);
    }
    perception_cache_ready_ = true;
  }

  // Resolve planning nodes into perception/static-graph index space lazily and
  // at most once per search.  Weighted A* normally touches only a fraction of
  // a large hybrid cloud; eagerly translating every node delayed the first
  // expansion without improving the selected route.
  if (planning_node_cache_.size() != pc_original_z_up_->points.size()) {
    planning_node_cache_.assign(
      pc_original_z_up_->points.size(), PlanningNodeCache{});
  }
  std::vector<int> ground_indices(1);
  std::vector<float> ground_distances(1);
  auto cached_perception_node =
    [this, &shared_data,
     &ground_indices, &ground_distances](
      size_t planning_index) -> const PlanningNodeCache& {
      PlanningNodeCache& cached = planning_node_cache_[planning_index];
      if (cached.initialized) {
        return cached;
      }
      cached.initialized = true;

      if (!perception_ground_cloud_ || perception_ground_cloud_->empty() ||
          !shared_data || !shared_data->sGraph_ptr_ ||
          perception_ground_kdtree_->nearestKSearch(
            pc_original_z_up_->points[planning_index], 1,
            ground_indices, ground_distances) <= 0 ||
          ground_indices[0] < 0 ||
          static_cast<size_t>(ground_indices[0]) >=
            perception_ground_cloud_->points.size()) {
        return cached;
      }

      const unsigned int perception_ground_index =
        static_cast<unsigned int>(ground_indices[0]);
      cached.perception_ground_index = perception_ground_index;
      cached.dgraph_value =
        perception_ros_->get_min_dGraphValue(perception_ground_index);
      cached.node_weight =
        shared_data->sGraph_ptr_->getNodeWeight(perception_ground_index);
      cached.valid = true;
      return cached;
    };

  ASLS_->Initial();
  ASLS_->updateNode(current_node);

  const size_t max_iterations = std::max<size_t>(pc_original_z_up_->points.size() * 4, 50000);
  size_t iter_count = 0;
  while(ASLS_->tryPopNodeWithMinimumF(current_node)){
    if (should_stop()) {
      RCLCPP_WARN(perception_ros_->get_logger(),
        "[A*] Planning %s after %lu expansions (cloud=%lu)",
        planning_cancelled_ ? "cancelled" : "timed out",
        iter_count, pc_original_z_up_->points.size());
      path.clear();
      return;
    }

    if(++iter_count > max_iterations){
      RCLCPP_WARN(perception_ros_->get_logger(),
        "[A*] Aborting: exceeded max iterations (%lu) without reaching goal. cloud=%lu start=%u goal=%u",
        max_iterations, pc_original_z_up_->points.size(), start, goal);
      path.clear();
      return;
    }

    if(current_node.self_index >= pc_original_z_up_->points.size()) {
      RCLCPP_WARN(perception_ros_->get_logger(),
        "[A*] Aborting: self_index=%u out of bounds (cloud=%lu)",
        current_node.self_index, pc_original_z_up_->points.size());
      path.clear();
      return;
    }
    if (current_node.parent_index >= pc_original_z_up_->points.size()) {
      RCLCPP_WARN(perception_ros_->get_logger(),
        "[A*] Aborting expansion: parent_index=%u out of bounds "
        "(node=%u cloud=%lu)",
        current_node.parent_index, current_node.self_index,
        pc_original_z_up_->points.size());
      ASLS_->closeNode(current_node);
      continue;
    }

    //RCLCPP_DEBUG(rclcpp::get_logger("astar"), "Expand node: %u", current_node.self_index);
    /*Get successors*/
    pcl::PointXYZI pcl_now = pc_original_z_up_->points[current_node.self_index];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    ASLS_->kdtree_ground_->radiusSearch(pcl_now, a_star_expanding_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    //@dealing with orphan node
    if(pointIdxRadiusSearch.size()<8){
      const int found = ASLS_->kdtree_ground_->nearestKSearch(
        pcl_now, 8, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      pointIdxRadiusSearch.resize(std::max(found, 0));
      pointRadiusSquaredDistance.resize(std::max(found, 0));
    }

    for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){
      if ((it & 0x7fU) == 0U && should_stop()) {
        RCLCPP_WARN(perception_ros_->get_logger(),
          "[A*] Planning %s while expanding node %u (cloud=%lu)",
          planning_cancelled_ ? "cancelled" : "timed out",
          current_node.self_index, pc_original_z_up_->points.size());
        path.clear();
        return;
      }

      int current_expanding_index = pointIdxRadiusSearch[it];
      if(current_expanding_index < 0 || static_cast<size_t>(current_expanding_index) >= pc_original_z_up_->points.size())
        continue;
      if (ASLS_->isClosed(static_cast<unsigned int>(current_expanding_index))) {
        continue;
      }
      ++neighbor_candidates;

      if (index_edge_validator_ &&
          !index_edge_validator_(
            current_node.self_index,
            static_cast<unsigned int>(current_expanding_index))) {
        ++index_edge_rejections;
        continue;
      }

      const pcl::PointXYZI& pcl_current =
        pc_original_z_up_->points[current_node.self_index];
      const pcl::PointXYZI& pcl_expanding =
        pc_original_z_up_->points[current_expanding_index];
      if (edge_validator_) {
        ++point_edge_checks;
        const auto edge_check_started_at = SteadyClock::now();
        const bool edge_is_valid =
          edge_validator_(pcl_current, pcl_expanding);
        point_edge_check_seconds += std::chrono::duration<double>(
          SteadyClock::now() - edge_check_started_at).count();
        if (!edge_is_valid) {
          ++point_edge_rejections;
          continue;
        }
      }

      const PlanningNodeCache& cached = use_perception_costs_ ?
        cached_perception_node(
          static_cast<size_t>(current_expanding_index)) :
        planning_node_cache_[static_cast<size_t>(current_expanding_index)];
      if (use_perception_costs_ && !cached.valid) {
        continue;
      }
      const double dGraphValue = use_perception_costs_ ?
        cached.dgraph_value : std::numeric_limits<double>::infinity();

      //@ Use intensity as ground edge weight for hybrid planning (v22 - Strong Planground Preference)
      //@
      //@ v22关键改进: 大幅提高离开planground的代价，确保规划主体在planground上
      //@
      //@ intensity=0 means planground point (no extra cost, preferred surface)
      //@ intensity>0 means ground point (extra cost already computed by calculateGroundCost)
      //@
      //@ Key design for hybrid planning v22:
      //@ 1. Moving on planground (0->0): no extra cost - encourages staying on planground
      //@ 2. Moving from planground to ground (0->positive):
      //@    cost = expanding_intensity + PLANGROUND_EXIT_PENALTY (v22新增!)
      //@    离开planground需要支付额外惩罚，防止规划器轻易离开planground
      //@ 3. Moving on ground (positive->positive):
      //@    cost = expanding_intensity + GROUND_STEP_PENALTY (v22新增!)
      //@    每走一步地面都要支付额外惩罚，累积效应使长地面路径代价极高
      //@ 4. Moving from ground back to planground (positive->0):
      //@    cost = 0 (FREE!) - 强烈鼓励返回planground
      //@
      //@ v22新增惩罚系数:
      //@ - PLANGROUND_EXIT_PENALTY = 10.0: 离开planground的固定惩罚
      //@ - GROUND_STEP_PENALTY = 5.0: 每步地面的额外惩罚
      //@   这两个惩罚确保规划器只在必要时才离开planground
      //@   并且一旦离开会尽快返回
      float current_intensity = pc_original_z_up_->points[current_node.self_index].intensity;
      float expanding_intensity = pc_original_z_up_->points[current_expanding_index].intensity;

      // v24: 大幅提高离开planground的惩罚，确保规划主体在planground上
      // PLANGROUND_EXIT_PENALTY: 离开planground的固定惩罚 (10.0 -> 50.0)
      //   提高5倍，确保规划器不会轻易离开planground
      // GROUND_STEP_PENALTY: 每步地面的额外惩罚 (5.0 -> 20.0)
      //   提高4倍，即使离开了planground，也会尽快返回
      // 这两个惩罚确保规划器只在planground确实严重绕路时才考虑走地面
      // 并且一旦离开会尽快返回planground
      const float PLANGROUND_EXIT_PENALTY = 50.0f;
      const float GROUND_STEP_PENALTY = 20.0f;

      float ground_edge_weight;
      if (current_intensity == 0.0f && expanding_intensity == 0.0f) {
        // Both planground: no extra cost - planner stays on preferred surface
        ground_edge_weight = 0.0f;
      } else if (current_intensity == 0.0f && expanding_intensity > 0.0f) {
        // Planground -> Ground: pay the ground cost + exit penalty
        // v22: 增加离开planground的固定惩罚，防止轻易离开
        ground_edge_weight = expanding_intensity + PLANGROUND_EXIT_PENALTY;
      } else if (current_intensity > 0.0f && expanding_intensity == 0.0f) {
        // Ground -> Planground: FREE transition!
        // This strongly encourages returning to planground
        ground_edge_weight = 0.0f;
      } else {
        // Both ground: pay the expanding node's intensity + ground step penalty
        // v22: 每走一步地面都要支付额外惩罚
        // 这会产生累积效应：地面路径越长，总代价越高
        // 规划器会倾向于尽快返回planground
        ground_edge_weight = expanding_intensity + GROUND_STEP_PENALTY;
      }
      float current_expanding_g = sqrt(pointRadiusSquaredDistance[it]);

      /*This is for lethal*/
      if(use_perception_costs_ && dGraphValue<inscribed_radius){
        //RCLCPP_DEBUG(rclcpp::get_logger("astar"), "%.2f,%.2f,%.2f, v: %.2f",pc_original_z_up_->points[(*it).first].x,pc_original_z_up_->points[(*it).first].y,pc_original_z_up_->points[(*it).first].z, dGraphValue);
        continue;
      }

      const pcl::PointXYZI& pcl_current_parent =
        pc_original_z_up_->points[current_node.parent_index];

      //@ check line-of-sight when distance is 2 times larger than inscribed_radius
      if(use_perception_costs_ &&
         current_expanding_g>=2*inscribed_radius){
        if(!isLineOfSightClear(pcl_current, pcl_expanding, inscribed_radius))
          continue;
      }

      const double factor = use_perception_costs_ ?
        exp(-1.0 * inflation_descending_rate *
          (dGraphValue - inscribed_radius)) : 0.0;

      //@ get current_parent, current, expanding to compute theta od expanding
      double theta = getThetaFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding);

      //if(getPitchFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding)>0.2)
      //  continue;

      float new_g = current_node.g + current_expanding_g + factor * 1.0 +
        (use_perception_costs_ ? cached.node_weight : 0.0f) +
        theta*turning_weight_ + ground_edge_weight;
      float new_h = static_cast<float>(
        heuristic_weight_ *
        std::sqrt(pcl::geometry::squaredDistance(pcl_expanding, pcl_goal)));
      float new_f = new_g + new_h;
      if(!std::isfinite(new_f)) continue;

      Node_t new_node;
      new_node.self_index = static_cast<unsigned int>(current_expanding_index);
      new_node.g = new_g;
      new_node.h = new_h;
      new_node.f = new_f;
      new_node.parent_index = current_node.self_index;
      new_node.is_opened = true;

      /*Check is in opened list*/
      if(ASLS_->isOpened(current_expanding_index)){
        if(ASLS_->getGVal(new_node)>new_g){
          ASLS_->updateNode(new_node);
        }
      }
      /*addNode*/
      else{
        ASLS_->updateNode(new_node);
      }


    }

    /*Close this node*/
    ASLS_->closeNode(current_node);

    /*If goal is in closed list, we are done*/
    if(ASLS_->isClosed(goal)){
      Node_t trace_back = ASLS_->getNode(goal);
      size_t trace_steps = 0;
      const size_t max_trace = pc_original_z_up_->points.size() + 16;
      while(true){
        if (trace_back.self_index >= pc_original_z_up_->points.size() ||
            trace_back.parent_index >= pc_original_z_up_->points.size()) {
          RCLCPP_WARN(perception_ros_->get_logger(),
            "[A*] Invalid traceback node=%u parent=%u (cloud=%lu), aborting",
            trace_back.self_index, trace_back.parent_index,
            pc_original_z_up_->points.size());
          path.clear();
          return;
        }
        path.push_back(trace_back.self_index);
        if (trace_back.self_index == trace_back.parent_index) {
          break;
        }
        if(++trace_steps > max_trace){
          RCLCPP_WARN(perception_ros_->get_logger(),
            "[A*] Traceback exceeded %lu steps, aborting", max_trace);
          path.clear();
          return;
        }
        trace_back = ASLS_->getNode(trace_back.parent_index);
      }
      std::reverse(path.begin(),path.end());
      const double elapsed_seconds = std::chrono::duration<double>(
        SteadyClock::now() - planning_started_at).count();
      RCLCPP_INFO(
        perception_ros_->get_logger(),
        "[A*] Path found after %lu expansions in %.3f s "
        "(cloud=%lu candidates=%lu blocked=%lu footprint=%lu/%lu %.3fs)",
        iter_count, elapsed_seconds, pc_original_z_up_->points.size(),
        neighbor_candidates, index_edge_rejections,
        point_edge_rejections, point_edge_checks,
        point_edge_check_seconds);
      return;
    }

    /*Check if*/
  }

  const double elapsed_seconds = std::chrono::duration<double>(
    SteadyClock::now() - planning_started_at).count();
  RCLCPP_WARN(
    perception_ros_->get_logger(),
    "[A*] Search exhausted after %lu expansions in %.3f s "
    "(cloud=%lu candidates=%lu blocked=%lu footprint=%lu/%lu %.3fs)",
    iter_count, elapsed_seconds, pc_original_z_up_->points.size(),
    neighbor_candidates, index_edge_rejections,
    point_edge_rejections, point_edge_checks,
    point_edge_check_seconds);
}
