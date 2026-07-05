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
#include <cmath>
#include <algorithm>

AstarList::AstarList(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_original_z_up){
  pc_original_z_up_ = pc_original_z_up;
  kdtree_ground_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_ground_->setInputCloud(pc_original_z_up_);
}

void AstarList::Initial(){
  as_list_.clear(); 
  for(unsigned int it=0; it!=pc_original_z_up_->points.size();it++){
    Node_t new_node = {.self_index=0, .g=0, .h=0, .f=0, .parent_index=0, .is_closed=false, .is_opened=false, .consecutive_ground_steps=0};
    as_list_[it] = new_node;
  }
  f_priority_set_.clear();
}

Node_t AstarList::getNode(unsigned int node_index){

  return as_list_[node_index];
}

float AstarList::getGVal(Node_t& a_node){
  return as_list_[a_node.self_index].g;
}

void AstarList::closeNode(Node_t& a_node){
  as_list_[a_node.self_index].is_closed = true;
}

void AstarList::updateNode(Node_t& a_node){
  as_list_[a_node.self_index] = a_node;
  f_p_ afp;
  afp.first = a_node.f; //made minimum f to be top so we can pop it
  afp.second = a_node.self_index;
  f_priority_set_.insert(afp);
  //ROS_DEBUG("Add node ---> %u with g: %f, h: %f, f: %f",a_node.self_index, a_node.g, a_node.h, a_node.f);
}

Node_t AstarList::getNode_wi_MinimumF(){
  //@ CRITICAL FIX: Check if frontier is empty before accessing begin()
  if (f_priority_set_.empty()) {
    Node_t empty_node;
    empty_node.self_index = 0;
    empty_node.is_closed = true;
    return empty_node;
  }
  
  auto first_it = f_priority_set_.begin();
  Node_t m_node = as_list_[(*first_it).second];
  if(!m_node.is_closed){
    f_priority_set_.erase(first_it);
    return m_node;
  }
  
  //Because we updateNode node even when new g value is smaller than that in openlist
  //We will have duplicate f value in the f_priority_set_
  int concern_cnt = 0;
  while(m_node.is_closed && !f_priority_set_.empty()){
    concern_cnt++;
    f_priority_set_.erase(first_it);
    first_it = f_priority_set_.begin();
    if (first_it == f_priority_set_.end()) {
      //@ CRITICAL FIX: Frontier became empty after erasing, return empty node
      Node_t empty_node;
      empty_node.self_index = 0;
      empty_node.is_closed = true;
      return empty_node;
    }
    m_node = as_list_[(*first_it).second];
  }
  return m_node;
}

bool AstarList::isClosed(unsigned int node_index){
  return as_list_[node_index].is_closed;
}

bool AstarList::isOpened(unsigned int node_index){
  return as_list_[node_index].is_opened;
}

bool AstarList::isFrontierEmpty(){
  return f_priority_set_.empty();
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
}

A_Star_on_Graph::~A_Star_on_Graph(){
  if(ASLS_)
    delete ASLS_;
}

void A_Star_on_Graph::updateGraph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up){
  ASLS_->pc_original_z_up_ = pc_original_z_up;
  ASLS_->kdtree_ground_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  ASLS_->kdtree_ground_->setInputCloud(pc_original_z_up_);
  
  //@ Update ground line-of-sight kdtree
  kdtree_ground_los_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_ground_los_->setInputCloud(pc_original_z_up_);
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

bool A_Star_on_Graph::isLineOfSightClear(pcl::PointXYZI& pcl_current, pcl::PointXYZI& pcl_expanding, double inscribed_radius){

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

void A_Star_on_Graph::getPath(
  unsigned int start, unsigned int goal,
  std::vector<unsigned int>& path){

  /*
  Create the first node which is start and add into frontier
  */
  if(start >= pc_original_z_up_->points.size() || goal >= pc_original_z_up_->points.size()){
    RCLCPP_WARN(perception_ros_->get_logger(),
      "[A*] Invalid start (%u) or goal (%u) for cloud size %lu",
      start, goal, pc_original_z_up_->points.size());
    return;
  }

  pcl::PointXYZI pcl_goal = pc_original_z_up_->points[goal];
  pcl::PointXYZI pcl_start = pc_original_z_up_->points[start];
  float f = sqrt(pcl::geometry::squaredDistance(pcl_start, pcl_goal));
  Node_t current_node = {.self_index=start, .g=0, .h=0, .f=f, .parent_index=start, .is_closed=false, .is_opened=true};

  ASLS_->Initial();
  ASLS_->updateNode(current_node);

  double inscribed_radius = perception_ros_->getGlobalUtils()->getInscribedRadius();
  double inflation_descending_rate = perception_ros_->getGlobalUtils()->getInflationDescendingRate();
  double max_obstacle_distance = perception_ros_->getGlobalUtils()->getMaxObstacleDistance();

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

  const size_t max_iterations = std::max<size_t>(pc_original_z_up_->points.size() * 4, 50000);
  size_t iter_count = 0;
  while(!ASLS_->isFrontierEmpty()){
    if(++iter_count > max_iterations){
      RCLCPP_WARN(perception_ros_->get_logger(),
        "[A*] Aborting: exceeded max iterations (%lu) without reaching goal. cloud=%lu start=%u goal=%u",
        max_iterations, pc_original_z_up_->points.size(), start, goal);
      path.clear();
      return;
    }
    /*Pop minimum F, we leverage prior queue, so we dont need to loop frontier everytime*/
    current_node = ASLS_->getNode_wi_MinimumF();
    if(current_node.self_index >= pc_original_z_up_->points.size()) {
      RCLCPP_WARN(perception_ros_->get_logger(),
        "[A*] Aborting: self_index=%u out of bounds (cloud=%lu)",
        current_node.self_index, pc_original_z_up_->points.size());
      path.clear();
      return;
    }

    //RCLCPP_DEBUG(rclcpp::get_logger("astar"), "Expand node: %u", current_node.self_index);
    /*Get successors*/
    pcl::PointXYZI pcl_now = pc_original_z_up_->points[current_node.self_index];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    ASLS_->kdtree_ground_->radiusSearch(pcl_now, a_star_expanding_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    //@dealing with orphan node
    if(pointIdxRadiusSearch.size()<8){
      ASLS_->kdtree_ground_->nearestKSearch(pcl_now, 8, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    }

    for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){

      int current_expanding_index = pointIdxRadiusSearch[it];
      if(current_expanding_index < 0 || static_cast<size_t>(current_expanding_index) >= pc_original_z_up_->points.size())
        continue;

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

      //@ dGraphValue is the distance to lethal
      double dGraphValue = perception_ros_->get_min_dGraphValue(current_expanding_index);

      /*This is for lethal*/
      if(dGraphValue<inscribed_radius){
        //RCLCPP_DEBUG(rclcpp::get_logger("astar"), "%.2f,%.2f,%.2f, v: %.2f",pc_original_z_up_->points[(*it).first].x,pc_original_z_up_->points[(*it).first].y,pc_original_z_up_->points[(*it).first].z, dGraphValue);
        continue;
      }

      // CRITICAL FIX: Validate parent_index before accessing to prevent segfault
      if (current_node.parent_index >= pc_original_z_up_->points.size()) {
        RCLCPP_WARN(perception_ros_->get_logger(),
          "[A*] Invalid parent_index=%u (cloud=%lu), skipping expansion",
          current_node.parent_index, pc_original_z_up_->points.size());
        continue;
      }
      pcl::PointXYZI pcl_current = pc_original_z_up_->points[current_node.self_index];
      pcl::PointXYZI pcl_current_parent = pc_original_z_up_->points[current_node.parent_index];
      pcl::PointXYZI pcl_expanding = pc_original_z_up_->points[current_expanding_index];

      //@ check line-of-sight when distance is 2 times larger than inscribed_radius
      if(current_expanding_g>=2*inscribed_radius){
        if(!isLineOfSightClear(pcl_current, pcl_expanding, inscribed_radius))
          continue;
      }
      
      double factor = exp(-1.0 * inflation_descending_rate * (dGraphValue - inscribed_radius));

      //@ get current_parent, current, expanding to compute theta od expanding
      double theta = getThetaFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding);
      
      //if(getPitchFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding)>0.2)
      //  continue;
      
      float node_weight = perception_ros_->getSharedDataPtr()->sGraph_ptr_->getNodeWeight(current_expanding_index);
      float new_g = current_node.g + current_expanding_g + factor * 1.0 + node_weight + theta*turning_weight_ + ground_edge_weight;
      float new_h = sqrt(pcl::geometry::squaredDistance(pcl_expanding, pcl_goal));
      float new_f = new_g + new_h;
      if(!std::isfinite(new_f)) continue;

      Node_t new_node = {.self_index=static_cast<unsigned int>(current_expanding_index), .g=new_g, .h=new_h, .f=new_f, .parent_index=static_cast<unsigned int>(current_node.self_index), .is_closed=false, .is_opened=true, .consecutive_ground_steps=0};

      /*Check is in closed list*/
      if(ASLS_->isClosed(current_expanding_index))
        continue;
      /*Check is in opened list*/
      else if(ASLS_->isOpened(current_expanding_index)){
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
      while(trace_back.self_index!=trace_back.parent_index){
        path.push_back(trace_back.self_index);
        if(++trace_steps > max_trace){
          RCLCPP_WARN(perception_ros_->get_logger(),
            "[A*] Traceback exceeded %lu steps, aborting", max_trace);
          path.clear();
          return;
        }
        trace_back = ASLS_->getNode(trace_back.parent_index);
      }
      path.push_back(trace_back.self_index);//Push start point
      std::reverse(path.begin(),path.end());
      break;
    }

    /*Check if*/
  }

}
