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
#include <global_planner/global_planner.h>

using namespace std::chrono_literals;

namespace global_planner
{

GlobalPlanner::GlobalPlanner(const std::string& name)
    : Node(name) 
{
  clock_ = this->get_clock();
}

rclcpp_action::GoalResponse GlobalPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  rclcpp::Rate r(20);
  while (is_active(current_handle_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Wait for current handle to join");
    r.sleep();
  }
  current_handle_.reset();
  current_handle_ = goal_handle;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&GlobalPlanner::makePlan, this, std::placeholders::_1), goal_handle}.detach();
}
  
void GlobalPlanner::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d){
  
  static_ground_size_ = 0;
  perception_3d_ros_ = perception_3d;
  graph_ready_ = false;
  has_initialized_ = false;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  global_plan_result_ = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
  
  pcl_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_planground_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_map_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  kdtree_ground_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  kdtree_planground_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  
  declare_parameter("turning_weight", rclcpp::ParameterValue(0.1));
  this->get_parameter("turning_weight", turning_weight_);
  RCLCPP_INFO(this->get_logger(), "turning_weight: %.2f", turning_weight_);    

  declare_parameter("enable_detail_log", rclcpp::ParameterValue(false));
  this->get_parameter("enable_detail_log", enable_detail_log_);
  RCLCPP_INFO(this->get_logger(), "enable_detail_log: %d", enable_detail_log_);    

  declare_parameter("a_star_expanding_radius", rclcpp::ParameterValue(1.0));
  this->get_parameter("a_star_expanding_radius", a_star_expanding_radius_);
  RCLCPP_INFO(this->get_logger(), "a_star_expanding_radius: %.2f", a_star_expanding_radius_);    

  declare_parameter("use_pre_graph", rclcpp::ParameterValue(false));
  this->get_parameter("use_pre_graph", use_pre_graph_);
  RCLCPP_INFO(this->get_logger(), "use_pre_graph: %d", use_pre_graph_);    

  //@ Planground parameters
  planground_ready_ = false;
  declare_parameter("planground_search_radius", rclcpp::ParameterValue(1.0));
  this->get_parameter("planground_search_radius", planground_search_radius_);
  RCLCPP_INFO(this->get_logger(), "planground_search_radius: %.2f", planground_search_radius_);

  declare_parameter("planground_fallback_ratio", rclcpp::ParameterValue(0.3));
  this->get_parameter("planground_fallback_ratio", planground_fallback_ratio_);
  RCLCPP_INFO(this->get_logger(), "planground_fallback_ratio: %.2f", planground_fallback_ratio_);

  declare_parameter("planground_downsample_leaf_size", rclcpp::ParameterValue(0.2));
  this->get_parameter("planground_downsample_leaf_size", planground_downsample_leaf_size_);
  RCLCPP_INFO(this->get_logger(), "planground_downsample_leaf_size: %.2f", planground_downsample_leaf_size_);

  //@ Hybrid planning parameters (v24 - Ultra-Strong Planground Preference with Maximum Anti-Detour 加强版)
  declare_parameter("use_hybrid_planner", rclcpp::ParameterValue(true));
  this->get_parameter("use_hybrid_planner", use_hybrid_planner_);
  RCLCPP_INFO(this->get_logger(), "use_hybrid_planner: %d", use_hybrid_planner_);

  declare_parameter("hybrid_planground_bias", rclcpp::ParameterValue(15.0));
  this->get_parameter("hybrid_planground_bias", hybrid_planground_bias_);
  RCLCPP_INFO(this->get_logger(), "hybrid_planground_bias: %.2f", hybrid_planground_bias_);

  declare_parameter("hybrid_downsample_leaf_size", rclcpp::ParameterValue(0.15));
  this->get_parameter("hybrid_downsample_leaf_size", hybrid_downsample_leaf_size_);
  RCLCPP_INFO(this->get_logger(), "hybrid_downsample_leaf_size: %.2f", hybrid_downsample_leaf_size_);

  declare_parameter("hybrid_max_ground_bridge_length", rclcpp::ParameterValue(0.12));
  this->get_parameter("hybrid_max_ground_bridge_length", hybrid_max_ground_bridge_length_);
  RCLCPP_INFO(this->get_logger(), "hybrid_max_ground_bridge_length: %.2f", hybrid_max_ground_bridge_length_);

  declare_parameter("hybrid_max_ground_cost", rclcpp::ParameterValue(250.0));
  this->get_parameter("hybrid_max_ground_cost", hybrid_max_ground_cost_);
  RCLCPP_INFO(this->get_logger(), "hybrid_max_ground_cost: %.2f", hybrid_max_ground_cost_);

  declare_parameter("hybrid_min_ground_cost", rclcpp::ParameterValue(150.0));
  this->get_parameter("hybrid_min_ground_cost", hybrid_min_ground_cost_);
  RCLCPP_INFO(this->get_logger(), "hybrid_min_ground_cost: %.2f", hybrid_min_ground_cost_);

  declare_parameter("hybrid_detour_ratio_threshold", rclcpp::ParameterValue(4.0));
  this->get_parameter("hybrid_detour_ratio_threshold", hybrid_detour_ratio_threshold_);
  RCLCPP_INFO(this->get_logger(), "hybrid_detour_ratio_threshold: %.2f", hybrid_detour_ratio_threshold_);

  declare_parameter("hybrid_distance_balance_threshold", rclcpp::ParameterValue(1.5));
  this->get_parameter("hybrid_distance_balance_threshold", hybrid_distance_balance_threshold_);
  RCLCPP_INFO(this->get_logger(), "hybrid_distance_balance_threshold: %.2f", hybrid_distance_balance_threshold_);

  declare_parameter("hybrid_ground_path_length_penalty", rclcpp::ParameterValue(5.0));
  this->get_parameter("hybrid_ground_path_length_penalty", hybrid_ground_path_length_penalty_);
  RCLCPP_INFO(this->get_logger(), "hybrid_ground_path_length_penalty: %.2f", hybrid_ground_path_length_penalty_);

  declare_parameter("hybrid_detour_balance_factor_lower_bound", rclcpp::ParameterValue(0.5));
  this->get_parameter("hybrid_detour_balance_factor_lower_bound", hybrid_detour_balance_factor_lower_bound_);
  RCLCPP_INFO(this->get_logger(), "hybrid_detour_balance_factor_lower_bound: %.2f", hybrid_detour_balance_factor_lower_bound_);

  declare_parameter("hybrid_detour_balance_factor_upper_bound", rclcpp::ParameterValue(3.0));
  this->get_parameter("hybrid_detour_balance_factor_upper_bound", hybrid_detour_balance_factor_upper_bound_);
  RCLCPP_INFO(this->get_logger(), "hybrid_detour_balance_factor_upper_bound: %.2f", hybrid_detour_balance_factor_upper_bound_);

  //@ v25 Edge penalty parameters - penalize ground points near cloud edges
  declare_parameter("hybrid_edge_penalty_radius", rclcpp::ParameterValue(0.5));
  this->get_parameter("hybrid_edge_penalty_radius", hybrid_edge_penalty_radius_);
  RCLCPP_INFO(this->get_logger(), "hybrid_edge_penalty_radius: %.2f", hybrid_edge_penalty_radius_);

  declare_parameter("hybrid_edge_penalty_weight", rclcpp::ParameterValue(200.0));
  this->get_parameter("hybrid_edge_penalty_weight", hybrid_edge_penalty_weight_);
  RCLCPP_INFO(this->get_logger(), "hybrid_edge_penalty_weight: %.2f", hybrid_edge_penalty_weight_);

  declare_parameter("hybrid_edge_penalty_falloff_rate", rclcpp::ParameterValue(2.0));
  this->get_parameter("hybrid_edge_penalty_falloff_rate", hybrid_edge_penalty_falloff_rate_);
  RCLCPP_INFO(this->get_logger(), "hybrid_edge_penalty_falloff_rate: %.2f", hybrid_edge_penalty_falloff_rate_);

  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = action_server_group_;
  
  perception_3d_check_timer_ = this->create_wall_timer(500ms, std::bind(&GlobalPlanner::checkPerception3DThread, this), action_server_group_);
  
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1, 
      std::bind(&GlobalPlanner::cbClickedPoint, this, std::placeholders::_1), sub_options);

  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 1,
      std::bind(&GlobalPlanner::cbGoalPose, this, std::placeholders::_1), sub_options);
  
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
  pub_static_graph_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("static_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_weighted_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("weighted_ground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  //@ Subscribe to planground point cloud
  sub_planground_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/planground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&GlobalPlanner::cbPlanground, this, std::placeholders::_1), sub_options);

  //@Create action server
  this->action_server_global_planner_ = rclcpp_action::create_server<dddmr_sys_core::action::GetPlan>(
    this,
    "/get_plan",
    std::bind(&GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&GlobalPlanner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
  
}

GlobalPlanner::~GlobalPlanner(){

  //perception_3d_ros_.reset();
  tf2Buffer_.reset();
  tfl_.reset();
  a_star_planner_.reset();
  a_star_planner_pre_graph_.reset();
  action_server_global_planner_.reset();
  kdtree_ground_.reset();
  kdtree_map_.reset();
  pcl_ground_.reset();
  pcl_map_.reset();
}

void GlobalPlanner::checkPerception3DThread(){
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    return;
  }
  
  if(static_ground_size_!=perception_3d_ros_->getSharedDataPtr()->static_ground_size_){
    std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
    *pcl_ground_ = *(perception_3d_ros_->getSharedDataPtr()->pcl_ground_);
    global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    *pcl_map_ = *(perception_3d_ros_->getSharedDataPtr()->pcl_map_);
    kdtree_ground_->setInputCloud(pcl_ground_);
    kdtree_map_->setInputCloud(pcl_map_);
    static_graph_ = *perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_; //@ node weight
    RCLCPP_INFO(this->get_logger(), "Ground and Kd-tree ground have been received from perception_3d.");
    getStaticGraphFromPerception3D();
    static_ground_size_ = perception_3d_ros_->getSharedDataPtr()->static_ground_size_;
  }

}

void GlobalPlanner::cbPlanground(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);
  
  if(cloud->points.empty()){
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Received empty planground cloud");
    return;
  }

  std::unique_lock<std::mutex> lock(protect_kdtree_planground_);
  *pcl_planground_ = *cloud;
  kdtree_planground_->setInputCloud(pcl_planground_);
  planground_ready_ = true;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, 
    "Planground updated with %lu points", pcl_planground_->points.size());
}

void GlobalPlanner::cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal){
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Received clicked goal before static layer is ready");
    return;
  }

  geometry_msgs::msg::PoseStamped start, goal;

  goal.pose.position.x = clicked_goal->point.x;
  goal.pose.position.y = clicked_goal->point.y;
  goal.pose.position.z = clicked_goal->point.z;

  geometry_msgs::msg::TransformStamped transformStamped;

  try
  {
    transformStamped = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    start.pose.position.x = transformStamped.transform.translation.x;
    start.pose.position.y = transformStamped.transform.translation.y;
    start.pose.position.z = transformStamped.transform.translation.z;
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_WARN(this->get_logger(), "Failed to transform robot pose from %s to %s: %s",
      robot_frame_.c_str(), global_frame_.c_str(), e.what());
    return;
  }
  
  //====================================================================
  // Make deep copies of planground and ground clouds to avoid data races
  //====================================================================
  pcl::PointCloud<pcl::PointXYZI>::Ptr planground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr planground_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr ground_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  {
    std::unique_lock<std::mutex> lock(protect_kdtree_planground_);
    if(!planground_ready_ || pcl_planground_->points.empty()){
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, 
        "[Planground] Planground not available (ready=%d), cannot plan via clicked point.",
        planground_ready_);
      return;
    }
    *planground_cloud = *pcl_planground_;
    *planground_kdtree = *kdtree_planground_;
  }
  {
    std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
    if(!pcl_ground_->points.empty()){
      *ground_cloud = *pcl_ground_;
      *ground_kdtree = *kdtree_ground_;
    }
  }

  unsigned int start_id, goal_id;
  unsigned int ground_start_id, ground_goal_id;
  std::vector<unsigned int> path;
  std::vector<unsigned int> smoothed_path;
  std::vector<unsigned int> smoothed_path_2nd;
  nav_msgs::msg::Path ros_path;

  //====================================================================
  // Directly use hybrid planning for clicked point
  //====================================================================
  if(use_hybrid_planner_ && !ground_cloud->points.empty()){
    RCLCPP_INFO(this->get_logger(), "[Hybrid] Planning for clicked point using hybrid planner.");
    
    // Find start on planground first (always use robot's current position from tf)
    bool start_on_planground = getStartIDOnCloud(start, start_id, planground_cloud, planground_kdtree, planground_search_radius_);
    
    // Try to find start on ground cloud if not on planground
    bool start_on_ground = false;
    if(!start_on_planground){
      RCLCPP_WARN(this->get_logger(), "[Hybrid] Start not found on planground, trying on ground cloud.");
      start_on_ground = getStartIDOnCloud(start, ground_start_id, ground_cloud, ground_kdtree, 0.5);
      if(!start_on_ground){
        RCLCPP_WARN(this->get_logger(), "[Hybrid] Cannot find start on ground cloud either. Will use start pose directly.");
      }
    }
    
    // Now try to find goal on planground (use getGoalIDOnCloud to avoid overwriting start_id)
    bool goal_on_planground = getGoalIDOnCloud(goal, goal_id, planground_cloud, planground_kdtree, planground_search_radius_);
    
    if(!goal_on_planground){
      RCLCPP_WARN(this->get_logger(), "[Hybrid] Goal not on planground, trying with ground goal.");
      
      // Find goal on ground (use getGoalIDOnCloud to avoid overwriting start_id)
      if(!getGoalIDOnCloud(goal, ground_goal_id, ground_cloud, ground_kdtree, 0.5)){
        RCLCPP_WARN(this->get_logger(), "[Hybrid] Cannot find goal on ground for clicked point.");
        return;
      }
      
      // Use hybrid planner with ground goal
      Hybrid_A_Star hybrid_planner(planground_cloud, ground_cloud, perception_3d_ros_, a_star_expanding_radius_);
      hybrid_planner.setDetourRatioThreshold(hybrid_detour_ratio_threshold_);
      hybrid_planner.setupPlangroundBias(hybrid_planground_bias_);
      hybrid_planner.setupTurningWeight(turning_weight_);
      hybrid_planner.setMaxGroundBridgeLength(hybrid_max_ground_bridge_length_);
      hybrid_planner.setMaxGroundCost(hybrid_max_ground_cost_);
      hybrid_planner.setMinGroundCost(hybrid_min_ground_cost_);
      hybrid_planner.setDownsampleLeafSize(hybrid_downsample_leaf_size_);
      hybrid_planner.setDistanceBalanceThreshold(hybrid_distance_balance_threshold_);
      hybrid_planner.setGroundPathLengthPenalty(hybrid_ground_path_length_penalty_);
      hybrid_planner.setDetourBalanceFactorLowerBound(hybrid_detour_balance_factor_lower_bound_);
      hybrid_planner.setDetourBalanceFactorUpperBound(hybrid_detour_balance_factor_upper_bound_);
      hybrid_planner.setEdgePenaltyParams(hybrid_edge_penalty_radius_, hybrid_edge_penalty_weight_, hybrid_edge_penalty_falloff_rate_);
      
      if(start_on_planground){
        // Start is on planground, goal is on ground - use getPathWithStartPoseAndGroundGoal
        RCLCPP_INFO(this->get_logger(), "[Hybrid] Start on planground, goal on ground, using getPathWithStartPoseAndGroundGoal.");
        hybrid_planner.getPathWithStartPoseAndGroundGoal(start, ground_goal_id, path);
      }

      else{
        // Both start and goal are NOT on planground
        // Use getPathWithStartPoseAndGroundGoal which adds the robot's actual position
        // to the hybrid cloud as the start point
        RCLCPP_INFO(this->get_logger(), "[Hybrid] Both start and goal not on planground, using getPathWithStartPoseAndGroundGoal.");
        hybrid_planner.getPathWithStartPoseAndGroundGoal(start, ground_goal_id, path);
      }
      
      if(path.empty()){
        RCLCPP_WARN(this->get_logger(), "[Hybrid] No path found via hybrid planning for clicked point.");
        return;
      }
      
      // Build ROS path from hybrid cloud points with smoothing
      // Use smoothPathToRosPath which properly interpolates Z using Catmull-Rom spline
      pcl::PointCloud<pcl::PointXYZI>::Ptr hybrid_cloud = hybrid_planner.getHybridCloud();
      
      hybrid_planner.smoothPathToRosPath(path, hybrid_cloud, ros_path, goal, global_frame_, 0.2);
      
      pub_path_->publish(ros_path);
      RCLCPP_INFO(this->get_logger(), "[Hybrid] Hybrid path found for clicked point (ground goal): %lu nodes, smoothed: %lu points", path.size(), ros_path.poses.size());
      return;
    }
    
    // Both start and goal are on planground, use hybrid planner with start pose
    // This ensures the path starts exactly from the robot's current position (from tf)
    // rather than from the nearest planground point, which may be offset
    Hybrid_A_Star hybrid_planner(planground_cloud, ground_cloud, perception_3d_ros_, a_star_expanding_radius_);
    hybrid_planner.setDetourRatioThreshold(hybrid_detour_ratio_threshold_);
    hybrid_planner.setupPlangroundBias(hybrid_planground_bias_);
    hybrid_planner.setupTurningWeight(turning_weight_);
    hybrid_planner.setMaxGroundBridgeLength(hybrid_max_ground_bridge_length_);
    hybrid_planner.setMaxGroundCost(hybrid_max_ground_cost_);
    hybrid_planner.setMinGroundCost(hybrid_min_ground_cost_);
    hybrid_planner.setDownsampleLeafSize(hybrid_downsample_leaf_size_);
    hybrid_planner.setDistanceBalanceThreshold(hybrid_distance_balance_threshold_);
    hybrid_planner.setGroundPathLengthPenalty(hybrid_ground_path_length_penalty_);
    hybrid_planner.setDetourBalanceFactorLowerBound(hybrid_detour_balance_factor_lower_bound_);
    hybrid_planner.setDetourBalanceFactorUpperBound(hybrid_detour_balance_factor_upper_bound_);
    hybrid_planner.setEdgePenaltyParams(hybrid_edge_penalty_radius_, hybrid_edge_penalty_weight_, hybrid_edge_penalty_falloff_rate_);
    
    // Use getPathWithStartPose to ensure the path starts from the robot's actual position

    // This works for both cases:
    // - Start on planground: path starts from robot position, goal is planground index
    // - Start on ground: path starts from robot position, goal is planground index
    //   getPathWithStartPose adds the start pose as a dedicated point in the hybrid cloud
    //   and builds the corridor from start_pose to goal_pose, so ground points near the
    //   robot's actual position will be included in the corridor
    hybrid_planner.getPathWithStartPose(start, goal_id, path);
    
    if(path.empty()){
      RCLCPP_WARN(this->get_logger(), "[Hybrid] No path found via hybrid planning for clicked point.");
      return;
    }
    
    // Determine which cloud to use for building ROS path
    // If getPath returned a planground-only path (no hybrid needed), use planground cloud directly
    pcl::PointCloud<pcl::PointXYZI>::Ptr hybrid_cloud = hybrid_planner.getHybridCloud();
    bool use_hybrid_cloud = (hybrid_cloud && !hybrid_cloud->empty() && hybrid_cloud->size() > planground_cloud->size());
    
    // Use smoothPathToRosPath for Catmull-Rom spline smoothing
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_for_smoothing = use_hybrid_cloud ? hybrid_cloud : planground_cloud;
    hybrid_planner.smoothPathToRosPath(path, cloud_for_smoothing, ros_path, goal, global_frame_, 0.2);
    ros_path.header.stamp = clock_->now();
    
    pub_path_->publish(ros_path);
    RCLCPP_INFO(this->get_logger(), "[Hybrid] Hybrid path found for clicked point: %lu nodes, smoothed: %lu poses", 
      path.size(), ros_path.poses.size());
    return;
  }
  
  //====================================================================
  // Fallback: plan on planground only (if hybrid planner is disabled or ground is empty)
  //====================================================================
  {
    if(!getStartGoalIDOnCloud(start, goal, start_id, goal_id, planground_cloud, planground_kdtree, planground_search_radius_)){
      RCLCPP_WARN(this->get_logger(), "[Planground] Cannot find start/goal on planground for clicked point.");
      return;
    }

    if(!use_pre_graph_){
      auto a_star = std::make_shared<A_Star_on_Graph>(planground_cloud, perception_3d_ros_, a_star_expanding_radius_);
      a_star->setupTurningWeight(turning_weight_);
      a_star->getPath(start_id, goal_id, path);
    }
    else{
      auto a_star_pre = std::make_shared<A_Star_on_PreGraph>(planground_cloud, static_graph_, perception_3d_ros_, a_star_expanding_radius_);
      a_star_pre->setupTurningWeight(turning_weight_);
      a_star_pre->getPath(start_id, goal_id, path);
    }

    if(path.empty()){
      RCLCPP_WARN(this->get_logger(), "[Planground] No path found on planground from: %u to %u", start_id, goal_id);
      return;
    }

    ros_path.header.frame_id = global_frame_;
    ros_path.header.stamp = clock_->now();
    for(auto it=0; it<path.size(); it++){
      geometry_msgs::msg::PoseStamped pst;
      pst.header = ros_path.header;
      pst.pose.position.x = planground_cloud->points[path[it]].x;
      pst.pose.position.y = planground_cloud->points[path[it]].y;
      pst.pose.position.z = planground_cloud->points[path[it]].z;
      
      if(it < path.size()-1){
        double vx = planground_cloud->points[path[it+1]].x - planground_cloud->points[path[it]].x;
        double vy = planground_cloud->points[path[it+1]].y - planground_cloud->points[path[it]].y;
        double vz = planground_cloud->points[path[it+1]].z - planground_cloud->points[path[it]].z;
        if(vz != 0){
          double unit = sqrt(vx*vx + vy*vy + vz*vz);
          tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);
          tf2::Vector3 up_vector(1.0, 0.0, 0.0);
          tf2::Vector3 right_vector = axis_vector.cross(up_vector);
          right_vector.normalized();
          tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
          q.normalize();
          pst.pose.orientation.x = q.getX();
          pst.pose.orientation.y = q.getY();
          pst.pose.orientation.z = q.getZ();
          pst.pose.orientation.w = q.getW();
        }
        else{
          double yaw = atan2(vy, vx);
          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, yaw);
          pst.pose.orientation.x = q.getX();
          pst.pose.orientation.y = q.getY();
          pst.pose.orientation.z = q.getZ();
          pst.pose.orientation.w = q.getW();
        }
      }
      ros_path.poses.push_back(pst);
    }
    ros_path.poses.push_back(goal);
    
    pub_path_->publish(ros_path);
    RCLCPP_INFO(this->get_logger(), "[Planground] Path found for clicked point: %lu nodes", path.size());
    return;
  }
}

void GlobalPlanner::cbGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose){
  auto clicked_goal = std::make_shared<geometry_msgs::msg::PointStamped>();
  clicked_goal->header = goal_pose->header;
  clicked_goal->point.x = goal_pose->pose.position.x;
  clicked_goal->point.y = goal_pose->pose.position.y;
  clicked_goal->point.z = goal_pose->pose.position.z;
  cbClickedPoint(clicked_goal);
}

bool GlobalPlanner::getStartIDOnCloud(const geometry_msgs::msg::PoseStamped& start, unsigned int& start_id,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr& kdtree, double search_radius){
  
  pcl::PointXYZI searchPoint;
  searchPoint.x = start.pose.position.x;
  searchPoint.y = start.pose.position.y;
  searchPoint.z = start.pose.position.z;
  
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  
  if(kdtree->radiusSearch(searchPoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
    start_id = pointIdxRadiusSearch[0];
    RCLCPP_INFO(this->get_logger(), "Start found on cloud at index %u", start_id);
    return true;
  }
  
  RCLCPP_WARN(this->get_logger(), "Start not found on cloud within radius %.2f", search_radius);
  return false;
}

bool GlobalPlanner::getGoalIDOnCloud(const geometry_msgs::msg::PoseStamped& goal, unsigned int& goal_id,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr& kdtree, double search_radius){
  
  pcl::PointXYZI searchPoint;
  searchPoint.x = goal.pose.position.x;
  searchPoint.y = goal.pose.position.y;
  searchPoint.z = goal.pose.position.z;
  
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  
  if(kdtree->radiusSearch(searchPoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
    goal_id = pointIdxRadiusSearch[0];
    RCLCPP_INFO(this->get_logger(), "Goal found on cloud at index %u", goal_id);
    return true;
  }
  
  RCLCPP_WARN(this->get_logger(), "Goal not found on cloud within radius %.2f", search_radius);
  return false;
}

bool GlobalPlanner::getStartGoalIDOnCloud(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
    unsigned int& start_id, unsigned int& goal_id,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr& kdtree, double search_radius){
  
  pcl::PointXYZI searchPoint;
  searchPoint.x = start.pose.position.x;
  searchPoint.y = start.pose.position.y;
  searchPoint.z = start.pose.position.z;
  
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  
  if(kdtree->radiusSearch(searchPoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
    start_id = pointIdxRadiusSearch[0];
    RCLCPP_INFO(this->get_logger(), "Start found on cloud at index %u", start_id);
  }
  else{
    RCLCPP_WARN(this->get_logger(), "Start not found on cloud within radius %.2f", search_radius);
    return false;
  }
  
  searchPoint.x = goal.pose.position.x;
  searchPoint.y = goal.pose.position.y;
  searchPoint.z = goal.pose.position.z;
  
  if(kdtree->radiusSearch(searchPoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
    goal_id = pointIdxRadiusSearch[0];
    RCLCPP_INFO(this->get_logger(), "Goal found on cloud at index %u", goal_id);
  }
  else{
    RCLCPP_WARN(this->get_logger(), "Goal not found on cloud within radius %.2f", search_radius);
    return false;
  }
  
  return true;
}

void GlobalPlanner::getStaticGraphFromPerception3D(){
  
  if(static_graph_.getSize() == 0){
    RCLCPP_WARN(this->get_logger(), "Static graph is empty from perception_3d.");
    return;
  }
  
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;
  graph_t* graph_ptr = static_graph_.getGraphPtr();
  for(auto it = graph_ptr->begin(); it != graph_ptr->end(); ++it){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = clock_->now();
    marker.ns = "static_graph";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // Use the node id as a placeholder - StaticGraph stores edges, not positions
    // The actual positions come from the point cloud
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
  }
  pub_static_graph_->publish(marker_array);
  RCLCPP_INFO(this->get_logger(), "Published static graph with %lu markers", static_graph_.getSize());
}

void GlobalPlanner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){
  
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
  
  //====================================================================
  // Make deep copies of planground and ground clouds to avoid data races
  //====================================================================
  pcl::PointCloud<pcl::PointXYZI>::Ptr planground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr planground_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr ground_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  {
    std::unique_lock<std::mutex> lock(protect_kdtree_planground_);
    if(!planground_ready_ || pcl_planground_->points.empty()){
      RCLCPP_WARN(this->get_logger(), "[Planground] Planground not available (ready=%d), cannot plan.",
        planground_ready_);
      goal_handle->abort(result);
      return;
    }
    *planground_cloud = *pcl_planground_;
    *planground_kdtree = *kdtree_planground_;
  }
  {
    std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
    if(!pcl_ground_->points.empty()){
      *ground_cloud = *pcl_ground_;
      *ground_kdtree = *kdtree_ground_;
    }
  }
  
  geometry_msgs::msg::PoseStamped start = goal->start;
  geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
  
  unsigned int start_id, goal_id;
  unsigned int ground_start_id, ground_goal_id;
  std::vector<unsigned int> path;
  std::vector<unsigned int> smoothed_path;
  std::vector<unsigned int> smoothed_path_2nd;
  nav_msgs::msg::Path ros_path;
  
  //====================================================================
  // Use hybrid planning if enabled and ground is available
  //====================================================================
  if(use_hybrid_planner_ && !ground_cloud->points.empty()){
    RCLCPP_INFO(this->get_logger(), "[Hybrid] Planning using hybrid planner.");
    
    // Find start on planground
    bool start_on_planground = getStartIDOnCloud(start, start_id, planground_cloud, planground_kdtree, planground_search_radius_);
    
    // Try to find start on ground cloud if not on planground
    bool start_on_ground = false;
    if(!start_on_planground){
      RCLCPP_WARN(this->get_logger(), "[Hybrid] Start not found on planground, trying on ground cloud.");
      start_on_ground = getStartIDOnCloud(start, ground_start_id, ground_cloud, ground_kdtree, 0.5);
      if(!start_on_ground){
        RCLCPP_WARN(this->get_logger(), "[Hybrid] Cannot find start on ground cloud either. Will use start pose directly.");
      }
    }
    
    // Find goal on planground
    bool goal_on_planground = getGoalIDOnCloud(goal_pose, goal_id, planground_cloud, planground_kdtree, planground_search_radius_);
    
    if(!goal_on_planground){
      RCLCPP_WARN(this->get_logger(), "[Hybrid] Goal not on planground, trying with ground goal.");
      
      // Find goal on ground
      if(!getGoalIDOnCloud(goal_pose, ground_goal_id, ground_cloud, ground_kdtree, 0.5)){
        RCLCPP_WARN(this->get_logger(), "[Hybrid] Cannot find goal on ground.");
        goal_handle->abort(result);
        return;
      }
      
      // Use hybrid planner with ground goal
      Hybrid_A_Star hybrid_planner(planground_cloud, ground_cloud, perception_3d_ros_, a_star_expanding_radius_);
      hybrid_planner.setDetourRatioThreshold(hybrid_detour_ratio_threshold_);
      hybrid_planner.setupPlangroundBias(hybrid_planground_bias_);
      hybrid_planner.setupTurningWeight(turning_weight_);
      hybrid_planner.setMaxGroundBridgeLength(hybrid_max_ground_bridge_length_);
      hybrid_planner.setMaxGroundCost(hybrid_max_ground_cost_);
      hybrid_planner.setMinGroundCost(hybrid_min_ground_cost_);
      hybrid_planner.setDownsampleLeafSize(hybrid_downsample_leaf_size_);
      hybrid_planner.setDistanceBalanceThreshold(hybrid_distance_balance_threshold_);
      hybrid_planner.setGroundPathLengthPenalty(hybrid_ground_path_length_penalty_);
      hybrid_planner.setDetourBalanceFactorLowerBound(hybrid_detour_balance_factor_lower_bound_);
      hybrid_planner.setDetourBalanceFactorUpperBound(hybrid_detour_balance_factor_upper_bound_);
      hybrid_planner.setEdgePenaltyParams(hybrid_edge_penalty_radius_, hybrid_edge_penalty_weight_, hybrid_edge_penalty_falloff_rate_);
      
      if(start_on_planground){
        RCLCPP_INFO(this->get_logger(), "[Hybrid] Start on planground, goal on ground, using getPathWithStartPoseAndGroundGoal.");
        hybrid_planner.getPathWithStartPoseAndGroundGoal(start, ground_goal_id, path);
      }
      else{
        RCLCPP_INFO(this->get_logger(), "[Hybrid] Both start and goal not on planground, using getPathWithStartPoseAndGroundGoal.");
        hybrid_planner.getPathWithStartPoseAndGroundGoal(start, ground_goal_id, path);
      }
      
      if(path.empty()){
        RCLCPP_WARN(this->get_logger(), "[Hybrid] No path found via hybrid planning.");
        goal_handle->abort(result);
        return;
      }
      
      // Build ROS path from hybrid cloud points with smoothing
      pcl::PointCloud<pcl::PointXYZI>::Ptr hybrid_cloud = hybrid_planner.getHybridCloud();
      
      // Use smoothPathToRosPath for Catmull-Rom spline smoothing
      hybrid_planner.smoothPathToRosPath(path, hybrid_cloud, ros_path, goal_pose, global_frame_, 0.2);
      ros_path.header.stamp = clock_->now();
      
      result->path = ros_path;
      pub_path_->publish(ros_path);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "[Hybrid] Hybrid path found (ground goal): %lu nodes, smoothed: %lu poses", 
        path.size(), ros_path.poses.size());
      return;
    }
    
    // Both start and goal are on planground, use hybrid planner with start pose
    Hybrid_A_Star hybrid_planner(planground_cloud, ground_cloud, perception_3d_ros_, a_star_expanding_radius_);
    hybrid_planner.setDetourRatioThreshold(hybrid_detour_ratio_threshold_);
    hybrid_planner.setupPlangroundBias(hybrid_planground_bias_);
    hybrid_planner.setupTurningWeight(turning_weight_);
    hybrid_planner.setMaxGroundBridgeLength(hybrid_max_ground_bridge_length_);
    hybrid_planner.setMaxGroundCost(hybrid_max_ground_cost_);
    hybrid_planner.setMinGroundCost(hybrid_min_ground_cost_);
    hybrid_planner.setDownsampleLeafSize(hybrid_downsample_leaf_size_);
    hybrid_planner.setDistanceBalanceThreshold(hybrid_distance_balance_threshold_);
    hybrid_planner.setGroundPathLengthPenalty(hybrid_ground_path_length_penalty_);
    hybrid_planner.setDetourBalanceFactorLowerBound(hybrid_detour_balance_factor_lower_bound_);
    hybrid_planner.setDetourBalanceFactorUpperBound(hybrid_detour_balance_factor_upper_bound_);
    hybrid_planner.setEdgePenaltyParams(hybrid_edge_penalty_radius_, hybrid_edge_penalty_weight_, hybrid_edge_penalty_falloff_rate_);
    
    hybrid_planner.getPathWithStartPose(start, goal_id, path);

    
    if(path.empty()){
      RCLCPP_WARN(this->get_logger(), "[Hybrid] No path found via hybrid planning.");
      goal_handle->abort(result);
      return;
    }
    
    // Determine which cloud to use for building ROS path
    pcl::PointCloud<pcl::PointXYZI>::Ptr hybrid_cloud = hybrid_planner.getHybridCloud();
    bool use_hybrid_cloud = (hybrid_cloud && !hybrid_cloud->empty() && hybrid_cloud->size() > planground_cloud->size());
    
    // Use smoothPathToRosPath for Catmull-Rom spline smoothing
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_for_smoothing = use_hybrid_cloud ? hybrid_cloud : planground_cloud;
    hybrid_planner.smoothPathToRosPath(path, cloud_for_smoothing, ros_path, goal_pose, global_frame_, 0.2);
    ros_path.header.stamp = clock_->now();
    
    result->path = ros_path;
    pub_path_->publish(ros_path);
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "[Hybrid] Hybrid path found: %lu nodes, smoothed: %lu poses", 
      path.size(), ros_path.poses.size());
    return;
  }
  
  //====================================================================
  // Fallback: plan on planground only
  //====================================================================
  {
    if(!getStartGoalIDOnCloud(start, goal_pose, start_id, goal_id, planground_cloud, planground_kdtree, planground_search_radius_)){
      RCLCPP_WARN(this->get_logger(), "[Planground] Cannot find start/goal on planground.");
      goal_handle->abort(result);
      return;
    }

    if(!use_pre_graph_){
      auto a_star = std::make_shared<A_Star_on_Graph>(planground_cloud, perception_3d_ros_, a_star_expanding_radius_);
      a_star->setupTurningWeight(turning_weight_);
      a_star->getPath(start_id, goal_id, path);
    }
    else{
      auto a_star_pre = std::make_shared<A_Star_on_PreGraph>(planground_cloud, static_graph_, perception_3d_ros_, a_star_expanding_radius_);
      a_star_pre->setupTurningWeight(turning_weight_);
      a_star_pre->getPath(start_id, goal_id, path);
    }

    if(path.empty()){
      RCLCPP_WARN(this->get_logger(), "[Planground] No path found on planground from: %u to %u", start_id, goal_id);
      goal_handle->abort(result);
      return;
    }

    // Use postSmoothPath for line-of-sight smoothing on planground path
    std::vector<unsigned int> smoothed_planground_path;
    postSmoothPath(path, smoothed_planground_path, planground_cloud);
    
    ros_path.header.frame_id = global_frame_;
    ros_path.header.stamp = clock_->now();
    for(auto it=0; it<smoothed_planground_path.size(); it++){
      geometry_msgs::msg::PoseStamped pst;
      pst.header = ros_path.header;
      pst.pose.position.x = planground_cloud->points[smoothed_planground_path[it]].x;
      pst.pose.position.y = planground_cloud->points[smoothed_planground_path[it]].y;
      pst.pose.position.z = planground_cloud->points[smoothed_planground_path[it]].z;
      
      if(it < smoothed_planground_path.size()-1){
        double vx = planground_cloud->points[smoothed_planground_path[it+1]].x - planground_cloud->points[smoothed_planground_path[it]].x;
        double vy = planground_cloud->points[smoothed_planground_path[it+1]].y - planground_cloud->points[smoothed_planground_path[it]].y;
        double vz = planground_cloud->points[smoothed_planground_path[it+1]].z - planground_cloud->points[smoothed_planground_path[it]].z;
        if(vz != 0){
          double unit = sqrt(vx*vx + vy*vy + vz*vz);
          tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);
          tf2::Vector3 up_vector(1.0, 0.0, 0.0);
          tf2::Vector3 right_vector = axis_vector.cross(up_vector);
          right_vector.normalized();
          tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
          q.normalize();
          pst.pose.orientation.x = q.getX();
          pst.pose.orientation.y = q.getY();
          pst.pose.orientation.z = q.getZ();
          pst.pose.orientation.w = q.getW();
        }
        else{
          double yaw = atan2(vy, vx);
          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, yaw);
          pst.pose.orientation.x = q.getX();
          pst.pose.orientation.y = q.getY();
          pst.pose.orientation.z = q.getZ();
          pst.pose.orientation.w = q.getW();
        }
      }
      ros_path.poses.push_back(pst);
    }
    ros_path.poses.push_back(goal_pose);
    
    result->path = ros_path;
    pub_path_->publish(ros_path);
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "[Planground] Path found: %lu nodes, smoothed: %lu nodes", 
      path.size(), smoothed_planground_path.size());
    return;
  }
}

void GlobalPlanner::postSmoothPath(std::vector<unsigned int>& path, std::vector<unsigned int>& smoothed_path,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
  
  if(path.size() < 3){
    smoothed_path = path;
    return;
  }
  
  // Simple line-of-sight smoothing using kdtree for obstacle checking
  smoothed_path.clear();
  smoothed_path.push_back(path[0]);
  
  // Build kdtree for the cloud to enable efficient nearest neighbor search
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);
  
  unsigned int current = 0;
  for(unsigned int i = 1; i < path.size() - 1; i++){
    // Check if there is direct line of sight from current to i+1
    pcl::PointXYZI p1 = cloud->points[path[current]];
    pcl::PointXYZI p2 = cloud->points[path[i+1]];
    
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    double dist = sqrt(dx*dx + dy*dy + dz*dz);
    
    // Sample points along the line and check if they stay close to the cloud
    bool has_obstacle = false;
    int num_samples = std::max(10, (int)(dist / 0.2));
    for(int s = 1; s < num_samples; s++){
      double ratio = (double)s / num_samples;
      pcl::PointXYZI sample;
      sample.x = p1.x + dx * ratio;
      sample.y = p1.y + dy * ratio;
      sample.z = p1.z + dz * ratio;
      
      // Check if sample point is close to any cloud point
      std::vector<int> idx(1);
      std::vector<float> dists(1);
      if(kdtree.nearestKSearch(sample, 1, idx, dists) > 0){
        // If the nearest cloud point is too far from the sample line,
        // there might be an obstacle (the path is deviating from the cloud)
        if(dists[0] > 0.5){
          has_obstacle = true;
          break;
        }
      }
    }
    
    if(!has_obstacle){
      // Skip intermediate points - direct line of sight
      continue;
    }
    else{
      smoothed_path.push_back(path[i]);
      current = i;
    }
  }
  smoothed_path.push_back(path[path.size()-1]);
  
  RCLCPP_DEBUG(this->get_logger(), "postSmoothPath: %lu -> %lu nodes", path.size(), smoothed_path.size());
}

nav_msgs::msg::Path GlobalPlanner::makeROSPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal){
  
  nav_msgs::msg::Path ros_path;
  
  //====================================================================
  // Make deep copies of planground and ground clouds to avoid data races
  //====================================================================
  pcl::PointCloud<pcl::PointXYZI>::Ptr planground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr planground_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr ground_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  {
    std::unique_lock<std::mutex> lock(protect_kdtree_planground_);
    if(!planground_ready_ || pcl_planground_->points.empty()){
      RCLCPP_WARN(this->get_logger(), "[makeROSPlan] Planground not available (ready=%d).", planground_ready_);
      return ros_path;
    }
    *planground_cloud = *pcl_planground_;
    *planground_kdtree = *kdtree_planground_;
  }
  {
    std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
    if(!pcl_ground_->points.empty()){
      *ground_cloud = *pcl_ground_;
      *ground_kdtree = *kdtree_ground_;
    }
  }
  
  unsigned int start_id, goal_id;
  unsigned int ground_start_id, ground_goal_id;
  std::vector<unsigned int> path;
  
  //====================================================================
  // Use hybrid planning if enabled and ground is available
  //====================================================================
  if(use_hybrid_planner_ && !ground_cloud->points.empty()){
    RCLCPP_INFO(this->get_logger(), "[makeROSPlan] Planning using hybrid planner.");
    
    // Find start on planground
    bool start_on_planground = getStartIDOnCloud(start, start_id, planground_cloud, planground_kdtree, planground_search_radius_);
    
    // Try to find start on ground cloud if not on planground
    bool start_on_ground = false;
    if(!start_on_planground){
      RCLCPP_WARN(this->get_logger(), "[makeROSPlan] Start not found on planground, trying on ground cloud.");
      start_on_ground = getStartIDOnCloud(start, ground_start_id, ground_cloud, ground_kdtree, 0.5);
    }
    
    // Find goal on planground
    bool goal_on_planground = getGoalIDOnCloud(goal, goal_id, planground_cloud, planground_kdtree, planground_search_radius_);
    
    if(!goal_on_planground){
      RCLCPP_WARN(this->get_logger(), "[makeROSPlan] Goal not on planground, trying with ground goal.");
      
      if(!getGoalIDOnCloud(goal, ground_goal_id, ground_cloud, ground_kdtree, 0.5)){
        RCLCPP_WARN(this->get_logger(), "[makeROSPlan] Cannot find goal on ground.");
        return ros_path;
      }
      
      Hybrid_A_Star hybrid_planner(planground_cloud, ground_cloud, perception_3d_ros_, a_star_expanding_radius_);
      hybrid_planner.setDetourRatioThreshold(hybrid_detour_ratio_threshold_);
      hybrid_planner.setupPlangroundBias(hybrid_planground_bias_);
      hybrid_planner.setupTurningWeight(turning_weight_);
      hybrid_planner.setMaxGroundBridgeLength(hybrid_max_ground_bridge_length_);
      hybrid_planner.setMaxGroundCost(hybrid_max_ground_cost_);
      hybrid_planner.setMinGroundCost(hybrid_min_ground_cost_);
      hybrid_planner.setDownsampleLeafSize(hybrid_downsample_leaf_size_);
      hybrid_planner.setDistanceBalanceThreshold(hybrid_distance_balance_threshold_);
      hybrid_planner.setGroundPathLengthPenalty(hybrid_ground_path_length_penalty_);
      hybrid_planner.setDetourBalanceFactorLowerBound(hybrid_detour_balance_factor_lower_bound_);
      hybrid_planner.setDetourBalanceFactorUpperBound(hybrid_detour_balance_factor_upper_bound_);
      hybrid_planner.setEdgePenaltyParams(hybrid_edge_penalty_radius_, hybrid_edge_penalty_weight_, hybrid_edge_penalty_falloff_rate_);
      
      if(start_on_planground){
        hybrid_planner.getPathWithStartPoseAndGroundGoal(start, ground_goal_id, path);
      }
      else{
        hybrid_planner.getPathWithStartPoseAndGroundGoal(start, ground_goal_id, path);
      }
      
      if(path.empty()){
        RCLCPP_WARN(this->get_logger(), "[makeROSPlan] No path found via hybrid planning.");
        return ros_path;
      }
      
      pcl::PointCloud<pcl::PointXYZI>::Ptr hybrid_cloud = hybrid_planner.getHybridCloud();
      
      // Use smoothPathToRosPath for Catmull-Rom spline smoothing
      hybrid_planner.smoothPathToRosPath(path, hybrid_cloud, ros_path, goal, global_frame_, 0.2);
      ros_path.header.stamp = clock_->now();
      
      RCLCPP_INFO(this->get_logger(), "[makeROSPlan] Hybrid path found (ground goal): %lu nodes, smoothed: %lu poses", 
        path.size(), ros_path.poses.size());
      return ros_path;
    }
    
    // Both start and goal are on planground
    Hybrid_A_Star hybrid_planner(planground_cloud, ground_cloud, perception_3d_ros_, a_star_expanding_radius_);
    hybrid_planner.setDetourRatioThreshold(hybrid_detour_ratio_threshold_);
    hybrid_planner.setupPlangroundBias(hybrid_planground_bias_);
    hybrid_planner.setupTurningWeight(turning_weight_);
    hybrid_planner.setMaxGroundBridgeLength(hybrid_max_ground_bridge_length_);
    hybrid_planner.setMaxGroundCost(hybrid_max_ground_cost_);
    hybrid_planner.setMinGroundCost(hybrid_min_ground_cost_);
    hybrid_planner.setDownsampleLeafSize(hybrid_downsample_leaf_size_);
    hybrid_planner.setDistanceBalanceThreshold(hybrid_distance_balance_threshold_);
    hybrid_planner.setGroundPathLengthPenalty(hybrid_ground_path_length_penalty_);
    hybrid_planner.setDetourBalanceFactorLowerBound(hybrid_detour_balance_factor_lower_bound_);
    hybrid_planner.setDetourBalanceFactorUpperBound(hybrid_detour_balance_factor_upper_bound_);
    hybrid_planner.setEdgePenaltyParams(hybrid_edge_penalty_radius_, hybrid_edge_penalty_weight_, hybrid_edge_penalty_falloff_rate_);
    
    hybrid_planner.getPathWithStartPose(start, goal_id, path);

    
    if(path.empty()){
      RCLCPP_WARN(this->get_logger(), "[makeROSPlan] No path found via hybrid planning.");
      return ros_path;
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr hybrid_cloud = hybrid_planner.getHybridCloud();
    bool use_hybrid_cloud = (hybrid_cloud && !hybrid_cloud->empty() && hybrid_cloud->size() > planground_cloud->size());
    
    // Use smoothPathToRosPath for Catmull-Rom spline smoothing
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_for_smoothing = use_hybrid_cloud ? hybrid_cloud : planground_cloud;
    hybrid_planner.smoothPathToRosPath(path, cloud_for_smoothing, ros_path, goal, global_frame_, 0.2);
    ros_path.header.stamp = clock_->now();
    
    RCLCPP_INFO(this->get_logger(), "[makeROSPlan] Hybrid path found: %lu nodes, smoothed: %lu poses", 
      path.size(), ros_path.poses.size());
    return ros_path;
  }
  
  //====================================================================
  // Fallback: plan on planground only
  //====================================================================
  {
    if(!getStartGoalIDOnCloud(start, goal, start_id, goal_id, planground_cloud, planground_kdtree, planground_search_radius_)){
      RCLCPP_WARN(this->get_logger(), "[makeROSPlan] Cannot find start/goal on planground.");
      return ros_path;
    }

    if(!use_pre_graph_){
      auto a_star = std::make_shared<A_Star_on_Graph>(planground_cloud, perception_3d_ros_, a_star_expanding_radius_);
      a_star->setupTurningWeight(turning_weight_);
      a_star->getPath(start_id, goal_id, path);
    }
    else{
      auto a_star_pre = std::make_shared<A_Star_on_PreGraph>(planground_cloud, static_graph_, perception_3d_ros_, a_star_expanding_radius_);
      a_star_pre->setupTurningWeight(turning_weight_);
      a_star_pre->getPath(start_id, goal_id, path);
    }

    if(path.empty()){
      RCLCPP_WARN(this->get_logger(), "[makeROSPlan] No path found on planground from: %u to %u", start_id, goal_id);
      return ros_path;
    }

    // Use postSmoothPath for line-of-sight smoothing on planground path
    std::vector<unsigned int> smoothed_planground_path;
    postSmoothPath(path, smoothed_planground_path, planground_cloud);
    
    ros_path.header.frame_id = global_frame_;
    ros_path.header.stamp = clock_->now();
    for(auto it=0; it<smoothed_planground_path.size(); it++){
      geometry_msgs::msg::PoseStamped pst;
      pst.header = ros_path.header;
      pst.pose.position.x = planground_cloud->points[smoothed_planground_path[it]].x;
      pst.pose.position.y = planground_cloud->points[smoothed_planground_path[it]].y;
      pst.pose.position.z = planground_cloud->points[smoothed_planground_path[it]].z;
      
      if(it < smoothed_planground_path.size()-1){
        double vx = planground_cloud->points[smoothed_planground_path[it+1]].x - planground_cloud->points[smoothed_planground_path[it]].x;
        double vy = planground_cloud->points[smoothed_planground_path[it+1]].y - planground_cloud->points[smoothed_planground_path[it]].y;
        double vz = planground_cloud->points[smoothed_planground_path[it+1]].z - planground_cloud->points[smoothed_planground_path[it]].z;
        if(vz != 0){
          double unit = sqrt(vx*vx + vy*vy + vz*vz);
          tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);
          tf2::Vector3 up_vector(1.0, 0.0, 0.0);
          tf2::Vector3 right_vector = axis_vector.cross(up_vector);
          right_vector.normalized();
          tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
          q.normalize();
          pst.pose.orientation.x = q.getX();
          pst.pose.orientation.y = q.getY();
          pst.pose.orientation.z = q.getZ();
          pst.pose.orientation.w = q.getW();
        }
        else{
          double yaw = atan2(vy, vx);
          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, yaw);
          pst.pose.orientation.x = q.getX();
          pst.pose.orientation.y = q.getY();
          pst.pose.orientation.z = q.getZ();
          pst.pose.orientation.w = q.getW();
        }
      }
      ros_path.poses.push_back(pst);
    }
    ros_path.poses.push_back(goal);
    
    RCLCPP_INFO(this->get_logger(), "[makeROSPlan] Planground path found: %lu nodes, smoothed: %lu nodes", 
      path.size(), smoothed_planground_path.size());
    return ros_path;
  }
}

} // namespace global_planner
