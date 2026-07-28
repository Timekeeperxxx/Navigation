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
#include <global_planner/b2_global_path_gate.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <limits>
#include <sstream>
#include <tf2/utils.h>

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

  declare_parameter("hybrid_reference_planning_time", rclcpp::ParameterValue(0.75));
  this->get_parameter("hybrid_reference_planning_time", hybrid_reference_planning_time_);
  // Zero means no wall-clock deadline. Search continues until it finds a
  // route, proves the finite graph disconnected, or a newer goal cancels it.
  declare_parameter("hybrid_max_planning_time", rclcpp::ParameterValue(0.0));
  this->get_parameter("hybrid_max_planning_time", hybrid_max_planning_time_);
  declare_parameter("hybrid_heuristic_weight", rclcpp::ParameterValue(2.5));
  this->get_parameter("hybrid_heuristic_weight", hybrid_heuristic_weight_);

  declare_parameter("global_ground_support_enabled", rclcpp::ParameterValue(true));
  this->get_parameter("global_ground_support_enabled", global_ground_support_enabled_);
  declare_parameter(
    "global_ground_support_turn_validation_enabled",
    rclcpp::ParameterValue(false));
  this->get_parameter(
    "global_ground_support_turn_validation_enabled",
    global_ground_support_turn_validation_enabled_);
  declare_parameter("global_ground_support_edge_step", rclcpp::ParameterValue(0.05));
  this->get_parameter("global_ground_support_edge_step", global_ground_support_edge_step_);
  declare_parameter(
    "global_ground_support_turn_angle_step",
    rclcpp::ParameterValue(0.08726646259971647));
  this->get_parameter(
    "global_ground_support_turn_angle_step",
    global_ground_support_turn_angle_step_);
  declare_parameter(
    "global_ground_support_turn_min_angle",
    rclcpp::ParameterValue(1.0471975511965976));
  this->get_parameter(
    "global_ground_support_turn_min_angle",
    global_ground_support_turn_min_angle_);
  declare_parameter("global_ground_support_bucket_size", rclcpp::ParameterValue(0.15));
  this->get_parameter(
    "global_ground_support_bucket_size",
    global_ground_support_config_.bucket_size);
  declare_parameter("global_ground_support_xy_tolerance", rclcpp::ParameterValue(0.15));
  this->get_parameter(
    "global_ground_support_xy_tolerance",
    global_ground_support_config_.xy_tolerance);
  declare_parameter("global_ground_support_z_tolerance", rclcpp::ParameterValue(0.20));
  this->get_parameter(
    "global_ground_support_z_tolerance",
    global_ground_support_config_.z_tolerance);
  declare_parameter("global_ground_support_planning_height", rclcpp::ParameterValue(0.32));
  this->get_parameter(
    "global_ground_support_planning_height",
    global_ground_support_config_.planning_height);
  declare_parameter("global_ground_support_circle_radius", rclcpp::ParameterValue(0.27));
  this->get_parameter(
    "global_ground_support_circle_radius",
    global_ground_support_config_.circle_radius);
  declare_parameter("global_ground_support_circle_offset", rclcpp::ParameterValue(0.205));
  this->get_parameter(
    "global_ground_support_circle_offset",
    global_ground_support_config_.circle_offset);
  declare_parameter("global_ground_support_circle_center_offset", rclcpp::ParameterValue(-0.425));
  this->get_parameter(
    "global_ground_support_circle_center_offset",
    global_ground_support_config_.circle_center_offset);
  declare_parameter("global_ground_support_footprint_probe_margin", rclcpp::ParameterValue(0.19));
  this->get_parameter(
    "global_ground_support_footprint_probe_margin",
    global_ground_support_config_.footprint_probe_margin);
  declare_parameter("global_ground_support_perimeter_samples", rclcpp::ParameterValue(16));
  this->get_parameter(
    "global_ground_support_perimeter_samples",
    global_ground_support_config_.perimeter_samples);
  declare_parameter("global_ground_support_radial_samples", rclcpp::ParameterValue(2));
  this->get_parameter(
    "global_ground_support_radial_samples",
    global_ground_support_config_.radial_samples);
  declare_parameter(
    "global_start_maneuver_max_forward_distance",
    rclcpp::ParameterValue(2.0));
  this->get_parameter(
    "global_start_maneuver_max_forward_distance",
    global_start_maneuver_max_forward_distance_);
  declare_parameter(
    "global_start_maneuver_forward_step",
    rclcpp::ParameterValue(0.05));
  this->get_parameter(
    "global_start_maneuver_forward_step",
    global_start_maneuver_forward_step_);
  declare_parameter(
    "global_start_maneuver_path_sample_step",
    rclcpp::ParameterValue(0.05));
  this->get_parameter(
    "global_start_maneuver_path_sample_step",
    global_start_maneuver_path_sample_step_);
  declare_parameter(
    "global_start_maneuver_yaw_sample_step",
    rclcpp::ParameterValue(0.08726646259971647));
  this->get_parameter(
    "global_start_maneuver_yaw_sample_step",
    global_start_maneuver_yaw_sample_step_);
  declare_parameter(
    "global_start_maneuver_max_join_distance",
    rclcpp::ParameterValue(12.0));
  this->get_parameter(
    "global_start_maneuver_max_join_distance",
    global_start_maneuver_max_join_distance_);

  RCLCPP_INFO(
    this->get_logger(),
    "Hybrid budgets: reference=%.2fs main=%.2fs heuristic_weight=%.2f; "
    "global ground footprint=%s edge_step=%.2fm turn_step=%.1fdeg "
    "turn_min=%.1fdeg",
    hybrid_reference_planning_time_, hybrid_max_planning_time_,
    hybrid_heuristic_weight_,
    global_ground_support_enabled_ ? "enabled" : "disabled",
    global_ground_support_edge_step_,
    global_ground_support_turn_angle_step_ * 180.0 / M_PI,
    global_ground_support_turn_min_angle_ * 180.0 / M_PI);

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
  pub_planner_ready_ = this->create_publisher<std_msgs::msg::Bool>("/nav/planner_ready", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_planning_status_ = this->create_publisher<std_msgs::msg::String>(
    "/nav/planning_status",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

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

  stop_goal_worker_.store(false);
  goal_worker_ = std::thread(&GlobalPlanner::goalWorkerLoop, this);
  
}

GlobalPlanner::~GlobalPlanner(){

  stop_goal_worker_.store(true);
  ++planning_generation_;
  pending_goal_cv_.notify_all();
  if (goal_worker_.joinable()) {
    goal_worker_.join();
  }

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

void GlobalPlanner::publishPlanningStatus(
  const std::string& status,
  const std::string& message,
  uint64_t generation,
  double elapsed_seconds)
{
  if (!pub_planning_status_) {
    return;
  }
  std_msgs::msg::String status_msg;
  std::ostringstream json;
  json << "{\"status\":\"" << status
       << "\",\"message\":\"" << message
       << "\",\"generation\":" << generation
       << ",\"elapsed_seconds\":" << std::fixed << std::setprecision(3)
       << std::max(0.0, elapsed_seconds) << "}";
  status_msg.data = json.str();
  pub_planning_status_->publish(status_msg);
}

std::shared_ptr<plan_env::GroundSupportIndex>
GlobalPlanner::makeGroundSupportIndex(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud) const
{
  if (!global_ground_support_enabled_ || !ground_cloud || ground_cloud->empty()) {
    return {};
  }
  auto support = std::make_shared<plan_env::GroundSupportIndex>(
    global_ground_support_config_);
  support->reserve(ground_cloud->size());
  for (const auto& point : ground_cloud->points) {
    support->addPoint(point.x, point.y, point.z);
  }
  return support;
}

A_Star_on_Graph::EdgeValidator GlobalPlanner::makeGroundEdgeValidator(
  const std::shared_ptr<plan_env::GroundSupportIndex>& support) const
{
  if (!support || support->empty()) {
    return {};
  }
  const double sample_step = std::clamp(
    global_ground_support_edge_step_, 0.02, 0.05);
  const double planning_height = support->config().planning_height;
  return [support, sample_step, planning_height](
    const pcl::PointXYZI& from,
    const pcl::PointXYZI& to) {
      const double dx = static_cast<double>(to.x) - from.x;
      const double dy = static_cast<double>(to.y) - from.y;
      const double dz = static_cast<double>(to.z) - from.z;
      const double distance = std::hypot(dx, dy);
      if (!std::isfinite(distance)) {
        return false;
      }
      const double yaw = distance > 1e-6 ? std::atan2(dy, dx) : 0.0;
      const int samples = std::max(
        1, static_cast<int>(std::ceil(distance / sample_step)));
      for (int sample = 0; sample <= samples; ++sample) {
        const double ratio =
          static_cast<double>(sample) / static_cast<double>(samples);
        const double x = static_cast<double>(from.x) + dx * ratio;
        const double y = static_cast<double>(from.y) + dy * ratio;
        const double ground_z = static_cast<double>(from.z) + dz * ratio;
        if (!support->isPoseSupported(
              x, y, ground_z + planning_height, yaw)) {
          return false;
        }
      }
      return true;
    };
}

Hybrid_A_Star::TurnValidator GlobalPlanner::makeGroundTurnValidator(
  const std::shared_ptr<plan_env::GroundSupportIndex>& support) const
{
  if (!global_ground_support_turn_validation_enabled_ ||
      !support || support->empty()) {
    return {};
  }
  const double yaw_step = std::clamp(
    global_ground_support_turn_angle_step_,
    0.017453292519943295,
    0.17453292519943295);
  const double minimum_turn = std::clamp(
    global_ground_support_turn_min_angle_,
    0.0,
    M_PI);
  const double planning_height = support->config().planning_height;
  return [support, yaw_step, minimum_turn, planning_height](
    const pcl::PointXYZI& previous,
    const pcl::PointXYZI& pivot,
    const pcl::PointXYZI& next) {
      const double incoming_dx =
        static_cast<double>(pivot.x) - previous.x;
      const double incoming_dy =
        static_cast<double>(pivot.y) - previous.y;
      const double outgoing_dx =
        static_cast<double>(next.x) - pivot.x;
      const double outgoing_dy =
        static_cast<double>(next.y) - pivot.y;
      if (std::hypot(incoming_dx, incoming_dy) <= 1e-6 ||
          std::hypot(outgoing_dx, outgoing_dy) <= 1e-6) {
        return true;
      }

      const double incoming_yaw = std::atan2(incoming_dy, incoming_dx);
      const double outgoing_yaw = std::atan2(outgoing_dy, outgoing_dx);
      const double yaw_delta = std::atan2(
        std::sin(outgoing_yaw - incoming_yaw),
        std::cos(outgoing_yaw - incoming_yaw));
      if (std::abs(yaw_delta) < minimum_turn) {
        return true;
      }

      const int samples = std::max(
        1, static_cast<int>(std::ceil(std::abs(yaw_delta) / yaw_step)));
      const double planning_z =
        static_cast<double>(pivot.z) + planning_height;
      for (int sample = 0; sample <= samples; ++sample) {
        const double ratio =
          static_cast<double>(sample) / static_cast<double>(samples);
        const double yaw = incoming_yaw + ratio * yaw_delta;
        if (!support->isPoseSupported(
              pivot.x, pivot.y, planning_z, yaw)) {
          return false;
        }
      }
      return true;
    };
}

void GlobalPlanner::configureHybridPlanner(
  Hybrid_A_Star& planner,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud,
  const std::function<bool()>& cancel_checker)
{
  planner.setReferencePlanningTime(hybrid_reference_planning_time_);
  planner.setMaxPlanningTime(hybrid_max_planning_time_);
  planner.setHeuristicWeight(hybrid_heuristic_weight_);
  planner.setCancelChecker(cancel_checker);
  const auto support = makeGroundSupportIndex(ground_cloud);
  planner.setEdgeValidator(makeGroundEdgeValidator(support));
  planner.setTurnValidator(makeGroundTurnValidator(support));
}

bool GlobalPlanner::finalizeB2GlobalPath(
  nav_msgs::msg::Path& path,
  const geometry_msgs::msg::PoseStamped& live_start,
  const geometry_msgs::msg::PoseStamped& exact_goal,
  bool enforce_goal_yaw,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud,
  const pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr& ground_kdtree)
{
  if (path.poses.empty() || !ground_cloud || ground_cloud->empty() ||
      !ground_kdtree) {
    RCLCPP_WARN(
      this->get_logger(),
      "[B2 path gate] Reject path: dense ground support is unavailable.");
    path.poses.clear();
    return false;
  }

  const auto support = makeGroundSupportIndex(ground_cloud);
  if (!support || support->empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "[B2 path gate] Reject path: ground-support index is unavailable.");
    path.poses.clear();
    return false;
  }

  const auto& start_q_msg = live_start.pose.orientation;
  const double start_q_norm = std::sqrt(
    start_q_msg.x * start_q_msg.x +
    start_q_msg.y * start_q_msg.y +
    start_q_msg.z * start_q_msg.z +
    start_q_msg.w * start_q_msg.w);
  if (!std::isfinite(start_q_norm) || start_q_norm <= 1e-9) {
    RCLCPP_WARN(
      this->get_logger(),
      "[B2 path gate] Reject path: live robot orientation is invalid.");
    path.poses.clear();
    return false;
  }
  tf2::Quaternion start_q(
    start_q_msg.x / start_q_norm,
    start_q_msg.y / start_q_norm,
    start_q_msg.z / start_q_norm,
    start_q_msg.w / start_q_norm);
  const double live_start_yaw = tf2::getYaw(start_q);

  std::optional<double> exact_goal_yaw;
  if (enforce_goal_yaw) {
    const auto& goal_q_msg = exact_goal.pose.orientation;
    const double goal_q_norm = std::sqrt(
      goal_q_msg.x * goal_q_msg.x +
      goal_q_msg.y * goal_q_msg.y +
      goal_q_msg.z * goal_q_msg.z +
      goal_q_msg.w * goal_q_msg.w);
    if (!std::isfinite(goal_q_norm) || goal_q_norm <= 1e-9) {
      RCLCPP_WARN(
        this->get_logger(),
        "[B2 path gate] Reject path: requested goal orientation is invalid.");
      path.poses.clear();
      return false;
    }
    tf2::Quaternion goal_q(
      goal_q_msg.x / goal_q_norm,
      goal_q_msg.y / goal_q_norm,
      goal_q_msg.z / goal_q_norm,
      goal_q_msg.w / goal_q_norm);
    exact_goal_yaw = tf2::getYaw(goal_q);
  }

  std::vector<B2StartManeuverPoint> candidate;
  candidate.reserve(path.poses.size() + 2);
  for (const auto& pose : path.poses) {
    const auto& position = pose.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[B2 path gate] Reject path: candidate contains a non-finite point.");
      path.poses.clear();
      return false;
    }
    candidate.push_back(
      B2StartManeuverPoint{position.x, position.y, position.z});
  }

  const B2StartManeuverPoint start_point{
    live_start.pose.position.x,
    live_start.pose.position.y,
    live_start.pose.position.z};
  if (candidate.empty() ||
      b2StartHorizontalDistance(candidate.front(), start_point) > 1e-3 ||
      std::abs(candidate.front().z - start_point.z) > 1e-3) {
    candidate.insert(candidate.begin(), start_point);
  } else {
    candidate.front() = start_point;
  }

  const B2StartManeuverPoint goal_point{
    exact_goal.pose.position.x,
    exact_goal.pose.position.y,
    exact_goal.pose.position.z};
  if (!candidate.empty()) {
    const double endpoint_xy =
      b2StartHorizontalDistance(candidate.back(), goal_point);
    const double endpoint_z = std::abs(candidate.back().z - goal_point.z);
    if (endpoint_xy <= hybrid_max_ground_bridge_length_ &&
        endpoint_z <= support->config().z_tolerance) {
      if (endpoint_xy > 1e-3 || endpoint_z > 1e-3) {
        candidate.push_back(goal_point);
      } else {
        candidate.back() = goal_point;
      }
    }
  }

  const double planning_height = support->config().planning_height;
  const B2PoseSupportQuery pose_supported =
    [support, planning_height](
      const B2StartManeuverPoint& point, double yaw) {
      return support->isPoseSupported(
        point.x, point.y, point.z + planning_height, yaw);
    };

  const double ground_radius = std::max(
    0.20,
    support->config().xy_tolerance +
      global_start_maneuver_path_sample_step_);
  const double ground_query_radius = std::hypot(
    ground_radius, support->config().z_tolerance);
  const double z_tolerance = support->config().z_tolerance;
  const B2GroundHeightQuery ground_height =
    [ground_cloud, ground_kdtree, ground_radius, ground_query_radius,
     z_tolerance](
      double x, double y, double reference_z, double& ground_z) {
      pcl::PointXYZI query;
      query.x = static_cast<float>(x);
      query.y = static_cast<float>(y);
      query.z = static_cast<float>(reference_z);
      query.intensity = 0.0f;
      std::vector<int> indices;
      std::vector<float> squared_distances;
      if (ground_kdtree->radiusSearch(
            query, ground_query_radius, indices, squared_distances) <= 0) {
        return false;
      }

      double best_score = std::numeric_limits<double>::infinity();
      bool found = false;
      for (const int index : indices) {
        if (index < 0 ||
            static_cast<std::size_t>(index) >= ground_cloud->size()) {
          continue;
        }
        const auto& point = ground_cloud->points[
          static_cast<std::size_t>(index)];
        const double xy_distance = std::hypot(
          static_cast<double>(point.x) - x,
          static_cast<double>(point.y) - y);
        const double z_distance =
          std::abs(static_cast<double>(point.z) - reference_z);
        if (xy_distance > ground_radius || z_distance > z_tolerance) {
          continue;
        }
        const double score = xy_distance + 0.5 * z_distance;
        if (score < best_score) {
          best_score = score;
          ground_z = point.z;
          found = true;
        }
      }
      return found;
    };

  B2GlobalPathGateConfig gate_config;
  gate_config.start_maneuver.maximum_forward_distance =
    global_start_maneuver_max_forward_distance_;
  gate_config.start_maneuver.forward_step =
    global_start_maneuver_forward_step_;
  gate_config.start_maneuver.path_sample_step =
    global_start_maneuver_path_sample_step_;
  gate_config.start_maneuver.yaw_sample_step =
    global_start_maneuver_yaw_sample_step_;
  gate_config.start_maneuver.maximum_join_distance =
    global_start_maneuver_max_join_distance_;
  gate_config.endpoint_xy_tolerance = 1e-3;
  gate_config.endpoint_z_tolerance = 1e-3;

  const B2GlobalPathGateResult gate_result = prepareB2GlobalPath(
    candidate, live_start_yaw, goal_point, exact_goal_yaw,
    gate_config, pose_supported, ground_height);
  if (gate_result.path.empty()) {
    const char* reason = "invalid path";
    switch (gate_result.status) {
      case B2GlobalPathGateStatus::ENDPOINT_MISMATCH:
        reason = "candidate did not reach the exact requested goal";
        break;
      case B2GlobalPathGateStatus::GOAL_POSE_UNSUPPORTED:
        reason = "requested goal yaw has no double-circle ground support";
        break;
      case B2GlobalPathGateStatus::TERMINAL_SWEEP_UNSUPPORTED:
        reason = "arrival-to-goal yaw sweep has no double-circle support";
        break;
      case B2GlobalPathGateStatus::PATH_TURN_UNSUPPORTED:
        reason = "an intermediate path turn has no double-circle support";
        break;
      case B2GlobalPathGateStatus::START_BLOCKED:
        reason = "no safe in-place turn or forward-only start corridor";
        break;
      default:
        break;
    }
    RCLCPP_WARN(
      this->get_logger(), "[B2 path gate] Reject path: %s.", reason);
    path.poses.clear();
    return false;
  }

  nav_msgs::msg::Path finalized;
  finalized.header = path.header;
  finalized.poses.reserve(gate_result.path.size());
  for (std::size_t index = 0; index < gate_result.path.size(); ++index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = finalized.header;
    pose.pose.position.x = gate_result.path[index].x;
    pose.pose.position.y = gate_result.path[index].y;
    pose.pose.position.z = gate_result.path[index].z;

    std::size_t next = index + 1;
    while (next < gate_result.path.size() &&
           b2StartHorizontalDistance(
             gate_result.path[index], gate_result.path[next]) <= 1e-6) {
      ++next;
    }
    if (next < gate_result.path.size()) {
      const double yaw = std::atan2(
        gate_result.path[next].y - gate_result.path[index].y,
        gate_result.path[next].x - gate_result.path[index].x);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
    } else if (!finalized.poses.empty()) {
      pose.pose.orientation = finalized.poses.back().pose.orientation;
    } else {
      pose.pose.orientation = live_start.pose.orientation;
    }
    finalized.poses.push_back(pose);
  }

  if (enforce_goal_yaw && !finalized.poses.empty()) {
    finalized.poses.back().pose.orientation =
      exact_goal.pose.orientation;
  }
  path = std::move(finalized);
  if (gate_result.status == B2GlobalPathGateStatus::FORWARD_REPAIRED) {
    RCLCPP_INFO(
      this->get_logger(),
      "[B2 path gate] Added %.2fm forward-only start escape before "
      "the normal turn; no reverse or lateral motion.",
      gate_result.forward_escape_distance);
  }
  return true;
}

void GlobalPlanner::enqueueGoal(
  const geometry_msgs::msg::PoseStamped& goal_pose,
  bool enforce_goal_yaw)
{
  geometry_msgs::msg::PoseStamped normalized_goal = goal_pose;
  const auto& position = goal_pose.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z)) {
    publishPlanningStatus(
      "rejected", "目标坐标包含非有限值", planning_generation_.load());
    RCLCPP_ERROR(this->get_logger(), "Reject non-finite navigation goal.");
    return;
  }

  if (enforce_goal_yaw) {
    auto& orientation = normalized_goal.pose.orientation;
    const double norm = std::sqrt(
      orientation.x * orientation.x +
      orientation.y * orientation.y +
      orientation.z * orientation.z +
      orientation.w * orientation.w);
    if (!std::isfinite(norm) || norm <= 1e-9) {
      publishPlanningStatus(
        "rejected", "目标朝向四元数无效", planning_generation_.load());
      RCLCPP_ERROR(
        this->get_logger(), "Reject navigation goal with invalid quaternion.");
      return;
    }
    orientation.x /= norm;
    orientation.y /= norm;
    orientation.z /= norm;
    orientation.w /= norm;
  }

  const auto received_at = clock_->now();
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(pending_goal_mutex_);
    if (last_enqueued_goal_) {
      const auto& previous = last_enqueued_goal_->pose.position;
      const double position_delta = std::hypot(
        position.x - previous.x, position.y - previous.y);
      const double z_delta = std::abs(position.z - previous.z);
      const double age =
        (received_at - last_enqueued_goal_time_).seconds();
      bool same_yaw_contract =
        enforce_goal_yaw == last_enqueued_goal_enforce_yaw_;
      if (same_yaw_contract && enforce_goal_yaw) {
        const auto& previous_q_msg =
          last_enqueued_goal_->pose.orientation;
        const auto& current_q_msg =
          normalized_goal.pose.orientation;
        const tf2::Quaternion previous_q(
          previous_q_msg.x, previous_q_msg.y,
          previous_q_msg.z, previous_q_msg.w);
        const tf2::Quaternion current_q(
          current_q_msg.x, current_q_msg.y,
          current_q_msg.z, current_q_msg.w);
        const double previous_yaw = tf2::getYaw(previous_q);
        const double current_yaw = tf2::getYaw(current_q);
        same_yaw_contract =
          std::abs(std::atan2(
            std::sin(current_yaw - previous_yaw),
            std::cos(current_yaw - previous_yaw))) <= 1e-4;
      }
      if (position_delta <= 1e-4 && z_delta <= 1e-4 &&
          same_yaw_contract &&
          age >= 0.0 && age < 1.0) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Ignore duplicate goal retransmission within %.3fs.", age);
        return;
      }
    }
    last_enqueued_goal_ = normalized_goal;
    last_enqueued_goal_enforce_yaw_ = enforce_goal_yaw;
    last_enqueued_goal_time_ = received_at;
    generation = ++planning_generation_;
    pending_goal_ =
      PendingGoal{normalized_goal, generation, enforce_goal_yaw};
  }

  // Do not publish an empty ROS path here.  SCAN treats /global_path as its
  // execution contract, and an empty path cannot atomically cancel an already
  // running B-spline.  The BotDog UI clears its own cached visualization as
  // soon as the goal is accepted; the existing safe trajectory remains valid
  // until a replacement path has passed all checks and is published.
  publishPlanningStatus("queued", "已接收目标，等待规划", generation);
  pending_goal_cv_.notify_one();
}

void GlobalPlanner::goalWorkerLoop()
{
  while (!stop_goal_worker_.load()) {
    PendingGoal pending;
    {
      std::unique_lock<std::mutex> lock(pending_goal_mutex_);
      pending_goal_cv_.wait(
        lock,
        [this]() {
          return stop_goal_worker_.load() || pending_goal_.has_value();
        });
      if (stop_goal_worker_.load()) {
        return;
      }
      pending = *pending_goal_;
      pending_goal_.reset();
    }
    planGoalPose(
      pending.pose, pending.generation, pending.enforce_goal_yaw);
  }
}

void GlobalPlanner::cbClickedPoint(
  const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header = clicked_goal->header;
  goal.pose.position.x = clicked_goal->point.x;
  goal.pose.position.y = clicked_goal->point.y;
  goal.pose.position.z = clicked_goal->point.z;
  goal.pose.orientation.w = 1.0;
  enqueueGoal(goal, false);
}

void GlobalPlanner::cbGoalPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose)
{
  enqueueGoal(*goal_pose, true);
}

void GlobalPlanner::planGoalPose(
  const geometry_msgs::msg::PoseStamped& requested_goal,
  uint64_t generation,
  bool enforce_goal_yaw)
{
  const auto started_at = std::chrono::steady_clock::now();
  const auto cancelled = [this, generation]() {
    return stop_goal_worker_.load() ||
           generation != planning_generation_.load();
  };
  bool path_published = false;
  struct StatusGuard
  {
    std::function<void()> finish;
    ~StatusGuard() { finish(); }
  } status_guard{
    [this, &cancelled, &path_published, generation, started_at]() {
      if (!path_published && !cancelled()) {
        const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started_at).count();
        publishPlanningStatus(
          "failed", "未找到满足同层 ground 足迹约束的路径",
          generation, elapsed);
      }
    }};
  const auto publish_path = [this, &cancelled, &path_published,
                             generation, started_at](
    nav_msgs::msg::Path& path) {
      if (cancelled() || path.poses.empty()) {
        return false;
      }
      path.header.frame_id = global_frame_;
      path.header.stamp = clock_->now();
      for (auto& pose : path.poses) {
        pose.header = path.header;
      }
      pub_path_->publish(path);
      path_published = true;
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started_at).count();
      publishPlanningStatus(
        "path_ready", "全局路径已生成", generation, elapsed);
      return true;
    };

  publishPlanningStatus("planning", "正在计算全局路径", generation);

  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Received clicked goal before static layer is ready");
    return;
  }

  geometry_msgs::msg::PoseStamped start, goal;
  goal = requested_goal;

  geometry_msgs::msg::TransformStamped transformStamped;

  try
  {
    transformStamped = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    start.pose.position.x = transformStamped.transform.translation.x;
    start.pose.position.y = transformStamped.transform.translation.y;
    start.pose.position.z = transformStamped.transform.translation.z;
    start.pose.orientation = transformStamped.transform.rotation;
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
    planground_kdtree->setInputCloud(planground_cloud);
  }
  {
    std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
    if(!pcl_ground_->points.empty()){
      *ground_cloud = *pcl_ground_;
      ground_kdtree->setInputCloud(ground_cloud);
    }
  }

  unsigned int start_id, goal_id;
  unsigned int ground_start_id, ground_goal_id;
  std::vector<unsigned int> path;
  std::vector<unsigned int> smoothed_path;
  std::vector<unsigned int> smoothed_path_2nd;
  nav_msgs::msg::Path ros_path;
  const auto configure_start_candidate =
    [this, &start, &ground_cloud, &cancelled](
      Hybrid_A_Star& planner) {
      configureHybridPlanner(planner, ground_cloud, cancelled);
      const auto strict_edge_validator =
        makeGroundEdgeValidator(makeGroundSupportIndex(ground_cloud));
      const double start_x = start.pose.position.x;
      const double start_y = start.pose.position.y;
      const double start_z = start.pose.position.z;
      planner.setEdgeValidator(
        [strict_edge_validator, start_x, start_y, start_z](
          const pcl::PointXYZI& from, const pcl::PointXYZI& to) {
          const bool leaves_exact_live_start =
            std::hypot(
              static_cast<double>(from.x) - start_x,
              static_cast<double>(from.y) - start_y) <= 0.02 &&
            std::abs(static_cast<double>(from.z) - start_z) <= 0.20;
          // This exemption only obtains a private position candidate. The
          // final B2 gate must either replace this edge with a supported
          // straight-forward escape or reject the whole path.
          return leaves_exact_live_start ||
                 !strict_edge_validator ||
                 strict_edge_validator(from, to);
        });
    };
  const auto configure_goal_candidate =
    [this, &goal, &ground_cloud, enforce_goal_yaw](
      Hybrid_A_Star& planner) {
      const auto support = makeGroundSupportIndex(ground_cloud);
      const auto edge_validator = makeGroundEdgeValidator(support);
      if (!support || !edge_validator) {
        return;
      }

      std::optional<double> requested_goal_yaw;
      if (enforce_goal_yaw) {
        const auto& orientation = goal.pose.orientation;
        const tf2::Quaternion q(
          orientation.x, orientation.y, orientation.z, orientation.w);
        requested_goal_yaw = tf2::getYaw(q);
      }
      const double planning_height = support->config().planning_height;
      const double yaw_step = global_start_maneuver_yaw_sample_step_;
      const B2PoseSupportQuery pose_supported =
        [support, planning_height](
          const B2StartManeuverPoint& point, double yaw) {
          return support->isPoseSupported(
            point.x, point.y, point.z + planning_height, yaw);
        };

      planner.setGoalConnectorValidator(
        [edge_validator, pose_supported, requested_goal_yaw, yaw_step](
          const pcl::PointXYZI& from,
          const pcl::PointXYZI& exact_goal) {
          if (!edge_validator(from, exact_goal)) {
            return false;
          }
          if (!requested_goal_yaw) {
            return true;
          }

          const B2StartManeuverPoint terminal{
            exact_goal.x, exact_goal.y, exact_goal.z};
          if (!pose_supported(terminal, *requested_goal_yaw)) {
            return false;
          }
          const double dx =
            static_cast<double>(exact_goal.x) - from.x;
          const double dy =
            static_cast<double>(exact_goal.y) - from.y;
          if (std::hypot(dx, dy) <= 1e-6) {
            return true;
          }
          const double arrival_yaw = std::atan2(dy, dx);
          return isB2StartTurnSupported(
            terminal, arrival_yaw, *requested_goal_yaw,
            yaw_step, pose_supported);
        });
    };

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
      goal.pose.position.z =
        ground_cloud->points[ground_goal_id].z;
      
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
      configure_start_candidate(hybrid_planner);
      configure_goal_candidate(hybrid_planner);
      pcl::PointXYZI exact_goal_point;
      exact_goal_point.x = static_cast<float>(goal.pose.position.x);
      exact_goal_point.y = static_cast<float>(goal.pose.position.y);
      exact_goal_point.z = static_cast<float>(goal.pose.position.z);
      exact_goal_point.intensity = 0.0f;
      
      if(start_on_planground){
        // Start is on planground, goal is on ground - use getPathWithStartPoseAndGroundGoal
        RCLCPP_INFO(this->get_logger(), "[Hybrid] Start on planground, goal on ground, using getPathWithStartPoseAndGroundGoal.");
        hybrid_planner.getPathWithStartPoseAndGroundGoal(
          start, ground_goal_id, path, &exact_goal_point);
      }

      else{
        // Both start and goal are NOT on planground
        // Use getPathWithStartPoseAndGroundGoal which adds the robot's actual position
        // to the hybrid cloud as the start point
        RCLCPP_INFO(this->get_logger(), "[Hybrid] Both start and goal not on planground, using getPathWithStartPoseAndGroundGoal.");
        hybrid_planner.getPathWithStartPoseAndGroundGoal(
          start, ground_goal_id, path, &exact_goal_point);
      }
      
      if(path.empty()){
        RCLCPP_WARN(this->get_logger(), "[Hybrid] No path found via hybrid planning for clicked point.");
        return;
      }
      
      // Build ROS path from hybrid cloud points with smoothing
      // Use smoothPathToRosPath which properly interpolates Z using Catmull-Rom spline
      pcl::PointCloud<pcl::PointXYZI>::Ptr hybrid_cloud = hybrid_planner.getHybridCloud();
      
      hybrid_planner.smoothPathToRosPath(path, hybrid_cloud, ros_path, goal, global_frame_, 0.2);
      if (!finalizeB2GlobalPath(
            ros_path, start, goal, enforce_goal_yaw,
            ground_cloud, ground_kdtree) ||
          !publish_path(ros_path)) {
        RCLCPP_WARN(
          this->get_logger(),
          "[Hybrid] Ground-safe path output is empty or this goal was superseded.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "[Hybrid] Hybrid path found for clicked point (ground goal): %lu nodes, smoothed: %lu points", path.size(), ros_path.poses.size());
      return;
    }
    goal.pose.position.z = planground_cloud->points[goal_id].z;
    
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
    configure_start_candidate(hybrid_planner);
    configure_goal_candidate(hybrid_planner);
    pcl::PointXYZI exact_goal_point;
    exact_goal_point.x = static_cast<float>(goal.pose.position.x);
    exact_goal_point.y = static_cast<float>(goal.pose.position.y);
    exact_goal_point.z = static_cast<float>(goal.pose.position.z);
    exact_goal_point.intensity = 0.0f;
    
    // Use getPathWithStartPose to ensure the path starts from the robot's actual position

    // This works for both cases:
    // - Start on planground: path starts from robot position, goal is planground index
    // - Start on ground: path starts from robot position, goal is planground index
    //   getPathWithStartPose adds the start pose as a dedicated point in the hybrid cloud
    //   and builds the corridor from start_pose to goal_pose, so ground points near the
    //   robot's actual position will be included in the corridor
    hybrid_planner.getPathWithStartPose(
      start, goal_id, path, &exact_goal_point);
    
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
    if (!finalizeB2GlobalPath(
          ros_path, start, goal, enforce_goal_yaw,
          ground_cloud, ground_kdtree) ||
        !publish_path(ros_path)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Hybrid] Ground-safe path output is empty or this goal was superseded.");
      return;
    }
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
    goal.pose.position.z = planground_cloud->points[goal_id].z;

    if(!use_pre_graph_){
      auto a_star = std::make_shared<A_Star_on_Graph>(planground_cloud, perception_3d_ros_, a_star_expanding_radius_);
      a_star->setupTurningWeight(turning_weight_);
      a_star->setMaxPlanningTime(hybrid_max_planning_time_);
      a_star->setHeuristicWeight(hybrid_heuristic_weight_);
      a_star->setCancelChecker(cancelled);
      const auto strict_edge_validator =
        makeGroundEdgeValidator(makeGroundSupportIndex(ground_cloud));
      const double start_x = start.pose.position.x;
      const double start_y = start.pose.position.y;
      const double start_z = start.pose.position.z;
      a_star->setEdgeValidator(
        [strict_edge_validator, start_x, start_y, start_z](
          const pcl::PointXYZI& from, const pcl::PointXYZI& to) {
          const bool leaves_exact_live_start =
            std::hypot(
              static_cast<double>(from.x) - start_x,
              static_cast<double>(from.y) - start_y) <= 0.02 &&
            std::abs(static_cast<double>(from.z) - start_z) <= 0.20;
          return leaves_exact_live_start ||
                 !strict_edge_validator ||
                 strict_edge_validator(from, to);
        });
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
    if (!finalizeB2GlobalPath(
          ros_path, start, goal, enforce_goal_yaw,
          ground_cloud, ground_kdtree) ||
        !publish_path(ros_path)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Planground] Ground-safe path output is empty or this goal was superseded.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[Planground] Path found for clicked point: %lu nodes", path.size());
    return;
  }
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
  graph_ready_ = true;
  std_msgs::msg::Bool ready_msg;
  ready_msg.data = true;
  pub_planner_ready_->publish(ready_msg);
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
      configureHybridPlanner(hybrid_planner, ground_cloud);
      
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
    configureHybridPlanner(hybrid_planner, ground_cloud);
    
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
      a_star->setMaxPlanningTime(hybrid_max_planning_time_);
      a_star->setHeuristicWeight(hybrid_heuristic_weight_);
      a_star->setEdgeValidator(
        makeGroundEdgeValidator(makeGroundSupportIndex(ground_cloud)));
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
      configureHybridPlanner(hybrid_planner, ground_cloud);
      
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
    configureHybridPlanner(hybrid_planner, ground_cloud);
    
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
      a_star->setMaxPlanningTime(hybrid_max_planning_time_);
      a_star->setHeuristicWeight(hybrid_heuristic_weight_);
      a_star->setEdgeValidator(
        makeGroundEdgeValidator(makeGroundSupportIndex(ground_cloud)));
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
