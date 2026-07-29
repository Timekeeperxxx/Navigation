
#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/reference_guide.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>

namespace
{
  builtin_interfaces::msg::Time toMsgTime(const rclcpp::Time &time)
  {
    const int64_t nanoseconds = time.nanoseconds();
    builtin_interfaces::msg::Time msg;
    msg.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    msg.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return msg;
  }

  template <typename T>
  void getParam(const rclcpp::Node::SharedPtr &node, const std::string &name, T &value, const T &default_value)
  {
    if (!node->has_parameter(name))
      node->declare_parameter<T>(name, default_value);
    node->get_parameter(name, value);
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(const rclcpp::Node::SharedPtr &nh)
  {
    node_ = nh;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    controller_execution_frozen_ = false;
    safety_execution_frozen_ = false;
    reverse_recovery_requested_ = false;
    reverse_recovery_active_ = false;
    b2_obstacle_recovery_latched_ = false;
    b2_recovery_subgoal_active_ = false;
    b2_recovery_waypoints_.clear();
    b2_recovery_waypoint_yaws_.clear();
    b2_recovery_waypoint_simultaneous_yaw_.clear();
    b2_recovery_leg_start_yaw_ = 0.0;
    b2_recovery_leg_end_yaw_ = 0.0;
    b2_recovery_leg_simultaneous_yaw_ = false;
    global_target_ground_validation_pending_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();
    replan_not_before_seconds_ = 0.0;

    /*  fsm param  */
    getParam(nh, "fsm/navi_mode", navi_mode_, -1);
    getParam(nh, "fsm/thresh_replan", replan_thresh_, -1.0);
    getParam(nh, "fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    getParam(nh, "fsm/planning_horizon", planning_horizon_, -1.0);
    getParam(
        nh, "fsm/reference_corner_stop_enabled",
        reference_corner_stop_enabled_, false);
    getParam(
        nh, "fsm/reference_corner_stop_angle",
        reference_corner_stop_angle_, 1.0471975511965976);
    reference_corner_stop_angle_ = std::max(
        0.0,
        std::min(
            reference_corner_stop_angle_,
            3.14159265358979323846));
    getParam(nh, "fsm/start_height_offset", start_height_offset_, 0.32);
    start_height_offset_ = std::max(0.0, start_height_offset_);
    getParam(nh, "fsm/final_goal_tolerance", final_goal_tolerance_, 0.12);
    final_goal_tolerance_ = std::max(0.05, final_goal_tolerance_);
    getParam(nh, "fsm/emergency_time_", emergency_time_, 1.0);
    // Keep the existing parameter name for launch-file compatibility. The
    // interval now limits every failed replan, not only safety-frozen ones.
    getParam(nh, "fsm/frozen_replan_retry_interval", replan_retry_interval_, 0.5);
    replan_retry_interval_ = std::max(0.05, replan_retry_interval_);
    getParam(nh, "fsm/execution_validation_enabled", execution_validation_enabled_, true);
    getParam(
        nh, "fsm/execution_validation_path_sample_dt",
        execution_validation_path_sample_dt_, 0.25);
    execution_validation_path_sample_dt_ =
        std::max(0.02, execution_validation_path_sample_dt_);
    getParam(
        nh, "fsm/b2_maximum_path_slope",
        b2_maximum_path_slope_, 0.70);
    b2_maximum_path_slope_ =
        std::max(0.0, b2_maximum_path_slope_);
    getParam(
        nh, "fsm/execution_validation_max_position_step",
        execution_validation_max_position_step_, 0.05);
    execution_validation_max_position_step_ =
        std::max(0.01, execution_validation_max_position_step_);
    getParam(
        nh, "fsm/execution_validation_max_yaw_step",
        execution_validation_max_yaw_step_, 0.08726646259971647);
    execution_validation_max_yaw_step_ =
        std::max(0.017453292519943295, execution_validation_max_yaw_step_);
    getParam(
        nh, "fsm/execution_validation_probe_radius",
        execution_validation_probe_radius_, 0.04);
    execution_validation_probe_radius_ =
        std::max(0.0, execution_validation_probe_radius_);
    getParam(
        nh, "fsm/execution_validation_probe_count",
        execution_validation_probe_count_, 16);
    execution_validation_probe_count_ =
        std::max(4, execution_validation_probe_count_);
    getParam(
        nh, "fsm/execution_validation_min_map_updates",
        execution_validation_min_map_updates_, 3);
    execution_validation_min_map_updates_ =
        std::max(1, execution_validation_min_map_updates_);
    getParam(
        nh, "fsm/execution_validation_allow_occupied_start",
        execution_validation_allow_occupied_start_, true);
    getParam(
        nh, "fsm/execution_validation_start_escape_distance",
        execution_validation_start_escape_distance_, 0.60);
    execution_validation_start_escape_distance_ =
        std::max(0.0, execution_validation_start_escape_distance_);
    getParam(nh, "fsm/b2_allow_reverse", b2_allow_reverse_, false);
    getParam(
        nh, "fsm/b2_reverse_recovery_enabled",
        b2_reverse_recovery_enabled_, true);
    getParam(
        nh, "fsm/b2_reverse_recovery_request_period",
        b2_reverse_recovery_request_period_, 0.50);
    b2_reverse_recovery_request_period_ =
        std::max(0.10, b2_reverse_recovery_request_period_);
    getParam(
        nh, "fsm/b2_reverse_recovery_min_replan_failures",
        b2_reverse_recovery_min_replan_failures_, 2);
    b2_reverse_recovery_min_replan_failures_ =
        std::max(1, b2_reverse_recovery_min_replan_failures_);
    getParam(
        nh, "fsm/b2_reverse_recovery_obstacle_probe_distance",
        b2_reverse_recovery_obstacle_probe_distance_, 0.25);
    b2_reverse_recovery_obstacle_probe_distance_ =
        std::max(0.0, b2_reverse_recovery_obstacle_probe_distance_);
    getParam(
        nh, "fsm/b2_reverse_recovery_obstacle_probe_step",
        b2_reverse_recovery_obstacle_probe_step_, 0.05);
    b2_reverse_recovery_obstacle_probe_step_ =
        std::max(0.02, b2_reverse_recovery_obstacle_probe_step_);
    getParam(
        nh, "fsm/b2_forward_seed_speed",
        b2_forward_seed_config_.speed, 0.15);
    getParam(
        nh, "fsm/b2_forward_seed_clearance_distance",
        b2_forward_seed_config_.clearance_distance, 0.25);
    getParam(
        nh, "fsm/b2_forward_seed_sample_step",
        b2_forward_seed_config_.sample_step, 0.025);
    getParam(
        nh, "fsm/b2_forward_seed_min_reference_alignment",
        b2_forward_seed_config_.minimum_reference_alignment, 0.25);
    getParam(
        nh, "fsm/b2_random_direction_probe_distance",
        b2_direction_guard_config_.probe_distance, 0.20);
    getParam(
        nh, "fsm/b2_random_direction_guard_distance",
        b2_direction_guard_config_.guard_distance, 0.60);
    getParam(
        nh, "fsm/b2_random_max_reference_heading_error",
        b2_direction_guard_config_.maximum_reference_heading_error,
        1.0471975511965976);
    getParam(
        nh, "fsm/b2_random_backtrack_tolerance",
        b2_direction_guard_config_.backtrack_tolerance, 0.03);
    getParam(
        nh, "fsm/b2_obstacle_detour_retreat_allowance",
        b2_direction_guard_config_.obstacle_detour_retreat_allowance, 0.35);
    getParam(
        nh, "fsm/b2_obstacle_detour_minimum_progress",
        b2_direction_guard_config_.obstacle_detour_minimum_progress, 0.20);
    getParam(
        nh, "fsm/b2_obstacle_detour_maximum_length_ratio",
        b2_direction_guard_config_.obstacle_detour_maximum_length_ratio, 2.50);
    b2_forward_seed_config_.speed =
        std::max(0.0, b2_forward_seed_config_.speed);
    b2_forward_seed_config_.clearance_distance =
        std::max(0.0, b2_forward_seed_config_.clearance_distance);
    b2_forward_seed_config_.sample_step =
        std::max(0.01, b2_forward_seed_config_.sample_step);
    b2_forward_seed_config_.minimum_reference_alignment = std::max(
        -1.0,
        std::min(
            b2_forward_seed_config_.minimum_reference_alignment, 1.0));
    b2_direction_guard_config_.probe_distance =
        std::max(0.02, b2_direction_guard_config_.probe_distance);
    b2_direction_guard_config_.guard_distance = std::max(
        b2_direction_guard_config_.probe_distance,
        b2_direction_guard_config_.guard_distance);
    b2_direction_guard_config_.maximum_reference_heading_error = std::max(
        0.0,
        std::min(
            b2_direction_guard_config_.maximum_reference_heading_error,
            M_PI));
    b2_direction_guard_config_.backtrack_tolerance =
        std::max(0.0, b2_direction_guard_config_.backtrack_tolerance);
    getParam(nh, "fsm/fail_safe", enable_fail_safe_, true);
    getParam(nh, "fsm/max_replan_fail_count", max_replan_fail_count_, 1000);
    getParam(nh, "grid_map/obstacles_inflation_z_up", self_inflation_z_up_, 0.0);
    getParam(nh, "grid_map/obstacles_inflation_z_down", self_inflation_z_down_, 0.0);
    getParam(nh, "grid_map/double_cylinder_radius", self_double_cylinder_radius_, 0.0);
    getParam(nh, "grid_map/double_cylinder_offset", self_double_cylinder_offset_, 0.0);
    getParam(nh, "grid_map/double_cylinder_center_offset", self_double_cylinder_center_offset_, 0.0);
    getParam(nh, "grid_map/body_height", body_height_, 0.4);
    getParam(nh, "grid_map/frame_id", self_inflation_frame_id_, std::string("world"));
    getParam(
        nh, "execution_frozen_topic", execution_frozen_topic_,
        std::string("/planning/b2_execution_frozen"));

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      getParam(nh, "fsm/waypoint_num", waypoint_num_, -1);

      if (waypoint_num_ <= 0)
      {
        ROS_ERROR("[SCANReplanFSM] navi_mode=2 requires ROS2 parameters fsm/waypoint_num and fsm/waypoint{i}_{x,y,z}.");
        rclcpp::shutdown();
        return;
      }
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        getParam(nh, "fsm/waypoint" + to_string(i) + "_x", preset_waypoints_[i](0), -1.0);
        getParam(nh, "fsm/waypoint" + to_string(i) + "_y", preset_waypoints_[i](1), -1.0);
        getParam(nh, "fsm/waypoint" + to_string(i) + "_z", preset_waypoints_[i](2), -1.0);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = nh->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SCANReplanFSM::checkCollisionCallback, this));

    std::string body_pose_topic;
    getParam(nh, "body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
    std::string replan_request_topic;
    getParam(nh, "replan_request_topic", replan_request_topic, std::string("/nav/replan_request"));
    std::string safety_execution_frozen_topic;
    getParam(
        nh, "safety_execution_frozen_topic",
        safety_execution_frozen_topic,
        std::string("/planning/safety_execution_frozen"));
    std::string reverse_recovery_request_topic;
    getParam(
        nh, "reverse_recovery_request_topic",
        reverse_recovery_request_topic,
        std::string("/planning/b2_reverse_recovery_request"));
    std::string reverse_recovery_status_topic;
    getParam(
        nh, "reverse_recovery_status_topic",
        reverse_recovery_status_topic,
        std::string("/planning/b2_reverse_recovery_status"));
    odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
        body_pose_topic, 1, std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    controller_execution_frozen_sub_ = nh->create_subscription<std_msgs::msg::Bool>(
        execution_frozen_topic_, 10,
        std::bind(
            &SCANReplanFSM::controllerExecutionFrozenCallback, this,
            std::placeholders::_1));
    safety_execution_frozen_sub_ =
        nh->create_subscription<std_msgs::msg::Bool>(
            safety_execution_frozen_topic,
            10,
            std::bind(
                &SCANReplanFSM::safetyExecutionFrozenCallback,
                this,
                std::placeholders::_1));
    reverse_recovery_status_sub_ =
        nh->create_subscription<std_msgs::msg::UInt8>(
            reverse_recovery_status_topic,
            10,
            std::bind(
                &SCANReplanFSM::reverseRecoveryStatusCallback,
                this,
                std::placeholders::_1));
    replan_request_sub_ = nh->create_subscription<std_msgs::msg::Bool>(
        replan_request_topic, 10, std::bind(&SCANReplanFSM::replanRequestCallback, this, std::placeholders::_1));

    bspline_pub_ = nh->create_publisher<scan_planner::msg::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh->create_publisher<scan_planner::msg::DataDisp>("/planning/data_display", 100);
    self_inflation_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("self_inflation", rclcpp::QoS(10).transient_local());
    emergency_stop_state_pub_ = nh->create_publisher<std_msgs::msg::Bool>(
        "/planning/scan_emergency_stop", rclcpp::QoS(1).reliable().transient_local());
    reverse_recovery_request_pub_ =
        nh->create_publisher<std_msgs::msg::Bool>(
            reverse_recovery_request_topic, 10);
    emergency_stop_state_pub_->publish(std_msgs::msg::Bool().set__data(false));

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/move_base_simple/goal", 1, std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      while (rclcpp::ok() && !have_odom_)
        rclcpp::spin_some(node_);
      planGlobalTrajbyGivenWps();
    }
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      // scan_initial_path_adapter keeps the latest global reference path as a
      // transient-local sample.  Matching that durability lets SCAN recover
      // the path after a planner-only restart without asking for a new goal.
      const auto path_qos =
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      path_sub_ = nh->create_subscription<nav_msgs::msg::Path>(
          "/initial_path", path_qos,
          std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    }
    else
      cout << "Wrong navi_mode_ value! navi_mode_=" << navi_mode_ << endl;
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = getPlanningStartPosition();

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory to first preset waypoint!");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      ROS_WARN("[SCANReplanFSM] Ignore RViz goal before receiving initial body pose.");
      return;
    }

    nav_msgs::msg::Path::SharedPtr path(new nav_msgs::msg::Path);
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[waypointCallback] Empty waypoint message, ignore.");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = getPlanningStartPosition();

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(getPlanningStartPosition(), odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      ROS_WARN("[planGlobalTrajByWaypoints] No waypoint to plan.");
      return false;
    }

    end_pt_ = waypoints.back();

    // In reference-path mode the polyline itself is rendered below. Publishing
    // one goal marker (and sleeping) for every dense global-path sample adds
    // visible latency without contributing to planning.
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH)
    {
      for (size_t i = 0; i < waypoints.size(); i++)
      {
        visualization_->displayGoalPoint(
            waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
      }
    }

    // A newly clicked reference path starts from a stationary boundary
    // condition.  TF differentiation can report lateral velocity while the
    // body is only rotating, which would otherwise bend the first spline away
    // from the supplied global path.
    const Eigen::Vector3d initial_velocity =
        navi_mode_ == NAVI_MODE::REFERENCE_PATH ? Eigen::Vector3d::Zero() : odom_vel_;
    const Eigen::Vector3d planning_start = getPlanningStartPosition();
    // Reference mode selects and validates every local lookahead directly on
    // active_waypoints_. Solving one large minimum-snap system for all dense
    // path samples is therefore redundant (and scales cubically). Keep only a
    // cheap endpoint-based placeholder in global_data_; it is never used as
    // the reference-mode execution geometry.
    bool success =
        navi_mode_ == NAVI_MODE::REFERENCE_PATH
        ? planner_manager_->planGlobalTraj(
            planning_start,
            initial_velocity,
            Eigen::Vector3d::Zero(),
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero())
        : planner_manager_->planGlobalTrajWaypoints(
            planning_start,
            initial_velocity,
            Eigen::Vector3d::Zero(),
            waypoints,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("Unable to generate global trajectory from waypoints!");
      return false;
    }

    // Reference-path mode validates each bounded local lookahead directly on
    // the original polyline.  Do not move its final target to a sample from a
    // smoothed polynomial, which can lie off the supplied global path.
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH && !adjustGlobalTargetIfOccupied())
      return false;

    std::vector<Eigen::Vector3d> gloabl_traj;
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      gloabl_traj = waypoints;
    }
    else
    {
      constexpr double step_size_t = 0.1;
      int i_end = floor(
          planner_manager_->global_data_.global_duration_ / step_size_t);
      gloabl_traj.resize(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] =
            planner_manager_->global_data_.global_traj_.evaluate(
            i * step_size_t);
      }
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      ROS_WARN("[navi_mode=%d] No active waypoint to plan.", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("[navi_mode=%d] Unable to generate trajectory to waypoint %d.", navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    ROS_INFO("[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f].",
             navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
    {
      global_target_ground_validation_pending_ = false;
      return true;
    }
    // A goal can arrive before the transient-local ground cloud callback.
    // Keep the target intact here; GEN_NEW_TRAJ remains fail-closed until the
    // support index is ready and will validate the actual local trajectory.
    if (!map->isGroundSupportReady())
    {
      global_target_ground_validation_pending_ = true;
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[ground support] Preserve the pending target while /mapground is "
          "still loading.");
      return true;
    }
    global_target_ground_validation_pending_ = false;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        ROS_WARN("[global target] Target [%.2f, %.2f, %.2f] is occupied; use backward collision-free point [%.2f, %.2f, %.2f].",
                 raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    ROS_ERROR("[global target] Target is occupied, and no collision-free point was found along the global trajectory.");
    return false;
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[pathCallback] Received empty /initial_path, ignore.");
      return;
    }

    trigger_ = true;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + body_height_; // Adjust for body height
      waypoints.push_back(wp);
    }
    active_waypoints_ = waypoints;
    reference_progress_segment_ = 0;
    reference_progress_ratio_ = 0.0;
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      // A fresh reference path is an explicit new GoTo request. It must be
      // able to recover the FSM from a stale emergency trajectory instead of
      // being discarded by EMERGENCY_STOP's next timer tick.
      replan_fail_count_ = 0;
      replan_not_before_seconds_ = 0.0;
      b2_obstacle_recovery_latched_ = false;
      b2_recovery_subgoal_active_ = false;
      b2_recovery_waypoints_.clear();
      b2_recovery_waypoint_yaws_.clear();
      b2_recovery_waypoint_simultaneous_yaw_.clear();
      need_hover_stop_ = false;

      // Do not keep following the previous GoTo trajectory while switching
      // to a newly generated global reference path. Hold the current pose
      // until the first collision-checked local spline for the new target is
      // ready.
      // This handoff is also required for the first target.  Before the first
      // trigger the FSM intentionally remains in INIT, but the controller's
      // goal-yaw generation barrier still needs a stationary spline between
      // the goal callback and its first executable spline.
      if (have_odom_)
      {
        callEmergencyStop(odom_pos_);
      }

      /*** FSM ***/
      if (exec_state_ != INIT)
      {
        if (exec_state_ == EMERGENCY_STOP)
          flag_escape_emergency_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      ROS_INFO("==========================================\n");
    }
    else
    {
      ROS_ERROR("❌ Unable to generate global trajectory!");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      ROS_INFO("[SCANReplanFSM] Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    // B2 has no commanded vertical degree of freedom.  Preserve z positions
    // for terrain geometry, but do not feed estimated terrain-following vz
    // into the trajectory boundary conditions as if it were controllable.
    odom_vel_(2) = 0.0;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
  }

  void SCANReplanFSM::controllerExecutionFrozenCallback(
      const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    if (msg->data && !controller_execution_frozen_)
    {
      odom_vel_.setZero();
      ROS_INFO("[execution freeze] Hold SCAN trajectory execution; replanning remains enabled with zero boundary velocity.");
    }
    controller_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::safetyExecutionFrozenCallback(
      const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    safety_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::reverseRecoveryStatusCallback(
      const std_msgs::msg::UInt8::ConstSharedPtr &msg)
  {
    const auto status =
        static_cast<B2ReverseRecoveryStatus>(msg->data);
    if (status == B2ReverseRecoveryStatus::ACTIVE)
    {
      reverse_recovery_active_ = true;
      return;
    }
    if (status == B2ReverseRecoveryStatus::IDLE)
      return;

    reverse_recovery_active_ = false;
    reverse_recovery_requested_ = false;
    if (reverse_recovery_request_pub_)
      reverse_recovery_request_pub_->publish(
          std_msgs::msg::Bool().set__data(false));

    const char *reason = "cancelled";
    switch (status)
    {
    case B2ReverseRecoveryStatus::DISTANCE_COMPLETE:
      reason = "0.5m distance limit";
      break;
    case B2ReverseRecoveryStatus::TIME_COMPLETE:
      reason = "2.0s time limit";
      break;
    case B2ReverseRecoveryStatus::SAFETY_BLOCKED:
      reason = "rear obstacle or ground safety block";
      break;
    case B2ReverseRecoveryStatus::ODOMETRY_INVALID:
      reason = "odometry unavailable";
      break;
    case B2ReverseRecoveryStatus::YAW_DRIFT:
      reason = "body yaw drifted during straight reverse";
      break;
    default:
      break;
    }
    ROS_WARN(
        "[B2 reverse recovery] Round ended (%s); keep the active target "
        "and retry forward planning at least twice. Another reverse round "
        "remains allowed only if the immediate dynamic-obstacle gate still "
        "confirms a trap.",
        reason);

    replan_fail_count_ = 0;
    replan_not_before_seconds_ = 0.0;
    if (exec_state_ == REVERSE_RECOVERY && have_target_)
      changeFSMExecState(GEN_NEW_TRAJ, "REVERSE_DONE");
  }

  void SCANReplanFSM::replanRequestCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    if (!msg->data || !have_odom_ || !have_target_)
      return;

    // The dynamic safety monitor freezes command execution before publishing
    // this request.  Replanning must still run while frozen: the replacement
    // execution path is what lets the monitor decide that a bypass is clear
    // and release the zero-velocity hold.
    if (exec_state_ == EXEC_TRAJ)
    {
      ROS_WARN("[dynamic avoidance] Obstacle persisted; request a frozen-start SCAN replan.");
      changeFSMExecState(REPLAN_TRAJ, "DYNAMIC");
    }
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (controller_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  bool SCANReplanFSM::replanAttemptReady() const
  {
    return replanAttemptDue(
        node_->now().seconds(),
        replan_not_before_seconds_);
  }

  void SCANReplanFSM::deferReplanAttempt()
  {
    replan_not_before_seconds_ = nextReplanTime(
        node_->now().seconds(),
        replan_retry_interval_);
  }

  bool SCANReplanFSM::requestB2ReverseRecovery(const char *source)
  {
    if (
        !b2_reverse_recovery_enabled_ ||
        !have_target_ ||
        !have_odom_ ||
        !reverse_recovery_request_pub_)
    {
      return false;
    }

    bool current_footprint_occupied = false;
    bool forward_probe_occupied = false;
    hasB2ImmediateDynamicObstacle(
        current_footprint_occupied,
        forward_probe_occupied);
    const bool newly_confirmed_obstacle_trap =
        shouldTriggerB2ReverseRecovery(
            replan_fail_count_,
            b2_reverse_recovery_min_replan_failures_,
            current_footprint_occupied,
            forward_probe_occupied,
            safety_execution_frozen_);
    const bool continue_confirmed_obstacle_recovery =
        shouldContinueLatchedB2ObstacleRecovery(
            b2_obstacle_recovery_latched_,
            replan_fail_count_,
            b2_reverse_recovery_min_replan_failures_,
            current_footprint_occupied,
            forward_probe_occupied,
            safety_execution_frozen_);
    if (!newly_confirmed_obstacle_trap &&
        !continue_confirmed_obstacle_recovery)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[B2 reverse recovery] %s replan failed %d time(s), but no "
          "immediate dynamic-obstacle trap was confirmed "
          "(current=%s forward=%s safety_frozen=%s). Keep forward replanning; "
          "ground/slope/direction/optimizer failures may not command reverse.",
          source,
          replan_fail_count_,
          current_footprint_occupied ? "blocked" : "clear",
          forward_probe_occupied ? "blocked" : "clear",
          safety_execution_frozen_ ? "true" : "false");
      return false;
    }

    if (!b2_obstacle_recovery_latched_)
    {
      b2_recovery_waypoints_.clear();
      b2_recovery_waypoint_yaws_.clear();
      b2_recovery_waypoint_simultaneous_yaw_.clear();
    }
    b2_obstacle_recovery_latched_ = true;
    reverse_recovery_requested_ = true;
    reverse_recovery_active_ = false;
    reverse_recovery_last_request_seconds_ = node_->now().seconds();
    reverse_recovery_request_pub_->publish(
        std_msgs::msg::Bool().set__data(true));
    ROS_WARN(
        "[B2 reverse recovery] %s forward replan failed. Request a "
        "generation-fresh rear obstacle/ground preflight for one "
        "straight-only reverse round; the active target and unlimited retry "
        "loop are retained.",
        source);
    changeFSMExecState(REVERSE_RECOVERY, source);
    return true;
  }

  bool SCANReplanFSM::hasB2ImmediateDynamicObstacle(
      bool &current_footprint_occupied,
      bool &forward_probe_occupied) const
  {
    current_footprint_occupied = false;
    forward_probe_occupied = false;
    if (!have_odom_ || !planner_manager_ || !planner_manager_->grid_map_)
      return false;

    const Eigen::Vector3d start = getPlanningStartPosition();
    const double yaw = getOdomYaw();
    current_footprint_occupied =
        isExecutionPoseDynamicallyOccupied(start, yaw);

    const Eigen::Vector3d forward(
        std::cos(yaw), std::sin(yaw), 0.0);
    for (
        double distance = b2_reverse_recovery_obstacle_probe_step_;
        distance <=
            b2_reverse_recovery_obstacle_probe_distance_ + 1e-9;
        distance += b2_reverse_recovery_obstacle_probe_step_)
    {
      if (isExecutionPoseDynamicallyOccupied(
              start + distance * forward, yaw))
      {
        forward_probe_occupied = true;
        break;
      }
    }
    return current_footprint_occupied || forward_probe_occupied;
  }

  Eigen::Vector3d SCANReplanFSM::getPlanningStartPosition() const
  {
    Eigen::Vector3d planning_start = odom_pos_;
    planning_start(2) += start_height_offset_;
    return planning_start;
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  Eigen::Vector2d SCANReplanFSM::getReferenceForwardDirection() const
  {
    if (b2_recovery_subgoal_active_)
    {
      const Eigen::Vector2d recovery_leg =
          (local_target_pt_ - getPlanningStartPosition()).head<2>();
      if (recovery_leg.norm() > 1e-4)
        return recovery_leg;
    }

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
        active_waypoints_.size() >= 2)
    {
      const Eigen::Vector2d direction = referenceForwardDirection(
          active_waypoints_,
          reference_progress_segment_,
          reference_progress_ratio_);
      if (direction.norm() > 1e-4)
        return direction;
    }

    return (end_pt_ - getPlanningStartPosition()).head<2>();
  }

  bool SCANReplanFSM::applyB2ForwardStartSeed()
  {
    if (b2_allow_reverse_ ||
        navi_mode_ != NAVI_MODE::REFERENCE_PATH ||
        !have_odom_)
      return false;

    B2ForwardSeedConfig body_seed_config = b2_forward_seed_config_;
    const double maximum_reference_error = std::max(
        0.0,
        std::min(
            b2_direction_guard_config_.maximum_reference_heading_error,
            3.14159265358979323846));
    body_seed_config.minimum_reference_alignment = std::max(
        body_seed_config.minimum_reference_alignment,
        std::cos(maximum_reference_error));

    const Eigen::Vector2d reference_direction =
        getReferenceForwardDirection();
    Eigen::Vector3d seed_velocity = Eigen::Vector3d::Zero();
    bool seeded = makeB2ForwardSeedVelocity(
        start_pt_,
        getOdomYaw(),
        reference_direction,
        body_seed_config,
        [this](const Eigen::Vector3d &position, double yaw) {
          return !isExecutionPoseOccupied(position, yaw);
        },
        seed_velocity);

    if (b2_obstacle_recovery_latched_)
    {
      // After a bounded reverse round the short fixed-yaw corridor can be
      // clear even though the retained reference is still blocked.  A zero
      // derivative lets the position-only B-spline choose an arbitrary first
      // tangent; on B2 that can rotate the long footprint over an unsupported
      // edge and makes the first three control points fail forever.  Preserve
      // the live body heading only after the complete 0.25 m corridor has
      // passed the normal obstacle + ground footprint validator.  If it is
      // not safe, keep zero velocity and let another bounded recovery round
      // run; never substitute an unverified reference-heading turn here.
      start_acc_.setZero();
      if (seeded)
      {
        start_vel_ = seed_velocity;
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[B2 motion] Latched obstacle recovery: seed the replacement "
            "trajectory along the verified current-heading escape corridor "
            "(yaw=%.3f speed=%.3fm/s clearance=%.3fm).",
            getOdomYaw(), start_vel_.head<2>().norm(),
            body_seed_config.clearance_distance);
        return true;
      }

      start_vel_.setZero();
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[B2 motion] Latched obstacle recovery: current-heading corridor "
          "is not safe; keep zero boundary velocity.");
      return false;
    }

    if (seeded)
    {
      start_vel_ = seed_velocity;
      start_acc_.setZero();
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[B2 motion] Seed stopped replan along current body heading: "
          "yaw=%.3f speed=%.3fm/s clearance=%.3fm.",
          getOdomYaw(), start_vel_.head<2>().norm(),
          b2_forward_seed_config_.clearance_distance);
      return true;
    }

    seeded = makeB2ReferenceSeedVelocity(
        start_pt_,
        getOdomYaw(),
        reference_direction,
        b2_forward_seed_config_,
        [this](const Eigen::Vector3d &position, double yaw) {
          return !isExecutionPoseOccupied(position, yaw);
        },
        seed_velocity,
        execution_validation_max_yaw_step_);
    if (!seeded)
      return false;

    start_vel_ = seed_velocity;
    start_acc_.setZero();
    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[B2 motion] Seed stopped replan along the outgoing reference "
        "direction after a verified in-place turn: yaw=%.3f "
        "speed=%.3fm/s clearance=%.3fm.",
        std::atan2(start_vel_.y(), start_vel_.x()),
        start_vel_.head<2>().norm(),
        b2_forward_seed_config_.clearance_distance);
    return true;
  }

  bool SCANReplanFSM::isB2TrajectoryDirectionSafe(
      UniformBspline &trajectory,
      bool enforce_initial_reference_cone,
      bool allow_bounded_obstacle_detour) const
  {
    const double duration = trajectory.getTimeSum();
    if (!std::isfinite(duration) || duration <= 1e-6)
      return false;

    std::vector<Eigen::Vector3d> path;
    constexpr double kSampleTime = 0.02;
    path.reserve(
        static_cast<size_t>(std::ceil(duration / kSampleTime)) + 1);
    for (double time = 0.0; time < duration; time += kSampleTime)
      path.push_back(trajectory.evaluateDeBoorT(time));
    path.push_back(trajectory.evaluateDeBoorT(duration));

    Eigen::Vector3d live_start = odom_pos_;
    live_start(2) = path.front()(2);
    if ((live_start - path.front()).head<2>().squaredNorm() > 1e-12)
      path.insert(path.begin(), live_start);
    else
      path.front() = live_start;

    const Eigen::Vector2d target_direction =
        local_target_pt_.head<2>() - path.front().head<2>();
    return isB2ForwardCandidate(
        path,
        target_direction,
        getReferenceForwardDirection(),
        b2_direction_guard_config_,
        enforce_initial_reference_cone,
        allow_bounded_obstacle_detour);
  }

  bool SCANReplanFSM::isExecutionPoseOccupied(
      const Eigen::Vector3d &position, double yaw) const
  {
    auto map = planner_manager_->grid_map_;
    if (!map->isGroundSupported(position, yaw))
      return true;
    return isExecutionPoseDynamicallyOccupied(position, yaw);
  }

  bool SCANReplanFSM::isExecutionPoseDynamicallyOccupied(
      const Eigen::Vector3d &position, double yaw) const
  {
    auto map = planner_manager_->grid_map_;
    if (map->getDynamicInflateOccupancy(position, yaw) != 0)
      return true;

    // The inflated cloud is visualized at voxel centres, while a continuous
    // circle centre can lie anywhere inside the voxel. Probe a small disk
    // around the body pose so this query matches the downstream monitor's
    // half-voxel path-corridor check.
    if (execution_validation_probe_radius_ <= 1e-6)
      return false;

    constexpr double kTwoPi = 6.28318530717958647692;
    for (int ring = 1; ring <= 2; ++ring)
    {
      const double radius =
          execution_validation_probe_radius_ * static_cast<double>(ring) / 2.0;
      for (int index = 0; index < execution_validation_probe_count_; ++index)
      {
        const double angle =
            kTwoPi * static_cast<double>(index) /
            static_cast<double>(execution_validation_probe_count_);
        Eigen::Vector3d probe = position;
        probe(0) += radius * std::cos(angle);
        probe(1) += radius * std::sin(angle);
        // Ground support belongs to the physical body pose and was checked
        // once above.  These small probes only bridge occupancy-voxel
        // discretization and must not rerun/expand the complete footprint.
        if (map->getDynamicInflateOccupancy(probe, yaw) != 0)
          return true;
      }
    }
    return false;
  }

  bool SCANReplanFSM::isTrajectorySafeForExecution(
      UniformBspline &trajectory,
      std::vector<double> *validated_yaw_schedule,
      double *validated_yaw_dt,
      bool use_recovery_yaw_contract,
      double recovery_start_yaw,
      double recovery_end_yaw,
      bool recovery_yaw_changes_with_translation)
  {
    if (!have_odom_)
      return false;
    if (!isExecutionMapReady())
      return false;

    const double duration = trajectory.getTimeSum();
    if (!std::isfinite(duration) || duration <= 1e-6)
      return false;

    // Match the path sampling published by closed_loop_controller. The
    // segment loop below then applies the same 5 cm / 5 degree continuous
    // sweep density as dynamic_avoidance_monitor.
    const std::size_t interval_count =
        b2YawSampleIntervalCount(
            duration, execution_validation_path_sample_dt_);
    if (interval_count == 0)
      return false;

    std::vector<Eigen::Vector3d> trajectory_path;
    trajectory_path.reserve(interval_count + 1);
    for (std::size_t index = 0; index <= interval_count; ++index)
    {
      const double time =
          duration * static_cast<double>(index) /
          static_cast<double>(interval_count);
      trajectory_path.push_back(trajectory.evaluateDeBoorT(time));
    }
    if (trajectory_path.size() < 2)
      return false;

    const B2PathGeometryCheck geometry_check =
        checkB2PathGeometry(
            trajectory_path, b2_maximum_path_slope_);
    if (!geometry_check.valid)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[B2 motion] Reject trajectory before publish: invalid terrain "
          "segment=%zu dxy=%.6fm dz=%.6fm slope=%.3f limit=%.3f.",
          geometry_check.segment,
          geometry_check.horizontal_distance,
          geometry_check.vertical_distance,
          geometry_check.slope,
          b2_maximum_path_slope_);
      return false;
    }

    std::vector<double> trajectory_yaws;
    if (use_recovery_yaw_contract)
    {
      if (!std::isfinite(recovery_start_yaw) ||
          !std::isfinite(recovery_end_yaw))
        return false;

      trajectory_yaws.reserve(trajectory_path.size());
      const double yaw_delta = normalizeB2Yaw(
          recovery_end_yaw - recovery_start_yaw);
      const Eigen::Vector2d chord =
          (trajectory_path.back() - trajectory_path.front()).head<2>();
      const double chord_length_squared = chord.squaredNorm();
      for (const Eigen::Vector3d &position : trajectory_path)
      {
        double progress = 1.0;
        if (recovery_yaw_changes_with_translation &&
            chord_length_squared > 1e-9)
        {
          progress =
              (position - trajectory_path.front())
                  .head<2>()
                  .dot(chord) /
              chord_length_squared;
          progress = std::max(0.0, std::min(1.0, progress));
        }
        trajectory_yaws.push_back(
            normalizeB2Yaw(
                recovery_start_yaw + progress * yaw_delta));
      }
    }
    else
    {
      trajectory_yaws =
          makeB2YawSchedule(trajectory_path, getOdomYaw());
    }
    if (trajectory_yaws.size() != trajectory_path.size())
      return false;

    // The controller publishes the remaining B-spline, and the monitor
    // prepends the live robot pose whenever tracking error separates it from
    // the first spline point.  Validate that connection as well; otherwise a
    // safe spline can still be reached through an unsafe first chord.
    std::vector<Eigen::Vector3d> path = trajectory_path;
    Eigen::Vector3d live_start = odom_pos_;
    live_start(2) = path.front()(2);
    std::vector<double> point_yaws;
    if (use_recovery_yaw_contract)
    {
      const bool prepend_live_pose =
          (live_start - path.front()).head<2>().squaredNorm() > 1e-12 ||
          std::abs(normalizeB2Yaw(
              trajectory_yaws.front() - getOdomYaw())) > 1e-6;
      if (prepend_live_pose)
      {
        path.insert(path.begin(), live_start);
        point_yaws.reserve(trajectory_yaws.size() + 1);
        point_yaws.push_back(getOdomYaw());
        point_yaws.insert(
            point_yaws.end(),
            trajectory_yaws.begin(),
            trajectory_yaws.end());
      }
      else
      {
        path.front() = live_start;
        point_yaws = trajectory_yaws;
      }
    }
    else
    {
      if ((live_start - path.front()).head<2>().squaredNorm() > 1e-12)
        path.insert(path.begin(), live_start);
      else
        path.front() = live_start;
      point_yaws = makeB2YawSchedule(path, getOdomYaw());
    }
    if (point_yaws.size() != path.size())
      return false;

    auto map = planner_manager_->grid_map_;
    if (!map->isGroundSupported(path.front(), point_yaws.front()))
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[ground support] Reject trajectory: current double-circle "
          "footprint has no complete same-floor ground support.");
      return false;
    }

    const bool occupied_at_start =
        execution_validation_enabled_ &&
        isExecutionPoseDynamicallyOccupied(
            path.front(), point_yaws.front());
    if (occupied_at_start && !execution_validation_allow_occupied_start_)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[execution validation] Reject trajectory: current double-circle "
          "footprint is occupied.");
      return false;
    }

    bool escaped_start_occupancy = !occupied_at_start;
    double travelled = 0.0;
    for (size_t segment = 0; segment + 1 < path.size(); ++segment)
    {
      const Eigen::Vector3d delta = path[segment + 1] - path[segment];
      const double distance = delta.norm();
      const double yaw_delta =
          normalizeB2Yaw(point_yaws[segment + 1] - point_yaws[segment]);
      const size_t position_samples = static_cast<size_t>(
          std::ceil(distance / execution_validation_max_position_step_));
      const size_t yaw_samples = static_cast<size_t>(
          std::ceil(
              std::abs(yaw_delta) /
              execution_validation_max_yaw_step_));
      const size_t sample_count =
          std::max<size_t>(1, std::max(position_samples, yaw_samples));

      for (size_t sample = 0; sample <= sample_count; ++sample)
      {
        if (segment > 0 && sample == 0)
          continue;
        const double ratio =
            static_cast<double>(sample) / static_cast<double>(sample_count);
        const Eigen::Vector3d position = path[segment] + ratio * delta;
        const double yaw = normalizeB2Yaw(
            point_yaws[segment] + ratio * yaw_delta);
        const double sample_travelled = travelled + ratio * distance;
        if (!map->isGroundSupported(position, yaw))
        {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "[ground support] Reject trajectory before publish: "
              "unsupported double-circle footprint at s=%.3fm "
              "xyz=(%.3f, %.3f, %.3f) yaw=%.3f.",
              sample_travelled, position(0), position(1), position(2), yaw);
          return false;
        }
        const bool occupied =
            execution_validation_enabled_ &&
            isExecutionPoseDynamicallyOccupied(position, yaw);

        if (occupied)
        {
          const bool allowed_start_escape =
              occupied_at_start &&
              !escaped_start_occupancy &&
              sample_travelled <=
                  execution_validation_start_escape_distance_;
          if (!allowed_start_escape)
          {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[execution validation] Reject trajectory before publish: "
                "double-circle collision at s=%.3fm xyz=(%.3f, %.3f, %.3f) "
                "yaw=%.3f.",
                sample_travelled, position(0), position(1), position(2), yaw);
            return false;
          }
        }
        else if (occupied_at_start && !escaped_start_occupancy)
        {
          escaped_start_occupancy = true;
        }
      }
      travelled += distance;
    }

    if (occupied_at_start && !escaped_start_occupancy)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[execution validation] Reject trajectory: it never leaves the "
          "current occupied footprint.");
      return false;
    }
    if (validated_yaw_schedule)
      *validated_yaw_schedule = trajectory_yaws;
    if (validated_yaw_dt)
      *validated_yaw_dt =
          duration / static_cast<double>(interval_count);
    return true;
  }

  bool SCANReplanFSM::isExecutionMapReady()
  {
    if (!planner_manager_->grid_map_->isGroundSupportReady())
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[ground support] Wait for the transient-local /mapground index; "
          "planning is fail-closed.");
      return false;
    }

    if (!execution_validation_enabled_)
      return true;

    const auto observation_count =
        planner_manager_->grid_map_->getObstacleObservationCount();
    if (observation_count >=
        static_cast<uint64_t>(execution_validation_min_map_updates_))
      return true;

    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[execution validation] Wait for the local obstacle map to warm up "
        "(%llu/%d sensor observations).",
        static_cast<unsigned long long>(observation_count),
        execution_validation_min_map_updates_);
    return false;
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    center += self_double_cylinder_center_offset_ * heading;
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {
      "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ",
      "EXEC_TRAJ", "REVERSE_RECOVERY", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {
      "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ",
      "EXEC_TRAJ", "REVERSE_RECOVERY", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      if (finishReferencePathIfGoalReached("GEN_NEW_TRAJ"))
        break;
      if (!replanAttemptReady())
        break;
      // Map warm-up is not a planning failure. Waiting here prevents the
      // 100 Hz FSM from exhausting max_replan_fail_count and replacing a
      // valid GoTo request with a stationary emergency spline.
      if (!isExecutionMapReady())
        break;
      if (global_target_ground_validation_pending_ &&
          !adjustGlobalTargetIfOccupied())
      {
        replan_fail_count_++;
        deferReplanAttempt();
        break;
      }

      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        replan_not_before_seconds_ = 0.0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        if (!requestB2ReverseRecovery("GEN_NEW_TRAJ"))
        {
          deferReplanAttempt();
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if (finishReferencePathIfGoalReached("REPLAN_TRAJ"))
        break;
      if (!replanAttemptReady())
        break;
      if (!isExecutionMapReady())
        break;
      if (global_target_ground_validation_pending_ &&
          !adjustGlobalTargetIfOccupied())
      {
        replan_fail_count_++;
        deferReplanAttempt();
        break;
      }

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        replan_not_before_seconds_ = 0.0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        if (!requestB2ReverseRecovery("REPLAN_TRAJ"))
        {
          deferReplanAttempt();
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }

      break;
    }

    case REVERSE_RECOVERY:
    {
      if (!have_target_)
      {
        reverse_recovery_requested_ = false;
        reverse_recovery_active_ = false;
        if (reverse_recovery_request_pub_)
          reverse_recovery_request_pub_->publish(
              std_msgs::msg::Bool().set__data(false));
        changeFSMExecState(WAIT_TARGET, "REVERSE_NO_TARGET");
        break;
      }

      if (
          reverse_recovery_requested_ &&
          !reverse_recovery_active_ &&
          node_->now().seconds() -
              reverse_recovery_last_request_seconds_ >=
              b2_reverse_recovery_request_period_)
      {
        reverse_recovery_last_request_seconds_ = node_->now().seconds();
        reverse_recovery_request_pub_->publish(
            std_msgs::msg::Bool().set__data(true));
      }
      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
      const bool reference_terminal_trajectory =
          navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
          !active_waypoints_.empty() &&
          (local_target_pt_ - active_waypoints_.back())
                  .head<2>()
                  .norm() < 1e-3;
      const double reference_final_goal_distance =
          navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
                  !active_waypoints_.empty()
              ? (active_waypoints_.back() - odom_pos_).head<2>().norm()
              : std::numeric_limits<double>::infinity();

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (b2_recovery_subgoal_active_)
        {
          // The recovery spline stops exactly on a footprint-validated
          // waypoint. Nominal spline time is not proof that the physical
          // body reached it: the controller can still have several
          // centimetres of tracking error. Replanning from that offset can
          // request a new yaw sweep outside the checked ground corridor.
          // Keep the old endpoint command active until odometry has actually
          // converged, then prune/switch the recovery leg.
          constexpr double kRecoveryEndpointTolerance = 0.03;
          constexpr double kRecoveryBoundedOvershootTolerance = 0.105;
          const Eigen::Vector2d recovery_endpoint_residual =
              (local_target_pt_ - odom_pos_).head<2>();
          const double recovery_endpoint_error =
              recovery_endpoint_residual.norm();
          if (!shouldAdvanceB2RecoveryLeg(
                  recovery_endpoint_residual,
                  getOdomYaw(),
                  kRecoveryEndpointTolerance,
                  kRecoveryBoundedOvershootTolerance))
          {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[B2 motion] Recovery leg time elapsed with %.3fm "
                "tracking error; endpoint remains reachable in the forward "
                "body half-plane, so keep converging to %.3fm.",
                recovery_endpoint_error,
                kRecoveryEndpointTolerance);
            return;
          }
          if (recovery_endpoint_error >
              kRecoveryEndpointTolerance)
          {
            RCLCPP_INFO(
                node_->get_logger(),
                "[B2 motion] Accept %.3fm bounded forward overshoot at "
                "recovery waypoint; rebase and validate the next leg from "
                "live odometry instead of commanding reverse.",
                recovery_endpoint_error);
          }
        }

        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        // A REFERENCE_PATH trajectory is only a bounded local lookahead.  In
        // particular, collision fallback can deliberately end it at an
        // earlier free point.  Finishing that spline is not equivalent to
        // reaching the final point of /initial_path: keep the original target
        // and build the next bounded segment from the current body position.
        if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && !active_waypoints_.empty())
        {
          if (reference_final_goal_distance > final_goal_tolerance_)
          {
            if (reference_terminal_trajectory)
            {
              // The terminal spline already gives the controller a valid
              // endpoint. Once its nominal time expires, keep tracking that
              // endpoint until the body actually enters the final XY
              // tolerance. Replanning here asks reboundReplan for a sub-0.2m
              // spline, which it intentionally reports as "Close to goal";
              // treating that no-op as failure used to trigger a pointless
              // reverse recovery away from the goal.
              RCLCPP_INFO_THROTTLE(
                  node_->get_logger(), *node_->get_clock(), 1000,
                  "[reference path] Terminal trajectory time elapsed %.3fm "
                  "before final goal; keep endpoint tracking instead of "
                  "replanning or reversing.",
                  reference_final_goal_distance);
              return;
            }
            ROS_INFO("[reference path] Local trajectory finished %.3fm before final goal; continue planning.",
                     reference_final_goal_distance);
            have_target_ = true;
            have_new_target_ = true;
            changeFSMExecState(GEN_NEW_TRAJ, "REFERENCE_CONTINUE");
            return;
          }

          finishReferencePathIfGoalReached("EXEC_TRAJ");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if (
          reference_terminal_trajectory &&
          reference_final_goal_distance <= 0.20)
      {
        // reboundReplan deliberately declines trajectories shorter than
        // 0.20m. The already published terminal spline is the correct command
        // in this convergence band, so do not convert that no-op into a
        // recovery failure.
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          ROS_INFO("Exiting EMERGENCY_STOP. Switching to WAIT_TARGET. Need a new target point.");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    // A planning-attempt count is diagnostic only.  An active navigation
    // target must survive any number of forward-plan/reverse-recovery rounds;
    // only independent safety faults are allowed to stop execution.
    const bool keep_active_target = have_target_;
    if (
        (controller_execution_frozen_ || keep_active_target) &&
        max_replan_fail_count_ > 0 &&
        replan_fail_count_ >= max_replan_fail_count_)
    {
      ROS_WARN(
          "Replan failed %d times; keep the active GoTo target and continue bounded retries.",
          replan_fail_count_);
      replan_fail_count_ = 0;
      return;
    }

    if (shouldEscalateReplanFailure(
            replan_fail_count_,
            max_replan_fail_count_,
            controller_execution_frozen_,
            keep_active_target))
    {
      ROS_WARN("Replan failed %d times. Emergency stop and wait for a new target.", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::finishReferencePathIfGoalReached(const char *source)
  {
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || active_waypoints_.empty())
      return false;

    const double final_goal_distance =
        (active_waypoints_.back() - odom_pos_).head<2>().norm();
    if (final_goal_distance > final_goal_tolerance_)
      return false;

    ROS_INFO("[reference path] Final XY goal reached in %s: distance=%.3fm tolerance=%.3fm; wait for the next target.",
             source, final_goal_distance, final_goal_tolerance_);
    active_waypoints_.clear();
    reference_progress_segment_ = 0;
    reference_progress_ratio_ = 0.0;
    have_target_ = false;
    have_new_target_ = false;
    replan_fail_count_ = 0;
    changeFSMExecState(WAIT_TARGET, source);
    return true;
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = getPlanningStartPosition();
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    if (controller_execution_frozen_)
    {
      start_vel_.setZero();
      start_acc_.setZero();
      applyB2ForwardStartSeed();
    }

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    // Reference-path mode must keep using the original global polyline.  The
    // previous implementation rebuilt a direct polynomial to the final goal
    // on every local replan; after repeated frozen replans that polynomial
    // drifted away from the supplied path and produced jumping local targets.
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&
        !planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      ROS_ERROR("[navi_mode=%d] Unable to refresh global trajectory from odom to current target.", navi_mode_);
      return false;
    }

    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH && !adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = getPlanningStartPosition();
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    if (controller_execution_frozen_ ||
        (navi_mode_ == NAVI_MODE::REFERENCE_PATH && have_new_target_))
    {
      start_vel_.setZero();
      start_acc_.setZero();
      applyB2ForwardStartSeed();
      return;
    }

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    if (controller_execution_frozen_)
    {
      start_vel_.setZero();
      start_acc_.setZero();
      return;
    }

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();

    // While the controller rotates in place it intentionally freezes the
    // active trajectory clock.  Replanning that same frozen first segment on
    // every collision timer tick makes the desired heading move continuously.
    // Dynamic avoidance still owns the independent hard safety freeze.
    if (controller_execution_frozen_)
      return;

    // A verified recovery leg may intentionally rotate while translating.
    // This legacy checker derives yaw from the position chord, so it would
    // evaluate a different double-circle footprint and continuously replace
    // a safe explicit-yaw leg. The full leg was swept before publication and
    // dynamic_avoidance_monitor continues to validate the live execution
    // path with the published yaw schedule.
    if (b2_recovery_subgoal_active_)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();
    const bool use_verified_b2_recovery_leg =
        b2_obstacle_recovery_latched_ &&
        b2_recovery_subgoal_active_;

    // reboundReplan commits its result to local_data_ before returning.
    // Retain the controller's currently active trajectory so a candidate
    // rejected by the execution safety sweep cannot poison the next retry.
    const LocalTrajData previous_local_traj = planner_manager_->local_data_;
    // First try the verified dense reference.  If its smoothed B-spline is
    // rejected, GEN_NEW_TRAJ/planFromCurrentTraj retries with
    // flag_randomPolyTraj=true.  Do not pass the same guide on that retry:
    // manager would otherwise ignore the random/A* fallback and regenerate
    // the identical unsafe candidate forever while the robot is held still.
    const std::vector<Eigen::Vector3d> *reference_guide =
        navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
                !flag_randomPolyTraj &&
                local_reference_guide_.size() >= 2
            ? &local_reference_guide_
            : nullptr;
    bool plan_success = use_verified_b2_recovery_leg
                            ? planner_manager_->planVerifiedB2RecoveryLeg(
                                  start_pt_, local_target_pt_)
                            : planner_manager_->reboundReplan(
                                  start_pt_, start_vel_, start_acc_,
                                  local_target_pt_, local_target_vel_,
                                  (have_new_target_ || flag_use_poly_init),
                                  flag_randomPolyTraj,
                                  reference_guide,
                                  false);
    bool used_verified_reference_rejoin_leg = false;
    double reference_rejoin_start_yaw = getOdomYaw();
    double reference_rejoin_end_yaw = reference_rejoin_start_yaw;
    bool reference_rejoin_simultaneous_yaw = false;
    std::size_t reference_rejoin_target_index = 0;
    std::size_t reference_rejoin_route_pose_count = 0;
    const char *reference_rejoin_reason = "guided planner failure";
    const char *reference_rejoin_motion = "stop-turn-forward";
    auto tryVerifiedReferenceRejoin =
        [&](const char *reason) {
      if (reference_guide == nullptr ||
          use_verified_b2_recovery_leg ||
          used_verified_reference_rejoin_leg)
      {
        return false;
      }

      // Rebound/A* can fail even while the supplied reference corridor is
      // usable: its control-point repairs may leave that corridor or enter a
      // live obstacle. A dynamically blocked guide must never be shortened to
      // its last free point: doing so deliberately drives B2 up to the
      // obstacle and removes the room needed to steer around it. In that
      // case, search only for a forward detour that rejoins beyond the full
      // blocked section. The simple swept stop-turn-forward connector remains
      // available for non-obstacle optimizer failures.
      reference_rejoin_start_yaw = getOdomYaw();
      Eigen::Vector3d reference_rejoin_target;
      std::size_t first_dynamically_blocked_segment = 0;
      std::size_t last_dynamically_blocked_segment = 0;
      const bool reference_dynamically_blocked =
          findB2ReferenceDynamicBlockage(
              *reference_guide,
              execution_validation_max_position_step_,
              [this](const Eigen::Vector3d &position, double yaw) {
                return isExecutionPoseDynamicallyOccupied(position, yaw);
              },
              &first_dynamically_blocked_segment,
              &last_dynamically_blocked_segment);
      if (reference_dynamically_blocked)
      {
        // This is stronger evidence than the downstream execution-path Bool:
        // the latter can become a spatially mismatched one-point path after
        // the last short recovery leg and incorrectly report "clear".
        // Latch the obstacle here so repeated planning failure plus SCAN's
        // own blocked forward probe can request the existing straight-only
        // reverse preflight without depending on that collapsed path.
        b2_obstacle_recovery_latched_ = true;
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[reference path] Dynamic obstacle blocks guide segments "
            "%zu..%zu; latch obstacle recovery, skip the last-safe-point "
            "connector, and search a forward bypass that rejoins beyond the "
            "obstacle. If no bypass remains and the forward probe is blocked, "
            "the straight reverse preflight no longer depends on a reduced "
            "execution path reporting safety_frozen.",
            first_dynamically_blocked_segment,
            last_dynamically_blocked_segment);
      }

      bool found_rejoin = false;
      reference_rejoin_simultaneous_yaw = false;
      reference_rejoin_route_pose_count = 0;
      reference_rejoin_motion = "stop-turn-forward";

      if (!reference_dynamically_blocked)
      {
        found_rejoin = findFarthestSafeB2ReferenceLeg(
            start_pt_,
            reference_rejoin_start_yaw,
            *reference_guide,
            0.20,
            execution_validation_max_yaw_step_,
            execution_validation_max_position_step_,
            [this](const Eigen::Vector3d &position, double yaw) {
              return !isExecutionPoseOccupied(position, yaw);
            },
            reference_rejoin_target,
            &reference_rejoin_target_index);
        reference_rejoin_route_pose_count = found_rejoin ? 2 : 0;
      }

      if (!found_rejoin)
      {
        B2ForwardDetourSearchConfig search_config;
        search_config.maximum_yaw_step =
            execution_validation_max_yaw_step_;
        search_config.maximum_position_step =
            execution_validation_max_position_step_;
        // The controller's position tolerance is 0.10 m.  A 0.15 m straight
        // lattice primitive (and the existing 0.20 m steering primitive)
        // guarantees that the first verified leg cannot be pruned at its
        // starting pose.
        search_config.translation_step =
            std::max(0.15, search_config.translation_step);

        std::vector<Eigen::Vector3d> steering_route;
        std::vector<double> steering_yaws;
        std::vector<bool> steering_simultaneous_yaw;
        double last_search_distance =
            std::numeric_limits<double>::infinity();
        for (std::size_t reverse_index = reference_guide->size();
             reverse_index > 1;
             --reverse_index)
        {
          const std::size_t candidate_index = reverse_index - 1;
          if (reference_dynamically_blocked &&
              candidate_index <= last_dynamically_blocked_segment)
          {
            continue;
          }
          const Eigen::Vector3d &candidate =
              (*reference_guide)[candidate_index];
          const double candidate_distance =
              (candidate - start_pt_).head<2>().norm();
          if (candidate_distance < 0.20)
            continue;

          // Try the complete bounded lookahead first, followed by at most
          // one target per 0.5 m while walking back toward the robot.  This
          // keeps failure latency bounded without overlooking a useful
          // earlier point when the far target lies beyond a tight bend.
          if (std::isfinite(last_search_distance) &&
              last_search_distance - candidate_distance < 0.50 &&
              candidate_index > 1)
          {
            continue;
          }
          last_search_distance = candidate_distance;
          steering_route.clear();
          steering_yaws.clear();
          steering_simultaneous_yaw.clear();
          if (!searchB2ForwardDetour(
                  start_pt_,
                  reference_rejoin_start_yaw,
                  candidate,
                  search_config,
                  [this](
                      const Eigen::Vector3d &position,
                      double yaw) {
                    return !isExecutionPoseOccupied(position, yaw);
                  },
                  steering_route,
                  &steering_yaws,
                  &steering_simultaneous_yaw) ||
              steering_route.size() < 2 ||
              steering_yaws.size() != steering_route.size() ||
              steering_simultaneous_yaw.size() !=
                  steering_route.size() ||
              (steering_route[1] - start_pt_).head<2>().norm() <
                  0.105)
          {
            continue;
          }

          reference_rejoin_target = steering_route[1];
          reference_rejoin_end_yaw = steering_yaws[1];
          reference_rejoin_simultaneous_yaw =
              steering_simultaneous_yaw[1];
          reference_rejoin_target_index = candidate_index;
          reference_rejoin_route_pose_count = steering_route.size();
          reference_rejoin_motion =
              reference_rejoin_simultaneous_yaw
                  ? "forward-steering"
                  : "stop-turn-forward";
          found_rejoin = true;
          break;
        }
      }

      if (!found_rejoin)
        return false;

      if (reference_rejoin_route_pose_count == 2)
      {
        const Eigen::Vector2d rejoin_delta =
            (reference_rejoin_target - start_pt_).head<2>();
        reference_rejoin_end_yaw =
            std::atan2(rejoin_delta.y(), rejoin_delta.x());
      }

      local_target_pt_ = reference_rejoin_target;
      local_target_vel_.setZero();
      const bool rejoin_plan_success =
          planner_manager_->planVerifiedB2RecoveryLeg(
              start_pt_, local_target_pt_);
      if (!rejoin_plan_success)
        return false;

      reference_rejoin_reason = reason;
      used_verified_reference_rejoin_leg = true;
      // This is a bounded, fully swept steering primitive.  Reuse the
      // recovery-leg execution semantics so the legacy tangent-only
      // collision timer cannot replace its explicit yaw schedule halfway
      // through the manoeuvre, and wait for live odometry to reach its
      // endpoint before selecting the next reference segment.
      b2_recovery_subgoal_active_ = true;
      return true;
    };

    if (!plan_success)
      plan_success =
          tryVerifiedReferenceRejoin("guided planner failure");
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {
      std::vector<double> validated_yaw_schedule;
      double validated_yaw_dt = 0.0;
      bool candidate_accepted = false;
      for (int validation_attempt = 0;
           validation_attempt < 2;
           ++validation_attempt)
      {
        auto info = &planner_manager_->local_data_;
        bool enforce_initial_reference_cone = flag_randomPolyTraj;
        bool allow_bounded_obstacle_detour =
            b2_recovery_subgoal_active_ ||
            used_verified_reference_rejoin_leg;
        bool current_dynamic_obstacle = false;
        bool forward_dynamic_obstacle = false;
        const bool immediate_dynamic_obstacle =
            hasB2ImmediateDynamicObstacle(
                current_dynamic_obstacle,
                forward_dynamic_obstacle);
        if (shouldUseB2BoundedObstacleDetour(
                flag_randomPolyTraj,
                immediate_dynamic_obstacle,
                b2_obstacle_recovery_latched_))
        {
          // A verified obstacle bypass may have to begin almost sideways to
          // leave the blocked reference corridor. This is not reverse
          // motion: the controller first aligns the body with the candidate
          // tangent, then drives forward along it. Keep detour mode latched
          // for the complete safety-frozen replan.
          enforce_initial_reference_cone = false;
          allow_bounded_obstacle_detour = true;
        }
        const bool direction_safe =
            b2_allow_reverse_ ||
            navi_mode_ != NAVI_MODE::REFERENCE_PATH ||
            isB2TrajectoryDirectionSafe(
                info->position_traj_,
                enforce_initial_reference_cone,
                allow_bounded_obstacle_detour);
        if (!direction_safe)
        {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "[B2 motion] Reject %s candidate: initial motion leaves the "
              "forward/reference cone or backtracks "
              "(obstacle_detour=%s current=%s forward=%s).",
              flag_randomPolyTraj
                  ? "unguided fallback"
                  : "guided reference",
              allow_bounded_obstacle_detour ? "true" : "false",
              current_dynamic_obstacle ? "blocked" : "clear",
              forward_dynamic_obstacle ? "blocked" : "clear");
        }

        validated_yaw_schedule.clear();
        validated_yaw_dt = 0.0;
        const bool use_explicit_verified_leg_yaw =
            use_verified_b2_recovery_leg ||
            used_verified_reference_rejoin_leg;
        const bool execution_safe =
            direction_safe &&
            isTrajectorySafeForExecution(
                info->position_traj_,
                &validated_yaw_schedule,
                &validated_yaw_dt,
                use_explicit_verified_leg_yaw,
                used_verified_reference_rejoin_leg
                    ? reference_rejoin_start_yaw
                    : b2_recovery_leg_start_yaw_,
                used_verified_reference_rejoin_leg
                    ? reference_rejoin_end_yaw
                    : b2_recovery_leg_end_yaw_,
                used_verified_reference_rejoin_leg
                    ? reference_rejoin_simultaneous_yaw
                    : b2_recovery_leg_simultaneous_yaw_);
        if (direction_safe && execution_safe)
        {
          candidate_accepted = true;
          break;
        }

        // The guided optimizer may report success and only be rejected by
        // the downstream direction/footprint sweep.  Give that case the same
        // deterministic reference-rejoin path as an optimizer failure;
        // otherwise the next FSM tick switches to random A* and can spin for
        // hundreds of attempts despite a verified forward arc being present.
        if (!tryVerifiedReferenceRejoin(
                direction_safe
                    ? "execution safety rejection"
                    : "direction guard rejection"))
        {
          break;
        }
      }

      if (!candidate_accepted)
      {
        // Do not replace a potentially usable active trajectory with one
        // that the downstream double-circle safety sweep will immediately
        // freeze.
        planner_manager_->local_data_ = previous_local_traj;
        return false;
      }

      if (used_verified_reference_rejoin_leg)
      {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[reference path] %s; execute deterministic %s reference "
            "rejoin leg toward guide[%zu] at (%.2f, %.2f), route=%zu poses, "
            "after the full direction and configured execution-safety sweep.",
            reference_rejoin_reason,
            reference_rejoin_motion,
            reference_rejoin_target_index,
            local_target_pt_.x(),
            local_target_pt_.y(),
            reference_rejoin_route_pose_count);
      }

      /* publish traj */
      auto info = &planner_manager_->local_data_;
      scan_planner::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = toMsgTime(info->start_time_);
      bspline.traj_id = info->traj_id_;
      bspline.emergency_stop = false;
      bspline.terminal_goal =
          navi_mode_ == NAVI_MODE::REFERENCE_PATH
              ? !active_waypoints_.empty() &&
                    (local_target_pt_ - active_waypoints_.back())
                            .head<2>()
                            .norm() < 1e-3
              : (local_target_pt_ - end_pt_).head<2>().norm() < 1e-3;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }
      bspline.yaw_pts = validated_yaw_schedule;
      bspline.yaw_dt = validated_yaw_dt;

      bspline_pub_->publish(bspline);
      emergency_stop_state_pub_->publish(std_msgs::msg::Bool().set__data(false));
      // Recovery is released only after odometry reaches and prunes the
      // final verified waypoint. Publishing the last leg is not completion:
      // clearing the cache here used to return to ordinary SCAN while B2 was
      // still inside the obstacle corridor.

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = toMsgTime(info->start_time_);
    bspline.traj_id = info->traj_id_;
    bspline.emergency_stop = true;
    bspline.terminal_goal = false;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }
    const double duration = info->position_traj_.getTimeSum();
    if (std::isfinite(duration) && duration > 1e-6)
    {
      const double yaw = getOdomYaw();
      bspline.yaw_pts = {yaw, yaw};
      bspline.yaw_dt = duration;
    }

    bspline_pub_->publish(bspline);
    emergency_stop_state_pub_->publish(std_msgs::msg::Bool().set__data(true));

    return true;
  }

  bool SCANReplanFSM::getReferencePathLocalTarget()
  {
    local_reference_guide_.clear();
    b2_recovery_subgoal_active_ = false;
    b2_recovery_leg_start_yaw_ = getOdomYaw();
    b2_recovery_leg_end_yaw_ = getOdomYaw();
    b2_recovery_leg_simultaneous_yaw_ = false;
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || active_waypoints_.empty())
      return false;

    if (active_waypoints_.size() == 1)
    {
      local_target_pt_ = active_waypoints_.front();
      local_target_vel_.setZero();
      local_reference_guide_.push_back(start_pt_);
      local_reference_guide_.push_back(local_target_pt_);
      return true;
    }

    const size_t segment_count = active_waypoints_.size() - 1;
    const ReferenceProgress canonical_progress =
        canonicalReferenceProgress(
            reference_progress_segment_,
            reference_progress_ratio_,
            segment_count);
    reference_progress_segment_ = canonical_progress.segment;
    reference_progress_ratio_ = canonical_progress.ratio;

    const size_t search_begin = std::min(reference_progress_segment_, segment_count - 1);
    size_t best_segment = search_begin;
    double best_ratio = search_begin == reference_progress_segment_ ? reference_progress_ratio_ : 0.0;
    double best_distance_squared = std::numeric_limits<double>::infinity();

    // Project the current body position onto the remaining original polyline.
    // Progress is monotonic so a loop or localization jitter cannot send the
    // local target back to a previously completed segment.
    for (size_t i = search_begin; i < segment_count; ++i)
    {
      const Eigen::Vector3d segment = active_waypoints_[i + 1] - active_waypoints_[i];
      const double length_squared = segment.head<2>().squaredNorm();
      if (length_squared < 1e-8)
        continue;

      double ratio = (start_pt_ - active_waypoints_[i]).head<2>().dot(segment.head<2>()) / length_squared;
      ratio = std::max(0.0, std::min(1.0, ratio));
      if (i == reference_progress_segment_)
        ratio = std::max(ratio, reference_progress_ratio_);

      const Eigen::Vector3d projected = active_waypoints_[i] + ratio * segment;
      const double distance_squared = (projected - start_pt_).head<2>().squaredNorm();
      if (distance_squared < best_distance_squared)
      {
        best_distance_squared = distance_squared;
        best_segment = i;
        best_ratio = ratio;
      }
    }

    const size_t previous_progress_segment =
        reference_progress_segment_;
    if (best_segment > previous_progress_segment)
    {
      reference_progress_segment_ = best_segment;
      reference_progress_ratio_ = best_ratio;
    }
    else
    {
      reference_progress_ratio_ = std::max(reference_progress_ratio_, best_ratio);
    }

    const ReferenceProgress updated_progress =
        canonicalReferenceProgress(
            reference_progress_segment_,
            reference_progress_ratio_,
            segment_count);
    reference_progress_segment_ = updated_progress.segment;
    reference_progress_ratio_ = updated_progress.ratio;

    best_segment = reference_progress_segment_;
    best_ratio = reference_progress_ratio_;
    double remaining = std::max(0.5, planning_horizon_);
    size_t target_segment = best_segment;
    double target_ratio = best_ratio;
    bool reached_end = false;

    for (size_t i = best_segment; i < segment_count; ++i)
    {
      const Eigen::Vector3d segment = active_waypoints_[i + 1] - active_waypoints_[i];
      const double length = segment.head<2>().norm();
      if (length < 1e-4)
        continue;

      const double from_ratio = i == best_segment ? best_ratio : 0.0;
      const double available = (1.0 - from_ratio) * length;
      target_segment = i;
      if (remaining <= available)
      {
        target_ratio = from_ratio + remaining / length;
        remaining = 0.0;
        break;
      }

      remaining -= available;
      target_ratio = 1.0;
    }

    if (remaining > 1e-4)
    {
      target_segment = segment_count - 1;
      target_ratio = 1.0;
      reached_end = true;
    }

    auto targetFromSegment = [&](size_t segment_index, double ratio) {
      return active_waypoints_[segment_index] +
             ratio * (active_waypoints_[segment_index + 1] - active_waypoints_[segment_index]);
    };
    local_target_pt_ = targetFromSegment(target_segment, target_ratio);

    auto targetOccupancy = [&](const Eigen::Vector3d &point, size_t segment_index) {
      return planner_manager_->grid_map_->getInflateOccupancy(
          point,
          estimateYawFromSegment(active_waypoints_[segment_index], active_waypoints_[segment_index + 1]));
    };

    if (targetOccupancy(local_target_pt_, target_segment) != 0)
    {
      bool found_free_target = false;
      const double occupied_target_yaw = estimateYawFromSegment(
          active_waypoints_[target_segment],
          active_waypoints_[target_segment + 1]);
      const bool target_dynamically_occupied =
          isExecutionPoseDynamicallyOccupied(
              local_target_pt_, occupied_target_yaw);

      auto distanceFromProgressToWaypoint = [&](size_t waypoint_index) {
        if (waypoint_index <= best_segment)
          return 0.0;

        double distance =
            (1.0 - best_ratio) *
            (active_waypoints_[best_segment + 1] -
             active_waypoints_[best_segment]).head<2>().norm();
        for (size_t segment = best_segment + 1;
             segment < waypoint_index && segment < segment_count;
             ++segment)
        {
          distance +=
              (active_waypoints_[segment + 1] -
               active_waypoints_[segment]).head<2>().norm();
        }
        return distance;
      };

      // A point can itself be free while sitting immediately after an
      // obstacle. The optimizer then has too little runway to turn the full
      // double-circle footprint and its smoothed spline cuts the obstacle
      // corner. Require a continuous free approach at least as long as the
      // footprint's longitudinal reach before using a forward target.
      auto hasFreeApproachRun = [&](size_t waypoint_index) {
        if (waypoint_index == 0)
          return false;

        const double required_clearance = std::max(
            0.60,
            std::abs(self_double_cylinder_center_offset_) +
                std::abs(self_double_cylinder_offset_) +
                self_double_cylinder_radius_);
        double remaining = required_clearance;
        size_t segment_index =
            std::min(waypoint_index - 1, segment_count - 1);
        double segment_ratio = 1.0;

        while (remaining > 1e-4)
        {
          const Eigen::Vector3d segment =
              active_waypoints_[segment_index + 1] -
              active_waypoints_[segment_index];
          const double segment_length = segment.head<2>().norm();
          if (segment_length > 1e-4)
          {
            const double available = segment_ratio * segment_length;
            const double check_distance = std::min(remaining, available);
            const int sample_count = std::max(
                1,
                static_cast<int>(std::ceil(
                    check_distance /
                    execution_validation_max_position_step_)));
            const double yaw = estimateYawFromSegment(
                active_waypoints_[segment_index],
                active_waypoints_[segment_index + 1]);

            for (int sample = 0; sample <= sample_count; ++sample)
            {
              const double distance_back =
                  check_distance * static_cast<double>(sample) /
                  static_cast<double>(sample_count);
              const double ratio =
                  segment_ratio - distance_back / segment_length;
              const Eigen::Vector3d position =
                  active_waypoints_[segment_index] +
                  std::max(0.0, ratio) * segment;
              if (isExecutionPoseOccupied(position, yaw))
                return false;
            }
            remaining -= check_distance;
          }

          if (remaining <= 1e-4)
            return true;
          if (segment_index == best_segment)
            return false;
          --segment_index;
          segment_ratio = 1.0;
        }
        return true;
      };

      // Always look beyond an occupied target first.  The old earlier-point
      // preference repeatedly moved B2 to the last free sample immediately
      // in front of a dynamic obstacle, where its long footprint no longer
      // had enough steering room.  A target on the far side gives local A*
      // and the heading-aware recovery lattice an actual bypass objective.
      const double max_forward_distance =
          std::max(1.0, 2.0 * planning_horizon_);
      for (size_t waypoint_index = target_segment + 1;
           waypoint_index < active_waypoints_.size();
           ++waypoint_index)
      {
        const double forward_distance =
            distanceFromProgressToWaypoint(waypoint_index);
        if (forward_distance > max_forward_distance)
          break;

        const size_t candidate_segment =
            std::min(waypoint_index, segment_count - 1);
        const Eigen::Vector3d &candidate =
            active_waypoints_[waypoint_index];
        if (targetOccupancy(candidate, candidate_segment) != 0 ||
            !hasFreeApproachRun(waypoint_index))
          continue;

        local_target_pt_ = candidate;
        target_segment = candidate_segment;
        target_ratio =
            waypoint_index == active_waypoints_.size() - 1 ? 1.0 : 0.0;
        reached_end =
            waypoint_index == active_waypoints_.size() - 1;
        found_free_target = true;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[reference path] Bounded lookahead is occupied; use the first "
            "forward collision-free point beyond the obstacle.");
        break;
      }

      // Retain the conservative shorter-target fallback only for occupancy
      // that is not present in the live dynamic obstacle layer.  A dynamic
      // blocker must be bypassed or waited out; it must never become an
      // instruction to creep closer.
      if (!found_free_target && !target_dynamically_occupied)
      {
        for (int i = static_cast<int>(target_segment);
             i >= static_cast<int>(best_segment);
             --i)
        {
          const size_t waypoint_index = static_cast<size_t>(i);
          if (waypoint_index <= best_segment ||
              distanceFromProgressToWaypoint(waypoint_index) < 0.4)
            continue;
          const Eigen::Vector3d &candidate =
              active_waypoints_[waypoint_index];
          if (targetOccupancy(candidate, waypoint_index) != 0)
            continue;

          local_target_pt_ = candidate;
          target_segment = waypoint_index;
          target_ratio = 0.0;
          reached_end = false;
          found_free_target = true;
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "[reference path] Non-dynamic lookahead occupancy has no "
              "bounded bypass target; use an earlier collision-free point.");
          break;
        }
      }

      if (!found_free_target)
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[reference path] No forward collision-free lookahead point found; "
            "keep the bounded target for fail-safe handling without moving backward.");
      }
    }

    bool stop_at_reference_corner = false;
    const size_t last_corner_vertex = std::min(
        segment_count - 1,
        target_segment +
            (target_ratio >= 1.0 - 1e-6 ? size_t{1} : size_t{0}));
    const size_t stop_corner =
        reference_corner_stop_enabled_
            ? findFirstReferenceStopCorner(
                  active_waypoints_,
                  best_segment + 1,
                  last_corner_vertex,
                  start_pt_,
                  reference_corner_stop_angle_,
                  0.20)
            : active_waypoints_.size();
    if (stop_corner < active_waypoints_.size())
    {
      // B2 must not inherit the aerial planner's assumption that a sharp
      // polyline vertex can be smoothed through at nonzero speed.  End this
      // local spline at the verified pivot with zero terminal velocity; the
      // next local spline starts only after the controller has aligned with
      // the outgoing segment.
      local_target_pt_ = active_waypoints_[stop_corner];
      target_segment = stop_corner - 1;
      target_ratio = 1.0;
      reached_end = false;
      stop_at_reference_corner = true;
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[reference path] B2 stop-turn corner at waypoint %zu; "
          "finish the incoming segment before changing heading.",
          stop_corner);
    }

    reached_end = reached_end ||
                  (local_target_pt_ - active_waypoints_.back()).head<2>().norm() < 1e-3;

    auto appendGuidePoint = [&](const Eigen::Vector3d &point) {
      if (local_reference_guide_.empty() ||
          (point - local_reference_guide_.back()).norm() > 1e-4)
        local_reference_guide_.push_back(point);
    };
    appendGuidePoint(start_pt_);
    appendGuidePoint(targetFromSegment(best_segment, best_ratio));
    for (size_t waypoint_index = best_segment + 1;
         waypoint_index <= target_segment &&
         waypoint_index < active_waypoints_.size();
         ++waypoint_index)
    {
      appendGuidePoint(active_waypoints_[waypoint_index]);
    }
    appendGuidePoint(local_target_pt_);

    if (b2_obstacle_recovery_latched_)
    {
      auto clearRecoveryWaypoints = [&]() {
        b2_recovery_waypoints_.clear();
        b2_recovery_waypoint_yaws_.clear();
        b2_recovery_waypoint_simultaneous_yaw_.clear();
      };
      if (b2_recovery_waypoints_.size() !=
              b2_recovery_waypoint_yaws_.size() ||
          b2_recovery_waypoints_.size() !=
              b2_recovery_waypoint_simultaneous_yaw_.size())
      {
        clearRecoveryWaypoints();
      }

      // The controller's position tolerance is 0.10m. Keep only already
      // reached lattice samples, but preserve the 10--20cm arc chords that
      // cannot be replaced by a safe longer straight line.
      constexpr double kMinimumRecoveryLegLength = 0.105;
      auto pruneReachedRecoveryWaypoints = [&]() {
        while (!b2_recovery_waypoints_.empty() &&
               (b2_recovery_waypoints_.front() - start_pt_)
                       .head<2>()
                       .norm() <= kMinimumRecoveryLegLength)
        {
          b2_recovery_waypoints_.erase(
              b2_recovery_waypoints_.begin());
          b2_recovery_waypoint_yaws_.erase(
              b2_recovery_waypoint_yaws_.begin());
          b2_recovery_waypoint_simultaneous_yaw_.erase(
              b2_recovery_waypoint_simultaneous_yaw_.begin());
        }
      };

      // Keep following the complete guide selected at the first safe
      // post-reverse pose. Re-running A* independently at every corner can
      // switch homotopy classes and walk into a dead-end branch even though
      // the original guide was valid all the way to its target.
      // Verified recovery legs use the manager's dedicated 0.05m lower
      // bound; ordinary SCAN replans retain their historical 0.20m limit.
      const bool had_cached_recovery_route =
          !b2_recovery_waypoints_.empty();
      pruneReachedRecoveryWaypoints();
      const bool completed_cached_recovery_route =
          had_cached_recovery_route &&
          b2_recovery_waypoints_.empty();
      if (completed_cached_recovery_route)
      {
        b2_obstacle_recovery_latched_ = false;
        clearRecoveryWaypoints();
        RCLCPP_INFO(
            node_->get_logger(),
            "[B2 motion] Reached the final verified detour waypoint; "
            "release obstacle recovery and resume the retained global "
            "target.");
      }

      B2ForwardDetourSearchConfig search_config;
      search_config.maximum_yaw_step =
          execution_validation_max_yaw_step_;
      auto recoveryPoseIsSafe =
          [this](const Eigen::Vector3d &position, double yaw) {
            return !isExecutionPoseOccupied(position, yaw);
          };

      if (b2_obstacle_recovery_latched_ &&
          !b2_recovery_waypoints_.empty() &&
          !isB2StopTurnForwardLegSafe(
              start_pt_,
              getOdomYaw(),
              b2_recovery_waypoints_.front(),
              execution_validation_max_yaw_step_,
              execution_validation_max_position_step_,
              recoveryPoseIsSafe))
      {
        // The robot can stop part-way through a recovery leg because SCAN
        // continuously replans. Its live yaw/XY then differs from the pose
        // that originally validated this cached endpoint. Reconnect only to
        // the current cached waypoint, preserving the remainder of the
        // already selected homotopy instead of searching to the far target
        // and switching sides again.
        const std::vector<Eigen::Vector3d> cached_suffix =
            b2_recovery_waypoints_;
        const std::vector<double> cached_suffix_yaws =
            b2_recovery_waypoint_yaws_;
        const std::vector<bool> cached_suffix_simultaneous_yaw =
            b2_recovery_waypoint_simultaneous_yaw_;
        std::vector<Eigen::Vector3d> rejoin_guide;
        std::vector<double> rejoin_yaws;
        std::vector<bool> rejoin_simultaneous_yaw;
        size_t rejoin_index = cached_suffix.size();
        for (size_t candidate = 0;
             candidate < cached_suffix.size();
             ++candidate)
        {
          if ((cached_suffix[candidate] - start_pt_)
                      .head<2>()
                      .norm() <= kMinimumRecoveryLegLength)
          {
            continue;
          }
          rejoin_guide.clear();
          if (searchB2ForwardDetour(
                  start_pt_,
                  getOdomYaw(),
                  cached_suffix[candidate],
                  search_config,
                  recoveryPoseIsSafe,
                  rejoin_guide,
                  &rejoin_yaws,
                  &rejoin_simultaneous_yaw) &&
              rejoin_guide.size() >= 2)
          {
            rejoin_index = candidate;
            break;
          }
        }

        if (rejoin_index < cached_suffix.size())
        {
          std::vector<Eigen::Vector3d> reconnected_waypoints;
          std::vector<double> reconnected_yaws;
          std::vector<bool> reconnected_simultaneous_yaw;
          reconnected_waypoints.reserve(
              rejoin_guide.size() - 1 +
              cached_suffix.size() - rejoin_index - 1);
          reconnected_yaws.reserve(
              rejoin_guide.size() - 1 +
              cached_suffix.size() - rejoin_index - 1);
          reconnected_simultaneous_yaw.reserve(
              rejoin_guide.size() - 1 +
              cached_suffix.size() - rejoin_index - 1);
          for (size_t index = 1; index < rejoin_guide.size(); ++index)
          {
            reconnected_waypoints.push_back(rejoin_guide[index]);
            reconnected_yaws.push_back(rejoin_yaws[index]);
            reconnected_simultaneous_yaw.push_back(
                rejoin_simultaneous_yaw[index]);
          }
          for (size_t index = rejoin_index + 1;
               index < cached_suffix.size();
               ++index)
          {
            if (reconnected_waypoints.empty() ||
                (cached_suffix[index] - reconnected_waypoints.back())
                        .head<2>()
                        .norm() > 1e-4)
            {
              reconnected_waypoints.push_back(cached_suffix[index]);
              reconnected_yaws.push_back(cached_suffix_yaws[index]);
              reconnected_simultaneous_yaw.push_back(
                  cached_suffix_simultaneous_yaw[index]);
            }
          }
          b2_recovery_waypoints_ = std::move(reconnected_waypoints);
          b2_recovery_waypoint_yaws_ =
              std::move(reconnected_yaws);
          b2_recovery_waypoint_simultaneous_yaw_ =
              std::move(reconnected_simultaneous_yaw);
          pruneReachedRecoveryWaypoints();
          RCLCPP_INFO(
              node_->get_logger(),
              "[B2 motion] Live pose no longer has a safe direct sweep to "
              "the cached recovery leg; inserted %zu verified SE(2) rejoin "
              "waypoint(s), skipped %zu unreachable stale waypoint(s), and "
              "retained the original route suffix.",
              rejoin_guide.size() - 1,
              rejoin_index);
        }
        else
        {
          // Never hand a waypoint that just failed the full B2 sweep back to
          // the legacy spline optimizer. Clear the stale cache so the normal
          // empty-cache branch below can rebuild a complete forward guide
          // from the live pose.
          clearRecoveryWaypoints();
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "[B2 motion] No remaining cached waypoint is safely "
              "rejoinable from the live pose; discard the stale cache and "
              "rebuild the forward guide without dropping the target.");
        }
      }

      if (b2_obstacle_recovery_latched_ &&
          b2_recovery_waypoints_.empty())
      {
        std::vector<Eigen::Vector3d> recovery_guide;
        std::vector<double> recovery_yaws;
        std::vector<bool> recovery_simultaneous_yaw;
        if (searchB2ForwardDetour(
                start_pt_,
                getOdomYaw(),
                local_target_pt_,
                search_config,
                recoveryPoseIsSafe,
                recovery_guide,
                &recovery_yaws,
                &recovery_simultaneous_yaw))
        {
          b2_recovery_waypoints_.assign(
              recovery_guide.begin() + 1,
              recovery_guide.end());
          b2_recovery_waypoint_yaws_.assign(
              recovery_yaws.begin() + 1,
              recovery_yaws.end());
          b2_recovery_waypoint_simultaneous_yaw_.assign(
              recovery_simultaneous_yaw.begin() + 1,
              recovery_simultaneous_yaw.end());
        }
      }

      if (!b2_recovery_waypoints_.empty())
      {
        // Never ask a cubic spline to smooth several recovery corners at
        // once: execute only the next visibility-validated straight leg.
        local_target_pt_ = b2_recovery_waypoints_.front();
        b2_recovery_subgoal_active_ = true;
        b2_recovery_leg_start_yaw_ = getOdomYaw();
        b2_recovery_leg_end_yaw_ =
            b2_recovery_waypoint_yaws_.front();
        b2_recovery_leg_simultaneous_yaw_ =
            b2_recovery_waypoint_simultaneous_yaw_.front();
        local_reference_guide_.clear();
        local_reference_guide_.push_back(start_pt_);
        local_reference_guide_.push_back(local_target_pt_);

        const Eigen::Vector2d leg =
            (local_target_pt_ - start_pt_).head<2>();
        if (leg.norm() > 1e-4)
        {
          start_vel_.setZero();
          start_vel_.head<2>() =
              std::max(0.05, b2_forward_seed_config_.speed) *
              leg.normalized();
          start_acc_.setZero();
        }
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[B2 motion] Execute heading-aware forward detour leg to "
            "(%.2f, %.2f): compressed guide=%zu poses, intermediate=%s. "
            "The complete turn, translation and ground footprint were "
            "validated.",
            local_target_pt_.x(), local_target_pt_.y(),
            b2_recovery_waypoints_.size() + 1,
            b2_recovery_subgoal_active_ ? "true" : "false");
      }
      else if (b2_obstacle_recovery_latched_)
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[B2 motion] No heading-aware forward detour is currently "
            "reachable; retain bounded straight recovery retries.");
      }
    }

    local_target_vel_.setZero();
    const Eigen::Vector2d tangent =
        (active_waypoints_[target_segment + 1] - active_waypoints_[target_segment]).head<2>();
    if (!reached_end && !stop_at_reference_corner &&
        !b2_recovery_subgoal_active_ && tangent.norm() > 1e-4)
    {
      local_target_vel_.head<2>() = tangent.normalized() * planner_manager_->pp_.max_vel_;
    }

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[reference path] progress=%zu+%.2f target=%zu+%.2f xyz=(%.2f, %.2f, %.2f) horizon=%.2f",
        reference_progress_segment_, reference_progress_ratio_, target_segment, target_ratio,
        local_target_pt_(0), local_target_pt_(1), local_target_pt_(2), planning_horizon_);
    return true;
  }

  void SCANReplanFSM::getLocalTarget()
  {
    if (getReferencePathLocalTarget())
      return;

    double t;

    double t_step = planning_horizon_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    double target_t = planner_manager_->global_data_.global_duration_;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
      {
        ROS_ERROR_STREAM("[getLocalTarget] last_progress_time mismatch: "
                         << "dist_to_progress_pt=" << dist
                         << ", planning_horizon=" << planning_horizon_
                         << ", last_progress_time=" << planner_manager_->global_data_.last_progress_time_);
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = t;
        break;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      target_t = planner_manager_->global_data_.global_duration_;
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;
      const Eigen::Vector3d original_target = local_target_pt_;
      const double start_to_goal = (start_pt_ - end_pt_).norm();

      auto isBoundedCandidate = [&](const Eigen::Vector3d &pt) {
        return (pt - original_target).norm() <= planning_horizon_ &&
               (pt - end_pt_).norm() <= start_to_goal + 0.5;
      };

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (isBoundedCandidate(pt) && targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, dist_min_t))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (isBoundedCandidate(pt) && targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        ROS_WARN_THROTTLE(1.0, "Local target in collision, adjusted to a nearby collision-free point.");
        target_t = adjusted_t;
      }
      else
      {
        // Prefer a known point from the original global reference path over a
        // polynomial sample that has drifted after repeated frozen replans.
        for (auto it = active_waypoints_.rbegin(); it != active_waypoints_.rend(); ++it)
        {
          if (isBoundedCandidate(*it) && targetOccupancy(*it) == 0)
          {
            local_target_pt_ = *it;
            local_target_vel_.setZero();
            ROS_WARN_THROTTLE(1.0, "Local target collision fallback uses a bounded reference-path point.");
            return;
          }
        }
        local_target_pt_ = original_target;
        ROS_WARN_THROTTLE(1.0, "Local target in collision and no bounded collision-free target was found.");
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      // cout << "AA" << endl;
    }
  }

} // namespace scan_planner
