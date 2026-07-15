
#include <plan_manage/scan_replan_fsm.h>
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
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    getParam(nh, "fsm/navi_mode", navi_mode_, -1);
    getParam(nh, "fsm/thresh_replan", replan_thresh_, -1.0);
    getParam(nh, "fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    getParam(nh, "fsm/planning_horizon", planning_horizon_, -1.0);
    getParam(nh, "fsm/start_height_offset", start_height_offset_, 0.3);
    start_height_offset_ = std::max(0.0, start_height_offset_);
    getParam(nh, "fsm/final_goal_tolerance", final_goal_tolerance_, 0.12);
    final_goal_tolerance_ = std::max(0.05, final_goal_tolerance_);
    getParam(nh, "fsm/emergency_time_", emergency_time_, 1.0);
    getParam(nh, "fsm/fail_safe", enable_fail_safe_, true);
    getParam(nh, "fsm/max_replan_fail_count", max_replan_fail_count_, 1000);
    getParam(nh, "grid_map/obstacles_inflation_z_up", self_inflation_z_up_, 0.0);
    getParam(nh, "grid_map/obstacles_inflation_z_down", self_inflation_z_down_, 0.0);
    getParam(nh, "grid_map/double_cylinder_radius", self_double_cylinder_radius_, 0.0);
    getParam(nh, "grid_map/double_cylinder_offset", self_double_cylinder_offset_, 0.0);
    getParam(nh, "grid_map/double_cylinder_center_offset", self_double_cylinder_center_offset_, 0.0);
    getParam(nh, "grid_map/body_height", body_height_, 0.4);
    getParam(nh, "grid_map/frame_id", self_inflation_frame_id_, std::string("world"));

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
    odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
        body_pose_topic, 1, std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = nh->create_subscription<std_msgs::msg::Bool>(
        "/planning/go2_execution_frozen", 10, std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));
    replan_request_sub_ = nh->create_subscription<std_msgs::msg::Bool>(
        replan_request_topic, 10, std::bind(&SCANReplanFSM::replanRequestCallback, this, std::placeholders::_1));

    bspline_pub_ = nh->create_publisher<scan_planner::msg::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh->create_publisher<scan_planner::msg::DataDisp>("/planning/data_display", 100);
    self_inflation_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("self_inflation", rclcpp::QoS(10).transient_local());

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
      path_sub_ = nh->create_subscription<nav_msgs::msg::Path>(
          "/initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
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

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    // A newly clicked reference path starts from a stationary boundary
    // condition.  TF differentiation can report lateral velocity while the
    // body is only rotating, which would otherwise bend the first spline away
    // from the supplied global path.
    const Eigen::Vector3d initial_velocity =
        navi_mode_ == NAVI_MODE::REFERENCE_PATH ? Eigen::Vector3d::Zero() : odom_vel_;
    const Eigen::Vector3d planning_start = getPlanningStartPosition();
    bool success = planner_manager_->planGlobalTrajWaypoints(
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
      return true;

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
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
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

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    if (msg->data && !go2_execution_frozen_)
    {
      odom_vel_.setZero();
      ROS_INFO("[execution freeze] Hold SCAN trajectory execution; replanning remains enabled with zero boundary velocity.");
    }
    go2_execution_frozen_ = msg->data;
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
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
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

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
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
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

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
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if (finishReferencePathIfGoalReached("REPLAN_TRAJ"))
        break;

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
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
          const double final_goal_distance =
              (active_waypoints_.back() - odom_pos_).head<2>().norm();
          if (final_goal_distance > final_goal_tolerance_)
          {
            ROS_INFO("[reference path] Local trajectory finished %.3fm before final goal; continue planning.",
                     final_goal_distance);
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
    if (replan_fail_count_ >= max_replan_fail_count_)
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

    if (go2_execution_frozen_)
    {
      start_vel_.setZero();
      start_acc_.setZero();
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

    if (go2_execution_frozen_ ||
        (navi_mode_ == NAVI_MODE::REFERENCE_PATH && have_new_target_))
    {
      start_vel_.setZero();
      start_acc_.setZero();
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

    if (go2_execution_frozen_)
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
    if (go2_execution_frozen_)
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

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = toMsgTime(info->start_time_);
      bspline.traj_id = info->traj_id_;

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

      bspline_pub_->publish(bspline);

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

    bspline_pub_->publish(bspline);

    return true;
  }

  bool SCANReplanFSM::getReferencePathLocalTarget()
  {
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || active_waypoints_.empty())
      return false;

    if (active_waypoints_.size() == 1)
    {
      local_target_pt_ = active_waypoints_.front();
      local_target_vel_.setZero();
      return true;
    }

    const size_t segment_count = active_waypoints_.size() - 1;
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

    reference_progress_segment_ = std::max(reference_progress_segment_, best_segment);
    if (reference_progress_segment_ == best_segment)
      reference_progress_ratio_ = std::max(reference_progress_ratio_, best_ratio);
    else
      reference_progress_ratio_ = best_ratio;

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
      for (int i = static_cast<int>(target_segment); i >= static_cast<int>(best_segment); --i)
      {
        const Eigen::Vector3d &candidate = active_waypoints_[static_cast<size_t>(i)];
        if ((candidate - start_pt_).head<2>().norm() < 0.4)
          continue;
        if (targetOccupancy(candidate, static_cast<size_t>(i)) == 0)
        {
          local_target_pt_ = candidate;
          target_segment = static_cast<size_t>(i);
          target_ratio = 0.0;
          reached_end = false;
          found_free_target = true;
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "[reference path] Lookahead target occupied; use earlier collision-free point on original path.");
          break;
        }
      }

      if (!found_free_target)
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[reference path] No collision-free lookahead point found on original path; keep bounded target for fail-safe handling.");
      }
    }

    reached_end = reached_end ||
                  (local_target_pt_ - active_waypoints_.back()).head<2>().norm() < 1e-3;

    local_target_vel_.setZero();
    const Eigen::Vector2d tangent =
        (active_waypoints_[target_segment + 1] - active_waypoints_[target_segment]).head<2>();
    if (!reached_end && tangent.norm() > 1e-4)
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
