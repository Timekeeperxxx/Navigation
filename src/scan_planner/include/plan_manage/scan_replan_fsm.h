#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_manage/b2_motion_policy.h>
#include <plan_manage/b2_reverse_recovery.h>
#include <plan_manage/b2_yaw_schedule.h>
#include <plan_env/grid_map.h>
#include <plan_manage/replan_failure_policy.h>
#include <scan_planner/msg/bspline.hpp>
#include <scan_planner/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

#ifndef ROS_INFO
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("scan_planner"), __VA_ARGS__)
#endif
#ifndef ROS_WARN
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("scan_planner"), __VA_ARGS__)
#endif
#ifndef ROS_ERROR
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("scan_planner"), __VA_ARGS__)
#endif
#ifndef ROS_WARN_THROTTLE
#define ROS_WARN_THROTTLE(period, ...) RCLCPP_WARN(rclcpp::get_logger("scan_planner"), __VA_ARGS__)
#endif
#ifndef ROS_INFO_THROTTLE
#define ROS_INFO_THROTTLE(period, ...) RCLCPP_INFO(rclcpp::get_logger("scan_planner"), __VA_ARGS__)
#endif
#ifndef ROS_ERROR_STREAM
#define ROS_ERROR_STREAM(args) RCLCPP_ERROR_STREAM(rclcpp::get_logger("scan_planner"), args)
#endif
#ifndef ROS_WARN_STREAM
#define ROS_WARN_STREAM(args) RCLCPP_WARN_STREAM(rclcpp::get_logger("scan_planner"), args)
#endif

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      REVERSE_RECOVERY,
      EMERGENCY_STOP
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner::msg::DataDisp data_disp_;

    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    bool reference_corner_stop_enabled_;
    double reference_corner_stop_angle_;
    double start_height_offset_;
    double final_goal_tolerance_;
    double emergency_time_;
    double replan_retry_interval_;
    bool execution_validation_enabled_;
    double execution_validation_path_sample_dt_;
    double b2_maximum_path_slope_;
    double execution_validation_max_position_step_;
    double execution_validation_max_yaw_step_;
    double execution_validation_probe_radius_;
    int execution_validation_probe_count_;
    int execution_validation_min_map_updates_;
    bool execution_validation_allow_occupied_start_;
    double execution_validation_start_escape_distance_;
    bool b2_allow_reverse_;
    bool b2_reverse_recovery_enabled_;
    double b2_reverse_recovery_request_period_;
    int b2_reverse_recovery_min_replan_failures_;
    double b2_reverse_recovery_obstacle_probe_distance_;
    double b2_reverse_recovery_obstacle_probe_step_;
    B2ForwardSeedConfig b2_forward_seed_config_;
    B2DirectionGuardConfig b2_direction_guard_config_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double self_double_cylinder_center_offset_;
    double body_height_;
    std::string self_inflation_frame_id_;
    std::string execution_frozen_topic_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool rviz_height_ready_;
    bool controller_execution_frozen_;
    bool safety_execution_frozen_;
    bool reverse_recovery_requested_;
    bool reverse_recovery_active_;
    bool b2_obstacle_recovery_latched_;
    bool b2_recovery_subgoal_active_;
    std::vector<Eigen::Vector3d> b2_recovery_waypoints_;
    std::vector<double> b2_recovery_waypoint_yaws_;
    std::vector<bool> b2_recovery_waypoint_simultaneous_yaw_;
    double b2_recovery_leg_start_yaw_;
    double b2_recovery_leg_end_yaw_;
    bool b2_recovery_leg_simultaneous_yaw_;
    bool global_target_ground_validation_pending_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    rclcpp::Time last_freeze_update_time_;
    double replan_not_before_seconds_{0.0};
    double reverse_recovery_last_request_seconds_{0.0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    std::vector<Eigen::Vector3d> local_reference_guide_;
    size_t reference_progress_segment_{0};
    double reference_progress_ratio_{0.0};
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr controller_execution_frozen_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_execution_frozen_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr reverse_recovery_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replan_request_sub_;
    rclcpp::Publisher<scan_planner::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<scan_planner::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reverse_recovery_request_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool adjustGlobalTargetIfOccupied();
    bool getReferencePathLocalTarget();
    void getLocalTarget();
    void finishProcess();
    bool finishReferencePathIfGoalReached(const char *source);
    void publishSelfInflationMarker();
    Eigen::Vector3d getPlanningStartPosition() const;
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    bool isTrajectorySafeForExecution(
        UniformBspline &trajectory,
        std::vector<double> *validated_yaw_schedule = nullptr,
        double *validated_yaw_dt = nullptr,
        bool use_recovery_yaw_contract = false,
        double recovery_start_yaw = 0.0,
        double recovery_end_yaw = 0.0,
        bool recovery_yaw_changes_with_translation = false);
    bool applyB2ForwardStartSeed();
    bool isB2TrajectoryDirectionSafe(
        UniformBspline &trajectory,
        bool enforce_initial_reference_cone,
        bool allow_bounded_obstacle_detour = false) const;
    Eigen::Vector2d getReferenceForwardDirection() const;
    bool isExecutionPoseOccupied(const Eigen::Vector3d &position, double yaw) const;
    bool isExecutionPoseDynamicallyOccupied(
        const Eigen::Vector3d &position, double yaw) const;
    bool isExecutionMapReady();
    void updateLocalTrajTimeFreeze();
    bool replanAttemptReady() const;
    void deferReplanAttempt();
    bool requestB2ReverseRecovery(const char *source);
    bool hasB2ImmediateDynamicObstacle(
        bool &current_footprint_occupied,
        bool &forward_probe_occupied) const;

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void controllerExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);
    void safetyExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);
    void reverseRecoveryStatusCallback(const std_msgs::msg::UInt8::ConstSharedPtr &msg);
    void replanRequestCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    void init(const rclcpp::Node::SharedPtr &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
