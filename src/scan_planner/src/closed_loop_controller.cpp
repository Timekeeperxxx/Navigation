#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bspline_opt/uniform_bspline.h"
#include "plan_manage/b2_reverse_recovery.h"
#include "plan_manage/b2_yaw_schedule.h"
#include "plan_manage/final_yaw_latch.h"
#include "plan_manage/goal_yaw_handoff.h"
#include "plan_manage/heading_alignment_latch.h"
#include "scan_planner/msg/bspline.hpp"

namespace
{
using scan_planner::UniformBspline;

constexpr double kMaxVYawLimit = 1.0;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr legacy_execution_frozen_pub;
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
    reverse_recovery_status_pub;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr execution_path_pub;
rclcpp::Subscription<scan_planner::msg::Bspline>::SharedPtr bspline_sub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr goal_yaw_sub;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_execution_frozen_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    reverse_recovery_request_sub;
rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    final_yaw_validation_sub;
rclcpp::TimerBase::SharedPtr cmd_timer;

bool receive_traj = false;
bool have_odom = false;
std::vector<UniformBspline> traj;
std::vector<double> b2_yaw_schedule;
double traj_duration = 0.0;
int64_t traj_id = 0;
bool emergency_stop_trajectory = false;
bool terminal_goal_trajectory = false;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;

double exec_time = 0.0;
rclcpp::Time last_update_time;
rclcpp::Time traj_start_time;
rclcpp::Time last_execution_path_publish_time;
rclcpp::Time last_odom_time;
rclcpp::Time reverse_recovery_generation_time;
rclcpp::Time last_safety_execution_frozen_time;

double time_forward;
double heading_error_threshold;
double heading_resume_threshold;
double kp_pos;
double kp_yaw;
double yaw_feedforward_gain;
double max_vx;
double max_vy;
double max_vyaw;
double min_translation_speed;
double min_in_place_vyaw;
double finish_dist;
double final_yaw_tolerance;
bool enable_final_yaw;
bool allow_reverse;
double reverse_recovery_speed;
double reverse_recovery_max_distance;
double reverse_recovery_max_duration;
double reverse_recovery_path_height_offset;
double reverse_recovery_path_sample_step;
bool have_goal_yaw = false;
bool safety_execution_frozen = false;
double goal_yaw = 0.0;
scan_planner::FinalYawLatch final_yaw_latch;
scan_planner::GoalYawHandoff goal_yaw_handoff;
scan_planner::HeadingAlignmentLatch heading_alignment_latch;
scan_planner::B2ReverseRecoveryPolicy reverse_recovery_policy;
std::string body_pose_topic;
std::string goal_pose_topic;
std::string execution_path_topic;
std::string execution_path_frame;
std::string safety_execution_frozen_topic;
std::string execution_frozen_topic;
std::string legacy_execution_frozen_topic;
std::string reverse_recovery_request_topic;
std::string reverse_recovery_status_topic;
double execution_path_publish_period;
double execution_path_sample_dt;
double goal_yaw_coalesce_window;
double final_yaw_sweep_max_step;
double final_yaw_validation_timeout;
double final_yaw_preflight_started_at = -1.0;
bool final_yaw_safety_aborted = false;
bool force_execution_path_publish = false;
std::string final_yaw_validation_topic;

enum class FinalYawValidationState
{
  IDLE,
  WAITING,
  APPROVED,
  DENIED,
};

FinalYawValidationState final_yaw_validation_state =
    FinalYawValidationState::IDLE;
double final_yaw_validation_generation = 0.0;

template <typename T>
T getParamWithDefault(const std::string &name, const T &default_value)
{
  if (!node->has_parameter(name))
    node->declare_parameter<T>(name, default_value);
  return node->get_parameter(name).get_value<T>();
}

bool loadRequiredParam(const std::string &name, double &value)
{
  if (node->has_parameter(name) && node->get_parameter(name, value))
    return true;

  RCLCPP_ERROR_STREAM(node->get_logger(), "[closed_loop_controller] missing required private parameter ~" << name);
  return false;
}

bool loadParams()
{
  bool ok = true;
  body_pose_topic = getParamWithDefault<std::string>("body_pose_topic", "/quad_0/body_pose");
  goal_pose_topic = getParamWithDefault<std::string>("goal_pose_topic", "/goal_pose");
  execution_path_topic = getParamWithDefault<std::string>("execution_path_topic", "/scan/execution_path");
  execution_path_frame = getParamWithDefault<std::string>("execution_path_frame", "map");
  safety_execution_frozen_topic = getParamWithDefault<std::string>(
      "safety_execution_frozen_topic", "/planning/safety_execution_frozen");
  final_yaw_validation_topic = getParamWithDefault<std::string>(
      "final_yaw_validation_topic", "/planning/final_yaw_validation");
  execution_frozen_topic = getParamWithDefault<std::string>(
      "execution_frozen_topic", "/planning/b2_execution_frozen");
  legacy_execution_frozen_topic = getParamWithDefault<std::string>(
      "legacy_execution_frozen_topic", "/planning/go2_execution_frozen");
  reverse_recovery_request_topic = getParamWithDefault<std::string>(
      "reverse_recovery_request_topic",
      "/planning/b2_reverse_recovery_request");
  reverse_recovery_status_topic = getParamWithDefault<std::string>(
      "reverse_recovery_status_topic",
      "/planning/b2_reverse_recovery_status");
  execution_path_publish_period = std::max(
      getParamWithDefault<double>("execution_path_publish_period", 0.1), 0.02);
  execution_path_sample_dt = std::max(
      getParamWithDefault<double>("execution_path_sample_dt", 0.1), 0.02);
  goal_yaw_coalesce_window = std::max(
      getParamWithDefault<double>("goal_yaw_coalesce_window", 0.5), 0.0);
  scan_planner::GoalYawHandoffConfig goal_yaw_handoff_config;
  goal_yaw_handoff_config.coalesce_window = goal_yaw_coalesce_window;
  goal_yaw_handoff.configure(goal_yaw_handoff_config);
  final_yaw_sweep_max_step = std::max(
      getParamWithDefault<double>(
          "final_yaw_sweep_max_step", 5.0 * M_PI / 180.0),
      1.0 * M_PI / 180.0);
  final_yaw_validation_timeout = std::max(
      getParamWithDefault<double>("final_yaw_validation_timeout", 2.0), 0.2);
  ok &= loadRequiredParam("time_forward", time_forward);
  ok &= loadRequiredParam("heading_error_threshold", heading_error_threshold);
  heading_resume_threshold = std::max(
      0.0, getParamWithDefault<double>("heading_resume_threshold", 0.35));
  ok &= loadRequiredParam("kp_pos", kp_pos);
  ok &= loadRequiredParam("kp_yaw", kp_yaw);
  yaw_feedforward_gain = std::max(
      0.0, getParamWithDefault<double>("yaw_feedforward_gain", 1.0));
  ok &= loadRequiredParam("max_vx", max_vx);
  ok &= loadRequiredParam("max_vy", max_vy);
  ok &= loadRequiredParam("max_vyaw", max_vyaw);
  min_translation_speed = std::max(
      0.0, getParamWithDefault<double>("min_translation_speed", 0.0));
  min_in_place_vyaw = std::max(
      0.0, getParamWithDefault<double>("min_in_place_vyaw", 0.0));
  ok &= loadRequiredParam("finish_dist", finish_dist);
  enable_final_yaw = getParamWithDefault<bool>("enable_final_yaw", true);
  allow_reverse = getParamWithDefault<bool>("allow_reverse", false);
  reverse_recovery_speed = std::max(
      0.01,
      getParamWithDefault<double>("reverse_recovery_speed", 0.15));
  reverse_recovery_path_height_offset = std::max(
      0.0,
      getParamWithDefault<double>(
          "reverse_recovery_path_height_offset", 0.32));
  reverse_recovery_path_sample_step = std::max(
      0.02,
      getParamWithDefault<double>(
          "reverse_recovery_path_sample_step", 0.05));
  scan_planner::B2ReverseRecoveryConfig reverse_config;
  reverse_recovery_max_distance = std::max(
      0.05, getParamWithDefault<double>(
      "reverse_recovery_max_distance", 0.50));
  reverse_recovery_max_duration = std::max(
      0.10, getParamWithDefault<double>(
      "reverse_recovery_max_duration", 2.0));
  reverse_config.maximum_distance = reverse_recovery_max_distance;
  reverse_config.maximum_duration = reverse_recovery_max_duration;
  reverse_config.minimum_preflight_duration =
      getParamWithDefault<double>(
          "reverse_recovery_minimum_preflight_duration", 0.40);
  reverse_config.safety_approval_timeout = getParamWithDefault<double>(
      "reverse_recovery_safety_approval_timeout", 1.5);
  reverse_config.odometry_timeout = getParamWithDefault<double>(
      "reverse_recovery_odometry_timeout", 0.50);
  reverse_config.maximum_yaw_drift = getParamWithDefault<double>(
      "reverse_recovery_max_yaw_drift", 0.15);
  reverse_recovery_policy.configure(reverse_config);
  final_yaw_tolerance = std::max(
      0.0, getParamWithDefault<double>("final_yaw_tolerance", 0.15));

  if (ok && max_vyaw > kMaxVYawLimit)
  {
    RCLCPP_WARN(node->get_logger(), "[closed_loop_controller] cap max_vyaw %.3f to %.3f rad/s.",
                max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  if (ok) {
    heading_alignment_latch.configure(
        heading_error_threshold, heading_resume_threshold);
    heading_resume_threshold = heading_alignment_latch.resumeThreshold();
    const double max_translation_speed = std::hypot(max_vx, max_vy);
    if (min_translation_speed > max_translation_speed) {
      RCLCPP_WARN(
          node->get_logger(),
          "[closed_loop_controller] cap min_translation_speed %.3f to planar limit %.3f m/s.",
          min_translation_speed, max_translation_speed);
      min_translation_speed = max_translation_speed;
    }
    min_in_place_vyaw = std::min(min_in_place_vyaw, max_vyaw);
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] B2 translation: min=%.3f max_x=%.3f "
        "max_y=%.3f m/s reverse=%s.",
        min_translation_speed, max_vx, max_vy,
        allow_reverse ? "enabled" : "disabled");
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] B2 heading gate: stop=%.3f resume=%.3f rad, "
        "minimum in-place yaw=%.3f rad/s.",
        heading_alignment_latch.stopThreshold(),
        heading_alignment_latch.resumeThreshold(),
        min_in_place_vyaw);
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] B2 yaw rate feed-forward gain=%.3f.",
        yaw_feedforward_gain);
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] final-yaw footprint preflight: "
        "ack_timeout=%.2fs yaw_step=%.3frad topic=%s.",
        final_yaw_validation_timeout, final_yaw_sweep_max_step,
        final_yaw_validation_topic.c_str());
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] goal-yaw handoff protocol: "
        "coalesce_window=%.3fs.",
        goal_yaw_coalesce_window);
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] B2 reverse recovery: straight-only "
        "vx=-%.3fm/s; each round ends at %.2fm or %.2fs, with no "
        "retry-count limit.",
        reverse_recovery_speed,
        reverse_recovery_max_distance,
        reverse_recovery_max_duration);
  }
  return ok;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double enforceMinimumSignedMagnitude(double value, double minimum)
{
  if (std::abs(value) <= 1e-9 || std::abs(value) >= minimum)
    return value;
  return std::copysign(minimum, value);
}

Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
{
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-6)
    return value;
  return value / norm * max_norm;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(msg, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

void rebuildB2YawSchedule()
{
  b2_yaw_schedule.clear();
  if (!have_odom || traj.empty() || traj_duration <= 1e-6)
    return;

  const std::size_t interval_count =
      scan_planner::b2YawSampleIntervalCount(
          traj_duration, execution_path_sample_dt);
  if (interval_count == 0)
    return;

  std::vector<Eigen::Vector3d> path;
  path.reserve(interval_count + 1);
  for (std::size_t index = 0; index <= interval_count; ++index)
  {
    const double time =
        traj_duration * static_cast<double>(index) /
        static_cast<double>(interval_count);
    path.push_back(traj[0].evaluateDeBoorT(time));
  }
  b2_yaw_schedule =
      scan_planner::makeB2YawSchedule(path, odom_yaw);
}

double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des)
{
  if (b2_yaw_schedule.size() >= 2)
  {
    return scan_planner::interpolateB2YawSchedule(
        b2_yaw_schedule, traj_duration, t_cur);
  }

  const double t_look = std::min(traj_duration, t_cur + time_forward);
  Eigen::Vector3d dir = traj[0].evaluateDeBoorT(t_look) - pos_des;

  if (dir.head<2>().squaredNorm() < 1e-4)
  {
    Eigen::Vector3d vel = traj[1].evaluateDeBoorT(t_cur);
    dir = vel;
  }

  if (dir.head<2>().squaredNorm() < 1e-4)
    return odom_yaw;

  return std::atan2(dir(1), dir(0));
}

void publishStop(double vyaw = 0.0)
{
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  cmd_vel_pub->publish(cmd);
}

void publishExecutionFrozen(bool frozen)
{
  std_msgs::msg::Bool msg;
  msg.data = frozen;
  execution_frozen_pub->publish(msg);
  if (legacy_execution_frozen_pub)
    legacy_execution_frozen_pub->publish(msg);
}

void publishReverseRecoveryStatus(
    scan_planner::B2ReverseRecoveryStatus status)
{
  if (!reverse_recovery_status_pub)
    return;
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<std::uint8_t>(status);
  reverse_recovery_status_pub->publish(msg);
}

void publishReverseRecoveryPath(const rclcpp::Time &now)
{
  if (!reverse_recovery_policy.active() || !have_odom)
    return;
  if (
      last_execution_path_publish_time.nanoseconds() > 0 &&
      (now - last_execution_path_publish_time).seconds() <
          execution_path_publish_period)
  {
    return;
  }

  const std::size_t intervals = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(
          std::ceil(
              reverse_recovery_max_distance /
              reverse_recovery_path_sample_step)));
  const double yaw = reverse_recovery_policy.startYaw();
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  // Latch the recovery round's base height. Using live odometry here creates
  // a positive feedback loop in simulation: TerrainZTracker subtracts body
  // height from this path, the next publication reads that lowered odometry,
  // and the robot sinks again on every publication.
  const double path_z =
      reverse_recovery_policy.startZ() +
      reverse_recovery_path_height_offset;

  nav_msgs::msg::Path path;
  path.header.frame_id = execution_path_frame;
  path.header.stamp = reverse_recovery_generation_time;
  path.poses.reserve(intervals + 1);
  for (std::size_t index = 0; index <= intervals; ++index)
  {
    const double distance =
        reverse_recovery_max_distance * static_cast<double>(index) /
        static_cast<double>(intervals);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x =
        reverse_recovery_policy.startX() - distance * cos_yaw;
    pose.pose.position.y =
        reverse_recovery_policy.startY() - distance * sin_yaw;
    pose.pose.position.z = path_z;
    pose.pose.orientation = quaternionFromYaw(yaw);
    path.poses.push_back(pose);
  }

  execution_path_pub->publish(path);
  last_execution_path_publish_time = now;
}

void resetFinalYawValidation()
{
  final_yaw_preflight_started_at = -1.0;
  final_yaw_safety_aborted = false;
  final_yaw_validation_state = FinalYawValidationState::IDLE;
  final_yaw_validation_generation = 0.0;
  force_execution_path_publish = true;
}

void publishExecutionPath(const rclcpp::Time &now)
{
  if (!receive_traj || traj.empty())
    return;
  if (!force_execution_path_publish &&
      last_execution_path_publish_time.nanoseconds() > 0 &&
      (now - last_execution_path_publish_time).seconds() < execution_path_publish_period)
    return;

  nav_msgs::msg::Path path;
  // Keep the trajectory generation time fixed in the header.  The safety
  // monitor uses message receipt time for liveness and this stamp to reject a
  // trajectory that belongs to an older global goal.
  path.header.stamp = traj_start_time;
  path.header.frame_id = execution_path_frame;

  auto append_point_with_yaw =
      [&](const Eigen::Vector3d &point, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point(0);
    pose.pose.position.y = point(1);
    pose.pose.position.z = point(2);
    pose.pose.orientation = quaternionFromYaw(yaw);
    path.poses.push_back(pose);
  };

  const double start = std::min(exec_time, traj_duration);
  const Eigen::Vector3d final_point =
      traj[0].evaluateDeBoorT(traj_duration);
  const double terminal_xy_error =
      (final_point - odom_pos).head<2>().norm();
  const double remaining_final_yaw =
      normalizeAngle(goal_yaw - odom_yaw);
  Eigen::Vector3d live_rotation_point = final_point;
  live_rotation_point.x() = odom_pos.x();
  live_rotation_point.y() = odom_pos.y();
  const bool terminal_position_reached =
      have_odom &&
      start >= traj_duration - 1e-6 &&
      terminal_xy_error <= finish_dist;
  const bool publish_final_yaw_sweep =
      terminal_position_reached &&
      terminal_goal_trajectory &&
      !emergency_stop_trajectory &&
      enable_final_yaw &&
      have_goal_yaw &&
      !final_yaw_safety_aborted &&
      std::abs(remaining_final_yaw) > final_yaw_tolerance;

  if (publish_final_yaw_sweep)
  {
    const auto yaw_sweep = scan_planner::makeB2InPlaceYawSchedule(
        odom_yaw, goal_yaw, final_yaw_sweep_max_step);
    for (double yaw : yaw_sweep)
      append_point_with_yaw(live_rotation_point, yaw);
    if (
        !yaw_sweep.empty() &&
        final_yaw_validation_state == FinalYawValidationState::IDLE)
    {
      final_yaw_preflight_started_at = now.seconds();
      final_yaw_validation_generation = traj_start_time.seconds();
      final_yaw_validation_state = FinalYawValidationState::WAITING;
      RCLCPP_INFO(
          node->get_logger(),
          "[closed_loop_controller] published full final-yaw footprint "
          "sweep (%zu samples, generation=%.6f); waiting for safety ACK.",
          yaw_sweep.size(), final_yaw_validation_generation);
    }
  }
  else if (
      terminal_position_reached &&
      terminal_goal_trajectory &&
      (final_yaw_latch.isComplete() || final_yaw_safety_aborted))
  {
    // Keep the safety layer aligned with the actual stationary B2 footprint.
    // A completed/denied final yaw is held at live odometry, not at the
    // position-spline tangent or its ideal endpoint.
    append_point_with_yaw(live_rotation_point, odom_yaw);
  }
  else
  {
    auto append_trajectory_pose = [&](double t) {
      const Eigen::Vector3d point = traj[0].evaluateDeBoorT(t);
      append_point_with_yaw(point, estimateDesiredYaw(t, point));
    };
    for (double t = start; t < traj_duration; t += execution_path_sample_dt)
      append_trajectory_pose(t);
    append_trajectory_pose(traj_duration);
  }

  execution_path_pub->publish(path);
  last_execution_path_publish_time = now;
  force_execution_path_publish = false;
}

void bsplineCallback(const scan_planner::msg::Bspline::ConstSharedPtr &msg)
{
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);
  const double new_traj_duration = pos_traj.getTimeSum();

  std::vector<double> received_yaw_schedule;
  if (!msg->yaw_pts.empty())
  {
    const bool values_finite = std::all_of(
        msg->yaw_pts.begin(), msg->yaw_pts.end(),
        [](double value) { return std::isfinite(value); });
    const double yaw_duration =
        msg->yaw_dt * static_cast<double>(msg->yaw_pts.size() - 1);
    const double duration_tolerance =
        std::max(0.02, 0.01 * std::abs(new_traj_duration));
    if (
        msg->yaw_pts.size() < 2 ||
        !values_finite ||
        !std::isfinite(msg->yaw_dt) ||
        msg->yaw_dt <= 0.0 ||
        !std::isfinite(new_traj_duration) ||
        std::abs(yaw_duration - new_traj_duration) >
            duration_tolerance)
    {
      RCLCPP_ERROR(
          node->get_logger(),
          "[closed_loop_controller] reject traj_id=%" PRId64
          ": invalid explicit B2 "
          "yaw schedule (samples=%zu dt=%.6f yaw_duration=%.6f "
          "position_duration=%.6f).",
          msg->traj_id, msg->yaw_pts.size(), msg->yaw_dt, yaw_duration,
          new_traj_duration);
      return;
    }
    received_yaw_schedule.assign(
        msg->yaw_pts.begin(), msg->yaw_pts.end());
  }

  const auto recovery_cancel_status = reverse_recovery_policy.cancel();
  if (
      recovery_cancel_status ==
      scan_planner::B2ReverseRecoveryStatus::CANCELLED)
  {
    publishReverseRecoveryStatus(recovery_cancel_status);
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] cancel straight reverse recovery because "
        "a newer trajectory arrived.");
  }

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = new_traj_duration;
  traj_id = msg->traj_id;
  emergency_stop_trajectory = msg->emergency_stop;
  terminal_goal_trajectory =
      msg->terminal_goal && !msg->emergency_stop;
  traj_start_time = rclcpp::Time(msg->start_time, node->get_clock()->get_clock_type());
  exec_time = 0.0;
  last_update_time = node->now();
  receive_traj = true;
  b2_yaw_schedule = std::move(received_yaw_schedule);
  if (b2_yaw_schedule.empty())
    rebuildB2YawSchedule();

  // A goal callback cannot authorize terminal yaw on the spline that happened
  // to be active at callback time.  Bind only after a newer emergency
  // stationary handoff and a still newer executable spline.
  const scan_planner::GoalYawTrajectoryResult goal_yaw_transition =
      goal_yaw_handoff.receiveTrajectory(
          msg->emergency_stop,
          msg->traj_id,
          traj_start_time.seconds());
  if (goal_yaw_transition.handoff_accepted)
  {
    have_goal_yaw = false;
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] accepted stationary handoff traj_id=%" PRId64
        " for pending goal generation=%" PRIu64 ".",
        msg->traj_id, goal_yaw_handoff.goalGeneration());
  }
  if (goal_yaw_transition.yaw_bound)
  {
    goal_yaw = goal_yaw_handoff.activeYaw();
    have_goal_yaw = true;
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] bound pending final yaw %.3f rad to "
        "traj_id=%" PRId64 " after generation-matched handoff.",
        goal_yaw, msg->traj_id);
  }
  else if (goal_yaw_handoff.hasPendingYaw())
  {
    have_goal_yaw = false;
  }
  else if (goal_yaw_handoff.hasActiveYaw())
  {
    goal_yaw = goal_yaw_handoff.activeYaw();
    have_goal_yaw = true;
  }

  final_yaw_latch.reset();
  heading_alignment_latch.reset();
  resetFinalYawValidation();

  RCLCPP_WARN(
      node->get_logger(),
      "[closed_loop_controller] received bspline traj_id=%" PRId64
      " duration=%.3f "
      "emergency_stop=%s terminal_goal=%s B2_yaw=%s samples=%zu",
      traj_id, traj_duration, emergency_stop_trajectory ? "true" : "false",
      terminal_goal_trajectory ? "true" : "false",
      msg->yaw_pts.empty() ? "legacy_fallback" : "explicit",
      b2_yaw_schedule.size());
}

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = yawFromQuaternion(msg->pose.pose.orientation);
  have_odom = true;
  last_odom_time = node->now();
  if (receive_traj && b2_yaw_schedule.empty())
    rebuildB2YawSchedule();
}

void goalYawCallback(const std_msgs::msg::Float64::ConstSharedPtr &msg)
{
  const scan_planner::GoalYawInputResult result =
      goal_yaw_handoff.receiveYaw(msg->data, node->now().seconds());
  if (result == scan_planner::GoalYawInputResult::REJECTED)
  {
    RCLCPP_WARN(
        node->get_logger(),
        "[closed_loop_controller] reject non-finite final goal yaw.");
    return;
  }
  if (result == scan_planner::GoalYawInputResult::DUPLICATE)
  {
    RCLCPP_DEBUG(
        node->get_logger(),
        "[closed_loop_controller] ignore duplicate goal_yaw retransmission.");
    return;
  }

  have_goal_yaw = goal_yaw_handoff.hasActiveYaw();
  if (have_goal_yaw)
    goal_yaw = goal_yaw_handoff.activeYaw();
  final_yaw_latch.reset();
  resetFinalYawValidation();
  RCLCPP_INFO(
      node->get_logger(),
      "[closed_loop_controller] %s final goal yaw=%.3f rad "
      "(generation=%" PRIu64 ", awaiting_handoff=%s).",
      result == scan_planner::GoalYawInputResult::NEW_GOAL
          ? "received new pending"
          : "coalesced companion",
      goal_yaw_handoff.hasPendingYaw()
          ? goal_yaw_handoff.pendingYaw()
          : goal_yaw_handoff.activeYaw(),
      goal_yaw_handoff.goalGeneration(),
      goal_yaw_handoff.awaitingHandoff() ? "true" : "false");
}

void goalPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
{
  const double received_yaw = yawFromQuaternion(msg->pose.orientation);
  const scan_planner::GoalYawInputResult result =
      goal_yaw_handoff.receivePose(
          msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z,
          received_yaw,
          node->now().seconds());
  if (result == scan_planner::GoalYawInputResult::REJECTED)
  {
    RCLCPP_WARN(
        node->get_logger(),
        "[closed_loop_controller] reject goal pose with non-finite "
        "position or yaw.");
    return;
  }
  if (result == scan_planner::GoalYawInputResult::DUPLICATE)
  {
    RCLCPP_DEBUG(
        node->get_logger(),
        "[closed_loop_controller] ignore duplicate goal_pose retransmission.");
    return;
  }

  have_goal_yaw = goal_yaw_handoff.hasActiveYaw();
  if (have_goal_yaw)
    goal_yaw = goal_yaw_handoff.activeYaw();
  final_yaw_latch.reset();
  resetFinalYawValidation();
  RCLCPP_INFO(
      node->get_logger(),
      "[closed_loop_controller] %s B2 goal pose yaw=%.3f rad "
      "(generation=%" PRIu64 ", awaiting_handoff=%s).",
      result == scan_planner::GoalYawInputResult::NEW_GOAL
          ? "received new pending"
          : "coalesced companion",
      goal_yaw_handoff.hasPendingYaw()
          ? goal_yaw_handoff.pendingYaw()
          : goal_yaw_handoff.activeYaw(),
      goal_yaw_handoff.goalGeneration(),
      goal_yaw_handoff.awaitingHandoff() ? "true" : "false");
}

void safetyExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
{
  safety_execution_frozen = msg->data;
  last_safety_execution_frozen_time = node->now();
}

void reverseRecoveryRequestCallback(
    const std_msgs::msg::Bool::ConstSharedPtr &msg)
{
  if (!msg->data)
    return;
  if (reverse_recovery_policy.active())
    return;
  if (!have_odom)
  {
    publishReverseRecoveryStatus(
        scan_planner::B2ReverseRecoveryStatus::ODOMETRY_INVALID);
    return;
  }
  const rclcpp::Time now = node->now();
  if (!reverse_recovery_policy.begin(
        now.seconds(), odom_pos.x(), odom_pos.y(), odom_pos.z(), odom_yaw))
  {
    publishReverseRecoveryStatus(
        scan_planner::B2ReverseRecoveryStatus::ODOMETRY_INVALID);
    return;
  }
  reverse_recovery_generation_time = now;
  last_execution_path_publish_time =
      rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
  publishReverseRecoveryStatus(
      scan_planner::B2ReverseRecoveryStatus::ACTIVE);
  RCLCPP_WARN(
      node->get_logger(),
      "[closed_loop_controller] requested one straight reverse recovery "
      "round: distance<=%.2fm duration<=%.2fs vx=-%.3fm/s. Waiting for "
      "rear obstacle and ground safety approval.",
      reverse_recovery_max_distance,
      reverse_recovery_max_duration,
      reverse_recovery_speed);
}

void finalYawValidationCallback(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr &msg)
{
  if (
      msg->data.size() < 2 ||
      final_yaw_validation_state == FinalYawValidationState::IDLE ||
      final_yaw_validation_state == FinalYawValidationState::DENIED)
  {
    return;
  }

  const double generation = msg->data[0];
  const double decision = msg->data[1];
  if (
      !std::isfinite(generation) ||
      !std::isfinite(decision) ||
      std::abs(generation - final_yaw_validation_generation) > 1e-3)
  {
    return;
  }

  if (decision > 0.5)
  {
    if (final_yaw_validation_state != FinalYawValidationState::APPROVED)
    {
      RCLCPP_INFO(
          node->get_logger(),
          "[closed_loop_controller] final-yaw footprint safety ACK approved "
          "for generation %.6f.",
          generation);
    }
    final_yaw_validation_state = FinalYawValidationState::APPROVED;
    return;
  }

  final_yaw_validation_state = FinalYawValidationState::DENIED;
  final_yaw_safety_aborted = true;
  final_yaw_preflight_started_at = -1.0;
  force_execution_path_publish = true;
  RCLCPP_WARN(
      node->get_logger(),
      "[closed_loop_controller] final yaw alignment denied by footprint "
      "safety ACK; keeping the reached XY goal.");
}

void cmdCallback()
{
  const rclcpp::Time now = node->now();
  if (reverse_recovery_policy.active())
  {
    publishReverseRecoveryPath(now);
    const double odometry_age =
        last_odom_time.nanoseconds() > 0
            ? std::max(0.0, (now - last_odom_time).seconds())
            : std::numeric_limits<double>::infinity();
    // A stale false from the previous forward path must not authorize reverse
    // motion. The monitor has to evaluate the newly published reverse path
    // and emit at least one generation-fresh safety sample first.
    const bool have_fresh_reverse_safety_sample =
        last_safety_execution_frozen_time.nanoseconds() > 0 &&
        last_safety_execution_frozen_time >=
            reverse_recovery_generation_time;
    const bool reverse_safety_frozen =
        !have_fresh_reverse_safety_sample || safety_execution_frozen;
    const scan_planner::B2ReverseRecoveryUpdate recovery =
        reverse_recovery_policy.update(
            now.seconds(),
            odom_pos.x(),
            odom_pos.y(),
            odom_yaw,
            odometry_age,
            reverse_safety_frozen);
    if (recovery.terminal)
    {
      publishReverseRecoveryStatus(recovery.status);
      RCLCPP_WARN(
          node->get_logger(),
          "[closed_loop_controller] straight reverse recovery round ended: "
          "status=%u distance=%.3fm elapsed=%.3fs. Hold position until SCAN "
          "replans forward or requests another round.",
          static_cast<unsigned int>(recovery.status),
          recovery.distance,
          recovery.elapsed);
    }

    if (recovery.command_reverse)
    {
      publishExecutionFrozen(false);
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x =
          -std::min(reverse_recovery_speed, std::abs(max_vx));
      // Recovery is deliberately one-dimensional: no lateral velocity and
      // no yaw command are allowed in this state.
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
      cmd_vel_pub->publish(cmd);
    }
    else
    {
      publishExecutionFrozen(true);
      publishStop();
    }
    last_update_time = now;
    return;
  }

  if (reverse_recovery_policy.holding())
  {
    publishExecutionFrozen(safety_execution_frozen);
    publishStop();
    last_update_time = now;
    return;
  }

  publishExecutionPath(now);
  const bool final_yaw_validation_timed_out =
      final_yaw_validation_state == FinalYawValidationState::WAITING &&
      final_yaw_preflight_started_at >= 0.0 &&
      now.seconds() - final_yaw_preflight_started_at >=
          final_yaw_validation_timeout;

  if (safety_execution_frozen)
  {
    if (final_yaw_validation_timed_out)
    {
      final_yaw_safety_aborted = true;
      final_yaw_validation_state = FinalYawValidationState::DENIED;
      final_yaw_preflight_started_at = -1.0;
      force_execution_path_publish = true;
      RCLCPP_WARN(
          node->get_logger(),
          "[closed_loop_controller] final-yaw safety ACK timed out while "
          "execution was frozen; skipping orientation alignment and keeping "
          "the reached XY goal.");
    }
    publishExecutionFrozen(true);
    publishStop();
    last_update_time = now;
    return;
  }

  if (!receive_traj || !have_odom)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  // An emergency stationary spline is a stop command, not a completed route.
  // Never let it enter the terminal-yaw latch or rotate toward a stale goal.
  if (emergency_stop_trajectory)
  {
    publishExecutionFrozen(false);
    publishStop();
    last_update_time = now;
    return;
  }

  double dt = (now - last_update_time).seconds();
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  const double t_eval = std::min(exec_time, traj_duration);
  Eigen::Vector3d pos_des = traj[0].evaluateDeBoorT(t_eval);
  Eigen::Vector3d vel_des = traj[1].evaluateDeBoorT(t_eval);
  Eigen::Vector2d pos_err(pos_des(0) - odom_pos(0), pos_des(1) - odom_pos(1));

  const double final_yaw_error = normalizeAngle(goal_yaw - odom_yaw);
  scan_planner::FinalYawDecision final_yaw_decision;
  if (terminal_goal_trajectory)
  {
    final_yaw_decision = final_yaw_latch.update(
        exec_time >= traj_duration,
        pos_err.norm(),
        finish_dist,
        enable_final_yaw && !final_yaw_safety_aborted,
        have_goal_yaw,
        final_yaw_error,
        final_yaw_tolerance);
  }

  if (final_yaw_decision.alignment_started)
  {
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] final yaw alignment started: target=%.3f current=%.3f "
        "error=%.3f tolerance=%.3f rad; XY hold is now latched.",
        goal_yaw, odom_yaw, final_yaw_error, final_yaw_tolerance);
  }
  if (
      final_yaw_decision.alignment_completed &&
      enable_final_yaw &&
      have_goal_yaw &&
      !final_yaw_safety_aborted)
  {
    final_yaw_validation_state = FinalYawValidationState::IDLE;
    final_yaw_preflight_started_at = -1.0;
    force_execution_path_publish = true;
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] final yaw alignment completed: target=%.3f current=%.3f "
        "error=%.3f tolerance=%.3f rad.",
        goal_yaw, odom_yaw, final_yaw_error, final_yaw_tolerance);
  }
  if (final_yaw_decision.hold_position)
  {
    publishExecutionFrozen(false);
    last_update_time = now;
    geometry_msgs::msg::Twist cmd;
    if (final_yaw_latch.isAligning())
    {
      if (final_yaw_validation_timed_out)
      {
        final_yaw_validation_state = FinalYawValidationState::DENIED;
        final_yaw_safety_aborted = true;
        final_yaw_preflight_started_at = -1.0;
        force_execution_path_publish = true;
        RCLCPP_WARN(
            node->get_logger(),
            "[closed_loop_controller] final-yaw safety ACK timed out; "
            "skipping orientation alignment and keeping the reached XY goal.");
      }
      else if (
          final_yaw_validation_state ==
          FinalYawValidationState::APPROVED)
      {
        cmd.angular.z = enforceMinimumSignedMagnitude(
            clamp(kp_yaw * final_yaw_error, -max_vyaw, max_vyaw),
            min_in_place_vyaw);
      }
    }
    cmd_vel_pub->publish(cmd);
    return;
  }

  const double yaw_des = estimateDesiredYaw(t_eval, pos_des);
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);
  const bool heading_alignment_active =
      heading_alignment_latch.update(yaw_err);
  const double yaw_rate_ff =
      !heading_alignment_active && b2_yaw_schedule.size() >= 2
          ? scan_planner::b2YawScheduleRate(
                b2_yaw_schedule, traj_duration, t_eval)
          : 0.0;
  const double vyaw_cmd = clamp(
      yaw_feedforward_gain * yaw_rate_ff + kp_yaw * yaw_err,
      -max_vyaw, max_vyaw);

  if (heading_alignment_active)
  {
    publishExecutionFrozen(true);
    publishStop(enforceMinimumSignedMagnitude(
        vyaw_cmd, min_in_place_vyaw));
    last_update_time = now; // freeze exec_time while rotating in place
    return;
  }

  publishExecutionFrozen(false);
  exec_time = std::min(traj_duration, exec_time + dt);
  last_update_time = now;

  pos_des = traj[0].evaluateDeBoorT(exec_time);
  vel_des = traj[1].evaluateDeBoorT(exec_time);

  pos_err = Eigen::Vector2d(pos_des(0) - odom_pos(0), pos_des(1) - odom_pos(1));
  Eigen::Vector2d vel_ff(vel_des(0), vel_des(1));
  Eigen::Vector2d vel_world = clampNorm(vel_ff + kp_pos * pos_err, std::max(max_vx, max_vy));
  const double planar_speed = vel_world.norm();
  if (
      pos_err.norm() >= finish_dist &&
      planar_speed > 1e-6 &&
      planar_speed < min_translation_speed) {
    vel_world *= min_translation_speed / planar_speed;
  }

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::msg::Twist cmd;
  const double body_vx = c * vel_world(0) + s * vel_world(1);
  cmd.linear.x = clamp(
      body_vx, allow_reverse ? -max_vx : 0.0, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;

  cmd_vel_pub->publish(cmd);
}
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>(
      "closed_loop_controller",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  if (!loadParams())
    return 1;

  bspline_sub = node->create_subscription<scan_planner::msg::Bspline>(
      "planning/bspline", 10, bsplineCallback);
  odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      body_pose_topic, 20, odomCallback);
  goal_yaw_sub = node->create_subscription<std_msgs::msg::Float64>(
      "goal_yaw", 10, goalYawCallback);
  goal_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic, 10, goalPoseCallback);
  safety_execution_frozen_sub = node->create_subscription<std_msgs::msg::Bool>(
      safety_execution_frozen_topic, 10, safetyExecutionFrozenCallback);
  reverse_recovery_request_sub =
      node->create_subscription<std_msgs::msg::Bool>(
          reverse_recovery_request_topic,
          10,
          reverseRecoveryRequestCallback);
  final_yaw_validation_sub =
      node->create_subscription<std_msgs::msg::Float64MultiArray>(
          final_yaw_validation_topic, 10, finalYawValidationCallback);
  cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
  reverse_recovery_status_pub =
      node->create_publisher<std_msgs::msg::UInt8>(
          reverse_recovery_status_topic, 10);
  execution_frozen_pub = node->create_publisher<std_msgs::msg::Bool>(
      execution_frozen_topic, 10);
  if (
      !legacy_execution_frozen_topic.empty() &&
      legacy_execution_frozen_topic != execution_frozen_topic)
  {
    legacy_execution_frozen_pub = node->create_publisher<std_msgs::msg::Bool>(
        legacy_execution_frozen_topic, 10);
  }
  execution_path_pub = node->create_publisher<nav_msgs::msg::Path>(execution_path_topic, 10);
  cmd_timer = node->create_wall_timer(std::chrono::milliseconds(10), cmdCallback);

  last_update_time = node->now();
  RCLCPP_WARN(node->get_logger(), "[closed_loop_controller] ready.");

  rclcpp::spin(node);

  // Explicitly tear down namespace-scope ROS entities before the context.
  // Static destruction after rclcpp::shutdown() used to make repeated RViz
  // simulation runs hang or terminate the process uncleanly.
  cmd_timer.reset();
  final_yaw_validation_sub.reset();
  reverse_recovery_request_sub.reset();
  safety_execution_frozen_sub.reset();
  goal_pose_sub.reset();
  goal_yaw_sub.reset();
  odom_sub.reset();
  bspline_sub.reset();
  execution_path_pub.reset();
  reverse_recovery_status_pub.reset();
  legacy_execution_frozen_pub.reset();
  execution_frozen_pub.reset();
  cmd_vel_pub.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
