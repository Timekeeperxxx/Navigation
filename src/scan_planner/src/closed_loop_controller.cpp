#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bspline_opt/uniform_bspline.h"
#include "plan_manage/b2_yaw_control.h"
#include "scan_planner/msg/bspline.hpp"

namespace
{
using scan_planner::UniformBspline;

constexpr double kMaxVYawLimit = 1.0;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr execution_path_pub;
rclcpp::Subscription<scan_planner::msg::Bspline>::SharedPtr bspline_sub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_position_sub;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr goal_yaw_sub;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr safety_speed_scale_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_execution_frozen_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr controller_reset_sub;
std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> task_start_subs;
rclcpp::TimerBase::SharedPtr cmd_timer;

bool receive_traj = false;
bool have_odom = false;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
int traj_id = 0;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;
Eigen::Vector3d goal_position = Eigen::Vector3d::Zero();

double exec_time = 0.0;
rclcpp::Time last_update_time;
rclcpp::Time traj_start_time;
rclcpp::Time last_execution_path_publish_time;

double time_forward;
double heading_error_threshold;
double heading_resume_threshold;
double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
double yaw_prediction_horizon;
double yaw_rate_filter_time_constant;
double yaw_rate_settle_threshold;
double yaw_reversal_neutral_time;
double desired_yaw_filter_time_constant;
double maximum_measured_yaw_rate;
double min_translation_speed;
double min_forward_x_speed;
double finish_dist;
double final_position_tolerance;
double final_position_kp;
double final_position_max_vx;
double final_position_max_vy;
double final_yaw_tolerance;
double final_yaw_start_dist;
double final_yaw_kp;
double final_yaw_min_vyaw;
double final_yaw_max_vyaw;
double final_yaw_timeout;
double final_yaw_progress_timeout;
double final_yaw_progress_epsilon;
double final_yaw_reversal_hold_time;
double final_yaw_reversal_hold_speed_scale;
double approach_slowdown_dist;
double approach_min_speed_scale;
bool enable_final_yaw;
bool allow_reverse;
bool final_position_allow_reverse;
bool final_yaw_timeout_mark_reached;
bool have_goal_position = false;
bool have_goal_yaw = false;
bool safety_execution_frozen = false;
bool have_safety_speed_scale = false;
bool final_yaw_aligning = false;
bool final_goal_reached = false;
bool execution_path_cleared_after_goal = false;
bool have_last_final_yaw_error = false;
bool have_last_final_yaw_cmd = false;
bool final_yaw_reversal_pending = false;
double goal_yaw = 0.0;
double last_final_yaw_error = 0.0;
double last_final_yaw_cmd = 0.0;
double safety_speed_scale = 1.0;
double safety_speed_scale_timeout = 1.0;
double best_abs_final_yaw_error = std::numeric_limits<double>::infinity();
std::string body_pose_topic;
std::string execution_path_topic;
std::string execution_path_frame;
std::string safety_execution_frozen_topic;
std::string safety_speed_scale_topic;
std::string controller_reset_topic;
std::string goal_position_topic;
std::vector<std::string> task_start_reset_topics;
double execution_path_publish_period;
double execution_path_sample_dt;
rclcpp::Time last_safety_speed_scale_time;
rclcpp::Time last_bspline_receive_time;
rclcpp::Time last_controller_reset_time;
rclcpp::Time final_yaw_align_start_time;
rclcpp::Time final_yaw_last_progress_time;
rclcpp::Time final_yaw_reversal_start_time;

scan_planner::B2YawRateEstimator yaw_rate_estimator;
scan_planner::B2DesiredYawFilter desired_yaw_filter;
scan_planner::B2PathYawControl path_yaw_control;

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
  execution_path_topic = getParamWithDefault<std::string>("execution_path_topic", "/scan/execution_path");
  execution_path_frame = getParamWithDefault<std::string>("execution_path_frame", "map");
  safety_execution_frozen_topic = getParamWithDefault<std::string>(
      "safety_execution_frozen_topic", "/planning/safety_execution_frozen");
  safety_speed_scale_topic = getParamWithDefault<std::string>(
      "safety_speed_scale_topic", "/planning/safety_speed_scale");
  controller_reset_topic = getParamWithDefault<std::string>(
      "controller_reset_topic", "/planning/controller_reset");
  goal_position_topic = getParamWithDefault<std::string>(
      "goal_position_topic", "/clicked_point");
  task_start_reset_topics = getParamWithDefault<std::vector<std::string>>(
      "task_start_reset_topics",
      std::vector<std::string>{"/nav_start", "/scheduled_task_start", "/patrol_task_start"});
  safety_speed_scale_timeout = std::max(
      getParamWithDefault<double>("safety_speed_scale_timeout", 1.0), 0.0);
  execution_path_publish_period = std::max(
      getParamWithDefault<double>("execution_path_publish_period", 0.1), 0.02);
  execution_path_sample_dt = std::max(
      getParamWithDefault<double>("execution_path_sample_dt", 0.1), 0.02);
  ok &= loadRequiredParam("time_forward", time_forward);
  ok &= loadRequiredParam("heading_error_threshold", heading_error_threshold);
  heading_resume_threshold = std::max(
      0.0,
      std::min(
          getParamWithDefault<double>("heading_resume_threshold", 0.35),
          heading_error_threshold));
  ok &= loadRequiredParam("kp_pos", kp_pos);
  ok &= loadRequiredParam("kp_yaw", kp_yaw);
  ok &= loadRequiredParam("max_vx", max_vx);
  ok &= loadRequiredParam("max_vy", max_vy);
  ok &= loadRequiredParam("max_vyaw", max_vyaw);
  yaw_prediction_horizon = std::max(
      0.0, getParamWithDefault<double>("yaw_prediction_horizon", 1.0));
  yaw_rate_filter_time_constant = std::max(
      0.0, getParamWithDefault<double>("yaw_rate_filter_time_constant", 0.15));
  yaw_rate_settle_threshold = std::max(
      0.0, getParamWithDefault<double>("yaw_rate_settle_threshold", 0.06));
  yaw_reversal_neutral_time = std::max(
      0.0, getParamWithDefault<double>("yaw_reversal_neutral_time", 0.35));
  desired_yaw_filter_time_constant = std::max(
      0.0, getParamWithDefault<double>("desired_yaw_filter_time_constant", 0.20));
  maximum_measured_yaw_rate = std::max(
      0.1, getParamWithDefault<double>("maximum_measured_yaw_rate", 2.0));
  min_translation_speed = std::max(
      0.0, getParamWithDefault<double>("min_translation_speed", 0.0));
  min_forward_x_speed = std::max(
      0.0, getParamWithDefault<double>("min_forward_x_speed", 0.25));
  ok &= loadRequiredParam("finish_dist", finish_dist);
  allow_reverse = getParamWithDefault<bool>("allow_reverse", false);
  final_position_tolerance = std::max(
      0.02, getParamWithDefault<double>("final_position_tolerance", finish_dist));
  final_position_kp = std::max(
      0.0, getParamWithDefault<double>("final_position_kp", 0.8));
  final_position_max_vx = std::max(
      0.0, getParamWithDefault<double>("final_position_max_vx", 0.10));
  final_position_max_vy = std::max(
      0.0, getParamWithDefault<double>("final_position_max_vy", 0.06));
  final_position_allow_reverse = getParamWithDefault<bool>(
      "final_position_allow_reverse", true);
  enable_final_yaw = getParamWithDefault<bool>("enable_final_yaw", true);
  final_yaw_tolerance = getParamWithDefault<double>("final_yaw_tolerance", 0.5);
  final_yaw_start_dist = std::max(
      getParamWithDefault<double>("final_yaw_start_dist", 0.5), finish_dist);
  final_yaw_kp = std::max(0.0, getParamWithDefault<double>("final_yaw_kp", 0.6));
  final_yaw_min_vyaw = std::max(
      0.0, getParamWithDefault<double>("final_yaw_min_vyaw", 0.25));
  final_yaw_max_vyaw = std::max(
      0.05, getParamWithDefault<double>("final_yaw_max_vyaw", 0.25));
  final_yaw_timeout = std::max(
      0.0, getParamWithDefault<double>("final_yaw_timeout", 8.0));
  final_yaw_progress_timeout = std::max(
      0.0, getParamWithDefault<double>("final_yaw_progress_timeout", 5.0));
  final_yaw_progress_epsilon = std::max(
      0.0, getParamWithDefault<double>("final_yaw_progress_epsilon", 0.05));
  final_yaw_reversal_hold_time = std::max(
      0.0, getParamWithDefault<double>("final_yaw_reversal_hold_time", 0.35));
  final_yaw_reversal_hold_speed_scale = clamp(
      getParamWithDefault<double>("final_yaw_reversal_hold_speed_scale", 0.35), 0.0, 1.0);
  final_yaw_timeout_mark_reached = getParamWithDefault<bool>(
      "final_yaw_timeout_mark_reached", false);
  approach_slowdown_dist = std::max(
      final_yaw_start_dist, getParamWithDefault<double>("approach_slowdown_dist", 1.5));
  approach_min_speed_scale = clamp(
      getParamWithDefault<double>("approach_min_speed_scale", 0.25), 0.05, 1.0);

  if (ok && max_vyaw > kMaxVYawLimit)
  {
    RCLCPP_WARN(node->get_logger(), "[closed_loop_controller] cap max_vyaw %.3f to %.3f rad/s.",
                max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  if (ok) {
    const double max_translation_speed = std::hypot(max_vx, max_vy);
    if (min_translation_speed > max_translation_speed) {
      RCLCPP_WARN(
          node->get_logger(),
          "[closed_loop_controller] cap min_translation_speed %.3f to planar limit %.3f m/s.",
          min_translation_speed, max_translation_speed);
      min_translation_speed = max_translation_speed;
    }
    if (min_forward_x_speed > max_vx) {
      RCLCPP_WARN(
          node->get_logger(),
          "[closed_loop_controller] cap min_forward_x_speed %.3f to max_vx %.3f m/s.",
          min_forward_x_speed, max_vx);
      min_forward_x_speed = max_vx;
    }
    if (final_yaw_max_vyaw > max_vyaw) {
      RCLCPP_WARN(
          node->get_logger(),
          "[closed_loop_controller] cap final_yaw_max_vyaw %.3f to max_vyaw %.3f rad/s.",
          final_yaw_max_vyaw, max_vyaw);
      final_yaw_max_vyaw = max_vyaw;
    }
    if (final_yaw_min_vyaw > final_yaw_max_vyaw) {
      RCLCPP_WARN(
          node->get_logger(),
          "[closed_loop_controller] cap final_yaw_min_vyaw %.3f to "
          "final_yaw_max_vyaw %.3f rad/s.",
          final_yaw_min_vyaw, final_yaw_max_vyaw);
      final_yaw_min_vyaw = final_yaw_max_vyaw;
    }
    final_position_max_vx = std::min(final_position_max_vx, max_vx);
    final_position_max_vy = std::min(final_position_max_vy, max_vy);
    yaw_rate_estimator.configure(
        yaw_rate_filter_time_constant, maximum_measured_yaw_rate);
    desired_yaw_filter.configure(desired_yaw_filter_time_constant);
    path_yaw_control.configure(
        heading_error_threshold,
        heading_resume_threshold,
        yaw_prediction_horizon,
        yaw_rate_settle_threshold,
        yaw_reversal_neutral_time);
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] translation speed range: min_norm=%.3f min_forward_x=%.3f max_x=%.3f max_y=%.3f m/s; final pose: start_dist=%.3f position_tolerance=%.3f position_kp=%.3f position_max=(%.3f,%.3f) position_reverse=%d yaw_tolerance=%.3f yaw_kp=%.3f yaw_speed=[%.3f,%.3f] rad/s; approach slowdown: dist=%.3f min_scale=%.3f.",
        min_translation_speed, min_forward_x_speed, max_vx, max_vy,
        final_yaw_start_dist, final_position_tolerance, final_position_kp,
        final_position_max_vx, final_position_max_vy,
        final_position_allow_reverse, final_yaw_tolerance, final_yaw_kp,
        final_yaw_min_vyaw, final_yaw_max_vyaw,
        approach_slowdown_dist, approach_min_speed_scale);
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] B2 path yaw: stop=%.3f resume=%.3f prediction=%.3fs "
        "rate_filter=%.3fs settle_rate=%.3frad/s reversal_neutral=%.3fs "
        "desired_filter=%.3fs reverse=%s.",
        heading_error_threshold,
        heading_resume_threshold,
        yaw_prediction_horizon,
        yaw_rate_filter_time_constant,
        yaw_rate_settle_threshold,
        yaw_reversal_neutral_time,
        desired_yaw_filter_time_constant,
        allow_reverse ? "enabled" : "disabled");
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] reset topic=%s task_start_topics=%zu; final yaw watchdog timeout=%.3f progress_timeout=%.3f progress_epsilon=%.3f mark_reached=%d.",
        controller_reset_topic.c_str(),
        task_start_reset_topics.size(),
        final_yaw_timeout,
        final_yaw_progress_timeout,
        final_yaw_progress_epsilon,
        final_yaw_timeout_mark_reached);
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

double normalizeAngleNear(double angle, double reference)
{
  angle = normalizeAngle(angle);
  while (angle - reference > M_PI)
    angle -= 2.0 * M_PI;
  while (angle - reference < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double finalYawError()
{
  if (!have_goal_yaw)
    return 0.0;

  const double raw_error = goal_yaw - odom_yaw;
  const double error = have_last_final_yaw_error
                           ? normalizeAngleNear(raw_error, last_final_yaw_error)
                           : normalizeAngle(raw_error);
  last_final_yaw_error = error;
  have_last_final_yaw_error = true;
  return error;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
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

double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des)
{
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

double effectiveSafetySpeedScale(const rclcpp::Time &now)
{
  if (!have_safety_speed_scale)
    return 1.0;
  if (safety_speed_scale_timeout > 0.0 &&
      last_safety_speed_scale_time.nanoseconds() > 0 &&
      (now - last_safety_speed_scale_time).seconds() > safety_speed_scale_timeout)
    return 1.0;
  return clamp(safety_speed_scale, 0.0, 1.0);
}

void applySafetySpeedScale(geometry_msgs::msg::Twist &cmd, const rclcpp::Time &now)
{
  const double scale = effectiveSafetySpeedScale(now);
  if (scale >= 0.999)
    return;

  cmd.linear.x *= scale;
  cmd.linear.y *= scale;
  cmd.angular.z *= scale;
  RCLCPP_INFO_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      1000,
      "[closed_loop_controller] apply safety speed scale=%.3f cmd_linear=(%.3f, %.3f) cmd_vyaw=%.3f.",
      scale,
      cmd.linear.x,
      cmd.linear.y,
      cmd.angular.z);
}

void publishStop(double vyaw = 0.0)
{
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  applySafetySpeedScale(cmd, node->now());
  cmd_vel_pub->publish(cmd);
}

void publishExecutionFrozen(bool frozen)
{
  std_msgs::msg::Bool msg;
  msg.data = frozen;
  execution_frozen_pub->publish(msg);
}

void publishExecutionPath(const rclcpp::Time &now)
{
  if (!receive_traj || traj.empty() || final_goal_reached)
    return;
  if (last_execution_path_publish_time.nanoseconds() > 0 &&
      (now - last_execution_path_publish_time).seconds() < execution_path_publish_period)
    return;

  nav_msgs::msg::Path path;
  // Keep the trajectory generation time fixed in the header.  The safety
  // monitor uses message receipt time for liveness and this stamp to reject a
  // trajectory that belongs to an older global goal.
  path.header.stamp = traj_start_time;
  path.header.frame_id = execution_path_frame;

  const double start = std::min(exec_time, traj_duration);
  auto append_pose = [&](double t) {
    const Eigen::Vector3d point = traj[0].evaluateDeBoorT(t);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point(0);
    pose.pose.position.y = point(1);
    pose.pose.position.z = point(2);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  };

  for (double t = start; t < traj_duration; t += execution_path_sample_dt)
    append_pose(t);
  append_pose(traj_duration);

  execution_path_pub->publish(path);
  last_execution_path_publish_time = now;
}

void clearExecutionPathAfterGoal(const rclcpp::Time &now)
{
  if (execution_path_cleared_after_goal)
    return;

  nav_msgs::msg::Path path;
  path.header.stamp = now;
  path.header.frame_id = execution_path_frame;
  execution_path_pub->publish(path);
  last_execution_path_publish_time = now;
  execution_path_cleared_after_goal = true;
}

void publishEmptyExecutionPath(const rclcpp::Time &now)
{
  if (!execution_path_pub)
    return;

  nav_msgs::msg::Path path;
  path.header.stamp = now;
  path.header.frame_id = execution_path_frame;
  execution_path_pub->publish(path);
  last_execution_path_publish_time = now;
  execution_path_cleared_after_goal = true;
}

void resetControllerState(const std::string &reason)
{
  const rclcpp::Time now = node->now();
  last_controller_reset_time = now;
  traj.clear();
  receive_traj = false;
  traj_duration = 0.0;
  traj_id = 0;
  exec_time = 0.0;
  traj_start_time = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
  last_update_time = now;

  final_yaw_aligning = false;
  final_goal_reached = false;
  have_last_final_yaw_error = false;
  have_last_final_yaw_cmd = false;
  final_yaw_reversal_pending = false;
  last_final_yaw_error = 0.0;
  last_final_yaw_cmd = 0.0;
  best_abs_final_yaw_error = std::numeric_limits<double>::infinity();
  final_yaw_align_start_time = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
  final_yaw_last_progress_time = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
  final_yaw_reversal_start_time = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
  desired_yaw_filter.reset();
  path_yaw_control.reset();

  safety_execution_frozen = false;
  safety_speed_scale = 1.0;
  have_safety_speed_scale = false;
  last_safety_speed_scale_time = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());

  publishExecutionFrozen(false);
  publishEmptyExecutionPath(now);
  publishStop();

  RCLCPP_WARN(
      node->get_logger(),
      "[closed_loop_controller] controller state reset: reason=%s.",
      reason.c_str());
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

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = traj[0].getTimeSum();
  traj_id = msg->traj_id;
  traj_start_time = rclcpp::Time(msg->start_time, node->get_clock()->get_clock_type());
  exec_time = 0.0;
  last_update_time = node->now();
  last_bspline_receive_time = last_update_time;
  receive_traj = true;
  final_yaw_aligning = false;
  final_goal_reached = false;
  execution_path_cleared_after_goal = false;
  have_last_final_yaw_error = false;
  have_last_final_yaw_cmd = false;
  final_yaw_reversal_pending = false;
  last_final_yaw_error = 0.0;
  last_final_yaw_cmd = 0.0;
  best_abs_final_yaw_error = std::numeric_limits<double>::infinity();
  final_yaw_align_start_time = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
  final_yaw_last_progress_time = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
  final_yaw_reversal_start_time = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());

  RCLCPP_WARN(node->get_logger(), "[closed_loop_controller] received bspline traj_id=%d duration=%.3f",
              traj_id, traj_duration);
}

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
  const double measured_yaw = yawFromQuaternion(msg->pose.pose.orientation);
  yaw_rate_estimator.update(measured_yaw, node->now().seconds());
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = measured_yaw;
  have_odom = true;
}

void goalYawCallback(const std_msgs::msg::Float64::ConstSharedPtr &msg)
{
  goal_yaw = normalizeAngle(msg->data);
  have_goal_yaw = true;
  have_last_final_yaw_error = false;
  have_last_final_yaw_cmd = false;
  final_yaw_reversal_pending = false;
  last_final_yaw_error = 0.0;
  last_final_yaw_cmd = 0.0;
  RCLCPP_INFO(
      node->get_logger(),
      "[closed_loop_controller] received goal yaw: goal=%.3f current=%.3f error=%.3f.",
      goal_yaw,
      odom_yaw,
      normalizeAngle(goal_yaw - odom_yaw));
}

void goalPositionCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr &msg)
{
  const Eigen::Vector3d next_goal(msg->point.x, msg->point.y, msg->point.z);
  const bool changed =
      !have_goal_position ||
      std::hypot(next_goal(0) - goal_position(0), next_goal(1) - goal_position(1)) > 1e-3;
  goal_position = next_goal;
  have_goal_position = true;
  if (changed)
  {
    final_yaw_aligning = false;
    final_goal_reached = false;
    execution_path_cleared_after_goal = false;
    have_last_final_yaw_error = false;
    have_last_final_yaw_cmd = false;
    final_yaw_reversal_pending = false;
    last_final_yaw_error = 0.0;
    last_final_yaw_cmd = 0.0;
    best_abs_final_yaw_error = std::numeric_limits<double>::infinity();
  }
  RCLCPP_INFO(
      node->get_logger(),
      "[closed_loop_controller] received navigation goal position: "
      "goal=(%.3f, %.3f, %.3f) current=(%.3f, %.3f, %.3f) distance=%.3f changed=%d.",
      goal_position(0), goal_position(1), goal_position(2),
      odom_pos(0), odom_pos(1), odom_pos(2),
      std::hypot(goal_position(0) - odom_pos(0), goal_position(1) - odom_pos(1)),
      changed);
}

void safetyExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
{
  const bool previous = safety_execution_frozen;
  safety_execution_frozen = msg->data;
  if (previous == safety_execution_frozen)
    return;
  if (safety_execution_frozen)
  {
    RCLCPP_WARN(
        node->get_logger(),
        "[closed_loop_controller] safety_execution_frozen changed: true topic=%s.",
        safety_execution_frozen_topic.c_str());
  }
  else
  {
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] safety_execution_frozen changed: false topic=%s.",
        safety_execution_frozen_topic.c_str());
  }
}

void safetySpeedScaleCallback(const std_msgs::msg::Float64::ConstSharedPtr &msg)
{
  const double previous = safety_speed_scale;
  safety_speed_scale = clamp(msg->data, 0.0, 1.0);
  last_safety_speed_scale_time = node->now();
  have_safety_speed_scale = true;
  if (std::abs(previous - safety_speed_scale) < 1e-3)
    return;
  RCLCPP_INFO(
      node->get_logger(),
      "[closed_loop_controller] safety_speed_scale changed: %.3f topic=%s.",
      safety_speed_scale,
      safety_speed_scale_topic.c_str());
}

void controllerResetCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
{
  if (!msg->data)
    return;
  resetControllerState("reset_topic");
}

void taskStartResetCallback(const std_msgs::msg::String::ConstSharedPtr &msg)
{
  resetControllerState("task_start:" + msg->data);
}

void cmdCallback()
{
  const rclcpp::Time now = node->now();
  publishExecutionPath(now);

  if (safety_execution_frozen)
  {
    RCLCPP_WARN_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        1000,
        "[closed_loop_controller] publish cmd_vel zero: reason=safety_execution_frozen "
        "receive_traj=%d have_odom=%d traj_id=%d exec_time=%.3f.",
        receive_traj,
        have_odom,
        traj_id,
        exec_time);
    publishExecutionFrozen(true);
    publishStop();
    last_update_time = now;
    return;
  }

  if (!receive_traj || !have_odom)
  {
    const double reset_age =
        last_controller_reset_time.nanoseconds() > 0
            ? (now - last_controller_reset_time).seconds()
            : -1.0;
    const double bspline_age =
        last_bspline_receive_time.nanoseconds() > 0
            ? (now - last_bspline_receive_time).seconds()
            : -1.0;
    RCLCPP_WARN_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        1000,
        "[closed_loop_controller] publish cmd_vel zero: reason=missing_input "
        "receive_traj=%d have_odom=%d traj_id=%d reset_age=%.3f last_bspline_age=%.3f "
        "final_goal_reached=%d final_yaw_aligning=%d safety_frozen=%d.",
        receive_traj,
        have_odom,
        traj_id,
        reset_age,
        bspline_age,
        final_goal_reached,
        final_yaw_aligning,
        safety_execution_frozen);
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  // After the current goal is fully reached (xy + yaw), keep stopped until a
  // new trajectory arrives. XY remains unlocked only for the next goal.
  if (final_goal_reached)
  {
    clearExecutionPathAfterGoal(now);
    RCLCPP_INFO_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        1000,
        "[closed_loop_controller] publish cmd_vel zero: reason=final_goal_reached "
        "traj_id=%d exec_time=%.3f final_yaw_aligning=%d.",
        traj_id,
        exec_time,
        final_yaw_aligning);
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
  const Eigen::Vector3d spline_final_pos = traj[0].evaluateDeBoorT(traj_duration);
  // Emergency-stop B-splines intentionally collapse to the current pose. The
  // spline endpoint therefore cannot identify navigation completion. Prefer
  // the real navigation goal so an emergency stop in the middle of a route
  // never enters terminal XY/yaw alignment at the stop location.
  const Eigen::Vector3d &final_pos = have_goal_position ? goal_position : spline_final_pos;
  const double final_dist =
      std::hypot(final_pos(0) - odom_pos(0), final_pos(1) - odom_pos(1));

  if (!final_yaw_aligning && final_dist <= final_yaw_start_dist)
  {
    final_yaw_aligning = true;
    final_yaw_align_start_time = now;
    final_yaw_last_progress_time = now;
    have_last_final_yaw_cmd = false;
    final_yaw_reversal_pending = false;
    last_final_yaw_cmd = 0.0;
    final_yaw_reversal_start_time = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
    best_abs_final_yaw_error = std::numeric_limits<double>::infinity();
    path_yaw_control.reset();
    desired_yaw_filter.reset();
    RCLCPP_INFO(
        node->get_logger(),
        "[closed_loop_controller] enter final pose alignment: final_dist=%.3f "
        "threshold=%.3f goal_source=%s final=(%.3f, %.3f).",
        final_dist,
        final_yaw_start_dist,
        have_goal_position ? "navigation_goal" : "spline_endpoint",
        final_pos(0),
        final_pos(1));
  }

  const double yaw_des_raw = estimateDesiredYaw(t_eval, pos_des);
  const double yaw_des = desired_yaw_filter.update(yaw_des_raw, now.seconds());
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);
  const scan_planner::B2YawControlResult yaw_control = path_yaw_control.update(
      yaw_err,
      yaw_rate_estimator.rate(),
      now.seconds(),
      kp_yaw,
      max_vyaw);
  const double vyaw_cmd = yaw_control.command;

  if (!final_yaw_aligning && yaw_control.alignment_active)
  {
    RCLCPP_WARN_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        500,
        "[closed_loop_controller] publish cmd_vel linear zero: reason=heading_error "
        "yaw_err=%.3f stop=%.3f resume=%.3f odom_yaw=%.3f "
        "yaw_des_raw=%.3f yaw_des=%.3f yaw_rate=%.3f predicted_err=%.3f "
        "cmd_w=%.3f reversal_braking=%d traj_id=%d exec_time=%.3f.",
        yaw_err,
        heading_error_threshold,
        heading_resume_threshold,
        odom_yaw,
        yaw_des_raw,
        yaw_des,
        yaw_rate_estimator.rate(),
        yaw_control.predicted_error,
        vyaw_cmd,
        yaw_control.reversal_braking,
        traj_id,
        exec_time);
    publishExecutionFrozen(true);
    publishStop(vyaw_cmd);
    last_update_time = now; // freeze exec_time while rotating in place
    return;
  }

  publishExecutionFrozen(false);
  exec_time = std::min(traj_duration, exec_time + dt);
  last_update_time = now;

  pos_des = traj[0].evaluateDeBoorT(exec_time);
  vel_des = traj[1].evaluateDeBoorT(exec_time);

  Eigen::Vector2d pos_err(pos_des(0) - odom_pos(0), pos_des(1) - odom_pos(1));
  Eigen::Vector2d vel_ff(vel_des(0), vel_des(1));
  Eigen::Vector2d vel_world = clampNorm(vel_ff + kp_pos * pos_err, std::max(max_vx, max_vy));
  if (final_dist < approach_slowdown_dist)
  {
    const double ratio = clamp(final_dist / approach_slowdown_dist, 0.0, 1.0);
    const double speed_scale = approach_min_speed_scale + (1.0 - approach_min_speed_scale) * ratio;
    vel_world *= speed_scale;
  }
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
  cmd.linear.x = clamp(body_vx, allow_reverse ? -max_vx : 0.0, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;

  if (
      min_forward_x_speed > 0.0 &&
      final_dist > final_yaw_start_dist &&
      body_vx > 1e-3 &&
      cmd.linear.x < min_forward_x_speed) {
    const double original_x = cmd.linear.x;
    cmd.linear.x = min_forward_x_speed;
    RCLCPP_INFO_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        500,
        "[closed_loop_controller] enforce min forward x speed: original_x=%.3f "
        "new_x=%.3f final_dist=%.3f final_yaw_start_dist=%.3f.",
        original_x,
        cmd.linear.x,
        final_dist,
        final_yaw_start_dist);
  }

  if (final_yaw_aligning)
  {
    const double final_yaw_error = have_goal_yaw ? finalYawError() : 0.0;
    const double abs_final_yaw_error = std::abs(final_yaw_error);
    if (final_yaw_align_start_time.nanoseconds() == 0)
    {
      final_yaw_align_start_time = now;
      final_yaw_last_progress_time = now;
      best_abs_final_yaw_error = abs_final_yaw_error;
    }
    else if (abs_final_yaw_error + final_yaw_progress_epsilon < best_abs_final_yaw_error)
    {
      best_abs_final_yaw_error = abs_final_yaw_error;
      final_yaw_last_progress_time = now;
    }

    const double final_yaw_elapsed = (now - final_yaw_align_start_time).seconds();
    const double final_yaw_no_progress =
        final_yaw_last_progress_time.nanoseconds() > 0
            ? (now - final_yaw_last_progress_time).seconds()
            : 0.0;
    const bool final_yaw_timed_out =
        (final_yaw_timeout > 0.0 && final_yaw_elapsed > final_yaw_timeout) ||
        (final_yaw_progress_timeout > 0.0 &&
         final_yaw_no_progress > final_yaw_progress_timeout &&
         abs_final_yaw_error > final_yaw_tolerance);

    // SCAN hands the final spline to this controller once the body enters the
    // terminal region.  Keep correcting XY while turning: a B2 can translate
    // noticeably during an in-place yaw and a one-way XY latch otherwise
    // reports success well outside the requested point.
    cmd = geometry_msgs::msg::Twist();
    const auto final_position_command =
        scan_planner::computeB2FinalPositionCommand(
            final_pos(0) - odom_pos(0),
            final_pos(1) - odom_pos(1),
            odom_yaw,
            final_position_tolerance,
            final_position_kp,
            final_position_max_vx,
            final_position_max_vy,
            final_position_allow_reverse);
    cmd.linear.x = final_position_command.body_x;
    cmd.linear.y = final_position_command.body_y;
    bool final_yaw_reversal_held = false;
    if (enable_final_yaw && have_goal_yaw && abs_final_yaw_error > final_yaw_tolerance)
    {
      double desired_vyaw =
          clamp(final_yaw_kp * final_yaw_error, -final_yaw_max_vyaw, final_yaw_max_vyaw);
      if (std::abs(desired_vyaw) < final_yaw_min_vyaw)
        desired_vyaw = std::copysign(final_yaw_min_vyaw, final_yaw_error);
      const bool reversing =
          have_last_final_yaw_cmd &&
          std::abs(last_final_yaw_cmd) > 1e-3 &&
          std::abs(desired_vyaw) > 1e-3 &&
          last_final_yaw_cmd * desired_vyaw < 0.0;

      if (reversing && final_yaw_reversal_hold_time > 0.0)
      {
        if (!final_yaw_reversal_pending)
        {
          final_yaw_reversal_pending = true;
          final_yaw_reversal_start_time = now;
        }

        const double reversal_elapsed = (now - final_yaw_reversal_start_time).seconds();
        if (reversal_elapsed < final_yaw_reversal_hold_time)
        {
          const double held_vyaw =
              clamp(last_final_yaw_cmd * final_yaw_reversal_hold_speed_scale,
                    -final_yaw_max_vyaw, final_yaw_max_vyaw);
          cmd.angular.z = held_vyaw;
          final_yaw_reversal_held = true;
        }
        else
        {
          cmd.angular.z = desired_vyaw;
          final_yaw_reversal_pending = false;
        }
      }
      else
      {
        cmd.angular.z = desired_vyaw;
        final_yaw_reversal_pending = false;
      }

      if (!final_yaw_reversal_held && std::abs(cmd.angular.z) > 1e-3)
      {
        last_final_yaw_cmd = cmd.angular.z;
        have_last_final_yaw_cmd = true;
      }
    }
    else
    {
      final_yaw_reversal_pending = false;
    }
    RCLCPP_INFO_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        500,
        "[closed_loop_controller] final pose aligning: current_final_dist=%.3f "
        "position_tolerance=%.3f xy_aligned=%d entry_threshold=%.3f "
        "yaw_error=%.3f yaw_tolerance=%.3f cmd_linear=(%.3f, %.3f) "
        "cmd_vyaw=%.3f reversal_hold=%d elapsed=%.3f no_progress=%.3f.",
        final_dist,
        final_position_tolerance,
        final_position_command.aligned,
        final_yaw_start_dist,
        final_yaw_error,
        final_yaw_tolerance,
        cmd.linear.x,
        cmd.linear.y,
        cmd.angular.z,
        final_yaw_reversal_held,
        final_yaw_elapsed,
        final_yaw_no_progress);

    if (
        final_yaw_timed_out &&
        final_yaw_timeout_mark_reached &&
        final_position_command.aligned)
    {
      final_goal_reached = true;
      final_yaw_aligning = false;
      have_last_final_yaw_cmd = false;
      final_yaw_reversal_pending = false;
      last_final_yaw_cmd = 0.0;
      cmd = geometry_msgs::msg::Twist();
      RCLCPP_WARN(
          node->get_logger(),
          "[closed_loop_controller] final yaw watchdog marked reached: "
          "elapsed=%.3f no_progress=%.3f yaw_error=%.3f best_abs_error=%.3f tolerance=%.3f.",
          final_yaw_elapsed,
          final_yaw_no_progress,
          final_yaw_error,
          best_abs_final_yaw_error,
          final_yaw_tolerance);
    }
    else if (final_yaw_timed_out)
    {
      resetControllerState("final_yaw_watchdog_timeout");
      return;
    }
    else if (scan_planner::isB2FinalPoseReached(
                 final_dist,
                 final_yaw_error,
                 final_position_tolerance,
                 final_yaw_tolerance,
                 enable_final_yaw && have_goal_yaw))
    {
      final_goal_reached = true;
      final_yaw_aligning = false;
      have_last_final_yaw_cmd = false;
      final_yaw_reversal_pending = false;
      last_final_yaw_cmd = 0.0;
      cmd = geometry_msgs::msg::Twist();
      RCLCPP_INFO(
          node->get_logger(),
          "[closed_loop_controller] navigation point reached by simultaneous XY and yaw alignment: "
          "current_final_dist=%.3f position_tolerance=%.3f yaw_error=%.3f yaw_tolerance=%.3f.",
          final_dist,
          final_position_tolerance,
          final_yaw_error,
          final_yaw_tolerance);
    }
  }

  applySafetySpeedScale(cmd, now);
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
  goal_position_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
      goal_position_topic, 10, goalPositionCallback);
  goal_yaw_sub = node->create_subscription<std_msgs::msg::Float64>(
      "goal_yaw", 10, goalYawCallback);
  safety_execution_frozen_sub = node->create_subscription<std_msgs::msg::Bool>(
      safety_execution_frozen_topic, 10, safetyExecutionFrozenCallback);
  safety_speed_scale_sub = node->create_subscription<std_msgs::msg::Float64>(
      safety_speed_scale_topic, 10, safetySpeedScaleCallback);
  controller_reset_sub = node->create_subscription<std_msgs::msg::Bool>(
      controller_reset_topic, 10, controllerResetCallback);
  for (const auto &topic : task_start_reset_topics)
  {
    if (topic.empty())
      continue;
    task_start_subs.push_back(
        node->create_subscription<std_msgs::msg::String>(
            topic, 10, taskStartResetCallback));
  }
  cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
  execution_frozen_pub = node->create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
  execution_path_pub = node->create_publisher<nav_msgs::msg::Path>(execution_path_topic, 10);
  cmd_timer = node->create_wall_timer(std::chrono::milliseconds(10), cmdCallback);

  last_update_time = node->now();
  RCLCPP_WARN(node->get_logger(), "[closed_loop_controller] ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
