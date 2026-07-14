#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bspline_opt/uniform_bspline.h"
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
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr goal_yaw_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_execution_frozen_sub;
rclcpp::TimerBase::SharedPtr cmd_timer;

bool receive_traj = false;
bool have_odom = false;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
int traj_id = 0;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;

double exec_time = 0.0;
rclcpp::Time last_update_time;
rclcpp::Time traj_start_time;
rclcpp::Time last_execution_path_publish_time;

double time_forward;
double heading_error_threshold;
double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
double finish_dist;
double final_yaw_tolerance;
bool enable_final_yaw;
bool have_goal_yaw = false;
bool safety_execution_frozen = false;
double goal_yaw = 0.0;
std::string body_pose_topic;
std::string execution_path_topic;
std::string execution_path_frame;
std::string safety_execution_frozen_topic;
double execution_path_publish_period;
double execution_path_sample_dt;

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
  execution_path_publish_period = std::max(
      getParamWithDefault<double>("execution_path_publish_period", 0.1), 0.02);
  execution_path_sample_dt = std::max(
      getParamWithDefault<double>("execution_path_sample_dt", 0.1), 0.02);
  ok &= loadRequiredParam("time_forward", time_forward);
  ok &= loadRequiredParam("heading_error_threshold", heading_error_threshold);
  ok &= loadRequiredParam("kp_pos", kp_pos);
  ok &= loadRequiredParam("kp_yaw", kp_yaw);
  ok &= loadRequiredParam("max_vx", max_vx);
  ok &= loadRequiredParam("max_vy", max_vy);
  ok &= loadRequiredParam("max_vyaw", max_vyaw);
  ok &= loadRequiredParam("finish_dist", finish_dist);
  enable_final_yaw = getParamWithDefault<bool>("enable_final_yaw", true);
  final_yaw_tolerance = getParamWithDefault<double>("final_yaw_tolerance", 0.15);

  if (ok && max_vyaw > kMaxVYawLimit)
  {
    RCLCPP_WARN(node->get_logger(), "[closed_loop_controller] cap max_vyaw %.3f to %.3f rad/s.",
                max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
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
}

void publishExecutionPath(const rclcpp::Time &now)
{
  if (!receive_traj || traj.empty())
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
  receive_traj = true;

  RCLCPP_WARN(node->get_logger(), "[closed_loop_controller] received bspline traj_id=%d duration=%.3f",
              traj_id, traj_duration);
}

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = yawFromQuaternion(msg->pose.pose.orientation);
  have_odom = true;
}

void goalYawCallback(const std_msgs::msg::Float64::ConstSharedPtr &msg)
{
  goal_yaw = normalizeAngle(msg->data);
  have_goal_yaw = true;
}

void safetyExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
{
  safety_execution_frozen = msg->data;
}

void cmdCallback()
{
  const rclcpp::Time now = node->now();
  publishExecutionPath(now);

  if (safety_execution_frozen)
  {
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

  double dt = (now - last_update_time).seconds();
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  const double t_eval = std::min(exec_time, traj_duration);
  Eigen::Vector3d pos_des = traj[0].evaluateDeBoorT(t_eval);
  Eigen::Vector3d vel_des = traj[1].evaluateDeBoorT(t_eval);

  const double yaw_des = estimateDesiredYaw(t_eval, pos_des);
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);
  const double vyaw_cmd = clamp(kp_yaw * yaw_err, -max_vyaw, max_vyaw);

  if (std::abs(yaw_err) > heading_error_threshold)
  {
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

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;

  if (exec_time >= traj_duration && pos_err.norm() < finish_dist)
  {
    cmd = geometry_msgs::msg::Twist();
    if (enable_final_yaw && have_goal_yaw)
    {
      const double final_yaw_error = normalizeAngle(goal_yaw - odom_yaw);
      if (std::abs(final_yaw_error) > final_yaw_tolerance)
        cmd.angular.z = clamp(kp_yaw * final_yaw_error, -max_vyaw, max_vyaw);
    }
  }

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
  safety_execution_frozen_sub = node->create_subscription<std_msgs::msg::Bool>(
      safety_execution_frozen_topic, 10, safetyExecutionFrozenCallback);
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
