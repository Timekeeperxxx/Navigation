#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
constexpr double kMaxVYawLimit = 1.0;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
rclcpp::TimerBase::SharedPtr sim_timer;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

double x = 0.0;
double y = 0.0;
double z = 0.0;
double yaw = 0.0;

double vx_cmd = 0.0;
double vy_cmd = 0.0;
double vyaw_cmd = 0.0;
double vx_world = 0.0;
double vy_world = 0.0;

double max_vx = 0.8;
double max_vy = 0.5;
double max_vyaw = kMaxVYawLimit;
double cmd_timeout = 0.3;
double sim_rate = 100.0;
bool publish_tf = false;
std::string frame_id = "world";
std::string child_frame_id = "base";
std::string body_pose_topic = "/quad_0/body_pose";

rclcpp::Time last_cmd_time;
rclcpp::Time last_sim_time;

template <typename T>
T getParam(const std::string &name, const T &default_value)
{
  if (!node->has_parameter(name))
    node->declare_parameter<T>(name, default_value);
  return node->get_parameter(name).get_value<T>();
}

bool getProvidedParam(const std::string &name, double &value)
{
  return node->has_parameter(name) && node->get_parameter(name, value);
}

bool getClosedLoopFallbackParam(const std::string &name, double &value)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/closed_loop_controller");
  if (!client->wait_for_service(std::chrono::milliseconds(100)))
    return false;

  const auto values = client->get_parameters({name});
  if (values.empty() || values[0].get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
    return false;

  value = values[0].as_double();
  return true;
}

void loadParamWithFallback(const std::string &private_name, const std::string &fallback_name,
                           double &value, double default_value)
{
  if (getProvidedParam(private_name, value))
    return;
  if (getClosedLoopFallbackParam(fallback_name, value))
    return;
  value = default_value;
  if (!node->has_parameter(private_name))
    node->declare_parameter<double>(private_name, value);
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw_value)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_value);
  return tf2::toMsg(q);
}

void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
{
  vx_cmd = clamp(msg->linear.x, -max_vx, max_vx);
  vy_cmd = clamp(msg->linear.y, -max_vy, max_vy);
  vyaw_cmd = clamp(msg->angular.z, -max_vyaw, max_vyaw);
  last_cmd_time = node->now();
}

void publishOdom(const rclcpp::Time &stamp)
{
  geometry_msgs::msg::Quaternion q = quaternionFromYaw(yaw);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = q;
  odom.twist.twist.linear.x = vx_world;
  odom.twist.twist.linear.y = vy_world;
  odom.twist.twist.angular.z = vyaw_cmd;
  odom_pub->publish(odom);

  if (!publish_tf || !tf_broadcaster)
    return;

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = frame_id;
  tf_msg.child_frame_id = child_frame_id;
  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = z;
  tf_msg.transform.rotation = q;
  tf_broadcaster->sendTransform(tf_msg);
}

void simCallback()
{
  const rclcpp::Time now = node->now();
  double dt = (now - last_sim_time).seconds();
  last_sim_time = now;
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  double vx = vx_cmd;
  double vy = vy_cmd;
  double wz = vyaw_cmd;
  if ((now - last_cmd_time).seconds() > cmd_timeout)
  {
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
  }

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  vx_world = c * vx - s * vy;
  vy_world = s * vx + c * vy;

  x += vx_world * dt;
  y += vy_world * dt;
  yaw = normalizeAngle(yaw + wz * dt);

  publishOdom(now);
}
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>(
      "go2_kinematic_sim",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  body_pose_topic = getParam<std::string>("body_pose_topic", "/quad_0/body_pose");
  x = getParam<double>("init_x", 0.0);
  y = getParam<double>("init_y", 0.0);
  z = getParam<double>("init_z", 0.3);
  yaw = getParam<double>("init_yaw", 0.0);
  loadParamWithFallback("max_vx", "max_vx", max_vx, 0.8);
  loadParamWithFallback("max_vy", "max_vy", max_vy, 0.5);
  loadParamWithFallback("max_vyaw", "max_vyaw", max_vyaw, kMaxVYawLimit);
  if (max_vyaw > kMaxVYawLimit)
  {
    RCLCPP_WARN(node->get_logger(), "[Go2 kinematic sim] cap max_vyaw %.3f to %.3f rad/s.",
                max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  cmd_timeout = getParam<double>("cmd_timeout", 0.3);
  sim_rate = getParam<double>("sim_rate", 100.0);
  publish_tf = getParam<bool>("publish_tf", false);
  frame_id = getParam<std::string>("frame_id", "world");
  child_frame_id = getParam<std::string>("child_frame_id", "base");

  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(body_pose_topic, 100);
  cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 20, cmdCallback);

  last_cmd_time = node->now();
  last_sim_time = node->now();
  sim_timer = node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / sim_rate)),
      simCallback);

  RCLCPP_WARN(node->get_logger(), "[Go2 kinematic sim] ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
