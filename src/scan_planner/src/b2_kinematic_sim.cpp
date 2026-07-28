#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "scan_planner/ground_z_tracker.h"
#include "scan_planner/terrain_z_tracker.h"

namespace
{
constexpr double kMaxVYawLimit = 1.0;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr execution_path_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub;
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
bool terrain_z_tracking_enabled = true;
std::string frame_id = "world";
std::string child_frame_id = "base";
std::string body_pose_topic = "/quad_0/body_pose";
std::string execution_path_topic = "/scan/execution_path";
std::string ground_topic = "/mapground";
scan_planner::TerrainZTracker terrain_z_tracker;
scan_planner::GroundZTracker ground_z_tracker;
std::size_t indexed_ground_data_size = 0;
std::uint32_t indexed_ground_width = 0;
std::uint32_t indexed_ground_height = 0;

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

void executionPathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
{
  if (!terrain_z_tracking_enabled || !msg)
    return;

  if ((!msg->header.frame_id.empty() && msg->header.frame_id != frame_id))
  {
    terrain_z_tracker.clearPath();
    RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "[B2 kinematic sim] Ignore execution path in frame '%s'; expected '%s'.",
        msg->header.frame_id.c_str(), frame_id.c_str());
    return;
  }

  std::vector<scan_planner::TerrainPathPoint> points;
  points.reserve(msg->poses.size());
  for (const auto &pose_stamped : msg->poses)
  {
    if (!pose_stamped.header.frame_id.empty() &&
        pose_stamped.header.frame_id != frame_id)
    {
      terrain_z_tracker.clearPath();
      RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "[B2 kinematic sim] Ignore execution path pose in frame '%s'; "
          "expected '%s'.",
          pose_stamped.header.frame_id.c_str(), frame_id.c_str());
      return;
    }
    const auto &position = pose_stamped.pose.position;
    points.push_back({position.x, position.y, position.z});
  }

  if (!terrain_z_tracker.setExecutionPath(points, node->now().seconds()) &&
      !points.empty())
  {
    RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "[B2 kinematic sim] Ignore invalid execution path: non-finite value, "
        "vertical jump, or slope above %.3f.",
        terrain_z_tracker.config().max_path_slope);
  }
}

void groundCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  if (!terrain_z_tracking_enabled || !msg)
    return;
  if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id)
  {
    ground_z_tracker.clear();
    indexed_ground_data_size = 0;
    indexed_ground_width = 0;
    indexed_ground_height = 0;
    RCLCPP_WARN(
        node->get_logger(),
        "[B2 kinematic sim] Ignore ground cloud in frame '%s'; expected '%s'.",
        msg->header.frame_id.c_str(), frame_id.c_str());
    return;
  }
  if (
      ground_z_tracker.ready() &&
      msg->data.size() == indexed_ground_data_size &&
      msg->width == indexed_ground_width &&
      msg->height == indexed_ground_height)
  {
    // nav_pcd_map_publisher republishes the immutable static cloud. Avoid
    // rebuilding the 143k-point spatial index every second.
    return;
  }

  std::vector<scan_planner::GroundSurfacePoint> points;
  points.reserve(
      static_cast<std::size_t>(msg->width) *
      static_cast<std::size_t>(msg->height));
  try
  {
    sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_iterator(*msg, "z");
    for (;
         x_iterator != x_iterator.end();
         ++x_iterator, ++y_iterator, ++z_iterator)
    {
      points.push_back(
          {
              static_cast<double>(*x_iterator),
              static_cast<double>(*y_iterator),
              static_cast<double>(*z_iterator),
          });
    }
  }
  catch (const std::exception &error)
  {
    ground_z_tracker.clear();
    RCLCPP_ERROR(
        node->get_logger(),
        "[B2 kinematic sim] Cannot decode ground cloud: %s",
        error.what());
    return;
  }

  if (!ground_z_tracker.setGroundPoints(points))
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "[B2 kinematic sim] Ground cloud contains no finite XYZ points.");
    return;
  }
  indexed_ground_data_size = msg->data.size();
  indexed_ground_width = msg->width;
  indexed_ground_height = msg->height;
  RCLCPP_INFO(
      node->get_logger(),
      "[B2 kinematic sim] Indexed %zu ground points; simulated z now follows "
      "the vertically continuous /mapground layer instead of feeding back "
      "from the SCAN execution path.",
      ground_z_tracker.pointCount());
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
  if (terrain_z_tracking_enabled)
  {
    // /mapground is the authoritative terrain source. The execution path is
    // retained only as a compatibility fallback while a ground cloud has not
    // arrived; using it after that would recreate an odometry/path feedback
    // loop whenever SCAN replans from a height-tracking error.
    if (ground_z_tracker.ready())
      z = ground_z_tracker.update(x, y, z, dt);
    else
      z = terrain_z_tracker.update(x, y, z, now.seconds(), dt);
  }

  publishOdom(now);
}
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>(
      "b2_kinematic_sim",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  body_pose_topic = getParam<std::string>("body_pose_topic", "/quad_0/body_pose");
  x = getParam<double>("init_x", 0.0);
  y = getParam<double>("init_y", 0.0);
  z = getParam<double>("init_z", 0.3);
  yaw = getParam<double>("init_yaw", 0.0);
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(yaw))
  {
    RCLCPP_FATAL(
        node->get_logger(),
        "[B2 kinematic sim] init_x/init_y/init_z/init_yaw must be finite.");
    rclcpp::shutdown();
    return 1;
  }
  loadParamWithFallback("max_vx", "max_vx", max_vx, 0.8);
  loadParamWithFallback("max_vy", "max_vy", max_vy, 0.5);
  loadParamWithFallback("max_vyaw", "max_vyaw", max_vyaw, kMaxVYawLimit);
  if (max_vyaw > kMaxVYawLimit)
  {
    RCLCPP_WARN(node->get_logger(), "[B2 kinematic sim] cap max_vyaw %.3f to %.3f rad/s.",
                max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  cmd_timeout = getParam<double>("cmd_timeout", 0.3);
  sim_rate = getParam<double>("sim_rate", 100.0);
  publish_tf = getParam<bool>("publish_tf", false);
  frame_id = getParam<std::string>("frame_id", "world");
  child_frame_id = getParam<std::string>("child_frame_id", "base");
  terrain_z_tracking_enabled =
      getParam<bool>("terrain_z_tracking_enabled", true);
  execution_path_topic = getParam<std::string>(
      "execution_path_topic", "/scan/execution_path");
  ground_topic = getParam<std::string>(
      "terrain_ground_topic", "/mapground");
  scan_planner::TerrainZTrackerConfig terrain_config;
  terrain_config.body_height =
      getParam<double>("terrain_body_height", 0.32);
  terrain_config.path_timeout =
      getParam<double>("terrain_path_timeout", 2.0);
  terrain_config.max_path_slope =
      getParam<double>("terrain_max_path_slope", 0.70);
  terrain_config.max_z_rate =
      getParam<double>("terrain_max_z_rate", 0.30);
  terrain_config.max_projection_distance =
      getParam<double>("terrain_max_projection_distance", 1.0);
  terrain_z_tracker.configure(terrain_config);
  scan_planner::GroundZTrackerConfig ground_config;
  ground_config.bucket_size =
      getParam<double>("terrain_ground_bucket_size", 0.20);
  ground_config.xy_tolerance =
      getParam<double>("terrain_ground_xy_tolerance", 0.15);
  ground_config.maximum_layer_distance =
      getParam<double>("terrain_ground_max_layer_distance", 0.50);
  ground_config.max_z_rate = terrain_config.max_z_rate;
  ground_z_tracker.configure(ground_config);

  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(body_pose_topic, 100);
  cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 20, cmdCallback);
  if (terrain_z_tracking_enabled)
  {
    execution_path_sub = node->create_subscription<nav_msgs::msg::Path>(
        execution_path_topic, 10, executionPathCallback);
    const auto ground_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    ground_sub =
        node->create_subscription<sensor_msgs::msg::PointCloud2>(
            ground_topic, ground_qos, groundCallback);
  }

  last_cmd_time = node->now();
  last_sim_time = node->now();
  sim_timer = node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / sim_rate)),
      simCallback);

  RCLCPP_WARN(
      node->get_logger(),
      "[B2 kinematic sim] ready. terrain_z_tracking=%s path=%s "
      "ground=%s body_height=%.3f timeout=%.2fs max_slope=%.3f "
      "ground_xy=%.3fm max_layer_step=%.3fm max_z_rate=%.3fm/s.",
      terrain_z_tracking_enabled ? "true" : "false",
      execution_path_topic.c_str(),
      ground_topic.c_str(),
      terrain_z_tracker.config().body_height,
      terrain_z_tracker.config().path_timeout,
      terrain_z_tracker.config().max_path_slope,
      ground_z_tracker.config().xy_tolerance,
      ground_z_tracker.config().maximum_layer_distance,
      terrain_z_tracker.config().max_z_rate);

  rclcpp::spin(node);

  // These objects live at namespace scope for the legacy callback layout.
  // Destroy ROS entities while the context and node are still valid instead
  // of leaving their static destructors to run after rclcpp::shutdown().
  sim_timer.reset();
  ground_sub.reset();
  execution_path_sub.reset();
  cmd_sub.reset();
  odom_pub.reset();
  tf_broadcaster.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
