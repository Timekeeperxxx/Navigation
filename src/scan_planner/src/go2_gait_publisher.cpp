#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

class Go2GaitPublisher {
 public:
  explicit Go2GaitPublisher(const rclcpp::Node::SharedPtr& node) : node_(node) {
    rate_ = getParam<double>("rate", 60.0);
    gait_frequency_ = getParam<double>("gait_frequency", 2.2);
    min_walk_speed_ = getParam<double>("min_walk_speed", 0.05);
    max_walk_speed_ = getParam<double>("max_walk_speed", 1.0);
    always_trot_ = getParam<bool>("always_trot", false);
    hip_swing_ = getParam<double>("hip_swing", 0.08);
    thigh_swing_ = getParam<double>("thigh_swing", 0.32);
    calf_swing_ = getParam<double>("calf_swing", 0.42);

    const std::string body_pose_topic = getParam<std::string>("body_pose_topic", "/quad_0/body_pose");
    const std::string joint_topic = getParam<std::string>("joint_topic", "/joint_states");

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        body_pose_topic, 10, std::bind(&Go2GaitPublisher::odomCallback, this, std::placeholders::_1));
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_topic, 10);
    timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_)),
        std::bind(&Go2GaitPublisher::timerCallback, this));

    joint_msg_.name = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
    joint_msg_.position.resize(joint_msg_.name.size(), 0.0);
    joint_msg_.velocity.resize(joint_msg_.name.size(), 0.0);
  }

 private:
  template <typename T>
  T getParam(const std::string& name, const T& default_value) {
    if (!node_->has_parameter(name))
      node_->declare_parameter<T>(name, default_value);
    return node_->get_parameter(name).get_value<T>();
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
    const double vx = odom->twist.twist.linear.x;
    const double vy = odom->twist.twist.linear.y;
    const double twist_speed = std::sqrt(vx * vx + vy * vy);
    rclcpp::Time stamp(odom->header.stamp);
    if (stamp.nanoseconds() == 0)
      stamp = node_->now();
    const double x = odom->pose.pose.position.x;
    const double y = odom->pose.pose.position.y;

    double pose_speed = 0.0;
    if (has_prev_pose_) {
      const double dt = (stamp - last_odom_time_).seconds();
      if (dt > 1e-4) {
        const double dx = x - last_odom_x_;
        const double dy = y - last_odom_y_;
        pose_speed = std::sqrt(dx * dx + dy * dy) / dt;
      }
    }

    horizontal_speed_ = twist_speed > min_walk_speed_ ? twist_speed : pose_speed;
    last_odom_x_ = x;
    last_odom_y_ = y;
    last_odom_time_ = stamp;
    has_prev_pose_ = true;
    has_odom_ = true;
  }

  void timerCallback() {
    const rclcpp::Time stamp = node_->now();
    if (!has_odom_) {
      publishStance(stamp);
      return;
    }

    const double speed_ratio =
        always_trot_ ? 0.45 : clamp((horizontal_speed_ - min_walk_speed_) /
                                        std::max(1e-3, max_walk_speed_ - min_walk_speed_),
                                    0.0, 1.0);

    if (speed_ratio <= 1e-3) {
      publishStance(stamp);
      return;
    }

    const double phase = 2.0 * kPi * gait_frequency_ * stamp.seconds();
    fillLeg(0, phase, speed_ratio, true);
    fillLeg(3, phase + kPi, speed_ratio, false);
    fillLeg(6, phase + kPi, speed_ratio, true);
    fillLeg(9, phase, speed_ratio, false);

    joint_msg_.header.stamp = stamp;
    joint_pub_->publish(joint_msg_);
  }

  void publishStance(const rclcpp::Time& stamp) {
    const std::array<double, 12> stance = {
        0.05, 0.82, -1.58,
        -0.05, 0.82, -1.58,
        0.05, 0.95, -1.62,
        -0.05, 0.95, -1.62};

    for (size_t i = 0; i < stance.size(); ++i) {
      joint_msg_.position[i] = stance[i];
      joint_msg_.velocity[i] = 0.0;
    }

    joint_msg_.header.stamp = stamp;
    joint_pub_->publish(joint_msg_);
  }

  void fillLeg(size_t offset, double phase, double speed_ratio, bool left_side) {
    const double s = std::sin(phase);
    const double c = std::cos(phase);
    const double swing = std::max(0.0, s);
    const double side = left_side ? 1.0 : -1.0;

    const bool rear_leg = offset >= 6;
    const double thigh_stance = rear_leg ? 0.95 : 0.82;
    const double calf_stance = rear_leg ? -1.62 : -1.58;

    joint_msg_.position[offset + 0] =
        side * (0.05 + hip_swing_ * speed_ratio * 0.35 * s);
    joint_msg_.position[offset + 1] =
        thigh_stance + thigh_swing_ * speed_ratio * c;
    joint_msg_.position[offset + 2] =
        calf_stance + calf_swing_ * speed_ratio * swing - 0.10 * speed_ratio * (1.0 - swing);

    joint_msg_.position[offset + 0] = clamp(joint_msg_.position[offset + 0], -1.0, 1.0);
    joint_msg_.position[offset + 1] = clamp(joint_msg_.position[offset + 1], -1.2, 3.0);
    joint_msg_.position[offset + 2] = clamp(joint_msg_.position[offset + 2], -2.6, -0.9);

    const double omega = 2.0 * kPi * gait_frequency_;
    joint_msg_.velocity[offset + 0] = side * hip_swing_ * speed_ratio * 0.35 * c * omega;
    joint_msg_.velocity[offset + 1] = -thigh_swing_ * speed_ratio * s * omega;
    joint_msg_.velocity[offset + 2] = calf_swing_ * speed_ratio * (s > 0.0 ? c : 0.0) * omega;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState joint_msg_;
  rclcpp::Time last_odom_time_;

  double horizontal_speed_ = 0.0;
  double last_odom_x_ = 0.0;
  double last_odom_y_ = 0.0;
  double rate_ = 60.0;
  double gait_frequency_ = 2.2;
  double min_walk_speed_ = 0.05;
  double max_walk_speed_ = 1.0;
  double hip_swing_ = 0.08;
  double thigh_swing_ = 0.32;
  double calf_swing_ = 0.42;
  bool always_trot_ = false;
  bool has_odom_ = false;
  bool has_prev_pose_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("go2_gait_publisher");
  Go2GaitPublisher publisher(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
