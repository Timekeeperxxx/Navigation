#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr const char * kRed = "\033[31m";
constexpr const char * kYellow = "\033[33m";
constexpr const char * kReset = "\033[0m";
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

class LivoxLidarTimestampMonitor : public rclcpp::Node
{
public:
  LivoxLidarTimestampMonitor()
  : Node("livox_lidar_timestamp_monitor")
  {
    topic_ = declare_parameter<std::string>("topic", "/livox/lidar");
    expected_frequency_ = declare_parameter<double>("expected_frequency", 10.0);
    tolerance_ratio_ = declare_parameter<double>("tolerance_ratio", 0.2);
    const auto qos_depth = declare_parameter<int64_t>("qos_depth", 100);

    if (!std::isfinite(expected_frequency_) || expected_frequency_ <= 0.0) {
      throw std::invalid_argument("expected_frequency 必须是大于 0 的有限数值");
    }
    if (!std::isfinite(tolerance_ratio_) || tolerance_ratio_ < 0.0 ||
      tolerance_ratio_ >= 1.0)
    {
      throw std::invalid_argument("tolerance_ratio 必须在 [0, 1) 范围内");
    }
    if (qos_depth < 2) {
      throw std::invalid_argument("qos_depth 必须大于或等于 2");
    }

    expected_interval_ = 1.0 / expected_frequency_;
    minimum_interval_ = expected_interval_ * (1.0 - tolerance_ratio_);
    maximum_interval_ = expected_interval_ * (1.0 + tolerance_ratio_);

    auto qos = rclcpp::SensorDataQoS()
      .keep_last(static_cast<size_t>(qos_depth));

    rclcpp::SubscriptionOptions options;
    options.event_callbacks.message_lost_callback =
      [this](rclcpp::QOSMessageLostInfo & info) {
        middleware_lost_count_ = static_cast<uint64_t>(info.total_count);
        std::ostringstream line;
        line << "[DDS丢帧] 本次=" << info.total_count_change
             << " 累计=" << info.total_count;
        print_colored(kRed, line.str());
      };

    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      topic_, qos,
      std::bind(&LivoxLidarTimestampMonitor::on_pointcloud, this, std::placeholders::_1),
      options);

    RCLCPP_INFO(
      get_logger(),
      "正在监测 %s: 期望频率=%.3f Hz, 正常源时间间隔=[%.6f, %.6f] s, "
      "QoS=BEST_EFFORT(depth=%lld)",
      topic_.c_str(), expected_frequency_, minimum_interval_, maximum_interval_,
      static_cast<long long>(qos_depth));
  }

  ~LivoxLidarTimestampMonitor() override
  {
    RCLCPP_INFO(
      get_logger(),
      "监测汇总: 收到=%llu, 雷达时间戳异常=%llu, 本机回调延迟=%llu, DDS报告丢帧=%llu",
      static_cast<unsigned long long>(received_count_),
      static_cast<unsigned long long>(source_anomaly_count_),
      static_cast<unsigned long long>(arrival_delay_count_),
      static_cast<unsigned long long>(middleware_lost_count_));
  }

private:
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;
  using SteadyClock = std::chrono::steady_clock;

  static std::string format_timestamp(int32_t seconds, uint32_t nanoseconds)
  {
    std::ostringstream text;
    text << seconds << '.' << std::setfill('0') << std::setw(9) << nanoseconds;
    return text.str();
  }

  static void print_colored(const char * color, const std::string & line)
  {
    std::cout << color << line << kReset << std::endl;
  }

  void on_pointcloud(const CustomMsg::ConstSharedPtr msg)
  {
    const auto arrival_time = SteadyClock::now();
    const int64_t timestamp_ns =
      static_cast<int64_t>(msg->header.stamp.sec) * kNanosecondsPerSecond +
      static_cast<int64_t>(msg->header.stamp.nanosec);
    const bool timestamp_valid = timestamp_ns > 0;
    const bool timebase_matches =
      timestamp_valid && msg->timebase == static_cast<uint64_t>(timestamp_ns);
    const std::string timestamp_text = format_timestamp(
      msg->header.stamp.sec, msg->header.stamp.nanosec);

    ++received_count_;
    if (!has_previous_frame_) {
      std::ostringstream line;
      line << "[首帧] frame=" << received_count_
           << " timestamp=" << timestamp_text << " s"
           << " timebase=" << msg->timebase << " ns"
           << " points=" << msg->point_num;
      if (timebase_matches) {
        std::cout << line.str() << std::endl;
      } else {
        ++source_anomaly_count_;
        line << " timestamp/timebase无效或不一致";
        print_colored(kRed, line.str());
      }
      previous_timestamp_ns_ = timestamp_ns;
      previous_arrival_time_ = arrival_time;
      has_previous_frame_ = true;
      return;
    }

    const double source_interval =
      static_cast<double>(timestamp_ns - previous_timestamp_ns_) /
      static_cast<double>(kNanosecondsPerSecond);
    const double arrival_interval =
      std::chrono::duration<double>(arrival_time - previous_arrival_time_).count();
    const bool source_interval_normal =
      source_interval >= minimum_interval_ && source_interval <= maximum_interval_;
    const bool arrival_interval_normal =
      arrival_interval >= minimum_interval_ && arrival_interval <= maximum_interval_;
    const bool source_normal =
      timestamp_valid && timebase_matches && source_interval_normal;
    const double source_frequency = source_interval > 0.0 ?
      1.0 / source_interval : std::numeric_limits<double>::infinity();

    std::ostringstream line;
    line << std::fixed << std::setprecision(6);
    line << (source_normal ? "[正常]" : "[雷达时间戳异常]")
         << " frame=" << received_count_
         << " timestamp=" << timestamp_text << " s"
         << " source_interval=" << source_interval << " s"
         << " arrival_interval=" << arrival_interval << " s"
         << " frequency=" << std::setprecision(3) << source_frequency << " Hz"
         << " points=" << msg->point_num;

    if (!source_normal) {
      ++source_anomaly_count_;
      if (source_interval > maximum_interval_) {
        const auto interval_multiple = static_cast<int64_t>(
          std::llround(source_interval / expected_interval_));
        const int64_t estimated_missing = std::max<int64_t>(0, interval_multiple - 1);
        line << " estimated_missing=" << estimated_missing;
      }
      if (!timebase_matches) {
        line << " timestamp/timebase不一致";
      }
      print_colored(kRed, line.str());
    } else if (!arrival_interval_normal) {
      ++arrival_delay_count_;
      line << " [仅本机回调延迟，源时间戳连续]";
      print_colored(kYellow, line.str());
    } else {
      std::cout << line.str() << std::endl;
    }

    previous_timestamp_ns_ = timestamp_ns;
    previous_arrival_time_ = arrival_time;
  }

  std::string topic_;
  double expected_frequency_{10.0};
  double tolerance_ratio_{0.2};
  double expected_interval_{0.1};
  double minimum_interval_{0.08};
  double maximum_interval_{0.12};
  bool has_previous_frame_{false};
  int64_t previous_timestamp_ns_{0};
  SteadyClock::time_point previous_arrival_time_{};
  uint64_t received_count_{0};
  uint64_t source_anomaly_count_{0};
  uint64_t arrival_delay_count_{0};
  uint64_t middleware_lost_count_{0};
  rclcpp::Subscription<CustomMsg>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<LivoxLidarTimestampMonitor>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & error) {
    std::cerr << kRed << "启动失败: " << error.what() << kReset << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
