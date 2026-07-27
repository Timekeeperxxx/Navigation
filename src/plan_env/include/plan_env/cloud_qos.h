#ifndef PLAN_ENV__CLOUD_QOS_H_
#define PLAN_ENV__CLOUD_QOS_H_

#include <rclcpp/qos.hpp>

namespace plan_env
{

inline rclcpp::QoS cloudSensorQos()
{
  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  return qos;
}

}  // namespace plan_env

#endif  // PLAN_ENV__CLOUD_QOS_H_
