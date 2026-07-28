#ifndef SCAN_PLANNER_B2_REVERSE_RECOVERY_H_
#define SCAN_PLANNER_B2_REVERSE_RECOVERY_H_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace scan_planner
{

enum class B2ReverseRecoveryState : std::uint8_t
{
  IDLE = 0,
  WAITING_FOR_SAFETY = 1,
  REVERSING = 2,
  HOLDING = 3,
};

enum class B2ReverseRecoveryStatus : std::uint8_t
{
  IDLE = 0,
  ACTIVE = 1,
  DISTANCE_COMPLETE = 2,
  TIME_COMPLETE = 3,
  SAFETY_BLOCKED = 4,
  ODOMETRY_INVALID = 5,
  YAW_DRIFT = 6,
  CANCELLED = 7,
};

struct B2ReverseRecoveryConfig
{
  double maximum_distance{0.50};
  double maximum_duration{2.0};
  double minimum_preflight_duration{0.40};
  double safety_approval_timeout{1.5};
  double odometry_timeout{0.50};
  double maximum_yaw_drift{0.15};
};

struct B2ReverseRecoveryUpdate
{
  B2ReverseRecoveryState state{B2ReverseRecoveryState::IDLE};
  B2ReverseRecoveryStatus status{B2ReverseRecoveryStatus::IDLE};
  bool command_reverse{false};
  bool terminal{false};
  double distance{0.0};
  double elapsed{0.0};
};

inline bool shouldTriggerB2ReverseRecovery(
    int consecutive_replan_failures,
    int minimum_replan_failures,
    bool current_footprint_dynamically_occupied,
    bool forward_probe_dynamically_occupied,
    bool safety_execution_frozen)
{
  if (consecutive_replan_failures < std::max(1, minimum_replan_failures))
    return false;

  // A stale/reduced execution path can make the downstream safety Bool false
  // even while the robot's current footprint is embedded in an obstacle.
  if (current_footprint_dynamically_occupied)
    return true;

  // A nearby obstacle beside a valid path is not a recovery request. Only use
  // the forward probe when the execution safety layer also stopped the path.
  return safety_execution_frozen &&
         forward_probe_dynamically_occupied;
}

inline bool shouldContinueLatchedB2ObstacleRecovery(
    bool obstacle_recovery_latched,
    int consecutive_replan_failures,
    int minimum_replan_failures,
    bool current_footprint_dynamically_occupied,
    bool forward_probe_dynamically_occupied,
    bool safety_execution_frozen)
{
  return obstacle_recovery_latched &&
      consecutive_replan_failures >=
          std::max(1, minimum_replan_failures) &&
      (current_footprint_dynamically_occupied ||
       forward_probe_dynamically_occupied ||
       safety_execution_frozen);
}

class B2ReverseRecoveryPolicy
{
public:
  void configure(const B2ReverseRecoveryConfig & config)
  {
    config_.maximum_distance =
        std::max(0.05, config.maximum_distance);
    config_.maximum_duration =
        std::max(0.10, config.maximum_duration);
    config_.minimum_preflight_duration =
        std::max(0.0, config.minimum_preflight_duration);
    config_.safety_approval_timeout =
        std::max(
            config_.minimum_preflight_duration + 0.10,
            config.safety_approval_timeout);
    config_.odometry_timeout =
        std::max(0.05, config.odometry_timeout);
    config_.maximum_yaw_drift =
        std::max(0.01, config.maximum_yaw_drift);
  }

  bool begin(
      double now, double x, double y, double z, double yaw)
  {
    if (!allFinite(now, x, y, z, yaw))
      return false;
    request_started_at_ = now;
    motion_started_at_ = -1.0;
    start_x_ = x;
    start_y_ = y;
    start_z_ = z;
    start_yaw_ = normalize(yaw);
    state_ = B2ReverseRecoveryState::WAITING_FOR_SAFETY;
    return true;
  }

  B2ReverseRecoveryStatus cancel()
  {
    const bool was_active =
        state_ == B2ReverseRecoveryState::WAITING_FOR_SAFETY ||
        state_ == B2ReverseRecoveryState::REVERSING;
    reset();
    return was_active
        ? B2ReverseRecoveryStatus::CANCELLED
        : B2ReverseRecoveryStatus::IDLE;
  }

  void reset()
  {
    state_ = B2ReverseRecoveryState::IDLE;
    request_started_at_ = -1.0;
    motion_started_at_ = -1.0;
  }

  B2ReverseRecoveryUpdate update(
      double now,
      double x,
      double y,
      double yaw,
      double odometry_age,
      bool safety_frozen)
  {
    B2ReverseRecoveryUpdate result;
    result.state = state_;
    if (
        state_ == B2ReverseRecoveryState::IDLE ||
        state_ == B2ReverseRecoveryState::HOLDING)
    {
      return result;
    }

    if (
        !allFinite(now, x, y, yaw, odometry_age) ||
        odometry_age < 0.0 ||
        odometry_age > config_.odometry_timeout)
    {
      return finish(B2ReverseRecoveryStatus::ODOMETRY_INVALID, now, x, y);
    }

    const double yaw_error =
        std::abs(normalize(yaw - start_yaw_));
    if (yaw_error > config_.maximum_yaw_drift)
      return finish(B2ReverseRecoveryStatus::YAW_DRIFT, now, x, y);

    if (state_ == B2ReverseRecoveryState::WAITING_FOR_SAFETY)
    {
      const double preflight_elapsed =
          std::max(0.0, now - request_started_at_);
      if (safety_frozen)
      {
        if (preflight_elapsed >= config_.safety_approval_timeout)
        {
          return finish(
              B2ReverseRecoveryStatus::SAFETY_BLOCKED, now, x, y);
        }
        result.status = B2ReverseRecoveryStatus::ACTIVE;
        return result;
      }
      if (preflight_elapsed < config_.minimum_preflight_duration)
      {
        result.status = B2ReverseRecoveryStatus::ACTIVE;
        return result;
      }
      motion_started_at_ = now;
      state_ = B2ReverseRecoveryState::REVERSING;
    }

    if (safety_frozen)
      return finish(B2ReverseRecoveryStatus::SAFETY_BLOCKED, now, x, y);

    result.distance = std::hypot(x - start_x_, y - start_y_);
    result.elapsed = std::max(0.0, now - motion_started_at_);
    if (result.distance >= config_.maximum_distance)
      return finish(
          B2ReverseRecoveryStatus::DISTANCE_COMPLETE, now, x, y);
    if (result.elapsed >= config_.maximum_duration)
      return finish(
          B2ReverseRecoveryStatus::TIME_COMPLETE, now, x, y);

    result.state = state_;
    result.status = B2ReverseRecoveryStatus::ACTIVE;
    result.command_reverse = true;
    return result;
  }

  B2ReverseRecoveryState state() const
  {
    return state_;
  }

  bool active() const
  {
    return state_ == B2ReverseRecoveryState::WAITING_FOR_SAFETY ||
           state_ == B2ReverseRecoveryState::REVERSING;
  }

  bool holding() const
  {
    return state_ == B2ReverseRecoveryState::HOLDING;
  }

  double startX() const { return start_x_; }
  double startY() const { return start_y_; }
  double startZ() const { return start_z_; }
  double startYaw() const { return start_yaw_; }

private:
  template<typename... Values>
  static bool allFinite(Values... values)
  {
    return (... && std::isfinite(static_cast<double>(values)));
  }

  static double normalize(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  B2ReverseRecoveryUpdate finish(
      B2ReverseRecoveryStatus status,
      double now,
      double x,
      double y)
  {
    B2ReverseRecoveryUpdate result;
    result.status = status;
    result.terminal = true;
    result.distance =
        allFinite(x, y) ? std::hypot(x - start_x_, y - start_y_) : 0.0;
    result.elapsed =
        motion_started_at_ >= 0.0 && std::isfinite(now)
            ? std::max(0.0, now - motion_started_at_)
            : 0.0;
    state_ = B2ReverseRecoveryState::HOLDING;
    result.state = state_;
    return result;
  }

  B2ReverseRecoveryConfig config_;
  B2ReverseRecoveryState state_{B2ReverseRecoveryState::IDLE};
  double request_started_at_{-1.0};
  double motion_started_at_{-1.0};
  double start_x_{0.0};
  double start_y_{0.0};
  double start_z_{0.0};
  double start_yaw_{0.0};
};

}  // namespace scan_planner

#endif  // SCAN_PLANNER_B2_REVERSE_RECOVERY_H_
