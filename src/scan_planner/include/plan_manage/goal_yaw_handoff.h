#ifndef SCAN_PLANNER_GOAL_YAW_HANDOFF_H_
#define SCAN_PLANNER_GOAL_YAW_HANDOFF_H_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace scan_planner
{

struct GoalYawHandoffConfig
{
  // Goal publishers commonly send PoseStamped and Float64 metadata for the
  // same logical request.  Coalesce the two channels only for a short window;
  // a later identical value is a real new request and needs a new handoff.
  double coalesce_window{0.5};
  double yaw_tolerance{1e-4};
  double position_tolerance{1e-4};
  double trajectory_time_tolerance{1e-3};
};

enum class GoalYawInputResult
{
  REJECTED,
  NEW_GOAL,
  DUPLICATE,
  COALESCED_UPDATE,
};

struct GoalYawTrajectoryResult
{
  bool handoff_accepted{false};
  bool yaw_bound{false};
};

/**
 * @brief Bind terminal yaw metadata to the correct GoTo trajectory generation.
 *
 * A goal callback never binds its yaw directly to the currently held spline.
 * The protocol requires:
 *
 *   new goal -> newer emergency stationary handoff -> newer normal spline
 *
 * This prevents a completed trajectory from re-entering terminal alignment
 * when the next goal arrives.  The class owns no ROS state so callback-order
 * and retransmission behavior can be tested deterministically.
 */
class GoalYawHandoff
{
public:
  explicit GoalYawHandoff(
      const GoalYawHandoffConfig &config = GoalYawHandoffConfig())
  : config_(sanitize(config))
  {
  }

  void configure(const GoalYawHandoffConfig &config)
  {
    config_ = sanitize(config);
    reset();
  }

  void reset()
  {
    have_logical_goal_ = false;
    logical_goal_started_at_ = 0.0;
    saw_pose_input_ = false;
    saw_yaw_input_ = false;
    pose_x_ = pose_y_ = pose_z_ = pose_yaw_ = 0.0;
    yaw_input_ = 0.0;

    have_pending_yaw_ = false;
    pending_yaw_ = 0.0;
    pending_started_at_ = 0.0;
    handoff_seen_ = false;
    handoff_traj_id_ = 0;
    handoff_start_time_ = 0.0;

    have_active_yaw_ = false;
    active_yaw_ = 0.0;
    goal_generation_ = 0;
  }

  GoalYawInputResult receiveYaw(
      const double yaw,
      const double receipt_time)
  {
    if (!std::isfinite(yaw) || !std::isfinite(receipt_time))
      return GoalYawInputResult::REJECTED;

    const double normalized_yaw = normalizeAngle(yaw);
    if (withinCoalesceWindow(receipt_time))
    {
      if (
          saw_yaw_input_ &&
          angleDistance(normalized_yaw, yaw_input_) <=
              config_.yaw_tolerance)
      {
        return GoalYawInputResult::DUPLICATE;
      }

      if (saw_pose_input_ && !saw_yaw_input_)
      {
        // Float64 is commonly the explicit terminal-yaw companion to a
        // PoseStamped request.  It updates that logical goal without requiring
        // a second stationary handoff.
        saw_yaw_input_ = true;
        yaw_input_ = normalized_yaw;
        updateLogicalGoalYaw(normalized_yaw);
        return GoalYawInputResult::COALESCED_UPDATE;
      }
    }

    beginNewGoal(normalized_yaw, receipt_time);
    saw_yaw_input_ = true;
    yaw_input_ = normalized_yaw;
    return GoalYawInputResult::NEW_GOAL;
  }

  GoalYawInputResult receivePose(
      const double x,
      const double y,
      const double z,
      const double yaw,
      const double receipt_time)
  {
    if (
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(yaw) || !std::isfinite(receipt_time))
    {
      return GoalYawInputResult::REJECTED;
    }

    const double normalized_yaw = normalizeAngle(yaw);
    if (withinCoalesceWindow(receipt_time))
    {
      if (
          saw_pose_input_ &&
          std::abs(x - pose_x_) <= config_.position_tolerance &&
          std::abs(y - pose_y_) <= config_.position_tolerance &&
          std::abs(z - pose_z_) <= config_.position_tolerance &&
          angleDistance(normalized_yaw, pose_yaw_) <=
              config_.yaw_tolerance)
      {
        return GoalYawInputResult::DUPLICATE;
      }

      if (saw_yaw_input_ && !saw_pose_input_)
      {
        // Support the opposite publication order as one logical target.  A
        // distinct later PoseStamped is still a new goal because the stored
        // pose signature will no longer match.
        saw_pose_input_ = true;
        pose_x_ = x;
        pose_y_ = y;
        pose_z_ = z;
        pose_yaw_ = normalized_yaw;
        updateLogicalGoalYaw(normalized_yaw);
        return GoalYawInputResult::COALESCED_UPDATE;
      }
    }

    beginNewGoal(normalized_yaw, receipt_time);
    saw_pose_input_ = true;
    pose_x_ = x;
    pose_y_ = y;
    pose_z_ = z;
    pose_yaw_ = normalized_yaw;
    return GoalYawInputResult::NEW_GOAL;
  }

  GoalYawTrajectoryResult receiveTrajectory(
      const bool emergency_stop,
      const std::int64_t trajectory_id,
      const double trajectory_start_time)
  {
    GoalYawTrajectoryResult result;
    if (
        !have_pending_yaw_ ||
        !std::isfinite(trajectory_start_time))
    {
      return result;
    }

    if (emergency_stop)
    {
      // Delivery order across ROS topics is not a generation guarantee.  Do
      // not let an older queued emergency spline satisfy a newer goal.
      if (
          trajectory_start_time +
              config_.trajectory_time_tolerance <
          pending_started_at_)
      {
        return result;
      }

      if (
          !handoff_seen_ ||
          trajectory_id > handoff_traj_id_ ||
          trajectory_start_time >
              handoff_start_time_ +
                  config_.trajectory_time_tolerance)
      {
        handoff_seen_ = true;
        handoff_traj_id_ = trajectory_id;
        handoff_start_time_ = trajectory_start_time;
        result.handoff_accepted = true;
      }
      return result;
    }

    if (
        !handoff_seen_ ||
        trajectory_id <= handoff_traj_id_ ||
        trajectory_start_time +
                config_.trajectory_time_tolerance <
            handoff_start_time_)
    {
      return result;
    }

    active_yaw_ = pending_yaw_;
    have_active_yaw_ = true;
    have_pending_yaw_ = false;
    handoff_seen_ = false;
    result.yaw_bound = true;
    return result;
  }

  bool hasPendingYaw() const
  {
    return have_pending_yaw_;
  }

  bool awaitingHandoff() const
  {
    return have_pending_yaw_ && !handoff_seen_;
  }

  bool hasSeenHandoff() const
  {
    return have_pending_yaw_ && handoff_seen_;
  }

  double pendingYaw() const
  {
    return pending_yaw_;
  }

  bool hasActiveYaw() const
  {
    return have_active_yaw_;
  }

  double activeYaw() const
  {
    return active_yaw_;
  }

  std::uint64_t goalGeneration() const
  {
    return goal_generation_;
  }

private:
  static GoalYawHandoffConfig sanitize(GoalYawHandoffConfig config)
  {
    config.coalesce_window = std::max(0.0, config.coalesce_window);
    config.yaw_tolerance = std::max(0.0, config.yaw_tolerance);
    config.position_tolerance = std::max(0.0, config.position_tolerance);
    config.trajectory_time_tolerance =
        std::max(0.0, config.trajectory_time_tolerance);
    return config;
  }

  static double normalizeAngle(const double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double angleDistance(const double lhs, const double rhs)
  {
    return std::abs(normalizeAngle(lhs - rhs));
  }

  bool withinCoalesceWindow(const double receipt_time) const
  {
    if (!have_logical_goal_ || receipt_time < logical_goal_started_at_)
      return false;
    return receipt_time - logical_goal_started_at_ <=
           config_.coalesce_window;
  }

  void beginNewGoal(const double yaw, const double receipt_time)
  {
    have_logical_goal_ = true;
    logical_goal_started_at_ = receipt_time;
    saw_pose_input_ = false;
    saw_yaw_input_ = false;

    have_pending_yaw_ = true;
    pending_yaw_ = yaw;
    pending_started_at_ = receipt_time;
    handoff_seen_ = false;
    handoff_traj_id_ = 0;
    handoff_start_time_ = 0.0;

    // A new goal must not inherit terminal-yaw authority from the previous
    // trajectory, even if the controller currently holds an emergency spline.
    have_active_yaw_ = false;
    ++goal_generation_;
  }

  void updateLogicalGoalYaw(const double yaw)
  {
    if (have_pending_yaw_)
    {
      pending_yaw_ = yaw;
      return;
    }
    if (have_active_yaw_)
      active_yaw_ = yaw;
  }

  GoalYawHandoffConfig config_;

  bool have_logical_goal_{false};
  double logical_goal_started_at_{0.0};
  bool saw_pose_input_{false};
  bool saw_yaw_input_{false};
  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_z_{0.0};
  double pose_yaw_{0.0};
  double yaw_input_{0.0};

  bool have_pending_yaw_{false};
  double pending_yaw_{0.0};
  double pending_started_at_{0.0};
  bool handoff_seen_{false};
  std::int64_t handoff_traj_id_{0};
  double handoff_start_time_{0.0};

  bool have_active_yaw_{false};
  double active_yaw_{0.0};
  std::uint64_t goal_generation_{0};
};

}  // namespace scan_planner

#endif  // SCAN_PLANNER_GOAL_YAW_HANDOFF_H_
