#ifndef SCAN_PLANNER_B2_YAW_CONTROL_H_
#define SCAN_PLANNER_B2_YAW_CONTROL_H_

#include <algorithm>
#include <cmath>

namespace scan_planner
{

inline double normalizeB2ControlAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double unwrapB2ControlAngleNear(double angle, double reference)
{
  return reference + normalizeB2ControlAngle(angle - reference);
}

// Estimate actual body yaw rate from pose feedback.  SCAN's controller only
// receives a pose odometry stream, while Unitree's commanded yaw rate can lag
// the body motion noticeably.  Keeping this estimate in the controller lets
// it brake before the body has already crossed the desired heading.
class B2YawRateEstimator
{
public:
  void configure(double filter_time_constant, double maximum_rate)
  {
    filter_time_constant_ = std::max(0.0, filter_time_constant);
    maximum_rate_ = std::max(0.1, maximum_rate);
  }

  void reset()
  {
    initialized_ = false;
    filtered_rate_ = 0.0;
    last_yaw_ = 0.0;
    last_time_ = 0.0;
  }

  double update(double yaw, double time)
  {
    if (!std::isfinite(yaw) || !std::isfinite(time))
      return filtered_rate_;

    if (!initialized_)
    {
      initialized_ = true;
      last_yaw_ = yaw;
      last_time_ = time;
      filtered_rate_ = 0.0;
      return filtered_rate_;
    }

    const double dt = time - last_time_;
    const double yaw_delta = normalizeB2ControlAngle(yaw - last_yaw_);
    last_yaw_ = yaw;
    last_time_ = time;
    if (dt <= 1e-4 || dt > 0.5)
    {
      filtered_rate_ = 0.0;
      return filtered_rate_;
    }

    const double raw_rate = std::max(
        -maximum_rate_, std::min(maximum_rate_, yaw_delta / dt));
    const double alpha = filter_time_constant_ <= 1e-6
                             ? 1.0
                             : dt / (filter_time_constant_ + dt);
    filtered_rate_ += alpha * (raw_rate - filtered_rate_);
    return filtered_rate_;
  }

  double rate() const
  {
    return filtered_rate_;
  }

private:
  bool initialized_{false};
  double filter_time_constant_{0.15};
  double maximum_rate_{2.0};
  double filtered_rate_{0.0};
  double last_yaw_{0.0};
  double last_time_{0.0};
};

// Low-pass a desired heading without introducing a discontinuity at +/-pi or
// when SCAN replaces one short local spline with the next.
class B2DesiredYawFilter
{
public:
  void configure(double filter_time_constant)
  {
    filter_time_constant_ = std::max(0.0, filter_time_constant);
  }

  void reset()
  {
    initialized_ = false;
    filtered_yaw_ = 0.0;
    last_time_ = 0.0;
  }

  double update(double desired_yaw, double time)
  {
    if (!std::isfinite(desired_yaw) || !std::isfinite(time))
      return initialized_ ? filtered_yaw_ : 0.0;

    if (!initialized_)
    {
      initialized_ = true;
      filtered_yaw_ = desired_yaw;
      last_time_ = time;
      return filtered_yaw_;
    }

    const double desired_near =
        unwrapB2ControlAngleNear(desired_yaw, filtered_yaw_);
    const double dt = time - last_time_;
    last_time_ = time;
    if (dt <= 0.0 || dt > 0.5)
    {
      filtered_yaw_ = desired_near;
      return filtered_yaw_;
    }

    const double alpha = filter_time_constant_ <= 1e-6
                             ? 1.0
                             : dt / (filter_time_constant_ + dt);
    filtered_yaw_ += alpha * (desired_near - filtered_yaw_);
    return filtered_yaw_;
  }

private:
  bool initialized_{false};
  double filter_time_constant_{0.20};
  double filtered_yaw_{0.0};
  double last_time_{0.0};
};

struct B2YawControlResult
{
  double command{0.0};
  double predicted_error{0.0};
  bool alignment_active{false};
  bool reversal_braking{false};
};

struct B2FinalPositionCommand
{
  double body_x{0.0};
  double body_y{0.0};
  double distance{0.0};
  bool aligned{true};
};

// Hold the final XY point after SCAN has handed the terminal pose to the
// controller.  A B2 can drift by more than ten centimetres while turning in
// place, so treating XY as a one-way latch leaves the robot visibly short of
// the goal.  This small body-frame correction remains bounded and works while
// the final-yaw controller keeps the requested heading.
inline B2FinalPositionCommand computeB2FinalPositionCommand(
    double error_world_x,
    double error_world_y,
    double body_yaw,
    double tolerance,
    double kp,
    double max_x,
    double max_y,
    bool allow_reverse)
{
  B2FinalPositionCommand result;
  if (
      !std::isfinite(error_world_x) ||
      !std::isfinite(error_world_y) ||
      !std::isfinite(body_yaw))
  {
    result.aligned = false;
    return result;
  }

  result.distance = std::hypot(error_world_x, error_world_y);
  result.aligned = result.distance <= std::max(0.0, tolerance);
  if (result.aligned)
    return result;

  const double c = std::cos(body_yaw);
  const double s = std::sin(body_yaw);
  const double gain = std::max(0.0, kp);
  const double desired_x = gain * (c * error_world_x + s * error_world_y);
  const double desired_y = gain * (-s * error_world_x + c * error_world_y);
  const double x_limit = std::max(0.0, max_x);
  const double y_limit = std::max(0.0, max_y);
  result.body_x = std::max(
      allow_reverse ? -x_limit : 0.0,
      std::min(x_limit, desired_x));
  result.body_y = std::max(-y_limit, std::min(y_limit, desired_y));
  return result;
}

inline bool isB2FinalPoseReached(
    double position_distance,
    double yaw_error,
    double position_tolerance,
    double yaw_tolerance,
    bool require_yaw)
{
  if (!std::isfinite(position_distance) || !std::isfinite(yaw_error))
    return false;
  const bool position_reached =
      position_distance <= std::max(0.0, position_tolerance);
  const bool yaw_reached =
      !require_yaw || std::abs(yaw_error) <= std::max(0.0, yaw_tolerance);
  return position_reached && yaw_reached;
}

// Stateful yaw policy for the ordinary path-following phase.  It combines a
// Schmitt heading gate with rate-based prediction and a neutral braking phase
// before reversing.  The neutral phase is important for a velocity-controlled
// B2: immediately sending the opposite saturated command while the body still
// rotates in the previous direction creates the observed left/right limit
// cycle.
class B2PathYawControl
{
public:
  void configure(
      double stop_error,
      double resume_error,
      double prediction_horizon,
      double settle_rate,
      double reversal_neutral_time,
      double command_epsilon = 1e-3)
  {
    stop_error_ = std::max(0.0, stop_error);
    resume_error_ = std::max(
        0.0, std::min(resume_error, stop_error_));
    prediction_horizon_ = std::max(0.0, prediction_horizon);
    settle_rate_ = std::max(0.0, settle_rate);
    reversal_neutral_time_ = std::max(0.0, reversal_neutral_time);
    command_epsilon_ = std::max(0.0, command_epsilon);
    reset();
  }

  void reset()
  {
    alignment_active_ = false;
    reversal_braking_ = false;
    reversal_start_time_ = 0.0;
    last_nonzero_command_ = 0.0;
  }

  B2YawControlResult update(
      double heading_error,
      double measured_yaw_rate,
      double time,
      double kp,
      double max_command)
  {
    B2YawControlResult result;
    if (
        !std::isfinite(heading_error) ||
        !std::isfinite(measured_yaw_rate) ||
        !std::isfinite(time))
    {
      result.alignment_active = alignment_active_;
      result.reversal_braking = true;
      return result;
    }

    const double abs_error = std::abs(heading_error);
    if (alignment_active_)
    {
      if (abs_error <= resume_error_ &&
          std::abs(measured_yaw_rate) <= settle_rate_)
      {
        alignment_active_ = false;
      }
    }
    else if (abs_error > stop_error_)
    {
      alignment_active_ = true;
    }

    result.predicted_error = normalizeB2ControlAngle(
        heading_error - measured_yaw_rate * prediction_horizon_);
    const double limit = std::max(0.0, max_command);
    const double desired_command = std::max(
        -limit,
        std::min(limit, std::max(0.0, kp) * result.predicted_error));

    const bool desired_nonzero =
        std::abs(desired_command) > command_epsilon_;
    const bool reverses_last_command =
        desired_nonzero &&
        std::abs(last_nonzero_command_) > command_epsilon_ &&
        desired_command * last_nonzero_command_ < 0.0;

    bool reversal_authorized = false;
    if (reversal_braking_)
    {
      // If the target moves back to the old side, the reversal vanished and
      // there is no reason to remain stopped.
      if (desired_nonzero &&
          desired_command * last_nonzero_command_ > 0.0)
      {
        reversal_braking_ = false;
      }
      else
      {
        const double neutral_elapsed = time - reversal_start_time_;
        if (
            neutral_elapsed < reversal_neutral_time_ ||
            std::abs(measured_yaw_rate) > settle_rate_)
        {
          result.alignment_active = alignment_active_;
          result.reversal_braking = true;
          return result;
        }
        reversal_braking_ = false;
        reversal_authorized = true;
        // The old direction has physically settled.  Forget its sign so a
        // near-zero predicted command on this cycle cannot retrigger another
        // full neutral interval on the next cycle.
        last_nonzero_command_ = 0.0;
      }
    }

    if (reverses_last_command && !reversal_authorized)
    {
      reversal_braking_ = true;
      reversal_start_time_ = time;
      result.alignment_active = alignment_active_;
      result.reversal_braking = true;
      return result;
    }

    // Also brake if feedback says the body is still rotating opposite to the
    // newly desired command, even when the controller was just reset and has
    // no previous command sign recorded.
    if (
        desired_nonzero &&
        std::abs(measured_yaw_rate) > settle_rate_ &&
        desired_command * measured_yaw_rate < 0.0)
    {
      result.alignment_active = alignment_active_;
      result.reversal_braking = true;
      return result;
    }

    result.command = desired_command;
    result.alignment_active = alignment_active_;
    result.reversal_braking = false;
    if (desired_nonzero)
      last_nonzero_command_ = desired_command;
    return result;
  }

private:
  double stop_error_{0.5};
  double resume_error_{0.35};
  double prediction_horizon_{1.0};
  double settle_rate_{0.06};
  double reversal_neutral_time_{0.35};
  double command_epsilon_{1e-3};
  bool alignment_active_{false};
  bool reversal_braking_{false};
  double reversal_start_time_{0.0};
  double last_nonzero_command_{0.0};
};

}  // namespace scan_planner

#endif  // SCAN_PLANNER_B2_YAW_CONTROL_H_
