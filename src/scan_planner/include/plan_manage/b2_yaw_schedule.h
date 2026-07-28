#ifndef SCAN_PLANNER_B2_YAW_SCHEDULE_H_
#define SCAN_PLANNER_B2_YAW_SCHEDULE_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Eigen>

namespace scan_planner
{

inline double normalizeB2Yaw(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double unwrapB2YawNear(double angle, double reference)
{
  return reference + normalizeB2Yaw(angle - reference);
}

inline std::size_t b2YawSampleIntervalCount(
    double duration, double maximum_sample_dt)
{
  if (!std::isfinite(duration) || duration <= 0.0)
    return 0;
  const double safe_dt =
      std::isfinite(maximum_sample_dt) && maximum_sample_dt > 1e-6
          ? maximum_sample_dt
          : duration;
  return std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(duration / safe_dt)));
}

// Describe an in-place B2 rotation with a shortest-angle, uniformly sampled
// yaw sweep.  The safety monitor consumes these poses before the controller
// starts rotating so the asymmetric double-circle footprint is checked at
// every intermediate attitude, not only at the current/final attitudes.
inline std::vector<double> makeB2InPlaceYawSchedule(
    double initial_yaw, double goal_yaw, double maximum_yaw_step)
{
  if (
      !std::isfinite(initial_yaw) ||
      !std::isfinite(goal_yaw) ||
      !std::isfinite(maximum_yaw_step) ||
      maximum_yaw_step <= 1e-6)
  {
    return {};
  }

  initial_yaw = normalizeB2Yaw(initial_yaw);
  const double yaw_delta = normalizeB2Yaw(goal_yaw - initial_yaw);
  const std::size_t interval_count = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(
          std::ceil(std::abs(yaw_delta) / maximum_yaw_step)));

  std::vector<double> yaws;
  yaws.reserve(interval_count + 1);
  for (std::size_t index = 0; index <= interval_count; ++index)
  {
    yaws.push_back(
        initial_yaw +
        yaw_delta * static_cast<double>(index) /
            static_cast<double>(interval_count));
  }
  return yaws;
}

// Generate the canonical B2 body-yaw samples for a uniformly sampled
// position trajectory. The first sample is always the live body yaw; interior
// samples bisect adjacent path tangents to avoid discontinuous corner snaps.
// Values remain unwrapped so interpolation is continuous across +/-pi.
inline std::vector<double> makeB2YawSchedule(
    const std::vector<Eigen::Vector3d> &path, double initial_yaw)
{
  if (path.empty())
    return {};

  initial_yaw = normalizeB2Yaw(initial_yaw);
  if (path.size() == 1)
    return {initial_yaw};

  std::vector<double> segment_yaws;
  segment_yaws.reserve(path.size() - 1);
  double previous_segment_yaw = initial_yaw;
  for (std::size_t index = 0; index + 1 < path.size(); ++index)
  {
    const Eigen::Vector2d delta =
        (path[index + 1] - path[index]).head<2>();
    if (delta.squaredNorm() > 1e-8)
    {
      previous_segment_yaw = unwrapB2YawNear(
          std::atan2(delta.y(), delta.x()), previous_segment_yaw);
    }
    segment_yaws.push_back(previous_segment_yaw);
  }

  std::vector<double> yaws(path.size(), initial_yaw);
  yaws.front() = initial_yaw;
  for (std::size_t index = 1; index + 1 < path.size(); ++index)
  {
    const double incoming = segment_yaws[index - 1];
    const double outgoing =
        unwrapB2YawNear(segment_yaws[index], incoming);
    const double corner_bisector =
        incoming + 0.5 * (outgoing - incoming);
    yaws[index] = unwrapB2YawNear(corner_bisector, yaws[index - 1]);
  }
  yaws.back() = unwrapB2YawNear(segment_yaws.back(), yaws[path.size() - 2]);
  return yaws;
}

inline double interpolateB2YawSchedule(
    const std::vector<double> &yaws, double duration, double time)
{
  if (yaws.empty())
    return 0.0;
  if (yaws.size() == 1 || !std::isfinite(duration) || duration <= 1e-9)
    return yaws.front();

  const double clamped_time = std::max(0.0, std::min(time, duration));
  const double sample_position =
      clamped_time / duration * static_cast<double>(yaws.size() - 1);
  const std::size_t lower = std::min<std::size_t>(
      static_cast<std::size_t>(std::floor(sample_position)),
      yaws.size() - 2);
  const double ratio = sample_position - static_cast<double>(lower);
  return yaws[lower] + ratio * (yaws[lower + 1] - yaws[lower]);
}

// Angular-rate feed-forward for the same piecewise-linear schedule used by
// interpolateB2YawSchedule().  A B2 must rotate continuously while following
// a curve; proportional yaw feedback alone leaves a steady heading lag and
// repeatedly trips the turn-in-place safety latch.
inline double b2YawScheduleRate(
    const std::vector<double> &yaws, double duration, double time)
{
  if (
      yaws.size() < 2 ||
      !std::isfinite(duration) ||
      duration <= 1e-9)
  {
    return 0.0;
  }

  const double interval_duration =
      duration / static_cast<double>(yaws.size() - 1);
  const double clamped_time = std::max(0.0, std::min(time, duration));
  const double sample_position = clamped_time / interval_duration;
  const std::size_t interval = std::min<std::size_t>(
      static_cast<std::size_t>(std::floor(sample_position)),
      yaws.size() - 2);
  return (yaws[interval + 1] - yaws[interval]) / interval_duration;
}

}  // namespace scan_planner

#endif  // SCAN_PLANNER_B2_YAW_SCHEDULE_H_
