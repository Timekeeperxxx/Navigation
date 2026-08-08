#ifndef GLOBAL_PLANNER__B2_START_MANEUVER_H_
#define GLOBAL_PLANNER__B2_START_MANEUVER_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

namespace global_planner
{

struct B2StartManeuverPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct B2StartManeuverConfig
{
  double maximum_forward_distance{2.0};
  double forward_step{0.05};
  double path_sample_step{0.05};
  double yaw_sample_step{0.08726646259971647};
  double maximum_join_distance{12.0};
};

enum class B2StartManeuverStatus
{
  DIRECT_SAFE,
  REPAIRED,
  BLOCKED,
  INVALID,
};

struct B2StartManeuverResult
{
  B2StartManeuverStatus status{B2StartManeuverStatus::INVALID};
  std::vector<B2StartManeuverPoint> path;
  double forward_distance{0.0};
  std::size_t join_index{0};
};

using B2PoseSupportQuery =
    std::function<bool(const B2StartManeuverPoint &, double)>;
using B2GroundHeightQuery =
    std::function<bool(double, double, double, double &)>;

inline double normalizeB2StartYaw(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double b2StartHorizontalDistance(
    const B2StartManeuverPoint & first,
    const B2StartManeuverPoint & second)
{
  return std::hypot(second.x - first.x, second.y - first.y);
}

inline bool isB2StartTurnSupported(
    const B2StartManeuverPoint & pivot,
    double from_yaw,
    double to_yaw,
    double yaw_step,
    const B2PoseSupportQuery & pose_supported)
{
  if (!pose_supported || !std::isfinite(from_yaw) ||
      !std::isfinite(to_yaw))
  {
    return false;
  }

  const double delta = normalizeB2StartYaw(to_yaw - from_yaw);
  const double safe_step =
      std::max(0.017453292519943295, std::abs(yaw_step));
  const std::size_t samples = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(
          std::ceil(std::abs(delta) / safe_step)));
  for (std::size_t sample = 0; sample <= samples; ++sample)
  {
    const double ratio =
        static_cast<double>(sample) / static_cast<double>(samples);
    if (!pose_supported(pivot, from_yaw + ratio * delta))
      return false;
  }
  return true;
}

inline bool appendSupportedB2StartSegment(
    const B2StartManeuverPoint & from,
    const B2StartManeuverPoint & to,
    double sample_step,
    const B2PoseSupportQuery & pose_supported,
    std::vector<B2StartManeuverPoint> * output = nullptr)
{
  const double distance = b2StartHorizontalDistance(from, to);
  if (!std::isfinite(distance) || distance <= 1e-6 || !pose_supported)
    return false;

  const double yaw = std::atan2(to.y - from.y, to.x - from.x);
  const double safe_step = std::max(0.02, std::abs(sample_step));
  const std::size_t samples = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(std::ceil(distance / safe_step)));
  for (std::size_t sample = 0; sample <= samples; ++sample)
  {
    if (sample == 0 && output)
      continue;
    const double ratio =
        static_cast<double>(sample) / static_cast<double>(samples);
    B2StartManeuverPoint point;
    point.x = from.x + ratio * (to.x - from.x);
    point.y = from.y + ratio * (to.y - from.y);
    point.z = from.z + ratio * (to.z - from.z);
    if (!pose_supported(point, yaw))
      return false;
    if (output)
      output->push_back(point);
  }
  return true;
}

inline std::size_t nextDistinctB2StartPoint(
    const std::vector<B2StartManeuverPoint> & path,
    std::size_t from)
{
  if (from >= path.size())
    return path.size();
  for (std::size_t index = from + 1; index < path.size(); ++index)
  {
    if (b2StartHorizontalDistance(path[from], path[index]) > 1e-4)
      return index;
  }
  return path.size();
}

inline bool isB2StartPathTranslationSupported(
    const std::vector<B2StartManeuverPoint> & path,
    double sample_step,
    const B2PoseSupportQuery & pose_supported,
    std::size_t begin_index = 0)
{
  if (path.size() < 2 || begin_index >= path.size() || !pose_supported)
    return false;
  for (std::size_t index = begin_index;
       index + 1 < path.size(); ++index)
  {
    if (b2StartHorizontalDistance(path[index], path[index + 1]) <= 1e-6)
    {
      if (std::abs(path[index + 1].z - path[index].z) > 1e-4)
        return false;
      continue;
    }
    if (!appendSupportedB2StartSegment(
          path[index], path[index + 1], sample_step, pose_supported))
    {
      return false;
    }
  }
  return true;
}

/**
 * @brief Make a position-only global path executable from the live B2 yaw.
 *
 * If the shortest in-place sweep onto the first path segment is supported,
 * the path is returned unchanged. Otherwise the helper searches only in the
 * robot's current forward direction for a ground-supported escape point, then
 * joins a later, already ground-safe part of the path. No reverse segment is
 * ever introduced.
 */
inline B2StartManeuverResult makeB2StartManeuver(
    const std::vector<B2StartManeuverPoint> & input_path,
    double start_yaw,
    const B2StartManeuverConfig & raw_config,
    const B2PoseSupportQuery & pose_supported,
    const B2GroundHeightQuery & ground_height)
{
  B2StartManeuverResult result;
  if (input_path.size() < 2 || !pose_supported ||
      !ground_height || !std::isfinite(start_yaw))
  {
    return result;
  }

  B2StartManeuverConfig config = raw_config;
  config.maximum_forward_distance =
      std::max(0.1, config.maximum_forward_distance);
  config.forward_step = std::max(0.05, config.forward_step);
  config.path_sample_step = std::max(0.02, config.path_sample_step);
  config.yaw_sample_step =
      std::max(0.017453292519943295, config.yaw_sample_step);
  config.maximum_join_distance =
      std::max(config.forward_step, config.maximum_join_distance);
  start_yaw = normalizeB2StartYaw(start_yaw);
  if (!pose_supported(input_path.front(), start_yaw))
  {
    result.status = B2StartManeuverStatus::BLOCKED;
    return result;
  }

  const std::size_t first_path_index =
      nextDistinctB2StartPoint(input_path, 0);
  if (first_path_index >= input_path.size())
    return result;
  const double first_path_yaw = std::atan2(
      input_path[first_path_index].y - input_path.front().y,
      input_path[first_path_index].x - input_path.front().x);
  if (isB2StartTurnSupported(
        input_path.front(), start_yaw, first_path_yaw,
        config.yaw_sample_step, pose_supported) &&
      isB2StartPathTranslationSupported(
        input_path, config.path_sample_step, pose_supported))
  {
    result.status = B2StartManeuverStatus::DIRECT_SAFE;
    result.path = input_path;
    return result;
  }

  // Remaining input length is used only to choose the least disruptive join
  // for a given escape distance.
  std::vector<double> suffix_length(input_path.size(), 0.0);
  for (std::size_t index = input_path.size() - 1; index > 0; --index)
  {
    suffix_length[index - 1] =
        suffix_length[index] +
        b2StartHorizontalDistance(input_path[index - 1], input_path[index]);
  }

  const B2StartManeuverPoint start = input_path.front();
  const double cos_start = std::cos(start_yaw);
  const double sin_start = std::sin(start_yaw);
  const std::size_t forward_steps = static_cast<std::size_t>(
      std::ceil(
          config.maximum_forward_distance / config.forward_step));
  std::vector<B2StartManeuverPoint> forward_path{start};

  for (std::size_t step = 1; step <= forward_steps; ++step)
  {
    const double distance = std::min(
        config.maximum_forward_distance,
        static_cast<double>(step) * config.forward_step);
    B2StartManeuverPoint escape;
    escape.x = start.x + distance * cos_start;
    escape.y = start.y + distance * sin_start;
    if (!ground_height(
          escape.x, escape.y, forward_path.back().z, escape.z) ||
        !appendSupportedB2StartSegment(
          forward_path.back(), escape, config.path_sample_step,
          pose_supported))
    {
      break;
    }
    forward_path.push_back(escape);

    double best_score = std::numeric_limits<double>::infinity();
    std::size_t best_join = input_path.size();
    for (std::size_t join = 1; join < input_path.size(); ++join)
    {
      const double connector_distance =
          b2StartHorizontalDistance(escape, input_path[join]);
      if (connector_distance <= 1e-4 ||
          connector_distance > config.maximum_join_distance)
      {
        continue;
      }
      const double connector_yaw = std::atan2(
          input_path[join].y - escape.y,
          input_path[join].x - escape.x);
      if (!isB2StartTurnSupported(
            escape, start_yaw, connector_yaw,
            config.yaw_sample_step, pose_supported) ||
          !appendSupportedB2StartSegment(
            escape, input_path[join], config.path_sample_step,
            pose_supported) ||
          (join + 1 < input_path.size() &&
           !isB2StartPathTranslationSupported(
             input_path, config.path_sample_step,
             pose_supported, join)))
      {
        continue;
      }

      const std::size_t outgoing =
          nextDistinctB2StartPoint(input_path, join);
      if (outgoing < input_path.size())
      {
        const double outgoing_yaw = std::atan2(
            input_path[outgoing].y - input_path[join].y,
            input_path[outgoing].x - input_path[join].x);
        if (!isB2StartTurnSupported(
              input_path[join], connector_yaw, outgoing_yaw,
              config.yaw_sample_step, pose_supported))
        {
          continue;
        }
      }

      const double score =
          connector_distance + suffix_length[join];
      if (score < best_score)
      {
        best_score = score;
        best_join = join;
      }
    }

    if (best_join >= input_path.size())
    {
      if (distance >= config.maximum_forward_distance - 1e-9)
        break;
      continue;
    }

    result.path = forward_path;
    if (!appendSupportedB2StartSegment(
          escape, input_path[best_join], config.path_sample_step,
          pose_supported, &result.path))
    {
      result.path.clear();
      continue;
    }
    result.path.insert(
        result.path.end(),
        input_path.begin() + static_cast<std::ptrdiff_t>(best_join + 1),
        input_path.end());
    result.status = B2StartManeuverStatus::REPAIRED;
    result.forward_distance = distance;
    result.join_index = best_join;
    return result;
  }

  result.status = B2StartManeuverStatus::BLOCKED;
  return result;
}

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__B2_START_MANEUVER_H_
