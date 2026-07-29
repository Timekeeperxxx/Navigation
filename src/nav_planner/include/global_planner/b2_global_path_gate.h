#ifndef GLOBAL_PLANNER__B2_GLOBAL_PATH_GATE_H_
#define GLOBAL_PLANNER__B2_GLOBAL_PATH_GATE_H_

#include <global_planner/b2_start_maneuver.h>

#include <cmath>
#include <optional>
#include <vector>

namespace global_planner
{

enum class B2GlobalPathGateStatus
{
  DIRECT_SAFE,
  FORWARD_REPAIRED,
  ENDPOINT_MISMATCH,
  GOAL_POSE_UNSUPPORTED,
  TERMINAL_SWEEP_UNSUPPORTED,
  PATH_TURN_UNSUPPORTED,
  START_BLOCKED,
  LOCAL_SAFETY_HANDOFF,
  INVALID,
};

struct B2GlobalPathGateConfig
{
  B2StartManeuverConfig start_maneuver;
  double endpoint_xy_tolerance{1e-3};
  double endpoint_z_tolerance{1e-3};
  bool delegate_pose_safety_to_local_planner{false};
};

struct B2GlobalPathGateResult
{
  B2GlobalPathGateStatus status{B2GlobalPathGateStatus::INVALID};
  std::vector<B2StartManeuverPoint> path;
  double forward_escape_distance{0.0};
};

inline B2GlobalPathGateResult prepareB2GlobalPath(
    const std::vector<B2StartManeuverPoint> & candidate,
    double live_start_yaw,
    const B2StartManeuverPoint & exact_goal,
    const std::optional<double> & exact_goal_yaw,
    const B2GlobalPathGateConfig & config,
    const B2PoseSupportQuery & pose_supported,
    const B2GroundHeightQuery & ground_height)
{
  B2GlobalPathGateResult result;
  if (candidate.size() < 2 || !pose_supported || !ground_height ||
      !std::isfinite(live_start_yaw) ||
      !std::isfinite(exact_goal.x) ||
      !std::isfinite(exact_goal.y) ||
      !std::isfinite(exact_goal.z) ||
      (exact_goal_yaw && !std::isfinite(*exact_goal_yaw)))
  {
    return result;
  }

  const B2StartManeuverPoint & candidate_goal = candidate.back();
  if (b2StartHorizontalDistance(candidate_goal, exact_goal) >
          std::max(0.0, config.endpoint_xy_tolerance) ||
      std::abs(candidate_goal.z - exact_goal.z) >
          std::max(0.0, config.endpoint_z_tolerance))
  {
    result.status = B2GlobalPathGateStatus::ENDPOINT_MISMATCH;
    return result;
  }

  // The global route has already passed graph/ground edge and turn checks.
  // When this handoff mode is enabled, do not reinterpret every raw polyline
  // vertex as a mandatory in-place rotation and veto the connected route a
  // second time. SCAN may smooth those vertices; its continuous safety check
  // remains responsible for live obstacles, not static ground support.
  if (config.delegate_pose_safety_to_local_planner)
  {
    result.path = candidate;
    result.path.back() = exact_goal;
    result.status = B2GlobalPathGateStatus::LOCAL_SAFETY_HANDOFF;
    return result;
  }

  const B2StartManeuverResult start_result =
      makeB2StartManeuver(
          candidate, live_start_yaw, config.start_maneuver,
          pose_supported, ground_height);
  if (start_result.status == B2StartManeuverStatus::BLOCKED)
  {
    result.status = B2GlobalPathGateStatus::START_BLOCKED;
    return result;
  }
  if (start_result.status != B2StartManeuverStatus::DIRECT_SAFE &&
      start_result.status != B2StartManeuverStatus::REPAIRED)
  {
    return result;
  }
  if (start_result.path.size() < 2)
    return result;

  for (std::size_t pivot = 1;
       pivot + 1 < start_result.path.size(); ++pivot)
  {
    std::size_t previous = pivot;
    while (
        previous > 0 &&
        b2StartHorizontalDistance(
            start_result.path[previous - 1],
            start_result.path[pivot]) <= 1e-6)
    {
      --previous;
    }
    const std::size_t next =
        nextDistinctB2StartPoint(start_result.path, pivot);
    if (previous == 0 || next >= start_result.path.size())
      continue;

    const double incoming_yaw = std::atan2(
        start_result.path[pivot].y -
            start_result.path[previous - 1].y,
        start_result.path[pivot].x -
            start_result.path[previous - 1].x);
    const double outgoing_yaw = std::atan2(
        start_result.path[next].y -
            start_result.path[pivot].y,
        start_result.path[next].x -
            start_result.path[pivot].x);
    if (!isB2StartTurnSupported(
          start_result.path[pivot],
          incoming_yaw,
          outgoing_yaw,
          config.start_maneuver.yaw_sample_step,
          pose_supported))
    {
      result.status =
          B2GlobalPathGateStatus::PATH_TURN_UNSUPPORTED;
      return result;
    }
  }

  if (exact_goal_yaw)
  {
    if (!pose_supported(exact_goal, *exact_goal_yaw))
    {
      result.status = B2GlobalPathGateStatus::GOAL_POSE_UNSUPPORTED;
      return result;
    }

    std::size_t incoming_index = start_result.path.size() - 1;
    while (incoming_index > 0 &&
           b2StartHorizontalDistance(
             start_result.path[incoming_index - 1],
             start_result.path.back()) <= 1e-6)
    {
      --incoming_index;
    }
    if (incoming_index == 0)
      return result;
    const B2StartManeuverPoint & previous =
        start_result.path[incoming_index - 1];
    const double arrival_yaw = std::atan2(
        exact_goal.y - previous.y,
        exact_goal.x - previous.x);
    if (!isB2StartTurnSupported(
          exact_goal, arrival_yaw, *exact_goal_yaw,
          config.start_maneuver.yaw_sample_step, pose_supported))
    {
      result.status =
          B2GlobalPathGateStatus::TERMINAL_SWEEP_UNSUPPORTED;
      return result;
    }
  }

  result.path = start_result.path;
  // The tolerance above is only for floating-point/cloud conversion noise.
  // A publishable result must still carry the exact requested endpoint, never
  // a nearby cloud sample.
  result.path.back() = exact_goal;
  result.forward_escape_distance = start_result.forward_distance;
  result.status =
      start_result.status == B2StartManeuverStatus::REPAIRED
          ? B2GlobalPathGateStatus::FORWARD_REPAIRED
          : B2GlobalPathGateStatus::DIRECT_SAFE;
  return result;
}

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__B2_GLOBAL_PATH_GATE_H_
