#ifndef SCAN_PLANNER_B2_MOTION_POLICY_H
#define SCAN_PLANNER_B2_MOTION_POLICY_H

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace scan_planner
{

struct B2ForwardSeedConfig
{
  double speed = 0.15;
  double clearance_distance = 0.25;
  double sample_step = 0.025;
  double minimum_reference_alignment = 0.25;
};

/**
 * Seed a stopped B2 trajectory with its current body heading.
 *
 * A zero start derivative lets the position-only B-spline choose an arbitrary
 * first tangent.  The controller then treats that tangent as body yaw and may
 * rotate the long, asymmetric B2 footprint over a nearby ground edge before
 * translating.  This helper only returns a non-zero seed after proving a
 * short, forward, fixed-yaw corridor with the same pose validator used by the
 * execution gate.
 */
inline bool makeB2ForwardSeedVelocity(
    const Eigen::Vector3d &start,
    double body_yaw,
    const Eigen::Vector2d &reference_direction,
    const B2ForwardSeedConfig &config,
    const std::function<bool(const Eigen::Vector3d &, double)> &pose_is_safe,
    Eigen::Vector3d &velocity)
{
  velocity.setZero();
  if (!start.allFinite() || !std::isfinite(body_yaw) ||
      !reference_direction.allFinite() ||
      reference_direction.norm() <= 1e-6 ||
      config.speed <= 1e-6 ||
      config.clearance_distance <= 1e-6 ||
      !pose_is_safe)
    return false;

  const Eigen::Vector2d heading(std::cos(body_yaw), std::sin(body_yaw));
  const Eigen::Vector2d reference = reference_direction.normalized();
  if (heading.dot(reference) < config.minimum_reference_alignment)
    return false;

  const double sample_step = std::max(0.01, config.sample_step);
  const std::size_t samples = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(
          std::ceil(config.clearance_distance / sample_step)));
  for (std::size_t index = 0; index <= samples; ++index)
  {
    const double distance =
        config.clearance_distance * static_cast<double>(index) /
        static_cast<double>(samples);
    Eigen::Vector3d position = start;
    position.head<2>() += distance * heading;
    if (!pose_is_safe(position, body_yaw))
      return false;
  }

  velocity.head<2>() = std::max(0.0, config.speed) * heading;
  return true;
}

/**
 * Seed a stopped B2 along the active reference direction.
 *
 * This is the safe fallback when the live body heading no longer agrees with
 * the next reference segment.  The controller can then align in place before
 * advancing the spline clock.  Validate both the complete shortest-yaw turn
 * at the current pose and the short forward corridor before exposing that
 * boundary velocity to the position-only B-spline optimizer.
 */
inline bool makeB2ReferenceSeedVelocity(
    const Eigen::Vector3d &start,
    double body_yaw,
    const Eigen::Vector2d &reference_direction,
    const B2ForwardSeedConfig &config,
    const std::function<bool(const Eigen::Vector3d &, double)> &pose_is_safe,
    Eigen::Vector3d &velocity,
    double maximum_yaw_step = 0.08726646259971647)
{
  velocity.setZero();
  if (!start.allFinite() || !std::isfinite(body_yaw) ||
      !reference_direction.allFinite() ||
      reference_direction.norm() <= 1e-6 ||
      config.speed <= 1e-6 ||
      config.clearance_distance <= 1e-6 ||
      !pose_is_safe)
    return false;

  const Eigen::Vector2d reference = reference_direction.normalized();
  const double reference_yaw =
      std::atan2(reference.y(), reference.x());
  const double yaw_delta = std::atan2(
      std::sin(reference_yaw - body_yaw),
      std::cos(reference_yaw - body_yaw));
  const double safe_yaw_step =
      std::max(0.017453292519943295, std::abs(maximum_yaw_step));
  const std::size_t yaw_samples = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(
          std::ceil(std::abs(yaw_delta) / safe_yaw_step)));
  for (std::size_t index = 0; index <= yaw_samples; ++index)
  {
    const double ratio =
        static_cast<double>(index) / static_cast<double>(yaw_samples);
    const double yaw = body_yaw + ratio * yaw_delta;
    if (!pose_is_safe(start, yaw))
      return false;
  }

  const double sample_step = std::max(0.01, config.sample_step);
  const std::size_t position_samples = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(
          std::ceil(config.clearance_distance / sample_step)));
  for (std::size_t index = 1; index <= position_samples; ++index)
  {
    const double distance =
        config.clearance_distance * static_cast<double>(index) /
        static_cast<double>(position_samples);
    Eigen::Vector3d position = start;
    position.head<2>() += distance * reference;
    if (!pose_is_safe(position, reference_yaw))
      return false;
  }

  velocity.head<2>() = std::max(0.0, config.speed) * reference;
  return true;
}

struct B2DirectionGuardConfig
{
  double probe_distance = 0.20;
  double guard_distance = 0.60;
  double maximum_reference_heading_error = 1.0471975511965976;
  double backtrack_tolerance = 0.03;
  double obstacle_detour_retreat_allowance = 0.35;
  double obstacle_detour_minimum_progress = 0.20;
  double obstacle_detour_maximum_length_ratio = 2.50;
};

struct B2ForwardDetourSearchConfig
{
  double position_resolution = 0.05;
  // Keep the lattice compact, but validate every transition at the same
  // continuous position density as the downstream execution gate.  These
  // are deliberately separate: a 5 cm state grid must not skip a narrow
  // unsupported strip halfway between two safe states.
  double maximum_position_step = 0.025;
  double translation_step = 0.10;
  double arc_translation_step = 0.20;
  int heading_bins = 24;
  double search_margin = 1.25;
  double maximum_search_extent = 5.0;
  int maximum_expansions = 30000;
  double goal_tolerance = 0.15;
  double maximum_yaw_step = 0.08726646259971647;
  double rotation_cost_per_radian = 0.10;
};

/**
 * Check whether the live B2 pose can safely rejoin one straight guide leg.
 *
 * A cached recovery waypoint was validated from the pose where the guide was
 * first generated.  Continuous replanning can begin again part-way through
 * that leg with a different XY/yaw.  Reusing the old endpoint without
 * checking the new turn-in-place sweep recreates a permanent s=0 collision.
 */
inline bool isB2StopTurnForwardLegSafe(
    const Eigen::Vector3d &start,
    double start_yaw,
    const Eigen::Vector3d &target,
    double maximum_yaw_step,
    double maximum_position_step,
    const std::function<bool(const Eigen::Vector3d &, double)> &pose_is_safe)
{
  if (!start.allFinite() || !target.allFinite() ||
      !std::isfinite(start_yaw) || !pose_is_safe)
    return false;

  const Eigen::Vector2d delta = (target - start).head<2>();
  if (delta.norm() <= 1e-6)
    return pose_is_safe(start, start_yaw);

  const double target_yaw = std::atan2(delta.y(), delta.x());
  const double yaw_delta =
      std::atan2(
          std::sin(target_yaw - start_yaw),
          std::cos(target_yaw - start_yaw));
  const double yaw_step =
      std::max(0.017453292519943295, std::abs(maximum_yaw_step));
  const int yaw_samples = std::max(
      1, static_cast<int>(std::ceil(std::abs(yaw_delta) / yaw_step)));
  for (int sample = 0; sample <= yaw_samples; ++sample)
  {
    const double ratio =
        static_cast<double>(sample) / static_cast<double>(yaw_samples);
    if (!pose_is_safe(start, start_yaw + ratio * yaw_delta))
      return false;
  }

  const double position_step =
      std::max(0.01, std::abs(maximum_position_step));
  const int position_samples = std::max(
      1, static_cast<int>(std::ceil(delta.norm() / position_step)));
  for (int sample = 1; sample <= position_samples; ++sample)
  {
    const double ratio =
        static_cast<double>(sample) /
        static_cast<double>(position_samples);
    if (!pose_is_safe(start + ratio * (target - start), target_yaw))
      return false;
  }
  return true;
}

/**
 * Select the farthest ordered reference point reachable by one verified
 * stop-turn-forward leg from the live B2 pose.
 *
 * The far local target can be safe on the original polyline while its direct
 * chord leaves that corridor.  Walking the guide backwards finds the most
 * useful deterministic rejoin without assuming that the complete lookahead
 * can be shortcut.
 */
inline bool findFarthestSafeB2ReferenceLeg(
    const Eigen::Vector3d &start,
    double start_yaw,
    const std::vector<Eigen::Vector3d> &reference_guide,
    double minimum_leg_length,
    double maximum_yaw_step,
    double maximum_position_step,
    const std::function<bool(const Eigen::Vector3d &, double)> &pose_is_safe,
    Eigen::Vector3d &target,
    std::size_t *target_index = nullptr)
{
  if (!start.allFinite() || !std::isfinite(start_yaw) ||
      reference_guide.size() < 2 || !pose_is_safe)
    return false;

  const double minimum_distance = std::max(0.02, minimum_leg_length);
  for (std::size_t reverse_index = reference_guide.size();
       reverse_index > 0;
       --reverse_index)
  {
    const std::size_t index = reverse_index - 1;
    const Eigen::Vector3d &candidate = reference_guide[index];
    if (!candidate.allFinite() ||
        (candidate - start).head<2>().norm() < minimum_distance)
      continue;
    if (!isB2StopTurnForwardLegSafe(
            start,
            start_yaw,
            candidate,
            maximum_yaw_step,
            maximum_position_step,
            pose_is_safe))
      continue;

    target = candidate;
    if (target_index)
      *target_index = index;
    return true;
  }
  return false;
}

/**
 * Find dynamic occupancy anywhere along an ordered B2 reference guide.
 *
 * Checking only guide vertices can miss a narrow obstacle between two path
 * samples.  Sample every segment at execution-validation density and use the
 * segment tangent as body yaw so the caller's double-circle occupancy query
 * covers the physical B2 footprint.  The reported segment range lets a
 * detour rejoin strictly beyond the complete blocked portion instead of
 * selecting the last free point immediately in front of it.
 */
inline bool findB2ReferenceDynamicBlockage(
    const std::vector<Eigen::Vector3d> &reference_guide,
    double maximum_position_step,
    const std::function<bool(const Eigen::Vector3d &, double)>
        &pose_is_dynamically_occupied,
    std::size_t *first_blocked_segment = nullptr,
    std::size_t *last_blocked_segment = nullptr)
{
  if (reference_guide.size() < 2 || !pose_is_dynamically_occupied)
    return false;

  const double position_step =
      std::max(0.01, std::abs(maximum_position_step));
  bool found_blockage = false;
  std::size_t first_blocked = 0;
  std::size_t last_blocked = 0;

  for (std::size_t segment_index = 0;
       segment_index + 1 < reference_guide.size();
       ++segment_index)
  {
    const Eigen::Vector3d &from = reference_guide[segment_index];
    const Eigen::Vector3d &to = reference_guide[segment_index + 1];
    if (!from.allFinite() || !to.allFinite())
      continue;

    const Eigen::Vector2d delta = (to - from).head<2>();
    const double length = delta.norm();
    if (length <= 1e-6)
      continue;

    const double yaw = std::atan2(delta.y(), delta.x());
    const int samples = std::max(
        1, static_cast<int>(std::ceil(length / position_step)));
    bool segment_blocked = false;
    for (int sample = 0; sample <= samples; ++sample)
    {
      const double ratio =
          static_cast<double>(sample) / static_cast<double>(samples);
      if (pose_is_dynamically_occupied(
              from + ratio * (to - from), yaw))
      {
        segment_blocked = true;
        break;
      }
    }

    if (!segment_blocked)
      continue;
    if (!found_blockage)
      first_blocked = segment_index;
    last_blocked = segment_index;
    found_blockage = true;
  }

  if (!found_blockage)
    return false;
  if (first_blocked_segment)
    *first_blocked_segment = first_blocked;
  if (last_blocked_segment)
    *last_blocked_segment = last_blocked;
  return true;
}

/**
 * Find a forward-only, heading-aware B2 detour.
 *
 * SCAN's historical local A* indexes only XYZ. That is sufficient for an
 * aerial robot, but not for the long asymmetric B2 footprint: the same XY
 * pose can be supported at one yaw and hang over a ground edge at another.
 * This small SE(2) lattice is used only while obstacle recovery is latched.
 * It permits in-place turns and forward translations, validating every yaw
 * sweep and translation sample through the caller's full footprint checker.
 */
inline bool searchB2ForwardDetour(
    const Eigen::Vector3d &start,
    double start_yaw,
    const Eigen::Vector3d &target,
    const B2ForwardDetourSearchConfig &config,
    const std::function<bool(const Eigen::Vector3d &, double)> &pose_is_safe,
    std::vector<Eigen::Vector3d> &path,
    std::vector<double> *path_yaws = nullptr,
    std::vector<bool> *simultaneous_yaw_motion = nullptr)
{
  path.clear();
  if (path_yaws)
    path_yaws->clear();
  if (simultaneous_yaw_motion)
    simultaneous_yaw_motion->clear();
  if (!start.allFinite() || !target.allFinite() ||
      !std::isfinite(start_yaw) || !pose_is_safe)
    return false;

  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  const double resolution =
      std::max(0.025, config.position_resolution);
  const double translation_step =
      std::max(resolution, config.translation_step);
  const int heading_bins = std::max(8, config.heading_bins);
  const double heading_step = kTwoPi / static_cast<double>(heading_bins);
  const double direct_distance = (target - start).head<2>().norm();
  if (direct_distance <= std::max(0.02, config.goal_tolerance))
  {
    if (isB2StopTurnForwardLegSafe(
            start,
            start_yaw,
            target,
            config.maximum_yaw_step,
            config.maximum_position_step,
            pose_is_safe))
    {
      path = {start, target};
      if (path_yaws)
      {
        const Eigen::Vector2d delta = (target - start).head<2>();
        const double target_yaw =
            std::atan2(delta.y(), delta.x());
        *path_yaws = {start_yaw, target_yaw};
      }
      if (simultaneous_yaw_motion)
        *simultaneous_yaw_motion = {false, false};
      return true;
    }
    // A nearby XY target is not automatically reachable by B2: the body
    // yaw sweep can still hit an obstacle or lose same-floor support. Keep
    // searching for a short forward arc instead of returning an unsafe leg.
  }

  const double extent = std::min(
      std::max(1.0, config.maximum_search_extent),
      std::max(1.0, direct_distance + std::max(0.25, config.search_margin)));
  const int half_cells =
      std::max(2, static_cast<int>(std::ceil(extent / resolution)));
  const int width = 2 * half_cells + 1;
  const std::size_t state_count =
      static_cast<std::size_t>(width) *
      static_cast<std::size_t>(width) *
      static_cast<std::size_t>(heading_bins);

  struct Record
  {
    double cost = std::numeric_limits<double>::infinity();
    int parent = -1;
    bool closed = false;
    signed char occupancy = -1;
  };
  struct QueueItem
  {
    double score;
    int state;
  };
  struct QueueGreater
  {
    bool operator()(const QueueItem &left, const QueueItem &right) const
    {
      return left.score > right.score;
    }
  };

  std::vector<Record> records(state_count);
  auto normalizeHeading = [&](int heading) {
    heading %= heading_bins;
    if (heading < 0)
      heading += heading_bins;
    return heading;
  };
  auto stateIndex = [&](int ix, int iy, int heading) {
    return
        ((iy + half_cells) * width + (ix + half_cells)) *
            heading_bins +
        normalizeHeading(heading);
  };
  auto decodeState = [&](int state, int &ix, int &iy, int &heading) {
    heading = state % heading_bins;
    const int cell = state / heading_bins;
    ix = cell % width - half_cells;
    iy = cell / width - half_cells;
  };
  auto headingYaw = [&](int heading) {
    return static_cast<double>(normalizeHeading(heading)) * heading_step;
  };
  auto positionAt = [&](int ix, int iy) {
    Eigen::Vector3d position = start;
    position.x() += static_cast<double>(ix) * resolution;
    position.y() += static_cast<double>(iy) * resolution;
    const Eigen::Vector2d target_delta =
        (target - start).head<2>();
    const double target_length_squared = target_delta.squaredNorm();
    double ratio = 0.0;
    if (target_length_squared > 1e-9)
    {
      ratio =
          (position - start).head<2>().dot(target_delta) /
          target_length_squared;
      ratio = std::max(0.0, std::min(1.0, ratio));
    }
    position.z() = start.z() + ratio * (target.z() - start.z());
    return position;
  };
  auto shortestYawDelta = [&](double from, double to) {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  };
  const double safe_yaw_step =
      std::max(0.017453292519943295, std::abs(config.maximum_yaw_step));
  auto yawSweepSafe = [&](const Eigen::Vector3d &position,
                          double from_yaw,
                          double to_yaw) {
    const double delta = shortestYawDelta(from_yaw, to_yaw);
    const int samples = std::max(
        1, static_cast<int>(std::ceil(std::abs(delta) / safe_yaw_step)));
    for (int sample = 0; sample <= samples; ++sample)
    {
      const double ratio =
          static_cast<double>(sample) / static_cast<double>(samples);
      if (!pose_is_safe(position, from_yaw + ratio * delta))
        return false;
    }
    return true;
  };
  auto translationSafe = [&](const Eigen::Vector3d &from,
                             const Eigen::Vector3d &to,
                             double yaw) {
    const double length = (to - from).head<2>().norm();
    const double sample_step = std::min(
        std::min(resolution, 0.05),
        std::max(0.01, std::abs(config.maximum_position_step)));
    const int samples = std::max(
        1, static_cast<int>(std::ceil(length / sample_step)));
    for (int sample = 1; sample <= samples; ++sample)
    {
      const double ratio =
          static_cast<double>(sample) / static_cast<double>(samples);
      if (!pose_is_safe(from + ratio * (to - from), yaw))
        return false;
    }
    return true;
  };
  auto forwardMotionSafe = [&](const Eigen::Vector3d &from,
                               double from_yaw,
                               const Eigen::Vector3d &to,
                               double to_yaw) {
    const double length = (to - from).head<2>().norm();
    const double yaw_delta = shortestYawDelta(from_yaw, to_yaw);
    const int position_samples = std::max(
        1,
        static_cast<int>(std::ceil(
            length /
            std::min(
                std::min(resolution, 0.05),
                std::max(
                    0.01,
                    std::abs(config.maximum_position_step))))));
    const int yaw_samples = std::max(
        1,
        static_cast<int>(std::ceil(
            std::abs(yaw_delta) / safe_yaw_step)));
    const int samples = std::max(position_samples, yaw_samples);
    for (int sample = 1; sample <= samples; ++sample)
    {
      const double ratio =
          static_cast<double>(sample) / static_cast<double>(samples);
      if (!pose_is_safe(
              from + ratio * (to - from),
              from_yaw + ratio * yaw_delta))
      {
        return false;
      }
    }
    return true;
  };
  auto statePoseSafe = [&](int state) {
    Record &record = records[static_cast<std::size_t>(state)];
    if (record.occupancy >= 0)
      return record.occupancy == 0;
    int ix = 0;
    int iy = 0;
    int heading = 0;
    decodeState(state, ix, iy, heading);
    const bool safe =
        pose_is_safe(positionAt(ix, iy), headingYaw(heading));
    record.occupancy = safe ? 0 : 1;
    return safe;
  };

  const int start_heading = normalizeHeading(
      static_cast<int>(std::llround(start_yaw / heading_step)));
  const int start_state = stateIndex(0, 0, start_heading);
  // The live pose is already at start_yaw. Do not require an artificial
  // in-place sweep to the nearest heading bin before search begins. Close to
  // an obstacle or ground edge that tiny quantization turn may be unsafe
  // while a simultaneous forward steering arc from the exact live yaw is
  // perfectly valid.
  if (!pose_is_safe(start, start_yaw))
    return false;
  records[static_cast<std::size_t>(start_state)].occupancy = 0;

  std::priority_queue<
      QueueItem, std::vector<QueueItem>, QueueGreater> open;
  records[static_cast<std::size_t>(start_state)].cost = 0.0;
  open.push(QueueItem{direct_distance, start_state});
  int goal_state = -1;
  double goal_connector_yaw = start_yaw;
  bool goal_connector_simultaneous_yaw = false;
  int expansions = 0;
  const int maximum_expansions =
      std::max(100, config.maximum_expansions);
  const double goal_tolerance =
      std::max(resolution, config.goal_tolerance);

  while (!open.empty() && expansions < maximum_expansions)
  {
    const QueueItem item = open.top();
    open.pop();
    Record &current_record =
        records[static_cast<std::size_t>(item.state)];
    if (current_record.closed)
      continue;
    current_record.closed = true;
    ++expansions;

    int ix = 0;
    int iy = 0;
    int heading = 0;
    decodeState(item.state, ix, iy, heading);
    const Eigen::Vector3d current_position = positionAt(ix, iy);
    const double current_yaw =
        item.state == start_state
            ? start_yaw
            : headingYaw(heading);
    const Eigen::Vector2d to_target =
        (target - current_position).head<2>();
    if (to_target.norm() <= goal_tolerance)
    {
      const double connector_yaw =
          std::atan2(to_target.y(), to_target.x());
      const bool stationary_turn_then_forward =
          to_target.norm() > 1e-6 &&
          yawSweepSafe(
              current_position, current_yaw, connector_yaw) &&
          translationSafe(current_position, target, connector_yaw);
      const bool forward_steering_connector =
          to_target.norm() > 1e-6 &&
          !stationary_turn_then_forward &&
          forwardMotionSafe(
              current_position, current_yaw, target, connector_yaw);
      if (to_target.norm() <= 1e-6 ||
          stationary_turn_then_forward ||
          forward_steering_connector)
      {
        goal_state = item.state;
        goal_connector_yaw =
            to_target.norm() <= 1e-6
                ? current_yaw
                : connector_yaw;
        goal_connector_simultaneous_yaw =
            forward_steering_connector;
        break;
      }
    }

    auto relax = [&](int next_state, double transition_cost) {
      if (next_state < 0 ||
          next_state >= static_cast<int>(records.size()) ||
          !statePoseSafe(next_state))
        return;
      Record &next_record =
          records[static_cast<std::size_t>(next_state)];
      if (next_record.closed)
        return;
      const double next_cost = current_record.cost + transition_cost;
      if (next_cost + 1e-12 >= next_record.cost)
        return;
      next_record.cost = next_cost;
      next_record.parent = item.state;
      int next_ix = 0;
      int next_iy = 0;
      int next_heading = 0;
      decodeState(
          next_state, next_ix, next_iy, next_heading);
      const double heuristic =
          (target - positionAt(next_ix, next_iy)).head<2>().norm();
      open.push(QueueItem{next_cost + heuristic, next_state});
    };

    for (int direction : {-1, 1})
    {
      const int next_heading = normalizeHeading(heading + direction);
      const double next_yaw = headingYaw(next_heading);
      if (!yawSweepSafe(current_position, current_yaw, next_yaw))
        continue;
      relax(
          stateIndex(ix, iy, next_heading),
          std::max(0.0, config.rotation_cost_per_radian) *
              std::abs(shortestYawDelta(current_yaw, next_yaw)));
    }

    // Forward steering arcs are essential for the long B2 footprint. Near a
    // ground edge the robot can be unable to rotate in place even though a
    // forward curve keeps both body circles supported and clears the
    // obstacle. The old rotate-then-straight lattice incorrectly declared
    // that situation trapped.
    for (int steering : {-1, 0, 1})
    {
      const int next_heading = normalizeHeading(heading + steering);
      const double next_yaw = headingYaw(next_heading);
      const double motion_length =
          steering == 0
              ? translation_step
              : std::max(
                    translation_step,
                    config.arc_translation_step);
      double proposed_x = current_position.x();
      double proposed_y = current_position.y();
      if (steering == 0)
      {
        proposed_x += motion_length * std::cos(current_yaw);
        proposed_y += motion_length * std::sin(current_yaw);
      }
      else
      {
        const double yaw_delta =
            shortestYawDelta(current_yaw, next_yaw);
        const double radius = motion_length / yaw_delta;
        proposed_x +=
            radius *
            (std::sin(current_yaw + yaw_delta) -
             std::sin(current_yaw));
        proposed_y +=
            -radius *
            (std::cos(current_yaw + yaw_delta) -
             std::cos(current_yaw));
      }

      const int next_ix = static_cast<int>(std::llround(
          (proposed_x - start.x()) / resolution));
      const int next_iy = static_cast<int>(std::llround(
          (proposed_y - start.y()) / resolution));
      if (std::abs(next_ix) > half_cells ||
          std::abs(next_iy) > half_cells ||
          (next_ix == ix && next_iy == iy))
        continue;

      const Eigen::Vector3d next_position =
          positionAt(next_ix, next_iy);
      if (!forwardMotionSafe(
              current_position,
              current_yaw,
              next_position,
              next_yaw))
        continue;

      const int next_state =
          stateIndex(next_ix, next_iy, next_heading);
      relax(
          next_state,
          (next_position - current_position).head<2>().norm() +
              0.25 *
                  std::max(0.0, config.rotation_cost_per_radian) *
                  std::abs(shortestYawDelta(current_yaw, next_yaw)));
    }
  }

  if (goal_state < 0)
    return false;

  std::vector<int> states;
  for (int state = goal_state; state >= 0;)
  {
    states.push_back(state);
    if (state == start_state)
      break;
    state = records[static_cast<std::size_t>(state)].parent;
  }
  if (states.empty() || states.back() != start_state)
    return false;
  std::reverse(states.begin(), states.end());

  std::vector<double> dense_yaws;
  std::vector<bool> dense_simultaneous_yaw_motion;
  path.push_back(start);
  dense_yaws.push_back(start_yaw);
  dense_simultaneous_yaw_motion.push_back(false);

  double previous_yaw = start_yaw;
  int ignored_ix = 0;
  int ignored_iy = 0;
  int ignored_heading = 0;
  decodeState(
      states.front(),
      ignored_ix,
      ignored_iy,
      ignored_heading);
  for (std::size_t state_index = 1;
       state_index < states.size();
       ++state_index)
  {
    int ix = 0;
    int iy = 0;
    int heading = 0;
    decodeState(states[state_index], ix, iy, heading);
    const Eigen::Vector3d position = positionAt(ix, iy);
    if ((position - path.back()).head<2>().norm() > resolution * 0.25)
    {
      const double state_yaw = headingYaw(heading);
      path.push_back(position);
      dense_yaws.push_back(state_yaw);
      dense_simultaneous_yaw_motion.push_back(
          std::abs(shortestYawDelta(
              previous_yaw, state_yaw)) > 1e-6);
    }
    previous_yaw = headingYaw(heading);
  }
  if ((target - path.back()).head<2>().norm() > 1e-4)
  {
    path.push_back(target);
    dense_yaws.push_back(goal_connector_yaw);
    dense_simultaneous_yaw_motion.push_back(
        goal_connector_simultaneous_yaw);
  }
  else
  {
    path.back() = target;
    dense_yaws.back() = goal_connector_yaw;
  }

  // The lattice contains many short staircase segments. Feeding those
  // directly to cubic B-spline parameterization can create a large overshoot
  // at the first turn even though every lattice pose is safe. Greedily keep
  // the farthest visible successor whose complete yaw sweep and translation
  // remain valid. This preserves the same safety contract while presenting a
  // small, stable polyline to SCAN's smoother.
  if (path.size() > 2)
  {
    const std::vector<Eigen::Vector3d> lattice_path = path;
    std::vector<Eigen::Vector3d> simplified;
    std::vector<double> simplified_yaws;
    std::vector<bool> simplified_simultaneous_yaw_motion;
    simplified.reserve(lattice_path.size());
    simplified.push_back(lattice_path.front());
    simplified_yaws.push_back(start_yaw);
    simplified_simultaneous_yaw_motion.push_back(false);
    std::size_t current = 0;
    double incoming_yaw = start_yaw;
    while (current + 1 < lattice_path.size())
    {
      std::size_t selected = current + 1;
      double selected_yaw = std::atan2(
          lattice_path[selected].y() - lattice_path[current].y(),
          lattice_path[selected].x() - lattice_path[current].x());
      bool selected_simultaneous_yaw_motion =
          dense_simultaneous_yaw_motion[selected];
      bool selected_by_stop_turn_visibility = false;
      for (std::size_t candidate = lattice_path.size() - 1;
           candidate > current;
           --candidate)
      {
        const Eigen::Vector2d delta =
            (lattice_path[candidate] - lattice_path[current]).head<2>();
        if (delta.norm() <= 1e-6)
          continue;
        const double candidate_yaw =
            std::atan2(delta.y(), delta.x());
        if (!yawSweepSafe(
                lattice_path[current], incoming_yaw, candidate_yaw) ||
            !translationSafe(
                lattice_path[current],
                lattice_path[candidate],
                candidate_yaw))
          continue;
        selected = candidate;
        selected_yaw = candidate_yaw;
        selected_simultaneous_yaw_motion = false;
        selected_by_stop_turn_visibility = true;
        break;
      }
      if (!selected_by_stop_turn_visibility)
        selected_yaw = dense_yaws[selected];
      simplified.push_back(lattice_path[selected]);
      simplified_yaws.push_back(selected_yaw);
      simplified_simultaneous_yaw_motion.push_back(
          selected_simultaneous_yaw_motion);
      incoming_yaw = selected_yaw;
      current = selected;
    }
    path = std::move(simplified);
    dense_yaws = std::move(simplified_yaws);
    dense_simultaneous_yaw_motion =
        std::move(simplified_simultaneous_yaw_motion);
  }
  if (path_yaws)
    *path_yaws = dense_yaws;
  if (simultaneous_yaw_motion)
    *simultaneous_yaw_motion =
        dense_simultaneous_yaw_motion;
  return path.size() >= 2;
}

/**
 * Keep bounded detour semantics active for the whole safety-frozen replan.
 *
 * An obstacle can leave the short current/forward probe after a recovery
 * displacement while still blocking the retained execution path. Dropping
 * detour mode at that point makes every following fallback obey the normal
 * reference cone again and recreates the permanent stop. The downstream
 * configured footprint and live-obstacle sweeps remain authoritative.
 */
inline bool shouldUseB2BoundedObstacleDetour(
    bool fallback_candidate,
    bool immediate_dynamic_obstacle,
    bool obstacle_recovery_latched)
{
  return fallback_candidate &&
      (immediate_dynamic_obstacle || obstacle_recovery_latched);
}

struct B2PathGeometryCheck
{
  bool valid = false;
  std::size_t segment = 0;
  double horizontal_distance = 0.0;
  double vertical_distance = 0.0;
  double slope = std::numeric_limits<double>::infinity();
};

/**
 * Reject a position spline that a ground robot cannot execute.
 *
 * Ground support proves that sampled poses lie on terrain, but it does not
 * rule out an interpolation cusp whose XY displacement approaches zero while
 * z changes. Such a segment produces both an unbounded terrain slope and an
 * unstable tangent-derived body yaw.
 */
inline B2PathGeometryCheck checkB2PathGeometry(
    const std::vector<Eigen::Vector3d> &path,
    double maximum_slope,
    double minimum_horizontal_distance = 1e-6,
    double maximum_degenerate_z_difference = 1e-4)
{
  B2PathGeometryCheck result;
  if (path.size() < 2 || !std::isfinite(maximum_slope))
    return result;

  const double safe_maximum_slope = std::max(0.0, maximum_slope);
  const double safe_minimum_horizontal =
      std::max(0.0, minimum_horizontal_distance);
  const double safe_degenerate_z =
      std::max(0.0, maximum_degenerate_z_difference);
  for (std::size_t index = 0; index + 1 < path.size(); ++index)
  {
    result.segment = index;
    if (!path[index].allFinite() || !path[index + 1].allFinite())
      return result;

    const Eigen::Vector3d delta = path[index + 1] - path[index];
    result.horizontal_distance = delta.head<2>().norm();
    result.vertical_distance = std::abs(delta.z());
    if (result.horizontal_distance <= safe_minimum_horizontal)
    {
      if (result.vertical_distance > safe_degenerate_z)
        return result;
      result.slope = 0.0;
      continue;
    }

    result.slope =
        result.vertical_distance / result.horizontal_distance;
    if (!std::isfinite(result.slope) ||
        result.slope > safe_maximum_slope)
      return result;
  }

  result.valid = true;
  return result;
}

/**
 * Reject an unguided fallback that initially leaves the forward/reference
 * cone or loses progress during its first body length.
 *
 * The check intentionally operates on sampled positions rather than the
 * derivative at t=0, which is often zero for a stopped replan.
 */
inline bool isB2ForwardCandidate(
    const std::vector<Eigen::Vector3d> &path,
    const Eigen::Vector2d &target_direction,
    const Eigen::Vector2d &reference_direction,
    const B2DirectionGuardConfig &config,
    bool enforce_initial_reference_cone = true,
    bool allow_bounded_obstacle_detour = false)
{
  if (path.size() < 2 ||
      !target_direction.allFinite() ||
      !reference_direction.allFinite() ||
      target_direction.norm() <= 1e-6 ||
      reference_direction.norm() <= 1e-6)
    return false;

  const Eigen::Vector2d start = path.front().head<2>();
  const double target_distance = target_direction.norm();
  const Eigen::Vector2d target = target_direction.normalized();
  const Eigen::Vector2d reference = reference_direction.normalized();
  const double probe_distance = std::max(0.02, config.probe_distance);
  const double guard_distance =
      std::max(probe_distance, config.guard_distance);
  constexpr double kPi = 3.14159265358979323846;
  const double max_reference_error = std::max(
      0.0,
      std::min(config.maximum_reference_heading_error, kPi));
  const double backtrack_tolerance =
      std::max(0.0, config.backtrack_tolerance);
  const double detour_retreat_allowance =
      std::max(backtrack_tolerance, config.obstacle_detour_retreat_allowance);
  const double detour_minimum_progress = std::min(
      std::max(0.0, config.obstacle_detour_minimum_progress),
      std::max(0.02, 0.25 * target_distance));
  const double detour_maximum_length_ratio =
      std::max(1.0, config.obstacle_detour_maximum_length_ratio);

  Eigen::Vector2d initial_displacement = Eigen::Vector2d::Zero();
  Eigen::Vector2d previous = start;
  double travelled = 0.0;
  double best_target_progress = 0.0;
  double best_reference_progress = 0.0;

  for (std::size_t index = 1; index < path.size(); ++index)
  {
    if (!path[index].allFinite())
      return false;
    const Eigen::Vector2d point = path[index].head<2>();
    travelled += (point - previous).norm();
    previous = point;

    const Eigen::Vector2d displacement = point - start;
    if (initial_displacement.norm() < probe_distance &&
        displacement.norm() >= probe_distance)
      initial_displacement = displacement;

    if (travelled <= guard_distance + 1e-9)
    {
      const double target_progress = displacement.dot(target);
      const double reference_progress = displacement.dot(reference);
      if (allow_bounded_obstacle_detour)
      {
        // A forward-driven bypass may initially move sideways or briefly
        // away from the goal after the body turns. Bound that spatial
        // retreat instead of confusing it with reverse wheel/body motion.
        if (target_progress < -detour_retreat_allowance ||
            reference_progress < -detour_retreat_allowance)
          return false;
      }
      else
      {
        if (target_progress < best_target_progress - backtrack_tolerance ||
            reference_progress <
                best_reference_progress - backtrack_tolerance)
          return false;
      }
      best_target_progress =
          std::max(best_target_progress, target_progress);
      best_reference_progress =
          std::max(best_reference_progress, reference_progress);
    }
  }

  if (initial_displacement.norm() < probe_distance)
    initial_displacement = path.back().head<2>() - start;
  if (initial_displacement.norm() <= 1e-6)
    return false;

  if (allow_bounded_obstacle_detour)
  {
    const Eigen::Vector2d final_displacement =
        path.back().head<2>() - start;
    const double direct_distance = final_displacement.norm();
    if (direct_distance <= 1e-6 ||
        final_displacement.dot(target) < detour_minimum_progress ||
        final_displacement.dot(reference) < detour_minimum_progress ||
        travelled >
            detour_maximum_length_ratio * direct_distance + 1e-9)
      return false;
    return true;
  }

  // A guided candidate already inherits the verified reference polyline.
  // Its monotonic target/reference progress checks above must still reject
  // backtracking and loops, but an obstacle bypass may legitimately begin
  // nearly sideways. Keep the narrower heading cone for unguided fallback
  // seeds, where no geometric guide exists.
  if (!enforce_initial_reference_cone)
    return true;

  const Eigen::Vector2d initial_direction =
      initial_displacement.normalized();
  if (initial_direction.dot(target) <= 0.0)
    return false;

  const double reference_error = std::abs(std::atan2(
      reference.x() * initial_direction.y() -
          reference.y() * initial_direction.x(),
      reference.dot(initial_direction)));
  return reference_error <= max_reference_error + 1e-12;
}

/**
 * Decide whether a forward-only B2 recovery leg may hand off after its
 * nominal trajectory duration.
 *
 * A walking controller can overshoot a short zero-velocity waypoint by a few
 * centimetres.  Once the target lies behind the body, a forward-only
 * controller cannot remove that residual error: waiting for a tighter radius
 * deadlocks the recovery route.  Accept only a bounded overshoot; an endpoint
 * that is still in front must continue converging to the precise tolerance.
 */
inline bool shouldAdvanceB2RecoveryLeg(
    const Eigen::Vector2d &target_residual,
    double body_yaw,
    double precise_tolerance = 0.03,
    double bounded_overshoot_tolerance = 0.105)
{
  if (!target_residual.allFinite() || !std::isfinite(body_yaw))
    return false;

  const double precise =
      std::max(0.0, precise_tolerance);
  const double bounded =
      std::max(precise, bounded_overshoot_tolerance);
  const double error = target_residual.norm();
  if (error <= precise)
    return true;
  if (error > bounded)
    return false;

  const Eigen::Vector2d body_forward(
      std::cos(body_yaw), std::sin(body_yaw));
  return target_residual.dot(body_forward) <= 0.0;
}

}  // namespace scan_planner

#endif  // SCAN_PLANNER_B2_MOTION_POLICY_H
