#include <gtest/gtest.h>

#include <global_planner/b2_global_path_gate.h>

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace
{

using global_planner::B2GlobalPathGateConfig;
using global_planner::B2GlobalPathGateStatus;
using global_planner::B2StartManeuverPoint;

constexpr double kPi = 3.14159265358979323846;
constexpr double kStartYaw = 45.0 * kPi / 180.0;
constexpr double kBoundaryPathYaw = -95.0 * kPi / 180.0;

double normalizedYaw(double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

B2StartManeuverPoint pointAlong(
    const B2StartManeuverPoint & start,
    double yaw,
    double distance)
{
  return B2StartManeuverPoint{
      start.x + distance * std::cos(yaw),
      start.y + distance * std::sin(yaw),
      start.z};
}

std::vector<B2StartManeuverPoint> straightPath()
{
  return {
      {0.0, 0.0, 0.32},
      {1.0, 0.0, 0.32},
      {2.0, 0.0, 0.32}};
}

std::vector<B2StartManeuverPoint> boundaryPath()
{
  const B2StartManeuverPoint start{0.80, 0.0, 0.32};
  return {
      start,
      pointAlong(start, kBoundaryPathYaw, 1.0),
      pointAlong(start, kBoundaryPathYaw, 2.0)};
}

bool constantGroundHeight(
    double,
    double,
    double previous_z,
    double & ground_z)
{
  ground_z = previous_z;
  return true;
}

bool halfPlaneSupportsB2(
    const B2StartManeuverPoint & point,
    double yaw)
{
  // Ground occupies x >= 0. The two effective B2 support circles are 0.22 m
  // and 0.63 m behind base_footprint, each with a 0.32 m effective radius.
  const double cos_yaw = std::cos(yaw);
  const double front_circle_x = point.x - 0.22 * cos_yaw;
  const double rear_circle_x = point.x - 0.63 * cos_yaw;
  return std::min(front_circle_x, rear_circle_x) - 0.32 >= -1e-12;
}

B2GlobalPathGateConfig testConfig()
{
  B2GlobalPathGateConfig config;
  config.start_maneuver.maximum_forward_distance = 1.0;
  config.start_maneuver.forward_step = 0.10;
  config.start_maneuver.path_sample_step = 0.05;
  config.start_maneuver.yaw_sample_step = 5.0 * kPi / 180.0;
  config.start_maneuver.maximum_join_distance = 5.0;
  config.endpoint_xy_tolerance = 1e-3;
  config.endpoint_z_tolerance = 1e-3;
  return config;
}

TEST(B2GlobalPathGate, RejectsEndpointTwoMetresFromExactGoal)
{
  const auto candidate = straightPath();
  const B2StartManeuverPoint exact_goal{4.0, 0.0, 0.32};

  const auto result = global_planner::prepareB2GlobalPath(
      candidate,
      0.0,
      exact_goal,
      std::nullopt,
      testConfig(),
      [](const B2StartManeuverPoint &, double) { return true; },
      constantGroundHeight);

  EXPECT_EQ(
      result.status, B2GlobalPathGateStatus::ENDPOINT_MISMATCH);
  EXPECT_TRUE(result.path.empty());
  EXPECT_DOUBLE_EQ(result.forward_escape_distance, 0.0);
}

TEST(B2GlobalPathGate, RejectsUnsupportedSpecifiedGoalYaw)
{
  const auto candidate = straightPath();
  const B2StartManeuverPoint exact_goal = candidate.back();
  constexpr double goal_yaw = kPi / 2.0;
  const auto pose_supported =
      [&exact_goal](const B2StartManeuverPoint & point, double yaw) {
        const bool at_goal =
            global_planner::b2StartHorizontalDistance(
              point, exact_goal) <= 1e-9 &&
            std::abs(point.z - exact_goal.z) <= 1e-9;
        return !at_goal ||
               std::abs(normalizedYaw(yaw - goal_yaw)) > 1e-9;
      };

  ASSERT_FALSE(pose_supported(exact_goal, goal_yaw));
  const auto result = global_planner::prepareB2GlobalPath(
      candidate,
      0.0,
      exact_goal,
      goal_yaw,
      testConfig(),
      pose_supported,
      constantGroundHeight);

  EXPECT_EQ(
      result.status, B2GlobalPathGateStatus::GOAL_POSE_UNSUPPORTED);
  EXPECT_TRUE(result.path.empty());
}

TEST(
  B2GlobalPathGate,
  RejectsUnsupportedTerminalSweepEvenWhenEndpointYawsAreSupported)
{
  const auto candidate = straightPath();
  const B2StartManeuverPoint exact_goal = candidate.back();
  constexpr double arrival_yaw = 0.0;
  constexpr double goal_yaw = kPi;
  constexpr double blocked_intermediate_yaw = kPi / 2.0;
  const auto pose_supported =
      [&exact_goal](const B2StartManeuverPoint & point, double yaw) {
        const bool at_goal =
            global_planner::b2StartHorizontalDistance(
              point, exact_goal) <= 1e-9 &&
            std::abs(point.z - exact_goal.z) <= 1e-9;
        return !at_goal ||
               std::abs(
                 normalizedYaw(yaw - blocked_intermediate_yaw)) >
                   1e-9;
      };

  ASSERT_TRUE(pose_supported(exact_goal, arrival_yaw));
  ASSERT_TRUE(pose_supported(exact_goal, goal_yaw));
  ASSERT_FALSE(pose_supported(exact_goal, blocked_intermediate_yaw));
  const auto result = global_planner::prepareB2GlobalPath(
      candidate,
      arrival_yaw,
      exact_goal,
      goal_yaw,
      testConfig(),
      pose_supported,
      constantGroundHeight);

  EXPECT_EQ(
      result.status,
      B2GlobalPathGateStatus::TERMINAL_SWEEP_UNSUPPORTED);
  EXPECT_TRUE(result.path.empty());
}

TEST(B2GlobalPathGate, RejectsUnsupportedIntermediateTurnSweep)
{
  const std::vector<B2StartManeuverPoint> candidate{
      {0.0, 0.0, 0.32},
      {1.0, 0.0, 0.32},
      {1.0, 1.0, 0.32}};
  const B2StartManeuverPoint exact_goal = candidate.back();
  const auto pose_supported =
      [](const B2StartManeuverPoint & point, double yaw) {
        const bool at_corner =
            std::hypot(point.x - 1.0, point.y) <= 1e-9;
        return !at_corner ||
               std::abs(normalizedYaw(yaw - kPi / 4.0)) > 1e-9;
      };

  const auto result = global_planner::prepareB2GlobalPath(
      candidate,
      0.0,
      exact_goal,
      std::nullopt,
      testConfig(),
      pose_supported,
      constantGroundHeight);

  EXPECT_EQ(
      result.status,
      B2GlobalPathGateStatus::PATH_TURN_UNSUPPORTED);
  EXPECT_TRUE(result.path.empty());
}

TEST(B2GlobalPathGate, AcceptsSupportedPathAndRestoresExactEndpoint)
{
  auto candidate = straightPath();
  const B2StartManeuverPoint exact_goal = candidate.back();
  candidate.back().x -= 5e-4;
  candidate.back().y += 4e-4;
  candidate.back().z -= 3e-4;
  constexpr double goal_yaw = 0.4;

  const auto result = global_planner::prepareB2GlobalPath(
      candidate,
      0.0,
      exact_goal,
      goal_yaw,
      testConfig(),
      [](const B2StartManeuverPoint &, double) { return true; },
      constantGroundHeight);

  ASSERT_EQ(result.status, B2GlobalPathGateStatus::DIRECT_SAFE);
  ASSERT_FALSE(result.path.empty());
  EXPECT_DOUBLE_EQ(result.path.back().x, exact_goal.x);
  EXPECT_DOUBLE_EQ(result.path.back().y, exact_goal.y);
  EXPECT_DOUBLE_EQ(result.path.back().z, exact_goal.z);
}

TEST(
  B2GlobalPathGate,
  RepairsBlockedStartWithForwardOnlyEscapeAndKeepsExactGoal)
{
  const auto candidate = boundaryPath();
  const B2StartManeuverPoint exact_goal = candidate.back();

  ASSERT_FALSE(global_planner::isB2StartTurnSupported(
      candidate.front(),
      kStartYaw,
      kBoundaryPathYaw,
      testConfig().start_maneuver.yaw_sample_step,
      halfPlaneSupportsB2));
  const auto result = global_planner::prepareB2GlobalPath(
      candidate,
      kStartYaw,
      exact_goal,
      std::nullopt,
      testConfig(),
      halfPlaneSupportsB2,
      constantGroundHeight);

  ASSERT_EQ(
      result.status, B2GlobalPathGateStatus::FORWARD_REPAIRED);
  EXPECT_NEAR(result.forward_escape_distance, 0.30, 1e-12);
  ASSERT_FALSE(result.path.empty());
  EXPECT_DOUBLE_EQ(result.path.back().x, exact_goal.x);
  EXPECT_DOUBLE_EQ(result.path.back().y, exact_goal.y);
  EXPECT_DOUBLE_EQ(result.path.back().z, exact_goal.z);

  const double heading_x = std::cos(kStartYaw);
  const double heading_y = std::sin(kStartYaw);
  const std::size_t forward_point_count = static_cast<std::size_t>(
      std::llround(
        result.forward_escape_distance /
        testConfig().start_maneuver.forward_step));
  ASSERT_GE(result.path.size(), forward_point_count + 1U);
  for (std::size_t index = 1; index <= forward_point_count; ++index)
  {
    const double dx = result.path[index].x - candidate.front().x;
    const double dy = result.path[index].y - candidate.front().y;
    const double forward = dx * heading_x + dy * heading_y;
    const double lateral = -dx * heading_y + dy * heading_x;
    EXPECT_GT(forward, 0.0);
    EXPECT_NEAR(
        forward,
        static_cast<double>(index) *
          testConfig().start_maneuver.forward_step,
        1e-12);
    EXPECT_NEAR(lateral, 0.0, 1e-12);
  }
}

TEST(
  B2GlobalPathGate,
  BlocksWithoutForwardCorridorAndNeverProbesLateralOrReverseEscape)
{
  const auto candidate = boundaryPath();
  const B2StartManeuverPoint exact_goal = candidate.back();
  const double heading_x = std::cos(kStartYaw);
  const double heading_y = std::sin(kStartYaw);
  std::vector<B2StartManeuverPoint> off_start_queries;
  const auto pose_supported =
      [&candidate, heading_x, heading_y, &off_start_queries](
          const B2StartManeuverPoint & point, double yaw) {
        const double dx = point.x - candidate.front().x;
        const double dy = point.y - candidate.front().y;
        if (std::hypot(dx, dy) > 1e-6)
        {
          off_start_queries.push_back(point);
          const double forward = dx * heading_x + dy * heading_y;
          const double lateral = -dx * heading_y + dy * heading_x;
          if (forward > 0.05 && std::abs(lateral) <= 1e-9)
            return false;
        }
        return halfPlaneSupportsB2(point, yaw);
      };

  const auto result = global_planner::prepareB2GlobalPath(
      candidate,
      kStartYaw,
      exact_goal,
      std::nullopt,
      testConfig(),
      pose_supported,
      constantGroundHeight);

  EXPECT_EQ(result.status, B2GlobalPathGateStatus::START_BLOCKED);
  EXPECT_TRUE(result.path.empty());
  EXPECT_DOUBLE_EQ(result.forward_escape_distance, 0.0);
  ASSERT_FALSE(off_start_queries.empty());
  for (const auto & point : off_start_queries)
  {
    const double dx = point.x - candidate.front().x;
    const double dy = point.y - candidate.front().y;
    const double forward = dx * heading_x + dy * heading_y;
    const double lateral = -dx * heading_y + dy * heading_x;
    EXPECT_GT(forward, 0.0);
    EXPECT_NEAR(lateral, 0.0, 1e-9);
  }
}

}  // namespace
