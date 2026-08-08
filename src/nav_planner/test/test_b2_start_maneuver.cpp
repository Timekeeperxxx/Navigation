#include <gtest/gtest.h>

#include <global_planner/b2_start_maneuver.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace
{

using global_planner::B2StartManeuverConfig;
using global_planner::B2StartManeuverPoint;
using global_planner::B2StartManeuverStatus;

constexpr double kPi = 3.14159265358979323846;
constexpr double kStartYaw = 45.0 * kPi / 180.0;
constexpr double kPathYaw = -95.0 * kPi / 180.0;

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

std::vector<B2StartManeuverPoint> boundaryPath()
{
  const B2StartManeuverPoint start{0.80, 0.0, 0.32};
  return {
      start,
      pointAlong(start, kPathYaw, 1.0),
      pointAlong(start, kPathYaw, 2.0)};
}

bool halfPlaneSupportsB2(
    const B2StartManeuverPoint & point,
    double yaw)
{
  // Ground occupies x >= 0.  The B2 circle centres are 0.22 m and
  // 0.63 m behind base_footprint.  A 0.32 m effective radius represents the
  // physical 0.27 m circle plus the configured 0.05 m tracking margin.
  const double cos_yaw = std::cos(yaw);
  const double front_circle_x = point.x - 0.22 * cos_yaw;
  const double rear_circle_x = point.x - 0.63 * cos_yaw;
  const double footprint_min_x =
      std::min(front_circle_x, rear_circle_x) - 0.32;
  return footprint_min_x >= -1e-12;
}

B2StartManeuverConfig testConfig()
{
  B2StartManeuverConfig config;
  config.maximum_forward_distance = 1.0;
  config.forward_step = 0.10;
  config.path_sample_step = 0.05;
  config.yaw_sample_step = 5.0 * kPi / 180.0;
  config.maximum_join_distance = 5.0;
  return config;
}

TEST(B2StartManeuver, KeepsPathUnchangedWhenInitialTurnIsDirectlySafe)
{
  const std::vector<B2StartManeuverPoint> path{
      {0.0, 0.0, 0.32},
      {1.0, 0.0, 0.32},
      {2.0, 0.0, 0.32}};
  std::size_t ground_height_queries = 0;

  const auto result = global_planner::makeB2StartManeuver(
      path,
      0.0,
      testConfig(),
      [](const B2StartManeuverPoint &, double) {
        return true;
      },
      [&ground_height_queries](
          double, double, double previous_z, double & ground_z) {
        ++ground_height_queries;
        ground_z = previous_z;
        return true;
      });

  EXPECT_EQ(result.status, B2StartManeuverStatus::DIRECT_SAFE);
  EXPECT_DOUBLE_EQ(result.forward_distance, 0.0);
  ASSERT_EQ(result.path.size(), path.size());
  for (std::size_t index = 0; index < path.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(result.path[index].x, path[index].x);
    EXPECT_DOUBLE_EQ(result.path[index].y, path[index].y);
    EXPECT_DOUBLE_EQ(result.path[index].z, path[index].z);
  }
  EXPECT_EQ(ground_height_queries, 0U);
}

TEST(B2StartManeuver, RepairsUnsafeContinuousTurnWithForwardOnlyEscape)
{
  const auto path = boundaryPath();
  const double first_path_yaw = std::atan2(
      path[1].y - path[0].y,
      path[1].x - path[0].x);

  // The live and outgoing endpoint attitudes are individually supported, but
  // their shortest continuous sweep crosses yaw=0 where the rear B2 circle
  // hangs over the ground edge.
  ASSERT_TRUE(halfPlaneSupportsB2(path.front(), kStartYaw));
  ASSERT_TRUE(halfPlaneSupportsB2(path.front(), first_path_yaw));
  ASSERT_FALSE(global_planner::isB2StartTurnSupported(
      path.front(),
      kStartYaw,
      first_path_yaw,
      testConfig().yaw_sample_step,
      halfPlaneSupportsB2));

  const auto result = global_planner::makeB2StartManeuver(
      path,
      kStartYaw,
      testConfig(),
      halfPlaneSupportsB2,
      [](double, double, double previous_z, double & ground_z) {
        ground_z = previous_z;
        return true;
      });

  ASSERT_EQ(result.status, B2StartManeuverStatus::REPAIRED);
  EXPECT_NEAR(result.forward_distance, 0.30, 1e-12);
  EXPECT_GE(result.join_index, 1U);
  ASSERT_GT(result.path.size(), path.size());

  const double heading_x = std::cos(kStartYaw);
  const double heading_y = std::sin(kStartYaw);
  const std::size_t forward_point_count =
      static_cast<std::size_t>(
          std::llround(
              result.forward_distance / testConfig().forward_step));
  ASSERT_GE(result.path.size(), forward_point_count + 1U);
  for (std::size_t index = 1; index <= forward_point_count; ++index)
  {
    const double dx = result.path[index].x - path.front().x;
    const double dy = result.path[index].y - path.front().y;
    const double forward = dx * heading_x + dy * heading_y;
    const double lateral = -dx * heading_y + dy * heading_x;
    EXPECT_NEAR(
        forward,
        static_cast<double>(index) * testConfig().forward_step,
        1e-12);
    EXPECT_NEAR(lateral, 0.0, 1e-12);
    EXPECT_GT(forward, 0.0);
  }

  // Revalidate the returned polyline as a sequence of forward translations
  // and continuous in-place turns. No unsafe intermediate yaw may be hidden
  // by checking only segment endpoint attitudes.
  double incoming_yaw = kStartYaw;
  for (std::size_t index = 0; index + 1 < result.path.size(); ++index)
  {
    if (global_planner::b2StartHorizontalDistance(
          result.path[index], result.path[index + 1]) <= 1e-6)
    {
      continue;
    }
    const double segment_yaw = std::atan2(
        result.path[index + 1].y - result.path[index].y,
        result.path[index + 1].x - result.path[index].x);
    EXPECT_TRUE(global_planner::isB2StartTurnSupported(
        result.path[index],
        incoming_yaw,
        segment_yaw,
        testConfig().yaw_sample_step,
        halfPlaneSupportsB2));
    EXPECT_TRUE(global_planner::appendSupportedB2StartSegment(
        result.path[index],
        result.path[index + 1],
        testConfig().path_sample_step,
        halfPlaneSupportsB2));
    incoming_yaw = segment_yaw;
  }
}

TEST(B2StartManeuver, ReportsBlockedWithoutTryingReverseEscape)
{
  const auto path = boundaryPath();
  const double heading_x = std::cos(kStartYaw);
  const double heading_y = std::sin(kStartYaw);
  std::vector<B2StartManeuverPoint> off_start_queries;

  const auto pose_supported =
      [&path, heading_x, heading_y, &off_start_queries](
          const B2StartManeuverPoint & point, double yaw) {
        const double dx = point.x - path.front().x;
        const double dy = point.y - path.front().y;
        const double distance = std::hypot(dx, dy);
        if (distance > 1e-6)
        {
          off_start_queries.push_back(point);
          const double forward = dx * heading_x + dy * heading_y;
          const double lateral = -dx * heading_y + dy * heading_x;
          if (forward > 0.05 && std::abs(lateral) <= 1e-9)
            return false;
        }
        return halfPlaneSupportsB2(point, yaw);
      };

  const auto result = global_planner::makeB2StartManeuver(
      path,
      kStartYaw,
      testConfig(),
      pose_supported,
      [](double, double, double previous_z, double & ground_z) {
        ground_z = previous_z;
        return true;
      });

  EXPECT_EQ(result.status, B2StartManeuverStatus::BLOCKED);
  EXPECT_TRUE(result.path.empty());
  EXPECT_DOUBLE_EQ(result.forward_distance, 0.0);
  ASSERT_FALSE(off_start_queries.empty());
  for (const auto & query : off_start_queries)
  {
    const double dx = query.x - path.front().x;
    const double dy = query.y - path.front().y;
    const double forward = dx * heading_x + dy * heading_y;
    const double lateral = -dx * heading_y + dy * heading_x;
    EXPECT_GT(forward, 0.0);
    EXPECT_NEAR(lateral, 0.0, 1e-9);
  }
}

}  // namespace
