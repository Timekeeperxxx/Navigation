#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "scan_planner/terrain_z_tracker.h"

namespace
{

using scan_planner::TerrainPathPoint;
using scan_planner::TerrainZTracker;
using scan_planner::TerrainZTrackerConfig;

TerrainZTracker makeTracker()
{
  TerrainZTrackerConfig config;
  config.body_height = 0.32;
  config.path_timeout = 2.0;
  config.max_path_slope = 0.50;
  config.max_z_rate = 0.20;
  config.max_projection_distance = 0.75;
  return TerrainZTracker(config);
}

TEST(TerrainZTracker, ProjectsXyAndSubtractsBodyHeight)
{
  auto tracker = makeTracker();
  ASSERT_TRUE(tracker.setExecutionPath(
      {{0.0, 0.0, 0.32}, {2.0, 0.0, 0.72}}, 10.0));

  const auto projection = tracker.project(1.0, 0.10, 0.0, 10.5);
  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.xy_distance, 0.10, 1e-9);
  EXPECT_NEAR(projection.target_base_z, 0.20, 1e-9);
}

TEST(TerrainZTracker, RateLimitsHeightChangeInBothDirections)
{
  auto tracker = makeTracker();
  ASSERT_TRUE(tracker.setExecutionPath(
      {{0.0, 0.0, 0.72}, {1.0, 0.0, 0.72}}, 10.0));

  EXPECT_NEAR(tracker.update(0.5, 0.0, 0.0, 10.1, 0.10), 0.02, 1e-9);

  ASSERT_TRUE(tracker.setExecutionPath(
      {{0.0, 0.0, 0.12}, {1.0, 0.0, 0.12}}, 11.0));
  EXPECT_NEAR(tracker.update(0.5, 0.0, 0.4, 11.1, 0.25), 0.35, 1e-9);
}

TEST(TerrainZTracker, EmptyAndStalePathsHoldCurrentHeight)
{
  auto tracker = makeTracker();
  EXPECT_FALSE(tracker.setExecutionPath({}, 10.0));
  EXPECT_DOUBLE_EQ(tracker.update(0.0, 0.0, -0.59, 10.1, 0.1), -0.59);

  ASSERT_TRUE(tracker.setExecutionPath(
      {{0.0, 0.0, 0.32}, {1.0, 0.0, 0.42}}, 10.0));
  EXPECT_DOUBLE_EQ(tracker.update(0.5, 0.0, -0.59, 12.01, 0.1), -0.59);

  tracker.clearPath();
  EXPECT_DOUBLE_EQ(tracker.update(0.5, 0.0, -0.59, 10.5, 0.1), -0.59);
}

TEST(TerrainZTracker, RejectsNonFiniteAndExcessiveSlopePaths)
{
  auto tracker = makeTracker();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(tracker.setExecutionPath(
      {{0.0, 0.0, 0.32}, {1.0, 0.0, nan}}, 10.0));
  EXPECT_FALSE(tracker.setExecutionPath(
      {{0.0, 0.0, 0.32}, {0.10, 0.0, 0.42}}, 10.0));
  EXPECT_FALSE(tracker.setExecutionPath(
      {{0.0, 0.0, 0.32}, {0.0, 0.0, 0.42}}, 10.0));
  EXPECT_FALSE(tracker.setExecutionPath(
      {{0.0, 0.0, 0.32}}, nan));
}

TEST(TerrainZTracker, ProjectionOutsideCorridorHoldsHeight)
{
  auto tracker = makeTracker();
  ASSERT_TRUE(tracker.setExecutionPath(
      {{0.0, 0.0, 0.32}, {1.0, 0.0, 0.42}}, 10.0));

  EXPECT_FALSE(tracker.project(0.5, 0.76, 0.0, 10.1).valid);
  EXPECT_DOUBLE_EQ(tracker.update(0.5, 0.76, -0.20, 10.1, 0.1), -0.20);
}

TEST(TerrainZTracker, EqualXyCandidatesPreferCurrentConnectedHeight)
{
  auto tracker = makeTracker();
  ASSERT_TRUE(tracker.setExecutionPath(
      {
          {0.0, 0.0, 0.32},
          {1.0, 0.0, 0.32},
          {3.0, 1.0, 1.32},
          {1.0, 0.0, 1.32},
          {0.0, 0.0, 1.32},
      },
      10.0));

  const auto lower = tracker.project(0.5, 0.0, 0.0, 10.1);
  ASSERT_TRUE(lower.valid);
  EXPECT_NEAR(lower.target_base_z, 0.0, 1e-9);

  const auto upper = tracker.project(0.5, 0.0, 1.0, 10.1);
  ASSERT_TRUE(upper.valid);
  EXPECT_NEAR(upper.target_base_z, 1.0, 1e-9);
}

TEST(TerrainZTracker, RepeatedLatchedRecoveryPathDoesNotRatchetHeight)
{
  auto tracker = makeTracker();
  constexpr double start_base_z = -0.59;
  constexpr double body_height = 0.32;
  double base_z = start_base_z;

  for (int round = 0; round < 100; ++round)
  {
    const double now = 10.0 + 0.01 * round;
    ASSERT_TRUE(tracker.setExecutionPath(
        {
            {1.0, 2.0, start_base_z + body_height},
            {0.5, 2.0, start_base_z + body_height},
        },
        now));
    base_z = tracker.update(1.0, 2.0, base_z, now, 0.01);
  }

  EXPECT_DOUBLE_EQ(base_z, start_base_z);
}

}  // namespace
