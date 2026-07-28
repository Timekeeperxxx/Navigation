#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "scan_planner/ground_z_tracker.h"

namespace
{

scan_planner::GroundZTracker makeTracker()
{
  scan_planner::GroundZTrackerConfig config;
  config.bucket_size = 0.20;
  config.xy_tolerance = 0.15;
  config.maximum_layer_distance = 0.50;
  config.max_z_rate = 0.30;
  return scan_planner::GroundZTracker(config);
}

TEST(GroundZTracker, ChoosesVerticallyContinuousOverlappingFloor)
{
  auto tracker = makeTracker();
  ASSERT_TRUE(tracker.setGroundPoints(
      {
          {12.37, -0.86, -11.34},
          {12.42, -0.91, -0.36},
      }));

  const auto upper = tracker.project(12.40, -0.89, -0.57);
  ASSERT_TRUE(upper.valid);
  EXPECT_NEAR(upper.target_base_z, -0.36, 1e-9);

  const auto lower = tracker.project(12.40, -0.89, -11.30);
  ASSERT_TRUE(lower.valid);
  EXPECT_NEAR(lower.target_base_z, -11.34, 1e-9);
}

TEST(GroundZTracker, CorrectsStationaryHeightAtBoundedRate)
{
  auto tracker = makeTracker();
  ASSERT_TRUE(tracker.setGroundPoints({{12.40, -0.89, -0.36}}));

  double z = -0.57;
  for (int index = 0; index < 100; ++index)
    z = tracker.update(12.40, -0.89, z, 0.01);

  EXPECT_NEAR(z, -0.36, 1e-9);
}

TEST(GroundZTracker, MissingSupportOrOtherLayerHoldsHeight)
{
  auto tracker = makeTracker();
  ASSERT_TRUE(tracker.setGroundPoints(
      {
          {0.0, 0.0, -11.0},
          {2.0, 0.0, 0.0},
      }));

  EXPECT_DOUBLE_EQ(tracker.update(0.0, 0.0, 0.0, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(tracker.update(1.0, 0.0, 0.0, 0.1), 0.0);
}

TEST(GroundZTracker, IgnoresNonFinitePoints)
{
  auto tracker = makeTracker();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(tracker.setGroundPoints({{nan, 0.0, 0.0}}));
  EXPECT_FALSE(tracker.ready());
}

}  // namespace
