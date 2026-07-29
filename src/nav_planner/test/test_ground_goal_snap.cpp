#include "global_planner/ground_goal_snap.h"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace
{

struct TestPoint
{
  double x;
  double y;
  double z;
};

TEST(GroundGoalSnap, RvizZeroDoesNotOverrideTheRobotFloorHint)
{
  const std::vector<TestPoint> ground{
    {4.06, -3.86, -2.25},
    {4.07, -3.86, 0.72},
    {4.03, -3.82, 0.746},
  };

  const auto result = global_planner::findGroundGoalSnap(
    ground, 4.064, -3.864, 0.746);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.index, 2U);
  EXPECT_DOUBLE_EQ(result.snapped_z, 0.746);
}

TEST(GroundGoalSnap, ChoosesThePreferredLayerAmongNearbyXYSamples)
{
  const std::vector<TestPoint> ground{
    {1.00, 2.00, 3.20},
    {1.08, 2.00, 0.75},
  };

  const auto result = global_planner::findGroundGoalSnap(
    ground, 1.00, 2.00, 0.74);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.index, 1U);
  EXPECT_DOUBLE_EQ(result.snapped_z, 0.75);
}

TEST(GroundGoalSnap, RejectsClicksOutsideTheGroundXYRadius)
{
  const std::vector<TestPoint> ground{{1.0, 1.0, 0.7}};

  const auto result = global_planner::findGroundGoalSnap(
    ground, 2.0, 2.0, 0.7);

  EXPECT_FALSE(result.valid);
}

TEST(GroundGoalSnap, RejectsASeparateFloorBeyondTheLayerLimit)
{
  const std::vector<TestPoint> ground{{1.0, 1.0, -3.0}};

  global_planner::GroundGoalSnapConfig config;
  config.maximum_layer_distance = 0.75;
  const auto result = global_planner::findGroundGoalSnap(
    ground, 1.0, 1.0, 0.7, config);

  EXPECT_FALSE(result.valid);
}

TEST(GroundGoalSnap, IgnoresNonFiniteCloudPoints)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<TestPoint> ground{
    {nan, 0.0, 0.7},
    {0.0, 0.0, nan},
    {0.02, 0.01, 0.72},
  };

  const auto result = global_planner::findGroundGoalSnap(
    ground, 0.0, 0.0, 0.7);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.index, 2U);
}

}  // namespace
