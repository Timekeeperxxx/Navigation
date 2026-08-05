#include <cmath>

#include <gtest/gtest.h>

#include "plan_env/robot_self_filter.h"

namespace
{

bool inside(double x, double y, double z)
{
  return plan_env::insideRobotSelfMask(
      x, y, z,
      1.0, 2.0, 0.0,
      -0.15, 0.95,
      0.27, 0.205, 0.0, 0.0);
}

TEST(RobotSelfFilterTest, CoversBodyAndBothB2FootprintCircles)
{
  EXPECT_TRUE(inside(1.0, 2.0, 0.40));
  EXPECT_TRUE(inside(1.205, 2.0, 0.40));
  EXPECT_TRUE(inside(0.795, 2.0, 0.40));
}

TEST(RobotSelfFilterTest, PreservesNearbyExternalObstacle)
{
  EXPECT_FALSE(inside(1.50, 2.0, 0.40));
}

TEST(RobotSelfFilterTest, PreservesPointsOutsideRobotHeight)
{
  EXPECT_FALSE(inside(1.0, 2.0, 1.10));
  EXPECT_FALSE(inside(1.0, 2.0, -0.30));
}

TEST(RobotSelfFilterTest, RotatesWithBodyYaw)
{
  EXPECT_TRUE(plan_env::insideRobotSelfMask(
      1.0, 2.205, 0.40,
      1.0, 2.0, M_PI_2,
      -0.15, 0.95,
      0.27, 0.205, 0.0, 0.0));
}

}  // namespace
