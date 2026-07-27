#include <gtest/gtest.h>

#include "plan_env/robot_self_filter.h"

namespace
{

constexpr double kBodyX = -12.0566;
constexpr double kBodyY = 0.9750;
constexpr double kBodyYaw = -1.6401;

bool inside(double x, double y, double z)
{
  return plan_env::insideRobotSelfMask(
      x, y, z,
      kBodyX, kBodyY, kBodyYaw,
      -0.27, 0.83,
      0.27, 0.205, -0.425, 0.0);
}

TEST(RobotSelfFilterTest, CoversObservedFrontShoulderReturn)
{
  EXPECT_TRUE(inside(-11.825, 0.875, 0.475));
}

TEST(RobotSelfFilterTest, CoversConfiguredRearBodyCylinder)
{
  const double heading_x = std::cos(kBodyYaw);
  const double heading_y = std::sin(kBodyYaw);
  EXPECT_TRUE(inside(
      kBodyX - 0.63 * heading_x,
      kBodyY - 0.63 * heading_y,
      0.30));
}

TEST(RobotSelfFilterTest, PreservesNearbyExternalObstacle)
{
  EXPECT_FALSE(inside(kBodyX + 0.45, kBodyY, 0.30));
}

TEST(RobotSelfFilterTest, PreservesPointAboveRobotAndSensor)
{
  EXPECT_FALSE(inside(kBodyX, kBodyY, 1.10));
}

}  // namespace
