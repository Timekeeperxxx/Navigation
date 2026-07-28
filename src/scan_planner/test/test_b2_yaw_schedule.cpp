#include <gtest/gtest.h>

#include <limits>

#include "plan_manage/b2_yaw_schedule.h"

namespace scan_planner
{
namespace
{

TEST(B2YawScheduleTest, StartsAtLiveBodyYaw)
{
  const std::vector<Eigen::Vector3d> path{
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0}};

  const auto yaws = makeB2YawSchedule(path, 0.4);

  ASSERT_EQ(yaws.size(), path.size());
  EXPECT_NEAR(yaws.front(), 0.4, 1e-12);
  EXPECT_NEAR(yaws.back(), 0.0, 1e-12);
}

TEST(B2YawScheduleTest, BisectsCornerWithoutAngleWrapJump)
{
  const std::vector<Eigen::Vector3d> path{
      {0.0, 0.0, 0.0},
      {-1.0, 0.05, 0.0},
      {-2.0, -0.05, 0.0}};

  const auto yaws = makeB2YawSchedule(path, 3.05);

  ASSERT_EQ(yaws.size(), path.size());
  for (std::size_t index = 1; index < yaws.size(); ++index)
    EXPECT_LT(std::abs(yaws[index] - yaws[index - 1]), M_PI);
}

TEST(B2YawScheduleTest, InterpolatesOnUniformTrajectoryTime)
{
  const std::vector<double> yaws{0.2, 0.6, 1.0};

  EXPECT_NEAR(interpolateB2YawSchedule(yaws, 2.0, 0.0), 0.2, 1e-12);
  EXPECT_NEAR(interpolateB2YawSchedule(yaws, 2.0, 0.5), 0.4, 1e-12);
  EXPECT_NEAR(interpolateB2YawSchedule(yaws, 2.0, 2.0), 1.0, 1e-12);
}

TEST(B2YawScheduleTest, ComputesPiecewiseAngularRateFeedForward)
{
  const std::vector<double> yaws{0.2, 0.6, 1.4};

  EXPECT_NEAR(b2YawScheduleRate(yaws, 2.0, 0.25), 0.4, 1e-12);
  EXPECT_NEAR(b2YawScheduleRate(yaws, 2.0, 1.25), 0.8, 1e-12);
  EXPECT_NEAR(b2YawScheduleRate(yaws, 2.0, 2.0), 0.8, 1e-12);
  EXPECT_DOUBLE_EQ(b2YawScheduleRate({0.2}, 2.0, 0.5), 0.0);
}

TEST(B2YawScheduleTest, UsesBoundedUniformSampleCount)
{
  EXPECT_EQ(b2YawSampleIntervalCount(1.0, 0.25), 4U);
  EXPECT_EQ(b2YawSampleIntervalCount(1.01, 0.25), 5U);
  EXPECT_EQ(b2YawSampleIntervalCount(0.0, 0.25), 0U);
}

TEST(B2YawScheduleTest, InPlaceSweepUsesShortestBoundedRotation)
{
  const double maximum_step = 5.0 * M_PI / 180.0;
  const auto yaws = makeB2InPlaceYawSchedule(
      170.0 * M_PI / 180.0,
      -170.0 * M_PI / 180.0,
      maximum_step);

  ASSERT_EQ(yaws.size(), 5U);
  EXPECT_NEAR(yaws.front(), 170.0 * M_PI / 180.0, 1e-12);
  EXPECT_NEAR(yaws.back(), 190.0 * M_PI / 180.0, 1e-12);
  for (std::size_t index = 1; index < yaws.size(); ++index)
  {
    EXPECT_GT(yaws[index] - yaws[index - 1], 0.0);
    EXPECT_LE(
        std::abs(yaws[index] - yaws[index - 1]),
        maximum_step + 1e-12);
  }
}

TEST(B2YawScheduleTest, RejectsInvalidInPlaceSweepParameters)
{
  EXPECT_TRUE(makeB2InPlaceYawSchedule(0.0, 1.0, 0.0).empty());
  EXPECT_TRUE(
      makeB2InPlaceYawSchedule(
          std::numeric_limits<double>::quiet_NaN(), 1.0, 0.1)
          .empty());
}

}  // namespace
}  // namespace scan_planner
