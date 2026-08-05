#include <gtest/gtest.h>

#include "plan_manage/b2_yaw_control.h"

namespace scan_planner
{
namespace
{

TEST(B2YawRateEstimatorTest, MeasuresAcrossAngleWrap)
{
  B2YawRateEstimator estimator;
  estimator.configure(0.0, 2.0);

  EXPECT_DOUBLE_EQ(estimator.update(3.13, 0.0), 0.0);
  const double rate = estimator.update(-3.13, 0.1);

  EXPECT_NEAR(rate, normalizeB2ControlAngle(-6.26) / 0.1, 1e-9);
  EXPECT_LT(std::abs(rate), 0.3);
}

TEST(B2DesiredYawFilterTest, DoesNotJumpAtAngleWrap)
{
  B2DesiredYawFilter filter;
  filter.configure(0.0);

  const double first = filter.update(3.13, 0.0);
  const double second = filter.update(-3.13, 0.1);

  EXPECT_NEAR(first, 3.13, 1e-12);
  EXPECT_LT(std::abs(second - first), 0.03);
  EXPECT_GT(second, M_PI);
}

TEST(B2PathYawControlTest, WaitsForYawRateToSettleBeforeWalking)
{
  B2PathYawControl control;
  control.configure(0.50, 0.35, 1.0, 0.06, 0.35);

  const auto enter = control.update(0.60, 0.0, 0.0, 1.5, 0.30);
  ASSERT_TRUE(enter.alignment_active);
  EXPECT_NEAR(enter.command, 0.30, 1e-12);

  const auto still_rotating =
      control.update(0.30, 0.20, 0.1, 1.5, 0.30);
  EXPECT_TRUE(still_rotating.alignment_active);

  const auto settled = control.update(0.30, 0.0, 0.2, 1.5, 0.30);
  EXPECT_FALSE(settled.alignment_active);
}

TEST(B2PathYawControlTest, UsesPredictionToReduceCommandBeforeOvershoot)
{
  B2PathYawControl control;
  control.configure(0.50, 0.35, 1.0, 0.06, 0.35);

  const auto result = control.update(0.40, 0.20, 0.0, 1.5, 0.30);

  EXPECT_NEAR(result.predicted_error, 0.20, 1e-12);
  EXPECT_NEAR(result.command, 0.30, 1e-12);
}

TEST(B2PathYawControlTest, InsertsNeutralPhaseBeforeCommandReversal)
{
  B2PathYawControl control;
  control.configure(0.50, 0.35, 1.0, 0.06, 0.35);

  const auto left = control.update(0.80, 0.0, 0.0, 1.5, 0.30);
  ASSERT_GT(left.command, 0.0);

  const auto reversal = control.update(-0.20, 0.0, 0.1, 1.5, 0.30);
  EXPECT_TRUE(reversal.reversal_braking);
  EXPECT_DOUBLE_EQ(reversal.command, 0.0);

  const auto neutral = control.update(-0.20, -0.10, 0.3, 1.5, 0.30);
  EXPECT_TRUE(neutral.reversal_braking);
  EXPECT_DOUBLE_EQ(neutral.command, 0.0);

  const auto reversed = control.update(-0.20, 0.0, 0.5, 1.5, 0.30);
  EXPECT_FALSE(reversed.reversal_braking);
  EXPECT_LT(reversed.command, 0.0);
}

TEST(B2PathYawControlTest, BrakesOppositeMeasuredMotionAfterReset)
{
  B2PathYawControl control;
  control.configure(0.50, 0.35, 1.0, 0.06, 0.35);

  const auto result = control.update(-0.40, 0.20, 0.0, 1.5, 0.30);

  EXPECT_TRUE(result.reversal_braking);
  EXPECT_DOUBLE_EQ(result.command, 0.0);
}

TEST(B2PathYawControlTest, ZeroAtBrakeReleaseDoesNotRestartNeutralPhase)
{
  B2PathYawControl control;
  control.configure(0.50, 0.35, 1.0, 0.06, 0.35);

  ASSERT_GT(control.update(0.80, 0.0, 0.0, 1.5, 0.30).command, 0.0);
  ASSERT_TRUE(
      control.update(-0.20, 0.0, 0.1, 1.5, 0.30).reversal_braking);

  const auto zero_release = control.update(0.0, 0.0, 0.5, 1.5, 0.30);
  EXPECT_FALSE(zero_release.reversal_braking);
  EXPECT_DOUBLE_EQ(zero_release.command, 0.0);

  const auto next_reverse = control.update(-0.20, 0.0, 0.51, 1.5, 0.30);
  EXPECT_FALSE(next_reverse.reversal_braking);
  EXPECT_LT(next_reverse.command, 0.0);
}

TEST(B2FinalPositionControlTest, CorrectsDriftInBodyFrame)
{
  const auto command = computeB2FinalPositionCommand(
      0.30, 0.10, 0.0, 0.12, 0.8, 0.10, 0.06, true);

  EXPECT_FALSE(command.aligned);
  EXPECT_NEAR(command.distance, std::hypot(0.30, 0.10), 1e-12);
  EXPECT_NEAR(command.body_x, 0.10, 1e-12);
  EXPECT_NEAR(command.body_y, 0.06, 1e-12);
}

TEST(B2FinalPositionControlTest, AllowsBoundedReverseAtTerminalPose)
{
  const auto command = computeB2FinalPositionCommand(
      -0.20, 0.0, 0.0, 0.12, 1.0, 0.08, 0.06, true);

  EXPECT_FALSE(command.aligned);
  EXPECT_NEAR(command.body_x, -0.08, 1e-12);
  EXPECT_DOUBLE_EQ(command.body_y, 0.0);
}

TEST(B2FinalPositionControlTest, StopsInsideTolerance)
{
  const auto command = computeB2FinalPositionCommand(
      0.05, -0.04, 1.0, 0.12, 1.0, 0.10, 0.06, true);

  EXPECT_TRUE(command.aligned);
  EXPECT_DOUBLE_EQ(command.body_x, 0.0);
  EXPECT_DOUBLE_EQ(command.body_y, 0.0);
}

TEST(B2FinalPoseTest, RequiresPositionAndYawAtTheSameTime)
{
  EXPECT_FALSE(isB2FinalPoseReached(0.18, 0.10, 0.12, 0.20, true));
  EXPECT_FALSE(isB2FinalPoseReached(0.08, 0.30, 0.12, 0.20, true));
  EXPECT_TRUE(isB2FinalPoseReached(0.08, 0.10, 0.12, 0.20, true));
}

}  // namespace
}  // namespace scan_planner
