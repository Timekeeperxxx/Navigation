#include <gtest/gtest.h>

#include <plan_manage/b2_reverse_recovery.h>

namespace
{

scan_planner::B2ReverseRecoveryPolicy makePolicy()
{
  scan_planner::B2ReverseRecoveryConfig config;
  config.maximum_distance = 0.50;
  config.maximum_duration = 2.0;
  config.minimum_preflight_duration = 0.20;
  config.safety_approval_timeout = 1.5;
  config.odometry_timeout = 0.50;
  config.maximum_yaw_drift = 0.15;
  scan_planner::B2ReverseRecoveryPolicy policy;
  policy.configure(config);
  return policy;
}

TEST(B2ReverseRecovery, CommandsOnlyStraightReverseAfterSafetyApproval)
{
  auto policy = makePolicy();
  ASSERT_TRUE(policy.begin(10.0, 1.0, 2.0, -0.59, 0.4));

  auto waiting = policy.update(
      10.2, 1.0, 2.0, 0.4, 0.01, true);
  EXPECT_FALSE(waiting.command_reverse);
  EXPECT_FALSE(waiting.terminal);

  auto driving = policy.update(
      10.3, 1.0, 2.0, 0.4, 0.01, false);
  EXPECT_TRUE(driving.command_reverse);
  EXPECT_FALSE(driving.terminal);
  EXPECT_EQ(
      driving.status,
      scan_planner::B2ReverseRecoveryStatus::ACTIVE);
}

TEST(B2ReverseRecoveryTrigger, RejectsGenericAndTransientPlanningFailures)
{
  EXPECT_FALSE(scan_planner::shouldTriggerB2ReverseRecovery(
      1, 2, true, true, true));
  EXPECT_FALSE(scan_planner::shouldTriggerB2ReverseRecovery(
      2, 2, false, false, true));
  EXPECT_FALSE(scan_planner::shouldTriggerB2ReverseRecovery(
      10, 2, false, true, false));
}

TEST(B2ReverseRecoveryTrigger, AcceptsOnlyConfirmedImmediateObstacleTrap)
{
  // Current-footprint occupancy remains authoritative even if a collapsed
  // execution path made the downstream safety Bool stale-false.
  EXPECT_TRUE(scan_planner::shouldTriggerB2ReverseRecovery(
      2, 2, true, false, false));
  EXPECT_TRUE(scan_planner::shouldTriggerB2ReverseRecovery(
      2, 2, false, true, true));
}

TEST(B2ReverseRecoveryTrigger, ContinuesConfirmedTrapAfterProbeClears)
{
  EXPECT_FALSE(scan_planner::shouldContinueLatchedB2ObstacleRecovery(
      false, 10, 2, false, false, true));
  EXPECT_FALSE(scan_planner::shouldContinueLatchedB2ObstacleRecovery(
      true, 1, 2, true, true, true));
  EXPECT_FALSE(scan_planner::shouldContinueLatchedB2ObstacleRecovery(
      true, 2, 2, false, false, false));
  EXPECT_TRUE(scan_planner::shouldContinueLatchedB2ObstacleRecovery(
      true, 2, 2, false, false, true));
  EXPECT_TRUE(scan_planner::shouldContinueLatchedB2ObstacleRecovery(
      true, 2, 2, false, true, false));
  EXPECT_TRUE(scan_planner::shouldContinueLatchedB2ObstacleRecovery(
      true, 2, 2, true, false, false));
}

TEST(
    B2ReverseRecoveryTrigger,
    ConfirmedGuideBlockageIgnoresCollapsedExecutionPathClear)
{
  // A one-point execution path can report safety_execution_frozen=false
  // after the last short detour leg. Once SCAN's full reference-footprint
  // sweep has latched a real obstacle, a blocked forward probe remains
  // sufficient evidence for the existing straight-only reverse preflight.
  EXPECT_TRUE(scan_planner::shouldContinueLatchedB2ObstacleRecovery(
      true, 2, 2, false, true, false));
}

TEST(B2ReverseRecovery, FreshSafetyApprovalStillWaitsForFullPreflight)
{
  auto policy = makePolicy();
  ASSERT_TRUE(policy.begin(10.0, 1.0, 2.0, -0.59, 0.4));

  const auto too_early = policy.update(
      10.1, 1.0, 2.0, 0.4, 0.01, false);
  EXPECT_FALSE(too_early.command_reverse);
  EXPECT_FALSE(too_early.terminal);

  const auto approved = policy.update(
      10.21, 1.0, 2.0, 0.4, 0.01, false);
  EXPECT_TRUE(approved.command_reverse);
  EXPECT_FALSE(approved.terminal);
}

TEST(B2ReverseRecovery, StopsCurrentRoundAtHalfMetre)
{
  auto policy = makePolicy();
  ASSERT_TRUE(policy.begin(0.0, 0.0, 0.0, -0.59, 0.0));
  ASSERT_TRUE(policy.update(
      0.21, 0.0, 0.0, 0.0, 0.0, false).command_reverse);

  const auto result = policy.update(
      1.0, -0.50, 0.0, 0.0, 0.0, false);
  EXPECT_TRUE(result.terminal);
  EXPECT_FALSE(result.command_reverse);
  EXPECT_EQ(
      result.status,
      scan_planner::B2ReverseRecoveryStatus::DISTANCE_COMPLETE);
  EXPECT_TRUE(policy.holding());
}

TEST(B2ReverseRecovery, StopsCurrentRoundAtTwoSeconds)
{
  auto policy = makePolicy();
  ASSERT_TRUE(policy.begin(0.0, 0.0, 0.0, -0.59, 0.0));
  ASSERT_TRUE(policy.update(
      0.21, 0.0, 0.0, 0.0, 0.0, false).command_reverse);

  const auto result = policy.update(
      2.22, -0.30, 0.0, 0.0, 0.0, false);
  EXPECT_TRUE(result.terminal);
  EXPECT_EQ(
      result.status,
      scan_planner::B2ReverseRecoveryStatus::TIME_COMPLETE);
}

TEST(B2ReverseRecovery, RearSafetyBlockNeverCommandsMotion)
{
  auto policy = makePolicy();
  ASSERT_TRUE(policy.begin(5.0, 0.0, 0.0, -0.59, 0.0));

  const auto result = policy.update(
      6.5, 0.0, 0.0, 0.0, 0.0, true);
  EXPECT_TRUE(result.terminal);
  EXPECT_FALSE(result.command_reverse);
  EXPECT_EQ(
      result.status,
      scan_planner::B2ReverseRecoveryStatus::SAFETY_BLOCKED);
}

TEST(B2ReverseRecovery, OdomLossAndYawDriftFailClosed)
{
  auto odom_policy = makePolicy();
  ASSERT_TRUE(odom_policy.begin(0.0, 0.0, 0.0, -0.59, 0.0));
  const auto stale = odom_policy.update(
      0.1, 0.0, 0.0, 0.0, 0.51, false);
  EXPECT_TRUE(stale.terminal);
  EXPECT_EQ(
      stale.status,
      scan_planner::B2ReverseRecoveryStatus::ODOMETRY_INVALID);

  auto yaw_policy = makePolicy();
  ASSERT_TRUE(yaw_policy.begin(0.0, 0.0, 0.0, -0.59, 0.0));
  const auto drifted = yaw_policy.update(
      0.1, 0.0, 0.0, 0.16, 0.0, false);
  EXPECT_TRUE(drifted.terminal);
  EXPECT_EQ(
      drifted.status,
      scan_planner::B2ReverseRecoveryStatus::YAW_DRIFT);
}

TEST(B2ReverseRecovery, HoldingStateCanStartAnotherUnlimitedRound)
{
  auto policy = makePolicy();
  ASSERT_TRUE(policy.begin(0.0, 0.0, 0.0, -0.59, 0.0));
  ASSERT_TRUE(policy.update(
      0.21, 0.0, 0.0, 0.0, 0.0, false).command_reverse);
  ASSERT_TRUE(policy.update(
      2.22, -0.30, 0.0, 0.0, 0.0, false).terminal);
  ASSERT_TRUE(policy.holding());

  ASSERT_TRUE(policy.begin(3.0, -0.30, 0.0, -0.59, 0.0));
  const auto next_round = policy.update(
      3.21, -0.30, 0.0, 0.0, 0.0, false);
  EXPECT_TRUE(next_round.command_reverse);
  EXPECT_FALSE(next_round.terminal);
}

TEST(B2ReverseRecovery, RecoveryPathHeightUsesLatchedRoundStart)
{
  auto policy = makePolicy();
  ASSERT_TRUE(policy.begin(10.0, 1.0, 2.0, -0.59, 0.4));
  EXPECT_DOUBLE_EQ(policy.startZ(), -0.59);

  // Normal policy updates receive only live planar odometry. The stored
  // height must remain the immutable reference for every path publication.
  ASSERT_FALSE(policy.update(
      10.2, 1.0, 2.0, 0.4, 0.01, true).terminal);
  EXPECT_DOUBLE_EQ(policy.startZ(), -0.59);
}

}  // namespace
