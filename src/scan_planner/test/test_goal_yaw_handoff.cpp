#include <limits>

#include <gtest/gtest.h>

#include "plan_manage/goal_yaw_handoff.h"

namespace
{

using scan_planner::GoalYawHandoff;
using scan_planner::GoalYawInputResult;

TEST(GoalYawHandoff, NewGoalAlwaysRequiresANewerEmergencyHandoff)
{
  GoalYawHandoff protocol;

  EXPECT_EQ(
      protocol.receiveYaw(0.7, 10.0),
      GoalYawInputResult::NEW_GOAL);
  EXPECT_TRUE(protocol.awaitingHandoff());
  EXPECT_FALSE(protocol.hasActiveYaw());

  const auto normal_without_handoff =
      protocol.receiveTrajectory(false, 12, 10.2);
  EXPECT_FALSE(normal_without_handoff.yaw_bound);
  EXPECT_TRUE(protocol.awaitingHandoff());

  const auto handoff = protocol.receiveTrajectory(true, 13, 10.3);
  EXPECT_TRUE(handoff.handoff_accepted);
  EXPECT_TRUE(protocol.hasSeenHandoff());

  const auto executable = protocol.receiveTrajectory(false, 14, 10.4);
  EXPECT_TRUE(executable.yaw_bound);
  EXPECT_FALSE(protocol.hasPendingYaw());
  ASSERT_TRUE(protocol.hasActiveYaw());
  EXPECT_NEAR(protocol.activeYaw(), 0.7, 1e-12);
}

TEST(GoalYawHandoff, EmergencyAlreadyActiveBeforeGoalDoesNotWaiveBarrier)
{
  GoalYawHandoff protocol;

  // A stationary spline that predates the callback belongs to the old goal.
  EXPECT_FALSE(
      protocol.receiveTrajectory(true, 5, 5.0).handoff_accepted);
  ASSERT_EQ(
      protocol.receiveYaw(0.2, 6.0),
      GoalYawInputResult::NEW_GOAL);

  EXPECT_FALSE(
      protocol.receiveTrajectory(false, 6, 6.1).yaw_bound);
  EXPECT_TRUE(protocol.awaitingHandoff());
  ASSERT_TRUE(
      protocol.receiveTrajectory(true, 7, 6.2).handoff_accepted);
  EXPECT_TRUE(
      protocol.receiveTrajectory(false, 8, 6.3).yaw_bound);
}

TEST(GoalYawHandoff, EmergencyGeneratedBeforeGoalCannotSatisfyHandoff)
{
  GoalYawHandoff protocol;

  ASSERT_EQ(
      protocol.receiveYaw(-0.4, 20.0),
      GoalYawInputResult::NEW_GOAL);
  const auto stale_handoff =
      protocol.receiveTrajectory(true, 20, 19.5);
  EXPECT_FALSE(stale_handoff.handoff_accepted);
  EXPECT_TRUE(protocol.awaitingHandoff());

  EXPECT_FALSE(
      protocol.receiveTrajectory(false, 21, 20.1).yaw_bound);
  EXPECT_TRUE(
      protocol.receiveTrajectory(true, 22, 20.2).handoff_accepted);
  EXPECT_TRUE(
      protocol.receiveTrajectory(false, 23, 20.3).yaw_bound);
}

TEST(GoalYawHandoff, NormalSplineMustBeNewerThanAcceptedHandoff)
{
  GoalYawHandoff protocol;

  ASSERT_EQ(
      protocol.receiveYaw(1.1, 30.0),
      GoalYawInputResult::NEW_GOAL);
  ASSERT_TRUE(
      protocol.receiveTrajectory(true, 40, 30.1).handoff_accepted);

  EXPECT_FALSE(
      protocol.receiveTrajectory(false, 39, 30.2).yaw_bound);
  EXPECT_FALSE(
      protocol.receiveTrajectory(false, 40, 30.3).yaw_bound);
  EXPECT_TRUE(protocol.hasSeenHandoff());

  EXPECT_TRUE(
      protocol.receiveTrajectory(false, 41, 30.4).yaw_bound);
  EXPECT_NEAR(protocol.activeYaw(), 1.1, 1e-12);
}

TEST(GoalYawHandoff, NewGoalAfterHandoffInvalidatesOldGeneration)
{
  GoalYawHandoff protocol;

  ASSERT_EQ(
      protocol.receiveYaw(0.1, 40.0),
      GoalYawInputResult::NEW_GOAL);
  ASSERT_TRUE(
      protocol.receiveTrajectory(true, 50, 40.1).handoff_accepted);

  ASSERT_EQ(
      protocol.receiveYaw(0.9, 41.0),
      GoalYawInputResult::NEW_GOAL);
  EXPECT_TRUE(protocol.awaitingHandoff());
  EXPECT_FALSE(
      protocol.receiveTrajectory(false, 51, 41.1).yaw_bound);

  ASSERT_TRUE(
      protocol.receiveTrajectory(true, 52, 41.2).handoff_accepted);
  ASSERT_TRUE(
      protocol.receiveTrajectory(false, 53, 41.3).yaw_bound);
  EXPECT_NEAR(protocol.activeYaw(), 0.9, 1e-12);
}

TEST(GoalYawHandoff, DuplicateYawDoesNotEraseObservedHandoff)
{
  GoalYawHandoff protocol;

  ASSERT_EQ(
      protocol.receiveYaw(0.3, 50.0),
      GoalYawInputResult::NEW_GOAL);
  ASSERT_TRUE(
      protocol.receiveTrajectory(true, 60, 50.1).handoff_accepted);

  EXPECT_EQ(
      protocol.receiveYaw(0.3, 50.2),
      GoalYawInputResult::DUPLICATE);
  EXPECT_TRUE(protocol.hasSeenHandoff());
  EXPECT_TRUE(
      protocol.receiveTrajectory(false, 61, 50.3).yaw_bound);
}

TEST(GoalYawHandoff, PoseThenYawCoalesceWithoutSecondHandoff)
{
  GoalYawHandoff protocol;

  ASSERT_EQ(
      protocol.receivePose(1.0, 2.0, 3.0, 0.0, 60.0),
      GoalYawInputResult::NEW_GOAL);
  ASSERT_TRUE(
      protocol.receiveTrajectory(true, 70, 60.1).handoff_accepted);

  EXPECT_EQ(
      protocol.receiveYaw(0.6, 60.2),
      GoalYawInputResult::COALESCED_UPDATE);
  EXPECT_TRUE(protocol.hasSeenHandoff());
  EXPECT_NEAR(protocol.pendingYaw(), 0.6, 1e-12);

  ASSERT_TRUE(
      protocol.receiveTrajectory(false, 71, 60.3).yaw_bound);
  EXPECT_NEAR(protocol.activeYaw(), 0.6, 1e-12);
}

TEST(GoalYawHandoff, YawThenPoseCoalesceBeforeHandoff)
{
  GoalYawHandoff protocol;

  ASSERT_EQ(
      protocol.receiveYaw(-0.2, 70.0),
      GoalYawInputResult::NEW_GOAL);
  EXPECT_EQ(
      protocol.receivePose(4.0, 5.0, 6.0, 0.8, 70.1),
      GoalYawInputResult::COALESCED_UPDATE);
  EXPECT_EQ(protocol.goalGeneration(), 1u);
  EXPECT_NEAR(protocol.pendingYaw(), 0.8, 1e-12);

  ASSERT_TRUE(
      protocol.receiveTrajectory(true, 80, 70.2).handoff_accepted);
  ASSERT_TRUE(
      protocol.receiveTrajectory(false, 81, 70.3).yaw_bound);
  EXPECT_NEAR(protocol.activeYaw(), 0.8, 1e-12);
}

TEST(GoalYawHandoff, DistinctPoseIsANewGoalEvenInsideCoalesceWindow)
{
  GoalYawHandoff protocol;

  ASSERT_EQ(
      protocol.receivePose(1.0, 2.0, 3.0, 0.4, 80.0),
      GoalYawInputResult::NEW_GOAL);
  ASSERT_TRUE(
      protocol.receiveTrajectory(true, 90, 80.1).handoff_accepted);

  EXPECT_EQ(
      protocol.receivePose(1.5, 2.0, 3.0, 0.4, 80.2),
      GoalYawInputResult::NEW_GOAL);
  EXPECT_EQ(protocol.goalGeneration(), 2u);
  EXPECT_TRUE(protocol.awaitingHandoff());
  EXPECT_FALSE(
      protocol.receiveTrajectory(false, 91, 80.3).yaw_bound);
}

TEST(GoalYawHandoff, IdenticalPoseRetransmissionKeepsHandoff)
{
  GoalYawHandoff protocol;

  ASSERT_EQ(
      protocol.receivePose(1.0, 2.0, 3.0, -0.7, 90.0),
      GoalYawInputResult::NEW_GOAL);
  ASSERT_TRUE(
      protocol.receiveTrajectory(true, 100, 90.1).handoff_accepted);

  EXPECT_EQ(
      protocol.receivePose(1.0, 2.0, 3.0, -0.7, 90.2),
      GoalYawInputResult::DUPLICATE);
  EXPECT_TRUE(protocol.hasSeenHandoff());
  EXPECT_TRUE(
      protocol.receiveTrajectory(false, 101, 90.3).yaw_bound);
}

TEST(GoalYawHandoff, IdenticalYawAfterWindowStartsNewGeneration)
{
  GoalYawHandoff protocol;

  ASSERT_EQ(
      protocol.receiveYaw(0.0, 100.0),
      GoalYawInputResult::NEW_GOAL);
  EXPECT_EQ(
      protocol.receiveYaw(0.0, 100.6),
      GoalYawInputResult::NEW_GOAL);
  EXPECT_EQ(protocol.goalGeneration(), 2u);
  EXPECT_TRUE(protocol.awaitingHandoff());
}

TEST(GoalYawHandoff, RejectsNonFiniteGoalWithoutChangingState)
{
  GoalYawHandoff protocol;

  EXPECT_EQ(
      protocol.receiveYaw(
          std::numeric_limits<double>::infinity(), 110.0),
      GoalYawInputResult::REJECTED);
  EXPECT_EQ(
      protocol.receivePose(
          0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 110.1),
      GoalYawInputResult::REJECTED);
  EXPECT_EQ(protocol.goalGeneration(), 0u);
  EXPECT_FALSE(protocol.hasPendingYaw());
  EXPECT_FALSE(protocol.hasActiveYaw());
}

}  // namespace
