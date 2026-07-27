#include <gtest/gtest.h>

#include <plan_manage/replan_failure_policy.h>

TEST(ReplanFailurePolicy, FrozenExecutionUsesRetryDeadline)
{
  EXPECT_TRUE(scan_planner::frozenReplanAttemptDue(false, 10.0, 20.0));
  EXPECT_FALSE(scan_planner::frozenReplanAttemptDue(true, 10.0, 10.5));
  EXPECT_TRUE(scan_planner::frozenReplanAttemptDue(true, 10.5, 10.5));
  EXPECT_DOUBLE_EQ(scan_planner::nextFrozenReplanTime(10.0, 0.5), 10.5);
}

TEST(ReplanFailurePolicy, FrozenFailureNeverDropsActiveTarget)
{
  EXPECT_FALSE(scan_planner::shouldEscalateReplanFailure(1000, 1000, true));
  EXPECT_FALSE(scan_planner::shouldEscalateReplanFailure(999, 1000, false));
  EXPECT_TRUE(scan_planner::shouldEscalateReplanFailure(1000, 1000, false));
}
