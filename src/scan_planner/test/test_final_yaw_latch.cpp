#include <gtest/gtest.h>

#include "plan_manage/final_yaw_latch.h"

namespace scan_planner
{
namespace
{

TEST(FinalYawLatchTest, DoesNotLatchBeforeTrajectoryFinishes)
{
  FinalYawLatch latch;

  const FinalYawDecision decision = latch.update(
      false, 0.05, 0.10, true, true, 1.0, 0.10);

  EXPECT_FALSE(decision.hold_position);
  EXPECT_FALSE(decision.alignment_started);
  EXPECT_FALSE(latch.isAligning());
  EXPECT_FALSE(latch.isComplete());
}

TEST(FinalYawLatchTest, StartsAlignmentInsideTerminalPositionTolerance)
{
  FinalYawLatch latch;

  const FinalYawDecision decision = latch.update(
      true, 0.09, 0.10, true, true, 1.0, 0.10);

  EXPECT_TRUE(decision.hold_position);
  EXPECT_TRUE(decision.alignment_started);
  EXPECT_TRUE(latch.isAligning());
}

TEST(FinalYawLatchTest, PositionDriftCannotInterruptAlignment)
{
  FinalYawLatch latch;
  latch.update(true, 0.09, 0.10, true, true, 1.0, 0.10);

  const FinalYawDecision decision = latch.update(
      true, 0.35, 0.10, true, true, 0.8, 0.10);

  EXPECT_TRUE(decision.hold_position);
  EXPECT_FALSE(decision.alignment_completed);
  EXPECT_TRUE(latch.isAligning());
}

TEST(FinalYawLatchTest, HoldsStillAfterYawAlignmentCompletes)
{
  FinalYawLatch latch;
  latch.update(true, 0.09, 0.10, true, true, 1.0, 0.10);

  const FinalYawDecision completed = latch.update(
      true, 0.25, 0.10, true, true, 0.08, 0.10);
  const FinalYawDecision held = latch.update(
      true, 0.50, 0.10, true, true, 0.50, 0.10);

  EXPECT_TRUE(completed.hold_position);
  EXPECT_TRUE(completed.alignment_completed);
  EXPECT_TRUE(latch.isComplete());
  EXPECT_TRUE(held.hold_position);
  EXPECT_FALSE(held.alignment_started);
}

TEST(FinalYawLatchTest, ResetAllowsTheNextTrajectoryToRun)
{
  FinalYawLatch latch;
  latch.update(true, 0.05, 0.10, true, true, 0.0, 0.10);
  ASSERT_TRUE(latch.isComplete());

  latch.reset();
  const FinalYawDecision decision = latch.update(
      false, 0.05, 0.10, true, true, 1.0, 0.10);

  EXPECT_FALSE(decision.hold_position);
  EXPECT_FALSE(latch.isComplete());
}

}  // namespace
}  // namespace scan_planner
