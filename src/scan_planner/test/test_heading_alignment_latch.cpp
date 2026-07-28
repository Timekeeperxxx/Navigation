#include <gtest/gtest.h>

#include "plan_manage/heading_alignment_latch.h"

namespace scan_planner
{
namespace
{

TEST(HeadingAlignmentLatchTest, DoesNotEnterInsideStopThreshold)
{
  HeadingAlignmentLatch latch;
  latch.configure(0.50, 0.35);

  EXPECT_FALSE(latch.update(0.49));
}

TEST(HeadingAlignmentLatchTest, StaysLatchedBetweenResumeAndStopThresholds)
{
  HeadingAlignmentLatch latch;
  latch.configure(0.50, 0.35);

  EXPECT_TRUE(latch.update(0.60));
  EXPECT_TRUE(latch.update(0.42));
  EXPECT_TRUE(latch.isAligning());
}

TEST(HeadingAlignmentLatchTest, ResumesOnlyAfterLowerThreshold)
{
  HeadingAlignmentLatch latch;
  latch.configure(0.50, 0.35);
  ASSERT_TRUE(latch.update(-0.60));

  EXPECT_FALSE(latch.update(-0.35));
  EXPECT_FALSE(latch.isAligning());
}

TEST(HeadingAlignmentLatchTest, ClampsInvalidResumeThreshold)
{
  HeadingAlignmentLatch latch;
  latch.configure(0.40, 0.70);

  EXPECT_DOUBLE_EQ(latch.stopThreshold(), 0.40);
  EXPECT_DOUBLE_EQ(latch.resumeThreshold(), 0.40);
}

}  // namespace
}  // namespace scan_planner
