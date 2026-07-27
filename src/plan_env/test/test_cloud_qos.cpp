#include <gtest/gtest.h>

#include "plan_env/cloud_qos.h"

TEST(CloudQos, UsesBestEffortSensorProfile)
{
  const auto profile = plan_env::cloudSensorQos().get_rmw_qos_profile();

  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 1U);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}
