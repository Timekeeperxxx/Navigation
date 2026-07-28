#include <gtest/gtest.h>

#include <plan_manage/reference_guide.h>
#include <plan_manage/b2_motion_policy.h>

#include <Eigen/Eigen>

#include <vector>

namespace
{

TEST(B2RecoveryLegCompletion, AcceptsOnlyBoundedForwardOvershoot)
{
  constexpr double yaw = 0.0;

  EXPECT_TRUE(scan_planner::shouldAdvanceB2RecoveryLeg(
      Eigen::Vector2d(0.02, 0.0), yaw));
  EXPECT_FALSE(scan_planner::shouldAdvanceB2RecoveryLeg(
      Eigen::Vector2d(0.05, 0.0), yaw));
  EXPECT_TRUE(scan_planner::shouldAdvanceB2RecoveryLeg(
      Eigen::Vector2d(-0.05, 0.0), yaw));
  EXPECT_FALSE(scan_planner::shouldAdvanceB2RecoveryLeg(
      Eigen::Vector2d(-0.11, 0.0), yaw));
}

TEST(ReferenceGuide, FindsFirstB2StopCornerInsideBoundedLookahead)
{
  const std::vector<Eigen::Vector3d> path{
      Eigen::Vector3d(0.0, 0.0, 0.32),
      Eigen::Vector3d(1.0, 0.0, 0.32),
      Eigen::Vector3d(1.0, 1.0, 0.32),
      Eigen::Vector3d(2.0, 1.0, 0.32)};

  EXPECT_EQ(
      scan_planner::findFirstReferenceStopCorner(
          path, 1, 2, path.front(), 0.35, 0.20),
      1U);
  EXPECT_EQ(
      scan_planner::findFirstReferenceStopCorner(
          path, 2, 2, path.front(), 0.35, 0.20),
      2U);
}

TEST(ReferenceGuide, IgnoresSmallOrAlreadyReachedCorner)
{
  const std::vector<Eigen::Vector3d> shallow{
      Eigen::Vector3d(0.0, 0.0, 0.32),
      Eigen::Vector3d(1.0, 0.0, 0.32),
      Eigen::Vector3d(2.0, 0.1, 0.32)};
  EXPECT_EQ(
      scan_planner::findFirstReferenceStopCorner(
          shallow, 1, 1, shallow.front(), 0.35, 0.20),
      shallow.size());

  const Eigen::Vector3d near_corner(0.90, 0.0, 0.32);
  EXPECT_EQ(
      scan_planner::findFirstReferenceStopCorner(
          shallow, 1, 1, near_corner, 0.05, 0.20),
      shallow.size());
}

TEST(ReferenceGuide, RetainsCornerAndTerrainHeight)
{
  const Eigen::Vector3d start(0.0, 0.0, 0.32);
  const Eigen::Vector3d corner(1.0, 0.0, 0.47);
  const Eigen::Vector3d target(1.0, 1.0, 0.71);
  const std::vector<Eigen::Vector3d> guide{
      start,
      Eigen::Vector3d(0.5, 0.0, 0.39),
      corner,
      Eigen::Vector3d(1.0, 0.5, 0.58),
      target};

  const auto sampled = scan_planner::resampleReferenceGuide(
      guide, start, target, 0.20, 7);

  ASSERT_GE(sampled.size(), 7U);
  EXPECT_TRUE(sampled.front().isApprox(start, 1e-12));
  EXPECT_TRUE(sampled.back().isApprox(target, 1e-12));

  bool retained_corner = false;
  for (const auto &point : sampled)
  {
    if (point.isApprox(corner, 1e-12))
      retained_corner = true;
  }
  EXPECT_TRUE(retained_corner);

  for (std::size_t index = 1; index < sampled.size(); ++index)
    EXPECT_LE((sampled[index] - sampled[index - 1]).norm(), 0.200001);
}

TEST(ReferenceGuide, AddsEnoughSamplesForShortSparseGuide)
{
  const Eigen::Vector3d start(0.0, 0.0, -2.3);
  const Eigen::Vector3d target(0.4, 0.0, -2.1);

  const auto sampled = scan_planner::resampleReferenceGuide(
      {start, target}, start, target, 0.20, 7);

  ASSERT_GE(sampled.size(), 7U);
  EXPECT_TRUE(sampled.front().isApprox(start, 1e-12));
  EXPECT_TRUE(sampled.back().isApprox(target, 1e-12));
  for (const auto &point : sampled)
    EXPECT_LT(point.z(), -2.0);
}

TEST(ReferenceGuide, UsesOutgoingDirectionAtCompletedSegment)
{
  const std::vector<Eigen::Vector3d> path{
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {1.0, -1.0, 0.0},
      {2.0, -1.0, 0.0}};

  EXPECT_TRUE(
      scan_planner::referenceForwardDirection(path, 0, 0.5)
          .isApprox(Eigen::Vector2d(1.0, 0.0), 1e-12));
  EXPECT_TRUE(
      scan_planner::referenceForwardDirection(path, 0, 1.0)
          .isApprox(Eigen::Vector2d(0.0, -1.0), 1e-12));
  EXPECT_TRUE(
      scan_planner::referenceForwardDirection(path, 1, 1.0)
          .isApprox(Eigen::Vector2d(1.0, 0.0), 1e-12));

  const auto outgoing =
      scan_planner::canonicalReferenceProgress(0, 1.0, 3);
  EXPECT_EQ(outgoing.segment, 1U);
  EXPECT_DOUBLE_EQ(outgoing.ratio, 0.0);

  const auto final =
      scan_planner::canonicalReferenceProgress(2, 1.0, 3);
  EXPECT_EQ(final.segment, 2U);
  EXPECT_DOUBLE_EQ(final.ratio, 1.0);
}

TEST(B2MotionPolicy, SeedsCurrentHeadingOnlyThroughSafeForwardCorridor)
{
  scan_planner::B2ForwardSeedConfig config;
  config.speed = 0.15;
  config.clearance_distance = 0.25;
  config.sample_step = 0.025;
  config.minimum_reference_alignment = 0.25;

  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  EXPECT_TRUE(scan_planner::makeB2ForwardSeedVelocity(
      Eigen::Vector3d::Zero(),
      -0.4,
      Eigen::Vector2d(1.0, -0.5),
      config,
      [](const Eigen::Vector3d &, double) { return true; },
      velocity));
  EXPECT_NEAR(velocity.head<2>().norm(), 0.15, 1e-12);
  EXPECT_NEAR(std::atan2(velocity.y(), velocity.x()), -0.4, 1e-12);

  EXPECT_FALSE(scan_planner::makeB2ForwardSeedVelocity(
      Eigen::Vector3d::Zero(),
      0.0,
      Eigen::Vector2d(-1.0, 0.0),
      config,
      [](const Eigen::Vector3d &, double) { return true; },
      velocity));

  EXPECT_FALSE(scan_planner::makeB2ForwardSeedVelocity(
      Eigen::Vector3d::Zero(),
      0.0,
      Eigen::Vector2d(1.0, 0.0),
      config,
      [](const Eigen::Vector3d &position, double) {
        return position.x() < 0.18;
      },
      velocity));
}

TEST(B2MotionPolicy, SeedsOutgoingReferenceAfterSafeInPlaceTurn)
{
  scan_planner::B2ForwardSeedConfig config;
  config.speed = 0.15;
  config.clearance_distance = 0.25;
  config.sample_step = 0.025;

  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  std::size_t pose_checks = 0;
  EXPECT_TRUE(scan_planner::makeB2ReferenceSeedVelocity(
      Eigen::Vector3d::Zero(),
      3.14159265358979323846,
      Eigen::Vector2d(1.0, 0.0),
      config,
      [&pose_checks](const Eigen::Vector3d &, double) {
        ++pose_checks;
        return true;
      },
      velocity));
  EXPECT_GT(pose_checks, 10U);
  EXPECT_NEAR(velocity.x(), 0.15, 1e-12);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-12);

  EXPECT_FALSE(scan_planner::makeB2ReferenceSeedVelocity(
      Eigen::Vector3d::Zero(),
      0.0,
      Eigen::Vector2d(0.0, 1.0),
      config,
      [](const Eigen::Vector3d &position, double yaw) {
        return position.head<2>().norm() > 1e-9 || yaw < 0.8;
      },
      velocity));
}

TEST(B2MotionPolicy, RevalidatesCachedLegFromLivePose)
{
  const Eigen::Vector3d start(0.0, 0.0, 0.32);
  const Eigen::Vector3d target(0.0, 1.0, 0.32);

  EXPECT_TRUE(scan_planner::isB2StopTurnForwardLegSafe(
      start,
      0.0,
      target,
      0.05,
      0.05,
      [](const Eigen::Vector3d &, double) { return true; }));

  // The endpoint and straight translation are free, but an intermediate
  // attitude of the long footprint is not. A cached XY-only leg must not be
  // reused from this changed live pose.
  EXPECT_FALSE(scan_planner::isB2StopTurnForwardLegSafe(
      start,
      0.0,
      target,
      0.05,
      0.05,
      [](const Eigen::Vector3d &position, double yaw) {
        return position.head<2>().norm() > 1e-6 ||
            std::abs(yaw - 0.75) > 0.08;
      }));
}

TEST(B2MotionPolicy, RejectsWrongWayOrBacktrackingCandidate)
{
  scan_planner::B2DirectionGuardConfig config;
  const Eigen::Vector2d target(1.0, -1.0);
  const Eigen::Vector2d reference(1.0, -1.0);

  const std::vector<Eigen::Vector3d> correct{
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.10, -0.04, 0.0),
      Eigen::Vector3d(0.20, -0.10, 0.0),
      Eigen::Vector3d(0.35, -0.22, 0.0),
      Eigen::Vector3d(0.50, -0.40, 0.0)};
  EXPECT_TRUE(scan_planner::isB2ForwardCandidate(
      correct, target, reference, config));

  const std::vector<Eigen::Vector3d> wrong_left{
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.10, 0.10, 0.0),
      Eigen::Vector3d(0.20, 0.20, 0.0),
      Eigen::Vector3d(0.35, 0.30, 0.0)};
  EXPECT_FALSE(scan_planner::isB2ForwardCandidate(
      wrong_left, target, reference, config));

  const std::vector<Eigen::Vector3d> backtracking{
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.15, -0.10, 0.0),
      Eigen::Vector3d(0.25, -0.20, 0.0),
      Eigen::Vector3d(0.15, -0.10, 0.0),
      Eigen::Vector3d(0.40, -0.30, 0.0)};
  EXPECT_FALSE(scan_planner::isB2ForwardCandidate(
      backtracking, target, reference, config));
}

TEST(B2MotionPolicy, KeepsBoundedDetourDuringLatchedObstacleRecovery)
{
  EXPECT_FALSE(scan_planner::shouldUseB2BoundedObstacleDetour(
      false, true, true));
  EXPECT_FALSE(scan_planner::shouldUseB2BoundedObstacleDetour(
      true, false, false));
  EXPECT_TRUE(scan_planner::shouldUseB2BoundedObstacleDetour(
      true, true, false));
  EXPECT_TRUE(scan_planner::shouldUseB2BoundedObstacleDetour(
      true, false, true));
}

TEST(B2MotionPolicy, HeadingAwareForwardDetourRoutesAroundObstacle)
{
  const Eigen::Vector3d start(0.0, 0.0, 0.0);
  const Eigen::Vector3d target(3.0, 0.0, 0.0);
  scan_planner::B2ForwardDetourSearchConfig config;
  config.maximum_search_extent = 4.0;
  config.maximum_expansions = 20000;

  std::vector<Eigen::Vector3d> path;
  ASSERT_TRUE(scan_planner::searchB2ForwardDetour(
      start,
      0.0,
      target,
      config,
      [](const Eigen::Vector3d &position, double) {
        return
            (position.head<2>() - Eigen::Vector2d(1.5, 0.0)).norm() >
            0.45;
      },
      path));
  ASSERT_GE(path.size(), 3u);
  EXPECT_LT((path.front() - start).norm(), 1e-9);
  EXPECT_LT((path.back() - target).norm(), 1e-9);

  double maximum_lateral_offset = 0.0;
  for (const Eigen::Vector3d &position : path)
  {
    maximum_lateral_offset =
        std::max(maximum_lateral_offset, std::abs(position.y()));
    EXPECT_GT(
        (position.head<2>() - Eigen::Vector2d(1.5, 0.0)).norm(),
        0.45 - 1e-9);
  }
  EXPECT_GT(maximum_lateral_offset, 0.40);
}

TEST(B2MotionPolicy, HeadingAwareForwardDetourFailsClosed)
{
  std::vector<Eigen::Vector3d> path;
  EXPECT_FALSE(scan_planner::searchB2ForwardDetour(
      Eigen::Vector3d::Zero(),
      0.0,
      Eigen::Vector3d(2.0, 0.0, 0.0),
      scan_planner::B2ForwardDetourSearchConfig(),
      [](const Eigen::Vector3d &, double) {
        return false;
      },
      path));
  EXPECT_TRUE(path.empty());
}

TEST(B2MotionPolicy, HeadingAwareDetourCanSteerForwardWhenPivotIsBlocked)
{
  scan_planner::B2ForwardDetourSearchConfig config;
  config.maximum_search_extent = 2.5;
  config.maximum_expansions = 30000;
  std::vector<Eigen::Vector3d> path;

  const auto safe_except_pivot = [](
      const Eigen::Vector3d &position, double yaw) {
    // At the start, any meaningful in-place rotation is blocked. Once the
    // body has moved forward, the same yaw range is free.
    if (position.head<2>().norm() < 0.08 &&
        std::abs(std::atan2(std::sin(yaw), std::cos(yaw))) > 0.08)
      return false;
    return true;
  };

  EXPECT_TRUE(scan_planner::searchB2ForwardDetour(
      Eigen::Vector3d(0.0, 0.0, 0.32),
      0.0,
      Eigen::Vector3d(0.0, 1.2, 0.32),
      config,
      safe_except_pivot,
      path));
  ASSERT_GE(path.size(), 3U);
  EXPECT_GT(path[1].x(), 0.05);
}

TEST(B2MotionPolicy, StartsSteeringFromExactLiveYawWithoutQuantizedPivot)
{
  scan_planner::B2ForwardDetourSearchConfig config;
  config.maximum_search_extent = 2.5;
  config.maximum_expansions = 30000;
  const double live_yaw = 0.13;
  std::vector<Eigen::Vector3d> path;
  std::vector<double> yaws;
  std::vector<bool> simultaneous_yaw;

  const auto exact_live_yaw_only_at_start =
      [live_yaw](const Eigen::Vector3d &position, double yaw) {
        if (position.head<2>().norm() >= 0.04)
          return true;
        const double error = std::atan2(
            std::sin(yaw - live_yaw),
            std::cos(yaw - live_yaw));
        return std::abs(error) <= 0.01;
      };

  ASSERT_TRUE(scan_planner::searchB2ForwardDetour(
      Eigen::Vector3d(0.0, 0.0, 0.32),
      live_yaw,
      Eigen::Vector3d(0.0, 1.2, 0.32),
      config,
      exact_live_yaw_only_at_start,
      path,
      &yaws,
      &simultaneous_yaw));
  ASSERT_GE(path.size(), 3U);
  ASSERT_EQ(path.size(), yaws.size());
  ASSERT_EQ(path.size(), simultaneous_yaw.size());
  EXPECT_NEAR(yaws.front(), live_yaw, 1e-12);
  EXPECT_TRUE(simultaneous_yaw[1]);
}

TEST(B2MotionPolicy, RejectsFlatGuidedLoopButAllowsForwardDetour)
{
  scan_planner::B2DirectionGuardConfig config;
  const Eigen::Vector2d forward(1.0, 0.0);
  const std::vector<Eigen::Vector3d> loop{
      {0.00, 0.00, 0.0},
      {0.02, 0.08, 0.0},
      {-0.03, 0.17, 0.0},
      {-0.12, 0.22, 0.0},
      {-0.21, 0.18, 0.0},
      {-0.25, 0.08, 0.0},
      {-0.20, -0.02, 0.0},
      {-0.08, -0.05, 0.0},
      {0.03, 0.00, 0.0},
      {0.25, 0.00, 0.0},
      {0.55, 0.00, 0.0}};
  EXPECT_TRUE(scan_planner::checkB2PathGeometry(loop, 0.70).valid);
  EXPECT_FALSE(scan_planner::isB2ForwardCandidate(
      loop, forward, forward, config));

  const std::vector<Eigen::Vector3d> forward_detour{
      {0.00, 0.00, 0.0},
      {0.08, 0.05, 0.0},
      {0.18, 0.15, 0.0},
      {0.30, 0.22, 0.0},
      {0.45, 0.20, 0.0},
      {0.62, 0.08, 0.0}};
  EXPECT_TRUE(scan_planner::isB2ForwardCandidate(
      forward_detour, forward, forward, config));

  const std::vector<Eigen::Vector3d> lateral_detour{
      {0.00, 0.00, 0.0},
      {0.00, 0.10, 0.0},
      {0.01, 0.22, 0.0},
      {0.08, 0.34, 0.0},
      {0.24, 0.40, 0.0},
      {0.48, 0.32, 0.0}};
  EXPECT_FALSE(scan_planner::isB2ForwardCandidate(
      lateral_detour, forward, forward, config));
  EXPECT_TRUE(scan_planner::isB2ForwardCandidate(
      lateral_detour, forward, forward, config, false));

  const std::vector<Eigen::Vector3d> bounded_retreat_detour{
      {0.00, 0.00, 0.0},
      {-0.10, 0.10, 0.0},
      {-0.16, 0.25, 0.0},
      {-0.05, 0.42, 0.0},
      {0.25, 0.48, 0.0},
      {0.65, 0.30, 0.0}};
  EXPECT_FALSE(scan_planner::isB2ForwardCandidate(
      bounded_retreat_detour, forward, forward, config, false));
  EXPECT_TRUE(scan_planner::isB2ForwardCandidate(
      bounded_retreat_detour, forward, forward, config, false, true));

  auto excessive_retreat = bounded_retreat_detour;
  excessive_retreat[1].x() = -0.40;
  EXPECT_FALSE(scan_planner::isB2ForwardCandidate(
      excessive_retreat, forward, forward, config, false, true));

  const std::vector<Eigen::Vector3d> detour_without_final_progress{
      {0.00, 0.00, 0.0},
      {-0.10, 0.10, 0.0},
      {-0.15, 0.25, 0.0},
      {0.05, 0.35, 0.0}};
  EXPECT_FALSE(scan_planner::isB2ForwardCandidate(
      detour_without_final_progress, forward, forward, config, false, true));
}

TEST(B2MotionPolicy, RejectsVerticalCuspAndExcessiveTerrainSlope)
{
  const std::vector<Eigen::Vector3d> valid_path{
      {0.0, 0.0, 0.0},
      {0.2, 0.0, 0.05},
      {0.4, 0.0, 0.10}};
  EXPECT_TRUE(
      scan_planner::checkB2PathGeometry(valid_path, 0.70).valid);

  const auto cusp = scan_planner::checkB2PathGeometry(
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.01}}, 0.70);
  EXPECT_FALSE(cusp.valid);
  EXPECT_EQ(cusp.segment, 0U);

  const auto steep = scan_planner::checkB2PathGeometry(
      {{0.0, 0.0, 0.0}, {0.1, 0.0, 0.08}}, 0.70);
  EXPECT_FALSE(steep.valid);
  EXPECT_GT(steep.slope, 0.70);
}

}  // namespace
