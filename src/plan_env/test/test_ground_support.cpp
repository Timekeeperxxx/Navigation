#include <gtest/gtest.h>

#include <plan_env/ground_support.h>

namespace
{

plan_env::GroundSupportConfig testConfig()
{
  plan_env::GroundSupportConfig config;
  config.bucket_size = 0.10;
  config.xy_tolerance = 0.08;
  config.z_tolerance = 0.12;
  config.planning_height = 0.32;
  config.circle_radius = 0.27;
  config.circle_offset = 0.205;
  config.circle_center_offset = -0.425;
  config.footprint_probe_margin = 0.08;
  config.perimeter_samples = 16;
  config.radial_samples = 2;
  return config;
}

void addPlane(
    plan_env::GroundSupportIndex & index,
    double min_x, double max_x,
    double min_y, double max_y,
    double z,
    double spacing = 0.05)
{
  for (double x = min_x; x <= max_x + 1e-9; x += spacing)
    for (double y = min_y; y <= max_y + 1e-9; y += spacing)
      index.addPoint(x, y, z);
}

}  // namespace

TEST(GroundSupportIndex, OpenPlaneSupportsCompleteDoubleCircle)
{
  plan_env::GroundSupportIndex index(testConfig());
  addPlane(index, -2.0, 2.0, -2.0, 2.0, -0.60);

  EXPECT_TRUE(index.isPoseSupported(0.0, 0.0, -0.28, 0.0));
  EXPECT_TRUE(index.isPoseSupported(0.0, 0.0, -0.28, 1.2));
}

TEST(GroundSupportIndex, EmptyAreaIsHardUnsupported)
{
  plan_env::GroundSupportIndex index(testConfig());
  addPlane(index, -2.0, 0.0, -2.0, 2.0, -0.60);
  addPlane(index, 2.0, 4.0, -2.0, 2.0, -0.60);

  EXPECT_FALSE(index.isPoseSupported(1.0, 0.0, -0.28, 0.0));
}

TEST(GroundSupportIndex, DifferentFloorDoesNotSupportPose)
{
  plan_env::GroundSupportIndex index(testConfig());
  addPlane(index, -2.0, 2.0, -2.0, 2.0, 2.40);

  EXPECT_FALSE(index.isPoseSupported(0.0, 0.0, -0.28, 0.0));
  EXPECT_TRUE(index.isPoseSupported(0.0, 0.0, 2.72, 0.0));
}

TEST(GroundSupportIndex, CompleteLowerFloorCannotFillHoleOnCurrentFloor)
{
  plan_env::GroundSupportIndex index(testConfig());
  addPlane(index, -2.0, 4.0, -2.0, 2.0, -3.00);
  addPlane(index, -2.0, 0.0, -2.0, 2.0, -0.60);
  addPlane(index, 2.0, 4.0, -2.0, 2.0, -0.60);

  EXPECT_FALSE(index.isPoseSupported(1.0, 0.0, -0.28, 0.0));
}

TEST(GroundSupportIndex, TenCentimeterGroundSamplingIsSupported)
{
  plan_env::GroundSupportConfig config = testConfig();
  config.bucket_size = 0.14;
  config.xy_tolerance = 0.14;
  config.z_tolerance = 0.20;
  config.footprint_probe_margin = 0.19;
  plan_env::GroundSupportIndex index(config);
  addPlane(index, -2.0, 2.0, -2.0, 2.0, -0.60, 0.10);

  EXPECT_TRUE(index.isPoseSupported(0.0, 0.0, -0.28, 0.0));
  EXPECT_TRUE(index.isPoseSupported(0.0, 0.0, -0.28, 0.37));
  EXPECT_TRUE(index.isPoseSupported(0.0, 0.0, -0.28, 1.21));
}

TEST(GroundSupportIndex, MissingInnerRingIsUnsupported)
{
  plan_env::GroundSupportConfig config = testConfig();
  config.circle_offset = 0.60;
  plan_env::GroundSupportIndex index(config);

  constexpr double kTwoPi = 6.283185307179586476925286766559;
  const double body_x = 0.0;
  const double body_y = 0.0;
  const double ground_z = -0.60;
  const double base_center_x = body_x + config.circle_center_offset;
  const double probe_radius =
      config.circle_radius + config.footprint_probe_margin;
  for (double offset : {config.circle_offset, -config.circle_offset})
  {
    const double circle_x = base_center_x + offset;
    index.addPoint(circle_x, body_y, ground_z);
    for (int sample = 0; sample < config.perimeter_samples; ++sample)
    {
      const double angle =
          kTwoPi * static_cast<double>(sample) /
          static_cast<double>(config.perimeter_samples);
      index.addPoint(
          circle_x + probe_radius * std::cos(angle),
          body_y + probe_radius * std::sin(angle),
          ground_z);
    }
  }

  EXPECT_FALSE(index.isPoseSupported(body_x, body_y, -0.28, 0.0));
}

TEST(GroundSupportIndex, FootprintCannotHangOverGroundEdge)
{
  plan_env::GroundSupportIndex index(testConfig());
  addPlane(index, -2.0, 0.0, -2.0, 2.0, -0.60);

  // The body origin still lies on ground, but the front of the double-circle
  // footprint extends beyond the x=0 edge.
  EXPECT_FALSE(index.isPoseSupported(0.0, 0.0, -0.28, 0.0));
  EXPECT_TRUE(index.isPoseSupported(-0.20, 0.0, -0.28, 0.0));
}

TEST(GroundSupportIndex, EmptyIndexFailsClosed)
{
  plan_env::GroundSupportIndex index(testConfig());

  EXPECT_FALSE(index.isPoseSupported(0.0, 0.0, 0.32, 0.0));
}
