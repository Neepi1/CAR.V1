#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "robot_docking_perception/depth_dock_geometry.hpp"

namespace
{

using robot_docking_perception::DockGeometryConfig;
using robot_docking_perception::PlanarPoint;

std::vector<PlanarPoint> make_dock_face(
  const DockGeometryConfig & config,
  const double forward_gap,
  const double dock_center_y,
  const double correction_yaw_rad)
{
  const double slope = -std::tan(correction_yaw_rad);
  std::vector<PlanarPoint> points;
  for (int index = 0; index <= 48; ++index) {
    const double y = dock_center_y - 0.1175 + 0.235 * static_cast<double>(index) / 48.0;
    const double x = config.charge_contact_x_m + forward_gap +
      slope * (y - config.charge_contact_y_m);
    points.push_back({x, y});
  }
  return points;
}

TEST(DepthDockGeometry, DefaultDepthWindowCannotBridgeMinimumHeadProtrusion)
{
  const DockGeometryConfig config;

  // The measured near-field head/face separation can fall to about 6 cm.
  // Keep one full depth-noise margin between the clustering window and that gap.
  EXPECT_LE(config.front_cluster_window_m, 0.04);
}

TEST(DepthDockGeometry, EstimatesCenteredFrontFacingDockInContactCoordinates)
{
  DockGeometryConfig config;
  config.min_points = 20U;
  config.min_lateral_span_m = 0.10;

  constexpr double kForwardGap = 0.42;
  constexpr double kDockCenterY = 0.03;
  const auto points = make_dock_face(config, kForwardGap, kDockCenterY, 0.0);

  const auto estimate = robot_docking_perception::estimate_dock_target(points, config);

  ASSERT_TRUE(estimate.valid) << estimate.reason;
  EXPECT_NEAR(estimate.forward_gap_m, kForwardGap, 1e-3);
  EXPECT_NEAR(estimate.lateral_error_m, kDockCenterY, 1e-3);
  EXPECT_NEAR(estimate.yaw_error_rad, 0.0, 1e-3);
  EXPECT_GT(estimate.confidence, 0.5);
}

TEST(DepthDockGeometry, RejectsCloserNarrowObjectAndPreservesCorrectionYawSign)
{
  DockGeometryConfig config;
  config.min_points = 20U;
  config.min_lateral_span_m = 0.10;

  constexpr double kForwardGap = 0.47;
  constexpr double kDockCenterY = -0.04;
  constexpr double kCorrectionYaw = 0.08;
  auto points = make_dock_face(config, kForwardGap, kDockCenterY, kCorrectionYaw);

  // A narrow object is closer and dense enough to pass the point-count gate, but it is
  // not wide enough to be the charging-dock face.
  for (int index = 0; index < 25; ++index) {
    const double y = -0.015 + 0.03 * static_cast<double>(index) / 24.0;
    points.push_back({config.charge_contact_x_m + 0.18, y});
  }
  points.push_back({config.charge_contact_x_m + 0.30, -0.40});
  points.push_back({config.charge_contact_x_m + 0.90, 0.38});

  const auto estimate = robot_docking_perception::estimate_dock_target(points, config);

  ASSERT_TRUE(estimate.valid) << estimate.reason;
  EXPECT_NEAR(estimate.forward_gap_m, kForwardGap, 1e-3);
  EXPECT_NEAR(estimate.lateral_error_m, kDockCenterY, 1e-3);
  EXPECT_NEAR(estimate.yaw_error_rad, kCorrectionYaw, 1e-3);
  EXPECT_GE(estimate.inlier_count, 45U);
}

TEST(DepthDockGeometry, SeparatesCloseNarrowHeadFromWideDockFace)
{
  DockGeometryConfig config;
  config.min_points = 20U;
  config.min_lateral_span_m = 0.10;

  constexpr double kFaceGap = 0.394;
  constexpr double kDockCenterY = -0.05;
  auto points = make_dock_face(config, kFaceGap, kDockCenterY, 0.0);

  // At close range the angled telescoping head can end less than 8 cm in front of
  // the fixed face. It must remain a separate narrow cluster, otherwise the mixed
  // line fit reports a false yaw or rejects the fixed face entirely.
  const double head_slope = std::tan(0.19);
  for (int repeat = 0; repeat < 3; ++repeat) {
    for (int index = 0; index <= 40; ++index) {
      const double y = -0.065 + 0.035 * static_cast<double>(index) / 40.0;
      const double x = config.charge_contact_x_m + 0.326 + head_slope * y;
      points.push_back({x, y});
    }
  }

  const auto estimate = robot_docking_perception::estimate_dock_target(points, config);

  ASSERT_TRUE(estimate.valid) << estimate.reason;
  EXPECT_NEAR(estimate.forward_gap_m, kFaceGap, 1e-3);
  EXPECT_NEAR(estimate.lateral_error_m, kDockCenterY, 2e-3);
  EXPECT_NEAR(estimate.yaw_error_rad, 0.0, 1e-3);
}

TEST(DepthDockGeometry, DoesNotReportNarrowGeometryAsDockFace)
{
  DockGeometryConfig config;
  config.min_points = 20U;
  config.min_lateral_span_m = 0.10;

  std::vector<PlanarPoint> points;
  for (int index = 0; index < 50; ++index) {
    const double y = -0.02 + 0.04 * static_cast<double>(index) / 49.0;
    points.push_back({config.charge_contact_x_m + 0.35, y});
  }

  const auto estimate = robot_docking_perception::estimate_dock_target(points, config);

  EXPECT_FALSE(estimate.valid);
  EXPECT_EQ(estimate.reason, "no_dense_wide_front_cluster");
}

TEST(DepthDockGeometry, DoesNotReportWideWallAsDockFace)
{
  DockGeometryConfig config;
  config.min_points = 20U;
  config.min_lateral_span_m = 0.10;
  config.max_lateral_span_m = 0.36;

  std::vector<PlanarPoint> points;
  for (int index = 0; index <= 120; ++index) {
    const double y = -0.45 + 0.90 * static_cast<double>(index) / 120.0;
    points.push_back({config.charge_contact_x_m + 0.45, y});
  }

  const auto estimate = robot_docking_perception::estimate_dock_target(points, config);

  EXPECT_FALSE(estimate.valid);
  EXPECT_EQ(estimate.reason, "no_dense_wide_front_cluster");
}

TEST(DepthDockGeometry, DoesNotReportCroppedBackgroundWallAsDockFeature)
{
  DockGeometryConfig config;
  config.min_points = 20U;
  config.min_lateral_span_m = 0.10;
  config.max_lateral_span_m = 0.36;

  // Captured failure signature at the charging pose: the real 0.113 m dock
  // protrusion is no longer visible, while an occluded slice of the background
  // wall fits inside the legacy 0.36 m width gate with high confidence.
  std::vector<PlanarPoint> points;
  for (int index = 0; index <= 120; ++index) {
    const double y = -0.4265 + 0.343 * static_cast<double>(index) / 120.0;
    points.push_back({config.charge_contact_x_m + 0.422, y});
  }

  const auto estimate = robot_docking_perception::estimate_dock_target(points, config);

  EXPECT_FALSE(estimate.valid);
  EXPECT_EQ(estimate.reason, "no_known_dock_feature_cluster");
}

TEST(DepthDockGeometry, AcceptsCalibratedNearProtrusionOnlyInHandoffZone)
{
  DockGeometryConfig config;
  config.min_points = 20U;
  config.min_lateral_span_m = 0.10;

  auto make_near_feature = [&config](const double forward_gap) {
      std::vector<PlanarPoint> points;
      constexpr double kRobustSpan = 0.113;
      constexpr double kFullSpan = kRobustSpan / 0.90;
      for (int index = 0; index <= 80; ++index) {
        const double y = -0.0418 - 0.5 * kFullSpan +
          kFullSpan * static_cast<double>(index) / 80.0;
        points.push_back({config.charge_contact_x_m + forward_gap, y});
      }
      return points;
    };

  const auto near_estimate = robot_docking_perception::estimate_dock_target(
    make_near_feature(0.243), config);
  const auto far_estimate = robot_docking_perception::estimate_dock_target(
    make_near_feature(0.45), config);

  ASSERT_TRUE(near_estimate.valid) << near_estimate.reason;
  EXPECT_NEAR(near_estimate.lateral_span_m, 0.113, 1e-3);
  EXPECT_NEAR(near_estimate.lateral_error_m, 0.0, 1e-3);
  EXPECT_FALSE(far_estimate.valid);
  EXPECT_EQ(far_estimate.reason, "no_known_dock_feature_cluster");
}

TEST(DepthDockGeometry, SkipsCroppedBackgroundAndUsesKnownFixedFace)
{
  DockGeometryConfig config;
  config.min_points = 20U;
  config.min_lateral_span_m = 0.10;

  auto points = make_dock_face(config, 0.55, 0.02, 0.0);
  for (int index = 0; index <= 120; ++index) {
    const double y = -0.4265 + 0.343 * static_cast<double>(index) / 120.0;
    points.push_back({config.charge_contact_x_m + 0.422, y});
  }

  const auto estimate = robot_docking_perception::estimate_dock_target(points, config);

  ASSERT_TRUE(estimate.valid) << estimate.reason;
  EXPECT_NEAR(estimate.forward_gap_m, 0.55, 1e-3);
  EXPECT_NEAR(estimate.lateral_error_m, 0.02, 1e-3);
}

}  // namespace
