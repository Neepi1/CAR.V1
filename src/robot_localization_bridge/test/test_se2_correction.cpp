#include <cmath>

#include "gtest/gtest.h"
#include "robot_localization_bridge/se2_correction.hpp"

namespace se2 = robot_localization_bridge::se2;

TEST(Se2Correction, YawAtLongOdomLeverArmIsNotRobotTranslation)
{
  const se2::Pose2D current_map_odom{};
  const se2::Pose2D odom_base{10.0, 0.0, 0.0};
  const se2::Pose2D measured_map_base{10.0, 0.0, se2::degrees_to_radians(2.0)};

  const auto solution = se2::solve_correction(
    current_map_odom, odom_base, measured_map_base);

  EXPECT_NEAR(solution.base_translation_m, 0.0, 1.0e-9);
  EXPECT_NEAR(solution.base_yaw_rad, se2::degrees_to_radians(2.0), 1.0e-9);
  EXPECT_GT(solution.map_odom_parameter_translation_m, 0.34);
  EXPECT_LT(solution.map_odom_parameter_translation_m, 0.36);
}

TEST(Se2Correction, GateMetricPreservesPhysicalMediumCorrection)
{
  const se2::Pose2D current_map_odom{};
  const se2::Pose2D odom_base{9.03, 0.0, 0.0};
  const se2::Pose2D measured_map_base{
    9.03, 0.092, se2::degrees_to_radians(-1.564)};

  const auto solution = se2::solve_correction(
    current_map_odom, odom_base, measured_map_base);

  EXPECT_NEAR(solution.base_translation_m, 0.092, 1.0e-9);
  EXPECT_LT(solution.base_translation_m, 0.15);
  EXPECT_GT(solution.map_odom_parameter_translation_m, 0.30);
}

TEST(Se2Correction, HypothesesAreComparedAtOneRobotReferencePose)
{
  const se2::Pose2D reference_odom_base{10.0, 0.0, 0.0};
  const auto first = se2::solve_correction(
    se2::Pose2D{}, reference_odom_base,
    se2::Pose2D{10.0, 0.0, se2::degrees_to_radians(2.0)});
  const auto second = se2::solve_correction(
    se2::Pose2D{}, reference_odom_base,
    se2::Pose2D{10.01, 0.0, se2::degrees_to_radians(2.1)});

  const auto agreement = se2::compare_map_odom_hypotheses_at_reference(
    first.target_map_odom, second.target_map_odom, reference_odom_base);

  EXPECT_NEAR(agreement.translation_m, 0.01, 1.0e-9);
  EXPECT_NEAR(agreement.yaw_rad, se2::degrees_to_radians(0.1), 1.0e-9);
  EXPECT_GT(
    std::hypot(
      second.target_map_odom.x - first.target_map_odom.x,
      second.target_map_odom.y - first.target_map_odom.y),
    agreement.translation_m);
}
