#include <cmath>

#include <gtest/gtest.h>

#include "robot_nav_config/elevator_scoped_path.hpp"

namespace
{

using robot_nav_config::ElevatorScopedMotionPhase;
using robot_nav_config::ElevatorScopedPathParameters;
using robot_nav_config::ElevatorScopedPose;
using robot_nav_config::make_elevator_scoped_path;

constexpr double kPi = 3.14159265358979323846;

TEST(ElevatorScopedPath, SerializesYawThenLateralThenForwardInGoalFrame)
{
  ElevatorScopedPathParameters parameters;
  parameters.max_distance_m = 2.5;
  parameters.translation_step_m = 0.20;
  parameters.rotation_step_rad = 0.10;

  const ElevatorScopedPose start{0.0, 0.0, 0.0};
  const ElevatorScopedPose goal{2.0, 1.0, kPi / 2.0};
  const auto path = make_elevator_scoped_path(start, goal, parameters);

  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->lateral_m, -2.0, 1.0e-9);
  EXPECT_NEAR(path->forward_m, 1.0, 1.0e-9);
  ASSERT_FALSE(path->samples.empty());

  bool saw_yaw = false;
  bool saw_lateral = false;
  bool saw_forward = false;
  for (const auto & sample : path->samples) {
    EXPECT_TRUE(std::isfinite(sample.pose.x));
    EXPECT_TRUE(std::isfinite(sample.pose.y));
    EXPECT_TRUE(std::isfinite(sample.pose.yaw));
    if (sample.phase == ElevatorScopedMotionPhase::kYaw) {
      EXPECT_FALSE(saw_lateral);
      EXPECT_FALSE(saw_forward);
      EXPECT_NEAR(sample.pose.x, start.x, 1.0e-9);
      EXPECT_NEAR(sample.pose.y, start.y, 1.0e-9);
      saw_yaw = true;
    } else if (sample.phase == ElevatorScopedMotionPhase::kLateral) {
      EXPECT_TRUE(saw_yaw);
      EXPECT_FALSE(saw_forward);
      EXPECT_NEAR(sample.pose.y, start.y, 1.0e-9);
      EXPECT_NEAR(sample.pose.yaw, goal.yaw, 1.0e-9);
      saw_lateral = true;
    } else if (sample.phase == ElevatorScopedMotionPhase::kForward) {
      EXPECT_TRUE(saw_lateral);
      EXPECT_NEAR(sample.pose.x, goal.x, 1.0e-9);
      EXPECT_NEAR(sample.pose.yaw, goal.yaw, 1.0e-9);
      saw_forward = true;
    }
  }

  EXPECT_TRUE(saw_yaw);
  EXPECT_TRUE(saw_lateral);
  EXPECT_TRUE(saw_forward);
  EXPECT_NEAR(path->samples.back().pose.x, goal.x, 1.0e-9);
  EXPECT_NEAR(path->samples.back().pose.y, goal.y, 1.0e-9);
  EXPECT_NEAR(path->samples.back().pose.yaw, goal.yaw, 1.0e-9);
}

TEST(ElevatorScopedPath, SupportsMeasuredLateralDominantLandingResidual)
{
  ElevatorScopedPathParameters parameters;
  const ElevatorScopedPose start{
    -2.615766, 0.377018, 0.435927};
  const ElevatorScopedPose goal{
    -2.262421, -0.109094, 0.105833533};

  const auto path = make_elevator_scoped_path(start, goal, parameters);

  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->forward_m, 0.300, 0.002);
  EXPECT_NEAR(path->lateral_m, -0.521, 0.002);
  EXPECT_NEAR(path->samples.back().pose.x, goal.x, 1.0e-9);
  EXPECT_NEAR(path->samples.back().pose.y, goal.y, 1.0e-9);
}

TEST(ElevatorScopedPath, LabelsNegativeForwardTravelAsReverse)
{
  ElevatorScopedPathParameters parameters;
  parameters.translation_step_m = 0.10;
  const auto path = make_elevator_scoped_path(
    ElevatorScopedPose{0.0, 0.0, 0.0},
    ElevatorScopedPose{-0.30, 0.0, 0.0}, parameters);

  ASSERT_TRUE(path.has_value());
  ASSERT_GT(path->samples.size(), 1U);
  for (std::size_t index = 1U; index < path->samples.size(); ++index) {
    EXPECT_EQ(path->samples[index].phase, ElevatorScopedMotionPhase::kReverse);
  }
}

TEST(ElevatorScopedPath, RejectsOutOfScopeOrInvalidRequests)
{
  ElevatorScopedPathParameters parameters;
  EXPECT_FALSE(
    make_elevator_scoped_path(
      ElevatorScopedPose{0.0, 0.0, 0.0},
      ElevatorScopedPose{2.51, 0.0, 0.0},
      parameters).has_value());
  EXPECT_FALSE(
    make_elevator_scoped_path(
      ElevatorScopedPose{0.0, 0.0, 0.0},
      ElevatorScopedPose{NAN, 0.0, 0.0},
      parameters).has_value());

  parameters.translation_step_m = 0.0;
  EXPECT_FALSE(
    make_elevator_scoped_path(
      ElevatorScopedPose{0.0, 0.0, 0.0},
      ElevatorScopedPose{1.0, 0.0, 0.0},
      parameters).has_value());
}

}  // namespace
