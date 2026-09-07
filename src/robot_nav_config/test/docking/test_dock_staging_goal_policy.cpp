#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "robot_nav_config/docking/dock_staging_goal_policy.hpp"

namespace
{

using robot_nav_config::DockStagingGoalLimits;
using robot_nav_config::DockStagingGoalPolicy;
using robot_nav_config::DockStagingGoalSample;

constexpr double kPi = 3.14159265358979323846;

TEST(DockStagingGoalPolicy, AcceptsCommissionedCaptureEnvelopeWithoutExactGoal)
{
  const DockStagingGoalPolicy policy({-0.40, 0.55, 0.25, kPi});

  const auto result = policy.assess({
    1.30, 1.20, 0.75,
    1.00, 1.00, 0.0});

  EXPECT_TRUE(result.reached);
  EXPECT_NEAR(result.forward_error_m, 0.30, 1e-9);
  EXPECT_NEAR(result.lateral_error_m, 0.20, 1e-9);
  EXPECT_NEAR(result.yaw_error_rad, 0.75, 1e-9);
}

TEST(DockStagingGoalPolicy, ProjectsErrorsInCommissionedGoalHeading)
{
  const DockStagingGoalPolicy policy({-0.40, 0.55, 0.25, kPi});

  const auto result = policy.assess({
    1.80, 2.30, -1.2,
    2.00, 2.00, kPi * 0.5});

  EXPECT_TRUE(result.reached);
  EXPECT_NEAR(result.forward_error_m, 0.30, 1e-9);
  EXPECT_NEAR(result.lateral_error_m, 0.20, 1e-9);
}

TEST(DockStagingGoalPolicy, RejectsOutsideEachBound)
{
  const DockStagingGoalPolicy policy({-0.40, 0.55, 0.25, 0.35});

  EXPECT_FALSE(policy.assess({0.0, 0.251, 0.0, 0.0, 0.0, 0.0}).reached);
  EXPECT_FALSE(policy.assess({0.551, 0.0, 0.0, 0.0, 0.0, 0.0}).reached);
  EXPECT_FALSE(policy.assess({-0.401, 0.0, 0.0, 0.0, 0.0, 0.0}).reached);
  EXPECT_FALSE(policy.assess({0.0, 0.0, 0.351, 0.0, 0.0, 0.0}).reached);
}

TEST(DockStagingGoalPolicy, RejectsInvalidConfigurationAndSamples)
{
  EXPECT_THROW(
    DockStagingGoalPolicy(DockStagingGoalLimits{0.10, -0.10, 0.25, 0.35}),
    std::invalid_argument);
  EXPECT_THROW(
    DockStagingGoalPolicy(DockStagingGoalLimits{-0.40, 0.55, -0.01, 0.35}),
    std::invalid_argument);

  const DockStagingGoalPolicy policy({-0.40, 0.55, 0.25, 0.35});
  auto sample = DockStagingGoalSample{};
  sample.current_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(policy.assess(sample).valid);
  EXPECT_FALSE(policy.assess(sample).reached);
}

}  // namespace
