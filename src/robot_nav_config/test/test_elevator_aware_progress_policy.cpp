#include <gtest/gtest.h>

#include "robot_nav_config/elevator_aware_progress_policy.hpp"
#include "robot_nav_config/elevator_scoped_progress_state.hpp"

namespace {

using robot_nav_config::ElevatorAwareProgressPolicy;
using robot_nav_config::ElevatorScopedProgressState;
using robot_nav_config::PoseProgressLimits;
using robot_nav_config::PoseProgressSample;

TEST(ElevatorAwareProgressPolicy, CountsFineYawOnlyForElevatorScopedMotion) {
  const PoseProgressLimits normal{0.03, 0.05, 12.0};
  const PoseProgressLimits elevator{0.015, 0.015, 20.0};
  const PoseProgressSample origin{0.0, 0.0, 0.0};
  const PoseProgressSample field_correction{0.0, 0.0, 0.0407};

  ElevatorAwareProgressPolicy normal_policy(normal, elevator);
  EXPECT_TRUE(normal_policy.check(origin, 0.0, false));
  EXPECT_FALSE(normal_policy.check(field_correction, 12.01, false));

  ElevatorAwareProgressPolicy elevator_policy(normal, elevator);
  EXPECT_TRUE(elevator_policy.check(origin, 0.0, true));
  EXPECT_TRUE(elevator_policy.check(field_correction, 12.01, true));
  EXPECT_TRUE(elevator_policy.check(field_correction, 31.99, true));
  EXPECT_FALSE(elevator_policy.check(field_correction, 32.02, true));
}

TEST(ElevatorAwareProgressPolicy, ExplicitWaitDoesNotConsumeMotionAllowance) {
  const PoseProgressLimits normal{0.03, 0.05, 12.0};
  const PoseProgressLimits elevator{0.015, 0.015, 20.0};
  const PoseProgressSample origin{0.0, 0.0, 0.0};

  ElevatorAwareProgressPolicy policy(normal, elevator);
  EXPECT_TRUE(
      policy.check(origin, 0.0, ElevatorScopedProgressState::kTracking));
  EXPECT_TRUE(
      policy.check(origin, 25.0, ElevatorScopedProgressState::kWaitClear));
  EXPECT_TRUE(
      policy.check(origin, 50.0, ElevatorScopedProgressState::kReplanning));

  // Resuming motion starts a fresh progress window instead of immediately
  // inheriting the time spent deliberately stopped.
  EXPECT_TRUE(
      policy.check(origin, 50.1, ElevatorScopedProgressState::kTracking));
  EXPECT_TRUE(
      policy.check(origin, 70.0, ElevatorScopedProgressState::kTracking));
  EXPECT_FALSE(
      policy.check(origin, 70.2, ElevatorScopedProgressState::kTracking));
}

} // namespace
