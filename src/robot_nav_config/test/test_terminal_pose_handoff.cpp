#include <limits>

#include "robot_nav_config/terminal_pose_handoff.hpp"
#include "gtest/gtest.h"

namespace robot_nav_config
{
namespace
{

TerminalPoseError observed_terminal_error()
{
  TerminalPoseError error;
  error.distance_m = 0.354192;
  error.forward_m = 0.126303;
  error.lateral_m = 0.330907;
  error.yaw_rad = 0.02;
  return error;
}

TEST(TerminalPoseHandoff, ObservedAckermannHairpinTriggersNormalHandoff) {
  TerminalHandoffParameters parameters;
  TerminalPathMetrics path;
  path.chord_m = 0.4031;
  path.length_m = 1.3703;
  path.max_cross_track_m = 0.2722;

  EXPECT_TRUE(
    should_start_terminal_handoff(
      observed_terminal_error(), path,
      parameters));
}

TEST(TerminalPoseHandoff, ReachableTerminalPathRemainsWithMppi) {
  TerminalHandoffParameters parameters;
  auto error = observed_terminal_error();
  error.distance_m = 0.35;
  error.forward_m = 0.33;
  error.lateral_m = 0.10;
  TerminalPathMetrics path;
  path.chord_m = 0.36;
  path.length_m = 0.39;
  path.max_cross_track_m = 0.03;

  EXPECT_FALSE(should_start_terminal_handoff(error, path, parameters));
}

TEST(TerminalPoseHandoff, LivePureLateralResidualTriggersWithoutHairpin) {
  TerminalHandoffParameters parameters;
  TerminalPoseError error;
  error.distance_m = 0.1353;
  error.forward_m = -0.0253;
  error.lateral_m = 0.1329;
  error.yaw_rad = 0.0431;

  TerminalPathMetrics path;
  path.chord_m = 0.12;
  path.length_m = 0.126;
  path.max_cross_track_m = 0.01;

  EXPECT_TRUE(should_start_terminal_handoff(error, path, parameters));
}

TEST(TerminalPoseHandoff, ResidualOutsideRecoveryEnvelopeFailsClosed) {
  TerminalHandoffParameters parameters;
  auto error = observed_terminal_error();
  error.distance_m = 0.41;
  TerminalPathMetrics path;
  path.chord_m = 0.41;
  path.length_m = 1.50;
  path.max_cross_track_m = 0.30;

  EXPECT_FALSE(should_start_terminal_handoff(error, path, parameters));
}

TEST(TerminalPoseHandoff, PureLongitudinalOvershootStartsAndCompletesPermittedReverse) {
  TerminalHandoffParameters parameters;
  parameters.minimum_abs_lateral_m = 0.06;
  TerminalPoseError error{0.096, -0.096, 0.0, 0.0};
  TerminalPathMetrics path{0.096, 0.096, 0.0};
  // Exercise admission before control: a controller-only reverse test misses
  // the bug where a pure longitudinal overshoot never enters this controller.
  ASSERT_TRUE(should_start_terminal_handoff(error, path, parameters));

  TerminalPoseHandoffController controller(parameters);
  controller.begin(0.0);
  TerminalControlInput input;
  input.error = error;
  bool saw_reverse = false;
  bool complete = false;
  constexpr double dt = 0.05;
  for (int step = 0; step < 200; ++step) {
    input.now_sec = step * dt;
    const auto output = controller.update(input);
    ASSERT_FALSE(output.failed);
    EXPECT_DOUBLE_EQ(output.command.linear_y, 0.0);
    EXPECT_DOUBLE_EQ(output.command.angular_z, 0.0);
    EXPECT_GE(output.command.linear_x, -parameters.reverse_max_speed_mps);
    EXPECT_LE(output.command.linear_x, 0.0);
    EXPECT_EQ(output.reverse_permit, output.command.linear_x < 0.0);
    saw_reverse = saw_reverse || output.reverse_permit;
    if (output.complete) {
      complete = true;
      EXPECT_LE(input.error.distance_m, parameters.goal_xy_tolerance_m);
      break;
    }
    input.error.forward_m -= output.command.linear_x * dt;
    input.error.distance_m = std::abs(input.error.forward_m);
    input.actual_linear_x_mps = output.command.linear_x;
  }
  EXPECT_TRUE(saw_reverse);
  EXPECT_TRUE(complete);
}

TEST(TerminalPoseHandoff, MixedOvershootBelowLateralThresholdStillStarts) {
  TerminalHandoffParameters parameters;
  parameters.minimum_abs_lateral_m = 0.06;
  TerminalPoseError error{std::hypot(0.05, 0.05), -0.05, 0.05, 0.0};
  TerminalPathMetrics path{error.distance_m, error.distance_m, 0.0};
  EXPECT_TRUE(should_start_terminal_handoff(error, path, parameters));
}

TEST(TerminalPoseHandoff, OvershootDoesNotWidenExistingAdmissionOrGoalTolerance) {
  TerminalHandoffParameters parameters;
  parameters.minimum_abs_lateral_m = 0.06;
  TerminalPathMetrics path{0.096, 0.096, 0.0};
  EXPECT_FALSE(should_start_terminal_handoff(
    TerminalPoseError{0.06, -0.06, 0.0, 0.0}, path, parameters));
  EXPECT_FALSE(should_start_terminal_handoff(
    TerminalPoseError{0.096, 0.096, 0.0, 0.0}, path, parameters));
  EXPECT_FALSE(should_start_terminal_handoff(
    TerminalPoseError{0.151, -0.151, 0.0, 0.0}, path, parameters));
  EXPECT_FALSE(should_start_terminal_handoff(
    TerminalPoseError{4.83, -0.10, 4.829, 0.0}, path, parameters));
  parameters.enabled = false;
  EXPECT_FALSE(should_start_terminal_handoff(
    TerminalPoseError{0.096, -0.096, 0.0, 0.0}, path, parameters));
}

TEST(TerminalPoseHandoff, AxisStagesKeepTaskRunningUntilStrictGoalAcceptance) {
  TerminalHandoffParameters parameters;
  TerminalPoseHandoffController controller(parameters);
  controller.begin(0.0);

  TerminalControlInput input;
  input.error = observed_terminal_error();
  input.error.yaw_rad = 0.10;
  input.now_sec = 0.0;
  auto output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kSettling);
  EXPECT_TRUE(output.active);
  EXPECT_FALSE(output.complete);

  input.now_sec = 0.31;
  output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kYaw);
  EXPECT_NE(output.command.angular_z, 0.0);
  EXPECT_EQ(output.command.linear_x, 0.0);
  EXPECT_EQ(output.command.linear_y, 0.0);
  EXPECT_FALSE(output.lateral_permit);

  input.error.yaw_rad = 0.01;
  input.now_sec = 0.40;
  output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kSettling);
  EXPECT_FALSE(output.complete);

  input.now_sec = 0.71;
  output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kLateral);
  EXPECT_EQ(output.command.linear_x, 0.0);
  EXPECT_NE(output.command.linear_y, 0.0);
  EXPECT_EQ(output.command.angular_z, 0.0);
  EXPECT_TRUE(output.lateral_permit);

  input.error.lateral_m = 0.01;
  input.now_sec = 0.80;
  output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kSettling);
  EXPECT_FALSE(output.complete);

  input.now_sec = 1.11;
  output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kForward);
  EXPECT_NE(output.command.linear_x, 0.0);
  EXPECT_EQ(output.command.linear_y, 0.0);
  EXPECT_EQ(output.command.angular_z, 0.0);
  EXPECT_FALSE(output.lateral_permit);

  input.error.distance_m = 0.02;
  input.error.forward_m = 0.01;
  input.now_sec = 1.20;
  output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kSettling);
  EXPECT_FALSE(output.complete);

  input.now_sec = 1.51;
  output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kComplete);
  EXPECT_TRUE(output.complete);
  EXPECT_FALSE(output.failed);
}

TEST(TerminalPoseHandoff, LocalizationCorrectionAfterCompletionReopensControl) {
  TerminalHandoffParameters parameters;
  parameters.settle_stable_duration_sec = 0.30;
  TerminalPoseHandoffController controller(parameters);
  controller.begin(0.0);

  TerminalControlInput input;
  input.now_sec = 0.31;
  auto output = controller.update(input);
  ASSERT_EQ(output.phase, TerminalControlPhase::kSettling);
  input.now_sec = 0.62;
  output = controller.update(input);
  ASSERT_EQ(output.phase, TerminalControlPhase::kComplete);
  ASSERT_TRUE(output.complete);

  // A later map->odom correction moves the canonical map goal in the local
  // control frame. Completion must not remain latched on the stale pose.
  input.error.distance_m = 0.12;
  input.error.forward_m = 0.12;
  input.now_sec = 0.70;
  output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kSettling);
  EXPECT_FALSE(output.complete);

  input.now_sec = 1.01;
  output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kForward);
  EXPECT_GT(output.command.linear_x, 0.0);
}

TEST(TerminalPoseHandoff, ReportsTimeoutAsTheExactFailureReason) {
  TerminalHandoffParameters parameters;
  parameters.total_timeout_sec = 1.0;
  TerminalPoseHandoffController controller(parameters);
  controller.begin(10.0);

  TerminalControlInput input;
  input.now_sec = 11.01;
  const auto output = controller.update(input);

  ASSERT_TRUE(output.failed);
  EXPECT_EQ(output.failure_reason, TerminalControlFailureReason::kTimeout);
}

TEST(TerminalPoseHandoff, SeparatesInvalidInputFromClockRegression) {
  TerminalPoseHandoffController invalid_controller;
  invalid_controller.begin(10.0);
  TerminalControlInput invalid_input;
  invalid_input.now_sec = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_output = invalid_controller.update(invalid_input);
  ASSERT_TRUE(invalid_output.failed);
  EXPECT_EQ(invalid_output.failure_reason,
            TerminalControlFailureReason::kInvalidInput);

  TerminalPoseHandoffController clock_controller;
  clock_controller.begin(10.0);
  TerminalControlInput regressed_input;
  regressed_input.now_sec = 9.9;
  const auto regressed_output = clock_controller.update(regressed_input);
  ASSERT_TRUE(regressed_output.failed);
  EXPECT_EQ(regressed_output.failure_reason,
            TerminalControlFailureReason::kClockRegression);
}

} // namespace
} // namespace robot_nav_config
