#include "gtest/gtest.h"
#include "robot_nav_config/terminal_pose_handoff.hpp"

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

TEST(TerminalPoseHandoff, ObservedAckermannHairpinTriggersNormalHandoff)
{
  TerminalHandoffParameters parameters;
  TerminalPathMetrics path;
  path.chord_m = 0.4031;
  path.length_m = 1.3703;
  path.max_cross_track_m = 0.2722;

  EXPECT_TRUE(should_start_terminal_handoff(observed_terminal_error(), path, parameters));
}

TEST(TerminalPoseHandoff, ReachableTerminalPathRemainsWithMppi)
{
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

TEST(TerminalPoseHandoff, LivePureLateralResidualTriggersWithoutHairpin)
{
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

TEST(TerminalPoseHandoff, ResidualOutsideRecoveryEnvelopeFailsClosed)
{
  TerminalHandoffParameters parameters;
  auto error = observed_terminal_error();
  error.distance_m = 0.41;
  TerminalPathMetrics path;
  path.chord_m = 0.41;
  path.length_m = 1.50;
  path.max_cross_track_m = 0.30;

  EXPECT_FALSE(should_start_terminal_handoff(error, path, parameters));
}

TEST(TerminalPoseHandoff, AxisStagesKeepTaskRunningUntilStrictGoalAcceptance)
{
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

}  // namespace
}  // namespace robot_nav_config
