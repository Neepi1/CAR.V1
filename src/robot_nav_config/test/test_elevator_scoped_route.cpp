#include <gtest/gtest.h>

#include "robot_nav_config/elevator_scoped_route.hpp"

namespace
{

using robot_nav_config::ElevatorScopedPose;
using robot_nav_config::ElevatorScopedRoutePhase;
using robot_nav_config::ElevatorScopedRouteSegment;
using robot_nav_config::ensure_elevator_scoped_route_boundary;
using robot_nav_config::make_elevator_scoped_route_segments;
using robot_nav_config::make_elevator_scoped_command_projection;
using robot_nav_config::make_elevator_scoped_segment_error;
using robot_nav_config::TerminalControlInput;
using robot_nav_config::TerminalControlPhase;
using robot_nav_config::TerminalHandoffParameters;
using robot_nav_config::TerminalPoseError;
using robot_nav_config::TerminalPoseHandoffController;
using robot_nav_config::TerminalVelocityCommand;

TEST(ElevatorScopedRoute, CompressesSearchSamplesIntoExecutableAxisSegments) {
  const std::vector<ElevatorScopedPose> poses{
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.75},
    {0.0, 0.0, 1.5707963267948966},
    {0.0, 0.1, 1.5707963267948966},
    {0.0, 0.2, 1.5707963267948966},
    {0.1, 0.2, 1.5707963267948966},
    {0.2, 0.2, 1.5707963267948966},
    {0.2, 0.1, 1.5707963267948966},
  };

  const auto segments = make_elevator_scoped_route_segments(poses);

  ASSERT_EQ(segments.size(), 5U);
  EXPECT_EQ(segments[0].phase, ElevatorScopedRoutePhase::kYaw);
  EXPECT_EQ(segments[0].end_index, 2U);
  EXPECT_EQ(segments[1].phase, ElevatorScopedRoutePhase::kForward);
  EXPECT_EQ(segments[1].end_index, 4U);
  EXPECT_EQ(segments[2].phase, ElevatorScopedRoutePhase::kLateral);
  EXPECT_EQ(segments[2].end_index, 6U);
  EXPECT_EQ(segments[3].phase, ElevatorScopedRoutePhase::kReverse);
  EXPECT_EQ(segments[3].end_index, 7U);
  EXPECT_EQ(segments[3].direction, -1);
  EXPECT_EQ(segments[4].phase, ElevatorScopedRoutePhase::kPose);
  EXPECT_EQ(segments[4].end_index, 7U);
}

TEST(ElevatorScopedRoute, DistinguishesReverseAndLateralDirectionChanges) {
  const std::vector<ElevatorScopedPose> poses{
    {0.0, 0.0, 0.0}, {-0.1, 0.0, 0.0}, {-0.2, 0.0, 0.0},
    {-0.2, 0.1, 0.0}, {-0.2, 0.2, 0.0}, {-0.2, 0.1, 0.0},
  };

  const auto segments = make_elevator_scoped_route_segments(poses);

  ASSERT_EQ(segments.size(), 4U);
  EXPECT_EQ(segments[0].phase, ElevatorScopedRoutePhase::kReverse);
  EXPECT_EQ(segments[0].end_index, 2U);
  EXPECT_EQ(segments[1].phase, ElevatorScopedRoutePhase::kLateral);
  EXPECT_EQ(segments[1].direction, 1);
  EXPECT_EQ(segments[1].end_index, 4U);
  EXPECT_EQ(segments[2].phase, ElevatorScopedRoutePhase::kLateral);
  EXPECT_EQ(segments[2].direction, -1);
  EXPECT_EQ(segments[2].end_index, 5U);
  EXPECT_EQ(segments[3].phase, ElevatorScopedRoutePhase::kPose);
  EXPECT_EQ(segments[3].end_index, 5U);
}

TEST(ElevatorScopedRoute, KeepsSinglePoseAsFinalTarget) {
  const auto segments = make_elevator_scoped_route_segments(
    std::vector<ElevatorScopedPose>{{1.0, 2.0, 0.3}});

  ASSERT_EQ(segments.size(), 1U);
  EXPECT_EQ(segments.front().phase, ElevatorScopedRoutePhase::kPose);
  EXPECT_EQ(segments.front().end_index, 0U);
}

TEST(ElevatorScopedRoute, RepairRejoinRemainsAnExecutionBoundary) {
  const std::vector<ElevatorScopedPose> poses{
    {0.0, 0.0, 0.0},
    {0.1, 0.0, 0.0},
    {0.2, 0.0, 0.0},
    {0.3, 0.0, 0.0},
  };
  auto segments = robot_nav_config::make_elevator_scoped_route_segments(poses);

  ASSERT_EQ(segments.size(), 2U);
  ASSERT_EQ(segments[0].phase, ElevatorScopedRoutePhase::kForward);
  ASSERT_EQ(segments[0].end_index, 3U);

  ASSERT_TRUE(ensure_elevator_scoped_route_boundary(segments, 2U));
  ASSERT_EQ(segments.size(), 3U);
  EXPECT_EQ(segments[0].phase, ElevatorScopedRoutePhase::kForward);
  EXPECT_EQ(segments[0].end_index, 2U);
  EXPECT_EQ(segments[1].phase, ElevatorScopedRoutePhase::kForward);
  EXPECT_EQ(segments[1].end_index, 3U);
  EXPECT_EQ(segments[2].phase, ElevatorScopedRoutePhase::kPose);
}

TEST(
  ElevatorScopedRoute,
  PlannedForwardSegmentCannotBeOverwrittenByLateralResidual) {
  const TerminalPoseError raw_error{0.32, 0.30, 0.11, 0.0};
  const ElevatorScopedRouteSegment planned_forward{
    ElevatorScopedRoutePhase::kForward, 42U, 1};

  const auto segment_error =
    make_elevator_scoped_segment_error(raw_error, planned_forward);

  EXPECT_NEAR(segment_error.distance_m, 0.30, 1.0e-9);
  EXPECT_NEAR(segment_error.forward_m, 0.30, 1.0e-9);
  EXPECT_NEAR(segment_error.lateral_m, 0.0, 1.0e-9)
    << "a lateral residual must not replace the planner's forward edge";
  EXPECT_NEAR(segment_error.yaw_rad, 0.0, 1.0e-9);

  TerminalHandoffParameters parameters;
  parameters.settle_stable_duration_sec = 0.0;
  TerminalPoseHandoffController controller(parameters);
  controller.begin(0.0);
  TerminalControlInput input;
  input.error = segment_error;
  input.now_sec = 0.0;
  const auto output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kForward);
  EXPECT_GT(output.command.linear_x, 0.0);
  EXPECT_NEAR(output.command.linear_y, 0.0, 1.0e-9);
}

TEST(
  ElevatorScopedRoute,
  PlannedLateralSegmentCannotBeOverwrittenByForwardResidual) {
  const TerminalPoseError raw_error{0.32, 0.30, -0.11, 0.0};
  const ElevatorScopedRouteSegment planned_lateral{
    ElevatorScopedRoutePhase::kLateral, 17U, -1};

  const auto segment_error =
    make_elevator_scoped_segment_error(raw_error, planned_lateral);

  EXPECT_NEAR(segment_error.distance_m, 0.11, 1.0e-9);
  EXPECT_NEAR(segment_error.forward_m, 0.0, 1.0e-9)
    << "a forward residual must not replace the planner's lateral edge";
  EXPECT_NEAR(segment_error.lateral_m, -0.11, 1.0e-9);
  EXPECT_NEAR(segment_error.yaw_rad, 0.0, 1.0e-9);

  TerminalHandoffParameters parameters;
  parameters.settle_stable_duration_sec = 0.0;
  TerminalPoseHandoffController controller(parameters);
  controller.begin(0.0);
  TerminalControlInput input;
  input.error = segment_error;
  input.now_sec = 0.0;
  const auto output = controller.update(input);
  EXPECT_EQ(output.phase, TerminalControlPhase::kLateral);
  EXPECT_LT(output.command.linear_y, 0.0);
  EXPECT_NEAR(output.command.linear_x, 0.0, 1.0e-9);
}

TEST(
  ElevatorScopedRoute,
  SegmentDirectionNeverReversesToChaseAnOvershotEndpoint) {
  const TerminalPoseError overshot_forward{0.08, -0.08, 0.0, 0.0};
  const auto forward_error = make_elevator_scoped_segment_error(
    overshot_forward,
    ElevatorScopedRouteSegment{ElevatorScopedRoutePhase::kForward, 3U, 1});
  EXPECT_NEAR(forward_error.distance_m, 0.0, 1.0e-9);
  EXPECT_NEAR(forward_error.forward_m, 0.0, 1.0e-9);

  const TerminalPoseError overshot_lateral{0.07, 0.0, -0.07, 0.0};
  const auto lateral_error = make_elevator_scoped_segment_error(
    overshot_lateral,
    ElevatorScopedRouteSegment{ElevatorScopedRoutePhase::kLateral, 9U, 1});
  EXPECT_NEAR(lateral_error.distance_m, 0.0, 1.0e-9);
  EXPECT_NEAR(lateral_error.lateral_m, 0.0, 1.0e-9);
}

TEST(
  ElevatorScopedRoute,
  FinalPoseStageRetainsFullResidualForExactConvergence) {
  const TerminalPoseError raw_error{0.32, 0.30, -0.11, 0.07};
  const auto final_error = make_elevator_scoped_segment_error(
    raw_error,
    ElevatorScopedRouteSegment{ElevatorScopedRoutePhase::kPose, 99U, 0});

  EXPECT_NEAR(final_error.distance_m, raw_error.distance_m, 1.0e-9);
  EXPECT_NEAR(final_error.forward_m, raw_error.forward_m, 1.0e-9);
  EXPECT_NEAR(final_error.lateral_m, raw_error.lateral_m, 1.0e-9);
  EXPECT_NEAR(final_error.yaw_rad, raw_error.yaw_rad, 1.0e-9);
}

TEST(
  ElevatorScopedRoute,
  CommandProjectionNeverExtendsPastTheActiveAxisEndpoint) {
  TerminalVelocityCommand reverse_command;
  reverse_command.linear_x = -0.08;
  const TerminalPoseError short_reverse{
    0.112269, -0.112269, 0.0, 0.0};
  const auto reverse_projection = make_elevator_scoped_command_projection(
    short_reverse, reverse_command, 0.15, 0.10);
  EXPECT_NEAR(reverse_projection.translation_m, 0.112269, 1.0e-9)
    << "the 15 cm probe must not inspect beyond an 11.23 cm route segment";
  EXPECT_NEAR(reverse_projection.rotation_rad, 0.0, 1.0e-9);

  TerminalVelocityCommand lateral_command;
  lateral_command.linear_y = 0.05;
  const TerminalPoseError long_lateral{0.430105, 0.0, 0.430105, 0.0};
  const auto lateral_projection = make_elevator_scoped_command_projection(
    long_lateral, lateral_command, 0.15, 0.10);
  EXPECT_NEAR(lateral_projection.translation_m, 0.15, 1.0e-9);

  TerminalVelocityCommand yaw_command;
  yaw_command.angular_z = 0.08;
  const TerminalPoseError short_yaw{0.0, 0.0, 0.0, 0.027780};
  const auto yaw_projection = make_elevator_scoped_command_projection(
    short_yaw, yaw_command, 0.15, 0.10);
  EXPECT_NEAR(yaw_projection.translation_m, 0.0, 1.0e-9);
  EXPECT_NEAR(yaw_projection.rotation_rad, 0.027780, 1.0e-9);
}

} // namespace
