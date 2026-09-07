#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "robot_api_server/features/navigation/terminal_control/navigation_terminal_control.hpp"

namespace navigation = robot_api_server::features::navigation;

namespace
{

class FakeTerminalRuntime : public navigation::NavigationTerminalRuntimePort
{
public:
  navigation::TerminalTimePoint terminal_now() const override {return now;}

  void terminal_sleep_for(const std::chrono::milliseconds duration) override
  {
    now += duration;
  }

  bool terminal_cancel_requested(std::uint64_t, std::string &) override {return false;}
  bool terminal_safety_hard_blocked(std::string &) override {return false;}

  navigation::FinalPoseCheck terminal_verify_final_pose(
    const robot_api_server::StoredPose &,
    bool) override
  {
    if (checks.empty()) {
      return navigation::FinalPoseCheck{};
    }
    const auto result = checks.front();
    checks.pop_front();
    return result;
  }

  void terminal_update_final_pose(
    std::uint64_t,
    const navigation::FinalPoseCheck &,
    const std::string &) override {}

  void terminal_set_job_phase(
    std::uint64_t,
    const std::string & phase,
    const std::string &) override
  {
    phases.push_back(phase);
  }

  void terminal_publish_command(const geometry_msgs::msg::Twist & command) override
  {
    commands.push_back(command);
  }

  void terminal_publish_motion_mode(const std::string & mode) override
  {
    modes.push_back(mode);
  }

  bool terminal_reverse_permit_available() const override {return true;}

  void terminal_publish_reverse_permit(bool enabled) override
  {
    reverse_permits.push_back(enabled);
  }

  void terminal_reset_actual_stop_stability() override {}

  bool terminal_wait_actual_stop(const std::string &, std::string & detail) override
  {
    detail = "settled";
    return true;
  }

  navigation::TerminalCostmapContext terminal_costmap_context(
    navigation::TerminalTimePoint) override
  {
    navigation::TerminalCostmapContext context;
    context.grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    context.grid_received_at = now;
    context.robot_pose_available = true;
    context.robot_pose_received_at = now;
    context.grid_frame = "base_link";
    context.grid->info.resolution = 0.05;
    context.grid->info.width = 40;
    context.grid->info.height = 40;
    context.grid->info.origin.position.x = -1.0;
    context.grid->info.origin.position.y = -1.0;
    context.grid->info.origin.orientation.w = 1.0;
    context.grid->data.assign(1600, 0);
    return context;
  }

  robot_api_server::RobotPoseSnapshot terminal_current_robot_pose() override
  {
    if (robot_poses.empty()) {
      return robot_api_server::RobotPoseSnapshot{};
    }
    const auto result = robot_poses.front();
    robot_poses.pop_front();
    return result;
  }

  bool terminal_dock_contact_blocked(std::string &) override {return false;}
  void terminal_reset_yaw_actual_stop_stability() override {}

  bool terminal_wait_yaw_actual_stop(const std::string &, std::string & detail) override
  {
    detail = "angular velocity settled";
    return true;
  }

  void terminal_warn(const std::string & warning) override
  {
    warnings.push_back(warning);
  }

  navigation::TerminalTimePoint now{
    navigation::TerminalTimePoint{} + std::chrono::seconds(1)};
  std::deque<navigation::FinalPoseCheck> checks;
  std::deque<robot_api_server::RobotPoseSnapshot> robot_poses;
  std::vector<geometry_msgs::msg::Twist> commands;
  std::vector<std::string> modes;
  std::vector<std::string> phases;
  std::vector<bool> reverse_permits;
  std::vector<std::string> warnings;
};

navigation::FinalPoseCheck pose_check(
  const double x,
  const double y,
  const double yaw,
  const double target_x,
  const double target_y,
  const double target_yaw)
{
  navigation::FinalPoseCheck check;
  check.pose_available = true;
  check.pose.available = true;
  check.pose.x = x;
  check.pose.y = y;
  check.pose.yaw = yaw;
  check.distance_m = std::hypot(target_x - x, target_y - y);
  check.yaw_error_rad = std::fabs(target_yaw - yaw);
  check.position_reached = check.distance_m <= 0.06;
  return check;
}

}  // namespace

TEST(NavigationTerminalControl, KeepsCommercialTerminalSpeedBands)
{
  navigation::TerminalControlConfig config;
  config.speed_limit_far_distance_m = 2.0;
  config.speed_limit_mid_distance_m = 1.2;
  config.speed_limit_near_distance_m = 0.6;
  config.speed_limit_crawl_distance_m = 0.15;
  config.speed_limit_far_mps = 1.20;
  config.speed_limit_mid_mps = 0.70;
  config.speed_limit_near_mps = 0.40;
  config.speed_limit_crawl_mps = 0.25;
  config.speed_limit_final_mps = 0.10;

  navigation::NavigationTerminalControl control(config);
  EXPECT_DOUBLE_EQ(control.speed_limit_for_distance(2.01), 1.20);
  EXPECT_DOUBLE_EQ(control.speed_limit_for_distance(2.00), 0.70);
  EXPECT_DOUBLE_EQ(control.speed_limit_for_distance(1.20), 0.40);
  EXPECT_DOUBLE_EQ(control.speed_limit_for_distance(0.60), 0.25);
  EXPECT_DOUBLE_EQ(control.speed_limit_for_distance(0.15), 0.10);
}

TEST(NavigationTerminalControl, PreservesLateralEntryAndCompletionHysteresis)
{
  navigation::TerminalControlConfig config;
  config.final_verify_enabled = true;
  config.api_velocity_correction_enabled = true;
  config.lateral_correction_enabled = true;
  config.goal_position_success_tolerance_m = 0.06;
  config.terminal_recovery_max_distance_m = 0.35;
  config.lateral_target_m = 0.03;
  config.lateral_trigger_m = 0.04;
  config.lateral_max_forward_m = 0.15;

  navigation::NavigationTerminalControl control(config);
  robot_api_server::StoredPose target;
  target.x = 0.0;
  target.y = 0.05;
  navigation::FinalPoseCheck check;
  check.pose_available = true;
  check.pose.available = true;
  check.pose.x = 0.0;
  check.pose.y = 0.0;
  check.pose.yaw = 0.0;
  check.distance_m = 0.05;
  check.yaw_error_rad = 0.0;

  const auto gate = control.lateral_correction_gate(target, check, "pose_required");
  EXPECT_TRUE(gate.allowed);
  EXPECT_NEAR(gate.forward_m, 0.0, 1e-12);
  EXPECT_NEAR(gate.lateral_m, 0.05, 1e-12);

  target.y = 0.035;
  check.distance_m = 0.035;
  const auto inside_hysteresis =
    control.lateral_correction_gate(target, check, "pose_required");
  EXPECT_FALSE(inside_hysteresis.allowed);
}

TEST(NavigationTerminalControl, ChoosesYawThenLateralThenForward)
{
  navigation::TerminalControlConfig config;
  config.goal_position_success_tolerance_m = 0.06;
  config.final_yaw_success_tolerance_rad = 0.045;
  config.lateral_target_m = 0.03;
  config.lateral_speed_mps = 0.04;
  config.lateral_kp = 0.8;
  config.final_yaw_kp = 1.8;
  config.final_yaw_min_speed_radps = 0.10;
  config.final_yaw_max_speed_radps = 0.50;
  config.final_yaw_slow_max_speed_radps = 0.22;
  config.final_yaw_slowdown_start_rad = 0.16;

  navigation::NavigationTerminalControl control(config);
  auto step = control.correction_step(0.10, 0.08, 0.20, 1.0);
  EXPECT_EQ(step.axis, navigation::TerminalCorrectionAxis::kYaw);
  EXPECT_GT(step.command.angular.z, 0.0);
  EXPECT_DOUBLE_EQ(step.command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(step.command.linear.y, 0.0);

  step = control.correction_step(0.10, 0.08, 0.01, 1.0);
  EXPECT_EQ(step.axis, navigation::TerminalCorrectionAxis::kLateral);
  EXPECT_GT(step.command.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(step.command.linear.x, 0.0);

  step = control.correction_step(0.10, 0.01, 0.01, 1.0);
  EXPECT_EQ(step.axis, navigation::TerminalCorrectionAxis::kForward);
  EXPECT_GT(step.command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(step.command.linear.y, 0.0);
}

TEST(NavigationTerminalControl, CostmapGuardFailsClosedAndReportsOccupiedCell)
{
  navigation::TerminalControlConfig config;
  config.costmap_guard_enabled = true;
  config.costmap_max_age_sec = 0.50;
  config.costmap_occupied_threshold = 50;
  navigation::NavigationTerminalControl control(config);

  const auto now = std::chrono::steady_clock::now();
  navigation::TerminalCostmapContext missing;
  auto result = control.costmap_path_clear(missing, 0.1, 0.0, now);
  EXPECT_FALSE(result.clear);
  EXPECT_NE(result.detail.find("unavailable"), std::string::npos);

  navigation::TerminalCostmapContext context;
  context.grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  context.grid_received_at = now;
  context.robot_pose_available = true;
  context.robot_pose_received_at = now;
  context.grid_frame = "base_link";
  context.grid->header.frame_id = "base_link";
  context.grid->info.resolution = 0.05;
  context.grid->info.width = 20;
  context.grid->info.height = 20;
  context.grid->info.origin.position.x = -0.5;
  context.grid->info.origin.position.y = -0.5;
  context.grid->info.origin.orientation.w = 1.0;
  context.grid->data.assign(400, 0);

  result = control.costmap_path_clear(context, 0.15, 0.0, now);
  EXPECT_TRUE(result.clear) << result.detail;

  std::fill(context.grid->data.begin(), context.grid->data.end(), 100);
  result = control.costmap_path_clear(context, 0.15, 0.0, now);
  EXPECT_FALSE(result.clear);
  EXPECT_NE(result.detail.find("cost=100"), std::string::npos);
}

TEST(NavigationTerminalControl, OwnsLateralExecutionAndPhysicalSettleSequence)
{
  navigation::TerminalControlConfig config;
  config.final_verify_enabled = true;
  config.api_velocity_correction_enabled = true;
  config.lateral_correction_enabled = true;
  config.reverse_permit_enabled = true;
  config.goal_position_success_tolerance_m = 0.06;
  config.terminal_recovery_max_distance_m = 0.40;
  config.final_yaw_success_tolerance_rad = 0.045;
  config.lateral_target_m = 0.03;
  config.lateral_trigger_m = 0.04;
  config.lateral_max_forward_m = 0.15;
  config.lateral_speed_mps = 0.04;
  config.lateral_kp = 0.8;
  config.lateral_timeout_sec = 2.0;
  config.zero_command_count = 2;

  navigation::NavigationTerminalControl control(config);
  FakeTerminalRuntime runtime;
  robot_api_server::StoredPose target;
  target.x = 0.0;
  target.y = 0.05;
  target.yaw = 0.0;
  const auto initial = pose_check(0.0, 0.0, 0.0, target.x, target.y, target.yaw);
  runtime.checks.push_back(initial);
  runtime.checks.push_back(pose_check(0.0, 0.05, 0.0, target.x, target.y, target.yaw));
  runtime.checks.push_back(pose_check(0.0, 0.05, 0.0, target.x, target.y, target.yaw));
  runtime.checks.push_back(pose_check(0.0, 0.05, 0.0, target.x, target.y, target.yaw));

  const auto result = control.run_lateral_correction(
    7U, target, initial, "pose_required", runtime);

  EXPECT_TRUE(result.attempted);
  EXPECT_TRUE(result.succeeded) << result.detail;
  EXPECT_TRUE(result.settle_confirmed);
  ASSERT_FALSE(runtime.commands.empty());
  const auto nonzero = std::find_if(
    runtime.commands.begin(), runtime.commands.end(),
    [](const auto & command) {return std::fabs(command.linear.y) > 1e-6;});
  ASSERT_NE(nonzero, runtime.commands.end());
  EXPECT_GT(nonzero->linear.y, 0.0);
  EXPECT_NE(std::find(runtime.modes.begin(), runtime.modes.end(), "side_slip"), runtime.modes.end());
  EXPECT_EQ(runtime.modes.back(), "auto");
  EXPECT_NE(std::find(runtime.reverse_permits.begin(), runtime.reverse_permits.end(), true),
    runtime.reverse_permits.end());
  EXPECT_FALSE(runtime.reverse_permits.back());
}

TEST(NavigationTerminalControl, OwnsFinalYawMotionAndSettledPoseRecheck)
{
  navigation::TerminalControlConfig config;
  config.final_yaw_success_tolerance_rad = 0.045;
  config.final_yaw_kp = 1.2;
  config.final_yaw_min_speed_radps = 0.06;
  config.final_yaw_max_speed_radps = 0.60;
  config.final_yaw_slow_max_speed_radps = 0.20;
  config.final_yaw_slowdown_start_rad = 0.30;
  config.final_yaw_timeout_sec = 2.0;
  config.final_yaw_max_xy_drift_m = 0.08;
  config.final_yaw_require_fresh_pose = true;
  config.robot_pose_freshness_sec = 0.5;
  config.zero_command_count = 2;

  navigation::NavigationTerminalControl control(config);
  FakeTerminalRuntime runtime;
  robot_api_server::StoredPose target;
  target.x = 1.0;
  target.y = 2.0;
  target.yaw = 0.10;
  const auto initial = pose_check(1.0, 2.0, 0.0, target.x, target.y, target.yaw);

  auto moving_pose = initial.pose;
  moving_pose.available = true;
  moving_pose.frame_id = "map";
  moving_pose.age_sec = 0.01;
  runtime.robot_poses.push_back(moving_pose);
  auto inside_stop_lead = moving_pose;
  inside_stop_lead.yaw = 0.06;
  runtime.robot_poses.push_back(inside_stop_lead);
  auto settled_pose = moving_pose;
  settled_pose.yaw = target.yaw;
  runtime.robot_poses.push_back(settled_pose);

  const auto result = control.run_final_yaw_motion(9U, target, initial, runtime);

  EXPECT_TRUE(result.attempted);
  EXPECT_TRUE(result.succeeded) << result.detail;
  EXPECT_LE(result.final_yaw_error_rad, config.final_yaw_success_tolerance_rad);
  const auto nonzero = std::find_if(
    runtime.commands.begin(), runtime.commands.end(),
    [](const auto & command) {return std::fabs(command.angular.z) > 1e-6;});
  ASSERT_NE(nonzero, runtime.commands.end());
  EXPECT_GT(nonzero->angular.z, 0.0);
}
