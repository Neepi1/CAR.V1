#include <string>

#include "gtest/gtest.h"

#include "robot_api_server/features/navigation/mission/navigation_goal_job.hpp"

namespace navigation = robot_api_server::features::navigation;

TEST(NavigationGoalJob, DefaultJsonPreservesAppContract)
{
  const navigation::NavigationGoalJob job;
  const auto json = navigation::navigation_goal_job_json(job);

  EXPECT_NE(json.find("\"id\":0"), std::string::npos);
  EXPECT_NE(json.find("\"state\":\"idle\""), std::string::npos);
  EXPECT_NE(json.find("\"phase\":\"idle\""), std::string::npos);
  EXPECT_NE(json.find("\"goal_completion_policy\":\"pose_required\""), std::string::npos);
  EXPECT_NE(json.find("\"native_nav2_goal_completion\":true"), std::string::npos);
  EXPECT_NE(json.find("\"final_distance_m\":-1.000000"), std::string::npos);
  EXPECT_NE(json.find("\"task_complete\":false"), std::string::npos);
  EXPECT_EQ(json.find("\"cancel_requested\""), std::string::npos);
}

TEST(NavigationGoalJob, PopulatedJsonKeepsDiagnosticsAndEscapesStrings)
{
  navigation::NavigationGoalJob job;
  job.id = 42U;
  job.state = "running";
  job.phase = "terminal_pose_lateral_correcting";
  job.pose_id = "delivery-1";
  job.detail = "line one\n\"quoted\"";
  job.target_x = 1.25;
  job.target_y = -2.5;
  job.target_yaw = 0.125;
  job.final_verify_retry_count = 2;
  job.final_yaw_align_attempted = true;
  job.final_yaw_align_succeeded = true;
  job.final_pose_verified = true;
  job.task_complete = true;

  const auto json = navigation::navigation_goal_job_json(job);
  EXPECT_NE(json.find("\"id\":42"), std::string::npos);
  EXPECT_NE(json.find("\"phase\":\"terminal_pose_lateral_correcting\""), std::string::npos);
  EXPECT_NE(json.find("line one\\n\\\"quoted\\\""), std::string::npos);
  EXPECT_NE(json.find("\"target\":{\"x\":1.250000,\"y\":-2.500000,\"yaw\":0.125000}"),
    std::string::npos);
  EXPECT_NE(json.find("\"final_verify_retry_count\":2"), std::string::npos);
  EXPECT_NE(json.find("\"final_yaw_align_attempted\":true"), std::string::npos);
  EXPECT_NE(json.find("\"task_complete\":true"), std::string::npos);
}

TEST(NavigationGoalJob, FactoryBuildsAcceptedBackgroundJob)
{
  navigation::NavigationGoalJobStartSpec start;
  start.id = 9U;
  start.pose_id = "delivery-1";
  start.building_id = "B15";
  start.floor_id = "F1";
  start.goal_completion_policy = "pose_required";
  start.native_nav2_goal_completion = true;
  start.api_final_yaw_align_enabled = true;
  start.post_nav2_final_verify_enabled = true;
  start.post_nav2_final_verify_wait_bridge_smoothing = true;
  start.final_verify_retry_max_count = 3;
  start.post_nav2_final_verify_api_velocity_correction_enabled = true;
  start.nav2_rotation_shim_enabled = true;
  start.target_x = 1.0;
  start.target_y = 2.0;
  start.target_yaw = 0.25;
  start.nav2_goal_yaw = 0.20;
  start.nav2_goal_yaw_source = "approach_heading";
  start.final_yaw_align_timeout_sec = 8.0;
  start.final_yaw_align_max_xy_drift_m = 0.08;
  start.final_yaw_align_cmd_topic = "/cmd_vel_api";
  start.final_yaw_align_bypass_collision_monitor = true;
  start.pre_navigation_undock = true;
  start.pre_navigation_undock_detail = "queued controlled undock";
  start.started_at = "2026-08-31T00:00:00Z";

  const auto job = navigation::make_navigation_goal_job(start);
  EXPECT_EQ(job.id, 9U);
  EXPECT_EQ(job.state, "running");
  EXPECT_EQ(job.phase, "accepted");
  EXPECT_EQ(job.detail,
    "navigation goal accepted; controlled undock queued before Nav2 goal send");
  EXPECT_TRUE(job.yaw_align_required);
  EXPECT_TRUE(job.pre_navigation_undock);
  EXPECT_EQ(job.pre_navigation_relocalization_detail,
    "normal path relocalization disabled; goal-start readiness will be checked in navigation background job");
  EXPECT_EQ(job.started_at, "2026-08-31T00:00:00Z");
}

TEST(NavigationGoalJob, FinishTransitionClassifiesTerminalStateAndFlags)
{
  navigation::NavigationGoalJob job;
  job.id = 1U;
  job.state = "running";

  navigation::NavigationGoalJobFinishSpec finish;
  finish.succeeded = true;
  finish.phase = "final_pose_verified";
  finish.detail = "done";
  finish.completed_at = "2026-08-31T00:01:00Z";
  finish.final_distance_m = 0.02;
  finish.final_yaw_error_rad = 0.01;
  finish.nav2_result_code = 1;
  finish.nav2_succeeded = true;
  finish.position_reached = true;
  navigation::apply_navigation_goal_job_finish(job, finish);
  EXPECT_EQ(job.state, "succeeded");
  EXPECT_TRUE(job.task_complete);
  EXPECT_FALSE(job.final_verify_failure_is_terminal);

  finish.succeeded = false;
  finish.phase = "degraded_final_pose_verify";
  navigation::apply_navigation_goal_job_finish(job, finish);
  EXPECT_EQ(job.state, "degraded");
  EXPECT_FALSE(job.task_complete);

  finish.phase = "failed_final_pose_verify";
  finish.final_yaw_align_requested = true;
  finish.final_yaw_align_succeeded = false;
  navigation::apply_navigation_goal_job_finish(job, finish);
  EXPECT_EQ(job.state, "failed");
  EXPECT_TRUE(job.final_verify_failure_is_terminal);
  EXPECT_TRUE(job.yaw_align_failed);

  finish.phase = "canceled";
  navigation::apply_navigation_goal_job_finish(job, finish);
  EXPECT_EQ(job.state, "canceled");
  EXPECT_FALSE(job.final_verify_failure_is_terminal);
  EXPECT_FALSE(job.yaw_align_failed);
}

TEST(NavigationGoalJob, AppliesFinalPoseBridgeAndYawDiagnostics)
{
  navigation::NavigationGoalJob job;
  navigation::apply_navigation_goal_final_pose(job, 0.04, 0.03, true, "pose ok");
  EXPECT_DOUBLE_EQ(job.final_distance_m, 0.04);
  EXPECT_DOUBLE_EQ(job.final_verify_xy_error_m, 0.04);
  EXPECT_TRUE(job.position_reached);
  EXPECT_EQ(job.final_pose_verify_reason, "pose ok");

  navigation::apply_navigation_goal_bridge_wait(job, 125.0, false, "bridge ready");
  EXPECT_DOUBLE_EQ(job.post_nav2_final_verify_bridge_wait_elapsed_ms, 125.0);
  EXPECT_FALSE(job.post_nav2_final_verify_bridge_wait_timeout);
  EXPECT_EQ(job.detail, "bridge ready");

  navigation::NavigationGoalFinalYawUpdate yaw;
  yaw.attempted = true;
  yaw.succeeded = false;
  yaw.blocked = true;
  yaw.blocked_reason = "docking_phase_active";
  yaw.duration_sec = 0.5;
  yaw.initial_yaw_error_rad = 0.2;
  yaw.final_yaw_error_rad = 0.1;
  yaw.observed_xy_drift_m = 0.01;
  navigation::apply_navigation_goal_final_yaw(job, yaw);
  EXPECT_TRUE(job.final_yaw_align_attempted);
  EXPECT_TRUE(job.final_yaw_align_blocked);
  EXPECT_TRUE(job.yaw_align_failed);
  EXPECT_EQ(job.final_yaw_align_blocked_reason, "docking_phase_active");
}
