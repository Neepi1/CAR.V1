#include <gtest/gtest.h>

#include <set>
#include <string>

#include <yaml-cpp/yaml.h>

#include "robot_api_server/features/navigation/navigation_state.hpp"

namespace navigation = robot_api_server::features::navigation;

namespace
{

navigation::NavigationStateSnapshot ready_snapshot()
{
  navigation::NavigationStateSnapshot snapshot;
  snapshot.runtime.mode = "NAVIGATION";
  snapshot.runtime.navigation_state = "running";
  snapshot.runtime.navigation_active = true;
  snapshot.runtime.healthy = true;
  snapshot.runtime.message = "navigation runtime ready";

  robot_api_server::RuntimeMapContext context;
  context.state = "ready";
  context.confirmed = true;
  context.startup_stage = "complete";
  context.message = "selected floor ready";
  context.map_id = "map-1";
  context.building_id = "B10";
  context.floor_id = "F1";
  context.updated_at_sec = 123.5;
  snapshot.runtime_map_context = context;

  snapshot.dock.occupancy_state = "UNDOCKED";
  snapshot.dock.occupancy_evidence = {"live_no_contact", "runtime_idle"};
  snapshot.dock.occupancy_reason = "live_undocked_no_contact";
  snapshot.dock.charging_session_age_sec = -1.0;
  snapshot.dock.pre_navigation_check_json =
    "{\"final_auto_undock_required\":false}";

  snapshot.amcl.available = true;
  snapshot.amcl.stale = true;
  snapshot.amcl.mode = "gated";
  snapshot.amcl.state = "STALE_FILE";
  snapshot.amcl.ready = false;
  snapshot.amcl.degraded = true;
  snapshot.amcl.degraded_reason = "stale legacy file";
  snapshot.amcl.age_ms = 9000.0;
  snapshot.amcl.process_alive = true;
  snapshot.amcl.scan_admission_alive = true;
  snapshot.amcl.pose_publisher_count = 1;
  snapshot.amcl.scan_admission_status_publisher_count = 1;

  snapshot.bridge.available = true;
  snapshot.bridge.has_map_to_odom = true;
  snapshot.bridge.map_to_odom_publisher_owner = "robot_localization_bridge";
  snapshot.bridge.safe_for_goal_start = true;
  snapshot.bridge.amcl_input_enabled = true;
  snapshot.bridge.amcl_ready = true;
  snapshot.bridge.amcl_status_source = "live_bridge";
  snapshot.bridge.amcl_process_ready = true;
  snapshot.bridge.amcl_seeded = true;
  snapshot.bridge.amcl_seed_response_ok = true;
  snapshot.bridge.amcl_static_standby = true;
  snapshot.bridge.amcl_tracking_ready = true;
  snapshot.bridge.amcl_correction_ready = false;
  snapshot.bridge.amcl_correction_pending = false;
  snapshot.bridge.amcl_not_moving_no_update_ok = true;

  snapshot.safety.status = "OK";
  snapshot.safety.motion_allowed = true;
  snapshot.safety.motion_allowed_valid = true;
  snapshot.post_relocalization_settle_json = "{\"complete\":true}";
  snapshot.post_undock_settle_json = "{\"complete\":true}";
  snapshot.navigation_goal_json =
    "{\"state\":\"running\",\"task_complete\":false}";
  snapshot.navigation_cancel_json = "{\"state\":\"idle\"}";
  return snapshot;
}

}  // namespace

TEST(NavigationState, PreservesCommercialStateContractAndBridgePrecedence)
{
  navigation::NavigationStateConfig config;
  const auto response = navigation::navigation_state_response(config, ready_snapshot());

  ASSERT_EQ(response.status, 200);
  ASSERT_EQ(response.content_type, "application/json");
  const auto document = YAML::Load(response.body);
  std::set<std::string> actual_keys;
  for (const auto & entry : document) {
    actual_keys.insert(entry.first.as<std::string>());
  }
  const std::set<std::string> expected_keys = {
    "ok", "mode", "active_runtime_mode", "state", "navigation_active",
    "healthy", "message", "runtime_map_context", "localization_degraded",
    "localization_degraded_reason", "using_triggered_baseline_only",
    "safe_for_goal_start", "goal_start_detail", "correction_active",
    "navigation_normal_path_relocalization_enabled",
    "docking_normal_path_relocalization_enabled",
    "force_accept_allowed_in_normal_path",
    "ordinary_navigation_triggered_relocalization",
    "docking_predock_triggered_relocalization", "native_nav2_goal_completion",
    "api_final_yaw_align_enabled", "nav2_rotation_shim_enabled",
    "localization_recovery_available", "removed_redundant_gates",
    "localization_recovery_required", "amcl_mode", "amcl_state",
    "amcl_start_result", "amcl_status_file_stale", "amcl_status_age_ms",
    "amcl_status_source", "amcl_ready", "amcl_degraded",
    "amcl_degraded_reason", "amcl_process_alive", "amcl_process_ready",
    "amcl_seeded", "amcl_seed_response_ok", "amcl_nomotion_pose_received",
    "amcl_static_standby", "amcl_tracking_ready", "amcl_correction_ready",
    "amcl_correction_pending", "amcl_not_moving_no_update_ok",
    "amcl_scan_admission_alive", "amcl_pose_publisher_count",
    "amcl_scan_admission_status_publisher_count", "dock_occupancy_state",
    "dock_occupancy_evidence", "dock_occupancy_reason",
    "charging_session_latched", "charging_session_age_sec",
    "charging_session_last_confirmed_at", "full_charge_idle_on_dock",
    "pre_navigation_dock_check", "blocked_by_docked_contact",
    "normal_motion_blocked_reason", "safety", "post_relocalization_settle",
    "post_undock_settle", "navigation_goal", "navigation_cancel"};
  EXPECT_EQ(actual_keys, expected_keys);
  EXPECT_TRUE(document["ok"].as<bool>());
  EXPECT_EQ(document["mode"].as<std::string>(), "NAVIGATION");
  EXPECT_EQ(document["active_runtime_mode"].as<std::string>(), "normal");
  EXPECT_TRUE(document["navigation_active"].as<bool>());
  EXPECT_EQ(document["runtime_map_context"]["map_id"].as<std::string>(), "map-1");
  EXPECT_FALSE(document["localization_degraded"].as<bool>());
  EXPECT_FALSE(document["using_triggered_baseline_only"].as<bool>());
  EXPECT_TRUE(document["safe_for_goal_start"].as<bool>());
  EXPECT_FALSE(document["localization_recovery_required"].as<bool>());
  EXPECT_EQ(document["amcl_status_source"].as<std::string>(), "live_bridge");
  EXPECT_TRUE(document["amcl_ready"].as<bool>());
  EXPECT_EQ(document["dock_occupancy_state"].as<std::string>(), "UNDOCKED");
  EXPECT_EQ(document["dock_occupancy_evidence"].size(), 2U);
  EXPECT_TRUE(document["charging_session_age_sec"].IsNull());
  EXPECT_TRUE(document["safety"]["motion_allowed"].as<bool>());
  EXPECT_TRUE(document["post_relocalization_settle"]["complete"].as<bool>());
  EXPECT_TRUE(document["post_undock_settle"]["complete"].as<bool>());
  EXPECT_EQ(document["navigation_goal"]["state"].as<std::string>(), "running");
  EXPECT_EQ(document["navigation_cancel"]["state"].as<std::string>(), "idle");
}

TEST(NavigationState, SeparatesTransitionFromRecoveryRequired)
{
  navigation::NavigationStateConfig config;
  auto snapshot = ready_snapshot();
  snapshot.bridge.safe_for_goal_start = false;
  snapshot.bridge.remaining_translation_error_m = 0.10;
  auto response = navigation::navigation_state_response(config, snapshot);
  auto document = YAML::Load(response.body);
  EXPECT_FALSE(document["safe_for_goal_start"].as<bool>());
  EXPECT_NE(
    document["goal_start_detail"].as<std::string>().find(
      "LOCALIZATION_TRANSITION_ACTIVE"),
    std::string::npos);
  EXPECT_FALSE(document["localization_recovery_required"].as<bool>());

  snapshot.bridge.safe_for_goal_start = true;
  snapshot.bridge.map_to_odom_publisher_owner = "unexpected_owner";
  response = navigation::navigation_state_response(config, snapshot);
  document = YAML::Load(response.body);
  EXPECT_TRUE(document["localization_recovery_required"].as<bool>());
  EXPECT_NE(
    document["goal_start_detail"].as<std::string>().find("LOCALIZATION_DEGRADED"),
    std::string::npos);
}
