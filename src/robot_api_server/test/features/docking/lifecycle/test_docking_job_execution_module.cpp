#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>

#include "robot_api_server/features/docking/lifecycle/docking_job_execution_module.hpp"

namespace
{

using robot_api_server::BmsChargingContactSnapshot;
using robot_api_server::DockingJob;
using robot_api_server::StoredPose;
using robot_api_server::features::docking::DockingJobExecutionModule;
using robot_api_server::features::docking::DockingJobExecutionModuleConfig;
using robot_api_server::features::docking::DockingJobExecutionModulePorts;
using robot_api_server::features::docking::DockingJobStore;
using robot_api_server::features::docking::DockingJobStorePorts;
using robot_api_server::features::docking::DockingRelocalizationSettleResult;

struct Observations
{
  std::mutex action_mutex;
  bool action_client_requested{false};
  bool tracked_goal{false};
  bool resumed{false};
  bool navigation_state_set{false};
  bool docking_state_set{false};
  bool bridge_checked{false};
  bool localization_triggered{false};
  bool settle_waited{false};
  bool terminal_speed_cleared{false};
  bool terminal_speed_published{false};
  bool reverse_permit_cleared{false};
  bool reverse_permit_updated{false};
  bool terminal_stop_reset{false};
  bool terminal_stop_waited{false};
};

DockingJobStore make_store()
{
  DockingJobStorePorts ports;
  ports.update_dock_contact_latch = [](bool, const auto &, const auto &, const auto &) {};
  ports.finish_runtime = [](const auto &, const auto &) {};
  ports.timestamp_now = []() {return "2026-09-03T00:00:00Z";};
  ports.set_global_correction_paused = [](auto, auto, const auto &, auto &) {return true;};
  return DockingJobStore(std::move(ports));
}

DockingJobExecutionModulePorts make_ports(Observations & observations)
{
  DockingJobExecutionModulePorts ports;
  ports.floor_runtime_operation_blocked =
    [](const auto &, auto & detail, auto *) {
      detail.clear();
      return false;
    };
  ports.resume_navigation_runtime =
    [&observations](const DockingJob &, auto & detail) {
      observations.resumed = true;
      detail = "resumed";
      return true;
    };
  ports.now = []() {return rclcpp::Time(123, 0, RCL_SYSTEM_TIME);};
  ports.navigation_action_mutex = [&observations]() -> std::mutex & {
      return observations.action_mutex;
    };
  ports.navigation_action_client = [&observations]() {
      observations.action_client_requested = true;
      return DockingJobExecutionModulePorts::ActionClient::SharedPtr{};
    };
  ports.track_navigation_goal =
    [&observations](auto, auto, auto, auto) {observations.tracked_goal = true;};
  ports.cancel_active_navigation_goal = [](auto & detail) {
      detail = "cancelled";
      return true;
    };
  ports.request_navigation_goal_cancel = [](const auto &) {return true;};
  ports.ordinary_final_yaw_align_active = []() {return false;};
  ports.record_docking_cmd_owner_conflict = []() {};
  ports.publish_final_yaw_align_zero_burst = []() {};
  ports.clear_teleop_command = []() {};
  ports.publish_teleop_zero_burst = []() {};
  ports.set_navigation_runtime_state =
    [&observations](auto, const auto &, const auto &, auto) {
      observations.navigation_state_set = true;
    };
  ports.set_docking_runtime_state =
    [&observations](auto, const auto &, const auto &, auto) {
      observations.docking_state_set = true;
    };
  ports.bridge_safe_for_goal_start =
    [&observations](const auto &, auto & detail) {
      observations.bridge_checked = true;
      detail = "safe";
      return true;
    };
  ports.trigger_localization_and_wait_for_result =
    [&observations](const auto &, auto & detail, auto, auto * sequence) {
      observations.localization_triggered = true;
      detail = "localized";
      if (sequence) {
        *sequence = 17U;
      }
      return true;
    };
  ports.wait_for_relocalization_settle =
    [&observations](auto sequence, const auto &, const auto &, const auto &) {
      observations.settle_waited = true;
      return DockingRelocalizationSettleResult{
      sequence == 17U, "", "settled"};
    };
  ports.clear_navigation_terminal_speed_limit =
    [&observations]() {observations.terminal_speed_cleared = true;};
  ports.publish_navigation_terminal_speed_limit_for_goal =
    [&observations](const StoredPose &) {observations.terminal_speed_published = true;};
  ports.clear_navigation_terminal_reverse_permit =
    [&observations](bool & active, const auto &) {
      observations.reverse_permit_cleared = true;
      active = false;
    };
  ports.update_navigation_terminal_reverse_permit_for_goal =
    [&observations](const StoredPose &, bool & active, auto &, const auto &) {
      observations.reverse_permit_updated = true;
      active = true;
    };
  ports.bms_charging_contact_snapshot = []() {
      BmsChargingContactSnapshot snapshot;
      snapshot.contact = true;
      return snapshot;
    };
  ports.reset_terminal_actual_stop_stability =
    [&observations]() {observations.terminal_stop_reset = true;};
  ports.wait_for_terminal_actual_stop =
    [&observations](const auto &, auto & detail, const bool require_mode) {
      observations.terminal_stop_waited = true;
      detail = require_mode ? "dual_ackermann_stopped" : "stopped";
      return true;
    };
  return ports;
}

TEST(DockingJobExecutionModule, RejectsIncompleteRuntimePorts)
{
  auto store = make_store();
  std::atomic<std::uint64_t> unknown_count{0U};
  EXPECT_THROW(
    DockingJobExecutionModule(
      DockingJobExecutionModuleConfig{}, store, unknown_count, {}),
    std::invalid_argument);
}

TEST(DockingJobExecutionModule, OwnsCanonicalJobStoreAndNavigationEvidence)
{
  auto store = make_store();
  {
    std::lock_guard<std::mutex> lock(store.mutex());
    store.job_unsafe().id = 41U;
    store.job_unsafe().state = "running";
    store.job_unsafe().cancel_requested = true;
  }
  std::atomic<std::uint64_t> unknown_count{0U};
  Observations observations;
  DockingJobExecutionModule module(
    DockingJobExecutionModuleConfig{}, store, unknown_count,
    make_ports(observations));

  EXPECT_EQ(&module.docking_job_mutex(), &store.mutex());
  EXPECT_EQ(&module.docking_job_unsafe(), &store.job_unsafe());
  EXPECT_TRUE(module.docking_cancel_requested(41U));
  module.set_docking_job_phase(41U, "NAV_TO_STAGING_NATIVE_NAV2");
  module.mark_docking_nav_goal_sent(41U, nullptr, "B10", "F1");

  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.phase, "NAV_TO_STAGING_NATIVE_NAV2");
  EXPECT_TRUE(snapshot.nav_goal_sent);
  EXPECT_TRUE(observations.tracked_goal);
}

TEST(DockingJobExecutionModule, PreservesTimeoutAndCrossDomainForwarding)
{
  auto store = make_store();
  std::atomic<std::uint64_t> unknown_count{0U};
  Observations observations;
  DockingJobExecutionModuleConfig config;
  config.navigation_start_wait_sec = 12.5;
  DockingJobExecutionModule module(
    config, store, unknown_count, make_ports(observations));

  EXPECT_EQ(
    module.docking_navigation_start_timeout(),
    std::chrono::milliseconds(12500));
  EXPECT_EQ(module.docking_goal_stamp().nanoseconds(), 123000000000LL);

  std::string detail;
  EXPECT_TRUE(module.resume_navigation_runtime_for_docking(DockingJob{}, detail));
  EXPECT_EQ(detail, "resumed");
  module.set_navigation_runtime_state(true, "running", "nav", true);
  module.set_docking_runtime_state(true, "running", "dock", true);
  EXPECT_TRUE(module.bridge_safe_for_goal_start("docking", detail));
  std::uint64_t sequence = 0U;
  EXPECT_TRUE(
    module.trigger_localization_and_wait_for_result(
      "predock", detail, 8.0, &sequence));
  EXPECT_EQ(sequence, 17U);
  EXPECT_TRUE(
    module.wait_for_docking_relocalization_settle_barrier(
      sequence, "predock", "staging").ok);

  StoredPose pose;
  module.clear_navigation_terminal_speed_limit();
  module.publish_navigation_terminal_speed_limit_for_goal(pose);
  bool permit_active = true;
  module.clear_navigation_terminal_reverse_permit(permit_active, "clear");
  EXPECT_FALSE(permit_active);
  auto refresh_at = std::chrono::steady_clock::time_point{};
  module.update_navigation_terminal_reverse_permit_for_goal(
    pose, permit_active, refresh_at, "update");
  EXPECT_TRUE(permit_active);
  module.reset_terminal_actual_stop_stability();
  EXPECT_TRUE(module.wait_for_terminal_actual_stop("terminal", detail, true));
  EXPECT_TRUE(module.bms_charging_contact_snapshot().contact);

  EXPECT_TRUE(observations.resumed);
  EXPECT_TRUE(observations.navigation_state_set);
  EXPECT_TRUE(observations.docking_state_set);
  EXPECT_TRUE(observations.bridge_checked);
  EXPECT_TRUE(observations.localization_triggered);
  EXPECT_TRUE(observations.settle_waited);
  EXPECT_TRUE(observations.terminal_speed_cleared);
  EXPECT_TRUE(observations.terminal_speed_published);
  EXPECT_TRUE(observations.reverse_permit_cleared);
  EXPECT_TRUE(observations.reverse_permit_updated);
  EXPECT_TRUE(observations.terminal_stop_reset);
  EXPECT_TRUE(observations.terminal_stop_waited);
}

TEST(DockingJobExecutionModule, FloorBlockPreventsPredockActionSideEffect)
{
  auto store = make_store();
  std::atomic<std::uint64_t> unknown_count{0U};
  Observations observations;
  auto ports = make_ports(observations);
  ports.floor_runtime_operation_blocked =
    [](const auto & operation, auto & detail, auto *) {
      detail = "blocked:" + operation;
      return true;
    };
  DockingJobExecutionModule module(
    DockingJobExecutionModuleConfig{}, store, unknown_count, std::move(ports));

  DockingJobExecutionModule::NavigateToPose::Goal goal;
  DockingJobExecutionModule::GoalHandle::SharedPtr handle;
  std::string detail;
  EXPECT_FALSE(module.send_predock_navigation_goal(goal, handle, detail));
  EXPECT_EQ(detail, "blocked:docking_predock_goal_submit");
  EXPECT_FALSE(observations.action_client_requested);
  EXPECT_EQ(unknown_count.load(), 0U);
}

}  // namespace
