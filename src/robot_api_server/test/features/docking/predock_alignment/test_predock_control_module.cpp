#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/docking/predock_alignment/predock_control_module.hpp"

namespace
{

using robot_api_server::features::docking::DockingJobStore;
using robot_api_server::features::docking::DockingJobStorePorts;
using robot_api_server::features::docking::predock_alignment::PredockAlignmentConfig;
using robot_api_server::features::docking::predock_alignment::PredockAlignmentPolicy;
using robot_api_server::features::docking::predock_alignment::PredockControlConfig;
using robot_api_server::features::docking::predock_alignment::PredockControlModule;
using robot_api_server::features::docking::predock_alignment::PredockControlPorts;
using robot_api_server::features::docking::predock_alignment::PredockMotionModeSnapshot;
using robot_api_server::features::docking::predock_alignment::PredockObservationSnapshot;
using robot_api_server::features::localization::BridgeStatusSnapshot;

robot_api_server::RobotPoseSnapshot aligned_pose()
{
  robot_api_server::RobotPoseSnapshot pose;
  pose.available = true;
  pose.frame_id = "map";
  pose.age_sec = 0.0;
  return pose;
}

BridgeStatusSnapshot ready_bridge()
{
  BridgeStatusSnapshot bridge;
  bridge.available = true;
  bridge.has_map_to_odom = true;
  bridge.map_to_odom_publisher_owner = "robot_localization_bridge";
  bridge.safe_for_goal_start = true;
  return bridge;
}

DockingJobStorePorts store_ports()
{
  DockingJobStorePorts ports;
  ports.update_dock_contact_latch = [](bool, const auto &, const auto &, const auto &) {};
  ports.finish_runtime = [](const auto &, const auto &) {};
  ports.timestamp_now = []() {return "2026-09-02T00:00:00Z";};
  ports.set_global_correction_paused = [](auto, auto, const auto &, auto &) {return true;};
  return ports;
}

struct FixtureState
{
  DockingJobStore * store{nullptr};
  robot_api_server::RobotPoseSnapshot pose{aligned_pose()};
  BridgeStatusSnapshot bridge{ready_bridge()};
  PredockObservationSnapshot observation{0.01, true, "fresh"};
  std::vector<geometry_msgs::msg::Twist> commands;
  std::vector<bool> pause_events;
  std::vector<std::string> phases;
  bool owner_available{true};
  int owner_acquire_count{0};
  bool fine_started{false};
};

PredockControlPorts control_ports(FixtureState & state)
{
  PredockControlPorts ports;
  ports.wait_for_current_pose = [&state](bool, std::string &) {return state.pose;};
  ports.current_pose = [&state]() {return state.pose;};
  ports.safety_motion_hard_blocked = [](std::string &) {return false;};
  ports.try_acquire_motion_owner = [&state](bool) {
      ++state.owner_acquire_count;
      return state.owner_available;
    };
  ports.release_motion_owner = []() {};
  ports.request_navigation_goal_cancel = [](const auto &) {};
  ports.publish_final_yaw_zero_burst = []() {};
  ports.publish_command = [&state](const auto & command) {state.commands.push_back(command);};
  ports.publish_forced_mode = [](const auto &) {};
  ports.clear_teleop_command = []() {};
  ports.publish_teleop_zero = []() {};
  ports.motion_mode_snapshot = []() {return PredockMotionModeSnapshot{};};
  ports.reset_actual_stop_stability = []() {};
  ports.wait_for_actual_stop = [](const auto &, auto & detail) {
      detail = "stopped";
      return true;
    };
  ports.yaw_stop_threshold_rad = [](double, const double tolerance) {return tolerance;};
  ports.bridge_status_snapshot = [&state]() {return state.bridge;};
  ports.bridge_safe_for_goal_start = [](const auto & bridge, const auto &, auto & detail) {
      detail = bridge.safe_for_goal_start ? "ready" : "LOCALIZATION_NOT_READY";
      return bridge.safe_for_goal_start;
    };
  ports.set_global_correction_paused = [&state](
      auto, const bool paused, const auto &, auto & detail) {
      state.pause_events.push_back(paused);
      detail = paused ? "paused" : "resumed";
      std::lock_guard<std::mutex> lock(state.store->mutex());
      state.store->job_unsafe().global_correction_paused = paused;
      return true;
    };
  ports.set_docking_runtime_state = [&state](
      bool, const auto & phase, const auto &, bool) {state.phases.push_back(phase);};
  ports.observation_snapshot = [&state]() {return state.observation;};
  ports.ensure_docking_manager_running = [](auto & detail) {
      detail = "resident";
      return true;
    };
  ports.floor_runtime_operation_blocked = [](const auto &, auto &) {return false;};
  ports.start_fine_docking = [&state](auto & detail) {
      state.fine_started = true;
      detail = "fine docking accepted";
      return true;
    };
  return ports;
}

robot_api_server::DockingJob running_job(const std::uint64_t id)
{
  robot_api_server::DockingJob job;
  job.id = id;
  job.state = "running";
  job.goal_completion_policy = "dock_staging";
  job.max_retries = 2;
  return job;
}

TEST(PredockControlModule, DeepInterfaceCompletesAlignedFineHandoffWithoutMotion)
{
  DockingJobStore store(store_ports());
  FixtureState state;
  state.store = &store;
  const auto job = running_job(41U);
  {
    std::lock_guard<std::mutex> lock(store.mutex());
    store.job_unsafe() = job;
    store.job_unsafe().post_predock_settle_complete = true;
  }
  PredockAlignmentPolicy policy(PredockAlignmentConfig{});
  PredockControlConfig config;
  config.fine_wait_for_bridge_smoothing_enabled = false;
  PredockControlModule module(
    rclcpp::get_logger("predock_control_test"),
    config,
    policy,
    store,
    control_ports(state));

  EXPECT_TRUE(module.start_fine_docking_handoff(job.id, job));
  const auto snapshot = store.snapshot();
  EXPECT_TRUE(state.fine_started);
  EXPECT_EQ(snapshot.phase, "FINE_ALIGN");
  EXPECT_TRUE(snapshot.dock_staging_handoff_ready);
  EXPECT_TRUE(snapshot.predock_pose_verified);
  EXPECT_TRUE(snapshot.fine_entry_ok);
  EXPECT_EQ(state.pause_events, std::vector<bool>({true}));
  for (const auto & command : state.commands) {
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
  }
}

TEST(PredockControlModule, ObservationFailureReleasesPauseAndTerminalizesOnce)
{
  DockingJobStore store(store_ports());
  FixtureState state;
  state.store = &store;
  state.observation = {-1.0, false, "missing"};
  const auto job = running_job(42U);
  {
    std::lock_guard<std::mutex> lock(store.mutex());
    store.job_unsafe() = job;
    store.job_unsafe().post_predock_settle_complete = true;
  }
  PredockAlignmentPolicy policy(PredockAlignmentConfig{});
  PredockControlConfig config;
  config.fine_wait_for_bridge_smoothing_enabled = false;
  PredockControlModule module(
    rclcpp::get_logger("predock_control_test"),
    config,
    policy,
    store,
    control_ports(state));

  EXPECT_FALSE(module.start_fine_docking_handoff(job.id, job));
  const auto snapshot = store.snapshot();
  EXPECT_FALSE(state.fine_started);
  EXPECT_EQ(snapshot.state, "failed");
  EXPECT_EQ(snapshot.failure_code, "DOCK_TARGET_OBSERVATION_TIMEOUT");
  EXPECT_EQ(state.pause_events, std::vector<bool>({true, false}));
}

TEST(PredockControlModule, DelegatesRecoverableYawAndLateralResidualToFineController)
{
  DockingJobStore store(store_ports());
  FixtureState state;
  state.store = &store;
  state.pose.y = 0.08;
  state.pose.yaw = 0.10;
  const auto job = running_job(44U);
  {
    std::lock_guard<std::mutex> lock(store.mutex());
    store.job_unsafe() = job;
    store.job_unsafe().post_predock_settle_complete = true;
  }
  PredockAlignmentPolicy policy(PredockAlignmentConfig{});
  PredockControlConfig config;
  config.delegate_staging_motion_to_manager = true;
  config.fine_wait_for_bridge_smoothing_enabled = false;
  config.yaw_align_timeout_sec = 0.01;
  config.yaw_align_period_ms = 1;
  PredockControlModule module(
    rclcpp::get_logger("predock_control_test"),
    config,
    policy,
    store,
    control_ports(state));

  EXPECT_TRUE(module.start_fine_docking_handoff(job.id, job));
  const auto snapshot = store.snapshot();
  EXPECT_TRUE(state.fine_started);
  EXPECT_EQ(state.owner_acquire_count, 0);
  EXPECT_FALSE(snapshot.predock_yaw_align_attempted);
  EXPECT_FALSE(snapshot.predock_lateral_align_attempted);
  EXPECT_TRUE(snapshot.dock_staging_handoff_ready);
  EXPECT_TRUE(snapshot.fine_entry_ok);
  for (const auto & command : state.commands) {
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
  }
}

TEST(PredockControlModule, HandoffProbeOnlyRecordsAfterRecoveryWindowAdmission)
{
  DockingJobStore store(store_ports());
  FixtureState state;
  state.store = &store;
  const auto job = running_job(43U);
  {
    std::lock_guard<std::mutex> lock(store.mutex());
    store.job_unsafe() = job;
  }
  PredockAlignmentPolicy policy(PredockAlignmentConfig{});
  PredockControlModule module(
    rclcpp::get_logger("predock_control_test"),
    PredockControlConfig{},
    policy,
    store,
    control_ports(state));

  state.pose.x = 2.0;
  const auto outside = module.probe_handoff(job.id, job);
  EXPECT_FALSE(outside.recovery_allowed);
  EXPECT_FALSE(store.snapshot().predock_pose_verified);

  state.pose.x = 0.0;
  const auto inside = module.probe_handoff(job.id, job);
  EXPECT_TRUE(inside.recovery_allowed);
  EXPECT_TRUE(store.snapshot().predock_pose_verified);
}

}  // namespace
