#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "robot_api_server/features/docking/lifecycle/pre_navigation_undock_module.hpp"

namespace robot_api_server::features::docking
{
namespace
{

class PreNavigationUndockHarness
{
public:
  PreNavigationUndockHarness()
  {
    DockingJobStorePorts store_ports;
    store_ports.update_dock_contact_latch =
      [](bool, const std::string &, const std::string &, const std::string &) {};
    store_ports.finish_runtime = [this](
      const std::string & state, const std::string & detail)
      {
        runtime.finish_docking(state, detail);
      };
    store_ports.timestamp_now = []() {return std::string("finished-at");};
    store_ports.set_global_correction_paused =
      [](std::uint64_t, bool, const std::string &, std::string &) {return true;};
    store = std::make_unique<DockingJobStore>(std::move(store_ports));

    runtime.set_docking(true, "docked", "on dock");
    runtime.set_docking_identity("dock-a");
    runtime.set_docking_status("docked");
  }

  PreNavigationUndockPorts ports()
  {
    PreNavigationUndockPorts result;
    result.clear_teleop_command = [this]() {++clear_count;};
    result.publish_zero_motion = [this]() {++zero_count;};
    result.prepare_controlled_undock = [this](std::string & detail) {
        ++takeover_count;
        detail = takeover_detail;
        if (!takeover_ok) {
          return false;
        }
        set_job_state("canceled", "finished");
        return true;
      };
    result.release_stale_fine_pause = [this](
      const std::string & context, std::string & detail)
      {
        stale_pause_context = context;
        detail = stale_pause_detail;
        return stale_pause_ok;
      };
    result.join_docking_worker = [this]() {++join_count;};
    result.runtime_snapshot = [this]() {return runtime.snapshot();};
    result.reconcile_stale_interlock = [this](
      const std::string & dock_id,
      const std::string & evidence,
      std::string & detail) {
        ++reconcile_count;
        reconcile_dock_id = dock_id;
        reconcile_evidence = evidence;
        detail = reconcile_detail;
        return reconcile_ok;
      };
    result.ensure_manager_running = [this](std::string & detail) {
        detail = ensure_detail;
        return ensure_ok;
      };
    result.call_undock_with_charging_retry = [this](
      std::string & detail,
      const bool allow_charging_retry,
      PreNavigationUndockServiceObservation * observation)
      {
        ++service_count;
        charging_retry_seen = allow_charging_retry;
        detail = service_detail;
        if (observation != nullptr) {
          observation->service_called = service_called;
          observation->service_success = service_success;
          observation->message = service_message;
        }
        return service_ok;
      };
    result.observe_undock_status = [this](
      DockingJob & job, const std::string & status)
      {
        ++observe_count;
        job.docking_status_after_request = status;
        job.undock_started_observed = status.find("undocking") != std::string::npos;
      };
    result.set_docking_runtime_state = [this](
      const bool active,
      const std::string & state,
      const std::string & detail)
      {
        runtime.set_docking(active, state, detail);
      };
    result.set_docking_identity = [this](const std::string & dock_id) {
        runtime.set_docking_identity(dock_id);
      };
    result.timestamp_now = []() {return std::string("started-at");};
    result.monotonic_now = [this]() {return now;};
    result.sleep_for = [this](const std::chrono::milliseconds duration) {
        ++sleep_count;
        now += duration;
        if (on_sleep) {
          on_sleep();
        }
      };
    return result;
  }

  std::unique_ptr<PreNavigationUndockModule> make_module(
    PreNavigationUndockConfig config = {})
  {
    return std::make_unique<PreNavigationUndockModule>(
      start_mutex, *store, config, ports());
  }

  void set_job_state(
    const std::string & state,
    const std::string & phase = "finished")
  {
    std::lock_guard<std::mutex> lock(store->mutex());
    store->job_unsafe().state = state;
    store->job_unsafe().phase = phase;
  }

  void set_relocalized_undocked(
    const bool succeeded,
    const std::string & readiness_detail = "")
  {
    std::lock_guard<std::mutex> lock(store->mutex());
    auto & job = store->job_unsafe();
    job.state = "undocked";
    job.phase = "finished";
    job.detail = succeeded ? "undocked and localized" : "undocked";
    job.post_undock_relocalization_requested = true;
    job.post_undock_relocalization_succeeded = succeeded;
    job.post_undock_navigation_readiness_detail = readiness_detail;
  }

  std::mutex start_mutex;
  application::runtime_mode::RuntimeModeCoordinator runtime;
  std::unique_ptr<DockingJobStore> store;
  std::chrono::steady_clock::time_point now{};
  std::function<void()> on_sleep;
  bool stale_pause_ok{true};
  std::string stale_pause_detail{"no stale pause"};
  std::string stale_pause_context;
  bool ensure_ok{true};
  std::string ensure_detail{"manager ready"};
  bool service_ok{true};
  bool service_called{true};
  bool service_success{true};
  std::string service_detail{"undock accepted"};
  std::string service_message{"accepted"};
  bool charging_retry_seen{false};
  int clear_count{0};
  int zero_count{0};
  bool takeover_ok{true};
  std::string takeover_detail{"superseded active docking job"};
  int takeover_count{0};
  int join_count{0};
  int service_count{0};
  bool reconcile_ok{true};
  std::string reconcile_detail{"stale interlock cleared"};
  std::string reconcile_dock_id;
  std::string reconcile_evidence;
  int reconcile_count{0};
  int observe_count{0};
  int sleep_count{0};
};

PreNavigationUndockRequest required_request()
{
  PreNavigationUndockRequest request;
  request.auto_undock_required = true;
  request.charging_contact_at_gate = true;
  request.runtime_docking_state = "docked";
  request.auto_undock_reason = "charging_contact";
  return request;
}

TEST(PreNavigationUndockModuleTest, MissingPortsAreRejected)
{
  std::mutex mutex;
  DockingJobStorePorts store_ports;
  DockingJobStore store(std::move(store_ports));
  EXPECT_THROW(
    PreNavigationUndockModule(mutex, store, {}, {}),
    std::invalid_argument);
}

TEST(PreNavigationUndockModuleTest, NoDockEvidenceAllowsNavigationWithoutSideEffects)
{
  PreNavigationUndockHarness harness;
  auto module = harness.make_module();
  PreNavigationUndockRequest request;
  request.auto_undock_reason = "not_docked";
  std::string detail;
  bool performed = true;

  EXPECT_TRUE(module->run_if_needed(request, detail, performed));
  EXPECT_FALSE(performed);
  EXPECT_EQ(detail, "not_docked");
  EXPECT_EQ(harness.clear_count, 0);
  EXPECT_EQ(harness.service_count, 0);
}

TEST(PreNavigationUndockModuleTest, ProvenRemoteUndockClearsInterlockWithoutMotion)
{
  PreNavigationUndockHarness harness;
  auto module = harness.make_module();
  PreNavigationUndockRequest request;
  request.recovery_action = "CLEAR_STALE_INTERLOCK";
  request.resolved_dock_id = "dock-a";
  request.reconcile_evidence = "dock_zone=CLEAR distance_m=1.8";
  std::string detail;
  bool performed = true;

  EXPECT_TRUE(module->run_if_needed(request, detail, performed));
  EXPECT_FALSE(performed);
  EXPECT_EQ(harness.reconcile_count, 1);
  EXPECT_EQ(harness.reconcile_dock_id, "dock-a");
  EXPECT_EQ(harness.clear_count, 0);
  EXPECT_EQ(harness.service_count, 0);
}

TEST(PreNavigationUndockModuleTest, ActiveNonDockedDockingJobBlocksNavigation)
{
  PreNavigationUndockHarness harness;
  auto module = harness.make_module();
  PreNavigationUndockRequest request;
  request.docking_active_not_docked_block = true;
  request.runtime_docking_state = "fine_docking";
  std::string detail;
  bool performed = true;

  EXPECT_FALSE(module->run_if_needed(request, detail, performed));
  EXPECT_FALSE(performed);
  EXPECT_EQ(detail, "docking is active but not docked: fine_docking");
}

TEST(PreNavigationUndockModuleTest, ConcurrentNonUndockJobIsSafelySuperseded)
{
  PreNavigationUndockHarness harness;
  harness.set_job_state("running", "fine_docking");
  PreNavigationUndockConfig config;
  config.relocalize_after_success = false;
  auto module = harness.make_module(config);
  std::string detail;
  bool performed = false;

  harness.on_sleep = [&harness]() {harness.set_job_state("undocked");};

  EXPECT_TRUE(module->run_if_needed(required_request(), detail, performed));
  EXPECT_TRUE(performed);
  EXPECT_EQ(detail, "undock accepted");
  EXPECT_EQ(harness.clear_count, 1);
  EXPECT_EQ(harness.zero_count, 1);
  EXPECT_EQ(harness.takeover_count, 1);
  EXPECT_EQ(harness.service_count, 1);
}

TEST(PreNavigationUndockModuleTest, PersistedDockIdentityIsAppliedToControlledUndock)
{
  PreNavigationUndockHarness harness;
  PreNavigationUndockConfig config;
  config.relocalize_after_success = false;
  auto module = harness.make_module(config);
  auto request = required_request();
  request.resolved_dock_id = "dock-from-persistent-latch";
  harness.on_sleep = [&harness]() {harness.set_job_state("undocked");};
  std::string detail;
  bool performed = false;

  EXPECT_TRUE(module->run_if_needed(request, detail, performed));
  EXPECT_TRUE(performed);
  EXPECT_EQ(harness.store->snapshot().dock_id, "dock-from-persistent-latch");
  EXPECT_EQ(
    harness.runtime.snapshot().docking_dock_id,
    "dock-from-persistent-latch");
}

TEST(PreNavigationUndockModuleTest, MissingDockIdentityStillUndocksAndRelocalizes)
{
  PreNavigationUndockHarness harness;
  harness.runtime.set_docking_identity("");
  auto module = harness.make_module();
  auto request = required_request();
  request.resolved_dock_id.clear();
  harness.on_sleep = [&harness]() {harness.set_relocalized_undocked(true);};
  std::string detail;
  bool performed = false;

  EXPECT_TRUE(module->run_if_needed(request, detail, performed));
  EXPECT_TRUE(performed);
  EXPECT_EQ(detail, "undocked and localized");
  EXPECT_EQ(harness.service_count, 1);
  const auto job = harness.store->snapshot();
  EXPECT_TRUE(job.dock_id.empty());
  EXPECT_TRUE(job.resume_navigation);
  EXPECT_TRUE(job.docking_service_success);
  EXPECT_TRUE(job.pending_goal_released_after_post_undock_settle);
}

TEST(PreNavigationUndockModuleTest, FailedTakeoverDoesNotSubmitUndock)
{
  PreNavigationUndockHarness harness;
  harness.set_job_state("running", "fine_docking");
  harness.takeover_ok = false;
  harness.takeover_detail = "failed to stop active docking owner";
  auto module = harness.make_module();
  std::string detail;
  bool performed = false;

  EXPECT_FALSE(module->run_if_needed(required_request(), detail, performed));
  EXPECT_TRUE(performed);
  EXPECT_EQ(detail, "failed to stop active docking owner");
  EXPECT_EQ(harness.takeover_count, 1);
  EXPECT_EQ(harness.service_count, 0);
}

TEST(PreNavigationUndockModuleTest, MissingDockIdentityDoesNotHideServiceFailure)
{
  PreNavigationUndockHarness harness;
  harness.runtime.set_docking_identity("");
  harness.service_ok = false;
  harness.service_success = false;
  harness.service_detail = "undock service rejected";
  auto module = harness.make_module();
  std::string detail;
  bool performed = false;

  EXPECT_FALSE(module->run_if_needed(required_request(), detail, performed));
  EXPECT_EQ(detail, "undock service rejected");
  EXPECT_EQ(harness.service_count, 1);
  const auto job = harness.store->snapshot();
  EXPECT_EQ(job.state, "failed");
  EXPECT_FALSE(job.ok);
  EXPECT_FALSE(job.pending_goal_released_after_post_undock_settle);
}

TEST(PreNavigationUndockModuleTest, MissingDockIdentityStillWaitsForRelocalization)
{
  PreNavigationUndockHarness harness;
  harness.runtime.set_docking_identity("");
  auto module = harness.make_module();
  harness.on_sleep = [&harness]() {
      harness.set_relocalized_undocked(false, "localization result unavailable");
    };
  std::string detail;
  bool performed = false;

  EXPECT_FALSE(module->run_if_needed(required_request(), detail, performed));
  EXPECT_EQ(harness.service_count, 1);
  EXPECT_EQ(harness.store->snapshot().state, "undocked");
  EXPECT_NE(detail.find("localization result unavailable"), std::string::npos);
  EXPECT_FALSE(harness.store->snapshot().pending_goal_released_after_post_undock_settle);
}

TEST(PreNavigationUndockModuleTest, NoMotionReconciliationStillRequiresDockIdentity)
{
  PreNavigationUndockHarness harness;
  auto module = harness.make_module();
  PreNavigationUndockRequest request;
  request.recovery_action = "CLEAR_STALE_INTERLOCK";
  std::string detail;
  bool performed = false;

  EXPECT_FALSE(module->run_if_needed(request, detail, performed));
  EXPECT_FALSE(performed);
  EXPECT_EQ(harness.reconcile_count, 0);
  EXPECT_EQ(harness.service_count, 0);
}

TEST(PreNavigationUndockModuleTest, ExistingUndockIsWaitedWithoutSecondServiceCall)
{
  PreNavigationUndockHarness harness;
  harness.set_job_state("running", "undocking");
  PreNavigationUndockConfig config;
  config.relocalize_after_success = false;
  auto module = harness.make_module(config);
  harness.on_sleep = [&harness]() {harness.set_job_state("undocked");};
  auto request = required_request();
  request.runtime_state_undocking = true;
  std::string detail;
  bool performed = false;

  EXPECT_TRUE(module->run_if_needed(request, detail, performed));
  EXPECT_TRUE(performed);
  EXPECT_EQ(harness.service_count, 0);
  EXPECT_EQ(detail, "undocked before navigation");
}

TEST(PreNavigationUndockModuleTest, ServiceFailureFinishesCanonicalJob)
{
  PreNavigationUndockHarness harness;
  harness.service_ok = false;
  harness.service_success = false;
  harness.service_detail = "undock service rejected";
  auto module = harness.make_module();
  std::string detail;
  bool performed = false;

  EXPECT_FALSE(module->run_if_needed(required_request(), detail, performed));
  const auto job = harness.store->snapshot();
  EXPECT_TRUE(performed);
  EXPECT_EQ(detail, "undock service rejected");
  EXPECT_EQ(job.state, "failed");
  EXPECT_FALSE(job.ok);
  EXPECT_FALSE(job.api_accepted);
  EXPECT_TRUE(job.docking_service_called);
  EXPECT_FALSE(job.docking_service_success);
  EXPECT_EQ(harness.stale_pause_context, "pre_navigation_undock_start");
}

TEST(PreNavigationUndockModuleTest, SuccessfulUndockWithoutRelocalizationReleasesNavigation)
{
  PreNavigationUndockHarness harness;
  PreNavigationUndockConfig config;
  config.relocalize_after_success = false;
  auto module = harness.make_module(config);
  harness.on_sleep = [&harness]() {harness.set_job_state("undocked");};
  std::string detail;
  bool performed = false;

  EXPECT_TRUE(module->run_if_needed(required_request(), detail, performed));
  const auto job = harness.store->snapshot();
  EXPECT_TRUE(performed);
  EXPECT_TRUE(harness.charging_retry_seen);
  EXPECT_EQ(harness.service_count, 1);
  EXPECT_TRUE(job.pending_goal_held_for_post_undock_settle);
  EXPECT_FALSE(job.pending_goal_released_after_post_undock_settle);
  EXPECT_EQ(
    job.docking_service_warning,
    "service_success_without_undocking_status_observed_yet");
}

TEST(PreNavigationUndockModuleTest, RelocalizationProofReleasesHeldGoal)
{
  PreNavigationUndockHarness harness;
  auto module = harness.make_module();
  harness.on_sleep = [&harness]() {harness.set_relocalized_undocked(true);};
  std::string detail;
  bool performed = false;

  EXPECT_TRUE(module->run_if_needed(required_request(), detail, performed));
  const auto job = harness.store->snapshot();
  EXPECT_EQ(detail, "undocked and localized");
  EXPECT_TRUE(job.pending_goal_released_after_post_undock_settle);
}

TEST(PreNavigationUndockModuleTest, MissingPostUndockReadinessProofBlocksNav2Goal)
{
  PreNavigationUndockHarness harness;
  auto module = harness.make_module();
  harness.on_sleep = [&harness]() {
      harness.set_relocalized_undocked(false, "AMCL not ready");
    };
  std::string detail;
  bool performed = false;

  EXPECT_FALSE(module->run_if_needed(required_request(), detail, performed));
  EXPECT_NE(detail.find("Nav2 goal not sent: AMCL not ready"), std::string::npos);
  EXPECT_FALSE(harness.store->snapshot().pending_goal_released_after_post_undock_settle);
}

TEST(PreNavigationUndockModuleTest, UndockWaitTimeoutIsBounded)
{
  PreNavigationUndockHarness harness;
  PreNavigationUndockConfig config;
  config.relocalize_after_success = false;
  config.auto_undock_timeout_sec = 0.2;
  config.poll_period = std::chrono::milliseconds(100);
  auto module = harness.make_module(config);
  std::string detail;
  bool performed = false;

  EXPECT_FALSE(module->run_if_needed(required_request(), detail, performed));
  EXPECT_EQ(detail, "timed out waiting for undock before navigation");
  EXPECT_EQ(harness.sleep_count, 2);
}

}  // namespace
}  // namespace robot_api_server::features::docking
