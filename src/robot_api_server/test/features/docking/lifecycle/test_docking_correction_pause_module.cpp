#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "robot_api_server/features/docking/lifecycle/docking_correction_pause_module.hpp"

namespace
{

using robot_api_server::features::docking::DockingCorrectionPauseConfig;
using robot_api_server::features::docking::DockingCorrectionPauseModule;
using robot_api_server::features::docking::DockingCorrectionPausePorts;
using robot_api_server::features::docking::DockingJobStore;
using robot_api_server::features::docking::DockingJobStorePorts;
using robot_api_server::features::localization::BridgeStatusSnapshot;
using robot_api_server::RobotPoseSnapshot;

DockingJobStorePorts store_ports()
{
  DockingJobStorePorts ports;
  ports.update_dock_contact_latch = [](bool, const auto &, const auto &, const auto &) {};
  ports.finish_runtime = [](const auto &, const auto &) {};
  ports.timestamp_now = []() {return "2026-09-03T00:00:00Z";};
  ports.set_global_correction_paused = [](auto, auto, const auto &, auto &) {return true;};
  return ports;
}

struct Harness
{
  DockingJobStore store{store_ports()};
  BridgeStatusSnapshot bridge;
  RobotPoseSnapshot pose;
  int request_count{0};
  bool requested_paused{false};
  bool requested_enabled{false};
  std::chrono::nanoseconds requested_timeout{0};
  bool request_result{true};
  std::string request_detail{"bridge ack"};
  std::string warning;
  std::string error;

  DockingCorrectionPausePorts ports()
  {
    DockingCorrectionPausePorts result;
    result.request_correction_pause = [this](
      const bool paused,
      std::string & detail,
      const std::chrono::nanoseconds timeout,
      const bool enabled)
      {
        ++request_count;
        requested_paused = paused;
        requested_timeout = timeout;
        requested_enabled = enabled;
        detail = request_detail;
        return request_result;
      };
    result.bridge_status_snapshot = [this]() {return bridge;};
    result.current_robot_pose = [this]() {return pose;};
    result.warn = [this](const std::string & message) {warning = message;};
    result.error = [this](const std::string & message) {error = message;};
    return result;
  }

  void set_job(
    const std::uint64_t id,
    const std::string & state = "running",
    const std::string & phase = "accepted",
    const bool paused = false)
  {
    std::lock_guard<std::mutex> lock(store.mutex());
    auto & job = store.job_unsafe();
    job.id = id;
    job.state = state;
    job.phase = phase;
    job.global_correction_paused = paused;
  }
};

TEST(DockingCorrectionPauseModuleTest, MissingPortsAreRejected)
{
  DockingJobStore store(store_ports());
  EXPECT_THROW(
    DockingCorrectionPauseModule(
      DockingCorrectionPauseConfig{}, store, DockingCorrectionPausePorts{}),
    std::invalid_argument);
}

TEST(DockingCorrectionPauseModuleTest, PauseSuccessUpdatesCanonicalJobAndPose)
{
  Harness harness;
  harness.set_job(7U);
  harness.pose.available = true;
  harness.pose.x = 1.0;
  harness.pose.y = 2.0;
  harness.pose.yaw = 0.3;
  harness.pose.age_sec = 0.1;
  DockingCorrectionPauseConfig config;
  config.enabled = false;
  config.service_timeout = std::chrono::milliseconds(321);
  DockingCorrectionPauseModule module(config, harness.store, harness.ports());

  std::string detail;
  EXPECT_TRUE(module.set_paused(7U, true, "docking_fine", detail));
  EXPECT_EQ(detail, "bridge ack");
  EXPECT_EQ(harness.request_count, 1);
  EXPECT_TRUE(harness.requested_paused);
  EXPECT_FALSE(harness.requested_enabled);
  EXPECT_EQ(harness.requested_timeout, std::chrono::milliseconds(321));

  const auto job = harness.store.snapshot();
  EXPECT_TRUE(job.global_correction_paused);
  EXPECT_EQ(job.pause_reason, "docking_fine");
  EXPECT_EQ(job.display_pose_source, "frozen_map_odom_plus_odom_base");
  EXPECT_DOUBLE_EQ(job.display_pose_x, 1.0);
  EXPECT_NE(job.detail.find("pause global correction reason=docking_fine ok=true"), std::string::npos);
}

TEST(DockingCorrectionPauseModuleTest, FailedPauseRecordsUnpausedState)
{
  Harness harness;
  harness.set_job(8U);
  harness.request_result = false;
  harness.request_detail = "bridge timeout";
  harness.pose.available = true;
  DockingCorrectionPauseModule module({}, harness.store, harness.ports());

  std::string detail;
  EXPECT_FALSE(module.set_paused(8U, true, "docking_fine", detail));
  const auto job = harness.store.snapshot();
  EXPECT_FALSE(job.global_correction_paused);
  EXPECT_TRUE(job.pause_reason.empty());
  EXPECT_EQ(job.display_pose_source, "map_base");
  EXPECT_NE(job.detail.find("ok=false detail=bridge timeout"), std::string::npos);
}

TEST(DockingCorrectionPauseModuleTest, OldJobIdCannotMutateCurrentJob)
{
  Harness harness;
  harness.set_job(9U, "running", "accepted", true);
  DockingCorrectionPauseModule module({}, harness.store, harness.ports());

  std::string detail;
  EXPECT_TRUE(module.set_paused(8U, false, "old_job", detail));
  const auto job = harness.store.snapshot();
  EXPECT_TRUE(job.global_correction_paused);
  EXPECT_EQ(job.phase, "accepted");
}

TEST(DockingCorrectionPauseModuleTest, ActiveFineOwnerPreventsStaleRelease)
{
  Harness harness;
  harness.set_job(10U, "running", "FINE_DOCKING_ENTRY_CHECK", true);
  harness.bridge.available = true;
  harness.bridge.map_odom_correction_paused = true;
  harness.bridge.correction_pause_reason = "docking_fine";
  DockingCorrectionPauseModule module({}, harness.store, harness.ports());

  std::string detail;
  EXPECT_FALSE(module.release_stale_if_needed("navigation", detail));
  EXPECT_EQ(harness.request_count, 0);
  EXPECT_NE(
    detail.find("active docking fine job still owns global correction pause before navigation"),
    std::string::npos);
}

TEST(DockingCorrectionPauseModuleTest, NoStaleEvidenceReturnsWithoutServiceCall)
{
  Harness harness;
  harness.set_job(11U, "failed", "finished", false);
  DockingCorrectionPauseModule module({}, harness.store, harness.ports());

  std::string detail;
  EXPECT_TRUE(module.release_stale_if_needed("resume", detail));
  EXPECT_EQ(harness.request_count, 0);
  EXPECT_EQ(detail, "no stale docking_fine correction pause before resume");
}

TEST(DockingCorrectionPauseModuleTest, StaleBridgePauseIsReleasedAndWarned)
{
  Harness harness;
  harness.set_job(12U, "failed", "finished", false);
  harness.bridge.available = true;
  harness.bridge.map_odom_frozen_due_to_pause = true;
  harness.bridge.correction_pause_reason = "docking_fine";
  DockingCorrectionPauseModule module({}, harness.store, harness.ports());

  std::string detail;
  EXPECT_TRUE(module.release_stale_if_needed("pre_navigation_undock_start", detail));
  EXPECT_EQ(harness.request_count, 1);
  EXPECT_FALSE(harness.requested_paused);
  EXPECT_NE(detail.find("bridge_had_pause=true"), std::string::npos);
  EXPECT_EQ(harness.warning, detail);
  EXPECT_TRUE(harness.error.empty());
}

TEST(DockingCorrectionPauseModuleTest, FailedStaleReleaseIsReportedAsError)
{
  Harness harness;
  harness.set_job(13U, "failed", "finished", true);
  harness.request_result = false;
  harness.request_detail = "service unavailable";
  DockingCorrectionPauseModule module({}, harness.store, harness.ports());

  std::string detail;
  EXPECT_FALSE(module.release_stale_if_needed("status_cleanup", detail));
  EXPECT_EQ(harness.request_count, 1);
  EXPECT_NE(detail.find("job_had_pause=true"), std::string::npos);
  EXPECT_EQ(harness.error, detail);
  EXPECT_TRUE(harness.warning.empty());
}

}  // namespace
