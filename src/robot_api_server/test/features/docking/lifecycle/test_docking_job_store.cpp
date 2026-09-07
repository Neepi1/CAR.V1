#include <gtest/gtest.h>

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "robot_api_server/features/docking/lifecycle/docking_job_store.hpp"

namespace
{

using robot_api_server::features::docking::DockingJobStore;
using robot_api_server::features::docking::DockingJobStorePorts;

TEST(DockingJobStore, ScopesPhaseCancelAndNavigationEvidenceToCurrentJob)
{
  DockingJobStorePorts ports;
  ports.update_dock_contact_latch = [](bool, const auto &, const auto &, const auto &) {};
  ports.finish_runtime = [](const auto &, const auto &) {};
  ports.timestamp_now = []() {return "2026-09-02T00:00:00Z";};
  ports.set_global_correction_paused = [](auto, auto, const auto &, auto &) {return true;};
  DockingJobStore store(std::move(ports));

  {
    std::lock_guard<std::mutex> lock(store.mutex());
    store.job_unsafe().id = 7U;
    store.job_unsafe().state = "running";
    store.job_unsafe().cancel_requested = true;
  }

  store.set_phase(6U, "wrong_job");
  store.mark_navigation_goal_sent(6U);
  EXPECT_FALSE(store.cancel_requested(6U));
  store.set_phase(7U, "NAV_TO_STAGING_NATIVE_NAV2");
  store.mark_navigation_goal_sent(7U);
  EXPECT_TRUE(store.cancel_requested(7U));

  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.phase, "NAV_TO_STAGING_NATIVE_NAV2");
  EXPECT_TRUE(snapshot.nav_goal_sent);
}

TEST(DockingJobStore, FinishesOnceAndReleasesPauseOutsideJobLock)
{
  std::vector<std::string> latch_events;
  std::vector<std::string> runtime_events;
  bool pause_callback_observed_unlocked = false;
  DockingJobStore * store_ptr = nullptr;
  DockingJobStorePorts ports;
  ports.update_dock_contact_latch =
    [&](const bool docked, const auto & source, const auto &, const auto & dock_id) {
      latch_events.push_back(
        std::string(docked ? "docked:" : "undocked:") + source + ":" + dock_id);
    };
  ports.finish_runtime = [&](const auto & state, const auto &) {
      runtime_events.push_back(state);
    };
  ports.timestamp_now = []() {return "2026-09-02T00:00:00Z";};
  ports.set_global_correction_paused =
    [&](auto, const bool paused, const auto &, auto &) {
      EXPECT_FALSE(paused);
      std::unique_lock<std::mutex> lock(store_ptr->mutex(), std::try_to_lock);
      pause_callback_observed_unlocked = lock.owns_lock();
      return true;
    };
  DockingJobStore store(std::move(ports));
  store_ptr = &store;

  {
    std::lock_guard<std::mutex> lock(store.mutex());
    auto & job = store.job_unsafe();
    job.id = 11U;
    job.state = "running";
    job.dock_id = "dock_a";
    job.global_correction_paused = true;
  }
  store.finish(11U, true, "docked", "contact verified");

  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.state, "docked");
  EXPECT_EQ(snapshot.phase, "finished");
  EXPECT_TRUE(snapshot.ok);
  EXPECT_EQ(snapshot.finished_at, "2026-09-02T00:00:00Z");
  ASSERT_EQ(latch_events.size(), 1U);
  EXPECT_EQ(latch_events.front(), "docked:docking_job:dock_a");
  ASSERT_EQ(runtime_events.size(), 1U);
  EXPECT_EQ(runtime_events.front(), "docked");
  EXPECT_TRUE(pause_callback_observed_unlocked);
}

TEST(DockingJobStore, FailureCodeAndPostUndockJsonRemainCanonical)
{
  DockingJobStorePorts ports;
  ports.update_dock_contact_latch = [](bool, const auto &, const auto &, const auto &) {};
  ports.finish_runtime = [](const auto &, const auto &) {};
  ports.timestamp_now = []() {return "2026-09-02T00:00:00Z";};
  ports.set_global_correction_paused = [](auto, auto, const auto &, auto &) {return true;};
  DockingJobStore store(std::move(ports));
  {
    std::lock_guard<std::mutex> lock(store.mutex());
    auto & job = store.job_unsafe();
    job.id = 12U;
    job.state = "running";
    job.post_undock_navigation_readiness_failure_code = "POST_UNDOCK_SETTLE_TIMEOUT";
    EXPECT_NE(
      store.post_undock_settle_json_locked().find("POST_UNDOCK_SETTLE_TIMEOUT"),
      std::string::npos);
  }

  store.finish_with_code(12U, "PREDOCK_YAW_HARD_FAIL", "yaw outside hard limit");
  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.state, "failed");
  EXPECT_FALSE(snapshot.ok);
  EXPECT_EQ(snapshot.failure_code, "PREDOCK_YAW_HARD_FAIL");
  EXPECT_EQ(snapshot.last_error_code, "PREDOCK_YAW_HARD_FAIL");
  EXPECT_NE(snapshot.detail.find("yaw outside hard limit"), std::string::npos);
}

}  // namespace
