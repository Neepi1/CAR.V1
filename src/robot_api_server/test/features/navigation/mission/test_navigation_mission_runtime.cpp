#include <atomic>
#include <future>
#include <string>

#include "gtest/gtest.h"

#include "robot_api_server/features/navigation/mission/navigation_mission_runtime.hpp"

namespace navigation = robot_api_server::features::navigation;

TEST(NavigationMissionRuntime, OwnsJobMutationCancellationAndWorkerLifetime)
{
  navigation::NavigationMissionRuntime runtime;
  navigation::NavigationGoalJobStartSpec start;
  start.pose_id = "delivery_01";
  start.started_at = "2026-09-01T00:00:00Z";

  std::promise<void> release_worker;
  auto release = release_worker.get_future().share();
  std::atomic<std::uint64_t> observed_job_id{0U};
  const auto job_id = runtime.start(
    start,
    [&observed_job_id, release](const std::uint64_t id) {
      observed_job_id.store(id, std::memory_order_release);
      release.wait();
    });

  EXPECT_GT(job_id, 0U);
  EXPECT_TRUE(runtime.running());
  EXPECT_TRUE(runtime.update(
      job_id,
      [](navigation::NavigationGoalJob & job) {
        job.phase = "waiting_for_nav2_result";
        job.detail = "test update";
      }));
  EXPECT_EQ(runtime.snapshot(job_id)->phase, "waiting_for_nav2_result");

  EXPECT_TRUE(runtime.request_cancel("operator requested cancel"));
  std::string cancel_detail;
  EXPECT_TRUE(runtime.cancel_requested(job_id, cancel_detail));
  EXPECT_EQ(cancel_detail, "operator requested cancel");

  navigation::NavigationGoalJobFinishSpec finish;
  finish.succeeded = false;
  finish.phase = "canceled";
  finish.detail = "canceled for test";
  finish.completed_at = "2026-09-01T00:00:01Z";
  EXPECT_TRUE(runtime.finish(job_id, finish));
  EXPECT_FALSE(runtime.running());
  EXPECT_NE(runtime.json().find("canceled for test"), std::string::npos);

  release_worker.set_value();
  runtime.shutdown();
  EXPECT_EQ(observed_job_id.load(std::memory_order_acquire), job_id);
}

TEST(NavigationMissionRuntime, RejectsMutationForSupersededIdentity)
{
  navigation::NavigationMissionRuntime runtime;
  EXPECT_FALSE(runtime.update(
      999U,
      [](navigation::NavigationGoalJob &) {}));
  std::string detail;
  EXPECT_TRUE(runtime.cancel_requested(999U, detail));
  EXPECT_EQ(detail, "navigation goal job was superseded");
}
