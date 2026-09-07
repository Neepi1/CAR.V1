#include <atomic>
#include <future>

#include "gtest/gtest.h"

#include "robot_api_server/features/navigation/runtime/navigation_cancel_runtime.hpp"

namespace navigation = robot_api_server::features::navigation;

TEST(NavigationCancelRuntime, OwnsCancelJobAndWorkerLifetime)
{
  navigation::NavigationCancelRuntime runtime;
  navigation::NavigationCancelStartSpec start;
  start.reason = "operator cancel";
  start.stop_stack = true;
  start.started_at = "2026-09-01T00:00:00Z";
  start.zero_velocity_published = true;

  std::promise<void> release_worker;
  auto release = release_worker.get_future().share();
  std::atomic<std::uint64_t> observed_id{0U};
  std::atomic<bool> observed_stop_stack{false};
  const auto job_id = runtime.start(
    start,
    [&observed_id, &observed_stop_stack, release](
      const std::uint64_t id, const bool stop_stack) {
      observed_stop_stack.store(stop_stack, std::memory_order_release);
      observed_id.store(id, std::memory_order_release);
      release.wait();
    });

  EXPECT_TRUE(runtime.running());
  EXPECT_TRUE(runtime.update_running(
      job_id,
      [](robot_api_server::NavigationCancelJob & job) {
        job.phase = "cancel_all_goals";
      }));
  EXPECT_EQ(runtime.snapshot().phase, "cancel_all_goals");

  navigation::NavigationCancelFinishSpec finish;
  finish.ok = true;
  finish.action_available = true;
  finish.cancel_all_requested = true;
  finish.cancel_all_ok = true;
  finish.stop_stack_ok = true;
  finish.detail = "navigation canceled and stack stopped";
  finish.finished_at = "2026-09-01T00:00:01Z";
  EXPECT_TRUE(runtime.finish(job_id, finish));
  EXPECT_FALSE(runtime.running());
  EXPECT_NE(runtime.json().find("navigation canceled"), std::string::npos);

  release_worker.set_value();
  runtime.join();
  EXPECT_EQ(observed_id.load(std::memory_order_acquire), job_id);
  EXPECT_TRUE(observed_stop_stack.load(std::memory_order_acquire));
}
