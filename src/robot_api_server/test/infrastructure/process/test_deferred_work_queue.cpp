#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>

#include "robot_api_server/infrastructure/process/deferred_work_queue.hpp"

using namespace std::chrono_literals;

namespace robot_api_server
{
namespace
{

TEST(DeferredWorkQueue, WaitingWorkDoesNotBlockPostingThread)
{
  DeferredWorkQueue queue;
  std::promise<void> response;
  auto response_future = response.get_future();
  std::promise<void> started;
  auto started_future = started.get_future();
  std::promise<void> completed;
  auto completed_future = completed.get_future();
  std::atomic<bool> response_observed{false};

  const auto before_post = std::chrono::steady_clock::now();
  ASSERT_TRUE(queue.post([&]() {
    started.set_value();
    response_observed.store(
      response_future.wait_for(500ms) == std::future_status::ready,
      std::memory_order_release);
    completed.set_value();
  }));
  const auto post_duration = std::chrono::steady_clock::now() - before_post;

  EXPECT_LT(post_duration, 100ms);
  ASSERT_EQ(started_future.wait_for(200ms), std::future_status::ready);
  response.set_value();
  ASSERT_EQ(completed_future.wait_for(500ms), std::future_status::ready);
  EXPECT_TRUE(response_observed.load(std::memory_order_acquire));
}

}  // namespace
}  // namespace robot_api_server
