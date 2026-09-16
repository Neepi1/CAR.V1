#include <atomic>
#include <future>
#include <stdexcept>
#include <thread>
#include <gtest/gtest.h>
#include "robot_api_server/features/elevator/execution/elevator_ros_executor.hpp"

using namespace std::chrono_literals;
using robot_api_server::ElevatorRosExecutor;
using robot_api_server::ElevatorRosExecutorUnavailable;

namespace {
constexpr auto known = "Taking data from action client but no ready event";
class TestExecutor : public rclcpp::executors::SingleThreadedExecutor {
public:
  explicit TestExecutor(rclcpp::ExecutorOptions options)
  : SingleThreadedExecutor(options) {}
  std::function<void()> step;
  void spin_once(std::chrono::nanoseconds timeout) override {
    EXPECT_EQ(timeout, 50ms);
    step();
  }
};
class ExecutorTest : public testing::Test {
protected:
  std::shared_ptr<rclcpp::Context> context = std::make_shared<rclcpp::Context>();
  rclcpp::ExecutorOptions options;
  void SetUp() override {context->init(0, nullptr); options.context = context;}
  void TearDown() override {context->shutdown("isolated test complete");}
};

TEST_F(ExecutorTest, KnownExceptionKeepsLoopAndPendingFutureAlive) {
  TestExecutor executor(options);
  std::promise<int> result;
  auto future = result.get_future();
  std::atomic<int> calls{0};
  ElevatorRosExecutor loop(executor, context);
  executor.step = [&] {
    if (++calls == 1) throw std::runtime_error(known);
    result.set_value(73);
    loop.request_stop();
  };
  loop.start();
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  loop.stop();
  EXPECT_EQ(future.get(), 73);
  EXPECT_EQ(loop.known_error_count(), 1u);
  EXPECT_FALSE(loop.failure());
  EXPECT_EQ(calls, 2);
}

TEST_F(ExecutorTest, RepeatedImmediateAndConcurrentStop) {
  for (int i = 0; i < 20; ++i) {
    rclcpp::executors::SingleThreadedExecutor executor(options);
    ElevatorRosExecutor loop(executor, context);
    std::thread starter([&] {loop.start();});
    std::thread stopper([&] {loop.stop();});
    starter.join(); stopper.join(); loop.stop(); loop.stop();
    EXPECT_FALSE(loop.running());
    EXPECT_TRUE(context->is_valid());
  }
}

TEST_F(ExecutorTest, IdleWaitAndLostCancelStillExit) {
  rclcpp::executors::SingleThreadedExecutor executor(options);
  ElevatorRosExecutor loop(executor, context);
  loop.start();
  auto stopped = std::async(std::launch::async, [&] {loop.stop();});
  EXPECT_EQ(stopped.wait_for(1s), std::future_status::ready);
}

TEST_F(ExecutorTest, StopConcurrentWithKnownExceptionNeverReenters) {
  TestExecutor executor(options);
  std::atomic<int> calls{0};
  ElevatorRosExecutor loop(executor, context);
  executor.step = [&] {++calls; loop.request_stop(); throw std::runtime_error(known);};
  loop.start();
  for (int i=0; i<100 && calls==0; ++i) std::this_thread::sleep_for(1ms);
  loop.stop();
  EXPECT_EQ(calls, 1);
  EXPECT_FALSE(loop.failure());
}

TEST_F(ExecutorTest, UnknownErrorsWakeWaitersAndPreserveOriginalException) {
  for (int kind=0; kind<4; ++kind) {
    TestExecutor executor(options);
    std::atomic<int> notifications{0};
    ElevatorRosExecutor loop(executor, context,
      [&] {++notifications; throw std::logic_error("notification failure");});
    executor.step = [kind] {
      if (kind==0) throw std::runtime_error(std::string(known) + "!");
      if (kind==1) throw std::runtime_error(std::string("prefix ") + known);
      if (kind==2) throw std::logic_error("unknown standard error");
      throw 17;
    };
    std::promise<int> result; auto future = result.get_future();
    loop.start();
    EXPECT_THROW(loop.wait_for(future, 5s), ElevatorRosExecutorUnavailable);
    loop.stop();
    EXPECT_EQ(loop.known_error_count(), 0u);
    ASSERT_TRUE(loop.failure());
    EXPECT_GE(notifications, 1);
    try {std::rethrow_exception(loop.failure());}
    catch (const std::exception & e) {EXPECT_NE(std::string(e.what()), "notification failure");}
    catch (int value) {EXPECT_EQ(value, 17); EXPECT_EQ(kind, 3);}
  }
}

TEST_F(ExecutorTest, KnownErrorStormIsLoggedThrottledAndInterruptible) {
  TestExecutor executor(options);
  std::atomic<int> logs{0};
  ElevatorRosExecutor loop(executor, context, {},
    [&](uint64_t count, uint64_t) {EXPECT_GT(count, 0u); ++logs;});
  executor.step = [] {throw std::runtime_error(known);};
  loop.start(); std::this_thread::sleep_for(100ms); loop.stop();
  EXPECT_GT(loop.known_error_count(), 0u);
  EXPECT_LT(loop.known_error_count(), 1000u);
  EXPECT_GT(logs, 0);
  EXPECT_FALSE(loop.failure());
}

TEST_F(ExecutorTest, IdleReturnsAreNotActionProgressAndDoNotCompleteFuture) {
  rclcpp::executors::SingleThreadedExecutor executor(options);
  ElevatorRosExecutor loop(executor, context);
  std::promise<int> result; auto future = result.get_future();
  loop.start();
  EXPECT_EQ(loop.wait_for(future, 120ms), std::future_status::timeout);
  loop.stop();
  EXPECT_EQ(future.wait_for(0ms), std::future_status::timeout);
  EXPECT_THROW(loop.wait_for(future, 2s), ElevatorRosExecutorUnavailable);
}

TEST_F(ExecutorTest, ThrowingRecoveryLoggerAndWorkerStopCannotEscapeOrSelfJoin) {
  TestExecutor executor(options);
  std::atomic<int> calls{0};
  ElevatorRosExecutor loop(executor, context, {},
    [](uint64_t, uint64_t) {throw 91;});
  executor.step=[&] {
    if (++calls==1) {throw std::runtime_error(known);}
    loop.stop();
  };
  loop.start();
  for (int i=0; i<100 && calls<2; ++i) {std::this_thread::sleep_for(1ms);}
  loop.stop();
  EXPECT_EQ(calls, 2); EXPECT_FALSE(loop.failure());
}
}  // namespace
