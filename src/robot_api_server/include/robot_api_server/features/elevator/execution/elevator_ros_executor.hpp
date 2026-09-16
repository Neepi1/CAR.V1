#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <exception>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace robot_api_server
{
// Internal transport failure, never a fabricated remote Action terminal result.
class ElevatorRosExecutorUnavailable : public std::runtime_error
{
public:
  ElevatorRosExecutorUnavailable()
  : std::runtime_error("local elevator ROS adapter unavailable; remote action outcome unknown") {}
};

// Owns only the adapter ROS worker. The owner keeps executor/node/clients alive
// through stop(), and joins its business workers before destroying the adapter.
// start is one-shot; stop-before-start permanently prevents starting.
class ElevatorRosExecutor final
{
public:
  using RecoveryLog = std::function<void(uint64_t, uint64_t)>;
  ElevatorRosExecutor(
    rclcpp::executors::SingleThreadedExecutor & executor,
    std::shared_ptr<rclcpp::Context> context,
    std::function<void()> wake_waiters = {}, RecoveryLog recovery_log = {})
  : executor_(executor), context_(std::move(context)),
    wake_waiters_(std::move(wake_waiters)), recovery_log_(std::move(recovery_log)) {}

  ~ElevatorRosExecutor() {stop();}
  ElevatorRosExecutor(const ElevatorRosExecutor &) = delete;
  ElevatorRosExecutor & operator=(const ElevatorRosExecutor &) = delete;

  void start()
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (started_ || stopping_.load()) {return;}
    started_ = true;
    worker_ = std::thread([this]() noexcept {run();});
  }

  void request_stop() noexcept
  {
    stopping_.store(true);
    notify();
    try {executor_.cancel();} catch (...) {record_failure(std::current_exception());}
  }

  void stop() noexcept
  {
    request_stop();
    // A callback can request stop, but only the owning thread joins. No worker
    // ever takes lifecycle_mutex_, including when another caller is joining.
    if (current_worker_ == this) {return;}
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (worker_.joinable()) {worker_.join();}
  }

  bool running() const noexcept {return running_.load();}
  bool unavailable() const noexcept
  {return stopping_.load() || failed_.load() || !context_->is_valid();}
  void check() const {if (unavailable()) {throw ElevatorRosExecutorUnavailable();}}
  uint64_t known_error_count() const noexcept {return known_errors_.load();}
  std::exception_ptr failure() const
  {std::lock_guard<std::mutex> lock(event_mutex_); return failure_;}

  // No business deadline extension and no fixed polling sleep. Completion is
  // checked after each executor turn; a turn/timeout is NOT Action progress.
  template<typename Future, typename Rep, typename Period>
  std::future_status wait_for(
    Future & future, std::chrono::duration<Rep, Period> duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    std::unique_lock<std::mutex> lock(event_mutex_);
    for (;;) {
      check();
      if (future.wait_for(std::chrono::nanoseconds(0)) == std::future_status::ready) {
        return std::future_status::ready;
      }
      if (std::chrono::steady_clock::now() >= deadline) {return std::future_status::timeout;}
      const auto generation = wake_generation_;
      event_cv_.wait_until(lock, deadline, [&] {
        return unavailable() || wake_generation_ != generation;
      });
    }
  }

private:
  void notify() noexcept
  {
    try {
      {std::lock_guard<std::mutex> lock(event_mutex_); ++wake_generation_;}
      event_cv_.notify_all();
      if (wake_waiters_) {wake_waiters_();}
    } catch (...) {
      // Never replace the original exception or throw from the thread boundary.
      std::fputs("elevator adapter: waiter notification failed\n", stderr);
      event_cv_.notify_all();
    }
  }

  void record_failure(std::exception_ptr error) noexcept
  {
    try {
      {std::lock_guard<std::mutex> lock(event_mutex_);
        if (!failure_) {failure_ = error;}}
      failed_.store(true);
      try {std::rethrow_exception(error);}
      catch (const std::exception & e) {
        std::fprintf(stderr, "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY: %s\n", e.what());
      } catch (...) {
        std::fputs("ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY: non-standard exception\n", stderr);
      }
    } catch (...) {
      failed_.store(true);
      std::fputs("elevator adapter: failure reporting failed\n", stderr);
    }
    notify();
  }

  void run() noexcept
  {
    current_worker_ = this;
    running_.store(true);
    try {
      uint64_t consecutive_errors = 0;
      while (!unavailable()) {
        try {
          executor_.spin_once(std::chrono::milliseconds(50));
          consecutive_errors = 0;  // not a claim of feedback/result progress
        } catch (const std::runtime_error & e) {
          if (std::strcmp(e.what(), "Taking data from action client but no ready event") != 0) {
            throw;
          }
          if (unavailable()) {break;}
          const auto count = ++known_errors_;
          ++consecutive_errors;
          if (count == 1 || count % 64 == 0) {
            try {
              if (recovery_log_) {recovery_log_(count, consecutive_errors);}
              else {std::fprintf(stderr,
                  "ELEVATOR_ACTION_NO_READY_EVENT count=%llu consecutive=%llu (not action progress)\n",
                  static_cast<unsigned long long>(count),
                  static_cast<unsigned long long>(consecutive_errors));}
            } catch (...) {
              std::fputs("elevator adapter: recovery logging failed\n", stderr);
            }
          }
          if (consecutive_errors >= 8) {
            std::unique_lock<std::mutex> lock(event_mutex_);
            event_cv_.wait_for(lock, std::chrono::milliseconds(10), [&] {return unavailable();});
          }
        }
        notify();
      }
    } catch (...) {record_failure(std::current_exception());}
    running_.store(false);
    notify();
    current_worker_ = nullptr;
  }

  rclcpp::executors::SingleThreadedExecutor & executor_;
  std::shared_ptr<rclcpp::Context> context_;
  std::function<void()> wake_waiters_;
  RecoveryLog recovery_log_;
  std::atomic<bool> stopping_{false}, failed_{false}, running_{false};
  std::atomic<uint64_t> known_errors_{0};
  mutable std::mutex event_mutex_;
  std::condition_variable event_cv_;
  std::exception_ptr failure_;
  uint64_t wake_generation_{0};
  std::mutex lifecycle_mutex_;
  bool started_{false};
  std::thread worker_;
  inline static thread_local ElevatorRosExecutor * current_worker_{nullptr};
};
}  // namespace robot_api_server
