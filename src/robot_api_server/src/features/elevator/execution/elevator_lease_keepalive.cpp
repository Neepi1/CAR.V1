#include "robot_api_server/features/elevator/execution/elevator_lease_keepalive.hpp"

#include <stdexcept>
#include <utility>

namespace robot_api_server
{

ElevatorLeaseKeepalive::ElevatorLeaseKeepalive(
  const std::chrono::milliseconds period,
  RenewCallback renew_callback,
  FailureCallback failure_callback)
: period_(period),
  renew_callback_(std::move(renew_callback)),
  failure_callback_(std::move(failure_callback))
{
  if (period_.count() <= 0 || !renew_callback_) {
    throw std::invalid_argument("invalid elevator lease keepalive options");
  }
}

ElevatorLeaseKeepalive::~ElevatorLeaseKeepalive()
{
  stop();
}

ElevatorLeaseKeepaliveStatus ElevatorLeaseKeepalive::start()
{
  std::unique_lock<std::mutex> lock(state_mutex_);
  if (running_) {
    return {true, {}, "elevator lease keepalive already running"};
  }
  if (worker_.joinable()) {
    lock.unlock();
    worker_.join();
    lock.lock();
  }
  stop_requested_ = false;
  last_failure_.reset();
  running_ = true;
  try {
    worker_ = std::thread([this]() {worker_loop();});
  } catch (const std::exception & exception) {
    running_ = false;
    stop_requested_ = true;
    return {
      false,
      "ELEVATOR_LEASE_KEEPALIVE_START_FAILED",
      exception.what(),
    };
  } catch (...) {
    running_ = false;
    stop_requested_ = true;
    return {
      false,
      "ELEVATOR_LEASE_KEEPALIVE_START_FAILED",
      "lease keepalive worker could not be started",
    };
  }
  return {true, {}, "elevator lease keepalive started"};
}

void ElevatorLeaseKeepalive::stop() noexcept
{
  request_stop();
  if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) {
    worker_.join();
  }
}

void ElevatorLeaseKeepalive::request_stop() noexcept
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stop_requested_ = true;
    running_ = false;
  }
  state_cv_.notify_all();
}

void ElevatorLeaseKeepalive::reset() noexcept
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (running_ || worker_.joinable()) {
    return;
  }
  stop_requested_ = false;
  last_failure_.reset();
}

ElevatorLeaseKeepaliveStatus ElevatorLeaseKeepalive::renew_now()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (last_failure_) {
      return *last_failure_;
    }
    if (!running_ || stop_requested_) {
      return {
        false,
        "ELEVATOR_LEASE_KEEPALIVE_NOT_RUNNING",
        "lease keepalive is not running",
      };
    }
  }

  ElevatorLeaseKeepaliveStatus status;
  try {
    std::lock_guard<std::mutex> renewal_lock(renewal_mutex_);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (last_failure_) {
        return *last_failure_;
      }
      if (!running_ || stop_requested_) {
        return {
          false,
          "ELEVATOR_LEASE_KEEPALIVE_NOT_RUNNING",
          "lease keepalive stopped before renewal",
        };
      }
    }
    status = renew_callback_();
  } catch (const std::exception & exception) {
    status = {
      false,
      "ELEVATOR_LEASE_KEEPALIVE_CALLBACK_EXCEPTION",
      exception.what(),
    };
  } catch (...) {
    status = {
      false,
      "ELEVATOR_LEASE_KEEPALIVE_CALLBACK_EXCEPTION",
      "lease renewal callback threw an unknown exception",
    };
  }
  return status.success ? status : record_failure(std::move(status));
}

ElevatorLeaseKeepaliveStatus ElevatorLeaseKeepalive::record_failure(
  ElevatorLeaseKeepaliveStatus status)
{
  FailureCallback failure_callback;
  bool first_failure = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!last_failure_) {
      last_failure_ = status;
      first_failure = true;
      failure_callback = failure_callback_;
    } else {
      status = *last_failure_;
    }
    stop_requested_ = true;
    running_ = false;
  }
  state_cv_.notify_all();
  if (first_failure && failure_callback) {
    try {
      failure_callback(status);
    } catch (...) {
      // The failure is already latched. A fail-safe notification hook must not
      // terminate the keepalive worker or erase the original renewal error.
    }
  }
  return status;
}

void ElevatorLeaseKeepalive::worker_loop() noexcept
{
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      if (state_cv_.wait_for(
          lock, period_, [this]() {return stop_requested_;}))
      {
        break;
      }
    }
    if (!renew_now().success) {
      break;
    }
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    running_ = false;
  }
  state_cv_.notify_all();
}

std::optional<ElevatorLeaseKeepaliveStatus>
ElevatorLeaseKeepalive::last_failure() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_failure_;
}

bool ElevatorLeaseKeepalive::running() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return running_;
}

ElevatorLeaseKeepaliveGroup::ElevatorLeaseKeepaliveGroup(
  const std::chrono::milliseconds execution_period,
  RenewCallback execution_renew_callback,
  const std::chrono::milliseconds mode_period,
  RenewCallback mode_renew_callback,
  FailureCallback failure_callback)
: failure_callback_(std::move(failure_callback)),
  execution_keepalive_(
    execution_period,
    std::move(execution_renew_callback),
    [this](const ElevatorLeaseKeepaliveStatus & status) {
      handle_channel_failure(status);
    }),
  mode_keepalive_(
    mode_period,
    std::move(mode_renew_callback),
    [this](const ElevatorLeaseKeepaliveStatus & status) {
      handle_channel_failure(status);
    })
{
}

ElevatorLeaseKeepaliveGroup::~ElevatorLeaseKeepaliveGroup()
{
  stop();
}

ElevatorLeaseKeepaliveStatus ElevatorLeaseKeepaliveGroup::start()
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (running()) {
    return {true, {}, "elevator lease keepalive group already running"};
  }

  execution_keepalive_.stop();
  mode_keepalive_.stop();
  execution_keepalive_.reset();
  mode_keepalive_.reset();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_failure_.reset();
  }

  const auto execution_started = execution_keepalive_.start();
  if (!execution_started.success) {
    return execution_started;
  }
  const auto mode_started = mode_keepalive_.start();
  if (!mode_started.success) {
    execution_keepalive_.request_stop();
    execution_keepalive_.stop();
    return mode_started;
  }

  const auto failure = last_failure();
  if (failure) {
    execution_keepalive_.request_stop();
    mode_keepalive_.request_stop();
    execution_keepalive_.stop();
    mode_keepalive_.stop();
    return *failure;
  }
  return {true, {}, "independent elevator lease keepalives started"};
}

void ElevatorLeaseKeepaliveGroup::stop() noexcept
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  execution_keepalive_.request_stop();
  mode_keepalive_.request_stop();
  execution_keepalive_.stop();
  mode_keepalive_.stop();
}

void ElevatorLeaseKeepaliveGroup::reset() noexcept
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (execution_keepalive_.running() || mode_keepalive_.running()) {
    return;
  }
  execution_keepalive_.reset();
  mode_keepalive_.reset();
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_failure_.reset();
}

ElevatorLeaseKeepaliveStatus ElevatorLeaseKeepaliveGroup::renew_now()
{
  if (const auto failure = last_failure()) {
    return *failure;
  }
  if (!running()) {
    return {
      false,
      "ELEVATOR_LEASE_KEEPALIVE_NOT_RUNNING",
      "independent lease keepalive group is not running",
    };
  }

  const auto execution = execution_keepalive_.renew_now();
  if (!execution.success) {
    return last_failure().value_or(execution);
  }
  const auto mode = mode_keepalive_.renew_now();
  if (!mode.success) {
    return last_failure().value_or(mode);
  }
  return {
    true,
    {},
    "execution and operating-mode leases renewed independently",
  };
}

std::optional<ElevatorLeaseKeepaliveStatus>
ElevatorLeaseKeepaliveGroup::last_failure() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_failure_;
}

bool ElevatorLeaseKeepaliveGroup::running() const
{
  if (last_failure()) {
    return false;
  }
  return execution_keepalive_.running() && mode_keepalive_.running();
}

void ElevatorLeaseKeepaliveGroup::handle_channel_failure(
  const ElevatorLeaseKeepaliveStatus & status) noexcept
{
  FailureCallback failure_callback;
  bool first_failure = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!last_failure_) {
      last_failure_ = status;
      failure_callback = failure_callback_;
      first_failure = true;
    }
  }

  // This path may run on either worker. Requesting stop is non-blocking so two
  // simultaneous failures cannot deadlock by trying to join one another.
  execution_keepalive_.request_stop();
  mode_keepalive_.request_stop();
  if (first_failure && failure_callback) {
    try {
      failure_callback(status);
    } catch (...) {
      // The transaction failure is already latched. Notification must never
      // terminate the worker or replace the original failure evidence.
    }
  }
}

}  // namespace robot_api_server
