#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace robot_api_server
{

class ElevatorLeaseKeepaliveGroup;

struct ElevatorLeaseKeepaliveStatus
{
  bool success{true};
  std::string code;
  std::string detail;
};

// Owns periodic renewal independently from any one blocking elevator effect.
// The callback is serialized across background and explicit renewals.
class ElevatorLeaseKeepalive final
{
public:
  using RenewCallback = std::function<ElevatorLeaseKeepaliveStatus()>;
  using FailureCallback =
    std::function<void(const ElevatorLeaseKeepaliveStatus &)>;

  ElevatorLeaseKeepalive(
    std::chrono::milliseconds period,
    RenewCallback renew_callback,
    FailureCallback failure_callback = {});
  ~ElevatorLeaseKeepalive();

  ElevatorLeaseKeepalive(const ElevatorLeaseKeepalive &) = delete;
  ElevatorLeaseKeepalive & operator=(const ElevatorLeaseKeepalive &) = delete;
  ElevatorLeaseKeepalive(ElevatorLeaseKeepalive &&) = delete;
  ElevatorLeaseKeepalive & operator=(ElevatorLeaseKeepalive &&) = delete;

  ElevatorLeaseKeepaliveStatus start();
  void stop() noexcept;
  void reset() noexcept;
  ElevatorLeaseKeepaliveStatus renew_now();
  std::optional<ElevatorLeaseKeepaliveStatus> last_failure() const;
  bool running() const;

private:
  friend class ElevatorLeaseKeepaliveGroup;

  void request_stop() noexcept;
  void worker_loop() noexcept;
  ElevatorLeaseKeepaliveStatus record_failure(
    ElevatorLeaseKeepaliveStatus status);

  std::chrono::milliseconds period_;
  RenewCallback renew_callback_;
  FailureCallback failure_callback_;

  mutable std::mutex state_mutex_;
  std::condition_variable state_cv_;
  std::thread worker_;
  bool running_{false};
  bool stop_requested_{false};
  std::optional<ElevatorLeaseKeepaliveStatus> last_failure_;
  std::mutex renewal_mutex_;
};

// Keeps the execution and operating-mode heartbeats on independent workers.
// A slow mode service must never consume the shorter execution-lease TTL.
// Either channel's first failure is latched for the whole transaction and
// requests both workers to stop; stop() performs the final joins before lease
// release.
class ElevatorLeaseKeepaliveGroup final
{
public:
  using RenewCallback = ElevatorLeaseKeepalive::RenewCallback;
  using FailureCallback = ElevatorLeaseKeepalive::FailureCallback;

  ElevatorLeaseKeepaliveGroup(
    std::chrono::milliseconds execution_period,
    RenewCallback execution_renew_callback,
    std::chrono::milliseconds mode_period,
    RenewCallback mode_renew_callback,
    FailureCallback failure_callback = {});
  ~ElevatorLeaseKeepaliveGroup();

  ElevatorLeaseKeepaliveGroup(const ElevatorLeaseKeepaliveGroup &) = delete;
  ElevatorLeaseKeepaliveGroup & operator=(
    const ElevatorLeaseKeepaliveGroup &) = delete;
  ElevatorLeaseKeepaliveGroup(ElevatorLeaseKeepaliveGroup &&) = delete;
  ElevatorLeaseKeepaliveGroup & operator=(
    ElevatorLeaseKeepaliveGroup &&) = delete;

  ElevatorLeaseKeepaliveStatus start();
  void stop() noexcept;
  void reset() noexcept;
  ElevatorLeaseKeepaliveStatus renew_now();
  std::optional<ElevatorLeaseKeepaliveStatus> last_failure() const;
  bool running() const;

private:
  void handle_channel_failure(
    const ElevatorLeaseKeepaliveStatus & status) noexcept;

  mutable std::mutex state_mutex_;
  mutable std::mutex lifecycle_mutex_;
  std::optional<ElevatorLeaseKeepaliveStatus> last_failure_;
  FailureCallback failure_callback_;
  ElevatorLeaseKeepalive execution_keepalive_;
  ElevatorLeaseKeepalive mode_keepalive_;
};

}  // namespace robot_api_server
