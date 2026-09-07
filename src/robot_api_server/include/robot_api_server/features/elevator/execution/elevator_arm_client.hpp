#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace robot_api_server
{

struct ElevatorArmHttpResponse
{
  int status{599};
  std::string body;
  std::string transport_error;
};

class ElevatorArmHttpTransport
{
public:
  virtual ~ElevatorArmHttpTransport() = default;

  virtual ElevatorArmHttpResponse request(
    const std::string & method,
    const std::string & path,
    const std::string & body,
    std::chrono::milliseconds timeout) = 0;
};

struct ElevatorArmClientOptions
{
  std::string host{"127.0.0.1"};
  std::uint16_t port{8083U};
  std::chrono::milliseconds request_timeout{12000};
  std::chrono::milliseconds task_timeout{180000};
  std::chrono::milliseconds poll_interval{200};
  std::size_t maximum_response_bytes{1024U * 1024U};
};

enum class ElevatorArmOutcomeKind
{
  kSucceeded,
  kManualConfirmationRequired,
  kFailed,
};

struct ElevatorArmOutcome
{
  ElevatorArmOutcomeKind kind{ElevatorArmOutcomeKind::kFailed};
  std::string code{"ELEVATOR_ARM_OPERATION_FAILED"};
  std::string detail;
  std::string task_id;

  bool succeeded() const noexcept
  {
    return kind == ElevatorArmOutcomeKind::kSucceeded;
  }
};

// Owns the functional-test-facing 8083 protocol. It submits ready, button, and
// release tasks in order and waits for each task result. Arm health, pose,
// busy, ready_for_press, and safe_to_drive status are deliberately not used as
// state-machine gates during the current feature-validation phase.
class ElevatorArmClient
{
public:
  explicit ElevatorArmClient(ElevatorArmClientOptions options);
  ElevatorArmClient(
    ElevatorArmClientOptions options,
    std::shared_ptr<ElevatorArmHttpTransport> transport);

  ElevatorArmOutcome press_hall_call(
    const std::string & transaction_id,
    std::uint64_t effect_sequence,
    const std::string & source_floor_id,
    const std::string & target_floor_id,
    const std::function<bool()> & cancellation_requested);
  ElevatorArmOutcome press_floor(
    const std::string & transaction_id,
    std::uint64_t effect_sequence,
    const std::string & target_floor_id,
    const std::function<bool()> & cancellation_requested);

  static std::optional<std::string> floor_button_label(
    const std::string & floor_id);
  static std::optional<std::string> hall_call_direction(
    const std::string & source_floor_id,
    const std::string & target_floor_id);

private:
  ElevatorArmOutcome run_button_sequence(
    const std::string & transaction_id,
    std::uint64_t effect_sequence,
    const std::string & action_path,
    const std::string & action_field,
    const std::string & action_value,
    const std::function<bool()> & cancellation_requested,
    bool hall_call);
  ElevatorArmOutcome run_task(
    const std::string & path,
    const std::string & request_body,
    const std::function<bool()> & cancellation_requested);
  ElevatorArmOutcome await_task(
    const ElevatorArmHttpResponse & accepted_response,
    const std::string & path,
    const std::function<bool()> & cancellation_requested);

  ElevatorArmClientOptions options_;
  std::shared_ptr<ElevatorArmHttpTransport> transport_;
};

}  // namespace robot_api_server
