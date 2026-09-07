#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace robot_api_server
{

struct FloorSwitchHttpTarget
{
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
};

struct FloorSwitchHttpRuntimeContext
{
  bool available{false};
  bool confirmed{false};
  std::string state;
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
  std::uint64_t explicit_relocalization_sequence{0U};
};

struct FloorSwitchHttpRuntimeAdmission
{
  bool permitted{false};
  bool already_active{false};
  std::string code;
  std::string detail;
};

FloorSwitchHttpRuntimeAdmission evaluate_floor_switch_runtime_admission(
  const FloorSwitchHttpTarget & target,
  const FloorSwitchHttpRuntimeContext & runtime);

struct FloorSwitchHttpOutcome
{
  bool terminal_proven{true};
  bool cancelled{false};
  bool success{false};
  std::uint16_t failure_code{0U};
  std::string detail;
  std::string active_building_id;
  std::string active_floor_id;
  std::string active_map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
  std::uint64_t explicit_relocalization_sequence{0U};
  bool runtime_context_valid{false};
  bool recovery_required{false};
};

struct FloorSwitchHttpSnapshot
{
  std::string transaction_id;
  std::string state{"IDLE"};
  std::string stage{"IDLE"};
  float progress{0.0F};
  std::string detail;
  FloorSwitchHttpTarget target;
  std::string active_building_id;
  std::string active_floor_id;
  std::string active_map_id;
  std::uint64_t active_asset_epoch{0U};
  std::string active_asset_digest;
  std::uint64_t explicit_relocalization_sequence{0U};
  std::uint64_t stage_sequence{0U};
  std::uint16_t failure_code{0U};
  bool goal_accepted{false};
  bool cancel_requested{false};
  bool success{false};
  bool runtime_context_valid{false};
  bool recovery_required{false};

  bool terminal() const noexcept;
};

struct FloorSwitchHttpStartDecision
{
  bool accepted{false};
  std::string code;
  std::string detail;
  FloorSwitchHttpSnapshot snapshot;
};

struct FloorSwitchHttpCancelDecision
{
  bool accepted{false};
  std::string code;
  std::string detail;
  FloorSwitchHttpSnapshot snapshot;
};

// Thread-safe transaction state shared by HTTP workers, the ROS action worker,
// and action feedback callbacks. The interface deliberately exposes only
// observable transaction events; ROS action types remain in the adapter.
class FloorSwitchHttpTransaction
{
public:
  FloorSwitchHttpStartDecision start(
    const std::string & transaction_id,
    const FloorSwitchHttpTarget & target);
  bool observe_goal_accepted(const std::string & transaction_id);
  bool observe_feedback(
    const std::string & transaction_id,
    const std::string & stage,
    float progress,
    const std::string & detail,
    std::uint64_t stage_sequence);
  FloorSwitchHttpCancelDecision request_cancel(
    const std::string & transaction_id);
  bool cancel_requested(const std::string & transaction_id) const;
  bool finish(
    const std::string & transaction_id,
    const FloorSwitchHttpOutcome & outcome);
  std::optional<FloorSwitchHttpSnapshot> snapshot(
    const std::string & transaction_id) const;
  FloorSwitchHttpSnapshot latest_snapshot() const;

private:
  mutable std::mutex mutex_;
  FloorSwitchHttpSnapshot snapshot_;
};

std::string floor_switch_http_snapshot_json(
  const FloorSwitchHttpSnapshot & snapshot);

}  // namespace robot_api_server
