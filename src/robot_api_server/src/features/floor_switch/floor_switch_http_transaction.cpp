#include "robot_api_server/features/floor_switch/floor_switch_http_transaction.hpp"

#include <algorithm>
#include <sstream>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server
{
namespace
{

bool exact_identity(
  const FloorSwitchHttpTarget & target,
  const FloorSwitchHttpOutcome & outcome)
{
  return outcome.active_building_id == target.building_id &&
         outcome.active_floor_id == target.floor_id &&
         outcome.active_map_id == target.map_id &&
         outcome.asset_epoch == target.asset_epoch &&
         outcome.asset_digest == target.asset_digest;
}

}  // namespace

FloorSwitchHttpRuntimeAdmission evaluate_floor_switch_runtime_admission(
  const FloorSwitchHttpTarget & target,
  const FloorSwitchHttpRuntimeContext & runtime)
{
  // Source readiness proves an idempotent no-op, not permission to load a target.
  // In particular, a failed initial localization must not prevent map selection.
  const bool same_logical_map =
    runtime.building_id == target.building_id &&
    runtime.floor_id == target.floor_id &&
    runtime.map_id == target.map_id;
  if (same_logical_map && runtime.available && runtime.confirmed &&
    runtime.state == "ready" && runtime.explicit_relocalization_sequence != 0U &&
    runtime.asset_epoch != 0U && !runtime.asset_digest.empty() &&
    runtime.asset_epoch == target.asset_epoch && runtime.asset_digest == target.asset_digest)
  {
    return {
      true, true, "FLOOR_SWITCH_ALREADY_ACTIVE",
      "the exact requested floor asset is already the confirmed runtime"};
  }
  return {
    true, false, "OK",
    "load and localize the requested target independently of source readiness"};
}

bool FloorSwitchHttpSnapshot::terminal() const noexcept
{
  return state == "COMPLETE" || state == "FAILED" ||
         state == "CANCELLED" || state == "UNKNOWN";
}

FloorSwitchHttpStartDecision FloorSwitchHttpTransaction::start(
  const std::string & transaction_id,
  const FloorSwitchHttpTarget & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    !snapshot_.transaction_id.empty() &&
    (snapshot_.state == "UNKNOWN" || snapshot_.recovery_required))
  {
    return FloorSwitchHttpStartDecision{
      false,
      "FLOOR_SWITCH_RECOVERY_REQUIRED",
      "the previous live floor-switch outcome is not safe to replace",
      snapshot_};
  }
  if (!snapshot_.transaction_id.empty() && !snapshot_.terminal()) {
    return FloorSwitchHttpStartDecision{
      false,
      "FLOOR_SWITCH_TRANSACTION_BUSY",
      "another live floor-switch transaction is still active",
      snapshot_};
  }

  snapshot_ = FloorSwitchHttpSnapshot{};
  snapshot_.transaction_id = transaction_id;
  snapshot_.state = "SUBMITTING";
  snapshot_.stage = "WAIT_GOAL_ACCEPTANCE";
  snapshot_.detail = "strict FloorSwitch action goal is being submitted";
  snapshot_.target = target;
  return FloorSwitchHttpStartDecision{true, "OK", "accepted", snapshot_};
}

bool FloorSwitchHttpTransaction::observe_goal_accepted(
  const std::string & transaction_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.transaction_id != transaction_id || snapshot_.terminal()) {
    return false;
  }
  snapshot_.goal_accepted = true;
  snapshot_.state = snapshot_.cancel_requested ? "CANCELING" : "RUNNING";
  snapshot_.stage = snapshot_.cancel_requested ?
    "CANCEL_REQUESTED" : "GOAL_ACCEPTED";
  snapshot_.detail = snapshot_.cancel_requested ?
    "cancel requested while the strict action goal was being accepted" :
    "strict FloorSwitch action goal accepted";
  return true;
}

bool FloorSwitchHttpTransaction::observe_feedback(
  const std::string & transaction_id,
  const std::string & stage,
  const float progress,
  const std::string & detail,
  const std::uint64_t stage_sequence)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    snapshot_.transaction_id != transaction_id || snapshot_.terminal() ||
    stage_sequence < snapshot_.stage_sequence)
  {
    return false;
  }
  snapshot_.stage_sequence = stage_sequence;
  snapshot_.stage = stage.empty() ? snapshot_.stage : stage;
  snapshot_.progress = std::clamp(progress, 0.0F, 1.0F);
  snapshot_.detail = detail;
  if (!snapshot_.cancel_requested) {
    snapshot_.state = "RUNNING";
  }
  return true;
}

FloorSwitchHttpCancelDecision FloorSwitchHttpTransaction::request_cancel(
  const std::string & transaction_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.transaction_id != transaction_id) {
    return FloorSwitchHttpCancelDecision{
      false,
      "FLOOR_SWITCH_TRANSACTION_NOT_FOUND",
      "the requested live floor-switch transaction is not retained",
      snapshot_};
  }
  if (snapshot_.terminal()) {
    return FloorSwitchHttpCancelDecision{
      false,
      "FLOOR_SWITCH_TRANSACTION_TERMINAL",
      "the live floor-switch transaction is already terminal",
      snapshot_};
  }
  snapshot_.cancel_requested = true;
  snapshot_.state = "CANCELING";
  snapshot_.stage = "CANCEL_REQUESTED";
  snapshot_.detail = "operator cancellation requested; awaiting explicit action terminal";
  return FloorSwitchHttpCancelDecision{true, "OK", "cancel accepted", snapshot_};
}

bool FloorSwitchHttpTransaction::cancel_requested(
  const std::string & transaction_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.transaction_id == transaction_id &&
         snapshot_.cancel_requested && !snapshot_.terminal();
}

bool FloorSwitchHttpTransaction::finish(
  const std::string & transaction_id,
  const FloorSwitchHttpOutcome & outcome)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.transaction_id != transaction_id || snapshot_.terminal()) {
    return false;
  }

  snapshot_.active_building_id = outcome.active_building_id;
  snapshot_.active_floor_id = outcome.active_floor_id;
  snapshot_.active_map_id = outcome.active_map_id;
  snapshot_.active_asset_epoch = outcome.asset_epoch;
  snapshot_.active_asset_digest = outcome.asset_digest;
  snapshot_.explicit_relocalization_sequence =
    outcome.explicit_relocalization_sequence;
  snapshot_.failure_code = outcome.failure_code;
  snapshot_.runtime_context_valid = outcome.runtime_context_valid;
  snapshot_.recovery_required = outcome.recovery_required;
  snapshot_.detail = outcome.detail;
  snapshot_.progress = 1.0F;

  if (!outcome.terminal_proven) {
    snapshot_.state = "UNKNOWN";
    snapshot_.stage = "TERMINAL_UNPROVEN";
    snapshot_.success = false;
    snapshot_.recovery_required = true;
    return true;
  }
  if (outcome.cancelled) {
    snapshot_.state = "CANCELLED";
    snapshot_.stage = "CANCELLED";
    snapshot_.success = false;
    return true;
  }

  const bool proven_success =
    outcome.success && exact_identity(snapshot_.target, outcome) &&
    outcome.runtime_context_valid && !outcome.recovery_required &&
    outcome.failure_code == 0U;
  snapshot_.success = proven_success;
  snapshot_.state = proven_success ? "COMPLETE" : "FAILED";
  snapshot_.stage = proven_success ? "COMPLETE" : "FAILED";
  if (outcome.success && !proven_success && snapshot_.detail.empty()) {
    snapshot_.detail =
      "action reported success without proving the exact target runtime identity";
  }
  return true;
}

std::optional<FloorSwitchHttpSnapshot> FloorSwitchHttpTransaction::snapshot(
  const std::string & transaction_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.transaction_id != transaction_id) {
    return std::nullopt;
  }
  return snapshot_;
}

FloorSwitchHttpSnapshot FloorSwitchHttpTransaction::latest_snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

std::string floor_switch_http_snapshot_json(
  const FloorSwitchHttpSnapshot & snapshot)
{
  std::ostringstream out;
  out << "{"
      << "\"transaction_id\":" << json_string(snapshot.transaction_id) << ","
      << "\"state\":" << json_string(snapshot.state) << ","
      << "\"stage\":" << json_string(snapshot.stage) << ","
      << "\"progress\":" << snapshot.progress << ","
      << "\"detail\":" << json_string(snapshot.detail) << ","
      << "\"building_id\":" << json_string(snapshot.target.building_id) << ","
      << "\"floor_id\":" << json_string(snapshot.target.floor_id) << ","
      << "\"map_id\":" << json_string(snapshot.target.map_id) << ","
      << "\"asset_epoch\":" << snapshot.target.asset_epoch << ","
      << "\"asset_digest\":" << json_string(snapshot.target.asset_digest) << ","
      << "\"active_building_id\":" << json_string(snapshot.active_building_id) << ","
      << "\"active_floor_id\":" << json_string(snapshot.active_floor_id) << ","
      << "\"active_map_id\":" << json_string(snapshot.active_map_id) << ","
      << "\"active_asset_epoch\":" << snapshot.active_asset_epoch << ","
      << "\"active_asset_digest\":" << json_string(snapshot.active_asset_digest) << ","
      << "\"explicit_relocalization_sequence\":"
      << snapshot.explicit_relocalization_sequence << ","
      << "\"stage_sequence\":" << snapshot.stage_sequence << ","
      << "\"failure_code\":" << snapshot.failure_code << ","
      << "\"goal_accepted\":" << (snapshot.goal_accepted ? "true" : "false") << ","
      << "\"cancel_requested\":" << (snapshot.cancel_requested ? "true" : "false") << ","
      << "\"success\":" << (snapshot.success ? "true" : "false") << ","
      << "\"terminal\":" << (snapshot.terminal() ? "true" : "false") << ","
      << "\"runtime_context_valid\":"
      << (snapshot.runtime_context_valid ? "true" : "false") << ","
      << "\"recovery_required\":"
      << (snapshot.recovery_required ? "true" : "false")
      << "}";
  return out.str();
}

}  // namespace robot_api_server
