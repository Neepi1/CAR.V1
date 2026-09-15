#include "robot_api_server/features/floor_switch/floor_runtime_interlock.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <sstream>

namespace robot_api_server
{
namespace
{

std::string uppercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {
      return static_cast<char>(std::toupper(character));
    });
  return value;
}

bool is_terminal_transition_state(const std::string & value)
{
  static constexpr std::array<const char *, 8> terminal_states{{
      "FAILED", "FAILED_LOCKED", "CANCELLED", "CANCELED",
      "COMPLETE", "COMPLETED", "BLOCKED", "REJECTED",
    }};
  const auto normalized = uppercase(value);
  return std::find(terminal_states.begin(), terminal_states.end(), normalized) !=
         terminal_states.end();
}

bool is_active_transition_state(const std::string & value)
{
  static constexpr std::array<const char *, 17> active_states{{
      "RUNNING",
      "VERIFYING_PRECONDITIONS",
      "ACQUIRING_CORRECTION_PAUSE",
      "INVALIDATING_RUNTIME_CONTEXT",
      "REPORTING_BEGIN_READY",
      "VERIFYING_PAUSE_HANDOFF",
      "LOADING_NAV_MAP",
      "LOADING_FILTERS",
      "RELOADING_LOCALIZER",
      "RELEASING_FLOOR_PAUSE",
      "TRIGGERING_EXPLICIT_LOCALIZATION",
      "VERIFYING_BRIDGE_READY",
      "CLEARING_COSTMAPS",
      "VERIFYING_FRESH_COSTMAPS",
      "COMMITTING_RUNTIME_CONTEXT",
      "COMPLETING",
      "FAILURE_CLEANUP",
    }};
  const auto normalized = uppercase(value);
  return std::find(active_states.begin(), active_states.end(), normalized) !=
         active_states.end();
}

FloorRuntimeInterlockDecision clear_decision()
{
  return FloorRuntimeInterlockDecision{};
}

bool is_offline_asset_edit(const std::string & operation)
{
  // Exact, audited operations only. "Save" is not sufficient: mapping saves,
  // live-pose capture and keepout edits can change or depend on runtime state.
  static constexpr std::array<const char *, 12> operations{{
    "elevator_config_save_draft", "elevator_config_save_draft_commit",
    "elevator_config_publish", "elevator_config_publish_commit",
    "elevator_config_rollback", "elevator_config_rollback_commit",
    "pose_save", "pose_save_commit", "pose_delete", "pose_delete_commit",
    "pose_batch_replace", "pose_batch_replace_commit",
  }};
  return std::find(operations.begin(), operations.end(), operation) != operations.end();
}

bool is_source_independent_operation(const std::string & operation)
{
  // These paths establish a new runtime or operate on mapping/explicit assets.
  // Keep entry, worker and commit checks aligned. Their own exact asset,
  // lifecycle, motion-admission and pending-side-effect checks still apply.
  static constexpr std::array<const char *, 17> operations{{
    "mapping_start", "mapping_start_worker", "mapping_start_context_clear",
    "mapping_process_launch", "mapping_save", "mapping_save_commit",
    "navigation_start", "navigation_runtime_resume", "navigation_runtime_resume_commit",
    "navigation_runtime_launch", "manual_localization", "localization_trigger",
    "live_floor_switch_start", "floor_switch", "floor_switch_noop_commit",
    "floor_switch_submit", "floor_selection_commit",
  }};
  return std::find(operations.begin(), operations.end(), operation) != operations.end();
}

}  // namespace

void FloorRuntimeInterlock::observe_floor_switch_status(
  const std::string & transaction_id,
  const std::string & state,
  const std::string & stage,
  const std::uint16_t failure_code,
  const std::string & detail)
{
  have_floor_status_ = true;
  const bool incoming_active = !is_terminal_transition_state(state) &&
    (is_active_transition_state(state) || is_active_transition_state(stage));
  if (floor_status_decision_.blocked && !transaction_id.empty() &&
    transaction_id != floor_status_decision_.transaction_id && !incoming_active)
  {
    // A rejected concurrent request reports its own terminal/preflight status,
    // not completion of the executor that still owns the active transaction.
    return;
  }
  floor_status_decision_ = clear_decision();
  // A terminal record may retain its last running stage or failure text. It is
  // task history, not proof that runtime resources remain actively owned.
  // Current health and the authoritative executor's admission check prove that.
  if (!incoming_active) {
    return;
  }

  floor_status_decision_.blocked = true;
  floor_status_decision_.code = "FLOOR_TRANSITION_ACTIVE";

  floor_status_decision_.transaction_id = transaction_id;
  std::ostringstream message;
  message << "floor switch transaction";
  if (!transaction_id.empty()) {
    message << " " << transaction_id;
  }
  message << " is blocked at state=" << state << " stage=" << stage;
  if (failure_code != 0U) {
    message << " failure_code=" << failure_code;
  }
  if (!detail.empty()) {
    message << ": " << detail;
  }
  floor_status_decision_.detail = message.str();
}

void FloorRuntimeInterlock::observe_localization_health(
  const bool transition_active,
  const bool runtime_context_valid,
  const std::string & detail)
{
  have_health_ = true;
  health_decision_ = clear_decision();
  // Human-readable failure details never override the current typed fields.
  if (transition_active || !runtime_context_valid) {
    health_decision_.blocked = true;
    health_decision_.code = transition_active ?
      "FLOOR_TRANSITION_ACTIVE" : "FLOOR_RUNTIME_CONTEXT_INVALID";
    health_decision_.detail = transition_active ?
      "localization bridge reports an active floor transition" :
      "localization bridge reports an invalid floor runtime context";
    if (!detail.empty()) {
      health_decision_.detail += ": " + detail;
    }
  }
}

FloorRuntimeInterlockDecision FloorRuntimeInterlock::decision() const
{
  // Either source can prove a current conflict. Do not let an invalid source
  // context obscure it and subsequently get bypassed by a recovery operation.
  if (floor_status_decision_.blocked) {
    return floor_status_decision_;
  }
  if (health_decision_.blocked) {
    return health_decision_;
  }

  auto decision = clear_decision();
  if (have_floor_status_ || have_health_) {
    decision.code = "FLOOR_RUNTIME_NO_NEGATIVE_EVIDENCE";
    decision.detail =
      "current typed floor evidence contains no active transition or invalid runtime context";
  }
  return decision;
}

FloorRuntimeInterlockDecision FloorRuntimeInterlock::decision_for_map_switch() const
{
  auto observation = *this;
  if (observation.health_decision_.code == "FLOOR_RUNTIME_CONTEXT_INVALID") {
    observation.health_decision_ = clear_decision();
  }
  return observation.decision();
}

FloorRuntimeInterlockDecision FloorRuntimeInterlock::decision_for_operation(
  const std::string & operation) const
{
  auto result = decision();
  if (result.code == "FLOOR_RUNTIME_CONTEXT_INVALID" &&
    (is_offline_asset_edit(operation) || is_source_independent_operation(operation)))
  {
    // Admission is not recovery completion. The original invalid evidence is
    // retained for goal submission, live-pose capture and readiness diagnostics.
    result.blocked = false;
    result.code = is_offline_asset_edit(operation) ?
      "FLOOR_RUNTIME_OFFLINE_ASSET_EDIT_ALLOWED" : "FLOOR_RUNTIME_SOURCE_INDEPENDENT_ALLOWED";
    result.detail = "operation does not require a valid source runtime; current context remains invalid";
  }
  return result;
}

}  // namespace robot_api_server
