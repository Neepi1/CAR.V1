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

bool contains_failed_lock(const std::string & value)
{
  return uppercase(value).find("FAILED_LOCKED") != std::string::npos;
}

bool is_active_transition_state(const std::string & value)
{
  static constexpr std::array<const char *, 16> active_states{{
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

}  // namespace

void FloorRuntimeInterlock::observe_floor_switch_status(
  const std::string & transaction_id,
  const std::string & state,
  const std::string & stage,
  const std::uint16_t failure_code,
  const std::string & detail)
{
  have_floor_status_ = true;
  if (floor_status_decision_.blocked &&
    floor_status_decision_.code == "FLOOR_TRANSITION_FAILED_LOCKED")
  {
    // Recovery is not a normal status transition. Keep the first failed lock
    // until the API is rebuilt with an explicit recovery protocol or the
    // complete runtime is restarted.
    return;
  }
  floor_status_decision_ = clear_decision();

  if (contains_failed_lock(state) || contains_failed_lock(stage) ||
    contains_failed_lock(detail))
  {
    floor_status_decision_.blocked = true;
    floor_status_decision_.code = "FLOOR_TRANSITION_FAILED_LOCKED";
  } else if (is_active_transition_state(state) || is_active_transition_state(stage)) {
    floor_status_decision_.blocked = true;
    floor_status_decision_.code = "FLOOR_TRANSITION_ACTIVE";
  } else {
    return;
  }

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
  if (health_decision_.blocked &&
    health_decision_.code == "FLOOR_TRANSITION_FAILED_LOCKED")
  {
    return;
  }
  health_decision_ = clear_decision();

  if (contains_failed_lock(detail)) {
    health_decision_.blocked = true;
    health_decision_.code = "FLOOR_TRANSITION_FAILED_LOCKED";
    health_decision_.detail = "localization bridge reports a failed floor lock: " + detail;
    return;
  }

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
  if (floor_status_decision_.blocked &&
    floor_status_decision_.code == "FLOOR_TRANSITION_FAILED_LOCKED")
  {
    return floor_status_decision_;
  }
  if (health_decision_.blocked &&
    health_decision_.code == "FLOOR_TRANSITION_FAILED_LOCKED")
  {
    return health_decision_;
  }
  if (health_decision_.blocked) {
    auto decision = health_decision_;
    if (decision.transaction_id.empty() && floor_status_decision_.blocked) {
      decision.transaction_id = floor_status_decision_.transaction_id;
    }
    return decision;
  }
  if (floor_status_decision_.blocked) {
    return floor_status_decision_;
  }

  auto decision = clear_decision();
  if (have_floor_status_ || have_health_) {
    decision.code = "FLOOR_RUNTIME_NO_NEGATIVE_EVIDENCE";
    decision.detail =
      "typed floor evidence contains no active transition or failed runtime lock";
  }
  return decision;
}

}  // namespace robot_api_server
