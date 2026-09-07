#include "robot_api_server/features/safety/safety_state.hpp"

#include <sstream>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::safety
{

SafetyMotionDecision evaluate_motion_allowed(
  const SafetyStateSnapshot & snapshot)
{
  if (!snapshot.motion_allowed_valid) {
    return {true, "motion_allowed unavailable"};
  }
  return {
    snapshot.motion_allowed,
    snapshot.motion_allowed ? "motion allowed" : "motion not allowed"};
}

SafetyHardBlockDecision evaluate_hard_block(
  const SafetyStateSnapshot & snapshot)
{
  if (!snapshot.motion_allowed_valid) {
    return {
      false,
      "motion_allowed unavailable; relying on robot_safety arbitration"};
  }
  if (snapshot.motion_allowed) {
    return {false, "motion allowed"};
  }
  if (snapshot.status == "COMMAND_STALE") {
    return {
      false,
      "motion_allowed false because command stream is stale; "
      "final yaw command may refresh robot_safety"};
  }
  return {
    true,
    "motion not allowed by robot_safety: " + snapshot.status};
}

std::string normal_motion_blocked_reason(
  const SafetyStateSnapshot & snapshot)
{
  return snapshot.status == "DOCKED_CONTACT_BLOCK" ?
         "DOCKED_CONTACT_BLOCK" : std::string{};
}

std::string safety_state_json(const SafetyStateSnapshot & snapshot)
{
  std::ostringstream body;
  body << "{\"status\":" << robot_api_server::json_string(snapshot.status)
       << ",\"motion_allowed\":"
       << (snapshot.motion_allowed ? "true" : "false")
       << ",\"motion_allowed_valid\":"
       << (snapshot.motion_allowed_valid ? "true" : "false")
       << "}";
  return body.str();
}

std::string safety_estop_response_json(const bool active)
{
  return std::string("{\"ok\":true,\"estop\":") +
         (active ? "true" : "false") + "}";
}

}  // namespace robot_api_server::features::safety
