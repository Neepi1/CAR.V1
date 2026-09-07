#pragma once

#include <string>

namespace robot_api_server::features::safety
{

struct SafetyStateSnapshot
{
  std::string status{"UNKNOWN"};
  bool motion_allowed{false};
  bool motion_allowed_valid{false};
};

struct SafetyMotionDecision
{
  bool allowed{false};
  std::string detail;
};

struct SafetyHardBlockDecision
{
  bool blocked{false};
  std::string detail;
};

SafetyMotionDecision evaluate_motion_allowed(
  const SafetyStateSnapshot & snapshot);

SafetyHardBlockDecision evaluate_hard_block(
  const SafetyStateSnapshot & snapshot);

std::string normal_motion_blocked_reason(
  const SafetyStateSnapshot & snapshot);

std::string safety_state_json(const SafetyStateSnapshot & snapshot);

std::string safety_estop_response_json(bool active);

}  // namespace robot_api_server::features::safety
