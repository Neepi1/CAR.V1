#pragma once

#include <cstdint>
#include <string>

namespace robot_api_server
{

struct FloorRuntimeInterlockDecision
{
  bool blocked{false};
  std::string code{"FLOOR_RUNTIME_LEGACY_UNSCOPED"};
  std::string detail{
    "no current floor-transition or localization evidence has been observed"};
  std::string transaction_id;
};

class FloorRuntimeInterlock
{
public:
  void observe_floor_switch_status(
    const std::string & transaction_id,
    const std::string & state,
    const std::string & stage,
    std::uint16_t failure_code,
    const std::string & detail);

  void observe_localization_health(
    bool transition_active,
    bool runtime_context_valid,
    const std::string & detail);

  FloorRuntimeInterlockDecision decision() const;
  FloorRuntimeInterlockDecision decision_for_map_switch() const;
  // Current evidence only. Source-independent recovery can bypass an invalid
  // source context, never an active transition; this does not grant motion.
  FloorRuntimeInterlockDecision decision_for_operation(const std::string & operation) const;

private:
  FloorRuntimeInterlockDecision floor_status_decision_;
  FloorRuntimeInterlockDecision health_decision_;
  bool have_floor_status_{false};
  bool have_health_{false};
};

}  // namespace robot_api_server
