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
    "no explicit floor-transition failure or active transaction has been observed"};
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

private:
  FloorRuntimeInterlockDecision floor_status_decision_;
  FloorRuntimeInterlockDecision health_decision_;
  bool have_floor_status_{false};
  bool have_health_{false};
};

}  // namespace robot_api_server
