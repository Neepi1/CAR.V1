#include "robot_safety/elevator_entry_collision_bypass_policy.hpp"

#include <cmath>

namespace robot_safety
{

bool elevator_entry_collision_bypass_authorized(
  const ElevatorEntryCollisionBypassPermit & permit,
  const ElevatorEntryCollisionBypassContext & context,
  const double now_sec,
  const double permit_timeout_sec) noexcept
{
  if (
    permit.transaction_id.empty() ||
    !std::isfinite(permit.received_at_sec) ||
    !std::isfinite(now_sec) ||
    !std::isfinite(permit_timeout_sec) ||
    permit_timeout_sec <= 0.0 ||
    now_sec < permit.received_at_sec ||
    now_sec - permit.received_at_sec > permit_timeout_sec)
  {
    return false;
  }

  return context.hold_clear &&
         context.operating_mode_contract_valid &&
         context.operating_mode_owner == "robot_elevator_manager" &&
         context.operating_mode_mission_id ==
         "elevator_" + permit.transaction_id &&
         context.operating_mode == "DOORWAY";
}

}  // namespace robot_safety
