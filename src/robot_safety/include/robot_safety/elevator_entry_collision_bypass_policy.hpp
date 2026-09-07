#pragma once

#include <string>

namespace robot_safety
{

struct ElevatorEntryCollisionBypassPermit
{
  std::string transaction_id;
  double received_at_sec{0.0};
};

struct ElevatorEntryCollisionBypassContext
{
  bool hold_clear{false};
  bool operating_mode_contract_valid{false};
  std::string operating_mode_owner;
  std::string operating_mode_mission_id;
  std::string operating_mode;
};

// This is deliberately narrower than an ordinary source selector. A fresh
// permit must name the exact active elevator transaction, and robot_safety
// must independently prove the mode owner, mission, hold, and DOORWAY mode
// contract. The elevator-test execution lease is intentionally not involved.
bool elevator_entry_collision_bypass_authorized(
  const ElevatorEntryCollisionBypassPermit & permit,
  const ElevatorEntryCollisionBypassContext & context,
  double now_sec,
  double permit_timeout_sec) noexcept;

}  // namespace robot_safety
