#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

#include "robot_map_asset_identity/map_asset_identity.hpp"

namespace robot_api_server
{

namespace elevator_floor_retry_detail
{

inline bool valid_identifier(const std::string_view value)
{
  return !value.empty() && value.size() <= 128U && value != "." &&
         value.find("..") == std::string_view::npos &&
         std::all_of(value.begin(), value.end(), [](const unsigned char character) {
      return (character >= 'a' && character <= 'z') ||
             (character >= 'A' && character <= 'Z') ||
             (character >= '0' && character <= '9') ||
             character == '-' || character == '_' || character == '.';
    });
}

}  // namespace elevator_floor_retry_detail

inline std::string floor_attempt_prefix(const std::string_view outer_transaction_id)
{
  if (!elevator_floor_retry_detail::valid_identifier(outer_transaction_id)) {
    throw std::invalid_argument("floor retry requires a valid outer transaction ID");
  }
  return "elv-floor-" + robot_map_asset_identity::sha256_hex(outer_transaction_id) + "-";
}

// The normal first attempt keeps its existing outer ID at the caller. Every
// retry consumes an already-reserved persistent command sequence; this helper
// neither allocates a sequence nor makes a volatile retry counter persistent.
inline std::string make_floor_retry_transaction_id(
  const std::string_view outer_transaction_id,
  const std::uint64_t persistent_sequence)
{
  if (persistent_sequence == 0U) {
    throw std::invalid_argument("floor retry requires a reserved nonzero persistent sequence");
  }
  return floor_attempt_prefix(outer_transaction_id) + std::to_string(persistent_sequence);
}

// This is ownership observation, not permission to release another module's resource.
inline bool floor_transaction_belongs_to(
  const std::string_view outer_transaction_id, const std::string_view candidate)
{
  if (!elevator_floor_retry_detail::valid_identifier(outer_transaction_id) ||
    !elevator_floor_retry_detail::valid_identifier(candidate))
  {
    return false;
  }
  if (candidate == outer_transaction_id) {
    return true;
  }
  const auto prefix = floor_attempt_prefix(outer_transaction_id);
  if (candidate.size() <= prefix.size() || candidate.substr(0U, prefix.size()) != prefix) {
    return false;
  }
  const auto suffix = candidate.substr(prefix.size());
  if (suffix.front() == '0') {
    return false;
  }
  std::uint64_t sequence = 0U;
  for (const auto character : suffix) {
    if (character < '0' || character > '9') {
      return false;
    }
    const auto digit = static_cast<std::uint64_t>(character - '0');
    if (sequence > (std::numeric_limits<std::uint64_t>::max() - digit) / 10U) {
      return false;
    }
    sequence = sequence * 10U + digit;
  }
  return sequence != 0U;
}

inline bool floor_resource_belongs_to(
  const std::string_view outer_transaction_id, const std::string_view resource_key)
{
  constexpr std::string_view owner_prefix = "robot_floor_manager:";
  return resource_key.size() > owner_prefix.size() &&
         resource_key.substr(0U, owner_prefix.size()) == owner_prefix &&
         floor_transaction_belongs_to(outer_transaction_id, resource_key.substr(owner_prefix.size()));
}

// Values are the existing robot_interfaces/action/FloorSwitch result contract.
// This does not prove an Action terminal state or that its writes/resources settled.
// The caller must separately require explicit ABORTED and cleanup evidence.
inline bool floor_failure_code_retryable(const std::uint16_t code)
{
  switch (code) {
    case 11U:  // TRANSACTION_CONFLICT
    case 30U:  // MOTION_HOLD_UNPROVEN
    case 31U:  // NAV_IDLE_UNPROVEN
    case 32U:  // STOPPED_UNPROVEN
    case 33U:  // EVIDENCE_STALE
    case 34U:  // RUNTIME_CONTEXT_UNPROVEN
    case 40U:  // LOCALIZER_RELOAD_UNPROVEN
      return true;
    default:
      return false;
  }
}

// Called only after the exact old Action has reached a proven terminal result.
// same_attempt_cleanup_complete means later typed cleanup completion for that
// attempt, never FAILED_LOCKED, an older generation, or another transaction.
inline bool floor_attempt_cleanup_ready(
  const bool result_requires_recovery,
  const bool same_attempt_cleanup_complete,
  const bool fresh_resources_absent)
{
  return fresh_resources_absent &&
         (!result_requires_recovery || same_attempt_cleanup_complete);
}

}  // namespace robot_api_server
