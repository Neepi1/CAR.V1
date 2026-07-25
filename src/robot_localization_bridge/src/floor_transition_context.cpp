#include "robot_localization_bridge/floor_transition_context.hpp"

#include <algorithm>
#include <cctype>

namespace robot_localization_bridge
{
namespace
{

bool safe_identifier(const std::string & value)
{
  if (
    value.empty() || value == "." || value.size() > 128U ||
    value.find("..") != std::string::npos ||
    value.find('/') != std::string::npos ||
    value.find('\\') != std::string::npos)
  {
    return false;
  }
  return std::all_of(
    value.begin(), value.end(),
    [](const unsigned char character) {
      return std::isalnum(character) != 0 ||
             character == static_cast<unsigned char>('-') ||
             character == static_cast<unsigned char>('_') ||
             character == static_cast<unsigned char>('.');
    });
}

bool canonical_sha256(const std::string & value)
{
  constexpr char prefix[] = "sha256:";
  constexpr std::size_t prefix_size = sizeof(prefix) - 1U;
  if (value.size() != prefix_size + 64U || value.compare(0U, prefix_size, prefix) != 0) {
    return false;
  }
  return std::all_of(
    value.begin() + static_cast<std::ptrdiff_t>(prefix_size), value.end(),
    [](const unsigned char character) {
      return std::isdigit(character) != 0 ||
             (character >= static_cast<unsigned char>('a') &&
             character <= static_cast<unsigned char>('f'));
    });
}

bool valid_identity(const FloorTransitionIdentity & identity)
{
  return safe_identifier(identity.transaction_id) &&
         safe_identifier(identity.building_id) &&
         safe_identifier(identity.floor_id) &&
         safe_identifier(identity.map_id) &&
         identity.asset_epoch > 0U &&
         canonical_sha256(identity.asset_digest);
}

bool same_identity(
  const FloorTransitionIdentity & left,
  const FloorTransitionIdentity & right)
{
  return left.transaction_id == right.transaction_id &&
         left.building_id == right.building_id &&
         left.floor_id == right.floor_id &&
         left.map_id == right.map_id &&
         left.asset_epoch == right.asset_epoch &&
         left.asset_digest == right.asset_digest;
}

bool same_asset(
  const FloorTransitionIdentity & left,
  const FloorTransitionIdentity & right)
{
  return left.building_id == right.building_id &&
         left.floor_id == right.floor_id &&
         left.map_id == right.map_id &&
         left.asset_epoch == right.asset_epoch &&
         left.asset_digest == right.asset_digest;
}

}  // namespace

FloorTransitionContextDecision FloorTransitionContext::begin(
  const FloorTransitionIdentity & identity,
  const bool floor_pause_owned,
  const std::uint64_t current_explicit_relocalization_sequence)
{
  if (!valid_identity(identity)) {
    return decision(
      false, false, FloorTransitionDecisionCode::kInvalidRequest,
      "BEGIN requires safe identifiers, nonzero epoch, and canonical sha256 digest");
  }
  if (state_.failed_locked) {
    return decision(
      false, false, FloorTransitionDecisionCode::kFailedLocked,
      "floor transition is failure-locked and requires an explicit recovery protocol");
  }
  if (state_.transition_active) {
    if (same_identity(state_.pending, identity)) {
      return decision(
        true, true, FloorTransitionDecisionCode::kOk,
        "idempotent BEGIN replay");
    }
    return decision(
      false, false, FloorTransitionDecisionCode::kConflict,
      "another floor transition is already active");
  }
  if (!floor_pause_owned) {
    return decision(
      false, false, FloorTransitionDecisionCode::kPauseUnproven,
      "exact robot_floor_manager correction-pause lease is not held");
  }

  state_.pending = identity;
  state_.transition_active = true;
  state_.runtime_context_valid = false;
  state_.recovery_required = true;
  state_.begin_explicit_relocalization_sequence =
    current_explicit_relocalization_sequence;
  state_.detail = "BEGIN_HELD";
  return decision(
    true, false, FloorTransitionDecisionCode::kOk,
    "source runtime context invalidated under floor-manager correction pause");
}

FloorTransitionContextDecision FloorTransitionContext::commit(
  const FloorTransitionIdentity & identity,
  const FloorTransitionCommitEvidence & evidence)
{
  if (!valid_identity(identity)) {
    return decision(
      false, false, FloorTransitionDecisionCode::kInvalidRequest,
      "COMMIT contains an invalid floor identity");
  }
  if (
    !state_.transition_active && state_.runtime_context_valid &&
    same_asset(state_.active, identity))
  {
    return decision(
      true, true, FloorTransitionDecisionCode::kOk,
      "idempotent COMMIT replay");
  }
  if (!state_.transition_active || !same_identity(state_.pending, identity)) {
    return decision(
      false, false, FloorTransitionDecisionCode::kIdentityMismatch,
      "COMMIT does not match the active pending transaction");
  }
  if (!evidence.correction_pause_released) {
    return decision(
      false, false, FloorTransitionDecisionCode::kPauseUnproven,
      "all correction-pause leases must be released before COMMIT");
  }
  if (
    evidence.explicit_relocalization_sequence <=
    state_.begin_explicit_relocalization_sequence)
  {
    return decision(
      false, false, FloorTransitionDecisionCode::kExplicitRelocalizationUnproven,
      "COMMIT requires a new explicit relocalization sequence after BEGIN");
  }
  if (
    !evidence.map_odom_valid || evidence.correction_active ||
    evidence.target_sequence == 0U ||
    evidence.current_sequence != evidence.target_sequence ||
    evidence.last_published_sequence < evidence.target_sequence)
  {
    return decision(
      false, false, FloorTransitionDecisionCode::kMapOdomUnsettled,
      "map->odom target is not valid, settled, and published");
  }

  state_.active = state_.pending;
  state_.pending = FloorTransitionIdentity{};
  state_.transition_active = false;
  state_.runtime_context_valid = true;
  state_.failed_locked = false;
  state_.recovery_required = false;
  state_.accepted_explicit_relocalization_sequence =
    evidence.explicit_relocalization_sequence;
  state_.detail = "COMMITTED";
  return decision(
    true, false, FloorTransitionDecisionCode::kOk,
    "target floor runtime context committed");
}

FloorTransitionContextDecision FloorTransitionContext::abort(
  const FloorTransitionIdentity & identity)
{
  if (state_.failed_locked && same_identity(state_.pending, identity)) {
    return decision(
      true, true, FloorTransitionDecisionCode::kOk,
      "idempotent ABORT replay; failure lock retained");
  }
  if (!state_.transition_active || !same_identity(state_.pending, identity)) {
    return decision(
      false, false, FloorTransitionDecisionCode::kIdentityMismatch,
      "ABORT does not match the active pending transaction");
  }

  state_.transition_active = false;
  state_.runtime_context_valid = false;
  state_.failed_locked = true;
  state_.recovery_required = true;
  state_.detail = "FAILED_LOCKED";
  return decision(
    true, false, FloorTransitionDecisionCode::kOk,
    "floor transition aborted; invalid runtime context and recovery lock retained");
}

bool FloorTransitionContext::candidate_allowed(
  const bool explicit_trigger,
  const bool corrections_paused) const
{
  if (state_.failed_locked) {
    return false;
  }
  if (!state_.transition_active) {
    return true;
  }
  return explicit_trigger && !corrections_paused;
}

FloorTransitionContextSnapshot FloorTransitionContext::snapshot() const
{
  return state_;
}

FloorTransitionContextDecision FloorTransitionContext::decision(
  const bool accepted,
  const bool idempotent,
  const FloorTransitionDecisionCode code,
  const std::string & message) const
{
  return {accepted, idempotent, code, message, state_};
}

const char * to_string(const FloorTransitionDecisionCode code) noexcept
{
  switch (code) {
    case FloorTransitionDecisionCode::kOk: return "OK";
    case FloorTransitionDecisionCode::kInvalidRequest: return "INVALID_REQUEST";
    case FloorTransitionDecisionCode::kPauseUnproven: return "PAUSE_UNPROVEN";
    case FloorTransitionDecisionCode::kConflict: return "CONFLICT";
    case FloorTransitionDecisionCode::kIdentityMismatch: return "IDENTITY_MISMATCH";
    case FloorTransitionDecisionCode::kExplicitRelocalizationUnproven:
      return "EXPLICIT_RELOCALIZATION_UNPROVEN";
    case FloorTransitionDecisionCode::kMapOdomUnsettled: return "MAP_ODOM_UNSETTLED";
    case FloorTransitionDecisionCode::kFailedLocked: return "FAILED_LOCKED";
  }
  return "UNKNOWN";
}

}  // namespace robot_localization_bridge
