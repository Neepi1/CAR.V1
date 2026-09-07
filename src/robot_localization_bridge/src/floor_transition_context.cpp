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

FloorTransitionContextDecision FloorTransitionContext::seed_active_source(
  const FloorTransitionIdentity & identity)
{
  if (!valid_identity(identity)) {
    return decision(
      false, false, FloorTransitionDecisionCode::kInvalidRequest,
      "active source seed requires an exact verified identity");
  }
  if (state_.transition_active || state_.failed_locked) {
    return decision(
      false, false, FloorTransitionDecisionCode::kConflict,
      "active source cannot be seeded during a transition or recovery lock");
  }
  if (valid_identity(state_.active)) {
    if (same_asset(state_.active, identity)) {
      return decision(
        true, true, FloorTransitionDecisionCode::kOk,
        "active source identity already seeded");
    }
    return decision(
      false, false, FloorTransitionDecisionCode::kConflict,
      "active source identity is already bound to another asset");
  }
  state_.active = identity;
  state_.runtime_context_valid = true;
  state_.recovery_required = false;
  state_.failed_locked = false;
  state_.detail = "ACTIVE_SOURCE_SEEDED";
  return decision(
    true, false, FloorTransitionDecisionCode::kOk,
    "verified live source identity seeded");
}

FloorTransitionContextDecision FloorTransitionContext::begin(
  const FloorTransitionIdentity & identity,
  const FloorTransitionIdentity & source,
  const bool floor_pause_owned,
  const std::uint64_t current_explicit_relocalization_sequence,
  const std::uint64_t command_sequence)
{
  if (!valid_identity(identity) || !valid_identity(source)) {
    return decision(
      false, false, FloorTransitionDecisionCode::kInvalidRequest,
      "BEGIN requires exact valid target and source identities");
  }
  FloorTransitionContextDecision sequence_rejection;
  if (!command_sequence_valid(
      identity, command_sequence, sequence_rejection))
  {
    return sequence_rejection;
  }
  if (state_.failed_locked) {
    return decision(
      false, false, FloorTransitionDecisionCode::kFailedLocked,
      "floor transition is failure-locked and requires an explicit recovery protocol");
  }
  if (!valid_identity(state_.active)) {
    return decision(
      false, false, FloorTransitionDecisionCode::kPreMutationUnproven,
      "BEGIN requires a bridge-verified active source identity",
      command_sequence);
  }
  if (!same_asset(state_.active, source)) {
    return decision(
      false, false, FloorTransitionDecisionCode::kIdentityMismatch,
      "BEGIN source identity does not match the bridge-verified active source",
      command_sequence);
  }
  if (state_.transition_active) {
    if (same_identity(state_.pending, identity)) {
      return decision(
      true, true, FloorTransitionDecisionCode::kOk,
      "idempotent BEGIN replay", command_sequence);
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
    "source runtime context invalidated under floor-manager correction pause",
    command_sequence);
}

FloorTransitionContextDecision FloorTransitionContext::commit(
  const FloorTransitionIdentity & identity,
  const FloorTransitionCommitEvidence & evidence,
  const std::uint64_t command_sequence)
{
  if (!valid_identity(identity)) {
    return decision(
      false, false, FloorTransitionDecisionCode::kInvalidRequest,
      "COMMIT contains an invalid floor identity");
  }
  FloorTransitionContextDecision sequence_rejection;
  if (!command_sequence_valid(
      identity, command_sequence, sequence_rejection))
  {
    return sequence_rejection;
  }
  if (
    !state_.transition_active && state_.runtime_context_valid &&
    same_asset(state_.active, identity))
  {
    return decision(
      true, true, FloorTransitionDecisionCode::kOk,
      "idempotent COMMIT replay", command_sequence);
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
    "target floor runtime context committed", command_sequence);
}

FloorTransitionContextDecision FloorTransitionContext::abort(
  const FloorTransitionIdentity & identity,
  const std::uint64_t command_sequence)
{
  if (!valid_identity(identity)) {
    return decision(
      false, false, FloorTransitionDecisionCode::kInvalidRequest,
      "ABORT contains an invalid floor identity");
  }
  FloorTransitionContextDecision sequence_rejection;
  if (!command_sequence_valid(
      identity, command_sequence, sequence_rejection))
  {
    return sequence_rejection;
  }
  if (state_.failed_locked && same_identity(state_.pending, identity)) {
    return decision(
      true, true, FloorTransitionDecisionCode::kOk,
      "idempotent ABORT replay; failure lock retained", command_sequence);
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
    "floor transition aborted; invalid runtime context and recovery lock retained",
    command_sequence);
}

FloorTransitionContextDecision FloorTransitionContext::abort_pre_mutation(
  const FloorTransitionIdentity & identity,
  const FloorTransitionPreMutationAbortEvidence & evidence,
  const std::uint64_t command_sequence)
{
  if (!valid_identity(identity) || !valid_identity(evidence.source)) {
    return decision(
      false, false, FloorTransitionDecisionCode::kInvalidRequest,
      "pre-mutation ABORT requires exact valid target and source identities");
  }
  FloorTransitionContextDecision sequence_rejection;
  if (!command_sequence_valid(
      identity, command_sequence, sequence_rejection))
  {
    return sequence_rejection;
  }
  if (state_.failed_locked) {
    return decision(
      false, false, FloorTransitionDecisionCode::kFailedLocked,
      "pre-mutation ABORT cannot clear an existing recovery lock",
      command_sequence);
  }
  if (!evidence.source_assets_unchanged) {
    return decision(
      false, false, FloorTransitionDecisionCode::kPreMutationUnproven,
      "source localizer/map->odom state changed or was not proven",
      command_sequence);
  }
  if (state_.transition_active) {
    if (!same_identity(state_.pending, identity)) {
      return decision(
        false, false, FloorTransitionDecisionCode::kIdentityMismatch,
        "pre-mutation ABORT does not match the active pending transaction",
        command_sequence);
    }
    if (!same_asset(state_.active, evidence.source)) {
      return decision(
        false, false, FloorTransitionDecisionCode::kIdentityMismatch,
        "pre-mutation ABORT source identity does not match the active source",
        command_sequence);
    }
    state_.pending = FloorTransitionIdentity{};
    state_.transition_active = false;
    state_.runtime_context_valid = true;
    state_.failed_locked = false;
    state_.recovery_required = false;
    state_.detail = "PREMUTATION_ABORTED";
    return decision(
      true, false, FloorTransitionDecisionCode::kOk,
      "pending BEGIN fenced and exact source runtime context restored",
      command_sequence);
  }
  if (
    state_.runtime_context_valid &&
    same_asset(state_.active, evidence.source))
  {
    return decision(
      true, true, FloorTransitionDecisionCode::kOk,
      "no BEGIN was active; higher command sequence fenced delayed BEGIN",
      command_sequence);
  }
  return decision(
    false, false, FloorTransitionDecisionCode::kPreMutationUnproven,
    "no exact valid source context was available for pre-mutation ABORT",
    command_sequence);
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
  const std::string & message,
  const std::uint64_t applied_sequence) const
{
  return {accepted, idempotent, code, message, applied_sequence, state_};
}

bool FloorTransitionContext::command_sequence_valid(
  const FloorTransitionIdentity & identity,
  const std::uint64_t command_sequence,
  FloorTransitionContextDecision & rejection)
{
  if (command_sequence == 0U) {
    rejection = decision(
      false, false, FloorTransitionDecisionCode::kInvalidRequest,
      "floor-transition command_sequence must be non-zero");
    return false;
  }
  const auto previous = last_command_sequences_.find(identity.transaction_id);
  if (
    previous != last_command_sequences_.cend() &&
    command_sequence <= previous->second)
  {
    rejection = decision(
      false, false, FloorTransitionDecisionCode::kStaleCommand,
      "floor-transition command_sequence is stale", previous->second);
    return false;
  }
  // Consume every syntactically valid command before semantic validation.
  // A delayed lower-sequence BEGIN can therefore never run after a compensating
  // pre-mutation ABORT, even if DDS/service callbacks are reordered.
  last_command_sequences_[identity.transaction_id] = command_sequence;
  return true;
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
    case FloorTransitionDecisionCode::kStaleCommand: return "STALE_COMMAND";
    case FloorTransitionDecisionCode::kPreMutationUnproven:
      return "PREMUTATION_UNPROVEN";
  }
  return "UNKNOWN";
}

}  // namespace robot_localization_bridge
