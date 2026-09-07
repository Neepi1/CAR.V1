#include "robot_safety/motion_interlock_arbiter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace robot_safety
{

MotionInterlockArbiter::MotionInterlockArbiter(
  const double min_execution_lease_sec,
  const double max_execution_lease_sec,
  std::string recovery_owner)
: min_execution_lease_sec_(min_execution_lease_sec),
  max_execution_lease_sec_(max_execution_lease_sec),
  recovery_owner_(std::move(recovery_owner))
{
  if (
    !std::isfinite(min_execution_lease_sec_) ||
    !std::isfinite(max_execution_lease_sec_) ||
    min_execution_lease_sec_ <= 0.0 ||
    max_execution_lease_sec_ < min_execution_lease_sec_ ||
    recovery_owner_.empty())
  {
    throw std::invalid_argument("invalid motion interlock configuration");
  }
}

InterlockDecision MotionInterlockArbiter::apply_hold(
  const HoldCommand & command, const double now_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return apply_hold_locked(command, now_sec);
}

InterlockDecision MotionInterlockArbiter::apply_hold_locked(
  const HoldCommand & command, const double now_sec)
{
  (void)expire_locked(now_sec);
  const auto key =
    hold_command_key(command.owner, command.transaction_id);
  const auto accepted_command = accepted_hold_commands_.find(key);
  const auto applied_sequence =
    accepted_command == accepted_hold_commands_.cend() ?
    0U : accepted_command->second.sequence;
  const auto reject =
    [this, now_sec, applied_sequence](
    const InterlockDecisionCode code, const std::string & message)
    {
      InterlockDecision decision;
      decision.code = code;
      decision.message = message;
      decision.applied_sequence = applied_sequence;
      decision.state = snapshot_locked(now_sec);
      return decision;
    };

  if (
    !std::isfinite(now_sec) ||
    command.owner.empty() ||
    command.transaction_id.empty())
  {
    return reject(InterlockDecisionCode::kInvalidRequest, "hold requires owner and transaction");
  }
  if (command.command_sequence > 0U) {
    // Once a client has announced sequencing for a key, even a malformed
    // sequenced command fences out a delayed legacy command for that key.
    sequenced_hold_keys_.insert(key);
  } else if (sequenced_hold_keys_.count(key) != 0U) {
    return reject(
      InterlockDecisionCode::kStaleCommand,
      "legacy hold command rejected after sequencing was observed");
  }
  if (accepted_command != accepted_hold_commands_.cend()) {
    const auto & previous = accepted_command->second;
    if (command.command_sequence < previous.sequence) {
      return reject(
        InterlockDecisionCode::kStaleCommand,
        "hold command sequence is older than the accepted sequence");
    }
    if (command.command_sequence == previous.sequence &&
      command.command_sequence > 0U)
    {
      if (
        command.operation == previous.operation &&
        command.reason == previous.reason)
      {
        InterlockDecision decision;
        decision.accepted = true;
        decision.code = InterlockDecisionCode::kOk;
        decision.message = "sequenced hold command already applied";
        decision.applied_sequence = previous.sequence;
        decision.state = snapshot_locked(now_sec);
        return decision;
      }
      return reject(
        InterlockDecisionCode::kStaleCommand,
        "hold command sequence was reused for different content");
    }
  }
  if (
    command.operation != HoldOperation::kAcquire &&
    command.operation != HoldOperation::kRelease)
  {
    return reject(InterlockDecisionCode::kInvalidRequest, "unsupported hold operation");
  }

  const auto exact = std::find_if(
    holds_.begin(), holds_.end(),
    [&command](const HoldRecord & hold) {
      return hold.owner == command.owner && hold.transaction_id == command.transaction_id;
    });

  if (command.operation == HoldOperation::kAcquire) {
    if (command.reason.empty()) {
      return reject(InterlockDecisionCode::kInvalidRequest, "hold acquisition requires a reason");
    }
    if (exact != holds_.end()) {
      const bool changed = exact->reason != command.reason;
      if (changed) {
        exact->reason = command.reason;
        generation_ += 1U;
        transition_reason_ = "motion_hold_updated";
      }
      InterlockDecision decision;
      decision.accepted = true;
      decision.changed = changed;
      decision.code = InterlockDecisionCode::kOk;
      decision.message = changed ? "motion hold updated" : "motion hold already active";
      decision.applied_sequence = command.command_sequence;
      decision.state = snapshot_locked(now_sec);
      if (command.command_sequence > 0U) {
        accepted_hold_commands_[key] = {
          command.command_sequence, command.operation, command.reason};
      }
      return decision;
    }

    holds_.push_back(HoldRecord{command.owner, command.transaction_id, command.reason});
    generation_ += 1U;
    transition_reason_ = "motion_hold_acquired";
    InterlockDecision decision;
    decision.accepted = true;
    decision.changed = true;
    decision.code = InterlockDecisionCode::kOk;
    decision.message = "motion hold acquired";
    decision.applied_sequence = command.command_sequence;
    decision.state = snapshot_locked(now_sec);
    if (command.command_sequence > 0U) {
      accepted_hold_commands_[key] = {
        command.command_sequence, command.operation, command.reason};
    }
    return decision;
  }

  if (exact == holds_.end()) {
    if (command.command_sequence > 0U) {
      if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
        return reject(
          InterlockDecisionCode::kConflict,
          "motion interlock generation is exhausted");
      }
      accepted_hold_commands_[key] = {
        command.command_sequence, command.operation, command.reason};
      generation_ += 1U;
      transition_reason_ = "motion_hold_release_fenced";
      InterlockDecision decision;
      decision.accepted = true;
      decision.changed = true;
      decision.code = InterlockDecisionCode::kOk;
      decision.message =
        "motion hold was already absent and is fenced from a delayed acquire";
      decision.applied_sequence = command.command_sequence;
      decision.state = snapshot_locked(now_sec);
      return decision;
    }
    return reject(
      InterlockDecisionCode::kNotOwner,
      "only the exact owner and transaction can release a hold");
  }
  holds_.erase(exact);
  generation_ += 1U;
  transition_reason_ = "motion_hold_released";
  InterlockDecision decision;
  decision.accepted = true;
  decision.changed = true;
  decision.code = InterlockDecisionCode::kOk;
  decision.message = "motion hold released";
  decision.applied_sequence = command.command_sequence;
  decision.state = snapshot_locked(now_sec);
  if (command.command_sequence > 0U) {
    accepted_hold_commands_[key] = {
      command.command_sequence, command.operation, command.reason};
  }
  return decision;
}

InterlockDecision MotionInterlockArbiter::release_hold_if_execution_idle(
  const ConditionalHoldReleaseCommand & command,
  const double now_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return release_hold_if_execution_idle_locked(command, now_sec);
}

InterlockDecision MotionInterlockArbiter::release_hold_if_execution_idle_locked(
  const ConditionalHoldReleaseCommand & command,
  const double now_sec)
{
  // Expiration is part of the same critical section. If expiration changes
  // the state, its generation change fences a caller holding older evidence.
  (void)expire_locked(now_sec);
  const auto key =
    hold_command_key(command.owner, command.transaction_id);
  const auto accepted_command = accepted_hold_commands_.find(key);
  const auto applied_sequence =
    accepted_command == accepted_hold_commands_.cend() ?
    0U : accepted_command->second.sequence;
  const auto reject =
    [this, now_sec, applied_sequence](
    const InterlockDecisionCode code, const std::string & message)
    {
      InterlockDecision decision;
      decision.code = code;
      decision.message = message;
      decision.applied_sequence = applied_sequence;
      decision.state = snapshot_locked(now_sec);
      return decision;
    };

  if (
    !std::isfinite(now_sec) ||
    command.owner.empty() ||
    command.transaction_id.empty() ||
    command.reason.empty() ||
    command.command_sequence == 0U)
  {
    return reject(
      InterlockDecisionCode::kInvalidRequest,
      "conditional hold release requires owner, transaction, reason, and "
      "a non-zero command sequence");
  }

  // Announcing a sequenced recovery command permanently fences legacy
  // commands for this exact hold key, including when this attempt is stale.
  sequenced_hold_keys_.insert(key);
  if (
    accepted_command != accepted_hold_commands_.cend() &&
    command.command_sequence <= accepted_command->second.sequence)
  {
    return reject(
      InterlockDecisionCode::kStaleCommand,
      "conditional hold release sequence is not newer than the accepted sequence");
  }
  if (command.expected_generation != generation_) {
    return reject(
      InterlockDecisionCode::kGenerationMismatch,
      "motion interlock generation changed after recovery evidence was observed");
  }
  if (execution_session_engaged_ || execution_lease_active_) {
    return reject(
      InterlockDecisionCode::kExecutionActive,
      "execution session or lease is active");
  }

  const auto exact = std::find_if(
    holds_.begin(), holds_.end(),
    [&command](const HoldRecord & hold) {
      return hold.owner == command.owner &&
             hold.transaction_id == command.transaction_id;
    });
  if (exact == holds_.end()) {
    return reject(
      InterlockDecisionCode::kNotOwner,
      "exact owner and transaction hold is not present");
  }
  if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
    return reject(
      InterlockDecisionCode::kConflict,
      "motion interlock generation is exhausted");
  }

  holds_.erase(exact);
  accepted_hold_commands_[key] = {
    command.command_sequence, HoldOperation::kRelease, command.reason};
  generation_ += 1U;
  transition_reason_ = "motion_hold_released_if_execution_idle";

  InterlockDecision decision;
  decision.accepted = true;
  decision.changed = true;
  decision.code = InterlockDecisionCode::kOk;
  decision.message =
    "motion hold released atomically while execution remained idle";
  decision.applied_sequence = command.command_sequence;
  decision.state = snapshot_locked(now_sec);
  return decision;
}

std::string MotionInterlockArbiter::hold_command_key(
  const std::string & owner,
  const std::string & transaction_id)
{
  return owner + '\x1f' + transaction_id;
}

InterlockDecision MotionInterlockArbiter::apply_execution(
  const ExecutionCommand & command, const double now_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return apply_execution_locked(command, now_sec);
}

InterlockDecision MotionInterlockArbiter::apply_execution_locked(
  const ExecutionCommand & command, const double now_sec)
{
  (void)expire_locked(now_sec);
  const auto reject =
    [this, now_sec](const InterlockDecisionCode code, const std::string & message)
    {
      InterlockDecision decision;
      decision.code = code;
      decision.message = message;
      decision.state = snapshot_locked(now_sec);
      return decision;
    };

  if (
    !std::isfinite(now_sec) ||
    command.owner.empty() ||
    command.mission_id.empty() ||
    command.transaction_id.empty() ||
    command.lease_id.empty())
  {
    return reject(
      InterlockDecisionCode::kInvalidRequest,
      "execution lease requires owner, mission, transaction, and lease");
  }
  if (
    command.operation != ExecutionOperation::kSet &&
    command.operation != ExecutionOperation::kRelease)
  {
    return reject(InterlockDecisionCode::kInvalidRequest, "unsupported execution operation");
  }

  if (command.operation == ExecutionOperation::kRelease) {
    const bool same_active_id =
      execution_session_engaged_ &&
      execution_lease_id_ == command.lease_id;
    if (same_active_id && !execution_tuple_matches(command)) {
      return reject(
        InterlockDecisionCode::kNotOwner,
        "only the exact execution tuple can close the current session");
    }
    const bool newly_retired =
      retired_execution_lease_ids_.insert(command.lease_id).second;
    const bool exact_session =
      execution_session_engaged_ && execution_tuple_matches(command);
    if (exact_session) {
      execution_session_engaged_ = false;
      execution_lease_active_ = false;
      execution_owner_.clear();
      execution_mission_id_.clear();
      execution_transaction_id_.clear();
      execution_lease_id_.clear();
      execution_expires_at_sec_ = 0.0;
    }
    if (newly_retired || exact_session) {
      generation_ += 1U;
      transition_reason_ = exact_session ?
        "execution_session_released" :
        "execution_lease_release_fenced";
    }

    InterlockDecision decision;
    decision.accepted = true;
    decision.changed = newly_retired || exact_session;
    decision.code = InterlockDecisionCode::kOk;
    decision.message = exact_session ?
      "execution session released" :
      "execution lease was already absent and is now fenced from late set";
    decision.state = snapshot_locked(now_sec);
    return decision;
  }
  if (lease_is_retired(command.lease_id)) {
    return reject(InterlockDecisionCode::kStaleLease, "execution lease has retired");
  }

  if (
    !std::isfinite(command.lease_duration_sec) ||
    command.lease_duration_sec < min_execution_lease_sec_ ||
    command.lease_duration_sec > max_execution_lease_sec_ ||
    (command.recovery && command.owner != recovery_owner_))
  {
    return reject(InterlockDecisionCode::kInvalidRequest, "invalid execution lease request");
  }

  if (execution_session_engaged_) {
    if (execution_lease_active_ && execution_tuple_matches(command)) {
      execution_expires_at_sec_ = now_sec + command.lease_duration_sec;
      InterlockDecision decision;
      decision.accepted = true;
      decision.code = InterlockDecisionCode::kOk;
      decision.message = "execution lease renewed";
      decision.state = snapshot_locked(now_sec);
      return decision;
    }
    if (execution_lease_active_ || !command.recovery || command.owner != recovery_owner_) {
      return reject(
        InterlockDecisionCode::kConflict,
        "execution session is owned or failure-locked");
    }
  }

  execution_session_engaged_ = true;
  execution_lease_active_ = true;
  execution_owner_ = command.owner;
  execution_mission_id_ = command.mission_id;
  execution_transaction_id_ = command.transaction_id;
  execution_lease_id_ = command.lease_id;
  execution_expires_at_sec_ = now_sec + command.lease_duration_sec;
  generation_ += 1U;
  transition_reason_ = command.recovery ?
    "execution_session_recovered" : "execution_session_acquired";

  InterlockDecision decision;
  decision.accepted = true;
  decision.changed = true;
  decision.code = InterlockDecisionCode::kOk;
  decision.message = command.recovery ?
    "execution session recovered" : "execution session acquired";
  decision.state = snapshot_locked(now_sec);
  return decision;
}

std::optional<MotionInterlockSnapshot> MotionInterlockArbiter::expire(const double now_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return expire_locked(now_sec);
}

std::optional<MotionInterlockSnapshot> MotionInterlockArbiter::expire_locked(
  const double now_sec)
{
  if (
    !execution_lease_active_ ||
    !std::isfinite(now_sec) ||
    now_sec < execution_expires_at_sec_)
  {
    return std::nullopt;
  }
  retire_execution_lease();
  execution_lease_active_ = false;
  execution_expires_at_sec_ = 0.0;
  generation_ += 1U;
  transition_reason_ = "execution_lease_expired";
  return snapshot_locked(now_sec);
}

MotionInterlockSnapshot MotionInterlockArbiter::snapshot(const double now_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_locked(now_sec);
}

MotionInterlockSnapshot MotionInterlockArbiter::snapshot_locked(
  const double now_sec) const
{
  MotionInterlockSnapshot state;
  state.generation = generation_;
  state.motion_blocked = !motion_permitted_locked(now_sec);
  state.hold_keys.reserve(holds_.size());
  for (const auto & hold : holds_) {
    state.hold_keys.push_back(hold.owner + ":" + hold.transaction_id);
  }
  state.execution_session_engaged = execution_session_engaged_;
  state.execution_lease_active =
    execution_lease_active_ && std::isfinite(now_sec) && now_sec < execution_expires_at_sec_;
  state.execution_owner = execution_owner_;
  state.execution_mission_id = execution_mission_id_;
  state.execution_transaction_id = execution_transaction_id_;
  state.execution_lease_id = execution_lease_id_;
  state.execution_lease_remaining_sec = state.execution_lease_active ?
    std::max(0.0, execution_expires_at_sec_ - now_sec) : 0.0;
  state.transition_reason = transition_reason_;
  return state;
}

bool MotionInterlockArbiter::motion_permitted(const double now_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return motion_permitted_locked(now_sec);
}

bool MotionInterlockArbiter::motion_permitted_locked(const double now_sec) const
{
  if (!holds_.empty()) {
    return false;
  }
  if (!execution_session_engaged_) {
    return true;
  }
  return execution_lease_active_ &&
         std::isfinite(now_sec) &&
         now_sec < execution_expires_at_sec_;
}

bool MotionInterlockArbiter::execution_tuple_matches(
  const ExecutionCommand & command) const
{
  return execution_owner_ == command.owner &&
         execution_mission_id_ == command.mission_id &&
         execution_transaction_id_ == command.transaction_id &&
         execution_lease_id_ == command.lease_id;
}

bool MotionInterlockArbiter::lease_is_retired(const std::string & lease_id) const
{
  return retired_execution_lease_ids_.count(lease_id) != 0U;
}

void MotionInterlockArbiter::retire_execution_lease()
{
  if (execution_lease_id_.empty() || lease_is_retired(execution_lease_id_)) {
    return;
  }
  retired_execution_lease_ids_.insert(execution_lease_id_);
}

}  // namespace robot_safety
