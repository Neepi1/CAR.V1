#include "robot_safety/motion_interlock_arbiter.hpp"

#include <algorithm>
#include <cmath>
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
  (void)expire(now_sec);
  const auto reject =
    [this, now_sec](const InterlockDecisionCode code, const std::string & message)
    {
      InterlockDecision decision;
      decision.code = code;
      decision.message = message;
      decision.state = snapshot(now_sec);
      return decision;
    };

  if (
    !std::isfinite(now_sec) ||
    command.owner.empty() ||
    command.transaction_id.empty())
  {
    return reject(InterlockDecisionCode::kInvalidRequest, "hold requires owner and transaction");
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
  const auto same_transaction = std::find_if(
    holds_.cbegin(), holds_.cend(),
    [&command](const HoldRecord & hold) {
      return hold.transaction_id == command.transaction_id;
    });

  if (command.operation == HoldOperation::kAcquire) {
    if (command.reason.empty()) {
      return reject(InterlockDecisionCode::kInvalidRequest, "hold acquisition requires a reason");
    }
    if (same_transaction != holds_.cend() && same_transaction->owner != command.owner) {
      return reject(
        InterlockDecisionCode::kConflict,
        "transaction is already held by another owner");
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
      decision.state = snapshot(now_sec);
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
    decision.state = snapshot(now_sec);
    return decision;
  }

  if (exact == holds_.end()) {
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
  decision.state = snapshot(now_sec);
  return decision;
}

InterlockDecision MotionInterlockArbiter::apply_execution(
  const ExecutionCommand & command, const double now_sec)
{
  (void)expire(now_sec);
  const auto reject =
    [this, now_sec](const InterlockDecisionCode code, const std::string & message)
    {
      InterlockDecision decision;
      decision.code = code;
      decision.message = message;
      decision.state = snapshot(now_sec);
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
  if (lease_is_retired(command.lease_id)) {
    return reject(InterlockDecisionCode::kStaleLease, "execution lease has retired");
  }

  if (command.operation == ExecutionOperation::kRelease) {
    if (!execution_lease_active_ || !execution_tuple_matches(command)) {
      return reject(
        InterlockDecisionCode::kNotOwner,
        "only the exact active execution lease can close the session");
    }
    retire_execution_lease();
    execution_session_engaged_ = false;
    execution_lease_active_ = false;
    execution_owner_.clear();
    execution_mission_id_.clear();
    execution_transaction_id_.clear();
    execution_lease_id_.clear();
    execution_expires_at_sec_ = 0.0;
    generation_ += 1U;
    transition_reason_ = "execution_session_released";

    InterlockDecision decision;
    decision.accepted = true;
    decision.changed = true;
    decision.code = InterlockDecisionCode::kOk;
    decision.message = "execution session released";
    decision.state = snapshot(now_sec);
    return decision;
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
      decision.state = snapshot(now_sec);
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
  decision.state = snapshot(now_sec);
  return decision;
}

std::optional<MotionInterlockSnapshot> MotionInterlockArbiter::expire(const double now_sec)
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
  return snapshot(now_sec);
}

MotionInterlockSnapshot MotionInterlockArbiter::snapshot(const double now_sec) const
{
  MotionInterlockSnapshot state;
  state.generation = generation_;
  state.motion_blocked = !motion_permitted(now_sec);
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
