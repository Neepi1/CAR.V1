#include "robot_localization_bridge/correction_pause_arbiter.hpp"

#include <algorithm>

namespace robot_localization_bridge
{

PauseDecision CorrectionPauseArbiter::apply(const PauseCommand & command)
{
  const auto reject =
    [this](
      const PauseDecisionCode code,
      const std::string & message,
      const std::uint64_t applied_sequence = 0U)
    {
      PauseDecision decision;
      decision.code = code;
      decision.message = message;
      decision.applied_sequence = applied_sequence;
      decision.state = snapshot();
      return decision;
    };

  if (command.owner.empty() || command.transaction_id.empty()) {
    return reject(
      PauseDecisionCode::kInvalidRequest,
      "correction pause requires owner and transaction");
  }
  if (
    command.operation != PauseOperation::kAcquire &&
    command.operation != PauseOperation::kRelease)
  {
    return reject(PauseDecisionCode::kInvalidRequest, "unsupported pause operation");
  }
  if (command.command_sequence == 0U) {
    return reject(
      PauseDecisionCode::kInvalidRequest,
      "correction pause command_sequence must be non-zero");
  }

  const auto key = command.owner + ":" + command.transaction_id;
  const auto previous_sequence = last_command_sequences_.find(key);
  if (
    previous_sequence != last_command_sequences_.cend() &&
    command.command_sequence <= previous_sequence->second)
  {
    return reject(
      PauseDecisionCode::kStaleCommand,
      "correction pause command_sequence is stale",
      previous_sequence->second);
  }
  // Consume every syntactically valid sequence, including a semantic
  // rejection. A delayed older request can therefore never become valid after
  // the conflicting lease changes.
  last_command_sequences_[key] = command.command_sequence;

  const auto exact = std::find_if(
    records_.begin(), records_.end(),
    [&command](const Record & record) {
      return record.owner == command.owner &&
             record.transaction_id == command.transaction_id;
    });

  if (command.operation == PauseOperation::kAcquire) {
    if (command.reason.empty()) {
      return reject(
        PauseDecisionCode::kInvalidRequest,
        "correction pause acquisition requires reason",
        command.command_sequence);
    }
    if (exact != records_.end()) {
      const bool changed = exact->reason != command.reason;
      if (changed) {
        exact->reason = command.reason;
        generation_ += 1U;
        transition_reason_ = "correction_pause_updated";
      }
      PauseDecision decision;
      decision.accepted = true;
      decision.changed = changed;
      decision.code = PauseDecisionCode::kOk;
      decision.message = changed ?
        "correction pause updated" : "correction pause already acquired";
      decision.applied_sequence = command.command_sequence;
      decision.state = snapshot();
      return decision;
    }

    records_.push_back(Record{command.owner, command.transaction_id, command.reason});
    generation_ += 1U;
    transition_reason_ = "correction_pause_acquired";
    PauseDecision decision;
    decision.accepted = true;
    decision.changed = true;
    decision.code = PauseDecisionCode::kOk;
    decision.message = "correction pause acquired";
    decision.applied_sequence = command.command_sequence;
    decision.state = snapshot();
    return decision;
  }

  if (exact == records_.end()) {
    generation_ += 1U;
    transition_reason_ = "correction_pause_release_fenced";
    PauseDecision decision;
    decision.accepted = true;
    decision.changed = true;
    decision.code = PauseDecisionCode::kOk;
    decision.message =
      "exact owner correction pause was already released";
    decision.applied_sequence = command.command_sequence;
    decision.state = snapshot();
    return decision;
  }
  records_.erase(exact);
  generation_ += 1U;
  transition_reason_ = "correction_pause_released";
  PauseDecision decision;
  decision.accepted = true;
  decision.changed = true;
  decision.code = PauseDecisionCode::kOk;
  decision.message = "correction pause released";
  decision.applied_sequence = command.command_sequence;
  decision.state = snapshot();
  return decision;
}

CorrectionPauseSnapshot CorrectionPauseArbiter::snapshot() const
{
  CorrectionPauseSnapshot state;
  state.generation = generation_;
  state.paused = !records_.empty();
  state.lease_keys.reserve(records_.size());
  for (const auto & record : records_) {
    state.lease_keys.push_back(record.owner + ":" + record.transaction_id);
  }
  state.transition_reason = transition_reason_;
  return state;
}

}  // namespace robot_localization_bridge
