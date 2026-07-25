#include "robot_mode_manager/mode_lease_arbiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace robot_mode_manager
{

ModeLeaseArbiter::ModeLeaseArbiter(
  const double min_lease_duration_sec,
  const double max_lease_duration_sec,
  std::string recovery_owner)
: min_lease_duration_sec_(min_lease_duration_sec),
  max_lease_duration_sec_(max_lease_duration_sec),
  recovery_owner_(std::move(recovery_owner))
{
  if (
    !std::isfinite(min_lease_duration_sec_) ||
    !std::isfinite(max_lease_duration_sec_) ||
    min_lease_duration_sec_ <= 0.0 ||
    max_lease_duration_sec_ < min_lease_duration_sec_ ||
    recovery_owner_.empty())
  {
    throw std::invalid_argument("invalid mode lease arbiter configuration");
  }
}

ModeDecision ModeLeaseArbiter::apply(const ModeCommand & command, const double now_sec)
{
  (void)expire(now_sec);

  const auto reject =
    [this, now_sec](const ModeDecisionCode code, const std::string & message)
    {
      ModeDecision decision;
      decision.code = code;
      decision.message = message;
      decision.state = snapshot(now_sec);
      return decision;
    };

  if (!std::isfinite(now_sec)) {
    return reject(ModeDecisionCode::kInvalidRequest, "invalid steady-clock timestamp");
  }

  if (command.operation == ModeOperation::kRelease) {
    if (command.owner.empty() || command.mission_id.empty() || command.lease_id.empty()) {
      return reject(ModeDecisionCode::kInvalidRequest, "release requires owner, mission, and lease");
    }
    if (is_retired(command.lease_id)) {
      return reject(ModeDecisionCode::kStaleLease, "lease has already retired");
    }
    if (
      !state_.lease_active ||
      state_.owner != command.owner ||
      state_.mission_id != command.mission_id ||
      state_.lease_id != command.lease_id)
    {
      return reject(ModeDecisionCode::kNotOwner, "only the exact active lease can release");
    }

    retire_active_lease();
    enter_normal("lease_released");
    ModeDecision decision;
    decision.accepted = true;
    decision.changed = true;
    decision.code = ModeDecisionCode::kOk;
    decision.message = "mode lease released";
    decision.state = snapshot(now_sec);
    return decision;
  }

  if (command.operation != ModeOperation::kSet) {
    return reject(ModeDecisionCode::kInvalidRequest, "unsupported mode operation");
  }

  OperatingMode requested_mode{OperatingMode::kNormal};
  if (!parse_mode(command.mode, requested_mode)) {
    return reject(ModeDecisionCode::kUnsupportedMode, "unsupported operating mode");
  }

  if (requested_mode == OperatingMode::kNormal) {
    if (
      !command.owner.empty() || !command.mission_id.empty() || !command.lease_id.empty() ||
      command.lease_duration_sec != 0.0)
    {
      return reject(ModeDecisionCode::kInvalidRequest, "NORMAL set must be anonymous");
    }
    if (state_.lease_active) {
      return reject(
        ModeDecisionCode::kLeaseConflict,
        "active lease must use exact release before NORMAL");
    }
    ModeDecision decision;
    decision.accepted = true;
    decision.code = ModeDecisionCode::kOk;
    decision.message = "already NORMAL";
    decision.state = snapshot(now_sec);
    return decision;
  }

  if (
    command.owner.empty() ||
    command.mission_id.empty() ||
    command.lease_id.empty() ||
    !std::isfinite(command.lease_duration_sec) ||
    command.lease_duration_sec < min_lease_duration_sec_ ||
    command.lease_duration_sec > max_lease_duration_sec_)
  {
    return reject(ModeDecisionCode::kInvalidRequest, "invalid non-NORMAL lease request");
  }
  if (is_retired(command.lease_id)) {
    return reject(ModeDecisionCode::kStaleLease, "lease has already retired");
  }

  const bool exact_active_lease =
    state_.lease_active &&
    state_.owner == command.owner &&
    state_.mission_id == command.mission_id &&
    state_.lease_id == command.lease_id;
  if (exact_active_lease) {
    const bool mode_changed = state_.mode != requested_mode;
    state_.mode = requested_mode;
    lease_expires_at_sec_ = now_sec + command.lease_duration_sec;
    if (mode_changed) {
      state_.generation += 1U;
      state_.transition_reason = "lease_mode_changed";
    }

    ModeDecision decision;
    decision.accepted = true;
    decision.changed = mode_changed;
    decision.code = ModeDecisionCode::kOk;
    decision.message = mode_changed ? "mode changed by active lease" : "mode lease renewed";
    decision.state = snapshot(now_sec);
    return decision;
  }

  if (state_.lease_active) {
    if (requested_mode != OperatingMode::kRecovery || command.owner != recovery_owner_) {
      return reject(ModeDecisionCode::kLeaseConflict, "another lease owns the operating mode");
    }
    retire_active_lease();
    state_.mode = requested_mode;
    state_.owner = command.owner;
    state_.mission_id = command.mission_id;
    state_.lease_id = command.lease_id;
    state_.lease_active = true;
    state_.generation += 1U;
    state_.transition_reason = "recovery_preempted";
    lease_expires_at_sec_ = now_sec + command.lease_duration_sec;

    ModeDecision decision;
    decision.accepted = true;
    decision.changed = true;
    decision.code = ModeDecisionCode::kOk;
    decision.message = "recovery lease preempted active mode";
    decision.state = snapshot(now_sec);
    return decision;
  }

  state_.mode = requested_mode;
  state_.owner = command.owner;
  state_.mission_id = command.mission_id;
  state_.lease_id = command.lease_id;
  state_.lease_active = true;
  state_.generation += 1U;
  state_.transition_reason = "lease_acquired";
  lease_expires_at_sec_ = now_sec + command.lease_duration_sec;

  ModeDecision decision;
  decision.accepted = true;
  decision.changed = true;
  decision.code = ModeDecisionCode::kOk;
  decision.message = "mode lease acquired";
  decision.state = snapshot(now_sec);
  return decision;
}

std::optional<ModeSnapshot> ModeLeaseArbiter::expire(const double now_sec)
{
  if (!state_.lease_active || !std::isfinite(now_sec) || now_sec < lease_expires_at_sec_) {
    return std::nullopt;
  }
  retire_active_lease();
  enter_normal("lease_expired");
  return snapshot(now_sec);
}

ModeSnapshot ModeLeaseArbiter::snapshot(const double now_sec) const
{
  auto current = state_;
  current.lease_remaining_sec = current.lease_active ?
    std::max(0.0, lease_expires_at_sec_ - now_sec) : 0.0;
  return current;
}

bool ModeLeaseArbiter::is_retired(const std::string & lease_id) const
{
  return retired_lease_ids_.count(lease_id) != 0U;
}

void ModeLeaseArbiter::retire_active_lease()
{
  if (!state_.lease_active || state_.lease_id.empty()) {
    return;
  }
  if (!is_retired(state_.lease_id)) {
    retired_lease_ids_.insert(state_.lease_id);
  }
}

void ModeLeaseArbiter::enter_normal(const std::string & reason)
{
  state_.mode = OperatingMode::kNormal;
  state_.owner.clear();
  state_.mission_id.clear();
  state_.lease_id.clear();
  state_.lease_remaining_sec = 0.0;
  state_.lease_active = false;
  state_.generation += 1U;
  state_.transition_reason = reason;
  lease_expires_at_sec_ = 0.0;
}

std::string to_string(const OperatingMode mode)
{
  switch (mode) {
    case OperatingMode::kNormal:
      return "NORMAL";
    case OperatingMode::kRamp:
      return "RAMP";
    case OperatingMode::kElevatorWait:
      return "ELEVATOR_WAIT";
    case OperatingMode::kElevatorRide:
      return "ELEVATOR_RIDE";
    case OperatingMode::kDoorway:
      return "DOORWAY";
    case OperatingMode::kRecovery:
      return "RECOVERY";
  }
  return "NORMAL";
}

bool parse_mode(const std::string & value, OperatingMode & mode)
{
  if (value == "NORMAL") {
    mode = OperatingMode::kNormal;
  } else if (value == "RAMP") {
    mode = OperatingMode::kRamp;
  } else if (value == "ELEVATOR_WAIT") {
    mode = OperatingMode::kElevatorWait;
  } else if (value == "ELEVATOR_RIDE") {
    mode = OperatingMode::kElevatorRide;
  } else if (value == "DOORWAY") {
    mode = OperatingMode::kDoorway;
  } else if (value == "RECOVERY") {
    mode = OperatingMode::kRecovery;
  } else {
    return false;
  }
  return true;
}

}  // namespace robot_mode_manager
