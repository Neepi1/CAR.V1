#include "robot_mission_manager/mission_fsm.hpp"

#include <algorithm>
#include <cstddef>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace robot_mission_manager
{
namespace
{

bool is_blank(const std::string & value)
{
  return value.empty() ||
         std::all_of(
    value.begin(), value.end(),
    [](const unsigned char character) {return std::isspace(character) != 0;});
}

bool safe_identifier(
  const std::string & value,
  const bool allow_empty = false,
  const bool allow_colon = false,
  const std::size_t max_length = 128U)
{
  if (value.empty()) {
    return allow_empty;
  }
  if (
    value == "." ||
    value.size() > max_length ||
    value.find("..") != std::string::npos ||
    value.find('/') != std::string::npos ||
    value.find('\\') != std::string::npos)
  {
    return false;
  }
  return std::all_of(
    value.begin(), value.end(),
    [allow_colon](const unsigned char character) {
      return std::isalnum(character) != 0 ||
             character == static_cast<unsigned char>('-') ||
             character == static_cast<unsigned char>('_') ||
             character == static_cast<unsigned char>('.') ||
             (allow_colon && character == static_cast<unsigned char>(':'));
    });
}

bool canonical_sha256(const std::string & value)
{
  constexpr char prefix[] = "sha256:";
  constexpr std::size_t prefix_size = sizeof(prefix) - 1U;
  if (value.size() != prefix_size + 64U ||
    value.compare(0U, prefix_size, prefix) != 0)
  {
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

MissionTransition rejected(const std::string & message)
{
  MissionTransition transition;
  transition.message = message;
  return transition;
}

MissionTransition ignored(const std::string & message)
{
  MissionTransition transition;
  transition.ignored = true;
  transition.message = message;
  return transition;
}

}  // namespace

MissionFsm::MissionFsm(std::string instance_id)
: instance_id_(std::move(instance_id))
{
  if (!safe_identifier(instance_id_, false, false, 32U)) {
    throw std::invalid_argument(
            "mission FSM requires a path-safe unique instance_id of at most 32 characters");
  }
}

MissionTransition MissionFsm::start(const MissionRequest & request)
{
  if (state_ == MissionState::kFailureLocked) {
    return rejected("mission FSM is failure-locked");
  }
  if (state_ != MissionState::kIdle || active_effect_.has_value()) {
    return rejected("another mission or effect is already active");
  }
  if (
    is_blank(request.mission_id) ||
    is_blank(request.building_id) ||
    is_blank(request.expected_source_floor_id) ||
    is_blank(request.expected_source_map_id) ||
    is_blank(request.target_floor_id) ||
    is_blank(request.target_map_id) ||
    is_blank(request.target_pose_id))
  {
    return rejected(
      "mission, building, source floor/map, target floor/map, and target pose are required");
  }
  if (
    !safe_identifier(request.mission_id, false, false, 64U) ||
    !safe_identifier(request.building_id) ||
    !safe_identifier(request.expected_source_floor_id) ||
    !safe_identifier(request.expected_source_map_id) ||
    !safe_identifier(request.target_floor_id) ||
    !safe_identifier(request.target_map_id) ||
    !safe_identifier(request.target_pose_id, false, true) ||
    !safe_identifier(request.preferred_elevator_id, true))
  {
    return rejected("mission request contains an unsafe identifier");
  }
  if (
    request.expected_asset_epoch == 0U ||
    !canonical_sha256(request.expected_asset_digest))
  {
    return rejected(
      "mission target requires a nonzero asset epoch and canonical SHA-256 digest");
  }

  mission_ = request;
  failure_reason_.clear();
  if (request.expected_source_floor_id == request.target_floor_id) {
    if (request.expected_source_map_id != request.target_map_id) {
      mission_.reset();
      return rejected(
        "same-floor map changes require an explicit floor-switch workflow, not an elevator task");
    }
    return emit_nav_target();
  }
  return emit_elevator_task();
}

MissionTransition MissionFsm::complete_effect(
  const EffectCompletion & completion)
{
  if (!active_effect_.has_value()) {
    return ignored("no active effect");
  }
  if (completion.transaction_id != active_effect_->transaction_id) {
    return ignored("event transaction does not match the active effect");
  }

  const EffectKind completed_kind = active_effect_->kind;

  if (completed_kind == EffectKind::kHoldAndCancel) {
    if (!completion.success) {
      MissionEffect retry = *active_effect_;
      retry.transaction_id = next_transaction_id();
      retry.reason = completion.message.empty() ?
        "hold-and-cancel failed; retry required" :
        "hold-and-cancel failed; retry required: " + completion.message;
      active_effect_ = retry;
      MissionTransition transition;
      transition.accepted = true;
      transition.message = retry.reason;
      transition.effect = std::move(retry);
      return transition;
    }
    active_effect_.reset();
    MissionTransition transition;
    transition.accepted = true;
    transition.message = "hold-and-cancel acknowledged; failure lock retained";
    return transition;
  }

  if (!completion.success) {
    return lock_failure(
      completion.message.empty() ? "active effect failed" : completion.message);
  }

  if (completed_kind == EffectKind::kElevatorTask) {
    if (
      !mission_.has_value() ||
      completion.final_building_id != mission_->building_id ||
      completion.final_floor_id != mission_->target_floor_id ||
      completion.final_map_id != mission_->target_map_id ||
      completion.final_zone != "TARGET_HALL" ||
      completion.safety_hold_active ||
      completion.final_asset_epoch != mission_->expected_asset_epoch ||
      !canonical_sha256(completion.final_asset_digest) ||
      completion.final_asset_digest != mission_->expected_asset_digest ||
      completion.explicit_relocalization_sequence == 0U ||
      !completion.runtime_context_valid ||
      completion.nav_goal_active ||
      completion.residual_mode_lease ||
      completion.residual_execution_lease ||
      completion.residual_correction_pause)
    {
      return lock_failure(
        "elevator success did not prove the frozen target asset identity, relocalization, "
        "TARGET_HALL, and zero residual goals/holds/leases");
    }
    active_effect_.reset();
    return emit_nav_target();
  }

  active_effect_.reset();
  state_ = MissionState::kSucceeded;
  MissionTransition transition;
  transition.accepted = true;
  transition.message = "mission completed";
  return transition;
}

MissionTransition MissionFsm::abort(const std::string & reason)
{
  return lock_failure(reason.empty() ? "mission aborted" : reason);
}

MissionTransition MissionFsm::cancel(const std::string & reason)
{
  return lock_failure(reason.empty() ? "mission cancelled" : reason);
}

MissionState MissionFsm::state() const noexcept
{
  return state_;
}

bool MissionFsm::has_active_effect() const noexcept
{
  return active_effect_.has_value();
}

std::optional<MissionEffect> MissionFsm::active_effect() const
{
  return active_effect_;
}

const std::string & MissionFsm::failure_reason() const noexcept
{
  return failure_reason_;
}

MissionTransition MissionFsm::emit_elevator_task()
{
  MissionEffect effect;
  effect.kind = EffectKind::kElevatorTask;
  effect.transaction_id = next_transaction_id();
  effect.mission_id = mission_->mission_id;
  effect.building_id = mission_->building_id;
  effect.expected_source_floor_id = mission_->expected_source_floor_id;
  effect.expected_source_map_id = mission_->expected_source_map_id;
  effect.target_floor_id = mission_->target_floor_id;
  effect.target_map_id = mission_->target_map_id;
  effect.preferred_elevator_id = mission_->preferred_elevator_id;
  effect.expected_asset_epoch = mission_->expected_asset_epoch;
  effect.expected_asset_digest = mission_->expected_asset_digest;

  state_ = MissionState::kUsingElevator;
  active_effect_ = effect;

  MissionTransition transition;
  transition.accepted = true;
  transition.message = "elevator task emitted";
  transition.effect = std::move(effect);
  return transition;
}

MissionTransition MissionFsm::emit_nav_target()
{
  MissionEffect effect;
  effect.kind = EffectKind::kNavTarget;
  effect.transaction_id = next_transaction_id();
  effect.mission_id = mission_->mission_id;
  effect.building_id = mission_->building_id;
  effect.target_floor_id = mission_->target_floor_id;
  effect.target_map_id = mission_->target_map_id;
  effect.target_pose_id = mission_->target_pose_id;
  effect.expected_asset_epoch = mission_->expected_asset_epoch;
  effect.expected_asset_digest = mission_->expected_asset_digest;

  state_ = MissionState::kNavigatingToTarget;
  active_effect_ = effect;

  MissionTransition transition;
  transition.accepted = true;
  transition.message = "navigation target emitted";
  transition.effect = std::move(effect);
  return transition;
}

MissionTransition MissionFsm::lock_failure(const std::string & reason)
{
  if (
    state_ == MissionState::kFailureLocked &&
    active_effect_.has_value() &&
    active_effect_->kind == EffectKind::kHoldAndCancel)
  {
    return rejected("hold-and-cancel is already active");
  }
  if (state_ == MissionState::kFailureLocked) {
    return rejected("mission FSM is failure-locked");
  }

  const std::string cancelled_transaction =
    active_effect_.has_value() ? active_effect_->transaction_id : "";
  active_effect_.reset();
  state_ = MissionState::kFailureLocked;
  failure_reason_ = reason;

  MissionEffect effect;
  effect.kind = EffectKind::kHoldAndCancel;
  effect.transaction_id = next_transaction_id();
  effect.mission_id = mission_.has_value() ? mission_->mission_id : "";
  effect.building_id = mission_.has_value() ? mission_->building_id : "";
  effect.target_floor_id = mission_.has_value() ? mission_->target_floor_id : "";
  effect.target_map_id = mission_.has_value() ? mission_->target_map_id : "";
  effect.expected_asset_epoch =
    mission_.has_value() ? mission_->expected_asset_epoch : 0U;
  effect.expected_asset_digest =
    mission_.has_value() ? mission_->expected_asset_digest : "";
  effect.cancel_transaction_id = cancelled_transaction;
  effect.cleanup_id =
    instance_id_ + "-" +
    (mission_.has_value() ? mission_->mission_id : "no-mission") +
    "-cleanup";
  effect.reason = failure_reason_;
  active_effect_ = effect;

  MissionTransition transition;
  transition.accepted = true;
  transition.message = "failure locked; hold-and-cancel emitted";
  transition.effect = std::move(effect);
  return transition;
}

std::string MissionFsm::next_transaction_id()
{
  const std::string mission_context =
    mission_.has_value() ? mission_->mission_id : "no-mission";
  return instance_id_ + "-" + mission_context + "-effect-" +
         std::to_string(next_transaction_sequence_++);
}

const char * to_string(const MissionState state) noexcept
{
  switch (state) {
    case MissionState::kIdle:
      return "IDLE";
    case MissionState::kUsingElevator:
      return "USING_ELEVATOR";
    case MissionState::kNavigatingToTarget:
      return "NAVIGATING_TO_TARGET";
    case MissionState::kSucceeded:
      return "SUCCEEDED";
    case MissionState::kFailureLocked:
      return "FAILURE_LOCKED";
  }
  return "UNKNOWN";
}

const char * to_string(const EffectKind kind) noexcept
{
  switch (kind) {
    case EffectKind::kElevatorTask:
      return "ELEVATOR_TASK";
    case EffectKind::kNavTarget:
      return "NAV_TARGET";
    case EffectKind::kHoldAndCancel:
      return "HOLD_AND_CANCEL";
  }
  return "UNKNOWN";
}

}  // namespace robot_mission_manager
