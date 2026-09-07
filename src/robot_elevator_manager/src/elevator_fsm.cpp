#include "robot_elevator_manager/elevator_fsm.hpp"

#include <utility>

namespace robot_elevator_manager
{
namespace
{

ElevatorEffect make_effect(
  const ElevatorEffectKind kind,
  std::string pose_id = "",
  std::string floor_id = "",
  std::string mode = "",
  std::string detail = "")
{
  return ElevatorEffect{
    kind,
    0U,
    "",
    std::move(pose_id),
    std::move(floor_id),
    std::move(mode),
    std::move(detail),
    "",
  };
}

bool valid_route(const ElevatorRoute & route)
{
  if (
    route.source.floor_id == route.target.floor_id &&
    route.source.map_id == route.target.map_id)
  {
    return false;
  }
  const ElevatorTopology topology{
    route.elevator_id,
    route.building_id,
    {route.source, route.target},
    route.schema_version,
  };
  return validate_topology(topology).ok();
}

}  // namespace

ElevatorFsm::ElevatorFsm(const ElevatorFsmOptions options)
: options_(options)
{
}

ElevatorFsmOutput ElevatorFsm::start(
  const std::string & transaction_id,
  const ElevatorRoute & route)
{
  if (state_ == ElevatorState::kLocked) {
    return inert("fsm_locked");
  }
  if (state_ != ElevatorState::kIdle) {
    return lock("start_received_while_not_idle", true);
  }
  if (!safe_asset_id(transaction_id)) {
    return inert("invalid_transaction_id");
  }
  if (route.schema_version != 2U && route.schema_version != 3U) {
    return inert("legacy_read_only");
  }
  if (!options_.mock_ports_enabled) {
    return inert("real_elevator_ports_not_integrated");
  }
  if (!valid_route(route)) {
    return inert("invalid_elevator_route");
  }
  transaction_id_ = transaction_id;
  route_ = route;
  return transition(
    ElevatorState::kNavigatingHallCall,
    navigation_effect(PoseRole::kHallCall, true));
}

ElevatorFsmOutput ElevatorFsm::dispatch(const ElevatorEvent & event)
{
  if (state_ == ElevatorState::kLocked) {
    return inert("fsm_locked");
  }
  if (state_ == ElevatorState::kComplete) {
    return inert("fsm_complete");
  }
  if (event.transaction_id != transaction_id_) {
    return inert("transaction_id_mismatch");
  }
  if (state_ == ElevatorState::kFailureCleanup) {
    if (event.effect_sequence != pending_effect_sequence_) {
      return inert("cleanup_effect_sequence_mismatch");
    }
    if (event.kind == ElevatorEventKind::kEffectSucceeded) {
      state_ = ElevatorState::kLocked;
      pending_effect_sequence_ = 0U;
      return ElevatorFsmOutput{
        true,
        true,
        state_,
        make_effect(ElevatorEffectKind::kNone),
        "failure_cleanup_acknowledged;fsm_locked",
      };
    }
    return lock(
      event.detail.empty() ?
      "failure_cleanup_not_confirmed;retry" :
      "failure_cleanup_not_confirmed;retry:" + event.detail,
      true);
  }
  if (event.kind == ElevatorEventKind::kCancelRequested) {
    return lock(
      event.detail.empty() ? "cancel_requested" : "cancel_requested:" + event.detail,
      true);
  }
  if (event.effect_sequence != pending_effect_sequence_) {
    return lock("effect_sequence_mismatch", true);
  }
  if (event.kind == ElevatorEventKind::kEffectFailed) {
    return lock(
      event.detail.empty() ? "port_effect_failed" : "port_effect_failed:" + event.detail,
      true);
  }
  if (state_ == ElevatorState::kIdle) {
    return lock("event_received_before_start", true);
  }

  if (event.kind != ElevatorEventKind::kEffectSucceeded) {
    return lock("unexpected_event_for_pending_effect", true);
  }
  return advance_effect_success();
}

ElevatorFsmOutput ElevatorFsm::advance_effect_success()
{
  switch (state_) {
    case ElevatorState::kNavigatingHallCall:
      return transition(
        ElevatorState::kAcquiringHallHold,
        make_effect(
          ElevatorEffectKind::kAcquireSafetyHold, "", route_->source.floor_id, "",
          "settle_before_call"));
    case ElevatorState::kAcquiringHallHold:
    case ElevatorState::kAcquiringExecutionLease:
    {
      auto effect = make_effect(
          ElevatorEffectKind::kMockPressCallButton, "", route_->source.floor_id, "",
          "mock_port");
      effect.panel_side = route_->source.hall_call_panel_side;
      return transition(ElevatorState::kPressingCallButton, std::move(effect));
    }
    case ElevatorState::kPressingCallButton:
      return transition(
        ElevatorState::kSettingElevatorWaitMode,
        make_effect(
          ElevatorEffectKind::kSetOperatingMode, "", route_->source.floor_id,
          "ELEVATOR_WAIT"));
    case ElevatorState::kSettingElevatorWaitMode:
      return transition(
        ElevatorState::kReleasingHallHold,
        make_effect(
          ElevatorEffectKind::kReleaseSafetyHold, "", route_->source.floor_id, "",
          "call_complete"));
    case ElevatorState::kReleasingHallHold:
      return transition(
        ElevatorState::kNavigatingSourceLanding,
        navigation_effect(PoseRole::kLanding, true));
    case ElevatorState::kNavigatingSourceLanding:
      return transition(
        ElevatorState::kWaitingSourceDoor,
        make_effect(
          ElevatorEffectKind::kMockWaitDoorOpen, "", route_->source.floor_id, "",
          "mock_source_door"));
    case ElevatorState::kWaitingSourceDoor:
      return transition(
        ElevatorState::kSettingDoorwayEntryMode,
        make_effect(
          ElevatorEffectKind::kSetOperatingMode, "", route_->source.floor_id, "DOORWAY"));
    case ElevatorState::kSettingDoorwayEntryMode:
      return transition(
        ElevatorState::kEnteringCabin,
        navigation_effect(PoseRole::kCabin, true));
    case ElevatorState::kEnteringCabin:
      if (route_->schema_version == 3U) {
        return transition(
          ElevatorState::kNavigatingCabinPanel,
          navigation_effect(PoseRole::kCabinPanel, true));
      }
      return transition(
        ElevatorState::kAcquiringCabinHold,
        make_effect(
          ElevatorEffectKind::kAcquireSafetyHold, "", route_->source.floor_id, "",
          "cabin_pose_settled"));
    case ElevatorState::kNavigatingCabinPanel:
      return transition(
        ElevatorState::kAcquiringCabinHold,
        make_effect(
          ElevatorEffectKind::kAcquireSafetyHold, "", route_->source.floor_id, "",
          "cabin_panel_pose_settled"));
    case ElevatorState::kAcquiringCabinHold:
    {
      auto effect = make_effect(
          ElevatorEffectKind::kMockPressTargetButton, "", route_->target.floor_id, "",
          "mock_port");
      effect.panel_side = route_->source.cabin_panel_side;
      return transition(ElevatorState::kPressingTargetButton, std::move(effect));
    }
    case ElevatorState::kPressingTargetButton:
      return transition(
        ElevatorState::kPausingCorrections,
        make_effect(
          ElevatorEffectKind::kPauseLocalizationCorrections, "",
          route_->source.floor_id, "", "owner_scoped_pause"));
    case ElevatorState::kPausingCorrections:
      return transition(
        ElevatorState::kSettingRideMode,
        make_effect(
          ElevatorEffectKind::kSetOperatingMode, "", route_->source.floor_id,
          "ELEVATOR_RIDE"));
    case ElevatorState::kSettingRideMode:
      return transition(
        ElevatorState::kRiding,
        make_effect(
          ElevatorEffectKind::kMockRide, "", route_->target.floor_id, "",
          "mock_port"));
    case ElevatorState::kRiding:
      return transition(
        ElevatorState::kWaitingTargetDoor,
        make_effect(
          ElevatorEffectKind::kMockWaitDoorOpen, "", route_->target.floor_id, "",
          "mock_target_door"));
    case ElevatorState::kWaitingTargetDoor:
    {
      const auto anchor_role = route_->schema_version == 3U ?
        PoseRole::kCabinPanel : PoseRole::kCabin;
      auto effect = make_effect(
        ElevatorEffectKind::kBeginFloorTransition,
        find_pose_id(route_->target, anchor_role).value_or(""),
        route_->target.floor_id, "",
        "floor_manager_acquires_pause_then_invalidates_source_context");
      effect.map_id = route_->target.map_id;
      return transition(ElevatorState::kBeginningFloorTransition, std::move(effect));
    }
    case ElevatorState::kBeginningFloorTransition:
      return transition(
        ElevatorState::kResumingCorrections,
        make_effect(
          ElevatorEffectKind::kResumeLocalizationCorrections, "",
          route_->source.floor_id, "",
          "floor_manager_pause_keeps_effective_pause_under_hold"));
    case ElevatorState::kResumingCorrections:
    {
      const auto anchor_role = route_->schema_version == 3U ?
        PoseRole::kCabinPanel : PoseRole::kCabin;
      auto effect = make_effect(
        ElevatorEffectKind::kSwitchFloor,
        find_pose_id(route_->target, anchor_role).value_or(""),
        route_->target.floor_id, "",
        "floor_manager_owns_pause_gap;hold_remains_active");
      effect.map_id = route_->target.map_id;
      return transition(ElevatorState::kSwitchingFloor, std::move(effect));
    }
    case ElevatorState::kSwitchingFloor:
    {
      const auto anchor_role = route_->schema_version == 3U ?
        PoseRole::kCabinPanel : PoseRole::kCabin;
      auto effect = make_effect(
        ElevatorEffectKind::kVerifyFloorReady,
        find_pose_id(route_->target, anchor_role).value_or(""),
        route_->target.floor_id);
      effect.map_id = route_->target.map_id;
      return transition(ElevatorState::kVerifyingFloorReady, std::move(effect));
    }
    case ElevatorState::kVerifyingFloorReady:
      return transition(
        ElevatorState::kSettingDoorwayExitMode,
        make_effect(
          ElevatorEffectKind::kSetOperatingMode, "", route_->target.floor_id, "DOORWAY"));
    case ElevatorState::kSettingDoorwayExitMode:
      if (route_->schema_version == 3U) {
        return transition(
          ElevatorState::kReturningToCabinCenter,
          navigation_effect(PoseRole::kCabin, false));
      }
      return transition(
        ElevatorState::kReleasingCabinHold,
        make_effect(
          ElevatorEffectKind::kReleaseSafetyHold, "", route_->target.floor_id, "",
          "target_floor_ready"));
    case ElevatorState::kReturningToCabinCenter:
      return transition(
        ElevatorState::kReleasingCabinHold,
        make_effect(
          ElevatorEffectKind::kReleaseSafetyHold, "", route_->target.floor_id, "",
          "target_cabin_center_ready"));
    case ElevatorState::kReleasingCabinHold:
      return transition(
        ElevatorState::kNavigatingTargetLanding,
        navigation_effect(PoseRole::kLanding, false));
    case ElevatorState::kNavigatingTargetLanding:
      return transition(
        ElevatorState::kAcquiringExitHold,
        make_effect(
          ElevatorEffectKind::kAcquireSafetyHold, "", route_->target.floor_id, "",
          "target_landing_pose_settled"));
    case ElevatorState::kAcquiringExitHold:
      return transition(
        ElevatorState::kReleasingOperatingMode,
        make_effect(
          ElevatorEffectKind::kReleaseOperatingMode, "", route_->target.floor_id, "",
          "release_exact_mode_lease"));
    case ElevatorState::kReleasingOperatingMode:
    case ElevatorState::kReleasingExecutionLease:
      return transition(
        ElevatorState::kReleasingExitHold,
        make_effect(
          ElevatorEffectKind::kReleaseSafetyHold, "", route_->target.floor_id, "",
          "exit_verified"));
    case ElevatorState::kReleasingExitHold:
      return transition(
        ElevatorState::kComplete,
        make_effect(
          ElevatorEffectKind::kComplete, "", route_->target.floor_id, "",
          "mock_elevator_flow_complete"));
    case ElevatorState::kIdle:
    case ElevatorState::kComplete:
    case ElevatorState::kFailureCleanup:
    case ElevatorState::kLocked:
      break;
  }
  return lock("effect_success_not_valid_for_state:" + to_string(state_), true);
}

ElevatorFsmOutput ElevatorFsm::transition(
  const ElevatorState next,
  ElevatorEffect effect)
{
  state_ = next;
  effect.sequence = next_effect_sequence_++;
  effect.transaction_id = transaction_id_;
  pending_effect_sequence_ = effect.sequence;
  return ElevatorFsmOutput{
    true,
    true,
    state_,
    std::move(effect),
    "transitioned_to:" + to_string(state_),
  };
}

ElevatorFsmOutput ElevatorFsm::lock(const std::string & reason, const bool accepted)
{
  state_ = ElevatorState::kFailureCleanup;
  auto effect = make_effect(ElevatorEffectKind::kHoldAndCancel, "", "", "", reason);
  effect.sequence = next_effect_sequence_++;
  effect.transaction_id = transaction_id_;
  pending_effect_sequence_ = effect.sequence;
  return ElevatorFsmOutput{
    accepted,
    true,
    state_,
    std::move(effect),
    reason,
  };
}

ElevatorFsmOutput ElevatorFsm::inert(const std::string & reason) const
{
  return ElevatorFsmOutput{
    false,
    false,
    state_,
    make_effect(ElevatorEffectKind::kNone),
    reason,
  };
}

ElevatorEffect ElevatorFsm::navigation_effect(
  const PoseRole role,
  const bool source) const
{
  const auto & floor = source ? route_->source : route_->target;
  auto effect = make_effect(
    ElevatorEffectKind::kNavigateToPose,
    find_pose_id(floor, role).value_or(""),
    floor.floor_id,
    "",
    to_string(role));
  effect.map_id = floor.map_id;
  if (role == PoseRole::kHallCall) {
    effect.navigation_intent = ElevatorNavigationIntent::kHallCall;
  } else if (route_->schema_version == 3U) {
    if (role == PoseRole::kLanding) {
      effect.navigation_intent = source ?
        ElevatorNavigationIntent::kReverseEntryStaging :
        ElevatorNavigationIntent::kTargetLanding;
    } else if (role == PoseRole::kCabin) {
      effect.navigation_intent = source ?
        ElevatorNavigationIntent::kReverseEnterCabin :
        ElevatorNavigationIntent::kReturnCabinCenter;
    } else if (role == PoseRole::kCabinPanel && source) {
      effect.navigation_intent =
        ElevatorNavigationIntent::kCabinPanelApproach;
    }
  } else if (role == PoseRole::kLanding) {
    effect.navigation_intent = source ?
      ElevatorNavigationIntent::kSourceLanding :
      ElevatorNavigationIntent::kTargetLanding;
  } else if (role == PoseRole::kCabin && source) {
    effect.navigation_intent = ElevatorNavigationIntent::kEnterCabin;
  }
  return effect;
}

ElevatorState ElevatorFsm::state() const noexcept
{
  return state_;
}

bool ElevatorFsm::locked() const noexcept
{
  return state_ == ElevatorState::kFailureCleanup ||
         state_ == ElevatorState::kLocked;
}

std::string to_string(const ElevatorState state)
{
  switch (state) {
    case ElevatorState::kIdle:
      return "IDLE";
    case ElevatorState::kNavigatingHallCall:
      return "NAVIGATING_HALL_CALL";
    case ElevatorState::kAcquiringHallHold:
      return "ACQUIRING_HALL_HOLD";
    case ElevatorState::kAcquiringExecutionLease:
      return "ACQUIRING_EXECUTION_LEASE";
    case ElevatorState::kPressingCallButton:
      return "PRESSING_CALL_BUTTON";
    case ElevatorState::kSettingElevatorWaitMode:
      return "SETTING_ELEVATOR_WAIT_MODE";
    case ElevatorState::kReleasingHallHold:
      return "RELEASING_HALL_HOLD";
    case ElevatorState::kNavigatingSourceLanding:
      return "NAVIGATING_SOURCE_LANDING";
    case ElevatorState::kWaitingSourceDoor:
      return "WAITING_SOURCE_DOOR";
    case ElevatorState::kSettingDoorwayEntryMode:
      return "SETTING_DOORWAY_ENTRY_MODE";
    case ElevatorState::kEnteringCabin:
      return "ENTERING_CABIN";
    case ElevatorState::kNavigatingCabinPanel:
      return "NAVIGATING_CABIN_PANEL";
    case ElevatorState::kAcquiringCabinHold:
      return "ACQUIRING_CABIN_HOLD";
    case ElevatorState::kPressingTargetButton:
      return "PRESSING_TARGET_BUTTON";
    case ElevatorState::kPausingCorrections:
      return "PAUSING_CORRECTIONS";
    case ElevatorState::kSettingRideMode:
      return "SETTING_RIDE_MODE";
    case ElevatorState::kRiding:
      return "RIDING";
    case ElevatorState::kWaitingTargetDoor:
      return "WAITING_TARGET_DOOR";
    case ElevatorState::kBeginningFloorTransition:
      return "BEGINNING_FLOOR_TRANSITION";
    case ElevatorState::kResumingCorrections:
      return "RESUMING_CORRECTIONS";
    case ElevatorState::kSwitchingFloor:
      return "SWITCHING_FLOOR";
    case ElevatorState::kVerifyingFloorReady:
      return "VERIFYING_FLOOR_READY";
    case ElevatorState::kSettingDoorwayExitMode:
      return "SETTING_DOORWAY_EXIT_MODE";
    case ElevatorState::kReturningToCabinCenter:
      return "RETURNING_TO_CABIN_CENTER";
    case ElevatorState::kReleasingCabinHold:
      return "RELEASING_CABIN_HOLD";
    case ElevatorState::kNavigatingTargetLanding:
      return "NAVIGATING_TARGET_LANDING";
    case ElevatorState::kAcquiringExitHold:
      return "ACQUIRING_EXIT_HOLD";
    case ElevatorState::kReleasingOperatingMode:
      return "RELEASING_OPERATING_MODE";
    case ElevatorState::kReleasingExecutionLease:
      return "RELEASING_EXECUTION_LEASE";
    case ElevatorState::kReleasingExitHold:
      return "RELEASING_EXIT_HOLD";
    case ElevatorState::kComplete:
      return "COMPLETE";
    case ElevatorState::kFailureCleanup:
      return "FAILURE_CLEANUP";
    case ElevatorState::kLocked:
      return "LOCKED";
  }
  return "UNKNOWN";
}

std::string to_string(const ElevatorEffectKind effect)
{
  switch (effect) {
    case ElevatorEffectKind::kNone:
      return "NONE";
    case ElevatorEffectKind::kNavigateToPose:
      return "NAVIGATE_TO_POSE";
    case ElevatorEffectKind::kAcquireSafetyHold:
      return "ACQUIRE_SAFETY_HOLD";
    case ElevatorEffectKind::kReleaseSafetyHold:
      return "RELEASE_SAFETY_HOLD";
    case ElevatorEffectKind::kAcquireExecutionLease:
      return "ACQUIRE_EXECUTION_LEASE";
    case ElevatorEffectKind::kReleaseExecutionLease:
      return "RELEASE_EXECUTION_LEASE";
    case ElevatorEffectKind::kMockPressCallButton:
      return "MOCK_PRESS_CALL_BUTTON";
    case ElevatorEffectKind::kMockPressTargetButton:
      return "MOCK_PRESS_TARGET_BUTTON";
    case ElevatorEffectKind::kMockWaitDoorOpen:
      return "MOCK_WAIT_DOOR_OPEN";
    case ElevatorEffectKind::kMockRide:
      return "MOCK_RIDE";
    case ElevatorEffectKind::kSetOperatingMode:
      return "SET_OPERATING_MODE";
    case ElevatorEffectKind::kReleaseOperatingMode:
      return "RELEASE_OPERATING_MODE";
    case ElevatorEffectKind::kPauseLocalizationCorrections:
      return "PAUSE_LOCALIZATION_CORRECTIONS";
    case ElevatorEffectKind::kResumeLocalizationCorrections:
      return "RESUME_LOCALIZATION_CORRECTIONS";
    case ElevatorEffectKind::kBeginFloorTransition:
      return "BEGIN_FLOOR_TRANSITION";
    case ElevatorEffectKind::kSwitchFloor:
      return "SWITCH_FLOOR";
    case ElevatorEffectKind::kVerifyFloorReady:
      return "VERIFY_FLOOR_READY";
    case ElevatorEffectKind::kComplete:
      return "COMPLETE";
    case ElevatorEffectKind::kHoldAndCancel:
      return "HOLD_AND_CANCEL";
  }
  return "NONE";
}

std::string to_string(const ElevatorNavigationIntent intent)
{
  switch (intent) {
    case ElevatorNavigationIntent::kNone:
      return "NONE";
    case ElevatorNavigationIntent::kHallCall:
      return "HALL_CALL";
    case ElevatorNavigationIntent::kSourceLanding:
      return "SOURCE_LANDING_FACE_CABIN";
    case ElevatorNavigationIntent::kEnterCabin:
      return "ENTER_CABIN";
    case ElevatorNavigationIntent::kReverseEntryStaging:
      return "REVERSE_ENTRY_STAGING";
    case ElevatorNavigationIntent::kReverseEnterCabin:
      return "REVERSE_ENTER_CABIN";
    case ElevatorNavigationIntent::kCabinPanelApproach:
      return "CABIN_PANEL_APPROACH";
    case ElevatorNavigationIntent::kReturnCabinCenter:
      return "RETURN_CABIN_CENTER";
    case ElevatorNavigationIntent::kTargetLanding:
      return "TARGET_LANDING_FACE_HALL";
  }
  return "NONE";
}

}  // namespace robot_elevator_manager
