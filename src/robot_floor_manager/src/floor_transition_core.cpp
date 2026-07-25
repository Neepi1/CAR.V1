#include "robot_floor_manager/floor_transition_core.hpp"

#include <algorithm>
#include <cstddef>
#include <cctype>
#include <utility>

namespace robot_floor_manager
{
namespace
{

bool safe_identifier(
  const std::string & value)
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

}  // namespace

FloorTransitionOutput FloorTransitionCore::start(
  const FloorTransitionRequest & request)
{
  if (request_.has_value()) {
    if (
      request.transaction_id == request_->transaction_id &&
      request.building_id == request_->building_id &&
      request.floor_id == request_->floor_id &&
      request.map_id == request_->map_id &&
      request.expected_asset_epoch == request_->expected_asset_epoch &&
      request.expected_asset_digest == request_->expected_asset_digest)
    {
      FloorTransitionOutput output;
      output.accepted = true;
      output.ignored = true;
      output.state = state_;
      output.effect = pending_effect_;
      output.runtime_context_valid = runtime_context_valid_;
      output.recovery_required = recovery_required_;
      output.message = "idempotent transaction replay";
      return output;
    }
    return inert("another floor transaction is already active or retained", false);
  }
  if (
    !safe_identifier(request.transaction_id) ||
    !safe_identifier(request.building_id) ||
    !safe_identifier(request.floor_id) ||
    !safe_identifier(request.map_id) ||
    request.expected_asset_epoch == 0U ||
    !canonical_sha256(request.expected_asset_digest))
  {
    return inert(
      "floor transition request requires safe identifiers, a nonzero asset epoch, "
      "and a canonical sha256 digest",
      false);
  }

  request_ = request;
  mutation_started_ = false;
  runtime_context_valid_ = true;
  recovery_required_ = false;
  return transition(
    FloorTransitionState::kVerifyingPreconditions,
    FloorTransitionEffectKind::kVerifyPreconditions,
    "require motion hold, Nav2 idle, and fresh stopped evidence");
}

FloorTransitionOutput FloorTransitionCore::dispatch(
  const FloorTransitionEvent & event)
{
  if (!request_.has_value()) {
    return inert("no floor transaction is active", true);
  }
  if (event.transaction_id != request_->transaction_id) {
    return inert("foreign floor transaction event ignored", true);
  }
  if (state_ == FloorTransitionState::kComplete ||
    state_ == FloorTransitionState::kFailedLocked)
  {
    return inert("floor transaction is terminal", true);
  }
  if (event.effect_sequence != pending_effect_.sequence) {
    if (state_ == FloorTransitionState::kFailureCleanup) {
      return inert("stale failure-cleanup acknowledgement ignored", true);
    }
    return fail("effect sequence mismatch");
  }
  if (state_ == FloorTransitionState::kFailureCleanup) {
    if (event.kind == FloorTransitionEventKind::kEffectSucceeded) {
      state_ = FloorTransitionState::kFailedLocked;
      pending_effect_ = FloorTransitionEffect{};
      FloorTransitionOutput output;
      output.accepted = true;
      output.transitioned = true;
      output.state = state_;
      output.runtime_context_valid = runtime_context_valid_;
      output.recovery_required = recovery_required_;
      output.message = "failure cleanup acknowledged; recovery lock retained";
      return output;
    }
    return fail(
      event.detail.empty() ?
      "failure cleanup not confirmed; retry" :
      "failure cleanup not confirmed; retry:" + event.detail);
  }
  if (event.kind == FloorTransitionEventKind::kCancelRequested) {
    return fail(
      event.detail.empty() ? "cancel requested" : "cancel requested:" + event.detail);
  }
  if (event.kind == FloorTransitionEventKind::kEffectFailed) {
    return fail(
      event.detail.empty() ? "floor transition effect failed" :
      "floor transition effect failed:" + event.detail);
  }
  return advance(event.evidence);
}

FloorTransitionOutput FloorTransitionCore::advance(
  const FloorTransitionEvidence & evidence)
{
  switch (state_) {
    case FloorTransitionState::kVerifyingPreconditions:
      if (!evidence.motion_hold_active || !evidence.nav_idle || !evidence.stopped) {
        return fail("preconditions require hold, Nav2 idle, and stopped evidence");
      }
      return transition(
        FloorTransitionState::kAcquiringCorrectionPause,
        FloorTransitionEffectKind::kAcquireCorrectionPause,
        "acquire floor-manager-owned correction pause");

    case FloorTransitionState::kAcquiringCorrectionPause:
      if (!evidence.floor_pause_owned || !evidence.correction_pause_effective) {
        return fail("floor correction-pause ownership was not proven");
      }
      return transition(
        FloorTransitionState::kInvalidatingRuntimeContext,
        FloorTransitionEffectKind::kInvalidateRuntimeContext,
        "invalidate ordinary source-floor runtime context");

    case FloorTransitionState::kInvalidatingRuntimeContext:
      if (
        !evidence.runtime_context_invalid ||
        !evidence.floor_pause_owned ||
        !evidence.correction_pause_effective)
      {
        return fail("source context invalidation and floor pause were not proven");
      }
      mutation_started_ = true;
      runtime_context_valid_ = false;
      recovery_required_ = true;
      return transition(
        FloorTransitionState::kReportingBeginReady,
        FloorTransitionEffectKind::kReportBeginReady,
        "begin barrier reached; caller may release only its own pause");

    case FloorTransitionState::kReportingBeginReady:
      return transition(
        FloorTransitionState::kVerifyingPauseHandoff,
        FloorTransitionEffectKind::kVerifyPauseHandoff,
        "verify caller pause released while floor pause remains effective");

    case FloorTransitionState::kVerifyingPauseHandoff:
      if (
        !evidence.caller_pause_released ||
        !evidence.floor_pause_owned ||
        !evidence.correction_pause_effective ||
        !evidence.runtime_context_invalid)
      {
        return fail("correction-pause ownership handoff was not atomic");
      }
      return transition(
        FloorTransitionState::kLoadingNavMap,
        FloorTransitionEffectKind::kLoadNavMap,
        "load target Nav2 map for the pending asset epoch");

    case FloorTransitionState::kLoadingNavMap:
      if (!target_asset_matches(evidence)) {
        return fail("Nav2 map evidence does not match target asset");
      }
      return transition(
        FloorTransitionState::kLoadingFilters,
        FloorTransitionEffectKind::kLoadFilters,
        "load enabled target filter masks from the same asset epoch");

    case FloorTransitionState::kLoadingFilters:
      if (!target_asset_matches(evidence)) {
        return fail("filter evidence does not match target asset");
      }
      return transition(
        FloorTransitionState::kReloadingLocalizer,
        FloorTransitionEffectKind::kReloadLocalizer,
        "reload target localizer asset rather than caching paths");

    case FloorTransitionState::kReloadingLocalizer:
      if (!target_asset_matches(evidence)) {
        return fail("localizer evidence does not match target asset");
      }
      return transition(
        FloorTransitionState::kReleasingFloorPause,
        FloorTransitionEffectKind::kReleaseFloorPause,
        "release exact floor pause before target explicit localization");

    case FloorTransitionState::kReleasingFloorPause:
      if (evidence.floor_pause_owned || evidence.correction_pause_effective) {
        return fail("another correction pause remains; target localization cannot start");
      }
      return transition(
        FloorTransitionState::kTriggeringExplicitLocalization,
        FloorTransitionEffectKind::kTriggerExplicitLocalization,
        "trigger target-epoch explicit localization under motion hold");

    case FloorTransitionState::kTriggeringExplicitLocalization:
      if (
        !target_asset_matches(evidence) ||
        evidence.explicit_relocalization_sequence == 0U)
      {
        return fail("explicit localization did not prove target epoch and sequence");
      }
      return transition(
        FloorTransitionState::kVerifyingBridgeReady,
        FloorTransitionEffectKind::kVerifyBridgeReady,
        "require bridge-accepted target map->odom and exact context");

    case FloorTransitionState::kVerifyingBridgeReady:
      if (
        !evidence.bridge_ready ||
        !evidence.runtime_context_invalid ||
        !target_context_matches(evidence) ||
        evidence.explicit_relocalization_sequence == 0U)
      {
        return fail("bridge readiness did not match the pending target context");
      }
      return transition(
        FloorTransitionState::kClearingCostmaps,
        FloorTransitionEffectKind::kClearCostmaps,
        "clear global and local costmaps with typed Nav2 services");

    case FloorTransitionState::kClearingCostmaps:
      if (!target_asset_matches(evidence)) {
        return fail("costmap clear acknowledgement does not match target epoch");
      }
      return transition(
        FloorTransitionState::kVerifyingFreshCostmaps,
        FloorTransitionEffectKind::kVerifyFreshCostmaps,
        "wait for fresh global and local costmaps from target epoch");

    case FloorTransitionState::kVerifyingFreshCostmaps:
      if (
        !target_context_matches(evidence) ||
        !evidence.global_costmap_fresh ||
        !evidence.local_costmap_fresh)
      {
        return fail("fresh target costmaps were not proven");
      }
      return transition(
        FloorTransitionState::kCommittingRuntimeContext,
        FloorTransitionEffectKind::kCommitRuntimeContext,
        "commit target runtime context only after every readiness barrier");

    case FloorTransitionState::kCommittingRuntimeContext:
      if (
        !target_context_matches(evidence) ||
        !evidence.runtime_context_valid ||
        !evidence.safe_for_goal_start ||
        evidence.explicit_relocalization_sequence == 0U)
      {
        return fail("target runtime context commit evidence is incomplete");
      }
      runtime_context_valid_ = true;
      recovery_required_ = false;
      return transition(
        FloorTransitionState::kCompleting,
        FloorTransitionEffectKind::kComplete,
        "floor transition committed");

    case FloorTransitionState::kCompleting:
      state_ = FloorTransitionState::kComplete;
      pending_effect_ = FloorTransitionEffect{};
      return FloorTransitionOutput{
        true,
        false,
        true,
        state_,
        pending_effect_,
        runtime_context_valid_,
        recovery_required_,
        "floor transition complete",
      };

    case FloorTransitionState::kIdle:
    case FloorTransitionState::kComplete:
    case FloorTransitionState::kFailureCleanup:
    case FloorTransitionState::kFailedLocked:
      break;
  }
  return fail("effect success is invalid for current floor transition state");
}

FloorTransitionOutput FloorTransitionCore::transition(
  const FloorTransitionState state,
  const FloorTransitionEffectKind effect,
  const std::string & detail)
{
  state_ = state;
  pending_effect_.kind = effect;
  pending_effect_.transaction_id = request_->transaction_id;
  pending_effect_.sequence = next_effect_sequence_++;
  pending_effect_.building_id = request_->building_id;
  pending_effect_.floor_id = request_->floor_id;
  pending_effect_.map_id = request_->map_id;
  pending_effect_.expected_asset_epoch = request_->expected_asset_epoch;
  pending_effect_.expected_asset_digest = request_->expected_asset_digest;
  pending_effect_.cleanup_id.clear();
  pending_effect_.detail = detail;
  return FloorTransitionOutput{
    true,
    false,
    true,
    state_,
    pending_effect_,
    runtime_context_valid_,
    recovery_required_,
    detail,
  };
}

FloorTransitionOutput FloorTransitionCore::fail(const std::string & reason)
{
  state_ = FloorTransitionState::kFailureCleanup;
  recovery_required_ = mutation_started_;
  pending_effect_.kind = FloorTransitionEffectKind::kHoldAndLock;
  pending_effect_.transaction_id = request_.has_value() ? request_->transaction_id : "";
  pending_effect_.sequence = next_effect_sequence_++;
  pending_effect_.building_id = request_.has_value() ? request_->building_id : "";
  pending_effect_.floor_id = request_.has_value() ? request_->floor_id : "";
  pending_effect_.map_id = request_.has_value() ? request_->map_id : "";
  pending_effect_.expected_asset_epoch =
    request_.has_value() ? request_->expected_asset_epoch : 0U;
  pending_effect_.expected_asset_digest =
    request_.has_value() ? request_->expected_asset_digest : "";
  pending_effect_.cleanup_id =
    request_.has_value() ? request_->transaction_id + "-cleanup" : "floor-cleanup";
  pending_effect_.detail = reason;
  return FloorTransitionOutput{
    true,
    false,
    true,
    state_,
    pending_effect_,
    runtime_context_valid_,
    recovery_required_,
    reason,
  };
}

FloorTransitionOutput FloorTransitionCore::inert(
  const std::string & reason,
  const bool ignored) const
{
  FloorTransitionOutput output;
  output.ignored = ignored;
  output.state = state_;
  output.runtime_context_valid = runtime_context_valid_;
  output.recovery_required = recovery_required_;
  output.message = reason;
  return output;
}

bool FloorTransitionCore::target_asset_matches(
  const FloorTransitionEvidence & evidence) const
{
  return evidence.asset_epoch == request_->expected_asset_epoch &&
         evidence.asset_digest == request_->expected_asset_digest;
}

bool FloorTransitionCore::target_context_matches(
  const FloorTransitionEvidence & evidence) const
{
  return target_asset_matches(evidence) &&
         evidence.active_building_id == request_->building_id &&
         evidence.active_floor_id == request_->floor_id &&
         evidence.active_map_id == request_->map_id;
}

FloorTransitionState FloorTransitionCore::state() const noexcept
{
  return state_;
}

bool FloorTransitionCore::runtime_context_valid() const noexcept
{
  return runtime_context_valid_;
}

bool FloorTransitionCore::recovery_required() const noexcept
{
  return recovery_required_;
}

std::optional<FloorTransitionRequest> FloorTransitionCore::active_request() const
{
  return request_;
}

const char * to_string(const FloorTransitionState state) noexcept
{
  switch (state) {
    case FloorTransitionState::kIdle: return "IDLE";
    case FloorTransitionState::kVerifyingPreconditions: return "VERIFYING_PRECONDITIONS";
    case FloorTransitionState::kAcquiringCorrectionPause: return "ACQUIRING_CORRECTION_PAUSE";
    case FloorTransitionState::kInvalidatingRuntimeContext: return "INVALIDATING_RUNTIME_CONTEXT";
    case FloorTransitionState::kReportingBeginReady: return "REPORTING_BEGIN_READY";
    case FloorTransitionState::kVerifyingPauseHandoff: return "VERIFYING_PAUSE_HANDOFF";
    case FloorTransitionState::kLoadingNavMap: return "LOADING_NAV_MAP";
    case FloorTransitionState::kLoadingFilters: return "LOADING_FILTERS";
    case FloorTransitionState::kReloadingLocalizer: return "RELOADING_LOCALIZER";
    case FloorTransitionState::kReleasingFloorPause: return "RELEASING_FLOOR_PAUSE";
    case FloorTransitionState::kTriggeringExplicitLocalization:
      return "TRIGGERING_EXPLICIT_LOCALIZATION";
    case FloorTransitionState::kVerifyingBridgeReady: return "VERIFYING_BRIDGE_READY";
    case FloorTransitionState::kClearingCostmaps: return "CLEARING_COSTMAPS";
    case FloorTransitionState::kVerifyingFreshCostmaps: return "VERIFYING_FRESH_COSTMAPS";
    case FloorTransitionState::kCommittingRuntimeContext: return "COMMITTING_RUNTIME_CONTEXT";
    case FloorTransitionState::kCompleting: return "COMPLETING";
    case FloorTransitionState::kComplete: return "COMPLETE";
    case FloorTransitionState::kFailureCleanup: return "FAILURE_CLEANUP";
    case FloorTransitionState::kFailedLocked: return "FAILED_LOCKED";
  }
  return "UNKNOWN";
}

const char * to_string(const FloorTransitionEffectKind effect) noexcept
{
  switch (effect) {
    case FloorTransitionEffectKind::kNone: return "NONE";
    case FloorTransitionEffectKind::kVerifyPreconditions: return "VERIFY_PRECONDITIONS";
    case FloorTransitionEffectKind::kAcquireCorrectionPause:
      return "ACQUIRE_CORRECTION_PAUSE";
    case FloorTransitionEffectKind::kInvalidateRuntimeContext:
      return "INVALIDATE_RUNTIME_CONTEXT";
    case FloorTransitionEffectKind::kReportBeginReady: return "REPORT_BEGIN_READY";
    case FloorTransitionEffectKind::kVerifyPauseHandoff: return "VERIFY_PAUSE_HANDOFF";
    case FloorTransitionEffectKind::kLoadNavMap: return "LOAD_NAV_MAP";
    case FloorTransitionEffectKind::kLoadFilters: return "LOAD_FILTERS";
    case FloorTransitionEffectKind::kReloadLocalizer: return "RELOAD_LOCALIZER";
    case FloorTransitionEffectKind::kReleaseFloorPause: return "RELEASE_FLOOR_PAUSE";
    case FloorTransitionEffectKind::kTriggerExplicitLocalization:
      return "TRIGGER_EXPLICIT_LOCALIZATION";
    case FloorTransitionEffectKind::kVerifyBridgeReady: return "VERIFY_BRIDGE_READY";
    case FloorTransitionEffectKind::kClearCostmaps: return "CLEAR_COSTMAPS";
    case FloorTransitionEffectKind::kVerifyFreshCostmaps:
      return "VERIFY_FRESH_COSTMAPS";
    case FloorTransitionEffectKind::kCommitRuntimeContext:
      return "COMMIT_RUNTIME_CONTEXT";
    case FloorTransitionEffectKind::kComplete: return "COMPLETE";
    case FloorTransitionEffectKind::kHoldAndLock: return "HOLD_AND_LOCK";
  }
  return "UNKNOWN";
}

}  // namespace robot_floor_manager
