#include "robot_floor_manager/floor_transition_executor.hpp"

#include <cstddef>
#include <utility>

namespace robot_floor_manager
{
namespace
{

bool exact_snapshot_matches(
  const FloorTransitionRequest & request,
  const FloorAssetSnapshot & snapshot)
{
  return snapshot.building_id == request.building_id &&
         snapshot.floor_id == request.floor_id &&
         snapshot.map_id == request.map_id &&
         snapshot.asset_epoch == request.expected_asset_epoch &&
         snapshot.asset_digest == request.expected_asset_digest;
}

FloorTransitionEvent event_for(
  const FloorTransitionOutput & output,
  const FloorTransitionEventKind kind,
  const std::string & detail,
  const FloorTransitionEvidence & evidence)
{
  FloorTransitionEvent event;
  event.kind = kind;
  event.transaction_id = output.effect.transaction_id;
  event.effect_sequence = output.effect.sequence;
  event.detail = detail;
  event.evidence = evidence;
  return event;
}

bool cancellation_allowed(const FloorTransitionState state)
{
  return state != FloorTransitionState::kCompleting &&
         state != FloorTransitionState::kComplete;
}

}  // namespace

const char * floor_transition_feedback_stage(
  const FloorTransitionOutput & output) noexcept
{
  if (output.state == FloorTransitionState::kReportingBeginReady) {
    return "CALLER_PAUSE_HANDOFF_READY";
  }
  return to_string(output.effect.kind);
}

bool floor_transition_caller_pause_handoff_ready(
  const FloorTransitionOutput & output) noexcept
{
  return
    output.state == FloorTransitionState::kReportingBeginReady ||
    output.state == FloorTransitionState::kVerifyingPauseHandoff;
}

float floor_transition_feedback_progress(
  const FloorTransitionState state) noexcept
{
  switch (state) {
    case FloorTransitionState::kIdle: return 0.0F;
    case FloorTransitionState::kVerifyingPreconditions: return 0.05F;
    case FloorTransitionState::kAcquiringCorrectionPause: return 0.10F;
    case FloorTransitionState::kInvalidatingRuntimeContext: return 0.15F;
    case FloorTransitionState::kReportingBeginReady: return 0.20F;
    case FloorTransitionState::kVerifyingPauseHandoff: return 0.25F;
    case FloorTransitionState::kLoadingNavMap: return 0.35F;
    case FloorTransitionState::kLoadingFilters: return 0.45F;
    case FloorTransitionState::kReloadingLocalizer: return 0.55F;
    case FloorTransitionState::kReleasingFloorPause: return 0.60F;
    case FloorTransitionState::kTriggeringExplicitLocalization: return 0.70F;
    case FloorTransitionState::kVerifyingBridgeReady: return 0.78F;
    case FloorTransitionState::kClearingCostmaps: return 0.84F;
    case FloorTransitionState::kVerifyingFreshCostmaps: return 0.90F;
    case FloorTransitionState::kCommittingRuntimeContext: return 0.96F;
    case FloorTransitionState::kCompleting: return 0.99F;
    case FloorTransitionState::kComplete: return 1.0F;
    case FloorTransitionState::kFailureCleanup:
    case FloorTransitionState::kFailed:
    case FloorTransitionState::kFailedLocked:
      return 1.0F;
  }
  return 0.0F;
}

FloorTransitionFailureDisposition floor_transition_failure_disposition(
  const FloorTransitionExecutionResult & execution) noexcept
{
  FloorTransitionFailureDisposition disposition;
  disposition.recovery_locked =
    execution.recovery_required ||
    !execution.runtime_context_valid ||
    execution.state == FloorTransitionState::kFailedLocked;
  disposition.canceled =
    execution.failure_code == "CANCELLED" &&
    !disposition.recovery_locked;
  if (disposition.canceled) {
    disposition.state = "CANCELED";
    disposition.stage = "CANCELED";
  } else if (disposition.recovery_locked) {
    disposition.state = "FAILED_LOCKED";
    disposition.stage = "RECOVERY_LOCKED";
  }
  return disposition;
}

FloorTransitionExecutor::FloorTransitionExecutor(
  FloorTransitionRuntimePort & runtime)
: runtime_(runtime)
{
}

FloorTransitionExecutionResult FloorTransitionExecutor::run(
  const FloorTransitionRequest & request,
  const FloorAssetSnapshot & snapshot,
  FloorTransitionCancelProbe cancel_requested,
  FloorTransitionProgressSink progress)
{
  if (!exact_snapshot_matches(request, snapshot)) {
    FloorTransitionExecutionResult result;
    result.failure_code = "ASSET_IDENTITY_MISMATCH";
    result.message =
      "verified floor snapshot does not match the exact transaction identity";
    result.runtime_context_valid = true;
    return result;
  }

  FloorTransitionCore core;
  auto output = core.start(request);
  FloorTransitionEvidence last_evidence;
  std::string failure_code;
  std::string failure_message;
  constexpr std::size_t kMaximumEffects = 32U;

  for (std::size_t step = 0U; step < kMaximumEffects; ++step) {
    if (progress) {
      progress(output);
    }
    if (output.state == FloorTransitionState::kComplete ||
      output.state == FloorTransitionState::kFailed ||
      output.state == FloorTransitionState::kFailedLocked)
    {
      return terminal_result(
        output, last_evidence, failure_code, failure_message);
    }
    if (!output.accepted || output.effect.kind == FloorTransitionEffectKind::kNone) {
      failure_code = failure_code.empty() ? "INTERNAL_ERROR" : failure_code;
      failure_message =
        output.message.empty() ? "floor transition produced no executable effect" :
        output.message;
      return terminal_result(
        output, last_evidence, failure_code, failure_message);
    }

    if (cancel_requested && cancel_requested() &&
      cancellation_allowed(output.state) &&
      output.effect.kind != FloorTransitionEffectKind::kHoldAndLock)
    {
      output = core.dispatch(
        event_for(
          output,
          FloorTransitionEventKind::kCancelRequested,
          "cancel requested",
          last_evidence));
      failure_code = "CANCELLED";
      failure_message = "floor transition cancellation entered safety cleanup";
      continue;
    }

    const auto effect_result = runtime_.perform(output.effect, snapshot);
    if (effect_result.success) {
      last_evidence = effect_result.evidence;
      output = core.dispatch(
        event_for(
          output,
          FloorTransitionEventKind::kEffectSucceeded,
          effect_result.detail,
          effect_result.evidence));
      continue;
    }

    if (failure_code.empty()) {
      failure_code = effect_result.failure_code.empty() ?
        "RUNTIME_EFFECT_FAILED" : effect_result.failure_code;
      failure_message = effect_result.detail.empty() ?
        "floor transition runtime effect failed" : effect_result.detail;
    }
    output = core.dispatch(
      event_for(
        output,
        FloorTransitionEventKind::kEffectFailed,
        failure_message,
        effect_result.evidence));
  }

  failure_code = "INTERNAL_ERROR";
  failure_message = "floor transition exceeded its bounded effect count";
  return terminal_result(output, last_evidence, failure_code, failure_message);
}

FloorTransitionExecutionResult FloorTransitionExecutor::terminal_result(
  const FloorTransitionOutput & output,
  const FloorTransitionEvidence & evidence,
  const std::string & failure_code,
  const std::string & message) const
{
  FloorTransitionExecutionResult result;
  result.success = output.state == FloorTransitionState::kComplete &&
    output.runtime_context_valid && !output.recovery_required;
  result.state = output.state;
  result.failure_code = result.success ? "" :
    (failure_code.empty() ? "RUNTIME_EFFECT_FAILED" : failure_code);
  result.message = message.empty() ? output.message : message;
  result.runtime_context_valid = output.runtime_context_valid;
  result.recovery_required = output.recovery_required;
  if (result.success) {
    result.active_building_id = evidence.active_building_id;
    result.active_floor_id = evidence.active_floor_id;
    result.active_map_id = evidence.active_map_id;
    result.asset_epoch = evidence.asset_epoch;
    result.asset_digest = evidence.asset_digest;
    result.explicit_relocalization_sequence =
      evidence.explicit_relocalization_sequence;
  }
  return result;
}

}  // namespace robot_floor_manager
