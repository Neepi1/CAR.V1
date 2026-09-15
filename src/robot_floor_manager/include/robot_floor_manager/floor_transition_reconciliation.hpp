#pragma once

#include <string>

namespace robot_floor_manager
{

enum class FloorTransitionCleanupAction
{
  kReleaseUnchangedSource,
  kRestoreSource,
  kRetainSafety,
};

// Chooses cleanup, not permission to move. Restoration still needs the bridge's
// exact source proof, durable context and acknowledged owner-scoped releases.
FloorTransitionCleanupAction select_floor_transition_cleanup(
  bool begin_established,
  bool begin_outcome_unknown,
  bool target_effect_dispatched);

struct ExplicitLocalizationReconciliation
{
  bool success{false};
  std::string code;
  std::string detail;
};

struct ExplicitLocalizationFailureClassification
{
  bool retryable{false};
  std::string failure_code;
};

// Classifies only failures that prove Isaac was not dispatched. Once a trigger
// transaction reaches dispatch, the caller must reconcile exact target
// evidence and must not create another trigger that can chase the prior result.
ExplicitLocalizationFailureClassification classify_explicit_localization_failure(
  const std::string & trigger_error);

// Resolves the transport outcome against generation-fenced target evidence.
// The RPC response is not the source of truth: an exact new target sequence
// proves completion even when the response was delayed or lost.
ExplicitLocalizationReconciliation reconcile_explicit_localization(
  bool trigger_call_succeeded,
  bool exact_target_evidence_proven,
  bool canceled,
  const std::string & trigger_error);

}  // namespace robot_floor_manager
