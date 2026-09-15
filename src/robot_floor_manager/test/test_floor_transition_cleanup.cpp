#include <gtest/gtest.h>

#include "robot_floor_manager/floor_transition_reconciliation.hpp"
#include "robot_localization_bridge/floor_transition_context.hpp"

namespace robot_floor_manager
{
namespace
{

using robot_localization_bridge::FloorTransitionContext;
using robot_localization_bridge::FloorTransitionContextDecision;
using robot_localization_bridge::FloorTransitionCommitEvidence;
using robot_localization_bridge::FloorTransitionDecisionCode;
using robot_localization_bridge::FloorTransitionIdentity;
using robot_localization_bridge::FloorTransitionPreMutationAbortEvidence;

FloorTransitionIdentity source_identity()
{
  return {
    "source-context", "B10", "F1", "map_f1", 22U,
    "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
  };
}

FloorTransitionIdentity target_identity()
{
  return {
    "floor-switch-23", "B10", "F2", "map_f2", 23U,
    "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
  };
}

FloorTransitionPreMutationAbortEvidence unchanged_source()
{
  return {source_identity(), true};
}

// This adapter replaces only service transport: cleanup is selected by the
// production policy and interpreted by the real bridge state machine. It does
// not fabricate a successful rollback or a runtime_context_valid result.
FloorTransitionContextDecision send_cleanup(
  const FloorTransitionCleanupAction action,
  FloorTransitionContext & context,
  const FloorTransitionIdentity & target,
  const FloorTransitionPreMutationAbortEvidence & evidence,
  const std::uint64_t sequence)
{
  if (action == FloorTransitionCleanupAction::kRestoreSource) {
    return context.abort_pre_mutation(target, evidence, sequence);
  }
  if (action == FloorTransitionCleanupAction::kRetainSafety) {
    return context.abort(target, sequence);
  }
  ADD_FAILURE() << "unchanged-source release must not send a bridge command";
  return {};
}

TEST(FloorTransitionCleanup, AppliedBeginWithoutTargetDispatchRestoresExactSource)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(source_identity()).accepted);
  ASSERT_TRUE(context.begin(target_identity(), source_identity(), true, 7U, 1U).accepted);
  ASSERT_FALSE(context.snapshot().runtime_context_valid);

  const auto action = select_floor_transition_cleanup(true, false, false);
  const auto restored = send_cleanup(action, context, target_identity(), unchanged_source(), 2U);

  ASSERT_TRUE(restored.accepted) << restored.message;
  EXPECT_TRUE(restored.state.runtime_context_valid);
  EXPECT_FALSE(restored.state.transition_active);
  EXPECT_FALSE(restored.state.failed_locked);
  EXPECT_FALSE(restored.state.recovery_required);
  EXPECT_EQ(restored.state.active.map_id, source_identity().map_id);
  EXPECT_EQ(restored.state.active.asset_digest, source_identity().asset_digest);
  EXPECT_TRUE(restored.state.pending.transaction_id.empty());
}

TEST(FloorTransitionCleanup, NoBeginAndNoTargetDispatchLeavesExactSourceUnchanged)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(source_identity()).accepted);

  EXPECT_EQ(
    select_floor_transition_cleanup(false, false, false),
    FloorTransitionCleanupAction::kReleaseUnchangedSource);

  const auto source = context.snapshot();
  EXPECT_TRUE(source.runtime_context_valid);
  EXPECT_FALSE(source.transition_active);
  EXPECT_FALSE(source.failed_locked);
  EXPECT_EQ(source.active.map_id, source_identity().map_id);
  EXPECT_EQ(source.active.asset_epoch, source_identity().asset_epoch);
}

TEST(FloorTransitionCleanup, UnknownBeginIsFencedBeforeItsDelayedArrival)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(source_identity()).accepted);

  // The caller sent BEGIN sequence 1 but has not received a response. Here
  // transport delivers cleanup sequence 2 before delivering that BEGIN.
  const auto action = select_floor_transition_cleanup(false, true, false);
  const auto restored = send_cleanup(action, context, target_identity(), unchanged_source(), 2U);
  ASSERT_TRUE(restored.accepted) << restored.message;
  EXPECT_TRUE(restored.idempotent);
  EXPECT_TRUE(restored.state.runtime_context_valid);

  const auto delayed = context.begin(target_identity(), source_identity(), true, 7U, 1U);
  EXPECT_FALSE(delayed.accepted);
  EXPECT_EQ(delayed.code, FloorTransitionDecisionCode::kStaleCommand);
  EXPECT_EQ(delayed.applied_sequence, 2U);
  EXPECT_TRUE(delayed.state.runtime_context_valid);
  EXPECT_FALSE(delayed.state.transition_active);
  EXPECT_EQ(delayed.state.active.map_id, source_identity().map_id);
}

TEST(FloorTransitionCleanup, UnknownBeginAlreadyAppliedCanRestoreWithoutTargetDispatch)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(source_identity()).accepted);
  ASSERT_TRUE(context.begin(target_identity(), source_identity(), true, 7U, 1U).accepted);

  // The same unknown RPC outcome can mean BEGIN was applied but its response
  // was lost. Cleanup must use the protocol, not assume no BEGIN happened.
  const auto action = select_floor_transition_cleanup(false, true, false);
  const auto restored = send_cleanup(action, context, target_identity(), unchanged_source(), 2U);

  ASSERT_TRUE(restored.accepted) << restored.message;
  EXPECT_FALSE(restored.idempotent);
  EXPECT_TRUE(restored.state.runtime_context_valid);
  EXPECT_FALSE(restored.state.transition_active);
  EXPECT_FALSE(restored.state.recovery_required);
  EXPECT_EQ(restored.state.active.asset_digest, source_identity().asset_digest);
}

TEST(FloorTransitionCleanup, TargetDispatchAbortsWithoutInventingSourceOrPermanentBridgeLock)
{
  // A timeout, cancellation or failed response cannot retract a dispatched
  // target request. These are policy inputs, not simulated LoadMap RPCs.
  for (const bool begin_established : {false, true}) {
    for (const bool begin_outcome_unknown : {false, true}) {
      SCOPED_TRACE(
        ::testing::Message() << "begin_established=" << begin_established <<
          ", begin_outcome_unknown=" << begin_outcome_unknown);
      FloorTransitionContext context;
      ASSERT_TRUE(context.seed_active_source(source_identity()).accepted);
      ASSERT_TRUE(context.begin(target_identity(), source_identity(), true, 7U, 1U).accepted);

      const auto action = select_floor_transition_cleanup(
        begin_established, begin_outcome_unknown, true);
      ASSERT_EQ(action, FloorTransitionCleanupAction::kRetainSafety);
      const auto locked = send_cleanup(action, context, target_identity(), unchanged_source(), 2U);

      ASSERT_TRUE(locked.accepted) << locked.message;
      EXPECT_FALSE(locked.state.runtime_context_valid);
      EXPECT_FALSE(locked.state.failed_locked);
      EXPECT_FALSE(locked.state.transition_active);
      EXPECT_FALSE(locked.state.recovery_required);
      EXPECT_FALSE(context.candidate_allowed(true, false));

      // Terminating the bridge transaction does not prove source restoration
      // or permission to move. The runtime adapter separately settles writes
      // before releasing its resources and admitting the next switch.
      const auto bypass = context.abort_pre_mutation(target_identity(), unchanged_source(), 3U);
      EXPECT_FALSE(bypass.accepted);
      EXPECT_EQ(bypass.code, FloorTransitionDecisionCode::kPreMutationUnproven);
      EXPECT_FALSE(bypass.state.failed_locked);
      EXPECT_FALSE(bypass.state.runtime_context_valid);
    }
  }
}

TEST(FloorTransitionCleanup, RestoreSelectionDoesNotReplaceSourceEvidence)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(source_identity()).accepted);
  ASSERT_TRUE(context.begin(target_identity(), source_identity(), true, 7U, 1U).accepted);
  auto unproven_source = unchanged_source();
  unproven_source.source_assets_unchanged = false;

  const auto action = select_floor_transition_cleanup(true, false, false);
  ASSERT_EQ(action, FloorTransitionCleanupAction::kRestoreSource);
  const auto rejected = send_cleanup(action, context, target_identity(), unproven_source, 2U);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.code, FloorTransitionDecisionCode::kPreMutationUnproven);
  EXPECT_FALSE(rejected.state.runtime_context_valid);
  EXPECT_TRUE(rejected.state.transition_active);
  EXPECT_TRUE(rejected.state.recovery_required);
  EXPECT_EQ(rejected.state.pending.transaction_id, target_identity().transaction_id);
}

TEST(FloorTransitionCleanup, UnknownSourceCannotBeInventedForRecovery)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.begin(target_identity(), {}, true, 7U, 1U).accepted);

  const auto action = select_floor_transition_cleanup(true, false, false);
  const auto rejected = send_cleanup(action, context, target_identity(), unchanged_source(), 2U);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.code, FloorTransitionDecisionCode::kIdentityMismatch);
  EXPECT_FALSE(rejected.state.runtime_context_valid);
  EXPECT_TRUE(rejected.state.active.map_id.empty());
  EXPECT_TRUE(rejected.state.transition_active);
  EXPECT_TRUE(rejected.state.recovery_required);
}

TEST(FloorTransitionCleanup, WrongTransactionCannotRestoreAnotherPendingTransition)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(source_identity()).accepted);
  ASSERT_TRUE(context.begin(target_identity(), source_identity(), true, 7U, 1U).accepted);
  auto wrong_transaction = target_identity();
  wrong_transaction.transaction_id = "other-floor-switch";

  const auto action = select_floor_transition_cleanup(true, false, false);
  const auto rejected = send_cleanup(action, context, wrong_transaction, unchanged_source(), 2U);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.code, FloorTransitionDecisionCode::kIdentityMismatch);
  EXPECT_FALSE(rejected.state.runtime_context_valid);
  EXPECT_TRUE(rejected.state.transition_active);
  EXPECT_EQ(rejected.state.pending.transaction_id, target_identity().transaction_id);

  const auto correct = send_cleanup(action, context, target_identity(), unchanged_source(), 2U);
  EXPECT_TRUE(correct.accepted) << correct.message;
  EXPECT_TRUE(correct.state.runtime_context_valid);
}

TEST(FloorTransitionCleanup, WrongSourceDigestCannotRestoreContext)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(source_identity()).accepted);
  ASSERT_TRUE(context.begin(target_identity(), source_identity(), true, 7U, 1U).accepted);
  auto wrong_source = unchanged_source();
  wrong_source.source.asset_digest = target_identity().asset_digest;

  const auto action = select_floor_transition_cleanup(true, false, false);
  const auto rejected = send_cleanup(action, context, target_identity(), wrong_source, 2U);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.code, FloorTransitionDecisionCode::kIdentityMismatch);
  EXPECT_FALSE(rejected.state.runtime_context_valid);
  EXPECT_TRUE(rejected.state.recovery_required);
  EXPECT_EQ(rejected.state.active.asset_digest, source_identity().asset_digest);
}

TEST(FloorTransitionCleanup, RestorationStillRequiresAHigherCommandSequence)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(source_identity()).accepted);
  ASSERT_TRUE(context.begin(target_identity(), source_identity(), true, 7U, 2U).accepted);

  const auto action = select_floor_transition_cleanup(true, false, false);
  const auto stale = send_cleanup(action, context, target_identity(), unchanged_source(), 1U);

  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.code, FloorTransitionDecisionCode::kStaleCommand);
  EXPECT_FALSE(stale.state.runtime_context_valid);
  EXPECT_TRUE(stale.state.transition_active);

  const auto fresh = send_cleanup(action, context, target_identity(), unchanged_source(), 3U);
  EXPECT_TRUE(fresh.accepted) << fresh.message;
  EXPECT_TRUE(fresh.state.runtime_context_valid);
}

TEST(FloorTransitionCleanup, SuccessfulTargetCommitKeepsItsExistingEvidenceContract)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(source_identity()).accepted);
  ASSERT_TRUE(context.begin(target_identity(), source_identity(), true, 7U, 1U).accepted);

  FloorTransitionCommitEvidence evidence;
  evidence.explicit_relocalization_sequence = 8U;
  evidence.map_odom_valid = true;
  evidence.current_sequence = 31U;
  evidence.target_sequence = 31U;
  evidence.last_published_sequence = 31U;
  const auto paused = context.commit(target_identity(), evidence, 2U);
  EXPECT_FALSE(paused.accepted);
  EXPECT_EQ(paused.code, FloorTransitionDecisionCode::kPauseUnproven);

  evidence.correction_pause_released = true;
  const auto committed = context.commit(target_identity(), evidence, 3U);
  ASSERT_TRUE(committed.accepted) << committed.message;
  EXPECT_TRUE(committed.state.runtime_context_valid);
  EXPECT_FALSE(committed.state.transition_active);
  EXPECT_FALSE(committed.state.failed_locked);
  EXPECT_FALSE(committed.state.recovery_required);
  EXPECT_EQ(committed.state.active.map_id, target_identity().map_id);
  EXPECT_EQ(committed.state.active.asset_digest, target_identity().asset_digest);
  EXPECT_EQ(committed.state.accepted_explicit_relocalization_sequence, 8U);
}

}  // namespace
}  // namespace robot_floor_manager
