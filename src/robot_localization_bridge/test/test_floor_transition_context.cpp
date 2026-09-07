#include <gtest/gtest.h>

#include "robot_localization_bridge/floor_transition_context.hpp"

namespace robot_localization_bridge
{
namespace
{

FloorTransitionIdentity identity()
{
  return {
    "elevator-tx-17",
    "B10",
    "F2",
    "map_f2",
    23U,
    "sha256:0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
  };
}

FloorTransitionCommitEvidence ready_commit()
{
  FloorTransitionCommitEvidence evidence;
  evidence.correction_pause_released = true;
  evidence.explicit_relocalization_sequence = 8U;
  evidence.map_odom_valid = true;
  evidence.correction_active = false;
  evidence.current_sequence = 31U;
  evidence.target_sequence = 31U;
  evidence.last_published_sequence = 31U;
  return evidence;
}

TEST(FloorTransitionContext, BeginRequiresExactFloorPauseBeforeInvalidatingContext)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(identity()).accepted);

  const auto rejected = context.begin(identity(), identity(), false, 7U, 1U);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_TRUE(rejected.state.runtime_context_valid);
  EXPECT_FALSE(rejected.state.transition_active);

  const auto accepted = context.begin(identity(), identity(), true, 7U, 2U);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_FALSE(accepted.state.runtime_context_valid);
  EXPECT_TRUE(accepted.state.transition_active);
  EXPECT_EQ(accepted.state.begin_explicit_relocalization_sequence, 7U);
}

TEST(FloorTransitionContext, BeginIsIdempotentOnlyForExactIdentity)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(identity()).accepted);
  ASSERT_TRUE(context.begin(identity(), identity(), true, 7U, 1U).accepted);

  const auto duplicate = context.begin(identity(), identity(), true, 7U, 2U);
  EXPECT_TRUE(duplicate.accepted);
  EXPECT_TRUE(duplicate.idempotent);

  auto conflicting = identity();
  conflicting.map_id = "map_other";
  const auto rejected =
    context.begin(conflicting, identity(), true, 7U, 3U);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.code, FloorTransitionDecisionCode::kConflict);
}

TEST(FloorTransitionContext, ActiveTransitionAdmitsOnlyUnpausedExplicitCandidate)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(identity()).accepted);
  ASSERT_TRUE(context.begin(identity(), identity(), true, 7U, 1U).accepted);

  EXPECT_FALSE(context.candidate_allowed(false, false));
  EXPECT_FALSE(context.candidate_allowed(true, true));
  EXPECT_TRUE(context.candidate_allowed(true, false));
}

TEST(FloorTransitionContext, CommitRequiresNewExplicitSequenceAndSettledPublishedTransform)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(identity()).accepted);
  ASSERT_TRUE(context.begin(identity(), identity(), true, 7U, 1U).accepted);

  auto evidence = ready_commit();
  evidence.explicit_relocalization_sequence = 7U;
  EXPECT_FALSE(context.commit(identity(), evidence, 2U).accepted);
  EXPECT_TRUE(context.snapshot().transition_active);

  evidence = ready_commit();
  evidence.last_published_sequence = 30U;
  EXPECT_FALSE(context.commit(identity(), evidence, 3U).accepted);
  EXPECT_TRUE(context.snapshot().transition_active);

  const auto committed = context.commit(identity(), ready_commit(), 4U);
  EXPECT_TRUE(committed.accepted);
  EXPECT_TRUE(committed.state.runtime_context_valid);
  EXPECT_FALSE(committed.state.transition_active);
  EXPECT_FALSE(committed.state.failed_locked);
  EXPECT_EQ(committed.state.active.map_id, "map_f2");
  EXPECT_EQ(committed.state.accepted_explicit_relocalization_sequence, 8U);
}

TEST(FloorTransitionContext, AbortLocksInvalidContextAndCannotBeBypassed)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.seed_active_source(identity()).accepted);
  ASSERT_TRUE(context.begin(identity(), identity(), true, 7U, 1U).accepted);

  const auto aborted = context.abort(identity(), 2U);
  EXPECT_TRUE(aborted.accepted);
  EXPECT_FALSE(aborted.state.runtime_context_valid);
  EXPECT_TRUE(aborted.state.failed_locked);
  EXPECT_FALSE(context.candidate_allowed(true, false));

  const auto duplicate = context.abort(identity(), 3U);
  EXPECT_TRUE(duplicate.accepted);
  EXPECT_TRUE(duplicate.idempotent);

  auto replacement = identity();
  replacement.transaction_id = "elevator-tx-18";
  EXPECT_FALSE(
    context.begin(replacement, identity(), true, 8U, 4U).accepted);
}

TEST(FloorTransitionContext, BeginRejectsUnseededSourceWithoutChangingContext)
{
  FloorTransitionContext context;

  const auto rejected = context.begin(
    identity(), identity(), true, 7U, 1U);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(
    rejected.code,
    FloorTransitionDecisionCode::kPreMutationUnproven);
  EXPECT_TRUE(rejected.state.runtime_context_valid);
  EXPECT_FALSE(rejected.state.transition_active);
  EXPECT_TRUE(rejected.state.active.map_id.empty());
}

FloorTransitionIdentity source_identity()
{
  auto source = identity();
  source.transaction_id = "source-context";
  source.floor_id = "F1";
  source.map_id = "map_f1";
  source.asset_epoch = 22U;
  source.asset_digest =
    "sha256:abcdef0123456789abcdef0123456789"
    "abcdef0123456789abcdef0123456789";
  return source;
}

FloorTransitionContext seeded_source_context()
{
  FloorTransitionContext context;
  const auto seeded = context.seed_active_source(source_identity());
  EXPECT_TRUE(seeded.accepted);
  EXPECT_FALSE(seeded.idempotent);
  EXPECT_TRUE(seeded.state.runtime_context_valid);
  EXPECT_EQ(seeded.state.active.map_id, "map_f1");
  return context;
}

TEST(FloorTransitionContext, PreMutationAbortRestoresAppliedBeginWithHigherSequence)
{
  auto context = seeded_source_context();
  const auto target = identity();
  ASSERT_TRUE(
    context.begin(target, source_identity(), true, 6U, 3U).accepted);

  FloorTransitionPreMutationAbortEvidence evidence;
  evidence.source = source_identity();
  evidence.source_assets_unchanged = true;
  const auto aborted =
    context.abort_pre_mutation(target, evidence, 4U);

  EXPECT_TRUE(aborted.accepted);
  EXPECT_TRUE(aborted.state.runtime_context_valid);
  EXPECT_FALSE(aborted.state.transition_active);
  EXPECT_FALSE(aborted.state.failed_locked);
  EXPECT_EQ(aborted.state.active.map_id, "map_f1");
}

TEST(FloorTransitionContext, BeginRejectsSourceMismatchBeforeInvalidation)
{
  auto context = seeded_source_context();
  auto wrong_source = source_identity();
  wrong_source.map_id = "wrong_source";

  const auto rejected =
    context.begin(identity(), wrong_source, true, 6U, 3U);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(
    rejected.code,
    FloorTransitionDecisionCode::kIdentityMismatch);
  EXPECT_TRUE(rejected.state.runtime_context_valid);
  EXPECT_FALSE(rejected.state.transition_active);
  EXPECT_EQ(rejected.state.active.map_id, "map_f1");
}

TEST(FloorTransitionContext, PreMutationAbortFencesBeginThatArrivesLater)
{
  auto context = seeded_source_context();
  const auto target = identity();
  FloorTransitionPreMutationAbortEvidence evidence;
  evidence.source = source_identity();
  evidence.source_assets_unchanged = true;

  const auto fenced =
    context.abort_pre_mutation(target, evidence, 4U);
  ASSERT_TRUE(fenced.accepted);
  ASSERT_TRUE(fenced.idempotent);

  const auto delayed_begin =
    context.begin(target, source_identity(), true, 6U, 3U);
  EXPECT_FALSE(delayed_begin.accepted);
  EXPECT_EQ(delayed_begin.code, FloorTransitionDecisionCode::kStaleCommand);
  EXPECT_EQ(delayed_begin.applied_sequence, 4U);
  EXPECT_TRUE(delayed_begin.state.runtime_context_valid);
  EXPECT_FALSE(delayed_begin.state.transition_active);
  EXPECT_EQ(delayed_begin.state.active.map_id, "map_f1");
}

TEST(FloorTransitionContext, PreMutationAbortRefusesChangedSourceAssets)
{
  auto context = seeded_source_context();
  ASSERT_TRUE(
    context.begin(identity(), source_identity(), true, 6U, 3U).accepted);
  FloorTransitionPreMutationAbortEvidence evidence;
  evidence.source = source_identity();
  evidence.source_assets_unchanged = false;

  const auto rejected =
    context.abort_pre_mutation(identity(), evidence, 4U);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(
    rejected.code,
    FloorTransitionDecisionCode::kPreMutationUnproven);
  EXPECT_FALSE(rejected.state.runtime_context_valid);
  EXPECT_TRUE(rejected.state.transition_active);
}

TEST(FloorTransitionContext, ActiveSourceSeedCannotRebindOrClearRecoveryLock)
{
  auto context = seeded_source_context();
  auto other = source_identity();
  other.map_id = "map_other";
  EXPECT_FALSE(context.seed_active_source(other).accepted);

  ASSERT_TRUE(
    context.begin(identity(), source_identity(), true, 6U, 3U).accepted);
  ASSERT_TRUE(context.abort(identity(), 4U).accepted);
  EXPECT_TRUE(context.snapshot().failed_locked);
  EXPECT_FALSE(context.seed_active_source(source_identity()).accepted);
  EXPECT_TRUE(context.snapshot().failed_locked);
  EXPECT_FALSE(context.snapshot().runtime_context_valid);
}

}  // namespace
}  // namespace robot_localization_bridge
