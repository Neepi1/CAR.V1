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

  const auto rejected = context.begin(identity(), false, 7U);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_TRUE(rejected.state.runtime_context_valid);
  EXPECT_FALSE(rejected.state.transition_active);

  const auto accepted = context.begin(identity(), true, 7U);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_FALSE(accepted.state.runtime_context_valid);
  EXPECT_TRUE(accepted.state.transition_active);
  EXPECT_EQ(accepted.state.begin_explicit_relocalization_sequence, 7U);
}

TEST(FloorTransitionContext, BeginIsIdempotentOnlyForExactIdentity)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.begin(identity(), true, 7U).accepted);

  const auto duplicate = context.begin(identity(), true, 7U);
  EXPECT_TRUE(duplicate.accepted);
  EXPECT_TRUE(duplicate.idempotent);

  auto conflicting = identity();
  conflicting.map_id = "map_other";
  const auto rejected = context.begin(conflicting, true, 7U);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.code, FloorTransitionDecisionCode::kConflict);
}

TEST(FloorTransitionContext, ActiveTransitionAdmitsOnlyUnpausedExplicitCandidate)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.begin(identity(), true, 7U).accepted);

  EXPECT_FALSE(context.candidate_allowed(false, false));
  EXPECT_FALSE(context.candidate_allowed(true, true));
  EXPECT_TRUE(context.candidate_allowed(true, false));
}

TEST(FloorTransitionContext, CommitRequiresNewExplicitSequenceAndSettledPublishedTransform)
{
  FloorTransitionContext context;
  ASSERT_TRUE(context.begin(identity(), true, 7U).accepted);

  auto evidence = ready_commit();
  evidence.explicit_relocalization_sequence = 7U;
  EXPECT_FALSE(context.commit(identity(), evidence).accepted);
  EXPECT_TRUE(context.snapshot().transition_active);

  evidence = ready_commit();
  evidence.last_published_sequence = 30U;
  EXPECT_FALSE(context.commit(identity(), evidence).accepted);
  EXPECT_TRUE(context.snapshot().transition_active);

  const auto committed = context.commit(identity(), ready_commit());
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
  ASSERT_TRUE(context.begin(identity(), true, 7U).accepted);

  const auto aborted = context.abort(identity());
  EXPECT_TRUE(aborted.accepted);
  EXPECT_FALSE(aborted.state.runtime_context_valid);
  EXPECT_TRUE(aborted.state.failed_locked);
  EXPECT_FALSE(context.candidate_allowed(true, false));

  const auto duplicate = context.abort(identity());
  EXPECT_TRUE(duplicate.accepted);
  EXPECT_TRUE(duplicate.idempotent);

  auto replacement = identity();
  replacement.transaction_id = "elevator-tx-18";
  EXPECT_FALSE(context.begin(replacement, true, 8U).accepted);
}

}  // namespace
}  // namespace robot_localization_bridge
