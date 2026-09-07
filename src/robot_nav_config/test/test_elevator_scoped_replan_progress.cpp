#include <cstdint>

#include <gtest/gtest.h>

#include "robot_nav_config/elevator_scoped_replan_progress.hpp"

namespace {

using robot_nav_config::ElevatorScopedReplanProgress;

TEST(ElevatorScopedReplanProgress,
     ChangedEvidenceCanSupersedeAnUnfinishedRouteRevision) {
  ElevatorScopedReplanProgress progress;
  constexpr std::uint64_t first_blockage = 101U;
  constexpr std::uint64_t blocked_revision = 202U;

  ASSERT_TRUE(progress.should_attempt_replan(first_blockage));
  progress.observe_replan_attempt(first_blockage);
  ASSERT_TRUE(progress.try_commit_route_revision(first_blockage));

  // The first revision has not reached any historical rejoin boundary.  A
  // different live blockage is nevertheless fresh evidence and must be able
  // to replace that revision immediately.
  EXPECT_TRUE(progress.should_attempt_replan(blocked_revision));
  progress.observe_replan_attempt(blocked_revision);
  EXPECT_TRUE(progress.try_commit_route_revision(blocked_revision));
  EXPECT_EQ(progress.active_route_revision_signature(), blocked_revision);
}

TEST(ElevatorScopedReplanProgress,
     IdenticalEvidenceIsNotSearchedAgainAfterARevisionIsCommitted) {
  ElevatorScopedReplanProgress progress;
  constexpr std::uint64_t signature = 303U;

  ASSERT_TRUE(progress.should_attempt_replan(signature));
  progress.observe_replan_attempt(signature);
  ASSERT_TRUE(progress.try_commit_route_revision(signature));

  EXPECT_FALSE(progress.should_attempt_replan(signature));
  EXPECT_FALSE(progress.try_commit_route_revision(signature));
}

TEST(ElevatorScopedReplanProgress,
     CostmapOscillationDoesNotReplayAnOlderAttempt) {
  ElevatorScopedReplanProgress progress;
  constexpr std::uint64_t snapshot_a = 404U;
  constexpr std::uint64_t snapshot_b = 505U;

  progress.observe_replan_attempt(snapshot_a);
  ASSERT_TRUE(progress.try_commit_route_revision(snapshot_a));
  progress.observe_replan_attempt(snapshot_b);
  ASSERT_TRUE(progress.try_commit_route_revision(snapshot_b));

  EXPECT_FALSE(progress.should_attempt_replan(snapshot_a));
  EXPECT_FALSE(progress.should_attempt_replan(snapshot_b));
}

TEST(ElevatorScopedReplanProgress,
     TransientSubmissionFailureCanForgetOnlyItsOwnAttempt) {
  ElevatorScopedReplanProgress progress;
  constexpr std::uint64_t transient = 606U;
  constexpr std::uint64_t retained = 707U;

  progress.observe_replan_attempt(transient);
  progress.observe_replan_attempt(retained);
  progress.forget_replan_attempt(transient);

  EXPECT_TRUE(progress.should_attempt_replan(transient));
  EXPECT_FALSE(progress.should_attempt_replan(retained));
}

TEST(ElevatorScopedReplanProgress, NewNav2PlanClearsActionLocalEvidence) {
  ElevatorScopedReplanProgress progress;
  constexpr std::uint64_t signature = 808U;

  progress.observe_replan_attempt(signature);
  ASSERT_TRUE(progress.try_commit_route_revision(signature));
  progress.reset();

  EXPECT_TRUE(progress.should_attempt_replan(signature));
  EXPECT_FALSE(progress.active_route_revision_signature().has_value());
}

} // namespace
