#include <gtest/gtest.h>

#include "robot_nav_config/elevator_scoped_execution_session.hpp"

namespace robot_nav_config {
namespace {

TEST(ElevatorScopedExecutionSession, NewSessionRequiresAFullControllerReset) {
  ElevatorScopedExecutionSession session;

  const auto transition = session.begin("transaction-1:7");

  EXPECT_TRUE(transition.accepted);
  EXPECT_TRUE(transition.reset_required);
  EXPECT_EQ(session.active_session_id(), "transaction-1:7");
}

TEST(ElevatorScopedExecutionSession,
     DuplicateBeginWithinOneActionPreservesMotionState) {
  ElevatorScopedExecutionSession session;
  ASSERT_TRUE(session.begin("transaction-1:7").accepted);

  const auto transition = session.begin("transaction-1:7");

  EXPECT_TRUE(transition.accepted);
  EXPECT_FALSE(transition.reset_required);
  EXPECT_EQ(session.active_session_id(), "transaction-1:7");
}

TEST(ElevatorScopedExecutionSession,
     ExternalAbortThenSameGoalRetryStillStartsANewSession) {
  ElevatorScopedExecutionSession session;
  ASSERT_TRUE(session.begin("transaction-1:7").accepted);
  ASSERT_TRUE(session.end("transaction-1:7").accepted);

  const auto transition = session.begin("transaction-2:7");

  EXPECT_TRUE(transition.accepted);
  EXPECT_TRUE(transition.reset_required);
  EXPECT_EQ(session.active_session_id(), "transaction-2:7");
}

TEST(ElevatorScopedExecutionSession,
     NewSessionResetsEvenWhenPreviousEndWasMissed) {
  ElevatorScopedExecutionSession session;
  ASSERT_TRUE(session.begin("transaction-1:7").accepted);

  const auto transition = session.begin("transaction-2:7");

  EXPECT_TRUE(transition.accepted);
  EXPECT_TRUE(transition.reset_required);
  EXPECT_EQ(session.active_session_id(), "transaction-2:7");
}

TEST(ElevatorScopedExecutionSession, StaleEndCannotClearTheNewerSession) {
  ElevatorScopedExecutionSession session;
  ASSERT_TRUE(session.begin("transaction-1:7").accepted);
  ASSERT_TRUE(session.begin("transaction-2:7").accepted);

  const auto transition = session.end("transaction-1:7");

  EXPECT_FALSE(transition.accepted);
  EXPECT_FALSE(transition.reset_required);
  EXPECT_EQ(session.active_session_id(), "transaction-2:7");
}

TEST(ElevatorScopedExecutionSession, MatchingEndIsIdempotent) {
  ElevatorScopedExecutionSession session;
  ASSERT_TRUE(session.begin("transaction-1:7").accepted);
  ASSERT_TRUE(session.end("transaction-1:7").accepted);

  const auto duplicate = session.end("transaction-1:7");

  EXPECT_TRUE(duplicate.accepted);
  EXPECT_FALSE(duplicate.reset_required);
  EXPECT_FALSE(session.has_active_session());
}

TEST(ElevatorScopedExecutionSession, EmptyIdentityFailsClosed) {
  ElevatorScopedExecutionSession session;

  EXPECT_FALSE(session.begin("").accepted);
  EXPECT_FALSE(session.end("").accepted);
  EXPECT_FALSE(session.has_active_session());
}

TEST(ElevatorScopedExecutionSession, LifecycleResetForgetsActiveAndEndedIds) {
  ElevatorScopedExecutionSession session;
  ASSERT_TRUE(session.begin("transaction-1:7").accepted);

  session.reset();

  EXPECT_FALSE(session.has_active_session());
  const auto end_after_reset = session.end("transaction-1:7");
  EXPECT_FALSE(end_after_reset.accepted);
}

}  // namespace
}  // namespace robot_nav_config
