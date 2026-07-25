#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "robot_elevator_manager/elevator_fsm.hpp"

namespace robot_elevator_manager
{
namespace
{

constexpr const char * kTransactionId = "elevator-tx-42";

FloorElevatorTopology make_floor(const std::string & floor_id, const std::string & prefix)
{
  return FloorElevatorTopology{
    floor_id,
    floor_id + "_map",
    {
      {PoseRole::kHallCall, prefix + "_hall_call"},
      {PoseRole::kHallWait, prefix + "_hall_wait"},
      {PoseRole::kDoorway, prefix + "_doorway"},
      {PoseRole::kCabin, prefix + "_cabin"},
      {PoseRole::kExit, prefix + "_exit"},
    },
    {{0.0, -0.6}, {0.0, 0.6}, {1.0, 0.0}, 0.05},
  };
}

ElevatorRoute make_route()
{
  return ElevatorRoute{
    "elevator_west",
    "building_1",
    make_floor("F1", "f1_west"),
    make_floor("F2", "f2_west"),
  };
}

ElevatorEvent success(const ElevatorFsmOutput & output)
{
  return ElevatorEvent{
    ElevatorEventKind::kEffectSucceeded,
    "",
    output.effect.sequence,
    output.effect.transaction_id,
  };
}

struct ExpectedEffect
{
  ElevatorEffectKind kind;
  std::string pose_id;
  std::string floor_id;
  std::string mode;
  std::string map_id{};
};

void expect_effect(
  const ElevatorFsmOutput & output,
  const ExpectedEffect & expected,
  const std::uint64_t previous_sequence)
{
  EXPECT_EQ(output.effect.kind, expected.kind);
  EXPECT_EQ(output.effect.transaction_id, kTransactionId);
  EXPECT_EQ(output.effect.pose_id, expected.pose_id);
  EXPECT_EQ(output.effect.floor_id, expected.floor_id);
  EXPECT_EQ(output.effect.mode, expected.mode);
  if (!expected.map_id.empty()) {
    EXPECT_EQ(output.effect.map_id, expected.map_id);
  }
  EXPECT_GT(output.effect.sequence, previous_sequence);
}

TEST(ElevatorFsm, EmitsOneOrderedMockPortEffectPerAcceptedEvent)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});

  auto output = fsm.start(kTransactionId, make_route());
  ASSERT_TRUE(output.accepted);
  expect_effect(
    output,
    {
      ElevatorEffectKind::kNavigateToPose,
      "f1_west_hall_call",
      "F1",
      "",
      "F1_map",
    },
    0U);

  const std::vector<ExpectedEffect> effects_before_inside{
    {ElevatorEffectKind::kAcquireSafetyHold, "", "F1", ""},
    {ElevatorEffectKind::kAcquireExecutionLease, "", "F1", ""},
    {ElevatorEffectKind::kMockPressCallButton, "", "F1", ""},
    {ElevatorEffectKind::kSetOperatingMode, "", "F1", "ELEVATOR_WAIT"},
    {ElevatorEffectKind::kReleaseSafetyHold, "", "F1", ""},
    {ElevatorEffectKind::kNavigateToPose, "f1_west_hall_wait", "F1", ""},
    {ElevatorEffectKind::kMockWaitDoorOpen, "", "F1", ""},
    {ElevatorEffectKind::kSetOperatingMode, "", "F1", "DOORWAY"},
    {ElevatorEffectKind::kNavigateToPose, "f1_west_doorway", "F1", ""},
    {ElevatorEffectKind::kNavigateToPose, "f1_west_cabin", "F1", ""},
    {ElevatorEffectKind::kVerifyFootprintInside, "", "F1", ""},
  };
  for (const auto & expected : effects_before_inside) {
    const auto previous_sequence = output.effect.sequence;
    output = fsm.dispatch(success(output));
    ASSERT_TRUE(output.accepted);
    EXPECT_TRUE(output.transitioned);
    expect_effect(output, expected, previous_sequence);
  }

  const auto verify_inside_sequence = output.effect.sequence;
  output = fsm.dispatch(
    {
      ElevatorEventKind::kFootprintInside,
      "",
      output.effect.sequence,
      kTransactionId,
    });
  ASSERT_TRUE(output.accepted);
  expect_effect(
    output,
    {ElevatorEffectKind::kAcquireSafetyHold, "", "F1", ""},
    verify_inside_sequence);

  const std::vector<ExpectedEffect> effects_before_outside{
    {ElevatorEffectKind::kMockPressTargetButton, "", "F2", ""},
    {ElevatorEffectKind::kPauseLocalizationCorrections, "", "F1", ""},
    {ElevatorEffectKind::kSetOperatingMode, "", "F1", "ELEVATOR_RIDE"},
    {ElevatorEffectKind::kMockRide, "", "F2", ""},
    {ElevatorEffectKind::kMockWaitDoorOpen, "", "F2", ""},
    {ElevatorEffectKind::kBeginFloorTransition, "", "F2", "", "F2_map"},
    {ElevatorEffectKind::kResumeLocalizationCorrections, "", "F1", ""},
    {ElevatorEffectKind::kSwitchFloor, "", "F2", "", "F2_map"},
    {ElevatorEffectKind::kVerifyFloorReady, "", "F2", "", "F2_map"},
    {ElevatorEffectKind::kSetOperatingMode, "", "F2", "DOORWAY"},
    {ElevatorEffectKind::kReleaseSafetyHold, "", "F2", ""},
    {ElevatorEffectKind::kNavigateToPose, "f2_west_doorway", "F2", ""},
    {ElevatorEffectKind::kNavigateToPose, "f2_west_exit", "F2", ""},
    {ElevatorEffectKind::kVerifyFootprintOutside, "", "F2", ""},
  };
  for (const auto & expected : effects_before_outside) {
    const auto previous_sequence = output.effect.sequence;
    output = fsm.dispatch(success(output));
    ASSERT_TRUE(output.accepted);
    EXPECT_TRUE(output.transitioned);
    expect_effect(output, expected, previous_sequence);
  }

  const auto verify_outside_sequence = output.effect.sequence;
  output = fsm.dispatch(
    {
      ElevatorEventKind::kFootprintOutside,
      "",
      output.effect.sequence,
      kTransactionId,
    });
  ASSERT_TRUE(output.accepted);
  expect_effect(
    output,
    {ElevatorEffectKind::kAcquireSafetyHold, "", "F2", ""},
    verify_outside_sequence);

  const std::vector<ExpectedEffect> completion_effects{
    {ElevatorEffectKind::kReleaseOperatingMode, "", "F2", ""},
    {ElevatorEffectKind::kReleaseExecutionLease, "", "F2", ""},
    {ElevatorEffectKind::kReleaseSafetyHold, "", "F2", ""},
    {ElevatorEffectKind::kComplete, "", "F2", ""},
  };
  for (const auto & expected : completion_effects) {
    const auto previous_sequence = output.effect.sequence;
    output = fsm.dispatch(success(output));
    ASSERT_TRUE(output.accepted);
    EXPECT_TRUE(output.transitioned);
    expect_effect(output, expected, previous_sequence);
  }

  EXPECT_EQ(fsm.state(), ElevatorState::kComplete);
  EXPECT_FALSE(fsm.locked());
}

TEST(ElevatorFsm, UnexpectedEventFailsClosedAndLocks)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});
  ASSERT_TRUE(fsm.start(kTransactionId, make_route()).accepted);

  const auto output = fsm.dispatch(
    {ElevatorEventKind::kFootprintInside, "out_of_order", 1U, kTransactionId});

  EXPECT_TRUE(output.accepted);
  EXPECT_TRUE(output.transitioned);
  EXPECT_EQ(output.effect.kind, ElevatorEffectKind::kHoldAndCancel);
  EXPECT_EQ(fsm.state(), ElevatorState::kFailureCleanup);
  EXPECT_TRUE(fsm.locked());
}

TEST(ElevatorFsm, PortFailureAndCancellationBothHoldCancelAndLock)
{
  for (const auto kind : {
      ElevatorEventKind::kEffectFailed,
      ElevatorEventKind::kCancelRequested,
    }) {
    ElevatorFsm fsm(ElevatorFsmOptions{true});
    const auto start = fsm.start(kTransactionId, make_route());
    ASSERT_TRUE(start.accepted);
    const ElevatorEvent event{
      kind,
      kind == ElevatorEventKind::kEffectFailed ? "navigation_failed" : "operator_cancel",
      start.effect.sequence,
      kTransactionId,
    };

    const auto output = fsm.dispatch(event);

    EXPECT_TRUE(output.accepted);
    EXPECT_EQ(output.effect.kind, ElevatorEffectKind::kHoldAndCancel);
    EXPECT_EQ(fsm.state(), ElevatorState::kFailureCleanup);
    EXPECT_TRUE(fsm.locked());
  }
}

TEST(ElevatorFsm, StraddlingDuringFullInsideVerificationHoldsAndLocks)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});
  auto output = fsm.start(kTransactionId, make_route());
  ASSERT_TRUE(output.accepted);
  for (int i = 0; i < 11; ++i) {
    output = fsm.dispatch(success(output));
    ASSERT_TRUE(output.accepted);
  }
  ASSERT_EQ(fsm.state(), ElevatorState::kVerifyingInside);

  output = fsm.dispatch(
    {
      ElevatorEventKind::kFootprintStraddling,
      "footprint_crosses_threshold",
      output.effect.sequence,
      kTransactionId,
    });

  EXPECT_TRUE(output.accepted);
  EXPECT_EQ(output.effect.kind, ElevatorEffectKind::kHoldAndCancel);
  EXPECT_TRUE(fsm.locked());
}

TEST(ElevatorFsm, LockedMachineCannotAdvanceOrEmitAnotherEffect)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});
  const auto start = fsm.start(kTransactionId, make_route());
  ASSERT_TRUE(start.accepted);
  const auto cleanup = fsm.dispatch(
    {
      ElevatorEventKind::kEffectFailed,
      "failed",
      start.effect.sequence,
      kTransactionId,
    });
  ASSERT_EQ(cleanup.effect.kind, ElevatorEffectKind::kHoldAndCancel);

  const auto output = fsm.dispatch(
    {
      ElevatorEventKind::kEffectSucceeded,
      "",
      cleanup.effect.sequence,
      kTransactionId,
    });

  EXPECT_TRUE(output.accepted);
  EXPECT_TRUE(output.transitioned);
  EXPECT_EQ(output.effect.kind, ElevatorEffectKind::kNone);
  EXPECT_EQ(fsm.state(), ElevatorState::kLocked);

  const auto after_locked = fsm.dispatch(success(cleanup));
  EXPECT_FALSE(after_locked.accepted);
  EXPECT_FALSE(after_locked.transitioned);
  EXPECT_EQ(after_locked.effect.kind, ElevatorEffectKind::kNone);
}

TEST(ElevatorFsm, FailureAfterExecutionLeaseAcquisitionNeverReleasesTheLease)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});
  auto output = fsm.start(kTransactionId, make_route());
  ASSERT_TRUE(output.accepted);

  output = fsm.dispatch(success(output));
  ASSERT_EQ(output.effect.kind, ElevatorEffectKind::kAcquireSafetyHold);
  output = fsm.dispatch(success(output));
  ASSERT_EQ(output.effect.kind, ElevatorEffectKind::kAcquireExecutionLease);
  output = fsm.dispatch(success(output));
  ASSERT_EQ(output.effect.kind, ElevatorEffectKind::kMockPressCallButton);

  output = fsm.dispatch(
    {
      ElevatorEventKind::kEffectFailed,
      "button_port_failed",
      output.effect.sequence,
      kTransactionId,
    });

  EXPECT_EQ(output.effect.kind, ElevatorEffectKind::kHoldAndCancel);
  EXPECT_NE(output.effect.kind, ElevatorEffectKind::kReleaseExecutionLease);
  EXPECT_TRUE(fsm.locked());

  const auto after_lock = fsm.dispatch(success(output));
  EXPECT_EQ(after_lock.effect.kind, ElevatorEffectKind::kNone);
  EXPECT_NE(after_lock.effect.kind, ElevatorEffectKind::kReleaseExecutionLease);
}

TEST(ElevatorFsm, StalePortCompletionCannotAdvanceANewerStep)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});
  auto output = fsm.start(kTransactionId, make_route());
  ASSERT_TRUE(output.accepted);
  const auto stale_sequence = output.effect.sequence;
  output = fsm.dispatch(success(output));
  ASSERT_TRUE(output.accepted);
  ASSERT_NE(output.effect.sequence, stale_sequence);

  output = fsm.dispatch(
    {
      ElevatorEventKind::kEffectSucceeded,
      "",
      stale_sequence,
      kTransactionId,
    });

  EXPECT_TRUE(output.accepted);
  EXPECT_EQ(output.effect.kind, ElevatorEffectKind::kHoldAndCancel);
  EXPECT_TRUE(fsm.locked());
}

TEST(ElevatorFsm, EventFromAnotherTransactionIsIgnoredWithoutAdvancing)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});
  const auto start = fsm.start(kTransactionId, make_route());
  ASSERT_TRUE(start.accepted);

  const auto foreign = fsm.dispatch(
    {
      ElevatorEventKind::kEffectSucceeded,
      "",
      start.effect.sequence,
      "old-elevator-tx",
    });

  EXPECT_FALSE(foreign.accepted);
  EXPECT_FALSE(foreign.transitioned);
  EXPECT_EQ(foreign.effect.kind, ElevatorEffectKind::kNone);
  EXPECT_EQ(fsm.state(), ElevatorState::kNavigatingHallCall);

  const auto current = fsm.dispatch(success(start));
  EXPECT_TRUE(current.accepted);
  EXPECT_EQ(current.effect.kind, ElevatorEffectKind::kAcquireSafetyHold);
}

TEST(ElevatorFsm, UnsafeTransactionIdFailsClosed)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});

  const auto output = fsm.start("../stale", make_route());

  EXPECT_FALSE(output.accepted);
  EXPECT_EQ(output.effect.kind, ElevatorEffectKind::kNone);
  EXPECT_EQ(fsm.state(), ElevatorState::kIdle);
  EXPECT_FALSE(fsm.locked());
}

TEST(ElevatorFsm, ProductionPortsRemainFailClosedUntilIntegrated)
{
  ElevatorFsm fsm;

  const auto output = fsm.start(kTransactionId, make_route());

  EXPECT_FALSE(output.accepted);
  EXPECT_EQ(output.effect.kind, ElevatorEffectKind::kNone);
  EXPECT_EQ(fsm.state(), ElevatorState::kIdle);
  EXPECT_FALSE(fsm.locked());
}

TEST(ElevatorFsm, FailedSafetyCleanupIsRetriedUntilAcknowledged)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});
  const auto start = fsm.start(kTransactionId, make_route());
  ASSERT_TRUE(start.accepted);
  const auto cleanup = fsm.dispatch(
    {
      ElevatorEventKind::kEffectFailed,
      "navigation_failed",
      start.effect.sequence,
      kTransactionId,
    });
  ASSERT_EQ(cleanup.effect.kind, ElevatorEffectKind::kHoldAndCancel);

  const auto retry = fsm.dispatch(
    {
      ElevatorEventKind::kEffectFailed,
      "safety_service_unavailable",
      cleanup.effect.sequence,
      kTransactionId,
    });
  ASSERT_TRUE(retry.accepted);
  EXPECT_EQ(retry.effect.kind, ElevatorEffectKind::kHoldAndCancel);
  EXPECT_GT(retry.effect.sequence, cleanup.effect.sequence);
  EXPECT_EQ(fsm.state(), ElevatorState::kFailureCleanup);

  const auto acknowledged = fsm.dispatch(success(retry));
  EXPECT_TRUE(acknowledged.accepted);
  EXPECT_EQ(acknowledged.effect.kind, ElevatorEffectKind::kNone);
  EXPECT_EQ(fsm.state(), ElevatorState::kLocked);
}

}  // namespace
}  // namespace robot_elevator_manager
