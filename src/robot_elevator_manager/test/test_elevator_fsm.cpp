#include <gtest/gtest.h>

#include <optional>
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
      {PoseRole::kLanding, prefix + "_landing"},
      {PoseRole::kCabin, prefix + "_cabin"},
    },
    std::nullopt,
  };
}

ElevatorRoute make_route()
{
  return ElevatorRoute{
    "elevator_west",
    "building_1",
    make_floor("F1", "f1_west"),
    make_floor("F2", "f2_west"),
    2U,
  };
}

FloorElevatorTopology make_reverse_entry_floor(
  const std::string & floor_id,
  const std::string & prefix,
  const PanelSide hall_side,
  const PanelSide cabin_side)
{
  return FloorElevatorTopology{
    floor_id,
    floor_id + "_map",
    {
      {PoseRole::kHallCall, prefix + "_hall_call"},
      {PoseRole::kLanding, prefix + "_landing"},
      {PoseRole::kCabin, prefix + "_cabin"},
      {PoseRole::kCabinPanel, prefix + "_cabin_panel"},
    },
    std::nullopt,
    hall_side,
    cabin_side,
  };
}

ElevatorRoute make_reverse_entry_route()
{
  return ElevatorRoute{
    "elevator_west",
    "building_1",
    make_reverse_entry_floor(
      "F1", "f1_west", PanelSide::kLeft, PanelSide::kRight),
    make_reverse_entry_floor(
      "F2", "f2_west", PanelSide::kRight, PanelSide::kLeft),
    3U,
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
  ElevatorNavigationIntent navigation_intent{ElevatorNavigationIntent::kNone};
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
  EXPECT_EQ(output.effect.navigation_intent, expected.navigation_intent);
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
      ElevatorNavigationIntent::kHallCall,
    },
    0U);

  const std::vector<ExpectedEffect> effects_before_cabin_hold{
    {ElevatorEffectKind::kAcquireSafetyHold, "", "F1", ""},
    {ElevatorEffectKind::kMockPressCallButton, "", "F1", ""},
    {ElevatorEffectKind::kSetOperatingMode, "", "F1", "ELEVATOR_WAIT"},
    {ElevatorEffectKind::kReleaseSafetyHold, "", "F1", ""},
    {
      ElevatorEffectKind::kNavigateToPose,
      "f1_west_landing", "F1", "", "",
      ElevatorNavigationIntent::kSourceLanding,
    },
    {ElevatorEffectKind::kMockWaitDoorOpen, "", "F1", ""},
    {ElevatorEffectKind::kSetOperatingMode, "", "F1", "DOORWAY"},
    {
      ElevatorEffectKind::kNavigateToPose,
      "f1_west_cabin", "F1", "", "",
      ElevatorNavigationIntent::kEnterCabin,
    },
    {ElevatorEffectKind::kAcquireSafetyHold, "", "F1", ""},
  };
  for (const auto & expected : effects_before_cabin_hold) {
    const auto previous_sequence = output.effect.sequence;
    output = fsm.dispatch(success(output));
    ASSERT_TRUE(output.accepted);
    EXPECT_TRUE(output.transitioned);
    expect_effect(output, expected, previous_sequence);
  }

  const std::vector<ExpectedEffect> effects_before_target_landing_hold{
    {ElevatorEffectKind::kMockPressTargetButton, "", "F2", ""},
    {ElevatorEffectKind::kPauseLocalizationCorrections, "", "F1", ""},
    {ElevatorEffectKind::kSetOperatingMode, "", "F1", "ELEVATOR_RIDE"},
    {ElevatorEffectKind::kMockRide, "", "F2", ""},
    {ElevatorEffectKind::kMockWaitDoorOpen, "", "F2", ""},
    {
      ElevatorEffectKind::kBeginFloorTransition,
      "f2_west_cabin", "F2", "", "F2_map",
    },
    {ElevatorEffectKind::kResumeLocalizationCorrections, "", "F1", ""},
    {ElevatorEffectKind::kSwitchFloor, "f2_west_cabin", "F2", "", "F2_map"},
    {ElevatorEffectKind::kVerifyFloorReady, "f2_west_cabin", "F2", "", "F2_map"},
    {ElevatorEffectKind::kSetOperatingMode, "", "F2", "DOORWAY"},
    {ElevatorEffectKind::kReleaseSafetyHold, "", "F2", ""},
    {
      ElevatorEffectKind::kNavigateToPose,
      "f2_west_landing", "F2", "", "",
      ElevatorNavigationIntent::kTargetLanding,
    },
    {ElevatorEffectKind::kAcquireSafetyHold, "", "F2", ""},
  };
  for (const auto & expected : effects_before_target_landing_hold) {
    const auto previous_sequence = output.effect.sequence;
    output = fsm.dispatch(success(output));
    ASSERT_TRUE(output.accepted);
    EXPECT_TRUE(output.transitioned);
    expect_effect(output, expected, previous_sequence);
  }

  const std::vector<ExpectedEffect> completion_effects{
    {ElevatorEffectKind::kReleaseOperatingMode, "", "F2", ""},
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

TEST(ElevatorFsm, V3UsesReverseEntryPanelAndReturnToCenterSequence)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});
  auto output = fsm.start(kTransactionId, make_reverse_entry_route());
  ASSERT_TRUE(output.accepted);

  std::vector<ExpectedEffect> navigation_effects;
  std::vector<ExpectedEffect> floor_effects;
  PanelSide call_panel_side = PanelSide::kUnknown;
  PanelSide cabin_panel_side = PanelSide::kUnknown;
  for (std::size_t guard = 0U;
    guard < 64U && fsm.state() != ElevatorState::kComplete;
    ++guard)
  {
    if (output.effect.kind == ElevatorEffectKind::kNavigateToPose) {
      navigation_effects.push_back(
        ExpectedEffect{
          output.effect.kind,
          output.effect.pose_id,
          output.effect.floor_id,
          output.effect.mode,
          output.effect.map_id,
          output.effect.navigation_intent,
        });
    }
    if (output.effect.kind == ElevatorEffectKind::kMockPressCallButton) {
      call_panel_side = output.effect.panel_side;
    }
    if (output.effect.kind == ElevatorEffectKind::kMockPressTargetButton) {
      cabin_panel_side = output.effect.panel_side;
    }
    if (
      output.effect.kind == ElevatorEffectKind::kBeginFloorTransition ||
      output.effect.kind == ElevatorEffectKind::kSwitchFloor ||
      output.effect.kind == ElevatorEffectKind::kVerifyFloorReady)
    {
      floor_effects.push_back(
        ExpectedEffect{
          output.effect.kind,
          output.effect.pose_id,
          output.effect.floor_id,
          output.effect.mode,
          output.effect.map_id,
          output.effect.navigation_intent,
        });
    }
    output = fsm.dispatch(success(output));
    ASSERT_TRUE(output.accepted);
  }

  ASSERT_EQ(fsm.state(), ElevatorState::kComplete);
  EXPECT_EQ(call_panel_side, PanelSide::kLeft);
  EXPECT_EQ(cabin_panel_side, PanelSide::kRight);
  ASSERT_EQ(navigation_effects.size(), 6U);
  EXPECT_EQ(navigation_effects[0].pose_id, "f1_west_hall_call");
  EXPECT_EQ(
    navigation_effects[0].navigation_intent,
    ElevatorNavigationIntent::kHallCall);
  EXPECT_EQ(navigation_effects[1].pose_id, "f1_west_landing");
  EXPECT_EQ(
    navigation_effects[1].navigation_intent,
    ElevatorNavigationIntent::kReverseEntryStaging);
  EXPECT_EQ(navigation_effects[2].pose_id, "f1_west_cabin");
  EXPECT_EQ(
    navigation_effects[2].navigation_intent,
    ElevatorNavigationIntent::kReverseEnterCabin);
  EXPECT_EQ(navigation_effects[3].pose_id, "f1_west_cabin_panel");
  EXPECT_EQ(
    navigation_effects[3].navigation_intent,
    ElevatorNavigationIntent::kCabinPanelApproach);
  EXPECT_EQ(navigation_effects[4].pose_id, "f2_west_cabin");
  EXPECT_EQ(
    navigation_effects[4].navigation_intent,
    ElevatorNavigationIntent::kReturnCabinCenter);
  EXPECT_EQ(navigation_effects[5].pose_id, "f2_west_landing");
  EXPECT_EQ(
    navigation_effects[5].navigation_intent,
    ElevatorNavigationIntent::kTargetLanding);

  ASSERT_EQ(floor_effects.size(), 3U);
  for (const auto & effect : floor_effects) {
    EXPECT_EQ(effect.pose_id, "f2_west_cabin_panel");
    EXPECT_EQ(effect.floor_id, "F2");
    EXPECT_EQ(effect.map_id, "F2_map");
  }
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

TEST(ElevatorFsm, NormalFlowNeverEmitsExecutionLeaseEffects)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});
  auto output = fsm.start(kTransactionId, make_route());
  ASSERT_TRUE(output.accepted);

  for (std::size_t guard = 0U;
    guard < 64U && fsm.state() != ElevatorState::kComplete;
    ++guard)
  {
    EXPECT_NE(output.effect.kind, ElevatorEffectKind::kAcquireExecutionLease);
    EXPECT_NE(output.effect.kind, ElevatorEffectKind::kReleaseExecutionLease);
    output = fsm.dispatch(success(output));
    ASSERT_TRUE(output.accepted);
  }

  EXPECT_EQ(fsm.state(), ElevatorState::kComplete);
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

TEST(ElevatorFsm, LegacyRouteIsReadOnlyEvenWhenMockPortsAreEnabled)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});
  auto route = make_route();
  route.schema_version = 1U;

  const auto output = fsm.start(kTransactionId, route);

  EXPECT_FALSE(output.accepted);
  EXPECT_FALSE(output.transitioned);
  EXPECT_EQ(output.effect.kind, ElevatorEffectKind::kNone);
  EXPECT_EQ(output.message, "legacy_read_only");
  EXPECT_EQ(fsm.state(), ElevatorState::kIdle);
}

TEST(ElevatorFsm, UnversionedRouteCannotImplicitlyBecomeExecutableV2)
{
  ElevatorFsm fsm(ElevatorFsmOptions{true});
  auto route = make_route();
  route.schema_version = 0U;

  const auto output = fsm.start(kTransactionId, route);

  EXPECT_FALSE(output.accepted);
  EXPECT_FALSE(output.transitioned);
  EXPECT_EQ(output.effect.kind, ElevatorEffectKind::kNone);
  EXPECT_EQ(output.message, "legacy_read_only");
  EXPECT_EQ(fsm.state(), ElevatorState::kIdle);
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
