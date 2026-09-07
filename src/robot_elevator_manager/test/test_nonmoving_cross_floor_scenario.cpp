#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "robot_elevator_manager/elevator_fsm.hpp"
#include "robot_floor_manager/floor_transition_core.hpp"
#include "robot_localization_bridge/correction_pause_arbiter.hpp"
#include "robot_mission_manager/mission_fsm.hpp"
#include "robot_mode_manager/mode_lease_arbiter.hpp"
#include "robot_safety/motion_interlock_arbiter.hpp"

namespace
{

using robot_elevator_manager::ElevatorEffectKind;
using robot_elevator_manager::ElevatorEvent;
using robot_elevator_manager::ElevatorEventKind;
using robot_elevator_manager::ElevatorFsm;
using robot_elevator_manager::ElevatorFsmOutput;
using robot_elevator_manager::ElevatorNavigationIntent;
using robot_elevator_manager::ElevatorRoute;
using robot_elevator_manager::FloorElevatorTopology;
using robot_elevator_manager::PoseRole;
using robot_floor_manager::FloorTransitionCore;
using robot_floor_manager::FloorTransitionEffectKind;
using robot_floor_manager::FloorTransitionEvent;
using robot_floor_manager::FloorTransitionEventKind;
using robot_floor_manager::FloorTransitionOutput;
using robot_floor_manager::FloorTransitionRequest;
using robot_localization_bridge::CorrectionPauseArbiter;
using robot_localization_bridge::PauseCommand;
using robot_localization_bridge::PauseOperation;
using robot_mission_manager::EffectCompletion;
using robot_mission_manager::EffectKind;
using robot_mission_manager::MissionFsm;
using robot_mission_manager::MissionRequest;
using robot_mode_manager::ModeCommand;
using robot_mode_manager::ModeLeaseArbiter;
using robot_mode_manager::ModeOperation;
using robot_safety::HoldCommand;
using robot_safety::HoldOperation;
using robot_safety::MotionInterlockArbiter;

FloorElevatorTopology floor(
  const std::string & floor_id,
  const std::string & map_id,
  const std::string & prefix)
{
  return {
    floor_id,
    map_id,
    {
      {PoseRole::kHallCall, prefix + "_hall_call"},
      {PoseRole::kLanding, prefix + "_landing"},
      {PoseRole::kCabin, prefix + "_cabin"},
    },
    std::nullopt,
  };
}

ElevatorEvent elevator_success(const ElevatorFsmOutput & output)
{
  return {
    ElevatorEventKind::kEffectSucceeded,
    "",
    output.effect.sequence,
    output.effect.transaction_id,
  };
}

FloorTransitionEvent floor_success(
  const FloorTransitionOutput & output,
  const FloorTransitionRequest & target,
  const bool floor_pause_owned,
  const bool correction_pause_effective,
  const bool caller_pause_released)
{
  FloorTransitionEvent event;
  event.kind = FloorTransitionEventKind::kEffectSucceeded;
  event.transaction_id = output.effect.transaction_id;
  event.effect_sequence = output.effect.sequence;
  auto & evidence = event.evidence;
  evidence.motion_hold_active = true;
  evidence.nav_idle = true;
  evidence.stopped = true;
  evidence.floor_pause_owned = floor_pause_owned;
  evidence.correction_pause_effective = correction_pause_effective;
  evidence.caller_pause_released = caller_pause_released;
  evidence.runtime_context_invalid = true;
  evidence.bridge_ready = true;
  evidence.amcl_ready = true;
  evidence.global_costmap_fresh = true;
  evidence.local_costmap_fresh = true;
  evidence.active_building_id = target.building_id;
  evidence.active_floor_id = target.floor_id;
  evidence.active_map_id = target.map_id;
  evidence.asset_digest = target.expected_asset_digest;
  evidence.asset_epoch = target.expected_asset_epoch;
  evidence.explicit_relocalization_sequence = 31U;
  if (output.effect.kind == FloorTransitionEffectKind::kCommitRuntimeContext) {
    evidence.runtime_context_valid = true;
    evidence.safe_for_goal_start = true;
  }
  return event;
}

bool owns_pause(
  const robot_localization_bridge::CorrectionPauseSnapshot & snapshot,
  const std::string & owner,
  const std::string & transaction_id)
{
  const std::string key = owner + ":" + transaction_id;
  return std::find(
    snapshot.lease_keys.begin(),
    snapshot.lease_keys.end(),
    key) != snapshot.lease_keys.end();
}

TEST(P6NonmovingScenario, CrossFloorFlowLeavesNoGoalHoldModeOrPause)
{
  MissionFsm mission("boot-integration");
  MissionRequest mission_request;
  mission_request.mission_id = "mission-cross-floor";
  mission_request.building_id = "building_1";
  mission_request.expected_source_floor_id = "F1";
  mission_request.expected_source_map_id = "map_f1";
  mission_request.target_floor_id = "F2";
  mission_request.target_map_id = "map_f2";
  mission_request.expected_asset_epoch = 23U;
  mission_request.expected_asset_digest =
    "sha256:0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef";
  mission_request.target_pose_id = "delivery_201";
  mission_request.preferred_elevator_id = "elevator_west";

  const auto mission_start = mission.start(mission_request);
  ASSERT_TRUE(mission_start.effect.has_value());
  ASSERT_EQ(mission_start.effect->kind, EffectKind::kElevatorTask);
  const std::string elevator_transaction =
    mission_start.effect->transaction_id;

  ElevatorRoute route{
    "elevator_west",
    "building_1",
    floor("F1", "map_f1", "f1"),
    floor("F2", "map_f2", "f2"),
    2U,
  };
  ElevatorFsm elevator(robot_elevator_manager::ElevatorFsmOptions{true});
  auto elevator_output = elevator.start(elevator_transaction, route);
  ASSERT_TRUE(elevator_output.accepted);

  MotionInterlockArbiter safety;
  ModeLeaseArbiter modes;
  CorrectionPauseArbiter pauses;
  FloorTransitionCore floor_switch;
  const std::string owner = "robot_elevator_manager";
  const std::string mode_lease_id = "mode-lease-integration";
  const std::string hold_id = elevator_transaction + "-hold";
  const std::string floor_transaction = elevator_transaction + "-floor";
  const FloorTransitionRequest floor_request{
    floor_transaction,
    "building_1",
    "F2",
    "map_f2",
    23U,
    "sha256:0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef",
  };
  FloorTransitionOutput floor_output;
  bool floor_started = false;
  bool floor_begin_ready = false;
  bool hold_active = false;
  bool active_nav_goal = false;
  std::vector<std::string> elevator_navigation_targets;
  std::vector<ElevatorNavigationIntent> elevator_navigation_intents;
  double now_sec = 10.0;

  std::size_t elevator_steps = 0U;
  constexpr std::size_t kMaxElevatorSteps = 96U;
  while (
    elevator_output.effect.kind != ElevatorEffectKind::kComplete &&
    elevator_steps < kMaxElevatorSteps)
  {
    ++elevator_steps;
    now_sec += 0.05;
    const auto effect = elevator_output.effect.kind;

    if (effect == ElevatorEffectKind::kNavigateToPose) {
      elevator_navigation_targets.push_back(elevator_output.effect.pose_id);
      elevator_navigation_intents.push_back(
        elevator_output.effect.navigation_intent);
      EXPECT_FALSE(active_nav_goal);
      active_nav_goal = true;
      EXPECT_FALSE(hold_active);
      EXPECT_TRUE(safety.motion_permitted(now_sec));
      active_nav_goal = false;
    } else if (effect == ElevatorEffectKind::kAcquireSafetyHold) {
      const auto decision = safety.apply_hold(
        HoldCommand{
          HoldOperation::kAcquire,
          owner,
          hold_id,
          elevator_output.effect.detail.empty() ?
          "integration_settle" : elevator_output.effect.detail,
        },
        now_sec);
      ASSERT_TRUE(decision.accepted);
      hold_active = true;
      EXPECT_FALSE(safety.motion_permitted(now_sec));
    } else if (effect == ElevatorEffectKind::kReleaseSafetyHold) {
      const auto decision = safety.apply_hold(
        HoldCommand{HoldOperation::kRelease, owner, hold_id, ""},
        now_sec);
      ASSERT_TRUE(decision.accepted);
      hold_active = false;
    } else if (effect == ElevatorEffectKind::kSetOperatingMode) {
      ModeCommand command;
      command.mode = elevator_output.effect.mode;
      command.owner = owner;
      command.mission_id = mission_request.mission_id;
      command.lease_id = mode_lease_id;
      command.lease_duration_sec = 30.0;
      const auto decision = modes.apply(command, now_sec);
      ASSERT_TRUE(decision.accepted);
    } else if (effect == ElevatorEffectKind::kReleaseOperatingMode) {
      ModeCommand command;
      command.operation = ModeOperation::kRelease;
      command.owner = owner;
      command.mission_id = mission_request.mission_id;
      command.lease_id = mode_lease_id;
      const auto decision = modes.apply(command, now_sec);
      ASSERT_TRUE(decision.accepted);
    } else if (effect == ElevatorEffectKind::kPauseLocalizationCorrections) {
      const auto decision = pauses.apply(
        PauseCommand{
          PauseOperation::kAcquire,
          owner,
          elevator_transaction,
          "elevator_ride",
          1U,
        });
      ASSERT_TRUE(decision.accepted);
      EXPECT_TRUE(decision.state.paused);
    } else if (effect == ElevatorEffectKind::kBeginFloorTransition) {
      EXPECT_EQ(elevator_output.effect.pose_id, "f2_cabin");
      floor_output = floor_switch.start(floor_request);
      ASSERT_EQ(
        floor_output.effect.kind,
        FloorTransitionEffectKind::kVerifyPreconditions);
      floor_output = floor_switch.dispatch(
        floor_success(floor_output, floor_request, false, true, false));
      ASSERT_EQ(
        floor_output.effect.kind,
        FloorTransitionEffectKind::kAcquireCorrectionPause);

      const auto pause_decision = pauses.apply(
        PauseCommand{
          PauseOperation::kAcquire,
          "robot_floor_manager",
          floor_transaction,
          "floor_transition",
          1U,
        });
      ASSERT_TRUE(pause_decision.accepted);
      floor_output = floor_switch.dispatch(
        floor_success(floor_output, floor_request, true, true, false));
      ASSERT_EQ(
        floor_output.effect.kind,
        FloorTransitionEffectKind::kInvalidateRuntimeContext);
      floor_output = floor_switch.dispatch(
        floor_success(floor_output, floor_request, true, true, false));
      ASSERT_EQ(
        floor_output.effect.kind,
        FloorTransitionEffectKind::kReportBeginReady);
      floor_started = true;
      floor_begin_ready = true;
      EXPECT_FALSE(floor_output.runtime_context_valid);
    } else if (effect == ElevatorEffectKind::kResumeLocalizationCorrections) {
      ASSERT_TRUE(floor_begin_ready);
      const auto decision = pauses.apply(
        PauseCommand{
          PauseOperation::kRelease,
          owner,
          elevator_transaction,
          "",
          2U,
        });
      ASSERT_TRUE(decision.accepted);
      EXPECT_TRUE(decision.state.paused);
    } else if (effect == ElevatorEffectKind::kSwitchFloor) {
      EXPECT_EQ(elevator_output.effect.pose_id, "f2_cabin");
      ASSERT_TRUE(floor_started);
      const auto handoff_state = pauses.snapshot();
      floor_output = floor_switch.dispatch(
        floor_success(
          floor_output,
          floor_request,
          owns_pause(handoff_state, "robot_floor_manager", floor_transaction),
          handoff_state.paused,
          !owns_pause(handoff_state, owner, elevator_transaction)));
      std::size_t floor_steps = 0U;
      constexpr std::size_t kMaxFloorSteps = 32U;
      while (
        floor_switch.state() !=
        robot_floor_manager::FloorTransitionState::kComplete &&
        floor_steps < kMaxFloorSteps)
      {
        ++floor_steps;
        if (floor_output.effect.kind ==
          FloorTransitionEffectKind::kReleaseFloorPause)
        {
          const auto pause_decision = pauses.apply(
            PauseCommand{
              PauseOperation::kRelease,
              "robot_floor_manager",
              floor_transaction,
              "",
              2U,
            });
          ASSERT_TRUE(pause_decision.accepted);
        }
        const auto pause_state = pauses.snapshot();
        floor_output = floor_switch.dispatch(
          floor_success(
            floor_output,
            floor_request,
            owns_pause(
              pause_state,
              "robot_floor_manager",
              floor_transaction),
            pause_state.paused,
            !owns_pause(pause_state, owner, elevator_transaction)));
        ASSERT_NE(
          floor_output.effect.kind,
          FloorTransitionEffectKind::kHoldAndLock);
      }
      ASSERT_EQ(
        floor_switch.state(),
        robot_floor_manager::FloorTransitionState::kComplete);
      EXPECT_TRUE(floor_switch.runtime_context_valid());
      EXPECT_FALSE(floor_switch.recovery_required());
      EXPECT_FALSE(pauses.snapshot().paused);
    } else if (effect == ElevatorEffectKind::kVerifyFloorReady) {
      EXPECT_EQ(elevator_output.effect.pose_id, "f2_cabin");
      ASSERT_EQ(
        floor_switch.state(),
        robot_floor_manager::FloorTransitionState::kComplete);
    }

    elevator_output = elevator.dispatch(elevator_success(elevator_output));
    ASSERT_TRUE(elevator_output.accepted);
  }

  ASSERT_EQ(elevator_output.effect.kind, ElevatorEffectKind::kComplete);
  EXPECT_EQ(
    elevator_navigation_targets,
    (std::vector<std::string>{
      "f1_hall_call",
      "f1_landing",
      "f1_cabin",
      "f2_landing",
    }));
  EXPECT_EQ(
    elevator_navigation_intents,
    (std::vector<ElevatorNavigationIntent>{
      ElevatorNavigationIntent::kHallCall,
      ElevatorNavigationIntent::kSourceLanding,
      ElevatorNavigationIntent::kEnterCabin,
      ElevatorNavigationIntent::kTargetLanding,
    }));
  EXPECT_FALSE(active_nav_goal);
  const auto safety_state = safety.snapshot(now_sec);
  const auto mode_state = modes.snapshot(now_sec);
  const auto pause_state = pauses.snapshot();
  EXPECT_TRUE(safety_state.hold_keys.empty());
  EXPECT_FALSE(safety_state.execution_session_engaged);
  EXPECT_EQ(mode_state.mode, robot_mode_manager::OperatingMode::kNormal);
  EXPECT_FALSE(mode_state.lease_active);
  EXPECT_FALSE(pause_state.paused);

  EffectCompletion elevator_completion;
  elevator_completion.transaction_id = elevator_transaction;
  elevator_completion.success = true;
  elevator_completion.final_building_id = "building_1";
  elevator_completion.final_floor_id = "F2";
  elevator_completion.final_map_id = "map_f2";
  elevator_completion.final_zone = "TARGET_HALL";
  elevator_completion.final_asset_epoch = 23U;
  elevator_completion.final_asset_digest =
    "sha256:0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef";
  elevator_completion.explicit_relocalization_sequence = 31U;
  elevator_completion.runtime_context_valid = true;
  elevator_completion.nav_goal_active = active_nav_goal;
  elevator_completion.safety_hold_active = !safety_state.hold_keys.empty();
  elevator_completion.residual_mode_lease = mode_state.lease_active;
  elevator_completion.residual_execution_lease =
    safety_state.execution_session_engaged;
  elevator_completion.residual_correction_pause = pause_state.paused;

  const auto target_nav = mission.complete_effect(elevator_completion);
  ASSERT_TRUE(target_nav.effect.has_value());
  EXPECT_EQ(target_nav.effect->kind, EffectKind::kNavTarget);
  EXPECT_EQ(target_nav.effect->target_pose_id, "delivery_201");
  EffectCompletion target_completion;
  target_completion.transaction_id = target_nav.effect->transaction_id;
  target_completion.success = true;
  target_completion.message = "target reached";
  const auto done = mission.complete_effect(target_completion);
  EXPECT_TRUE(done.accepted);
  EXPECT_EQ(
    mission.state(),
    robot_mission_manager::MissionState::kSucceeded);
}

}  // namespace
