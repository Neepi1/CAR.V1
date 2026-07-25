# robot_interfaces

Shared ROS 2 interface definitions for the delivery robot stack. This package
contains schemas only. It does not execute a mission, move the robot, switch a
floor, publish TF, or grant a motion permission.

## P6 contracts

### Elevator and mission

- `action/ElevatorTask`: one correlated cross-floor elevator transaction. Its
  goal freezes the target `building/floor/map/expected_asset_epoch/
  expected_asset_digest`; its result must report the exact final
  `building/floor/map/asset_epoch/asset_digest`, explicit-relocalization
  sequence, runtime-context validity, final zone, and residual
  goal/hold/lease/pause proof.
- `action/MissionTask`: same-floor or cross-floor mission orchestration with an
  optional preferred elevator, explicit target map/pose context, and the same
  frozen positive `expected_asset_epoch + expected_asset_digest` used by its
  elevator effect. The current pure mission core accepts a direct same-floor
  mission only when source and target map also match; a same-floor map change
  requires a separate floor-switch workflow.
- `action/PressButton`: future arm adapter boundary for hall-call and
  target-floor button presses.
- `msg/ElevatorObservation`: future vision adapter boundary for correlated,
  timestamped door and observed-floor evidence.
- `msg/ArmState`: future arm availability, stowed, moving, pressing, and fault
  state.

`robot_elevator_manager` and `robot_mission_manager` currently provide pure C++
state-machine cores, and their non-moving integration test composes the related
floor, mode, safety, and correction-pause arbiters. There is no ROS action
server, Nav2 adapter, arm adapter, or vision adapter yet.

### Floor transaction

- `action/FloorSwitch`: intended live atomic-switch contract with
  `transaction_id`, target `building/floor/map`, exact
  `expected_asset_epoch + expected_asset_digest`, explicit-relocalization
  sequence, runtime-context validity, and recovery-required result fields.
- `msg/FloorSwitchStatus`: intended authoritative selected/pending/active floor
  context and per-stage readiness state. Requested epoch/digest have separate
  fields so a blocked request is never mislabeled as active asset evidence.
- `srv/BeginFloorTransition`: intended localization-side begin/commit/abort
  transaction boundary.
- `srv/SwitchFloor`: legacy selection/live-switch service retained for the
  existing floor-manager node.
- `srv/ApplyFloorAssets` and `srv/TriggerLocalization`: lower-level legacy
  localizer wrapper calls.

`robot_floor_manager` now contains a pure C++ transition core for this ordered
barrier, including correction-pause handoff, source-context invalidation,
target-epoch readiness, retryable cleanup, and failure lock. The new action and
transition service still have no live ROS adapter. The existing
`/floor_manager/switch_floor` service is not equivalent to the new atomic
success barrier.

### Safety interlock

- `srv/SetMotionHold`: owner- and transaction-scoped persistent stop hold.
- `srv/SetExecutionLease`: bounded steady-clock execution heartbeat and explicit
  recovery/release contract.
- `msg/MotionInterlockState`: authoritative hold keys, execution owner/session,
  TTL, generation, mode-contract validity, normal-source restriction, and
  effective blocked state.

These interfaces are wired into the current `robot_safety` source and covered
by pure arbiter tests plus a manual isolated ROS smoke test. Real elevator
readiness still requires ROS action integration, deployment in the single
production safety process, process-loss testing, and physical stop validation.

### Localization correction ownership

- `srv/SetCorrectionPause`: composable owner- and transaction-scoped correction
  pause acquire/release contract.
- `msg/CorrectionPauseState`: authoritative effective pause, generation, and
  active owner keys.
- `msg/LocalizationHealth`: floor/map/asset epoch, explicit-relocalization
  sequence, transition, AMCL, and runtime-context fields used by the future
  floor-switch barrier.

The owner-scoped correction pause is wired into
`robot_localization_bridge`. It coexists with the legacy
`std_srvs/SetBool` endpoint; releasing one owner cannot release another owner.
The expanded localization-health schema is not proof that every current
publisher populates or validates all new fields.

### Operating mode

- `srv/SetMode`: single lease-owned command interface.
- `msg/OperatingModeState`: authoritative state and heartbeat for `NORMAL`,
  `RAMP`, `ELEVATOR_WAIT`, `ELEVATOR_RIDE`, `DOORWAY`, and `RECOVERY`.

Non-`NORMAL` modes require an owner, mission ID, lease ID, and bounded lease
duration. These names are mission profiles, not Ranger chassis motion modes,
and never publish velocity commands. A mode lease is not a safety permission;
elevator motion additionally requires the `robot_safety` execution interlock.

## Interface availability is not runtime readiness

An interface file being generated means only that packages can compile against
the contract. Production readiness additionally requires:

1. exactly one expected service/action server and authoritative state publisher;
2. adapters that correlate every response with the same mission, transaction,
   lease, map, and asset epoch;
3. timeout, cancellation, restart, and stale-message tests;
4. preservation of the canonical TF and final command ownership rules;
5. isolated ROS integration tests followed by supervised hardware acceptance.

In particular, the arm/vision messages and actions are reserved seams, not
active sensor evidence. Mock responses must never be accepted as production
door, floor, button-contact, or arm-stowed evidence.

Runtime ownership still follows the canonical repository policy:
`robot_localization_bridge` is the only `map -> odom` publisher,
`robot_local_state` is the only `odom -> base_link` publisher, and
`robot_safety` is the only final `/cmd_vel` publisher.

## ElevatorTask and MissionTask compatibility note

The exact target-identity fields intentionally change the generated ROS action
types. Every future `MissionTask`/`ElevatorTask` producer and consumer must be
rebuilt and deployed together. There is no permissive legacy mode: an epoch of
`0`, an empty/non-canonical digest, a missing final building, or any mismatch
in the frozen tuple is invalid. This is safe to introduce now because the
repository still has no live mission/elevator action adapter.
