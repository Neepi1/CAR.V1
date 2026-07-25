# robot_elevator_manager

This package currently provides a pure C++ elevator safety core. It does not
start a ROS node, publish velocity or TF, call Nav2, move the robot, or integrate
the arm/vision button module.

## Implemented tracer bullet

- `elevator_topology` validates path-safe elevator/building/floor/map IDs,
  path-safe `pose_id` values, unique floors, and exactly one pose for each required role:
  `hall_call`, `hall_wait`, `doorway`, `cabin`, and `exit`.
- `elevator_topology_loader` parses schema-versioned
  `maps_release/<building_id>/elevators.yaml` catalogs and returns no usable
  catalog unless every elevator passes the same topology/geometry validation.
- `elevator_release_loader` is the non-moving runtime preparation boundary. It
  fixes `.elevator_config/current` once (or accepts an exact release ID), reads
  only `releases/<release_id>/`, verifies the six published files and legacy
  release identity, requires each map binding to carry an unquoted positive
  `uint64` `asset_epoch`, the exact
  `njrh-map-asset-bundle-v1` digest contract, and a canonical SHA-256 digest,
  cross-checks those exact identities between configuration and manifest, cross-checks
  configuration/topology/internal poses, deterministically selects an
  elevator, and returns one frozen source/target plan containing both source
  and target map epochs/digests. On Jetson/Linux it uses
  a pinned directory FD plus `openat(O_NOFOLLOW)`, rejects soft/hard links and
  files over 2 MiB, and detects files changed while being read.
- `threshold_footprint` classifies an already transformed, complete robot
  footprint against a directed door threshold. `INSIDE` and `OUTSIDE` require
  every vertex to clear the configured margin and remain between both door-jamb
  clearances. A crossing, clearance-band contact, jamb violation, malformed
  footprint, or malformed threshold is fail-closed as `STRADDLING`/invalid.
- `elevator_fsm` emits one external port effect for each accepted event. Every
  effect carries a path-safe transaction ID and monotonically increasing
  sequence; a foreign transaction is ignored and a stale completion cannot
  advance a newer step. Mock call-button, target-button, door, and ride effects
  follow the same effect boundary intended for later arm/vision adapters.
- After the source-hall safety hold is confirmed, the FSM acquires an
  owner-scoped safety execution session before any `ELEVATOR_*`/`DOORWAY`
  motion can be unlocked. The future adapter owns heartbeat renewal. The
  session is released only after full-footprint `OUTSIDE`, under the exit hold,
  and after the operating-mode lease has been released.
- A port failure, cancellation, out-of-order event, stale completion, or failed
  footprint gate enters `FAILURE_CLEANUP` and emits `HOLD_AND_CANCEL`. A failed
  cleanup acknowledgement re-emits that safety effect with a new sequence; only
  a successful acknowledgement enters permanent `LOCKED`.
- A locked FSM has no in-process reset, emits no further effects, and never
  releases its execution session.

`HOLD_AND_CANCEL` is an instruction to the future ROS adapter, not an action
performed by this library. Every non-`NONE` effect has `accepted=true`; an
invalid start request produces no effect before motion begins. The adapter must
first obtain/maintain a
`robot_safety` stop hold, then cancel its single active Nav2 goal and wait for a
terminal/settled result. It must not release the hold merely by destroying and
recreating the FSM.

## Mock flow

The core sequences these boundaries without executing them:

1. Navigate to source hall call, acquire a stop hold, then acquire the
   owner-scoped safety execution session.
2. Mock press call, enter `ELEVATOR_WAIT`, release hold, and navigate to wait.
3. Mock source door open, enter `DOORWAY`, navigate doorway then cabin.
4. Require the full footprint to be `INSIDE`, acquire hold, and mock press target.
5. Pause localization corrections, enter `ELEVATOR_RIDE`, mock ride and target door.
6. Ask floor manager to acquire its own correction-pause lease and invalidate
   the source runtime context; only after that acknowledgement release the
   elevator-owned correction pause. Effective pause therefore never drops while
   source-floor assets are still active.
7. Run the floor-switch transaction for the exact target floor/map and verify
   its committed readiness while the safety hold remains active.
8. Enter `DOORWAY`, release hold, navigate doorway then exit.
9. Require the full footprint to be `OUTSIDE`, acquire hold, release the exact
   operating-mode lease back to `NORMAL`, release the execution session, then
   release hold and complete.

The example in `config/elevators.example.yaml` documents the loader schema for
`maps_release/<building_id>/elevators.yaml`. The top-level
`mock_ports_enabled` key is required, parsed, and retained in
`ElevatorTopologyCatalog`; the checked-in example sets it to `false`.
`ElevatorFsmOptions` also defaults to `false`, so production-port startup stays
fail-closed. Unit tests and the non-moving mock scenario must opt in explicitly
with `ElevatorFsmOptions{true}`. The loader is still pure C++ and there is no
ROS runtime adapter or automatic activation of the catalog; a future adapter
must deliberately propagate the loaded value when constructing the FSM.
These role poses are internal elevator waypoints; the ordinary delivery API
must not expose them as normal delivery destinations.

## Validation boundary

The package GTests cover topology and YAML validation, complete-footprint
threshold classification, ordered effects, transaction/sequence fencing,
fail-closed production ports, immutable release loading, and retryable safety
cleanup. Release-loader tests cover selector pinning, legacy publisher FNV
vectors, strict metadata JSON, positive exact `uint64` map epochs, canonical
map SHA-256, symlink/hard-link/path escape rejection, size bounds,
topology/configuration/internal-pose
cross-checks, stable automatic elevator selection, and frozen-return behavior.
The non-moving
cross-floor scenario additionally composes the mission, floor-transition,
safety, mode, and correction-pause pure cores and verifies the successful
synthetic path leaves no goal, hold, lease, or pause.

No test in this package starts a ROS action server, calls live Nav2, reloads a
real floor, publishes Twist or TF, or moves the chassis.

## Real hardware work still required

- Survey and save all five pose roles on every served floor.
- Measure each door threshold endpoints, cabin-side reference, and clearance in
  the corresponding floor map frame.
- Feed the complete transformed Ranger Mini 3 footprint, not only `base_link`,
  into both threshold gates.
- Implement the elevator ROS action/port adapter that drives Nav2 and calls the
  existing owner-scoped safety, operating-mode, correction-pause, and pure
  floor-transition contracts.
- Connect the frozen authoritative map epoch/digest to real localizer reload
  identity/evidence, target-epoch costmap readiness, and explicit recovery
  transaction before converting a frozen release into a `FloorSwitchGoal`.
- Connect arm/vision button and door observations through the mock effect
  boundary, then validate timeouts and correlation IDs.
- Test entry/exit with an empty stationary elevator before any loaded or public
  operation.
