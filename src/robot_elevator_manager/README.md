# robot_elevator_manager

This package currently provides a pure C++ elevator safety core. It does not
start a ROS node, publish velocity or TF, call Nav2, move the robot, or integrate
the arm/vision button module.

## Production recovery policy (2026-08-21)

`ElevatorExecutionOptions::persistent_recovery_lock_enabled` preserves the
historical strict behavior only as a compatibility default for isolated tests
and forensic tools. The production API server always passes `false`; there is
no runtime parameter that can turn the persistent latch back on.

With that production option, failure still requests stop/cancel and retains
the owner hold only while automatic cleanup is executing. Cleanup and restart
reconciliation retry automatically until release is proven. They never end in
terminal `LOCKED`, never require an operator recovery transaction, and never
create a global `ELEVATOR_EXECUTION_RECOVERY_REQUIRED` API interlock. A legacy
retained journal is migrated into the same automatic cleanup path on startup.
This does not weaken estop, watchdog, live transaction mutual exclusion,
`robot_safety`, or command arbitration.

## Automatic button effect policy (2026-08-25)

The core remains independent of ROS, HTTP, arm hardware, and vision. A runtime
port may now advertise `automatic_button_control`; only then do the existing
call-button and target-button effects cross the port boundary. Door-open and
floor-arrival effects always remain strict operator confirmations.

An automatic hall-call port may explicitly return
`operator_confirmation_required` when a deployed physical capability is known
to be unavailable. The wrapper then journals
`AUTOMATIC_BUTTON_MANUAL_FALLBACK` and restores the exact
`CALL_BUTTON_PRESSED` gate. The production adapter issues that result only
after the arm `release` task has completed. It does not require a health,
ready-state, avoidance-pose, or `safe_to_drive` proof during feature
validation. Other automatic-button failures use the normal failure path;
the core never infers a fallback from an error string or HTTP status.

## Implemented tracer bullet

- `elevator_topology` validates path-safe elevator/building/floor/map IDs,
  path-safe `pose_id` values, unique floors, and exactly one pose for each required role:
  schema v2 uses `hall_call`, `landing`, and `cabin`; schema v3 uses
  `hall_call`, `landing`, `cabin`, and `cabin_panel`, plus explicit
  `hall_call_panel_side` and `cabin_panel_side` (`LEFT`/`RIGHT`) on every
  floor. Schema v1 retains the historical five-role and threshold contract
  only for integrity validation.
- `elevator_topology_loader` parses schema-versioned
  `maps_release/<building_id>/elevators.yaml` catalogs and returns no usable
  catalog unless every elevator passes its exact schema contract. V2 forbids
  legacy threshold geometry.
- `elevator_release_loader` is the non-moving runtime preparation boundary. It
  fixes `.elevator_config/current` once (or accepts an exact release ID), reads
  only `releases/<release_id>/`, verifies the six published files and legacy
  release identity, requires each map binding to carry an unquoted positive
  `uint64` `asset_epoch`, the exact
  `njrh-map-asset-bundle-v1` digest contract, and a canonical SHA-256 digest,
  cross-checks those exact identities between configuration and manifest, cross-checks
  configuration/topology/internal poses, deterministically selects an
  elevator, and returns one frozen source/target plan containing both source
  and target map epochs/digests. Schema v2 and v3 can become runtime plans;
  a valid v1 release returns `LEGACY_READ_ONLY`. On Jetson/Linux it uses
  a pinned directory FD plus `openat(O_NOFOLLOW)`, rejects soft/hard links and
  files over 2 MiB, and detects files changed while being read.
- `threshold_footprint` remains available to validate historical schema-v1
  geometry, but it is not part of the schema-v2 FSM or runtime plan. It classifies an already transformed, complete robot
  footprint against a directed door threshold. `INSIDE` and `OUTSIDE` require
  every vertex to clear the configured margin and remain between both door-jamb
  clearances. A crossing, clearance-band contact, jamb violation, malformed
  footprint, or malformed threshold is fail-closed as `STRADDLING`/invalid.
- `elevator_fsm` emits one external port effect for each accepted event. Every
  effect carries a path-safe transaction ID and monotonically increasing
  sequence; a foreign transaction is ignored and a stale completion cannot
  advance a newer step. Mock call-button, target-button, door, and ride effects
  follow the same effect boundary intended for later arm/vision adapters.
  `ElevatorRoute` carries its content schema. V2 retains the historical
  three-point sequence. V3 emits the commissioned four-point reverse-entry
  sequence and propagates the configured panel side with the future
  arm/vision button effect. V1 is rejected as `legacy_read_only` before an
  effect is emitted.
- After the source-hall safety hold is confirmed, the FSM calls the hall
  button and then establishes the transaction-owned operating mode. It never
  emits `ACQUIRE_EXECUTION_LEASE` or `RELEASE_EXECUTION_LEASE`; those enum
  values remain only so historical journal strings can still be classified.
  Every motion segment is still released only under the exact mode contract,
  and the operating mode is released under the target-landing exit hold.
- A port failure, cancellation, out-of-order event, or stale completion enters
  `FAILURE_CLEANUP` and emits `HOLD_AND_CANCEL`. A failed cleanup acknowledgement
  re-emits that effect with a new sequence. Snapshot schema v3 durably records
  `failure_origin_state`, `failure_origin_effect_kind`, and the cleanup runtime
  identity. Every ordinary failure through `SWITCHING_FLOOR` cleans against the
  exact source-floor identity because the live runtime has not committed the
  target yet. Failures from `VERIFYING_FLOOR_READY` through finalization clean
  against the exact target-floor identity. Once the adapter reconciles owned
  actions/resources and proves the transaction hold release, the module
  terminates `FAILED`/`CANCELLED`; it does not create a persistent task lock.
  `RETAIN_LOCK` is reserved for internal cleanup/storage states whose runtime
  identity cannot be established, not normal elevator-stage failure.
- A prepare rejection may bypass cleanup only when the runtime adapter
  explicitly classifies it as rejected before every runtime effect. That
  proven-zero-effect path persists `FAILED / RUNTIME_PREFLIGHT`, preserves the
  original failure code, and acquires no hold, mode lease, or
  localization pause. The execution wrapper never infers this classification
  from an error-code string. Missing or unknown disposition remains
  cleanup-required and fail-closed.
- A locked FSM has no in-process reset and is a maintenance state for corrupt,
  unsupported, or genuinely unreconciled evidence. It is not the normal result
  of a navigation, doorway, confirmation, ride, floor-transition, or cancel
  failure.
- The journaled execution wrapper does not let a new transaction replace a
  terminal `FAILED`/`CANCELLED` snapshot while that snapshot still proves an
  owner-scoped safety hold, while its hold state is unknown, or replace any
  `LOCKED` snapshot. `start` returns
  `ELEVATOR_EXECUTION_RECOVERY_REQUIRED` and preserves the original journal
  identity until recovery is explicitly reconciled. Only a terminal snapshot
  with authoritative `safety_hold_state_known=true` and
  `safety_hold_active=false` and, when runtime effects were applied,
  `runtime_resources_reconciled=true` may be replaced.
- A nonterminal or retained-hold journal found after process restart is never
  resumed as a mission. Recovery re-establishes the exact sequenced owner hold,
  always reconciles unknown Nav2 action state, and reconciles FloorSwitch only
  after the durable event stream contains `RUNTIME_EFFECT_INTENT` for
  `BEGIN_FLOOR_TRANSITION` or `SWITCH_FLOOR`. Unknown/legacy checkpoints remain
  conservative and require FloorSwitch. A proven pre-transition checkpoint
  such as `NAVIGATING_HALL_CALL` therefore cannot be locked merely because the
  floor Action server is absent. Recovery still proves fresh
  wheel/local-odometry settle. A current-schema historical normal-stage lock
  with failure code `ELEVATOR_EXECUTION_ON_SITE_SERVICE_REQUIRED` is first
  converted to `SOURCE_OUTSIDE` or `TARGET_OUTSIDE` only when its journaled
  current floor/map exactly matches that side, then ordinary cleanup finishes
  it as `FAILED`. Other locked evidence stays in maintenance recovery.
  Startup-only endpoint unavailability is counted separately from cleanup proof
  failures: the exact owner hold is reacquired first, then each action server
  actually required by the durable transaction phase receives at most 40
  bounded restart attempts. The ordinary
  startup class also includes a single submitted cancel-all response that is
  still pending. The runtime port must retain that request future between
  attempts and must not submit another cancel-all until the first response is
  proven. The ordinary three-attempt limit still applies to genuine
  cleanup-proof failures. If that
  startup window was exhausted, the next production full-chain cold start
  finalizes only the exact current-schema `ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN`
  ordinary-stage snapshot whose current floor/map exactly matches its
  `SOURCE_OUTSIDE` or `TARGET_OUTSIDE` disposition. It becomes retryable
  `FAILED`, records `NORMAL_STAGE_LOCK_CLEARED_ON_COLD_START`, and does not
  claim odometry-stop evidence. `RETAIN_LOCK`, prepare, identity mismatch,
  corrupt journal, and storage/audit failure shapes remain locked.
- Explicit recovery accepts only the exact `transaction_id`,
  `expected_state=LOCKED`, `effect_sequence`, `operator_id`, and `reason`. It is
  available for a strict historical preflight-orphan whitelist and for an
  outside failure whose runtime resources remain unreconciled. One additional
  field-recovery case is intentionally narrow: a retained
  `ENTERING_CABIN + NAVIGATE_TO_POSE` failure may be reclassified to
  `SOURCE_OUTSIDE` only when the starting operator explicitly supplies
  `source_outside_confirmed=true`, the confirmed/current/source floor IDs are
  identical, and the exact transaction hold is known active. All other cabin,
  doorway, target-landing, and floor-transition dispositions require on-site
  service rather than a remote reset.
  The single recovery request runs asynchronous
  journal-first `RECOVERY_VERIFYING` and `RECOVERY_RELEASE_PENDING` phases.
  `RECOVERY_VERIFYING` remains `LOCKED`; the durable release-pending
  checkpoint is `FAILURE_CLEANUP`, non-terminal while the owner hold is still
  retained, so a restart rebinds and finalizes instead of treating it as a new
  operator lock. Recovery acquires a hold, cancels unknown actions, correlates
  non-empty cancel responses with post-response terminal
  evidence for the exact action goal IDs, prove Nav2/teleop/mapping/docking
  idle, prove no execution session, mode lease or localization pause, prove
  fresh wheel/local odometry settled, run the final cancel barrier immediately
  before release, release only this transaction's hold, and prove its absence.
  Only then is `FAILED / RECOVERY_COMPLETE` persisted atomically. The on-site
  confirmation is first persisted as `ON_SITE_SOURCE_OUTSIDE_CONFIRMED` with
  operator, floor, reason, and sequence; it is evidence to start verification,
  never proof that release already succeeded. Any failed proof or write leaves
  `LOCKED`. That observation remains durable across later effect-sequence
  advances. A retry of the same `SOURCE_OUTSIDE` cleanup reuses the recorded
  observation, does not append a second field claim, and still completes only
  after every runtime, stop, hold-release, and hold-absence proof succeeds.
- Explicit recovery also creates immutable, sequence-scoped audit snapshots
  beside the active journal:
  `history/<transaction_id>.<effect_sequence>.recovery_release_pending.yaml`
  must be durably installed before the physical hold release is called, and
  `history/<transaction_id>.<effect_sequence>.recovery_complete.yaml` is
  durably installed only after hold absence and dual-odometry stop are proven.
  Audit files are exclusive-create and never overwritten. A collision or any
  file/directory durability failure leaves the active transaction `LOCKED`;
  a crash/retry may reuse an existing path only when its deterministic bytes
  and parsed checkpoint/transaction/sequence/snapshot semantics match exactly.
  Phase-aware automatic cleanup uses parallel `cleanup_release_pending` and
  `cleanup_complete` checkpoints. Restart rebinds the exact context, re-proves
  all barriers, and strictly creates or verifies the pending audit before a
  physical release. Complete-audit replay compares the full safety and frozen
  identity subset while permitting only timestamp/event/error/detail
  differences, so a crash after physical release cannot create a false audit
  collision.
- The active journal keeps a v1 compatibility envelope and writes snapshot
  schema v3. UNKNOWN hold state is projected as active for a previous reader,
  so a software rollback remains fail-closed. Only the field-observed v1/v2
  preflight-orphan shape (`LOCKED`, sequence 2, the exact ordered five-event
  and five-error prepare/cleanup history, and current source identity) is
  migrated to `SOURCE_OUTSIDE`; any extra, missing, duplicated, or reordered
  record keeps the legacy transaction at `RETAIN_LOCK`. A strict legacy
  `COMPLETE` shape requires the exact ordered 60-event FSM history, no pending
  confirmation or error, hold absent, and current target identity before it
  migrates
  `runtime_resources_reconciled=true`. Missing legacy `runtime_applied` remains
  conservatively interpreted as possibly applied. An unreadable or unsupported
  journal exposes a separate recovery interlock even when no snapshot can be
  rendered.

The active recovery journal is evidence. Operators and maintenance tools must
not delete or edit `maps_release/.elevator_test/active.yaml` to clear an
interlock.

`HOLD_AND_CANCEL` is an instruction to the future ROS adapter, not an action
performed by this library. Every non-`NONE` effect has `accepted=true`; an
invalid start request produces no effect before motion begins. The adapter must
first obtain/maintain a
`robot_safety` stop hold, then cancel its single active Nav2 goal and wait for a
terminal/settled result. It must not release the hold merely by destroying and
recreating the FSM.

## Schema-v3 reverse-entry mock flow

The core sequences these boundaries without executing them:

1. Navigate to source `hall_call` and acquire a stop hold.
2. Mock press call using `hall_call_panel_side`, establish the exact
   transaction-owned `ELEVATOR_WAIT` mode, release
   hold, turn to the commissioned `landing.yaw` (approximately 180 degrees
   from `hall_call.yaw`), adjust longitudinally, then move laterally to
   `landing`.
3. After the source door-open confirmation, enter `DOORWAY` and reverse from
   `landing` to the aligned source `cabin` pose.
4. Move laterally from `cabin` to `cabin_panel` in the configured
   `cabin_panel_side`, acquire hold, and mock press the target-floor button.
5. Pause localization corrections, enter `ELEVATOR_RIDE`, mock ride and target door.
6. Ask floor manager to acquire its own correction-pause lease and invalidate
   the source runtime context; only after that acknowledgement release the
   elevator-owned correction pause. Effective pause therefore never drops while
   source-floor assets are still active.
7. Run the floor-switch transaction for the exact target floor/map, perform
   target-map Isaac coarse relocalization plus AMCL tracking/static-standby
   readiness, and prove a new fresh stable target-map robot pose while the
   safety hold remains active. The source and target `cabin_panel` coordinates
   are not compared and the robot is not assumed to stand exactly on either
   commissioned panel point after the switch.
8. Enter `DOORWAY` and send the target-floor `cabin` goal. Nav2 uses the live
   target-map robot pose as its start, converges to `cabin`, then releases hold
   and moves forward to the target `landing`.
9. After the target landing reports settled, acquire hold and release the exact
   operating-mode lease back to `NORMAL`, then release hold and complete.

`landing` has one physical map position but two directional runtime intents.
The FSM emits `SOURCE_LANDING_FACE_CABIN` on entry and
`TARGET_LANDING_FACE_HALL` on exit. The configured source `landing.yaw` is the
commissioned ingress door axis and is reused by `enter_cabin`; the adapter must
not substitute the `landing -> cabin` chord because the cabin target may be
laterally offset. Exit keeps the directed `cabin -> landing` heading so the
same stored ingress yaw is not incorrectly reused in the opposite direction.

Schema v2 remains executable for existing published releases but is not the
commissioning target for the reverse-entry workflow. The example in
`config/elevators.example.yaml` documents schema v3 for
`maps_release/<building_id>/elevators.yaml`. The top-level
`mock_ports_enabled` key is required, parsed, and retained in
`ElevatorTopologyCatalog`; the checked-in example sets it to `false`.
`ElevatorFsmOptions` also defaults to `false`, so production-port startup stays
fail-closed. Unit tests and the non-moving mock scenario must opt in explicitly
with `ElevatorFsmOptions{true}`. The loader remains pure C++. The manual
commissioning runtime adapter lives in `robot_api_server`; it pins the release,
constructs the FSM with production ports, and selects the elevator-scoped Nav2
behavior tree only for landing/cabin transitions. This package still publishes
neither Twist nor TF.
These role poses are internal elevator waypoints; the ordinary delivery API
must not expose them as normal delivery destinations.

## Validation boundary

The package GTests cover schema-v1/v2/v3 topology and YAML validation, historical
threshold classification, ordered effects, transaction/sequence fencing,
fail-closed production ports, immutable release loading, and retryable safety
cleanup, including a crash/reload at the legacy
`RECOVERY_RELEASE_PENDING` checkpoint. Release-loader tests cover selector
pinning, legacy publisher FNV
vectors, strict metadata JSON, positive exact `uint64` map epochs, canonical
map SHA-256, symlink/hard-link/path escape rejection, size bounds,
topology/configuration/internal-pose
cross-checks, stable automatic elevator selection, and frozen-return behavior.
The non-moving
cross-floor scenario additionally composes the mission, floor-transition,
safety, mode, and correction-pause pure cores and verifies the successful
synthetic path never emits an execution-lease effect and leaves no goal, hold,
mode lease, or pause.

No test in this package starts a ROS action server, calls live Nav2, reloads a
real floor, publishes Twist or TF, or moves the chassis.

## Restart lock recovery contract (2026-08-05)

Runtime floor/map identity and physical elevator occupancy are now separate
facts. `physical_zone` is durably tracked as `SOURCE_OUTSIDE`, `DOORWAY`,
`CABIN`, `TARGET_OUTSIDE`, or `UNKNOWN`; `interrupted_state` and
`interrupted_expected_confirmation` preserve the real step hidden by a
restart `LOCKED` state. Older schema-v3 journals recover these fields only
from an exact `OPERATOR_CONFIRMATION_REQUIRED` audit record.

Restart reconciliation may automatically release a transaction hold only
when both the cleanup identity and the physical zone are proven outside. A
`CABIN`, `DOORWAY`, or `UNKNOWN` checkpoint remains stopped and locked even
when all runtime resources are already idle. The module exports
`allowed_recovery_actions()` so clients do not infer unlock authority from a
failure code. `CONFIRM_SOURCE_OUTSIDE_AND_RELEASE` requires the exact source
floor plus independent operator confirmations for outside position,
stationary vehicle, and clear door zone; the runtime port must still prove
idle actions, exact assets, dual-odometry stop, and the owner hold before the
journal-first release phase begins. A persisted confirmation can expose only
`RETRY_SAFETY_VERIFICATION`, never a second invented field observation.

The runtime adapter also scopes action recovery from durable effect intent.
Before `BEGIN_FLOOR_TRANSITION`/`SWITCH_FLOOR`, no FloorSwitch Action could
have been submitted. If a full-runtime restart has rebound that pre-floor
failure to the other endpoint of the same frozen release, cleanup may use that
currently confirmed endpoint after exact asset/localization/TF proof and fresh
dual-odometry stop. A different building/floor/map is rejected. Once a floor
intent exists, this endpoint reconciliation is disabled and the recorded
outside-floor identity plus FloorSwitch terminal proof remain mandatory.

The bounded restart-startup retry class also includes correction-pause
timeout, operating-mode timeout, and runtime-context-not-ready. Identity
mismatch and unknown action outcomes remain ordinary three-attempt failures.

## Real hardware work still required

- Survey and save `hall_call`, `landing`, `cabin`, and `cabin_panel` on every
  served floor; record both panel sides relative to the robot heading at the
  corresponding commissioned pose.
- Before rear-lidar installation, validate every reverse ingress with an empty
  stationary car and an operator at the emergency stop. Source code support
  for reverse entry does not prove rear blind-zone coverage.
- Verify that each landing-to-cabin route enters and exits the open elevator
  reliably; Nav2, live obstacle sensing, collision monitor, and robot safety
  remain responsible for motion clearance.
- Validate the deployed `robot_api_server` ROS port adapter against the real
  Nav2, owner-scoped safety, operating-mode, correction-pause, and atomic
  floor-transition endpoints.
- Connect the frozen authoritative map epoch/digest to real localizer reload
  identity/evidence, target-epoch costmap readiness, and explicit recovery
  transaction before converting a frozen release into a `FloorSwitchGoal`.
- Connect arm/vision button and door observations through the mock effect
  boundary, then validate timeouts and correlation IDs.
- Test entry/exit with an empty stationary elevator before any loaded or public
  operation.
