# robot_nav_config

The optional NAVLITE internal-snapshot candidate records the true per-computation
costmap/inputs and final pre-shift control sequence using a bounded file writer.
Default disabled; it adds no ROS entities and does not change control/safety.
Not activated by editing/building this source. See
[recording and evidence limits](../../scripts/diagnostics/README_nav_event_lite.md)
and [snapshot schema](../../scripts/diagnostics/native_mppi_snapshot_schema.md).

Fixed Nav2 and canonical TF defaults for the first production-oriented scaffold.

Diagnostic-only NAVLITE stop/return events are a local, unactivated candidate.
They do not change hold, footprint, repair or velocity decisions. See
[stop branch audit](../../docs/navlite_stop_branch_audit.md).

## Parameters

- Local geometry candidate: the confirmed physical rectangle is `0.94 x 0.72 m`,
  matching StopZone. Both costmaps retain `0.03 m` padding and use `0.75 m`
  inflation, matched by MPPI and the local keepout inflation layer.
  Clearing/FootprintApproach and repair envelopes grow with the footprint;
  speeds, safety policy and repair margins are unchanged. Not deployed.
  See [geometry, regression scope and hardware gaps](docs/body_geometry_alignment.md).

- Ordinary MPPI enables native Humble `ConstraintCritic` (power 1, weight 4).
  The local costmap shares the existing global keepout mask and applies matched
  post-filter inflation (0.75 m / 6.0) for MPPI footprint checks. Motion limits,
  terminal tolerances and elevator test policy are unchanged.
  See [scope, tests and activation](docs/local_keepout_and_constraints.md).

- Elevator direct-path endpoints use latest TF for Nav2 arrival checking,
  matching the controller's canonical map goal. Path creation time, geometry,
  0.06 m / 0.05 rad tolerances and collision policy are unchanged. See
  [goal timestamp regression](docs/elevator_goal_stamp.md).

- Cabin center -> cabin panel uses the independent `ElevatorCabinPanelFollowPath`
  instance with a `0.20 m/s` lateral cap. Entry, return-center and egress retain
  `0.40 m/s`; ordinary navigation, goal tolerances and motion policy are unchanged.
  See [panel speed isolation](docs/elevator_scoped_motion.md#cabin-panel-speed-isolation).

- Ordinary persistent-recovery candidate: the Ranger recovery tree retains the
  same outer task, retries verified progress failures after 0/5/10-second backoff,
  and allows one startup rearm per actual progress episode. It reports explicit
  wait/recovery status for the API execution budget. This candidate is staged
  only; the preceding MPPI output-filter change still needs physical acceptance.
  No YAML geometry, motion limit, elevator or predock tree changes are included.
  See [recovery protocol](docs/ordinary_navigation_recovery.md).

- MPPI output finalization now reapplies the active velocity and native motion-model
  constraints after Humble's signed Savitzky-Golay filter, before command selection
  and horizon shifting. Its history records the constrained selected command.
  Ordinary forward-only MPPI cannot leak filter-generated reverse commands; current
  speed limits and Ackermann curvature remain enforced. No low-speed stop gate,
  steering-centering change or task-recovery redesign is included. See
  [output constraints](docs/chassis_response_model.md#post-filter-output-constraints).

- MPPI now has a measured-response adapter, `robot_nav_config::RangerMPPIController`,
  in the independent `chassis_dynamics` module. It retains Humble's optimizer and
  critics but predicts the existing smoother plus CAN-identified acceleration,
  braking and steering response before scoring trajectories. No new stop gate or
  micro-motion suppression is added. Predock transit shares this FollowPath;
  elevator-specific and contact docking control are unchanged. See
  [response model](docs/chassis_response_model.md) for coefficients, evidence
  limits, isolated tests and separately authorized activation/hardware validation.

- The 2026-09-08 clearance candidate keeps the scan mask and padded physical
  footprint at `0.39/0.28 m`, changes StopZone to `0.47/0.36 m`, and adds a
  separate finite MPPI planning preference at `0.52/0.41 m`. Local repair first
  searches that preferred envelope with a bounded fallback to its original
  hard clearance. No footprint-clearing enlargement or new stop gate is used.
  Near-goal control, elevator profiles and docking contact control are unchanged.
  See [planning clearance](docs/planning_clearance.md); activation and moving
  validation require separate explicit authorization.

- Ordinary Ranger navigation now selects
  `navigate_to_pose_ranger_lattice_recovery.xml`. A real progress-checker abort
  gets at most one internal replan/startup-alignment/FollowPath retry while the
  original NavigateToPose stays active. Humble has an empty FollowPath result,
  so a Nav2-private preparation service checks the actual progress-failure
  evidence and binds rearming to the exact freshly planned path. Normal hot
  path updates do not rearm rotation. Predock, elevator and the Smac2D baseline
  trees are unchanged. See [ordinary task recovery](docs/ordinary_navigation_recovery.md).

- Planner: `nav2_smac_planner/SmacPlanner2D`
- Optional planner profile reserved for `SmacHybrid`
- Controller: `robot_nav_config::RangerMPPIController` (installed Humble MPPI optimizer)
- Fallback controller: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
- Ranger-matched MPPI: the controller keeps the 1.2 m/s field speed target.
  Candidate trajectories now predict the measured chassis response together
  with the unchanged `velocity_smoother` open-loop ramp (`smoothing_frequency=30.0`,
  `max_accel=[0.55, 0.20, 0.90]`,
  `max_decel=[-0.95, -0.30, -1.10]`)
  and 1.5 m near-goal critic windows. The primary controller runs at 15 Hz with
  `model_dt=0.0666666667`, `vx_std=0.30`, and `wz_std=0.32` so ordinary
  building navigation retains its sampling settings. The optimizer is still
  Humble MPPI, hosted by the Ranger response adapter. Closed-loop velocity-smoother feedback is
  avoided on Ranger Mini 3 because odom=0 plus the chassis motion-mode deadband
  can pin angular output at the single-cycle acceleration increment before the
  chassis starts moving. Normal API point navigation publishes a
  distance-based `/speed_limit` so the robot steps down from 1.20 m/s outside
  2.4 m to 0.08 m/s inside 0.35 m, then restores the cruise limit when the
  Nav2 task exits. It must not publish `0.0` as a clear signal, because
  controller-server treats that as a stop limit.
- Field behavior trees: `navigate_to_pose.xml` runs a 1 Hz `RateController`
  around `ComputePathToPose`, followed by `FollowPath`; `navigate_through_poses.xml`
  applies the same rate limit to `ComputePathThroughPoses`. This preserves
  obstacle-aware global replanning. `GoalScopedRotationShimController` forwards
  every replacement path to MPPI but arms startup heading alignment only when
  the navigation goal changes; a 1 Hz replan for the same goal therefore cannot
  restart the pure-yaw shim. Terminal goal-yaw rotation remains enabled and is
  independent of this startup-only gate. The Savitzky-Golay `SmoothPath` BT
  pass and BT `Wait` recovery were rolled back after field delivery tests showed
  a mid-route pure-spin/replan chain. `controller_server.failure_tolerance=10.0`
  keeps the same `FollowPath` alive while a short-lived pedestrian obstacle
  clears, while remaining below the ordinary 12 s progress-checker bound.
  Collision monitoring uses the user-selected `0.47 x 0.36 m` hard-stop half-envelope,
  independent of MPPI's `0.08 m` soft collision margin, and a 2 s projected
  footprint approach check. It does not clear costmaps, spin, reverse, or create
  a new navigation goal while waiting.
  See [collision monitor geometry](docs/collision_monitor_geometry.md) for clearances
  and the remaining hardware validation.
- Ordinary local path repair closes the `ranger_lattice` dynamic-obstacle gap:
  its global costmap intentionally remains static, so a local-only person does
  not invalidate the BT's retained path. `OrdinaryLocalPathRepairRuntime`
  inspects the next 4.0 m of that path on a detached rolling-local-costmap
  snapshot every 0.20 s. A persistent lethal/unknown intersection runs
  forward-only Dubins Hybrid A* with the same `0.81 m` minimum turning radius
  and the padded rectangular footprint plus the retained `0.08 m` hard repair
  margin. The independent preferred-clearance first pass adds another `0.05 m`,
  with fallback to that original hard envelope. A valid
  prefix is joined back to the untouched path suffix and passed to MPPI through
  the existing `setPlan()` while the same `FollowPath` action remains active.
  A blocked distant reference or missing rejoin does not force zero: MPPI keeps
  computing while the worker retries. Only MPPI's confirmed Humble no-control
  exception requests zero and pauses progress, with computation retried on every
  cycle. Other faults still propagate. Rejoin exits are spatially separated,
  extend beyond the inspection horizon within the local map, and use independent
  graphs with one shared search budget. Returned paths reattach to the latest
  robot pose through a footprint-checked forward connector. Accepted replacements are observable on
  `/ranger_mini3/ordinary_local_repair_path`. See
  [`docs/ordinary_local_path_repair.md`](docs/ordinary_local_path_repair.md).
- Pre-dock navigation uses `navigate_to_predock.xml`, not the ordinary 1 Hz
  replanning tree. It computes one SmacPlanner2D path per Nav2 action attempt and
  follows that path without replacing it every second. This prevents a
  non-kinematic 2D path's changing first segment from repeatedly re-entering
  RotationShim on the Ackermann chassis. A Nav2 abort is still retried by the
  docking job, which creates a new path on the next bounded attempt.
  Its `FollowPath` explicitly selects the ordinary `goal_checker`, so Nav2 must
  finish the commissioned predock pose at `0.06 m / 0.05 rad` before the
  docking job can prove the chassis stopped and hand ownership to the
  near-field controller. The asymmetric `DockStagingGoalChecker` remains
  registered only as an inactive compatibility/diagnostic plugin; no active
  behavior tree selects it.
- BT server waits: `default_server_timeout=5000` and
  `wait_for_service_timeout=8000` keep Nav2 from aborting a valid goal when
  Jetson/Fast DDS/costmap load delays internal action or service acknowledgement.
- Smoother server: `nav2_smoother::SimpleSmoother` remains configured for
  lifecycle compatibility and tooling; the active field BT does not call
  `SmoothPath`.
- Progress checker: `robot_nav_config::ElevatorAwareProgressChecker`; ordinary
  tracking and background local repair retain the original pose-progress limits;
  explicit elevator waits and ordinary MPPI no-control waits pause the timer.
- Ordinary/elevator goal checker: `nav2_controller::SimpleGoalChecker` with
  `stateful=false`, plus
  Humble RotationShim's private `.position_checker.stateful=false`,
  so terminal XY and yaw are rechecked together after dynamic-obstacle
  interruption or near-goal avoidance drift. The ordinary pose-required
  tolerances are `xy_goal_tolerance=0.06` and `yaw_goal_tolerance=0.05`.
  `navigate_to_predock.xml` selects this same checker. The additional
  `DockStagingGoalChecker` is retained but inactive and cannot relax predock
  completion.
- Goal-scoped RotationShim handoff: `angular_dist_threshold=0.45` and
  `angular_disengage_threshold=0.075`. This keeps ordinary path-entry and
  mid-route Ackermann turns smooth instead of stopping for small heading
  changes, but once a pure-yaw shim starts it turns closer to the target before
  handing back to MPPI. `rotate_to_heading_once=true` prevents the same target's
  replanned path from re-entering startup rotation, while a changed target arms
  it again. Startup heading is measured directly in the common path/pose frame;
  a transient `map -> base_link` future-extrapolation error therefore holds zero
  and retries instead of permanently handing an unaligned path to MPPI. Ranger
  spin-tail is handled downstream by
  `robot_safety`, which waits for actual `/wheel/odom` yaw rate to settle
  before releasing the next linear command after a pure spin.
- Humble 1.1.19 terminal RotationShim braking: the project wrapper backports
  the newer upstream stopping envelope for goal-heading-only pure rotation.
  With `terminal_rotation_braking_enabled=true` and `closed_loop=true`, the
  requested `0.60rad/s` is reduced as the remaining yaw shrinks using
  `sqrt(2 * max_angular_accel * remaining_yaw)`. Startup speed and MPPI path
  tracking are unchanged; set the switch to `false` for a one-parameter
   rollback.
- Ordinary MPPI is forward-only (`vx_min=0.0`): terminal overshoot is handled by
  the existing controller-native handoff, not by mid-route MPPI reverse samples
  that the final arbiter cannot execute. The velocity smoother retains negative-X
  support for terminal, elevator and docking motion. See
  [ordinary forward and terminal reverse](docs/ordinary_forward_terminal_reverse.md).
- Controller-native terminal pose handoff: when a pose-required goal is within
  `0.40 m`, body-frame forward residual is within `0.15 m`, lateral residual is
  at least `0.06 m`, and the residual is either lateral-dominant or represented
  by an Ackermann hairpin, or the target is behind the robot and outside the
  XY goal tolerance within the same distance/forward envelope,
  `GoalScopedRotationShimController` keeps the same `FollowPath` action running
  and serializes yaw, side-slip, forward/reverse, and stop settle. MPPI remains
  Ackermann-only. Side-slip and reverse require fresh controller lifecycle
  permits and remain costmap checked through the standard smoother, collision,
  safety, and chassis chain. The strict `0.06 m` / `0.05 rad` goal gate is not
  widened.
- Elevator-scoped motion: the manual elevator runtime selects
  `navigate_elevator_scoped_motion.xml` for `source_landing`, `enter_cabin`,
  and `target_landing`. A `hall_call` within the same 2.5 m envelope and with a
  fresh API map pose selects the separate
  `navigate_elevator_hall_call_scoped_motion.xml`; farther or unproven starts
  retain ordinary Nav2. A clear nearby hall-call route starts with the normal
  in-place turn toward its travel chord. Only a footprint-blocked turn/direct
  route enters the full 16-heading fallback, which may combine translations
  and multiple checked turns before continuing.
  Door/cabin ingress keeps its independently commissioned heading policy. The
  dedicated
  planner rejects requests beyond 2.5 m and searches a 16-heading lattice with
  serial forward/reverse/lateral/spin motion. Cabin entry begins in the live yaw
  proven at the landing pose
  and finishes at the cabin pose's independently commissioned yaw. A fully free
  route remains direct; a hard-blocked route must detour; and a direct route
  crossing soft inflation is now compared with searched alternatives by total
  motion cost and soft-cost exposure. A clearance-improving alternative may
  cost up to 10 percent more before it is rejected. The dedicated controller
  is capped at `0.40 m/s` forward, reverse, and lateral and `0.50 rad/s` yaw;
  the smoother admits those elevator-only commands while its acceleration and
  deceleration limits remain unchanged. `robot_safety` selects the matching
  reverse/lateral limits only during the typed elevator execution session.
  Ordinary `FollowPath` and its terminal handoff retain their existing speed
  caps. The dedicated controller
  compresses the result into axis segments and binds each segment's phase and
  direction to the command generator. Endpoint residual on another axis can
  no longer overwrite a searched detour or a planned intermediate turn. An
  explicit final-pose stage runs only after all searched motion segments finish.
  Schema-v3 source `hall_call -> landing` has its own
  `navigate_elevator_reverse_entry_staging.xml` profile and executes goal yaw,
  lateral alignment, then forward/reverse closure. Later reverse-entry, cabin
  panel, return-center, and egress actions retain the independent
  longitudinal-first reverse-docking profile. The schema-v2/general BTs retry
  only `ComputePathToPose` at most three times; each prescribed schema-v3 BT
  waits up to 30 seconds. Their single `FollowPath` remains outside the retry
  wrapper, so no active controller action is reset and no costmap is cleared.
  Each scoped Nav2 action is fenced by an explicit private controller session
  identified by `transaction_id:effect_sequence`. A new `BEGIN` clears any
  terminal timer, failed state, route index, cached path, blockage history, and
  pending replan generation left by the previous action; periodic plan updates
  inside the same action retain the session and may preserve motion. A matching
  `END` is idempotent, and a stale `END` cannot clear a newer action. A scoped
  controller fails closed when no active session has been admitted.
  Live recovery runs on an immutable costmap snapshot in a dedicated worker,
  never inside the 15 Hz controller callback. A hard-blocked command is held at
  exact zero while the worker runs one bounded 60,000-expansion/1-second search
  from the current pose to the canonical final goal using forward, reverse,
  lateral, and spin transitions. The accepted result replaces the entire
  unfinished route; production code carries no reference suffix, rejoin index,
  or pending-repair lock. Identical costmap/start/segment/block-evidence
  signatures are cached for the action so A/B/A costmap oscillation is not
  searched repeatedly. Changed evidence may immediately produce a later route
  revision even if the previous revision was not completed. The returned path
  is revalidated on the current live costmap before commit. The 3 second
  threshold remains a notice only.
  Every accepted revision is transformed back into the canonical map frame, and
  the final-pose stage always targets the original map goal. A material
  cumulative `map->odom` change (`>=0.08 m` or `>=0.08 rad`) invalidates the
  active directional segments. The controller holds zero, coalesces the burst
  until it is stable within `0.015 m / 0.015 rad` for `0.60 s`, then launches
  one asynchronous full four-wheel-steer route search from the corrected pose.
  This is event-driven rather than periodic, so normal AMCL noise and an
  unchanged transform cannot restart `FollowPath`. Typed progress states pause
  the progress timer only during explicit replanning, blocked waiting, or
  localization settling; returning to tracking resets its baseline.
  Command clearance projects at most 0.15 m for translation and 0.10 rad for
  yaw, but is additionally capped by the active axis segment's remaining
  distance/yaw. It therefore never rejects a short checked segment because a
  fixed lookahead extended beyond that segment's endpoint.
  The command callback waits on the local costmap's normal mutex before checking
  projected footprints. A concurrent costmap update is therefore delay, not a
  synthetic `costmap_busy` obstacle or zero-command pulse; waits over 20 ms are
  logged. Transform, bounds, lethal, keepout, and unknown failures remain
  fail-closed.
  `ElevatorAwareProgressChecker` keeps ordinary navigation at
  `0.03 m / 0.05 rad / 12 s`, but a fresh `TRACKING` state from either elevator
  controller selects `0.015 m / 0.015 rad / 20 s`. This lets a real 0.0407 rad
  fine-yaw correction reset progress instead of aborting just outside the
  unchanged `0.05 rad` goal gate. The scoped profile expires after 0.50 s and
  resets its baseline before ordinary navigation resumes; it never reports
  progress without measured translation or yaw. It uses the same fresh
  lateral/reverse permits and fixed command chain. Shared global/local
  inflation remains `0.60/0.60 m`: the local radius covers the padded
  rectangular footprint's approximately `0.480 m` circumscribed radius plus
  the `0.08 m` MPPI collision margin on the `0.05 m` grid. Derived non-lethal
  costs `1..253` do not
  independently block this special route, but intersecting lethal, keepout, or
  unknown cells still fail closed. Every non-elevator navigation request keeps
  the existing planner/controller and cannot select the scoped profile. See
  [`docs/elevator_scoped_motion.md`](docs/elevator_scoped_motion.md).
- Local obstacle topic: `/scan`
- `suppress_third_party_tf`: true

## Ranger Mini3 State Lattice (Experimental)

The repository now owns an opt-in Ranger planner derived from the stock
`SmacPlannerLattice`, using a lattice generated for
the current 5 cm global costmap, the conservative 0.81 m Ranger controller
turning-radius contract, and 16 heading bins. Its 104 checked-in primitives are
72 forward dual-Ackermann motions plus 32 one-heading-bin in-place rotations.
The lattice may select a spin transition. The existing
`GoalScopedRotationShimController` can execute a path-entry spin and the final
goal heading through `robot_safety` and `ranger_base`; arbitrary mid-path spin
handoff is not yet a V1 runtime capability. The staged gate therefore rejects
plans containing mid-path spins. Lateral motion is not a Lattice primitive or
an MPPI sample; it is available only to docking/API recovery or the bounded
controller-native terminal handoff described above. Long routes remain
forward-only by contract.

For a clear corridor whose goal yaw agrees with the start-to-goal bearing,
`RangerMini3LatticePlanner` replaces the stock Lattice's partial-spin-plus-arc
choice with a footprint-checked straight path. `GoalScopedRotationShimController`
then owns the complete path-entry spin before MPPI receives the straight path.
The shortcut samples every 2.5 cm, requires center cost `0`, checks the full
Ranger footprint, and is disabled for paths shorter than 1 m or goals whose yaw
differs from the corridor bearing by more than 0.20 rad. Any failed sample keeps
the unmodified stock Lattice path, so this policy does not make in-place spin
cheap in the middle of an obstacle-avoidance route.

Stock Lattice endpoint quantization is handled separately from route search.
The wrapper preserves all searched poses and appends the exact requested goal
only across a full-footprint-checked residue bounded to `0.08 m / 0.21 rad`.
This lets the ordinary `0.06 m / 0.05 rad` GoalChecker evaluate the actual
commissioned pose, including predock yaw, rather than the nearest 5 cm / 16-bin
lattice state. A blocked or larger residue fails closed; no planner weights,
motion primitives, MPPI settings, or intermediate path geometry are changed.

The default remains `SmacPlanner2D`. Set the runtime profile explicitly to
`NJRH_NAV2_PLANNER_PROFILE=ranger_lattice` only for the staged A/B procedure in
[`docs/ranger_mini3_smac_lattice.md`](docs/ranger_mini3_smac_lattice.md). The
profile is validated before Nav2 starts and selects a dedicated Humble BT that
retains a valid path, replanning only when the goal changes or the costmap
invalidates that path.

The first 2026-07-21 moving gate rejected V1: 1 Hz replanning changed a
near-goal Lattice branch into 4.49-5.49 m replacement loops, so native Nav2
stalled and the API terminal correction had to finish the pose. A second gate
then exposed a separate Humble RotationShim TF race: a roughly 2 ms future
extrapolation made the stock shim abandon startup alignment after one command.
The Lattice-only valid-path BT and the TF-safe startup alignment guard fix both
failures without bypassing `robot_safety` or changing the Ranger SDK.

The first post-fix field pair passed in both directions. Native Nav2 completed
both approximately 11 m legs with no reverse/lateral commands, no mid-route
spin/drive oscillation, and correct `SPINNING -> DUAL_ACKERMAN` feedback. Final
map-frame errors were 5.94 cm / 0.44 degrees and 5.27 cm / 1.74 degrees. The
profile remains experimental until the ten-leg repeatability, dynamic-obstacle,
narrow-passage, loaded-radius, and braking gates pass.

A later plan-only reproduction used the opposite-heading collinear calibration
points. The stock Lattice path deviated up to 1.286 m because it selected only
about 107.7 degrees of path-entry spin and completed the heading change with a
0.81 m-radius arc, despite all 558 samples on the direct global-costmap centerline
having cost `0`. Lowering `rotation_penalty` fixed that route but introduced
mid-path spin segments in an obstacle case, so the production profile retains
`rotation_penalty=3.0` and uses the bounded direct-corridor policy above.

## Notes

Diagnostic-only MPPI failure/recovery events and isolated test boundaries:
[NAVLITE evidence](../../docs/navlite_mppi_failure_evidence.md). Not deployed.
The 2026-09-20 candidate adds startup schema evidence and validates real plugin
failure/recovery through the lightweight file recorder. See the
[cross-module evidence boundary](../../docs/navlite_chain_evidence.md).

- This package owns the policy artifacts, not the live TF publication.
- Wrapper packages must consume this policy and keep non-canonical TF disabled by default.
- `config/local_reuse_sources.yaml` records the current validated `D:/codespace/car` and Jetson workspace inputs used by the wrapper configs.
- The same reuse manifest now also records the Dockerfile and dashboard entrypoints used by the temporary `NJRH-car` runtime on Jetson.
