# robot_nav_config

Fixed Nav2 and canonical TF defaults for the first production-oriented scaffold.

## Parameters

- Planner: `nav2_smac_planner/SmacPlanner2D`
- Optional planner profile reserved for `SmacHybrid`
- Controller: `nav2_mppi_controller::MPPIController`
- Fallback controller: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
- Ranger-matched MPPI: the controller keeps the 1.2 m/s field speed target,
  but measured chassis response is handled by `velocity_smoother`
  in open-loop ramp mode (`smoothing_frequency=30.0`,
  `max_accel=[0.55, 0.0, 0.45]`,
  `max_decel=[-0.70, 0.0, -0.45]`, `max_velocity=[1.20, 0.0, 0.70]`)
  and 1.5 m near-goal critic windows. The primary controller runs at 15 Hz with
  `model_dt=0.0666666667`, `vx_std=0.30`, and `wz_std=0.32` so ordinary
  building navigation uses less jagged sample commands without changing the
  Nav2 plugin types. Closed-loop velocity-smoother feedback is
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
  a mid-route pure-spin/replan chain. `controller_server.failure_tolerance=3.0`
  handles very short MPPI no-trajectory intervals; longer blocked corridors
  should be handled by explicit API/mission wait or retry policy.
- Pre-dock navigation uses `navigate_to_predock.xml`, not the ordinary 1 Hz
  replanning tree. It computes one SmacPlanner2D path per Nav2 action attempt and
  follows that path without replacing it every second. This prevents a
  non-kinematic 2D path's changing first segment from repeatedly re-entering
  RotationShim on the Ackermann chassis. A Nav2 abort is still retried by the
  docking job, which creates a new path on the next bounded attempt.
- BT server waits: `default_server_timeout=5000` and
  `wait_for_service_timeout=8000` keep Nav2 from aborting a valid goal when
  Jetson/Fast DDS/costmap load delays internal action or service acknowledgement.
- Smoother server: `nav2_smoother::SimpleSmoother` remains configured for
  lifecycle compatibility and tooling; the active field BT does not call
  `SmoothPath`.
- Progress checker: `nav2_controller::PoseProgressChecker`
- Goal checker: `nav2_controller::SimpleGoalChecker` with `stateful=false`, plus
  Humble RotationShim's private `.position_checker.stateful=false`,
  so terminal XY and yaw are rechecked together after dynamic-obstacle
  interruption or near-goal avoidance drift. The ordinary pose-required
  tolerances are `xy_goal_tolerance=0.06` and `yaw_goal_tolerance=0.05`.
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
- Controller-native terminal pose handoff: when a pose-required goal is within
  `0.40 m`, body-frame forward residual is within `0.15 m`, and the residual is
  either lateral-dominant or represented by an Ackermann hairpin,
  `GoalScopedRotationShimController` keeps the same `FollowPath` action running
  and serializes yaw, side-slip, forward/reverse, and stop settle. MPPI remains
  Ackermann-only. Side-slip and reverse require fresh controller lifecycle
  permits and remain costmap checked through the standard smoother, collision,
  safety, and chassis chain. The strict `0.06 m` / `0.05 rad` goal gate is not
  widened.
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

- This package owns the policy artifacts, not the live TF publication.
- Wrapper packages must consume this policy and keep non-canonical TF disabled by default.
- `config/local_reuse_sources.yaml` records the current validated `D:/codespace/car` and Jetson workspace inputs used by the wrapper configs.
- The same reuse manifest now also records the Dockerfile and dashboard entrypoints used by the temporary `NJRH-car` runtime on Jetson.
