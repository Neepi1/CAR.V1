# Ranger MPPI response prediction (2026-09-08)

## Scope and reuse

`chassis_dynamics` links the installed Humble 1.1.19 MPPI optimizer; it does not
vendor, patch or upgrade Nav2. `RangerMPPIController` supplies the same Controller
interface and follows upstream lifecycle, costmap/parameter locking, path
handling and exception behavior. Its optimizer subclass replaces the virtual
motion model and finalizes filtered controls under the active constraints.
The original model stays alive because Humble's
parameter handler retains references into it. Apache-2.0 upstream attribution
is retained in the adapter source.

`FollowPath.primary_controller` selects this adapter. It affects ordinary
Ackermann travel, predock transit and any distant hall approach using FollowPath.
RotationShim startup/final yaw, terminal pose handoff, RPP, elevator-specific
controllers, docking contact/undock, safety and the chassis driver are unchanged.
The same optimizer no-control exception still reaches the existing wait/retry
handler. No new stop condition, low-speed cutoff, recovery or safety gate is added.

## Identification, not assumed hardware specifications

The original captures are on Jetson under
`/workspaces/njrh-v3/workspace1/reports/ranger_chassis_dynamics_test/`:

| Dataset | Role | SHA256 of samples.csv |
| --- | --- | --- |
| 20260630T011801Z_standard_low_mid_v2_standard | fit | 6bf96c31c6543f38ac3bf6827d019406947a2c2a21150d4de4c1cbc623100290 |
| 20260630T011908Z_standard_low_mid_v2_standard | held out | 6cd2dabcf9ccf179179c242d8b431d905ff8ad969ced744ad049136c63000192 |
| 20260630T013101Z_linear_1p2_nav_speed_v1_linear | fit | 08c79b1c0fa6ad4a41507fa183a3bed726660172052ed593f4773b89eb4c73ec |
| 20260630T013457Z_linear_1p2_nav_speed_v1_linear | held out | 660d56668fd554c080f495a0856cfa6c1fc62a5341ea67da9b7e59e5651d9191 |

`tools/fit_chassis_response.py` is offline, reads final `cmd_vx/cmd_wz` and CAN
feedback, and splits the captures BEFORE fitting. Script-input-to-final-command
transport time is therefore not counted again as plant delay. A bounded delay
grid and least squares identify a delayed, slew-limited first-order longitudinal
model, and a delayed first-order inner-steering-angle model. The steering
conversion uses the current Ranger double-Ackermann geometry, not a bicycle
`atan(L*w/v)` approximation.

| Coefficient | Fitted value used |
| --- | ---: |
| Linear delay | 0.11 s |
| Linear time constant | 0.01823 s |
| Acceleration slope | 0.61888 m/s² |
| Deceleration slope | 1.80150 m/s² |
| Steering delay | 0.11 s |
| Steering time constant | 0.05402 s |
| Wheelbase / track | 0.494 / 0.364 m |

These are identified approximation coefficients, NOT certified maximum
acceleration, braking, steering rate or pure electronics latency. In particular
the 1.80 m/s² plant coefficient does not replace the existing output smoother's
0.95 m/s² deceleration. Held-out longitudinal RMSE is about 0.011–0.025 m/s;
held-out steering-step RMSE is about 0.0048–0.0053 rad. Signed braking integrals
in the replay differ from the old summary's positive-only CAN integration when
the recorded signal has a small negative stop tail. No physical ground-truth
distance is inferred from CAN alone.

## Prediction chain

For each sampled control sequence:

1. Begin with controller-server's current measured body velocity, preserving
   odometry as the initial state rather than using the previous requested speed.
2. Predict the existing OPEN_LOOP smoother using its signed X/Z acceleration
   and deceleration limits. The config contract checks both copies against the
   actual smoother; limits are NOT applied again to the returned command.
3. Apply the identified plant response. Already-issued final `/cmd_vel` history
   supplies inputs in the initial delay interval. Fresh `/cmd_vel_nav` initializes
   the smoother. These two small Twist subscriptions have bounded storage and
   no publishers. Missing/stale feedback falls back to odometry initialization;
   it never blocks navigation or produces a new stop gate.
4. Integrate predicted body velocity/yaw into trajectories. The unchanged native
   obstacle critics and RangerClearanceCritic score these trajectories, not the
   unexecuted velocity requests. Prediction visualization uses the same model.

Prediction state is independent for every rollout and every optimizer retry.
Hot `setPlan` updates only the path; it does not reset physical response or
restart an action. The goal checker, 0.06 m / 0.05 rad tolerances, fixed velocity
chain, minimum turning radius, costmaps and all obstacle envelopes remain intact.

## Evidence limits and verification

- At zero body speed, odometry cannot reveal wheel angle. The model explicitly
  initializes neutral steering there; while moving it derives steering from
  observed body curvature. It does not claim to observe the physical wheel angles.
- The data cover forward 0.20/0.60/1.20 m/s and the recorded steering steps, not
  a complete large-angle slew, micro-speed deadband, load/slip or mode-transition
  identification. This is an Ackermann prediction model, not a three-mode model.
- Delay resolution in production is bounded by `model_dt`; a discrete model is
  an approximation, not a braking certificate. Current load and floor conditions
  still need supervised verification. StopZone safety claims are unchanged.
- Unit tests cover plant braking, rise, signed geometry and issued-command history.
  `replay_chassis_can` runs the SAME production C++ plant over raw CSV captures.
  MPPI tests compare native/measured braking trajectories, load the real plugin,
  execute synthetic-map optimization, hot path updates and speed limits, and
  report the 1200x48 model/full-controller timing. Existing repair tests verify
  no-control waiting still recognizes both native and measured MPPI.
- `setSpeedLimit` retains Humble's absolute/percentage/reset parameter semantics.
  Post-filter finalization below now prevents its previous filter overshoot
  (an isolated 0.20 m/s limit produced about 0.204 m/s).
- ROS tests run on localhost domain 212 with no chassis or navigation goals.
  Reports/builds are in `/tmp/njrh_reports/mppi_dynamics_20260908`.

## Post-filter output constraints

The installed Humble 1.1.19 filter uses signed nine-point weights. Constraining
the sequence only BEFORE filtering does not constrain its output. The isolated
counterexample uses 48 future zero velocities and history `[0.2, 0, 0, 0]`:
native filtering produces `vx[0]=-0.0181818` and selected `vx[1]=-0.0042503`.
These are synthetic reproduction values, not reconstructed pre-filter telemetry.
The field trace `20260908T134925Z_sidepass_v3_NtrMlW` independently contains tiny
negative raw commands despite `vx_min=0`; this fix targets that specific defect,
not every observed zero or steering movement.

Finalization now runs once after native optimization/fallback:

1. Apply the installed native filter.
2. Project the WHOLE sequence into the current signed velocity bounds and native
   motion-model constraints, including Ackermann minimum radius. Nonholonomic Y
   is zero; zero X cannot retain Ackermann yaw. No minimum moving speed is added.
3. Commit history once with the selected projected command (index 1 when shifting,
   otherwise 0), retaining the three actually issued older controls unchanged.
4. Use native command selection and horizon shifting. Warm-start prediction and
   optimum visualization consume this same corrected sequence.

The small `evalControl` adapter follows
[Humble's control flow](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp);
sampling, critics, retries and exceptions still use the installed implementation.
No system Nav2 binary or header is patched. `RangerDynamics.enabled=false` disables
only measured prediction, not output constraints. Selecting the original native
plugin bypasses BOTH changes, including this bug fix.

This is control-constraint consistency, not post-filter collision certification:
as in native Humble, the finalized sequence is not rescored after filtering.
Existing costmap critics and the downstream collision/safety chain remain intact.
It does not prove that positive micro-motion, steering chatter or repeated
blockages are resolved; those need separate recorded hardware acceptance.
Build and regression evidence belongs in
`/tmp/njrh_reports/mppi_output_constraints_20260908`.

## Activation and rollback

Only verified, allowlisted files/libraries are synchronized. Loaded library
inodes must never be overwritten; deployment uses sibling files and atomic
replacement. No node or runtime restart is part of this implementation turn.
The candidate becomes active only after a separately authorized complete
`njrh-runtime.service` restart. Until then, the existing process uses the old
model. To roll back, select `nav2_mppi_controller::MPPIController` as
`FollowPath.primary_controller` in both configs and perform the same authorized
complete restart. `RangerDynamics.enabled=false` also retains the native model
inside the adapter for isolated A/B testing.

Hardware acceptance: first a clear-space low-speed ordinary route with start,
braking, turns and terminal completion, then predock transit and the agreed
dynamic-obstacle scenarios. Record prediction/control deadlines and command vs
actual velocity. Repeated micro-motion is a separate next change; this phase
does not claim to resolve it.
