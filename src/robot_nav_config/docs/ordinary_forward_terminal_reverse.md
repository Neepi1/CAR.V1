# Ordinary forward tracking and terminal reverse

The 2026-09-07T17:12:45Z observation showed ordinary MPPI repeatedly requesting
about -0.08 m/s after local path repair. Collision monitor passed reverse, but
the final arbiter rejected it outside the terminal reverse-permit window.
The robot remained stationary and Nav2 aborted for lack of progress.

## Paired correction

- Both ordinary FollowPath configurations use `vx_min=0.0`. Negative trajectories
  are excluded from MPPI optimization, not merely zeroed after optimization.
- The existing terminal handoff additionally admits an overshot target behind
  the robot (`forward_m < 0`) while XY error exceeds the unchanged 0.06 m goal
  tolerance. Admission still requires finite inputs, enabled handoff, distance
  <= 0.40 m and absolute forward error <= 0.15 m.
- This additional entry does not require a lateral residual. The existing lateral
  entry/dominance/hairpin branch is otherwise unchanged.
- Admission leads to the same stop-settle, yaw, lateral and longitudinal stages.
  Reverse is capped at 0.08 m/s and uses the existing controller reverse permit.
  The projected-footprint check and full final velocity chain remain unchanged.

The shared velocity smoother must retain its negative velocity limit for those
explicit terminal/elevator/docking commands. Do not set it to forward-only.
Safety permissions, final pose tolerances, StopZone, progress timeout, elevator
profiles, and fine docking are not changed. Pre-dock travel shares ordinary
FollowPath and receives this correction; near-contact docking does not.

## Verification and limits

The admission-to-control regression starts from a pure 9.6 cm overshoot with
zero lateral error, then checks bounded reverse plus its permit and completion
under deterministic synthetic odometry. Other cases cover mixed overshoot,
an already-accepted pose, a forward target, disabled handoff, and targets outside
the existing recovery envelope. Existing lateral/yaw/settling tests remain.
Configuration tests bind ordinary MPPI to forward-only while preserving terminal
reverse and smoother compatibility.

These are offline regressions, not measured braking or dynamic-person acceptance.
After the next authorized full runtime restart, supervised validation must
cover moving-obstacle detours, obstacle removal, pure overshoot, lateral terminal
closure and pre-dock approach. This resolves the demonstrated planning/execution
contract mismatch; it does not prove every dynamic-obstacle route will succeed.
