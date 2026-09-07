# Goal-scoped Rotation Shim

## Problem

ROS 2 Humble's stock `RotationShimController::setPlan()` marks every received
path as new. The ordinary navigation BT intentionally replans at 1 Hz, so an
unchanged navigation target could repeatedly re-arm startup rotation and make
the Ranger alternate between `SPINNING` and `DUAL_ACKERMAN`.

## Runtime contract

`robot_nav_config::GoalScopedRotationShimController` is a thin Humble-compatible
wrapper around the stock controller:

- every path update is still forwarded to the MPPI primary controller;
- startup heading alignment is armed once for a navigation goal;
- replans whose final goal remains within `0.01 m` and `0.01 rad` do not re-arm
  startup rotation after MPPI has taken control;
- a changed goal re-arms startup rotation;
- terminal goal-heading rotation remains controlled by
  `rotate_to_goal_heading=true` and is not suppressed;
- terminal pure-yaw commands are limited by
  `sqrt(2 * max_angular_accel * remaining_yaw)` when
  `terminal_rotation_braking_enabled=true`; this backports the upstream Nav2
  stopping envelope missing from Humble 1.1.19 without changing MPPI commands;
- after a controller idle interval, an identical target can be treated as a new
  action attempt.

## Terminal pose handoff

The wrapper also owns the bounded terminal case that an Ackermann trajectory
cannot close cleanly. It does not cancel the Nav2 action or hand control to the
API. While `FollowPath` remains active, the wrapper may replace the MPPI command
with a terminal command when all of these conditions hold:

- map-frame distance is at most `0.40 m`;
- body-frame forward residual is at most `0.15 m`;
- the lateral residual is at least `0.06 m`; this commissioned threshold keeps
  the observed `0.079 m` predock residual eligible instead of leaving MPPI to
  repeat Ackermann corrections; and
- either the residual is lateral-dominant or the remaining path has the
  configured hairpin length/cross-track evidence.

The controller then executes exactly one axis at a time: yaw, lateral
side-slip, forward or bounded reverse, and finally a stopped-state settle. The
commercial goal remains `0.06 m` / `0.05 rad`; terminal handoff does not widen
the goal checker. Every translation is checked against the local costmap. A
fresh `/ranger_mini3/nav_terminal_lateral_enable` or
`/ranger_mini3/nav_terminal_reverse_enable` lease is required for
`robot_safety` to pass the corresponding normal-navigation command. Those
lifecycle publishers are activated and deactivated with the controller so an
inactive plugin cannot leave a stale capability behind.

Return-to-dock now selects the same `goal_checker` as ordinary pose-required
navigation. Nav2 must reach both `0.06 m` XY and `0.05 rad` yaw at the
commissioned predock pose before its action may succeed. The separate
`DockStagingGoalChecker` remains compiled and registered only for compatibility
and diagnostics; no active behavior tree selects it. If it is used during an
explicit rollback, its reported scalar tolerance remains the largest inscribed
circle of its asymmetric rectangle, never the circumscribed radius.

MPPI itself remains Ackermann (`vy_max=0`). The narrow Y range in
`velocity_smoother` exists only to carry a permitted terminal side-slip through
the normal Nav2 command chain. The API's proactive near-goal cancellation is
disabled and remains only a post-abort fallback.

This changes neither the 1 Hz planner rate nor the command chain. Commands still
flow through `velocity_smoother`, `collision_monitor`, `robot_safety`, and
`ranger_base`.

Downstream `robot_safety` treats a fresh normal Nav2 command as an active
stream even while the confirmed chassis mode is PARALLEL. Its idle mode-exit
timer may publish zero only after that stream is stale; otherwise timer zeros
would interrupt the bounded terminal side-slip and stretch it beyond the
handoff timeout. The isolated regression is
`robot_safety/test/run_isolated_normal_lateral_watchdog_smoke.sh`.

This ordinary near-goal handoff is separate from the elevator-scoped profile.
The elevator runtime selects a dedicated behavior tree, planner, and controller
for landing/cabin transitions because those paths may begin with a larger
lateral residual and may be rejected by the Ackermann global planner before
ordinary `FollowPath` starts. See
[`elevator_scoped_motion.md`](elevator_scoped_motion.md). The dedicated profile
is not selectable by ordinary App navigation and does not widen this wrapper's
0.40 m activation envelope.

## Rollback

Set `FollowPath.plugin` in both Nav2 parameter copies back to
`nav2_rotation_shim_controller::RotationShimController` and rebuild
`robot_nav_config`. No BT, MPPI, goal-checker, or chassis parameter rollback is
required.

## Hardware validation

After a full `njrh-runtime.service` restart:

1. Verify the controller plugin and parameters without sending a motion goal.
2. When the robot is next cleared for motion, run one route requiring a large
   startup heading change and keep the 1 Hz replanner active.
3. Confirm only one startup `SPINNING` episode occurs before sustained
   Ackermann drive, while path updates continue at about 1 Hz.
4. Confirm final yaw still converges at the target.
5. Repeat with a genuinely different target and confirm startup rotation is
   armed again.
6. For a pre-dock goal, confirm native Nav2 success occurs only after the live
   pose is within `0.06 m` and `0.05 rad` of the commissioned predock pose, then
   confirm the stopped-state handoff occurs before any docking-manager command.
7. Place the robot at a safe 8--15 cm lateral residual inside the terminal
   envelope. Confirm the same `FollowPath` action remains active, lateral output
   reaches `/cmd_vel`, native Nav2 succeeds, and `/cmd_vel_api.linear.y` stays
   zero.
