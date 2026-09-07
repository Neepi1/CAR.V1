# Docking near-field controller

## Ownership boundary

Return-to-dock is split into two motion domains:

1. Nav2 owns the route to the commissioned predock pose and may succeed only
   when the ordinary `goal_checker` verifies both `0.06 m` XY and `0.05 rad`
   yaw.
2. The Nav2 action reaches a proven terminal result and the API holds zero
   until wheel/local odometry proves the chassis is stopped.
3. `robot_docking_manager` then becomes the only owner of all camera-guided
   near-field motion after `/docking/start`.

`navigate_to_predock.xml`, ordinary point navigation, and all elevator behavior
trees explicitly select the original `goal_checker` with the unchanged
`0.06 m / 0.05 rad` success limits. The legacy asymmetric staging checker is
registered only for compatibility and is not selected by an active tree.
Because the 5 cm / 16-heading Lattice discretizes its terminal state, the
Ranger wrapper keeps the searched route intact and appends the exact
commissioned goal across a bounded, full-footprint-checked terminal residue.
Nav2 therefore evaluates predock yaw against the saved pose, not a quantized
path endpoint.

With `docking_delegate_staging_motion_to_manager=true`, `robot_api_server`
only verifies admission, freezes global corrections, records evidence, and
calls `/docking/start`. It does not acquire the docking velocity owner or
publish yaw/lateral commands. The physical path remains:

```text
robot_docking_manager -> /cmd_vel_docking -> robot_safety -> /cmd_vel -> ranger_base
```

The change does not alter ordinary Nav2, AMCL, EKF, map/odom TF ownership,
charging detection, or the undocking safety contract.

## Monotonic phase schedule

```text
yaw_capture (SPINNING)
  -> vector_approach (PARALLEL vx + vy, wz = 0)
  -> final_approach (PARALLEL, forward speed capped)
  -> contact_verify (straight vx only)
  -> contact_stopping / docked
```

Yaw capture always requires three distinct observations inside `0.5deg`, even
when the first observation is already aligned. Entering a translation phase
latches that successful capture: the contact handoff uses the latched phase,
not a second raw `0.5deg` comparison. Translation therefore does not return to
yaw capture for ordinary camera noise; only the `3deg` re-entry threshold can
request recapture. One large-yaw recapture is permitted. A second large
recurrence stops with `near_field_yaw_realign_budget_exhausted` instead of
alternating indefinitely between chassis modes.

The Ranger Mini 3 driver cannot combine `SPINNING` angular motion with
`PARALLEL` translation. Therefore the controller never publishes nonzero
`angular.z` together with nonzero `linear.x` or `linear.y`. In PARALLEL mode,
forward and lateral commands are combined as a single vector and capped by
`controller.max_parallel_speed_mps`.

## Final insertion and retry

- `controller.final_approach_window_m=0.10` caps visual forward approach at
  `0.05m/s` near handoff.
- Lateral correction remains available until the final `0.06m`, and locks
  only if the lateral residual is already inside tolerance.
- Contact verification is straight-only. It crawls at `0.05m/s` and drops to
  `0.02m/s` only in its last `0.06m`.
- The contact timeout is computed from the configured distance and both speed
  zones, multiplied by `1.5`, plus `2s`, with a `3s` minimum and the existing
  `16s` hard cap.
- A failed insertion backs out by attempted distance plus `0.08m`, clamped to
  `0.20..0.60m`, before reacquiring the observation. Retry count remains two.
- Any non-complete translation decision that would otherwise publish zero on
  all three axes fails closed as `near_field_no_action_available`; it cannot
  remain active indefinitely while silently commanding no motion.

BMS charging contact still causes an immediate zero command followed by
wheel-odometry and motion-state stop confirmation. No timeout is allowed to
declare charging success without that evidence.

## Verification

Automated tests cover distinct-sample initial yaw capture, latched-yaw contact
handoff, unlatched and recapture-threshold boundaries, combined
forward/lateral output, vector-speed capping, yaw hysteresis, one bounded
recapture, monotonic final phase, the final-six-centimeter lateral lock,
actionless-decision rejection, distance-derived timeout, and adaptive backoff.

Real-hardware acceptance still requires stationary supervision and a fresh
recording of `/dock/target_observation`, `/docking/status`, `/motion_state`,
`/wheel/odom`, `/cmd_vel_docking`, `/cmd_vel_safe`, `/cmd_vel`, and
`/battery_state`. Stop the trial on repeated mode switching, increasing yaw or
lateral error, stale observation/odometry, no physical response to a nonzero
command, or any unexpected safety arbitration. Do not tune ordinary navigation
parameters during this acceptance.
