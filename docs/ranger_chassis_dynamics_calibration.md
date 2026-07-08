# Ranger Mini 3 Chassis Dynamics Calibration

This document defines the field test used to measure Ranger Mini 3 chassis
dynamics that are not specified in the vendor manual.

## Scope

Measured items:

- command-chain latency from the test input topic to final `/cmd_vel`
- chassis response delay from `/cmd_vel` to CAN 0x221 and `/wheel/odom`
- acceleration slope and steady-state velocity gain
- deceleration time and stop distance after zero command
- Ackermann steering response delay using CAN 0x221 steering feedback
- SDK or firmware smoothing inferred from step command versus CAN feedback
- motion-mode switch latency from `/ranger_mini3_mode_controller/status`

The test does not change FAST-LIO2, JT128, Nav2 plugins, localization assets, or
the canonical TF tree.

## Command Path

The script publishes to the same safety-chain input used by navigation after
collision monitoring:

```text
run_ranger_chassis_dynamics_test.sh
  -> /cmd_vel_collision_checked
  -> robot_safety
  -> /cmd_vel
  -> ranger_base_node
```

It does not publish directly to `/cmd_vel`.

## Reports

Reports are written under:

```text
reports/ranger_chassis_dynamics_test/<timestamp>_<label>_<profile>/
```

Files:

- `environment.md`: ROS/CAN/topic environment snapshot
- `planned_segments.json`: exact command sequence
- `candump.log`: raw CAN capture
- `samples.csv`: synchronized command, CAN, odom, and mode samples
- `metrics.json`: machine-readable metrics
- `summary.md`: human-readable result table

## Standard Low/Mid-Speed Test

Run only in a clear area with E-stop available:

```bash
cd /workspaces/njrh-v3/workspace1
source /opt/ros/humble/setup.bash
source install/setup.bash

bash scripts/jetson/runtime_overlay/scripts/run_ranger_chassis_dynamics_test.sh \
  --profile standard \
  --countdown-sec 5 \
  --label standard_low_mid
```

The `standard` profile includes:

- 0.20 m/s step and brake
- 0.60 m/s step and brake
- left/right Ackermann steering steps at 0.20 m/s
- positive/negative spinning-mode steps
- Ackermann mode return check

It does not run the 1.20 m/s segment.

## High-Speed Segment

Use this only in a long straight lane:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_ranger_chassis_dynamics_test.sh \
  --profile linear \
  --include-high-speed \
  --countdown-sec 5 \
  --label linear_high_speed_1p2
```

This adds:

- 1.20 m/s straight step
- zero-command braking from 1.20 m/s

## Dry Run

Dry-run prints the command sequence without moving the chassis:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_ranger_chassis_dynamics_test.sh \
  --profile standard \
  --dry-run \
  --countdown-sec 0 \
  --output-root /tmp/njrh_dynamics_dryrun \
  --label dryrun
```

## Interpreting Key Metrics

- `cmd_chain_latency_sec`: time from script command segment start to final
  `/cmd_vel` reaching 50 percent of the target.
- `actual_delay_10pct_sec`: time for CAN 0x221 or wheel odom to reach 10
  percent of target after the command starts.
- `rise_10_to_90_sec`: measured acceleration rise time.
- `stop_time_sec`: time after zero command until linear and yaw rates are near
  zero.
- `can_integrated_stop_distance_m`: distance integrated from CAN 0x221 linear
  feedback after a zero-command segment starts.
- `wheel_odom_stop_distance_m`: `/wheel/odom` pose distance after a zero-command
  segment starts. Compare this with CAN distance to catch odom lag or replay
  artifacts.
- `steering_delay_10pct_sec`: time for CAN steering feedback to reach 10
  percent of the expected Ackermann inner steering angle.
- `mode_switch_latency_sec`: time for actual motion mode to match the expected
  mode.

If `/cmd_vel` steps immediately but CAN 0x221 rises slowly, the smoothing or
acceleration limiting is inside the official SDK or chassis firmware. If
`/cmd_vel` itself ramps slowly, the delay is in the upstream command chain.

## Applied Velocity Smoother Profile

The field reports from 2026-06-30 and 2026-07-01 show that 1.20 m/s braking
continues for about 0.56-0.58 m, while 0.30-0.40 m/s terminal braking still
continues for about 5.5-10.2 cm. The production Nav2 profile therefore keeps
the measured 1.20 m/s cruise cap. The 2026-07-05 spin-to-drive handoff pass
uses closed-loop `velocity_smoother` feedback from `/local_state/odometry`, so
the smoother cannot build up a high linear command while downstream
`robot_safety` is holding the chassis stopped after a spin. The 2026-07-03
ordinary-navigation smoothness pass keeps this measured
acceleration envelope, raises the smoother output to 30 Hz, and moves terminal
speed-limit steps earlier so the controller does not ask for hard endpoint
braking:

```yaml
feedback: "CLOSED_LOOP"
smoothing_frequency: 30.0
max_velocity: [1.20, 0.0, 0.70]
min_velocity: [-0.08, 0.0, -0.70]
max_accel: [0.55, 0.0, 0.90]
max_decel: [-0.95, 0.0, -1.10]
odom_topic: /local_state/odometry
odom_duration: 0.2
```

The linear acceleration remains conservative against the measured 1.2 m/s rise
time. Linear deceleration is reduced from the older `-0.95` value to avoid
asking the terminal controller to brake harder than the chassis reliably tracks
near the goal. Angular limits are capped to MPPI's `wz_max=0.70` and use a
more conservative angular ramp because field spin tests showed measurable
post-zero yaw drift.

The earlier closed-loop no-motion concern applied to a 20 Hz smoother with
`max_accel.z=0.45`, where each angular step could sit near the chassis
deadband. The current profile uses 30 Hz and `max_accel.z=0.90`, and the field
failure being addressed is the opposite case: open-loop smoothing accumulated a
large command while the final safety gate was intentionally outputting zero.

## 2026-07-05 Odom-Only Linear Bias Check

The odom redline must run with AMCL in `shadow` mode. In this mode AMCL
candidates are recorded but `map->odom` corrections are not accepted, so the
post-goal relocalization delta is measuring wheel/chassis odom drift rather
than localization correction performance.

The first shadow-only delivery leg to `delivery_675235` ended with about
25.0 cm `base_link` relocalization delta and about 29.1 cm `map->odom` delta.
Motion segmentation showed the error grew mainly in near-straight and terminal
slow motion, while the previously verified 1.5 m Ackermann circles should not
be rescaled globally.

The Ranger SDK overlay therefore applies a near-straight-only DUAL_ACKERMAN
linear odom scale:

```yaml
dual_ackermann_linear_odom_scale: 0.960
dual_ackermann_linear_odom_scale_max_abs_yaw_rate: 0.060
```

The scale is applied only when the selected yaw rate is within the threshold;
arc turns keep the official linear velocity so curvature and turn radius are
not distorted.

Shadow-only validation after the change:

- `delivery_512355`: post-goal `map->odom` delta was about 16.2 cm, with only
  about 0.6 cm forward residual and about 16.2 cm lateral residual.
- `delivery_675235`: post-goal `map->odom` delta was about 7.8 cm, with about
  2.1 cm forward residual and about 7.5 cm lateral residual.

This reduces the original forward odom bias without relying on AMCL acceptance.
The remaining route-dependent lateral residual should be investigated as a
separate scan-map, lateral frame, or route curvature issue, not by globally
scaling Ackermann odom.

## 2026-07-05 Negative Spin Scale Follow-Up

With AMCL still in `shadow` mode, repeated delivery redlines showed the
remaining error on the `delivery_675235 -> delivery_512355` leg was mostly
lateral in `map->base_link`: about 19.2 cm left after explicit post-goal
relocalization. The forward component was only about 1.9 cm, so changing the
near-straight linear scale was not the next variable.

The initial route segment contains a large negative SPINNING turn. Reducing
the negative SPINNING yaw odom scale from `1.011` to `1.004` was an
intermediate field step that reduced the same leg's `map->base_link`
correction to about 10.8 cm total, with only about 1.8 cm lateral residual.
Later two-point delivery runs compared `/wheel/odom` yaw with projected
LiDAR-IMU yaw during the initial negative SPINNING segment. Wheel odom
initially accumulated about 2.9 to 3.3 degrees too much yaw over roughly
160 to 165 degrees of physical rotation. The first negative-spin correction
to `0.981` removed the large over-integration. Dedicated positive and
negative `180deg` spin samples at `0.60rad/s` then fitted only the stable
rotation window, excluding startup latency and stop tail. Stable yaw-rate
ratios suggested `0.97995` for positive spin and `0.98035` for negative spin.
However, the stop instant still showed wheel yaw ahead of projected LiDAR IMU
yaw by about `2.8deg` positive and `3.5deg` negative. The field defaults are
therefore being tested as `0.980` positive and `0.966` negative to prioritize
stop-instant wheel/IMU agreement before separately tuning stop-tail
compensation. The intermediate `0.965` positive / `0.961` negative AB and the
`0.991` positive AB were rejected for positive spin because they increased the
stop-instant wheel/IMU error; negative `0.966` reduced the stop-instant error
to below `1deg` in single-sample verification. A
follow-up attempt to raise the negative default to `0.985`
based on delivery-spin-only samples was rejected: valid two-point navigation
still required `15cm` to `18cm` relocalization corrections and yaw residual
worsened on the return leg.

The temporary attempt to raise near-straight linear odom scale from `0.960` to
`0.970` was rejected by field data: the `delivery_512355` correction worsened
to about 15.0 cm total and lateral error grew to about 12.4 cm. The production
overlay therefore keeps neutral SPINNING yaw scaling for the wheel odom source,
because navigation heading is now owned by the local-state EKF using corrected
IMU yaw-rate:

```yaml
spinning_yaw_scale_positive: 0.976386
spinning_yaw_scale_negative: 0.977672
dual_ackermann_linear_odom_scale: 0.960
dual_ackermann_linear_odom_scale_max_abs_yaw_rate: 0.060
```
