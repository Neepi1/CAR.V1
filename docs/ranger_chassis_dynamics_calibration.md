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

The stop-latency probe waits for fresh `/wheel/odom`, the command publisher's
`robot_safety` subscription, and a zero command observed on both
`/cmd_vel_safe` and `/cmd_vel` before it starts the motion clock. The default
discovery timeout is 10 seconds and can be changed with
`--discovery-timeout-sec`. A readiness timeout refuses to move the chassis.
The report separates yaw accumulated before the stop request from yaw tail
accumulated after the stop request. Odom, local-state, motion-state, and
corrected-IMU subscriptions are latest-only, while an independent executor
keeps callbacks current during command publication. The report records wheel
message age and compares wheel yaw tail with integrated
`/lidar_imu_bias_corrected` yaw tail.

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
to below `1deg` in single-sample verification. An older attempt near `0.985`
was inconclusive because online corrections and
stale relocalization results were mixed into the comparison. The 2026-07-10
bounded A/B froze bridge corrections and bracketed the same two-point route at
`0.977672` and `1.0`. The fitted and repeated field value is `0.986000`.

The temporary attempt to raise near-straight linear odom scale from `0.960` to
`0.970` was rejected by field data: the `delivery_512355` correction worsened
to about 15.0 cm total and lateral error grew to about 12.4 cm. The production
overlay keeps the validated sign-specific SPINNING scales while wheel pose/yaw
remains the long-term EKF anchor and corrected IMU supplies dynamic yaw-rate:

```yaml
spinning_yaw_scale_positive: 0.976386
spinning_yaw_scale_negative: 0.986000
dual_ackermann_linear_odom_scale: 0.960
dual_ackermann_linear_odom_scale_max_abs_yaw_rate: 0.060
```

## 2026-07-13 Dual Ackermann Near-Straight Yaw A/B

A correction-frozen two-point audit separated the wheel/IMU yaw residual from
online AMCL corrections. `/wheel/odom` and `/local_state/odometry` remained
aligned, and the initial SPINNING residual was below one degree. The later
near-straight custom corrections were larger than the measured full-leg
wheel/IMU residual and had the wrong net effect:

- previous near-straight scale: `1.120` for both signs
- previous fixed bias: `-0.0041 rad/m`
- predicted residual with those two corrections removed: about `+0.09deg` on
  the return leg and `+0.43deg` on the outbound leg

The first field A/B kept the feedback twist, near-straight linear scale, and
sign-specific SPINNING scales unchanged, while restoring only these yaw terms
to neutral:

```yaml
dual_ackermann_near_straight_yaw_scale_positive: 1.000
dual_ackermann_near_straight_yaw_scale_negative: 1.000
dual_ackermann_near_straight_yaw_bias_per_meter: 0.0
```

Three correction-frozen round trips were then run between `delivery_512355`
and `delivery_675235`. The six legs kept `/wheel/odom` and
`/local_state/odometry` within `0.014mm`, but the median explicit
relocalization correction was still `13.95cm / 1.12deg`. The median full-leg
wheel/IMU yaw residual was `-1.41deg`, so the all-neutral candidate did not
pass the `10cm / 0.5deg` acceptance target.

Replaying only DUAL_ACKERMAN intervals with `0.003 <= |wz| <= 0.060rad/s`
separated the feedback signs:

| feedback sign | intervals | wheel yaw | projected IMU yaw | fitted scale |
|---|---:|---:|---:|---:|
| positive | 129 | `41.044deg` | `42.733deg` | `1.041151` |
| negative | 206 | `-77.930deg` | `-76.180deg` | `0.977549` |

The next single-model A/B therefore uses those sign-specific scales while the
per-meter bias remains neutral and the SPINNING scales remain unchanged:

```yaml
dual_ackermann_near_straight_yaw_scale_positive: 1.041151
dual_ackermann_near_straight_yaw_scale_negative: 0.977549
dual_ackermann_near_straight_yaw_bias_per_meter: 0.0
```

The second three-round-trip validation retained those fitted scales. Compared
with the all-neutral baseline:

| metric across six legs | neutral | sign-specific fit |
|---|---:|---:|
| median odom closure | `7.30cm` | `6.80cm` |
| median absolute wheel/IMU yaw residual | `1.41deg` | `0.48deg` |
| median explicit map->odom correction | `13.95cm` | `10.31cm` |
| median absolute correction yaw | `1.12deg` | `0.54deg` |
| maximum correction translation | `27.72cm` | `14.57cm` |

The fitted model materially improves both directions without changing the
linear or SPINNING scales. It remains just outside the strict `10cm / 0.5deg`
all-leg target. The post-relocalization scan-map search itself still requested
`2.5cm` to `12.5cm` offsets, so this dataset does not support another odom
parameter step below that truth-source floor. Keep the fitted values until a
surveyed pose or independent motion-capture reference is available.

## 2026-07-13 Physical Marker Linear Scale

Two physical floor markers, `calibration2` and `calibration3`, provide a
`13.9557m` surveyed map baseline. With the near-straight linear scale at
`0.960`, the wheel displacement under-reported the measured longitudinal
travel by about `37.9cm` in one direction and `27.8cm` in the reverse
direction. This rejects `0.960` independently of Isaac or AMCL endpoint
corrections.

The final A/B used AMCL `shadow` mode. Across both legs, `map->odom` remained
unchanged, AMCL generated `297` candidates, and the bridge accepted zero AMCL
corrections. Ground-marker endpoint offsets and raw wheel displacement gave:

| leg | physical displacement | wheel displacement | fitted scale |
|---|---:|---:|---:|
| calibration3 to calibration2 | `13.6625m` | `13.7873m` | `0.99095` |
| calibration2 to calibration3 | `13.7921m` | `13.9159m` | `0.99110` |

The two independently fitted values differ by only `0.00015`. The runtime uses
their rounded common value:

```yaml
dual_ackermann_linear_odom_scale: 0.991
dual_ackermann_linear_odom_scale_max_abs_yaw_rate: 0.060
```

The post-apply validation retained AMCL shadow mode and accepted no AMCL
corrections. Its independent physical-marker comparison was:

| leg | physical displacement | scaled wheel displacement | distance residual |
|---|---:|---:|---:|
| calibration3 to calibration2 | `13.9149m` | `13.9056m` | `-9mm` (`-0.067%`) |
| calibration2 to calibration3 | `13.9507m` | `13.9203m` | `-30mm` (`-0.218%`) |

Both directions are below the `1%` wheel-distance target. The measurements are
manual ground-marker offsets, so the small residual does not justify another
parameter step or extra decimal place.

This decision changes only near-straight linear integration. The same round
trip still exposed a physical lateral closure mismatch: the robot closed about
`4cm` forward and `18.5cm` right of its measured start, while wheel odom closed
about `1.2cm` forward and `10.0cm` right. Sign-specific yaw calibration,
SPINNING calibration, and Ackermann arc feedback therefore remain separate
work. AMCL gating and the local-state profile are unchanged by the linear
scale.

## 2026-07-14 Spin Error Split by Callback Receipt Time

`record_cmd_vel_stop_latency.sh` now records event-level callback receipt times
for the command chain, wheel/local odometry, corrected IMU, raw IMU, and Ranger
motion state. The primary split is the first final `/cmd_vel` zero callback
after a nonzero command. Raw IMU three-axis norm defines physical motion onset
and sustained stop, while corrected IMU `base_link` yaw-rate is the yaw
reference.

The `+180deg` and `-180deg` tests at `0.6rad/s` showed:

| metric | positive | negative |
|---|---:|---:|
| zero publish to final `/cmd_vel` receipt | `1.90ms` | `5.33ms` |
| wheel motion onset lead over IMU | `0.410s` | `0.267s` |
| wheel - IMU before zero | `+11.709deg` | `-9.864deg` |
| wheel - IMU after zero | `+0.449deg` | `+0.345deg` |
| wheel - IMU total | `+12.158deg` | `-9.519deg` |

The physical tail was `4.2-4.7deg`, but wheel odometry recorded it within
`0.45deg`. The dominant mismatch therefore forms during SPINNING startup before
the zero command, not in the stop-command transport or an unobserved tail.
Stable wheel and IMU yaw rates agreed within about `1.1%`, so changing one
global yaw scale would distort steady motion while hiding a transition error.

The complete report and event CSV files are under
`reports/ranger_spin_receive_time_split/`.

## 2026-07-14 Mode-Aware Spin Yaw Correction

The receipt-time split rejects a global yaw-scale or fixed-angle patch. The
steady wheel and IMU rates agree, and the physical stop tail is already visible
in wheel odom. The actionable defect is the false wheel-yaw interval created
while Ranger reports `SPINNING` before the chassis body starts rotating.

Production now uses `LOCAL_STATE_EKF_PROFILE=wheel_spin_imu`:

1. Preserve upstream `/wheel/odom` unchanged for diagnostics and rollback.
2. On actual `/motion_state.motion_mode=2`, anchor the last corrected wheel
   x/y/yaw and start integrating `/lidar_imu_bias_corrected.angular_velocity.z`.
3. Keep x/y fixed for a pure spin and continue IMU integration after the final
   zero command until yaw-rate stays below `0.02rad/s` for `0.30s`.
4. Preserve the resulting x/y/yaw offsets when wheel odom resumes, so the
   published pose is continuous rather than jumping at the mode boundary.
5. Fall back to raw wheel odom if IMU age exceeds `0.20s`; force bounded spin
   completion after `2.0s` of zero command if the stop detector cannot settle.
6. Feed corrected wheel `x/y/yaw/vx` and corrected IMU yaw-rate to the EKF.
   Wheel yaw-rate is excluded from this profile to avoid duplicate dynamic yaw
   inputs.

This changes neither Ranger SDK output nor FAST-LIO2 inputs. TF ownership also
stays unchanged: `robot_local_state` remains the only `odom -> base_link`
publisher. Rollback is one profile change to `wheel_imu`, followed by a full
`njrh-runtime.service` restart.

Validation results:

| test | corrected result | fallback/gap |
|---|---:|---:|
| shadow `+180deg @ 0.6rad/s` | corrected - IMU `+0.030deg` | `0 / 0` |
| shadow `-180deg @ 0.6rad/s` | corrected - IMU `+0.018deg` | `0 / 0` |
| production `+90deg @ 0.6rad/s` | local - IMU `+0.114deg`, x/y drift `0.16mm` | `0 / 0` |
| production `-90deg @ 0.6rad/s` | local - IMU `-0.019deg`, x/y drift below sample resolution | `0 / 0` |
| `delivery_675235 -> delivery_512355` | `5.00cm / 0.22deg`, AMCL accepted `0` | no navigation failure |
| `delivery_512355 -> delivery_675235` | `4.78cm / 0.55deg`, AMCL accepted `0` | no navigation failure |

After the production spin and navigation runs, the status counters were
`completed=6`, `imu_fallback=0`, `imu_gap=0`, and `forced_settle=1`. The forced
settle is the bounded `2.0s` zero-command completion path, not an IMU outage;
retain this counter in longer repeatability tests rather than silently treating
it as an ordinary settle.

The two navigation reports are
`/tmp/spin_imu_to_512_20260714_01/summary.md` and
`/tmp/spin_imu_return_675_20260714_01/summary.md` on the Jetson container.
AMCL remained in shadow mode. Its maximum candidates (`0.407m` and `0.933m`)
were not accepted and must be diagnosed as scan-map/localization candidates,
not attributed to this local odom correction.
