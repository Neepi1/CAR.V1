# Ranger Spin Settle Handoff

Date: 2026-07-04
Updated: 2026-07-14

## Scope

This phase only addresses Ranger Mini 3 pure-yaw stop tail and the handoff from spin to the next linear command.

It does not change FAST-LIO2, JT128, pointcloud QoS, AMCL matching, Nav2 plugin types, or the chassis SDK odom integration.

## Problem

Field tests showed that a commanded pure-yaw stop can still leave residual physical angular motion. If Nav2 or API immediately starts the next linear segment, the robot can enter that segment with a few degrees of unobserved heading tail. On a following straight segment this looks like an unexpected arc even when odom later reports the motion correctly.

The 2026-07-06 CAN audit narrowed this down below the ROS command chain:

- CAN `0x111` had already switched to zero motion command.
- CAN `0x221` still reported about five 50 Hz feedback frames of residual yaw rate.
- CAN `0x291` still reported `motion_mode=SPINNING` with `mode_changing=1`.

This is therefore not a Nav2, AMCL, or robot_safety FIFO delay. It is the Ranger Mini 3 chassis/firmware transition tail while leaving SPINNING mode.

## Changes

- `robot_nav_config` keeps `RotationShimController` in front of MPPI and restores the field smoother handoff:
  - `FollowPath.angular_dist_threshold=0.45`
  - `FollowPath.angular_disengage_threshold=0.075`
- `robot_safety` subscribes `/wheel/odom` and holds the first linear command after pure spin until actual yaw rate is settled or a bounded timeout expires.
- `run_local_state.sh` now keeps `imu_gyro_bias_filter_node` resident by default even when `LOCAL_STATE_EKF_PROFILE=wheel_only`. This keeps `/lidar_imu_bias_corrected` available for `robot_safety` spin-tail detection without fusing IMU into `/local_state/odometry`.
- The patched `ranger_base` wrapper keeps all-zero commands in SPINNING mode while the latest wheel-odom yaw rate still shows spin motion or chassis feedback reports a mode transition. Only after the wheel yaw rate settles does a following zero command switch back to DUAL_ACKERMAN. A 2026-07-06 verification run showed `/motion_state.angular_velocity` can stay at zero during SPINNING, so wheel odom twist is the stop-settle source of truth.
- `robot_api_server` waits for actual `/wheel/odom` yaw-rate settle after API-owned final yaw and predock yaw alignment, then re-reads pose before declaring success.
- Runtime overlay config mirrors the source config for Jetson deployment.
- `verify_nav2_progress_checker_config.sh` now expects the updated RotationShim thresholds.

Runtime knobs:

- `RANGER_SPINNING_ZERO_CMD_HOLD_ENABLED=true`
- `RANGER_SPINNING_ZERO_CMD_HOLD_WZ_THRESHOLD_RADPS=0.030`
- `LOCAL_STATE_IMU_BIAS_FILTER_ENABLED=true`

## 2026-07-06 Verification

The first post-patch check exposed an important false signal: `/motion_state.angular_velocity` stayed at zero during SPINNING while `/wheel/odom.twist.twist.angular.z` still showed real yaw motion. The hold predicate was therefore changed to use the wheel-odom yaw rate cached inside `ranger_base`.

After that correction, a fixed-duration spin stop test at `-0.60 rad/s` for `1.2s` reported:

- `/cmd_vel_collision_checked`, `/cmd_vel_safe`, and `/cmd_vel` reached zero together at `0.161s`.
- `/wheel/odom` yaw rate entered the stop threshold at `0.301s`.
- The last moving wheel-odom sample was at `0.281s`.
- Wheel yaw changed from about `-26.0deg` near command zero to `-27.9deg` final, so the stop tail was about `1.9deg` in this sample.
- `motion_mode` stayed SPINNING until the wheel yaw rate was under the configured threshold, then returned to DUAL_ACKERMAN.

The same stop-latency probe was extended to command Ackermann `linear.x + angular.z` so left/right arc braking could be measured without relying on the larger arc-odom scripts. With `linear.x=0.25m/s`, `|angular.z|=0.166667rad/s`, and `command_sec=1.2s`:

- Left Ackermann: final `/cmd_vel` reached zero at `0.162s`; wheel odom twist entered the stop threshold at `0.321s`; yaw tail from final command zero to final settle was about `+0.30deg`.
- Right Ackermann: final `/cmd_vel` reached zero at `0.160s`; wheel odom twist entered the stop threshold at `0.321s`; yaw tail from final command zero to final settle was about `-0.13deg`.

That means left/right Ackermann has a small normal braking tail, but not the large SPINNING mode-exit tail. The older 45deg arc-odom target script is not a reliable stop-latency probe because it hides the first zero-command burst before writing settle samples; use `record_cmd_vel_stop_latency.sh --linear-speed ... --angular-speed ...` for stop propagation.

Later two-point navigation runs between `delivery_512355` and `delivery_675235`
showed the stop-settle handoff was not sufficient by itself. During the initial
negative SPINNING segment, `/wheel/odom` yaw accumulated about `2.9deg` to
`3.3deg` more rotation than the projected LiDAR IMU yaw over roughly
`160deg` to `165deg` of commanded rotation. The first correction to `0.981`
removed the large over-integration. Dedicated positive and negative `180deg`
spin samples at `0.60rad/s` then compared stable-motion `/wheel/odom` yaw-rate
against projected LiDAR IMU yaw-rate. The trimmed stable-rate fits suggested
`0.97995` for positive spin and `0.98035` for negative spin. However, the
stop instant still showed wheel yaw ahead of projected LiDAR IMU yaw by about
`2.8deg` positive and `3.5deg` negative. The field defaults are therefore
being tested as `0.980` positive and `0.966` negative to prioritize
stop-instant wheel/IMU agreement before separately tuning stop-tail
compensation. The intermediate `0.965` positive / `0.961` negative AB and the
`0.991` positive AB were rejected for positive spin because they increased the
stop-instant wheel/IMU error; negative `0.966` reduced the stop-instant error
to below `1deg` in single-sample verification. A
follow-up attempt to raise the negative default to `0.985` based on
delivery-spin-only samples was rejected: valid two-point navigation still
required `15cm` to `18cm` relocalization corrections and yaw residual worsened
on the return leg. This is an odom integration calibration, not a command-speed
change.

## 2026-07-14 Spin-IMU Incident And Correction

A commanded 180-degree spin followed by a long straight test was invalidated
after the robot entered an unsafe trajectory. The primary spin overshoot was in
the test harness, not an IMU scale jump:

- both motion scripts used subscription depth 50 and called one
  `rclpy.spin_once()` per control iteration while consuming seven active topics;
- when the script decided to stop at `177.765deg`, an independent recorder was
  already at `216.897deg`, equivalent to about 1.14 seconds of stale feedback at
  0.6 rad/s;
- the settled spin IMU delta was `3.9252rad`, or `224.90deg`, showing that the
  IMU observed the physical overspin rather than creating it.

The spin-aware wheel preprocessor also applied corrected yaw and corrected x/y
with different transforms. It previously used constant additive x/y offsets
after changing yaw. That violates the rigid-pose contract. It now anchors a
single transform at spin settle:

```text
p_corrected = R(yaw_offset) * p_wheel + translation
yaw_corrected = yaw_wheel + yaw_offset
```

Regression evidence:

- the new public-interface GTest fails on the old implementation and passes on
  the SE(2) implementation;
- historical replay at 13.6559 seconds reduces the false corrected lateral
  displacement from `2.091115m` to `0.0000002m`;
- an isolated ROS domain test with high-rate multi-topic traffic completed a
  simulated 180-degree spin at `180.122deg`; wheel feedback age was 16.97ms max
  and 9.91ms P95;
- both motion scripts now use depth-1 latest samples, a continuously running
  executor, and a default 0.20-second hard feedback-age limit. Stale feedback
  causes a zero command and failed test rather than continued motion.

These are software and replay gates only. The corrected profile still requires
low-speed `+30/-30deg` field validation in a clear area before any larger-angle
or spin-then-straight test.

## Rollback

The change is reversible by restoring:

- `FollowPath.angular_dist_threshold` and `FollowPath.angular_disengage_threshold`
- `spin_to_drive_settle_enabled=false`
- `RANGER_SPINNING_ZERO_CMD_HOLD_ENABLED=false`
- explicit non-neutral `RANGER_SPINNING_YAW_SCALE_*` overrides only for wheel-odom diagnostics
- `yaw_align_actual_stop_check_enabled=false`

No stored map assets, localization assets, or chassis SDK files are changed by this phase.

`LOCAL_STATE_IMU_BIAS_FILTER_ENABLED=false` restores the old wheel-only helper
behavior, but that also removes the corrected IMU input used by
`robot_safety` to verify physical spin-tail settle. It should only be used for
diagnostics where spin-to-drive safety is explicitly not under test.

## Hardware Validation Still Required

- Ordinary navigation from `delivery_512355` to `delivery_675235` with AMCL in shadow/observe mode.
- Spin then straight test at the current production spin speed.
- Return-to-dock predock yaw alignment check, focusing on whether residual spin tail causes lateral drift.
