# Ranger Spin Receive-Time Error Split

## Scope

This report separates Ranger Mini 3 spin yaw error at the callback receipt time
of the first final `/cmd_vel` zero command. It does not use AMCL, Isaac, or a
relocalization correction as the yaw reference.

Test conditions:

- command path: `/cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base`
- command: `+180deg` and `-180deg` at `0.6rad/s`
- start boundary: first nonzero `/cmd_vel` callback after the quiet handshake
- split boundary: first zero `/cmd_vel` callback after that nonzero command
- physical stop boundary: raw `/lidar_imu` three-axis angular-rate norm below
  `0.035rad/s` for `0.30s`
- yaw reference: `/lidar_imu_bias_corrected.angular_velocity.z`, transformed to
  `base_link` and integrated by callback receipt time
- raw IMU norm is used only for motion onset/stop timing; it is not treated as
  yaw because it also contains roll/pitch motion

## Evidence

| metric | +180deg | -180deg |
|---|---:|---:|
| first zero publish to final `/cmd_vel` receipt | 1.90ms | 5.33ms |
| final nonzero `/cmd_vel` to wheel motion onset | 0.559s | 0.609s |
| final nonzero `/cmd_vel` to IMU motion onset | 0.968s | 0.876s |
| wheel-reported onset lead over IMU | 0.410s | 0.267s |
| stable wheel yaw rate | +0.5761rad/s | -0.5817rad/s |
| stable IMU yaw rate | +0.5777rad/s | -0.5759rad/s |
| wheel yaw before zero receipt | +179.479deg | -179.426deg |
| IMU yaw before zero receipt | +167.770deg | -169.562deg |
| wheel - IMU before zero | +11.709deg | -9.864deg |
| wheel yaw after zero to physical stop | +4.685deg | -4.342deg |
| IMU yaw after zero to physical stop | +4.236deg | -4.688deg |
| wheel - IMU after zero | +0.449deg | +0.345deg |
| wheel - IMU total | +12.158deg | -9.519deg |
| local odom - wheel total | -0.175deg | -0.247deg |

Artifacts:

- `20260714T113153Z_spin_pos180_rxsplit_rawimu_01/`
- `20260714T113246Z_spin_neg180_rxsplit_rawimu_01/`

Each artifact contains `samples.csv`, event-level `receive_events.csv`,
`metrics.json`, and `summary.md`.

## Conclusion

The dominant yaw mismatch is generated before the final zero command reaches
`ranger_base`, not after it. In the positive sample, 11.709deg of the final
12.158deg wheel/IMU mismatch already existed at zero receipt. In the negative
sample, the pre-zero mismatch was -9.864deg and the post-zero tail mismatch
partially cancelled it by +0.345deg.

The chassis has a real physical stop tail of about 4.2-4.7deg at this speed, but
wheel odometry records that tail within 0.35-0.45deg. Waiting for settle remains
necessary for a clean spin-to-drive handoff, but more waiting cannot remove the
dominant endpoint yaw error.

The zero command itself traverses the safety chain in 2-5ms, so the measured
error is not caused by a `robot_safety`, DDS, or final-command FIFO delay. The
stable wheel and IMU yaw rates agree within about 1.1%, which also means a
single global spin scale is not the primary correction for this dataset.

The distinguishing event is SPINNING startup: wheel odometry begins reporting
and integrating yaw 0.27-0.41s before both raw and corrected LiDAR IMU observe
body rotation. The most likely remaining source is the chassis feedback/model
during steering-to-SPINNING transition, where
`state.motion_state.angular_velocity` becomes nonzero before equivalent body
yaw is measured. The current local-state output remains anchored to wheel pose
and follows the wheel total within 0.25deg, so Nav2 inherits this pre-zero error.

## Next Verification

Capture steering-actuator angles and the internal CAN motion feedback together
with the same receive-time IMU boundary for `30/60/90/180deg` in both
directions. If the startup area remains an approximately fixed angle, correct
the SPINNING transition interval rather than changing the steady-state yaw
scale. If it grows proportionally with commanded angle after transition, only
then refit the directional yaw scales.
