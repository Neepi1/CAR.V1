# ranger_mini3_mode_controller

C++ command-shaping layer between `robot_safety` and the AgileX `ranger_base_node`.

## Runtime Contract

- Input: `/cmd_vel_safe`
- Output: `/cmd_vel`
- Feedback/status: `/ranger_mini3_mode_controller/status`
- Desired mode hint: `/ranger_mini3/desired_motion_mode`
- Forced mode input: `/ranger_mini3/forced_mode`
- Runtime reverse permission: `/ranger_mini3/docking_allow_reverse` and `/ranger_mini3/teleop_allow_reverse`, plus legacy `/ranger_mini3/allow_reverse`

This node does not write CAN frames directly. `ranger_base_node` remains the only CAN owner.

## Current Policy

- Reject lateral/crab commands by default; `linear.y` is not allowed during normal navigation.
- Allow `linear.y` only when `/ranger_mini3/forced_mode` is set to `crab`/`lateral`, which is reserved for near-field docking.
- Clamp reverse commands out during navigation; `linear.x < 0` becomes zero unless a fresh docking or mapping-teleop reverse permission is present.
- Keep normal forward driving in Ackermann mode. Steering requests above `0.698 rad` (40 deg) are yaw-rate limited instead of being converted to spin while `|linear.x| > auto_spin_max_linear_mps`.
- Allow automatic high-curvature spin only at very low speed (`auto_spin_max_linear_mps`, default `0.08 m/s`) or when the upstream command is pure yaw (`linear.x == 0` and `angular.z != 0`).
- Keep low-speed spin latched until the request drops below `0.489 rad` (28 deg), with `0.4 s` entry debounce and `1.0 s` minimum hold time to avoid Ackermann/spin chatter.
- Pass normal forward turns as Ackermann-style `Twist` commands with yaw-rate clamping.
- Reverse permission is short-lived (`reverse_enable_timeout_s`, default `0.75 s`) and tracked per source, so an idle App teleop publisher cannot cancel a live docking undock permit.

The current upstream Ranger ROS2 driver exposes `0x00` as front/rear dual Ackermann. A true front-only Ackermann mode is not exposed through the current `/cmd_vel -> ranger_base_node` interface. Crab mode is therefore intentionally not used by MPPI/Nav2; it is a short-range docking override only.
