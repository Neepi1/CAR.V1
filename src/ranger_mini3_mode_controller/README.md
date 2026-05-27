# ranger_mini3_mode_controller

C++ command-shaping layer between `robot_safety` and the AgileX `ranger_base_node`.

## Runtime Contract

- Input: `/cmd_vel_safe`
- Output: `/cmd_vel`
- Feedback/status: `/ranger_mini3_mode_controller/status`
- Desired mode hint: `/ranger_mini3/desired_motion_mode`
- Forced mode input: `/ranger_mini3/forced_mode`
- Runtime reverse permission: `/ranger_mini3/allow_reverse`

This node does not write CAN frames directly. `ranger_base_node` remains the only CAN owner.

## Current Policy

- Reject lateral/crab commands by default; `linear.y` is not allowed during normal navigation.
- Allow `linear.y` only when `/ranger_mini3/forced_mode` is set to `crab`/`lateral`, which is reserved for near-field docking.
- Clamp reverse commands out during navigation; `linear.x < 0` becomes zero unless a fresh `/ranger_mini3/allow_reverse=true` mapping-teleop permission is present.
- Convert sustained steering requests above `0.785 rad` (45 deg) to pure spin commands.
- Keep spin latched until the request drops below `0.489 rad` (28 deg), with `0.4 s` entry debounce and `1.0 s` minimum hold time to avoid Ackermann/spin chatter.
- Pass normal forward turns as Ackermann-style `Twist` commands.
- Reverse permission is short-lived (`reverse_enable_timeout_s`, default `0.75 s`) so a dead App or API server cannot leave navigation reverse-enabled.

The current upstream Ranger ROS2 driver exposes `0x00` as front/rear dual Ackermann. A true front-only Ackermann mode is not exposed through the current `/cmd_vel -> ranger_base_node` interface. Crab mode is therefore intentionally not used by MPPI/Nav2; it is a short-range docking override only.
