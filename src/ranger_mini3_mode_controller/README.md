# ranger_mini3_mode_controller

C++ command-shaping layer between `robot_safety` and the AgileX `ranger_base_node`.

## Runtime Contract

- Input: `/cmd_vel_safe`
- Output: `/cmd_vel`
- Feedback/status: `/ranger_mini3_mode_controller/status`
- Desired mode hint: `/ranger_mini3/desired_motion_mode`

This node does not write CAN frames directly. `ranger_base_node` remains the only CAN owner.

## Current Policy

- Reject lateral/crab commands by default; `linear.y` is not allowed.
- Clamp reverse commands out; `linear.x < 0` becomes zero.
- Convert steering requests above `0.698 rad` to pure spin commands.
- Pass normal forward turns as Ackermann-style `Twist` commands.

The current upstream Ranger ROS2 driver exposes `0x00` as front/rear dual Ackermann. A true front-only Ackermann mode is not exposed through the current `/cmd_vel -> ranger_base_node` interface.
