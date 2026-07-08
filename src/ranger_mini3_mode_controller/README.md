# ranger_mini3_mode_controller

C++ motion-mode observer for the AgileX `ranger_base_node`.

## Runtime Contract

- Input: `/cmd_vel_safe` diagnostic mirror from `robot_safety`
- Output: `/ranger_mini3/mode_controller_shadow_cmd_vel`
- Feedback/status: `/ranger_mini3_mode_controller/status`
- Desired mode hint: `/ranger_mini3/desired_motion_mode` (`std_msgs/String`, official Ranger
  `motion_mode` enum JSON with legacy string retained)
- Actual mode feedback: `/motion_state` and `/system_state`
- Forced mode input: `/ranger_mini3/forced_mode`
- Runtime reverse permission: `/ranger_mini3/docking_allow_reverse` and `/ranger_mini3/teleop_allow_reverse`, plus legacy `/ranger_mini3/allow_reverse`

This node does not write CAN frames directly and is not in the commercial critical
speed path. `robot_safety` owns the final `/cmd_vel` publisher, and `ranger_base_node`
remains the only CAN owner.

## Official Passthrough Only

The legacy repository-owned Ackermann shaping profile has been removed. The node
now has one production behavior: official passthrough. It shadows the
`robot_safety` final command mirror
and publishes only `/ranger_mini3/mode_controller_shadow_cmd_vel` plus motion-mode
diagnostics. It does not own `/cmd_vel`, so a mode/status failure cannot break the
commercial command path `robot_safety -> /cmd_vel -> ranger_base_node`. The official
AgileX `ranger_base_node` and SDK therefore interpret the production Twist once.

## Ranger Motion Mode Contract

Phase O1 keeps the existing `/ranger_mini3/desired_motion_mode` topic type as
`std_msgs/String`, but changes the value to the official AgileX Ranger enum semantics:

- `MOTION_MODE_DUAL_ACKERMAN = 0`
- `MOTION_MODE_PARALLEL = 1`
- `MOTION_MODE_SPINNING = 2`
- `MOTION_MODE_SIDE_SLIP = 3`

The mode controller maps its internal decisions to that enum:
`dual_ackermann -> 0`, `crab/parallel -> 1`, `spin -> 2`, `side_slip -> 3`, and
`park -> 0` with zero velocity.
Pure yaw commands, including RotationShim/final-yaw-align output after `robot_safety`,
therefore publish desired `MOTION_MODE_SPINNING`.

Actual chassis mode is read from the official Ranger `/motion_state` and `/system_state`
messages. `/ranger_mini3_mode_controller/status` includes both `desired_motion_mode`
and `actual_motion_mode`, plus `mode_aligned` and `mode_alignment_state`. The command is
not blocked while waiting for actual feedback because the upstream Ranger driver derives
its actual mode from incoming `/cmd_vel`; blocking the first spin command would deadlock
mode switching. Wheel-odom and EKF guards must use `actual_motion_mode`, not the desired
hint.

## Current Policy

- Preserve normal `linear.x`, `linear.y`, and `angular.z` from `/cmd_vel_safe` only when policy allows it.
- Clamp reverse commands out during navigation; `linear.x < 0` becomes zero unless a fresh docking, mapping-teleop, or bounded ordinary final-verify retry reverse permission is present.
- Reject lateral output by default when `lateral_policy=reject`, reporting `diff_reason=lateral_not_allowed`.
- `/ranger_mini3/forced_mode=side_slip` or legacy `crab` is the docking-only lateral exception in official passthrough. While active, `linear.y` is preserved and desired motion mode is `MOTION_MODE_SIDE_SLIP=3`; normal navigation still rejects lateral commands.
- `/ranger_mini3/forced_mode=park` publishes zero.
- `/ranger_mini3_mode_controller/status` exposes `legacy_custom_ackermann_removed=true`; any stale `mode_controller_profile=custom` override is ignored with a warning.
- Reverse permission is short-lived (`reverse_enable_timeout_s`, default `0.75 s`) and tracked per source, so an idle App teleop publisher cannot cancel a live docking undock permit.

The current upstream Ranger ROS2 driver exposes `0x00` as front/rear dual Ackermann. A true front-only Ackermann mode is not exposed through the current `/cmd_vel -> ranger_base_node` interface. Crab mode is therefore intentionally not used by MPPI/Nav2; it is a short-range docking override only.
