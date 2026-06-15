# Phase R4 Ranger Official Passthrough

Phase R4 makes `ranger_mini3_mode_controller` a safety/status passthrough layer
by default instead of a repository-owned Ackermann shaping layer.

Production command chain stays:

```text
Nav2 / final_yaw_align / docking / teleop
  -> /cmd_vel_collision_checked or /cmd_vel_docking
  -> robot_safety
  -> /cmd_vel_safe
  -> ranger_mini3_mode_controller
  -> /cmd_vel
  -> official ranger_base_node
```

Default profile:

```yaml
mode_controller_profile: official_passthrough
```

In `official_passthrough`, normal navigation output preserves:

```text
output.linear.x == input.linear.x
output.linear.y == input.linear.y
output.angular.z == input.angular.z
```

Allowed safety differences are reported in
`/ranger_mini3_mode_controller/status.diff_reason`: `timeout_zero`,
`startup_zero`, `park_requested`, `reverse_not_allowed`, and
`lateral_not_allowed`.

The node still publishes `/ranger_mini3/desired_motion_mode`, but the desired
mode is diagnostic only and has source `predicted_from_cmd_vel_safe`. Actual
mode still comes from `/motion_state` or `/system_state`. A desired/actual
mismatch can warn, but it does not rewrite `/cmd_vel`.

The previous custom model remains available:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_ranger_mode_controller_profile.sh \
  --profile custom \
  --restart
```

Return to production:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_ranger_mode_controller_profile.sh \
  --profile official_passthrough \
  --restart
```

Verify without moving the robot:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_ranger_official_passthrough.sh \
  --profile official_passthrough \
  --compare-cmd \
  --duration-sec 20
```

A/B observation while the operator manually sends the same short navigation
goal:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_ranger_official_passthrough_ab.sh \
  --profile official_passthrough \
  --duration-sec 180 \
  --apply \
  --restart
```

If official passthrough reduces Ackermann-turn localization error, the legacy
custom curvature/yaw-rate shaping was a likely contributor. If the error remains,
the next odometry phase should compare official feedback twist and steering
against `/wheel/odom` and reduce reliance on `/wheel/odom.pose`.
