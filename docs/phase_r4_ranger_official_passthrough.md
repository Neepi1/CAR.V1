# Phase R4 Ranger Official Passthrough

Phase R4 no longer keeps a repository-owned Ranger Mini 3 Ackermann shaping
model. The legacy `custom` profile was removed after field tests showed the
official Ranger driver/SDK odometry and command interpretation are the trusted
baseline.

The mode controller now has one behavior: official passthrough. It observes the
post-safety command mirror and publishes diagnostics:

```text
robot_safety
  -> /cmd_vel_safe
  -> ranger_mini3_mode_controller
  -> /ranger_mini3/mode_controller_shadow_cmd_vel
  -> /ranger_mini3/desired_motion_mode
  -> /ranger_mini3_mode_controller/status
```

In the current runtime, `robot_safety` remains the final `/cmd_vel` publisher and
the official `ranger_base_node` remains the only CAN owner. The mode controller
does not write CAN frames and does not run a second Ackermann model.

Normal navigation output is preserved except for explicit guard rails:

```text
output.linear.x == input.linear.x, except reverse is clamped without a fresh permit
output.linear.y == input.linear.y, except normal-navigation lateral is rejected
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

The status payload exposes:

```text
mode_controller_profile=official_passthrough
legacy_custom_ackermann_removed=true
cmd_vel_passthrough=true
```

`mode_controller_profile=custom` is no longer accepted by helper scripts and is
ignored by the node with a warning if a stale parameter override survives in an
old deployment.

Verify without moving the robot:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_ranger_official_passthrough.sh \
  --compare-cmd \
  --duration-sec 20
```

Runtime restarts must use the full service owner:

```bash
sudo systemctl restart njrh-runtime.service
```
