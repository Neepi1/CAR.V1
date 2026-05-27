# robot_docking_manager

`robot_docking_manager` is the near-field docking controller for the Ranger Mini 3 charging dock. It uses the front GS2 scan for final alignment only; long-range return-to-dock remains a Nav2 task.

Runtime contract:

- Input scan: `/dock/gs2_scan` in `gs2_link`
- Charging state: `/battery_state`
- Start/stop services: `/docking/start`, `/docking/stop`
- Status: `/docking/status`
- Command output: `/cmd_vel_collision_checked`
- Mode request: `/ranger_mini3/forced_mode=crab` during active docking, then `auto` on stop or `park` after charging is detected

The node deliberately publishes before `robot_safety`, not directly to `/cmd_vel_safe` or `/cmd_vel`. The command path remains:

```text
robot_docking_manager
  -> /cmd_vel_collision_checked
  -> robot_safety
  -> /cmd_vel_safe
  -> ranger_mini3_mode_controller
  -> /cmd_vel
  -> ranger_base_node
```

`rosbag` is not required for normal docking execution. It is still required for commercial tuning and regression: record `/dock/gs2_scan`, `/battery_state`, `/tf`, `/cmd_vel_collision_checked`, `/cmd_vel_safe`, `/ranger_mini3/forced_mode`, and `/ranger_mini3_mode_controller/status` during real docking trials.

The first field profile is intentionally conservative: GS2 yaw line-fitting is disabled by default, lateral error is low-pass filtered and deadbanded, and active docking forces Ranger crab mode so `linear.y` can correct lateral offset directly. The GS2 lateral sign is inverted for the current physical mounting through `controller.lateral_command_sign=-1.0`; if a future mount reports `y` with the ROS `base_link` sign, change that value back to `1.0`. Docking is staged: while lateral or yaw error is outside the priority threshold, `max_forward_while_lateral_mps=0.000` keeps the robot from advancing. The lateral tolerance is `1.5cm`, and `min_lateral_speed_mps=0.025` keeps small residual corrections above the downstream Ranger command deadband. Once centered, `lock_lateral_during_final_insert=true` forces `linear.y=0` so the final contact insert is straight forward only.

Charging detection is a global hard stop. If `/battery_state` reports charging/full or the charging current exceeds `charging.min_current_a`, the node immediately publishes zero velocity and enters `Docked`, even if the controller is still in the align phase.
