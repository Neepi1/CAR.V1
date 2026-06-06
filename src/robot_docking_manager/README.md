# robot_docking_manager

`robot_docking_manager` is the near-field docking controller for the Ranger Mini 3 charging dock. It uses the front GS2 scan for final alignment only; long-range return-to-dock remains a Nav2 task.

Runtime contract:

- Input scan: `/dock/gs2_scan` in `gs2_link`
- Charging state: `/battery_state`
- Undock odometry confirmation: `/local_state/odometry`
- Start/stop/undock services: `/docking/start`, `/docking/stop`, `/docking/undock`
- Status: `/docking/status`
- Command output: `/cmd_vel_docking`
- Mode request: `/ranger_mini3/forced_mode=crab` during active docking, then `auto` on stop or `park` after charging is detected
- Reverse enable: `/ranger_mini3/docking_allow_reverse=true` only during controlled undocking, then `false`

The node deliberately publishes before `robot_safety`, not directly to `/cmd_vel_safe` or `/cmd_vel`. The command path remains:

```text
robot_docking_manager
  -> /cmd_vel_docking
  -> robot_safety
  -> /cmd_vel_safe
  -> ranger_mini3_mode_controller
  -> /cmd_vel
  -> ranger_base_node
```

`rosbag` is not required for normal docking execution. It is still required for commercial tuning and regression: record `/dock/gs2_scan`, `/battery_state`, `/local_state/odometry`, `/tf`, `/cmd_vel_docking`, `/cmd_vel_collision_checked`, `/cmd_vel_safe`, `/ranger_mini3/forced_mode`, and `/ranger_mini3_mode_controller/status` during real docking trials.

The field profile is intentionally conservative: GS2 yaw line-fitting is enabled, lateral and yaw error are low-pass filtered and deadbanded, and active docking forces Ranger crab mode so `linear.y` can correct lateral offset directly while `angular.z` squares the car to the dock. The GS2 lateral sign is inverted for the current physical mounting through `controller.lateral_command_sign=-1.0`; if a future mount reports `y` with the ROS `base_link` sign, change that value back to `1.0`. The yaw gain is also signed for the current mount as `controller.kyaw=-0.70`; if the first field test rotates away from the dock face, flip only this sign before changing the magnitude. Docking is staged: while lateral or yaw error is outside the priority threshold, `max_forward_while_lateral_mps=0.000` keeps the robot from advancing. The lateral tolerance is `1.5cm`, the yaw tolerance is `2deg`, and `min_lateral_speed_mps=0.025` keeps small residual corrections above the downstream Ranger command deadband. Once centered and squared, `lock_lateral_during_final_insert=true` forces `linear.y=0` and `angular.z=0` so the final contact insert is straight forward only.

Charging detection is a global hard stop. If `/battery_state` reports charging/full, charge current exceeds `charging.min_current_a`, or `present=true` with voltage inside the configured contact range, the node immediately publishes zero velocity and enters `Docked`, even if the controller is still in the align phase.

Undocking is a separate low-speed safety strategy, not an App velocity command. `/docking/undock` is accepted only from `Docked` or when live charging contact is detected/inferred. Inference uses `power_supply_status=CHARGING/FULL`, charge current, `present=true` with valid voltage, or `present=true` with full SOC and valid voltage until a dedicated charging-contact GPIO/topic is available; full SOC plus pack voltage alone is not contact because the same values can appear away from the charger. The node releases park/charging hold, enables reverse at the Ranger mode controller, and backs out along `base_link` negative X using `undock.speed_mps`. Completion is confirmed by actual planar movement on `undock.odom_topic`; elapsed command time is not treated as distance. If odometry is missing/stale, the robot makes no progress for `undock.no_progress_timeout_s`, or `undock.timeout_s` expires before `undock.distance_m`, the node stops and reports a failed undock status. Commands still go through `/cmd_vel_docking` and `robot_safety`; any downstream obstacle, estop, or chassis rejection must stop the robot before `/cmd_vel`.
