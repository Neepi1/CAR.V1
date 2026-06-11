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

Undocking is a separate low-speed safety strategy, not an App velocity command. `/docking/undock` is accepted from `Docked`, live charging contact, or the explicit persistent dock-contact latch written by BMS contact, docking success, or maintenance confirmation. Inference uses `power_supply_status=CHARGING/FULL`, charge current, `present=true` with valid voltage, or `present=true` with full SOC and valid voltage until a dedicated charging-contact GPIO/topic is available; full SOC plus pack voltage alone is not contact because the same values can appear away from the charger. The node releases park/charging hold, enables reverse at the Ranger mode controller, and backs out along `base_link` negative X using the retained calibrated speed `undock.speed_mps=0.06`. Completion is confirmed by actual planar movement on `undock.odom_topic`; elapsed command time is not treated as distance.

The charging dock is treated as a push-in spring mechanism: the robot charges only after pushing into the internal switch travel, so undocking must move continuously at the controlled speed through that same switch zone. The runtime must not replace this with a stop-and-wait de-energize gate, because stopping on the switch contact can leave the DC contacts arcing. Continuity is enforced downstream by `robot_safety`, which holds a fresh `/cmd_vel_docking` command inside its docking-priority window while still keeping ordinary Nav2 reverse disabled.

The undock state machine separates release delay, first-motion wait, and after-motion progress monitoring. `undock.command_settle_s` lets park/forced-mode/reverse-enable changes settle before nonzero `/cmd_vel_docking` is sent. During the following `waiting_first_motion` phase the node must continuously publish `/ranger_mini3/docking_allow_reverse=true` and `/cmd_vel_docking.linear.x=-undock.speed_mps`; `undock.motion_start_timeout_s` is the grace window after that first nonzero command is sent, not a delay before sending commands. The retained calibrated speed remains `undock.speed_mps=0.06`.

`/docking/status` carries the state-machine evidence used for field diagnosis. Typical strings are `undocking command_settle elapsed=0.30/0.50 ...`, `undocking waiting_first_motion distance=0.000/0.600 cmd_x=-0.060 cmd_count=12 reverse_enable=true start_elapsed=0.6/6.0`, and `undocking active distance=0.034/0.600 cmd_x=-0.060 cmd_count=... last_progress_age=0.2`. If first odometry motion never exceeds `undock.progress_epsilon_m`, the failure is `undock_failed_motion_start_timeout distance=... cmd_count=... cmd_x=...`; `cmd_count=0` is classified separately as `undock_failed_no_command_published` because that is a state-machine command-publishing bug, not a chassis traction problem. Only after first motion does `undock.no_progress_timeout_s` detect a mid-undock stall and report `undock_failed_no_progress distance=...`. `undock.timeout_s` must cover command settle plus first-motion wait plus `undock.distance_m / undock.speed_mps` and margin. Commands still go through `/cmd_vel_docking` and `robot_safety`; any downstream obstacle, estop, or chassis rejection must stop the robot before `/cmd_vel`. This does not enable ordinary Nav2 reverse.

For command-observation reconciliation, `/docking/status` now exposes parseable `phase`, `cmd_count`, `reverse_enable_count`, `last_cmd_x`, `last_cmd_stamp_age_s`, `command_start_elapsed_s`, `motion_start_timeout_s`, `first_motion_started`, and `failure_reason` fields on running and terminal undock states where applicable. A report with internal `cmd_count>0` but no observed `/cmd_vel_docking` sample is treated as an observer/topic-identity problem until the topic graph proves otherwise; `cmd_count=0` must not be reported as `undock_failed_motion_start_timeout`.
