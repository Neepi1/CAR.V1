# GS2 docking lidar integration

The EAI/YDLIDAR GS2 is integrated as `robot_eai_gs2`.

Runtime contract:

```text
GS2 serial UART
  -> robot_eai_gs2/gs2_driver_node
  -> /dock/gs2_scan   frame_id=gs2_link
  -> /dock/gs2_points frame_id=gs2_link
```

This driver is for near-field docking only. It is not part of the canonical JT128 navigation chain and it must not publish `map->odom`, `odom->base_link`, or any static mount transform.

Recommended docking use:

```text
Nav2 reaches pre_dock_pose
robot_docking_manager reads /dock/gs2_scan
robot_docking_manager estimates dock lateral/yaw/distance error
robot_docking_manager requests Ranger side-slip and sends low-speed x/y/yaw correction
robot_safety arbitrates final low-speed command
BMS charging state confirms contact
```

Control-chain rule:

```text
robot_docking_manager
  -> /cmd_vel_docking
  -> robot_safety
  -> /cmd_vel_safe
  -> ranger_mini3_mode_controller
  -> /cmd_vel
  -> ranger_base_node
```

Do not publish docking control directly to `/cmd_vel_safe` or `/cmd_vel`; that would bypass the final safety arbiter.

The current field profile prioritizes stable contact over aggressive correction: `mode.use_crab_mode=true`, `mode.crab_forced_mode=side_slip`, `mode.yaw_forced_mode=spinning`, `use_yaw_fit=true`, `front_cluster_x_window_m=0.015`, `min_lateral_span_m=0.035`, `min_confidence=0.10`, `yaw_fit_min_lateral_span_m=0.055`, `filter_alpha=0.25`, `lateral_deadband_m=0.005`, `lateral_command_sign=-1.0`, `kyaw=0.70`, `controller.yaw_spin_priority_enabled=true`, `min_lateral_speed_mps=0.025`, `max_lateral_speed_mps=0.04`, `max_forward_while_lateral_mps=0.000`, and `lock_lateral_during_final_insert=true`. The detector keeps the nearest front-face cluster before fitting yaw so edge/background returns inside the 30 cm GS2 range do not flip the fitted angle. A partial left/right view can still drive lateral correction, but yaw fitting is suppressed until the visible face span is large enough. The negative lateral sign matches the current GS2 mounting where the detected charger `y` sign is opposite the vehicle correction direction. The positive yaw gain matches the observed staged yaw-spin controller behavior: when GS2 fitted yaw was positive, negative `cmd_wz` increased the yaw error, so the closed-loop correction must command positive body yaw for positive fitted yaw. This makes docking staged by chassis mode: correct yaw first with `spinning` and a yaw-only command only when yaw exceeds the 5 degree fine tolerance, switch to `side_slip` for lateral correction with `cmd_wz=0`, lock lateral/yaw velocity to zero, then crawl straight forward. `/docking/status` exposes `phase`, `forced_mode`, `cmd_vx`, `cmd_vy`, `cmd_wz`, `desired_contact_vy`, `pivot_comp_vy`, and `charge_contact_x` for field validation. If the robot visibly oscillates sideways, reduce `controller.ky_lateral` first; if it rotates away from the dock angle instead of squaring up before the side-slip stage, flip only `controller.kyaw` before changing the magnitude.

Mounting rule:

- Optical face points at the charger.
- Rotate the GS2 so its fan scans horizontally across the charger face.
- `base_link -> gs2_link` is single-sourced in `robot_description`.
- `base_link -> charge_contact_link` is also single-sourced in `robot_description`.
- Current physical mount assumption: front centerline, flush with the front body plane, height `0.290m`.
- Current TF value: `xyz=[0.36, 0.0, 0.290]`, `rpy=[0.0, 0.0, 0.0]`.
- Charging contact center: `xyz=[0.398, 0.0, 0.255]`, 3.8 cm ahead of `gs2_link` on vehicle `+X`.
- Refine `gs2_x` after measuring the exact `base_link` origin to front-panel distance on the finished body.

Charging contact geometry:

```yaml
housing_lateral_length_m: 0.235
housing_vertical_width_m: 0.080
electrode_lateral_length_m: 0.185
electrode_vertical_width_m: 0.030
positive_electrode_position: upper
negative_electrode_position: lower
```

The positive and negative strips are already height-aligned with the dock contacts, so docking control is a planar `x/y/yaw` problem. Because the Ranger Mini 3 supports both spin and lateral motion as separate chassis modes, the docking controller uses a yaw-first `spinning` phase for heading error and only then uses `linear.y` in `side_slip` for left/right body correction when `/docking/start` is active. `ranger_mini3_mode_controller` keeps normal navigation lateral commands rejected, but allows docking lateral commands while `/ranger_mini3/forced_mode=side_slip` is active. The controller must target `charge_contact_link -> dock_contact_link`, not `base_link -> dock_contact_link`. Once `/battery_state.current` exceeds the configured charging threshold, docking control must stop immediately even if the controller has not reached the contact-verification state.

RViz validation:

```text
Fixed Frame: base_link
LaserScan topic: /dock/gs2_scan
LaserScan QoS reliability: Best Effort
```

With the current `rpy=[0,0,0]`, a board placed in front of the vehicle should appear on the `+X` side of `base_link`.

Bring-up:

```bash
sudo bash /home/nvidia/workspaces/njrh-v3/workspace1/scripts/jetson/install_gs2_udev.sh
ls -l /dev/gs2

source /opt/ros/humble/setup.bash
source /workspaces/njrh-v3/workspace1/install/setup.bash
ros2 launch robot_eai_gs2 gs2.launch.py serial_port:=/dev/gs2
```

Start the near-field docking controller only when entering docking or controlled undocking mode:

```bash
## Diagnostic manual start only; production starts this as a resident common service.
bash /workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_docking_manager.sh
ros2 service call /docking/start std_srvs/srv/Trigger {}
ros2 topic echo /docking/status
```

For App-triggered undocking, call `POST /api/v1/docking/undock`; the API forwards the intent to `/docking/undock`. Do not publish reverse `/cmd_vel` directly, because reverse permission and final motion arbitration must remain inside `robot_docking_manager`, `robot_safety`, and the Ranger mode controller. During undock the docking manager owns `/ranger_mini3/docking_allow_reverse`; App teleop uses a separate `/ranger_mini3/teleop_allow_reverse` permit so idle teleop stop messages cannot cancel a live undock. Undocking completion is odometry-confirmed on `/local_state/odometry`; stale odometry, no physical progress, or timeout must report a failed undock instead of `undocked`. After odometry-confirmed undock, `robot_api_server` triggers Isaac relocalization and verifies that the new localization is reflected in `map -> base_link`; manual undock records the result for diagnostics, while auto-undock before a navigation goal requires the relocalization to succeed before Nav2 is commanded.

Normal execution does not require `rosbag`. Use `rosbag2` for tuning and regression captures of `/dock/gs2_scan`, `/battery_state`, `/local_state/odometry`, `/tf`, `/cmd_vel_docking`, `/cmd_vel_collision_checked`, `/cmd_vel_safe`, `/ranger_mini3/forced_mode`, and `/ranger_mini3_mode_controller/status`.

On the current Jetson, the GS2 CP2102 adapter appears as `10c4:ea60` and should be aliased to `/dev/gs2`.
When the runtime is started through `njrh-runtime.service`, the host runner resolves `/dev/gs2` to the real tty device and passes it into the container as `NJRH_GS2_SERIAL_PORT`. This avoids falling back to an unrelated USB serial port after USB re-enumeration.

Bench validation:

```bash
ros2 topic hz /dock/gs2_scan
ros2 topic echo /dock/gs2_scan --once
ros2 service call /gs2_driver_node/stop_scan std_srvs/srv/Empty {}
ros2 service call /gs2_driver_node/start_scan std_srvs/srv/Empty {}
```
