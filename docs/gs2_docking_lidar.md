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
robot_docking_manager forces Ranger crab mode for centimeter-level lateral correction
robot_safety arbitrates final low-speed command
BMS charging state confirms contact
```

Control-chain rule:

```text
robot_docking_manager
  -> /cmd_vel_collision_checked
  -> robot_safety
  -> /cmd_vel_safe
  -> ranger_mini3_mode_controller
  -> /cmd_vel
  -> ranger_base_node
```

Do not publish docking control directly to `/cmd_vel_safe` or `/cmd_vel`; that would bypass the final safety arbiter.

The current field profile prioritizes stable contact over aggressive correction: `use_yaw_fit=false`, `filter_alpha=0.25`, `lateral_deadband_m=0.005`, `lateral_command_sign=-1.0`, `min_lateral_speed_mps=0.025`, `max_lateral_speed_mps=0.04`, `max_forward_while_lateral_mps=0.000`, and `lock_lateral_during_final_insert=true`. The negative lateral sign matches the current GS2 mounting where the detected charger `y` sign is opposite the vehicle correction direction. This makes docking three-stage: correct lateral/yaw first, lock lateral velocity to zero, then crawl straight forward. The minimum lateral speed is intentional because the downstream Ranger mode controller drops lateral commands below its deadband. If the robot visibly oscillates sideways, reduce `controller.ky_lateral` first; do not increase `max_lateral_speed_mps` until the GS2 scan is stable.

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

The positive and negative strips are already height-aligned with the dock contacts, so docking control is a planar `x/y/yaw` problem. Because the Ranger Mini 3 supports four-wheel steering, the docking controller uses crab `linear.y` for lateral correction instead of Ackermann steering when `/docking/start` is active. The controller must target `charge_contact_link -> dock_contact_link`, not `base_link -> dock_contact_link`. Once `/battery_state.current` exceeds the configured charging threshold, docking control must stop immediately even if the controller has not reached the contact-verification state.

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

Start the near-field docking controller only when entering docking mode:

```bash
bash /workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_docking_manager.sh
ros2 service call /docking/start std_srvs/srv/Trigger {}
ros2 topic echo /docking/status
```

Normal execution does not require `rosbag`. Use `rosbag2` for tuning and regression captures of `/dock/gs2_scan`, `/battery_state`, `/tf`, `/cmd_vel_collision_checked`, `/cmd_vel_safe`, `/ranger_mini3/forced_mode`, and `/ranger_mini3_mode_controller/status`.

On the current Jetson, the GS2 CP2102 adapter appears as `10c4:ea60` and should be aliased to `/dev/gs2`.
When the runtime is started through `njrh-runtime.service`, the host runner resolves `/dev/gs2` to the real tty device and passes it into the container as `NJRH_GS2_SERIAL_PORT`. This avoids falling back to an unrelated USB serial port after USB re-enumeration.

Bench validation:

```bash
ros2 topic hz /dock/gs2_scan
ros2 topic echo /dock/gs2_scan --once
ros2 service call /gs2_driver_node/stop_scan std_srvs/srv/Empty {}
ros2 service call /gs2_driver_node/start_scan std_srvs/srv/Empty {}
```
