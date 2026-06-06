# Jetson Autostart

Production boot autostart is handled by a host `systemd` unit named `njrh-runtime.service`.

The service does not start the Web dashboard. It starts:

- `NJRH-car` Docker container
- exactly one foreground `run_common_services.sh` inside the container

The systemd runner intentionally calls `njrh_container.sh start` instead of `start-runtime`. `start-runtime` is a manual convenience command that starts common services in the background; using it from systemd and then foregrounding `run_common_services.sh` would create duplicate common-service nodes.

Install and enable:

```bash
cd /home/nvidia/workspaces/njrh-v3/workspace1
bash scripts/jetson/install_njrh_autostart.sh install
```

Install and start immediately:

```bash
cd /home/nvidia/workspaces/njrh-v3/workspace1
bash scripts/jetson/install_njrh_autostart.sh install-start
```

Check:

```bash
systemctl is-enabled njrh-runtime.service
systemctl status njrh-runtime.service --no-pager
bash scripts/jetson/njrh_container.sh status
```

Environment is stored in:

```text
/etc/njrh/runtime.env
```

Edit it to set `ROBOT_API_TOKEN` for the Android app API.

## Autostart Node Set

The boot service starts common infrastructure first:

- `hesai_ros_driver_node`
- `pointcloud_axis_remap_node`
- `imu_axis_remap_node`
- `ranger_base_node`
- `robot_description_static_tf_node`
- `gs2_driver_node` from `robot_eai_gs2`, publishing `/dock/gs2_scan` and `/dock/gs2_points` in `gs2_link`
- `robot_localization/ekf_node` as `robot_local_state`, fusing `/wheel/odom` with system-time `/lidar_imu`
- `local_perception_node`
- `floor_manager_node`
- `robot_safety_node`
- `mode_controller_node` from `ranger_mini3_mode_controller`
- `robot_api_server_node`

By default `run_common_services.sh` then runs
`NJRH_RESIDENT_NAVIGATION_AUTOSTART=auto`: if
`maps_release/last_navigation_map.json` matches a valid `current/manifest.json`,
it starts the resident localization/Nav2 runtime for that last manually selected
map. If no valid last map exists, the robot remains in `NO_MAP` with common
services alive.

It does not start:

- Web dashboard
- PGO
- slam_toolbox
- Isaac localization stack, `robot_localization_bridge`, or Nav2 unless a valid
  last navigation map is found or `NJRH_RESIDENT_NAVIGATION_AUTOSTART=true` is
  explicitly configured

GS2 autostart can be disabled for bench tests by setting this in `/etc/njrh/runtime.env`:

```bash
NJRH_GS2_AUTOSTART=false
```

For GS2 startup, `njrh-runtime.service` resolves the host `/dev/gs2` symlink before entering the container and exports the real tty path as `NJRH_GS2_SERIAL_PORT`. This prevents the container from accidentally using an unrelated `/dev/ttyUSB*` device after USB re-enumeration.
