# Jetson Autostart

Production boot autostart is handled by a host `systemd` unit named `njrh-runtime.service`.

The service does not start the Web dashboard. It starts:

- `NJRH-car` Docker container
- exactly one foreground `run_common_services.sh` inside the container

The systemd runner intentionally calls `njrh_container.sh start` instead of `start-runtime`. `start-runtime` is a manual convenience command that starts common services in the background; using it from systemd and then foregrounding `run_common_services.sh` would create duplicate common-service nodes.

On `systemctl stop` or `systemctl restart`, the runner stops both common-service
processes and resident navigation/localization children from the previous run.
This is a service-bound cleanup path, not a runtime watchdog: normal navigation
does not self-restart Nav2, AMCL, Isaac localization, or `robot_localization_bridge`.

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

Daily full-chain restart:

```bash
sudo systemctl restart njrh-runtime.service
```

Do not use `docker exec -d ... run_floor_navigation.sh` as a daily restart
entrypoint. `run_floor_navigation.sh` is a compatibility/debug wrapper and is
blocked by default because a transient owner can exit before resident navigation
is ready and then clean up Nav2/localization children. `stop_floor_navigation.sh`
is a broad debug cleanup script, not the product restart path.

The production systemd path uses non-login `bash -c` for preparation and the
foreground common-service owner inside an already-running container. Startup
readiness groups related Ranger, IMU, localization, and costmap checks into
bounded C++ probes; grouping reduces Fast DDS discovery churn but preserves the
individual publisher, freshness, lifecycle, and message requirements.

On the 2026-07-10 final build, three complete service restarts reached confirmed
navigation ready in `66.055 s`, `59.321 s`, and `69.676 s`; resident-navigation
times were `43 s`, `43 s`, and `52 s`. See
[`phase_s4_boot_to_navigation_critical_path.md`](phase_s4_boot_to_navigation_critical_path.md)
for the startup contract and formal five-run acceptance procedure.

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
- `docking_manager_node` from `robot_docking_manager`, exposing resident `/docking/start`, `/docking/stop`, and `/docking/undock`
- `robot_api_server_node`

By default `run_common_services.sh` then runs
`NJRH_RESIDENT_NAVIGATION_AUTOSTART=auto`: if
`maps_release/last_navigation_map.json` matches a valid `current/manifest.json`,
it starts the resident localization/Nav2 runtime for that last manually selected
map. If no valid last map exists, the robot remains in `NO_MAP` with common
services alive.

Startup keeps `NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=false`.
MapServer/Isaac loading starts only after canonical local-state admission;
field A/B showed earlier loading can starve TF/DDS discovery even when odometry
messages are already fresh. `NJRH_COMMON_LOCAL_STATE_BACKGROUND_START=true`
still starts local state alongside pointcloud/GS2 helpers, with a bounded final
readiness recheck instead of a full service restart when cold startup finishes
just after the first window.

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

Docking-manager autostart can be disabled only for controlled bench diagnostics:

```bash
NJRH_DOCKING_MANAGER_AUTOSTART=false
```

For GS2 startup, `njrh-runtime.service` resolves the host `/dev/gs2` symlink before entering the container and exports the real tty path as `NJRH_GS2_SERIAL_PORT`. This prevents the container from accidentally using an unrelated `/dev/ttyUSB*` device after USB re-enumeration.
