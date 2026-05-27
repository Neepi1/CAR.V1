# Runtime Service Lifecycle

The runtime is split into common services and mode services.

Common services should stay up during daily operation:

- JT128 driver plus canonical pointcloud/IMU remap
- Ranger chassis driver
- `robot_description` static TF publisher
- `robot_eai_gs2` GS2 near-field docking lidar driver (`/dock/gs2_scan`, `/dock/gs2_points`)
- `robot_local_state` EKF (`/wheel/odom` + `/lidar_imu` gyro yaw-rate -> `/local_state/odometry`)
- `robot_local_perception`
- `robot_safety`
- `ranger_mini3_mode_controller`
- `robot_floor_manager`
- `robot_api_server`

`robot_api_server` is supervised inside the common-service layer. If the API process exits, `run_robot_api_server_supervised.sh` restarts it after a short delay. `njrh_container.sh start-runtime` and `start-common` now also require `GET /api/v1/status` on port `8080` to become healthy before reporting common services as ready. The API process also enforces a bounded HTTP connection count and returns `503` when overloaded instead of creating unbounded detached request threads.

The Web dashboard is not part of the production runtime. It is only a manual observation/debug window.

Mode services are allowed to start and stop when switching between navigation and mapping:

- Navigation: Isaac localization stack, `robot_localization_bridge`, Nav2, velocity smoother, collision monitor.
- Mapping: FAST-LIO2, PGO, slam_toolbox 2D mapping, scan slicing helpers.
- Docking: `robot_docking_manager`, started only for near-field charging alignment after Nav2 reaches the pre-dock pose.
- Current field-default mapping: slam_toolbox 2D mapping only. FAST-LIO2/PGO are retained as optional formal mapping services, not the default daily mapping mode.

By default, runtime scripts reuse common services:

```bash
NJRH_REUSE_COMMON_SERVICES=true
```

Force restart is explicit and should be used only for repair:

```bash
NJRH_FORCE_RESTART_DRIVER=true
NJRH_FORCE_RESTART_CANONICAL_TF=true
NJRH_FORCE_RESTART_NAV_HELPERS=true
```

Disable GS2 common-service startup only for bench tests or when the sensor is physically disconnected:

```bash
NJRH_GS2_AUTOSTART=false
```

When enabled through `njrh-runtime.service`, the host runner resolves `/dev/gs2` to its real tty device and passes that path into the container as `NJRH_GS2_SERIAL_PORT`.

Start common services:

```bash
NJRH_DASHBOARD_HOST=192.168.31.23 bash scripts/jetson/njrh_container.sh start-runtime
```

If the container already exists and you only need to restart the common layer:

```bash
bash scripts/jetson/njrh_container.sh start-common
```

Enable boot autostart on the Jetson host:

```bash
cd /home/nvidia/workspaces/njrh-v3/workspace1
bash scripts/jetson/install_njrh_autostart.sh install
```

The host `njrh-runtime.service` owns the production common-service process. It starts or reuses the container with `njrh_container.sh start`, then runs one foreground `run_common_services.sh` process for systemd supervision. Do not make the systemd runner call `start-runtime`, because that command also starts common services in the background.

Start the Web dashboard only when debugging:

```bash
bash scripts/jetson/njrh_container.sh start-dashboard
```

Daily navigation can then start only the navigation mode layer:

```bash
docker exec -it NJRH-car bash -lc \
  'cd /workspaces/njrh-v3/workspace1 && NJRH_BUILDING_ID=building_1 NJRH_FLOOR_ID=floor_1 bash scripts/jetson/runtime_overlay/scripts/run_occupancy_grid_localization.sh'

docker exec -it NJRH-car bash -lc \
  'cd /workspaces/njrh-v3/workspace1 && NJRH_BUILDING_ID=building_1 NJRH_FLOOR_ID=floor_1 bash scripts/jetson/runtime_overlay/scripts/run_nav2_navigation.sh'
```

`run_nav2_navigation.sh` always removes stale repo-owned standard Nav2 nodes before launching a new standard navigation stack. This is required because orphaned lifecycle nodes can remain `unconfigured` after a failed startup; in that state `robot_safety` receives no `/cmd_vel_collision_checked` stream and keeps `/cmd_vel_safe` at zero by watchdog.

Default field mapping should stop navigation mode services, keep common services alive, then start the slam_toolbox 2D mapping chain. Optional formal 3D mapping may start FAST-LIO2/PGO explicitly. The Web dashboard is still a test UI; its stop-core path now keeps driver/chassis/common services alive by default.
