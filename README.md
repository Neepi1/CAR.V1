# multi_floor_delivery_robot

ROS 2 Humble multi-floor indoor/outdoor delivery robot navigation stack scaffold for Jetson Orin + JT128 + Ranger Mini 3.

## Current Status

This repository currently contains the phase-ordered baseline required by `02_实现任务清单.yaml`:

- `P0`: local car-project reuse scanning, TF audit tooling, canonical TF policy
- `P1`: workspace skeleton, dependency resolution baseline, initial package scaffolds
- `P2/P4/P5`: first-round wrapper and bringup scaffolds for the critical path packages
- current local reuse source: `D:\codespace\car`
- current Jetson host context: `nvidia@192.168.31.23:/home/nvidia/workspaces/njrh-v3/workspace1`

The implementation intentionally prioritizes:

1. Local car-project reuse before any network fetch
2. Canonical TF tree governance
3. Wrapper isolation for JT128 / FAST-LIO2 / PGO / Isaac localizer
4. Single `map->odom` and single `odom->base_link`

The repository now also carries the occupancy-builder extension required by the v3 update:

- `reports/occupancy_builder_design.md`
- `src/robot_occupancy_builder`
- `docs/occupancy_builder_workflow.md`
- live draft contract: `JT128 + /mapping/frontend_pose -> /mapping/draft_map`
- release rebuild contract: `raw bag + optimized trajectory -> nav_map + localizer_map`

## Jetson Runtime

The current field runtime path reuses the validated Jetson `car` stack inside a dedicated `NJRH-car` container instead of introducing a second temporary operator UI.

- Jetson host workspace: `/home/nvidia/workspaces/njrh-v3/workspace1`
- Jetson upstream asset workspace: `/home/nvidia/workspaces/isaac_ros-dev`
- upstream compatibility mount inside container: `/workspaces/isaac_ros-dev`
- runtime container: `NJRH-car`
- runtime image default: `njrh-car:latest`
- runtime image build network mode: `host`
- runtime image fallback when Jetson rebuild is blocked: `isaac_ros_dev-aarch64:latest`
- dashboard URL: `http://192.168.31.23:2048` when explicitly started for debug only
- operator guide: `docs/jetson_njrh_container_runtime.md`
- runtime orchestration owner: this repository's `scripts/jetson/runtime_overlay`
- CAN up/down helpers are now present in the overlay as `scripts/jetson/runtime_overlay/scripts/bringup_ranger_can.sh` and `shutdown_ranger_can.sh`, matching the reused dashboard's expected filenames
- live 2D mapping owner: `slam_toolbox` launched by this repository's `scripts/jetson/runtime_overlay/scripts/run_projected_map.sh`, exposed to the reused dashboard through the compatibility `/api/projected_map/latest` endpoint and explicitly reusing the existing TF tree without injecting extra static sensor TF
- web `/api/mapping2d/start` now enters that same repository-owned `slam_toolbox` chain directly and no longer falls back to the historical `run_jt128_2d_mapping.sh` / cartographer path
- current live 2D source for the operator view: canonical `/lidar_points + /lidar_imu -> FAST-LIO2 /cloud_registered_body(lidar_link) -> nav_cloud_preprocessor(lidar_level_link) -> /points_nav(lidar_level_link) -> pointcloud_to_laserscan(target_frame=lidar_level_link) -> C++ scan_republisher_node -> slam_toolbox /map`
- repository-owned 2D scan contract owner: `scripts/jetson/runtime_overlay/config/jt128_scan_slam2d.yaml`, shared for slice geometry by `slam_toolbox` mapping and Isaac relocalization sensing; live 2D mapping can use FAST-LIO2's deskewed current-frame cloud, while stationary Isaac relocalization now consumes canonical raw `/lidar_points` directly. Both paths slice in the leveled frame `lidar_level_link` (`base_link -> lidar_level_link`: same XYZ as `lidar_link`, yaw aligned to `lidar_link`, roll/pitch forced to zero) with the current slice band `-0.75m .. 0.35m` in that leveled frame, selected to recover more structural returns with the current tilted JT128 installation
- live `slam_toolbox` mapping consumes the recovered ~16 Hz scan stream with `minimum_time_interval=0.05`, `minimum_travel_heading=0.02`, `scan_buffer_size=30`, and `transform_timeout=0.35` so fast in-place rotation uses denser scan updates instead of dropping every other frame.
- Fast-LIO runtime now accepts only the canonical sensor topics `/lidar_points` and `/lidar_imu` with `sensor_frame_id=lidar_link`; the repository-owned runtime no longer permits the old Fast-LIO-only remap path
- The shared FAST-LIO2 runtime profile is optimized for live 2D/formal mapping: it keeps `/cloud_registered_body` enabled, uses `point_filter_num=4` with `max_iteration=3`, and disables Path/Laser-map publisher work (`path_en=false`, `map_en=false`) to reduce frontend load without changing the canonical sensor installation TF. Navigation relocalization does not start this profile by default.
- repository wrapper contract now matches the Jetson runtime: `robot_fastlio_mapping` is canonical-only and no longer exposes `hesai_lidar_fastlio` as a runtime fallback
- JT128 point-cloud web view now defaults to the chassis frame `base_link`; raw sensor coordinates remain available only as an explicit debug toggle
- JT128 ingress follows the validated project path: the driver publishes vendor raw `/jt128/vendor/points_raw` and `/jt128/vendor/imu_raw`, then repository-owned remap helpers rotate raw axes into the canonical sensor frames and publish `/lidar_points(lidar_link)` and `/lidar_imu(imu_link)`. The static TF chain keeps only the physical installation pose from `base_link` to `lidar_link` / `imu_link`.
- the canonical JT128 ingress remains repository-owned, and runtime now requires the compiled `robot_hesai_jt128` pointcloud and imu remap nodes; Python remap fallbacks were removed so a missing binary fails fast instead of degrading runtime performance
- The C++ pointcloud remap has an in-place fast path for the current JT128 canonical matrix, which keeps `/lidar_points`, FAST-LIO2 `/cloud_registered_body`, `/points_nav`, and `/scan` near the live vendor stream rate during 2D mapping and raw-pointcloud relocalization sensing.
- The canonical `/lidar_points` publisher uses sensor-data QoS (`best_effort`, depth `1`) by default so 1.8 MB JT128 clouds do not stall behind reliable DDS delivery and lose effective rate versus the 20 Hz source stream
- Dashboard driver readiness now keys off the live Hesai driver plus the canonical pointcloud/IMU remap helpers and recent `/lidar_points` samples
- Dashboard driver readiness now treats a fresh `/lidar_points` cache as authoritative even if the auxiliary `ros2 topic hz /lidar_points` probe times out while collecting rate output. This prevents false `Timed out waiting for lidar driver output` failures where `lidar_recent=True` and the operator can independently confirm a stable pointcloud rate.
- Dashboard `slam_toolbox` startup now allows a longer first-map warmup window before declaring failure, so slow first `/map` publication no longer aborts an otherwise healthy mapping start
- The live `slam_toolbox` chain keeps a repository-owned C++ `scan_republisher_node` between `/scan_raw` and `/scan`, defaulting to pass-through. Full scan reversal is opt-in via `NJRH_SLAM2D_FLIP_SCAN=1` for field debugging only
- the 2D mapping and occupancy-localization launch scripts now both clear stale `scan_republisher_node` processes before relaunch, preventing duplicate `/scan` publishers from blocking the flatscan path
- The 2D web view is back to rendering OccupancyGrid in its normal y-up convention and no longer carries a web-only orientation workaround
- Dashboard `开始3D建图` now starts the formal mapping backend as well: JT128 canonical ingress + FAST-LIO2 frontend + PGO backend + live `slam_toolbox` 2D view
- dashboard save behavior: paired `PGO 3D save -> current slam_toolbox /map snapshot -> Isaac localizer asset generation`, and the standalone 2D save path now follows the same `slam_toolbox nav map + localizer png/yaml` rule
- Current field default is the Web 2D mapping path: use `/api/mapping2d/start` and the standalone 2D save action for `slam_toolbox` maps. The 3D/PGO path is retained for optional formal mapping, not the default daily mapping flow.
- dashboard runtime asset directories `maps/`, `maps3d/`, and `waypoints/` are now writable project-owned mirrors seeded from the upstream `car` workspace instead of read-only symlinks
- dashboard `停止底层感知` now targets the direct JT128 driver plus FAST-LIO2, slam_toolbox live 2D chain, and related mapping-side helpers as a unit
- default navigation params owner: this repository's `scripts/jetson/runtime_overlay/config/nav2.yaml`
- local perception runtime owner: this repository's `scripts/jetson/runtime_overlay/scripts/run_local_perception.sh`
- local perception runtime now reuses the validated `car` JT128 nav preprocessor contract but executes a repo-owned filter that transforms `/lidar_points` into `base_link`, applies range/height/sector/body masks, publishes `/perception/obstacle_points` for local marking plus real-return clearing, and publishes synthetic `/perception/clearing_points` ray endpoints for no-return local costmap clearing. NORMAL mode keeps marking local but looks farther ahead for avoidance (`range_filter.max=5.50`, `height_filter=0.40..1.30`) and enables voxel outlier filtering so sparse far returns are not repeatedly inflated into local obstacles. Clearing rays use a balanced Jetson profile (`0.75 deg`, range steps `0.50/1.00/2.00/3.50/5.50/8.00 m`, 8 z endpoints, `max_points=30000`) to preserve stale-obstacle clearing without excessive VoxelLayer raytracing load.
- local perception stamps `/perception/obstacle_points` and `/perception/clearing_points` with the latest available `odom -> base_link` TF time (`output_stamp_tf_target_frame=odom`) instead of raw processing wall time. With `require_output_stamp_tf=true`, frames are skipped until that TF is available, preventing Nav2 local costmap message-filter drops where perception clouds arrive outside the costmap TF cache.
- local-state production runtime uses `robot_localization/ekf_node` as `robot_local_state`: `/wheel/odom` provides planar chassis velocity and `/lidar_imu` provides JT128 gyro yaw-rate. Field mapping/navigation currently selects the wheel-only C++ passthrough mode; that mode republishes Ranger `/wheel/odom` into the project canonical `/local_state/odometry` with `odom_yaw_offset_rad=0.0` and `rotate_odom_position_with_yaw_offset=false`, treating Ranger SDK odometry as the chassis truth and only renaming the child frame to canonical `base_link`. JT128 raw axis alignment is single-sourced in the canonical remap helpers (`x=raw_y`, `y=-raw_x`, `z=raw_z`), while the static TF chain keeps only vehicle installation pose. The current JT128 physical install uses `lidar_yaw=pi` and `imu_yaw=pi`; this correction belongs in `base_link -> lidar_link` / `base_link -> imu_link`, not in `odom->base_link`. The JT128 driver runtime forces `use_timestamp_type=1` so point cloud and IMU headers use host/system time, avoiding mixed-clock EKF drift during left/right rotation. The IMU remap node overrides the vendor zero covariance on gyro data (`angular_velocity_covariance_diagonal=0.10/0.10/0.25`) so the EKF treats LiDAR IMU yaw-rate as a weak stabilizing constraint instead of an absolute truth source.
- Ranger native odom is kept as a separate semantic frame: `run_ranger_chassis.sh` starts the upstream driver with `base_frame=ranger_base_link`, while `robot_local_state` is the only owner that republishes the project canonical `base_link`. This prevents the upstream SDK/driver frame label from directly polluting the canonical TF tree.
- local Nav2 voxel occupancy projection now requires at least 2 marked voxels in a column before a 2D local-costmap cell becomes occupied (`voxel_layer.mark_threshold=2`), which suppresses single-voxel residue after clearing while preserving real obstacles that span multiple height bins.
- local dynamic-obstacle avoidance is tuned for Ranger Mini 3 four-wheel-drive/four-wheel-steering with a repository-owned C++ `ranger_mini3_mode_controller` between `robot_safety` and `ranger_base_node`: lateral/crab commands are rejected, reverse is clamped out, large steering requests are converted to spin, and normal forward turns are passed as Ackermann-style `Twist` commands.
- MPPI now uses Ranger Mini 3's documented Ackermann radius envelope (`min_turning_r=0.81`, `wz_max=0.70`) instead of sampling sub-physical 0.35 m turns; its obstacle critic matches the local inflation layer (`inflation_radius=0.35`, `cost_scaling_factor=6.0`). The local window is `10m x 10m`, the prediction horizon is `44 * 0.09s = 3.96s`, and PathAlign is relaxed (`max_path_occupancy_ratio=0.05`) so dynamic obstacles can be skirted instead of forcing the robot to stay glued to the global path.
- local-costmap-only debug mode is now repository-owned: Web `只启动局部障碍地图` starts JT128 driver prerequisites, chassis odometry, canonical TF helpers, `robot_local_perception`, and `src/robot_bringup/launch/local_costmap_debug.launch.py`; it publishes `/local_costmap/costmap` for obstacle-layer verification without planner/BT/robot_safety control output
- final safety arbitration runtime owner: this repository's C++ `robot_safety_node`, launched by `scripts/jetson/runtime_overlay/scripts/run_robot_safety.sh`
- robot safety runtime now publishes explicit arbitration state on `/safety/status` and `/safety/motion_allowed`, publishes safe motion commands on `/cmd_vel_safe`, and no longer has a Python fallback path
- Ranger Mini 3 mode adaptation runtime owner: this repository's `scripts/jetson/runtime_overlay/scripts/run_ranger_mini3_mode_controller.sh`; it consumes `/cmd_vel_safe` and is the only repository-owned publisher to `/cmd_vel` before `ranger_base_node`
- GS2 near-field docking lidar runtime owner: `src/robot_eai_gs2`, with helper `scripts/jetson/runtime_overlay/scripts/run_gs2_driver.sh`; it publishes `/dock/gs2_scan` and `/dock/gs2_points` in `gs2_link` and deliberately does not publish TF. The current front-center mount is single-sourced by `robot_description` as `base_link -> gs2_link`, `xyz=[0.36, 0.0, 0.290]`, `rpy=[0.0, 0.0, 0.0]`.
- Docking contact geometry is now explicit: `base_link -> charge_contact_link` is single-sourced by `robot_description` at `xyz=[0.398, 0.0, 0.255]`, which is 3.8 cm ahead of `gs2_link` on the same centerline. Docking config lives in `src/robot_nav_config/config/docking.yaml` and the Jetson overlay copy.
- Near-field charging alignment owner: `src/robot_docking_manager`, started only in docking mode with `scripts/jetson/runtime_overlay/scripts/run_docking_manager.sh`. It reads `/dock/gs2_scan`, confirms contact through `/battery_state`, treats positive charging current as an immediate hard stop in any docking phase, exposes `/docking/start` and `/docking/stop`, publishes `/docking/status`, and sends low-speed commands to `/cmd_vel_collision_checked` so `robot_safety` remains the final arbiter.
- Common services now start the GS2 driver after `robot_description_static_tf_node`, so `base_link -> gs2_link` is available before `/dock/gs2_scan` is used. Set `NJRH_GS2_AUTOSTART=false` only when the GS2 is physically disconnected during bench tests.
- Ranger chassis startup is single-instance guarded per CAN interface by `run_ranger_chassis.sh`; if the dashboard already owns `ranger_base_node` on `can0`, localization attaches as a monitor instead of starting a second driver against the same CAN bus.
- repository-owned bringup now includes `src/robot_bringup/launch/localization_bringup.launch.py` and `navigation_bringup.launch.py`, wiring the canonical stack to the repo-owned standard navigation chain with `robot_nav_config/config/nav2.yaml`
- repository-owned standard navigation now explicitly launches `behavior_server`, `velocity_smoother`, and `collision_monitor`, with remaps that force all Nav2 motion outputs through `cmd_vel_nav_raw -> cmd_vel_nav -> cmd_vel_collision_checked -> robot_safety -> /cmd_vel_safe -> ranger_mini3_mode_controller -> /cmd_vel -> ranger_base_node`
- multi-floor runtime assets now have a repository-owned contract: `maps_release/<building_id>/<floor_id>/nav`, `localizer`, `filters`, `reports`, and `poses.yaml`. `robot_floor_manager` provides `/floor_manager/switch_floor`, validates the floor bundle, loads the Nav2 map, applies localizer assets, triggers relocalization, and clears costmaps without publishing TF.
- `scripts/jetson/njrh_container.sh` now prepares `maps_release` as an App/API-writable bind-mounted asset root on container start/common-service start (`root:root`, directory mode `2775`, file mode `664`). This keeps map, pose, keepout, runtime preview, and dashboard-generated assets under one container ownership model.
- Nav2 costmap filters are now wired into the standard navigation path. `standard_navigation.launch.py` starts keepout and speed mask map servers plus their `costmap_filter_info_server` nodes; global costmap consumes `KeepoutFilter` and `SpeedFilter`. Runtime uses the selected floor's `filters/keepout_mask.yaml` and `filters/speed_mask.yaml`, or generates same-size neutral masks from the current Nav2 map when no floor bundle is selected.
- saved flat Web maps can be promoted into a floor bundle with `scripts/jetson/runtime_overlay/scripts/promote_map_to_floor.sh <map_name> <building_id> <floor_id>`, and runtime localization/navigation can select the bundle through `NJRH_BUILDING_ID` + `NJRH_FLOOR_ID`.
- Web dashboard floor controls are test-only: list floor assets, promote a saved map into a floor bundle, select a floor bundle for later Web-launched stacks, and call `/floor_manager/switch_floor`. Selection-only floor switches no longer require a live `/map_server`; full map/localizer/Nav2 loading happens when navigation resume is requested. They exercise the repository-owned floor contract during field testing and are not the production mission UI.
- Android / external App integration now has a separate production gateway package, `src/robot_api_server`. It exposes a narrow HTTP API for status, safety stop/resume, floor switching, localization trigger, map listing, `POST /api/v1/mapping/2d/start` for repository-owned `slam_toolbox` mapping startup, `POST /api/v1/mapping/2d/stop` / `POST /api/v1/mapping/stop` for stopping the App-started 2D mapping process group, `POST /api/v1/mapping/2d/save` / `POST /api/v1/mapping/save` for saving the current slam_toolbox occupancy into runtime previews plus `maps_release/<building>/<floor>/maps/<map_id>/manifest.json` business map assets, and live `GET /api/v1/mapping/2d/map` PNG rendering from the current `slam_toolbox /map`, plus `WS /ws/v1/teleop` for low-speed App mapping movement. Activated maps are copied into `maps_release/<building>/<floor>/current/` as fixed role files (`nav_map.yaml`, `localizer_map.png`) so Nav2/Isaac/floor_manager do not depend on user-visible names; stale root-owned `current/` directories are quarantined before the backend recreates the runtime mirror. Editable App overlays are backend-owned through `GET /api/v1/maps/semantic_layer`, `GET /api/v1/maps/poses`, `GET /api/v1/maps/filters/keepout`, `POST /api/v1/maps/poses`, `PUT /api/v1/maps/poses/{pose_id}`, `DELETE /api/v1/maps/poses/{pose_id}`, `PUT /api/v1/maps/poses/batch`, legacy `POST /api/v1/maps/poses/save`, and `POST /api/v1/maps/filters/keepout/save`; Android must not restore points or keepout lines from local files. `GET /api/v1/status` now also subscribes Ranger `/battery_state` and returns `bms.soc` as the real chassis battery percentage for the App. Page-scoped `POST /api/v1/subscriptions/acquire|heartbeat|release` controls `status`, `live_map`, `scan`, `tf`, and `teleop`; high-rate `/map`, `/scan`, and `/tf` are subscribed only while acquired and are TTL-released after App disconnect/crash. Saved map PNG preview remains explicit through `?source=saved` or `?name=<map>`. WebSocket teleop publishes only to `/cmd_vel_collision_checked`, so it still goes through `robot_safety` and the Ranger Mini 3 mode controller instead of bypassing the safety chain; reverse is enabled only during active mapping teleop via `/ranger_mini3/allow_reverse`, while navigation keeps `allow_reverse:false`.
- App 2D mapping stop/save now closes the full mapping-side chain, not only `slam_toolbox`: `run_projected_map.sh`, slam_toolbox launch, scan preprocessing/republishing nodes, and the FAST-LIO2 deskew source (`fastlio_mapping` / `laser_mapping`) are all cleaned up while common driver, chassis, TF, local-state, safety, and API services remain alive.
- Runtime service ownership is now split into long-lived common services and mode services. Common services can be started with `scripts/jetson/runtime_overlay/scripts/run_common_services.sh`; navigation and mapping scripts reuse them by default instead of killing/restarting driver, chassis, TF, local-state, local-perception, safety, floor-manager, or App API processes.
- `scripts/jetson/njrh_container.sh start-runtime` now starts the container plus production common ROS services. Use `start-dashboard` only when the debug Web observation window is needed.
- Runtime permission policy is explicit: new `NJRH-car` containers default to `root`, production ROS services and `robot_api_server` run as `root`, runtime writes use `umask 0002`, and released/runtime assets must stay `root:root` (`2775` directories, `664` files). See [reports/runtime_permission_audit.md](reports/runtime_permission_audit.md).
- Jetson boot autostart is installed through `scripts/jetson/install_njrh_autostart.sh`; it enables host `systemd` unit `njrh-runtime.service`, which starts the container and common ROS services but not the Web dashboard.
- web `标准导航` now enters repository-owned `src/robot_bringup/launch/standard_navigation.launch.py` for the Nav2 stack itself, while preserving the dashboard's existing separate localization startup step
- web navigation startup no longer starts FAST-LIO2 for the localization handoff; `run_occupancy_grid_localization.sh` now consumes canonical raw `/lidar_points` for stationary Isaac relocalization and standard Nav2 still waits for `map -> odom` before starting
- web `重启定位` now follows the same ownership rule: it restarts the navigation localization stack and raw `/lidar_points` sensing chain without launching a FAST-LIO process
- web navigation and `重启定位` no longer wait for `base_link -> lidar_link` before the localization stack starts; that static TF is now waited on only after `run_occupancy_grid_localization.sh` has launched the canonical TF helpers
- web standard navigation now starts the localization stack, triggers Isaac relocalization, then waits for a live `localization_result` and `map -> odom` before bringing up the standard Nav2 stack; this avoids deadlocking on localization data that has not been triggered yet
- Standard navigation now also enforces the map lifecycle boundary: `run_occupancy_grid_localization.sh` activates `/map_server` and waits for `/map`, while `run_nav2_navigation.sh` refuses to start Nav2 until `/map_server` is active and then waits for `/global_costmap/costmap` to resize from the static map. This prevents the planner from running against Nav2's default 5 m x 5 m costmap window.
- JT128 ingress now forces the upstream Hesai helper into its system-time timestamp profile (`use_timestamp_type=1`) while preserving repository-owned topic/frame remaps, so `/lidar_points`, `/lidar_imu`, `/wheel/odom`, and EKF output share the same ROS time base
- `robot_localization_bridge` now runs as C++, latches a successful Isaac `localization_result`, and keeps publishing the derived `map -> odom` from live odometry instead of timing out one-shot occupancy-localizer results after one second
- Standard navigation keeps `RotationShimController` in front of MPPI so Ranger Mini 3 can perform direct heading alignment / turn-in-place behavior on large heading errors, while MPPI remains the primary path-following and obstacle-avoidance controller.
- occupancy localization sensing is now repository-owned as well: it uses canonical raw `/lidar_points(lidar_link) -> nav_cloud_preprocessor(lidar_level_link) -> /points_nav(lidar_level_link) -> pointcloud_to_laserscan(target_frame=lidar_level_link) -> C++ scan_republisher_node -> /scan -> laser_scan_to_flatscan -> /flatscan`, instead of launching FAST-LIO2 or the older upstream `jt128_nav_sensing.launch.py` filter chain
- Isaac NITROS graph cache under `/tmp/isaac_ros_nitros/graphs` is prepared by the container launcher as `root:root` with `1777` permissions, so the root runtime can create graph folders without mixed ownership
- live TF cutover status on Jetson:
  - the live 2D mapping path no longer starts any extra static TF publishers
  - raw `/lidar_points` now publishes with `header.frame_id=lidar_link`; `/lidar_imu` publishes with `header.frame_id=imu_link`
  - mapping live graph no longer exposes `hesai_lidar_fastlio` in the canonical tree
  - localization entrypoint now prepares a PNG-backed localizer yaml instead of feeding Nav2 `pgm` directly into Isaac localizer
  - live `map -> odom` still depends on successful localizer matching; current `test-11` / `test-12` assets start the localizer stack but do not yet yield healthy localization

Windows entrypoint:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\jetson\Invoke-NJRHJetson.ps1 -Action start-runtime
powershell -ExecutionPolicy Bypass -File .\scripts\jetson\Invoke-NJRHJetson.ps1 -Action open-dashboard
```

## Canonical TF

```text
map
 └── odom                      (only robot_localization_bridge)
      └── base_link            (only robot_local_state)
           ├── lidar_link      (static)
           ├── imu_link        (static)
           ├── base_footprint  (optional static)
           └── other static frames
```

See [docs/tf_canonical_policy.md](docs/tf_canonical_policy.md) for ownership, suppression rules, and wrapper-level TF constraints.

## Local Reuse

- Windows car repository: `D:\codespace\car`
- Jetson host workspace: `/home/nvidia/workspaces/njrh-v3/workspace1`
- Jetson container workspace: `/workspaces/njrh-v3/workspace1`
- Jetson upstream asset workspace: `/home/nvidia/workspaces/isaac_ros-dev`
- JT128 lidar IP: `192.168.1.201`
- Jetson lidar host IP: `192.168.1.100`
- preferred Jetson interface: `eth1`
- fallback interface seen in recent tests: `eth0`

## Bootstrapping

```bash
source scripts/env.sh
export CAR_PROJECT_ROOT=/path/to/car
./scripts/bootstrap.sh
python3 scripts/scan_car_project.py --root .
python3 scripts/tf_audit.py --root .
python3 scripts/resolve_third_party.py --root .
```

## First-Round Deliverables

- `reports/car_project_reuse_report.md`
- `reports/tf_audit_report.md`
- `reports/third_party_resolution_report.md`
- `reports/occupancy_builder_design.md`
- `src/robot_interfaces`
- `src/robot_description`
- `src/robot_chassis_bridge`
- `src/robot_hesai_jt128`
- `src/robot_fastlio_mapping`
- `src/robot_pgo_mapping`
- `src/robot_map_toolkit`
- `src/robot_local_state`
- `src/robot_global_localization`
- `src/robot_localization_bridge`
- `src/robot_local_perception`
- `src/robot_occupancy_builder`
- `src/robot_safety`
- `src/robot_floor_manager`
- `src/ranger_mini3_mode_controller`
- `src/robot_nav_config`
- `src/robot_bringup`
- `src/robot_system_tests`
- `scripts/jetson/njrh_container.sh`
- `scripts/jetson/Invoke-NJRHJetson.ps1`
- `docs/jetson_njrh_container_runtime.md`
