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
- dashboard URL: `http://192.168.31.23:2048`
- operator guide: `docs/jetson_njrh_container_runtime.md`
- runtime orchestration owner: this repository's `scripts/jetson/runtime_overlay`
- CAN up/down helpers are now present in the overlay as `scripts/jetson/runtime_overlay/scripts/bringup_ranger_can.sh` and `shutdown_ranger_can.sh`, matching the reused dashboard's expected filenames
- live 2D mapping owner: `slam_toolbox` launched by this repository's `scripts/jetson/runtime_overlay/scripts/run_projected_map.sh`, exposed to the reused dashboard through the compatibility `/api/projected_map/latest` endpoint and explicitly reusing the existing TF tree without injecting extra static sensor TF
- web `/api/mapping2d/start` now enters that same repository-owned `slam_toolbox` chain directly and no longer falls back to the historical `run_jt128_2d_mapping.sh` / cartographer path
- current live 2D source for the operator view: canonical `/lidar_points(lidar_link) -> nav_cloud_preprocessor(lidar_level_link) -> /points_nav(lidar_level_link) -> pointcloud_to_laserscan(target_frame=lidar_level_link) -> C++ scan_republisher_node -> slam_toolbox /map`
- repository-owned 2D scan contract owner: `scripts/jetson/runtime_overlay/config/jt128_scan_slam2d.yaml`, shared by both slam_toolbox mapping and Isaac relocalization sensing; it now uses the leveled slice frame `lidar_level_link` (`base_link -> lidar_level_link`: same XYZ as `lidar_link`, yaw aligned to `lidar_link`, roll/pitch forced to zero) with the current slice band `-0.85m .. -0.20m` in that leveled frame, which corresponds to roughly `0.20m .. 0.85m` in `base_link` at the current lidar height
- Fast-LIO runtime now accepts only the canonical sensor topics `/lidar_points` and `/lidar_imu` with `sensor_frame_id=lidar_link`; the repository-owned runtime no longer permits the old Fast-LIO-only remap path
- repository wrapper contract now matches the Jetson runtime: `robot_fastlio_mapping` is canonical-only and no longer exposes `hesai_lidar_fastlio` as a runtime fallback
- JT128 point-cloud web view now defaults to the chassis frame `base_link`; raw sensor coordinates remain available only as an explicit debug toggle
- JT128 ingress now follows the validated canonical path again: the driver publishes vendor raw `/jt128/vendor/points_raw` and `/jt128/vendor/imu_raw`, then repository-owned remap helpers normalize them into public `/lidar_points` and `/lidar_imu`
- the canonical JT128 ingress remains repository-owned, and runtime now requires the compiled `robot_hesai_jt128` pointcloud and imu remap nodes; Python remap fallbacks were removed so a missing binary fails fast instead of degrading runtime performance
- Dashboard driver readiness now keys off the live Hesai driver plus the canonical pointcloud/IMU remap helpers and recent `/lidar_points` samples
- Dashboard `slam_toolbox` startup now allows a longer first-map warmup window before declaring failure, so slow first `/map` publication no longer aborts an otherwise healthy mapping start
- The live `slam_toolbox` chain keeps a repository-owned C++ `scan_republisher_node` between `/scan_raw` and `/scan`, defaulting to pass-through. Full scan reversal is opt-in via `NJRH_SLAM2D_FLIP_SCAN=1` for field debugging only
- the 2D mapping and occupancy-localization launch scripts now both clear stale `scan_republisher_node` processes before relaunch, preventing duplicate `/scan` publishers from blocking the flatscan path
- The 2D web view is back to rendering OccupancyGrid in its normal y-up convention and no longer carries a web-only orientation workaround
- Dashboard `开始3D建图` now starts the formal mapping backend as well: JT128 canonical ingress + FAST-LIO2 frontend + PGO backend + live `slam_toolbox` 2D view
- dashboard save behavior: paired `PGO 3D save -> current slam_toolbox /map snapshot -> Isaac localizer asset generation`, and the standalone 2D save path now follows the same `slam_toolbox nav map + localizer png/yaml` rule
- dashboard runtime asset directories `maps/`, `maps3d/`, and `waypoints/` are now writable project-owned mirrors seeded from the upstream `car` workspace instead of read-only symlinks
- dashboard `停止底层感知` now targets the direct JT128 driver plus FAST-LIO2, slam_toolbox live 2D chain, and related mapping-side helpers as a unit
- default navigation params owner: this repository's `scripts/jetson/runtime_overlay/config/nav2.yaml`
- local perception runtime owner: this repository's `scripts/jetson/runtime_overlay/scripts/run_local_perception.sh`
- local perception runtime now reuses the validated `car` JT128 nav preprocessor contract but executes a repo-owned filter that transforms `/lidar_points` into `base_link`, applies range/height/sector/body masks, publishes `/perception/obstacle_points` for local marking plus real-return clearing, and publishes synthetic `/perception/clearing_points` ray endpoints for no-return local costmap clearing. NORMAL mode keeps marking local (`range_filter.max=4.50`, `height_filter=0.40..1.30`) and enables voxel outlier filtering so sparse far returns are not repeatedly inflated into local obstacles. Clearing rays now use a balanced Jetson profile (`0.75 deg`, range steps `0.50/1.00/2.00/3.50/6.00 m`, 8 z endpoints, `max_points=30000`) to reduce VoxelLayer raytracing load while preserving stale-obstacle clearing.
- local perception stamps `/perception/obstacle_points` and `/perception/clearing_points` with the latest available `odom -> base_link` TF time (`output_stamp_tf_target_frame=odom`) instead of raw processing wall time. With `require_output_stamp_tf=true`, frames are skipped until that TF is available, preventing Nav2 local costmap message-filter drops where perception clouds arrive outside the costmap TF cache.
- local Nav2 voxel occupancy projection now requires at least 2 marked voxels in a column before a 2D local-costmap cell becomes occupied (`voxel_layer.mark_threshold=2`), which suppresses single-voxel residue after clearing while preserving real obstacles that span multiple height bins.
- local dynamic-obstacle avoidance is tuned for Ranger Mini 3 four-wheel-drive/four-wheel-steering with a repository-owned C++ `ranger_mini3_mode_controller` between `robot_safety` and `ranger_base_node`: lateral/crab commands are rejected, reverse is clamped out, large steering requests are converted to spin, and normal forward turns are passed as Ackermann-style `Twist` commands.
- MPPI now uses Ranger Mini 3's documented Ackermann radius envelope (`min_turning_r=0.81`, `wz_max=0.70`) instead of sampling sub-physical 0.35 m turns; its obstacle critic now matches the local inflation layer (`inflation_radius=0.35`, `cost_scaling_factor=6.0`) and PathAlign is relaxed so dynamic obstacles can be skirted instead of forcing the robot to stay glued to the global path.
- local-costmap-only debug mode is now repository-owned: Web `只启动局部障碍地图` starts JT128 driver prerequisites, chassis odometry, canonical TF helpers, `robot_local_perception`, and `src/robot_bringup/launch/local_costmap_debug.launch.py`; it publishes `/local_costmap/costmap` for obstacle-layer verification without planner/BT/robot_safety control output
- final safety arbitration runtime owner: this repository's C++ `robot_safety_node`, launched by `scripts/jetson/runtime_overlay/scripts/run_robot_safety.sh`
- robot safety runtime now publishes explicit arbitration state on `/safety/status` and `/safety/motion_allowed`, publishes safe motion commands on `/cmd_vel_safe`, and no longer has a Python fallback path
- Ranger Mini 3 mode adaptation runtime owner: this repository's `scripts/jetson/runtime_overlay/scripts/run_ranger_mini3_mode_controller.sh`; it consumes `/cmd_vel_safe` and is the only repository-owned publisher to `/cmd_vel` before `ranger_base_node`
- Ranger chassis startup is single-instance guarded per CAN interface by `run_ranger_chassis.sh`; if the dashboard already owns `ranger_base_node` on `can0`, localization attaches as a monitor instead of starting a second driver against the same CAN bus.
- repository-owned bringup now includes `src/robot_bringup/launch/localization_bringup.launch.py` and `navigation_bringup.launch.py`, wiring the canonical stack to the repo-owned standard navigation chain with `robot_nav_config/config/nav2.yaml`
- repository-owned standard navigation now explicitly launches `behavior_server`, `velocity_smoother`, and `collision_monitor`, with remaps that force all Nav2 motion outputs through `cmd_vel_nav_raw -> cmd_vel_nav -> cmd_vel_collision_checked -> robot_safety -> /cmd_vel_safe -> ranger_mini3_mode_controller -> /cmd_vel -> ranger_base_node`
- web `标准导航` now enters repository-owned `src/robot_bringup/launch/standard_navigation.launch.py` for the Nav2 stack itself, while preserving the dashboard's existing separate localization startup step
- web navigation startup no longer spins FAST-LIO2 back up during the localization handoff, and standard Nav2 now waits for `map -> odom` to appear before the navigation stack is started
- web `重启定位` now follows the same rule: it restarts only the navigation localization stack and no longer starts FAST-LIO2 before Isaac localization tears it back down
- web navigation and `重启定位` no longer wait for `base_link -> lidar_link` before the localization stack starts; that static TF is now waited on only after `run_occupancy_grid_localization.sh` has launched the canonical TF helpers
- web standard navigation now starts the localization stack, triggers Isaac relocalization, then waits for a live `localization_result` and `map -> odom` before bringing up the standard Nav2 stack; this avoids deadlocking on localization data that has not been triggered yet
- Standard navigation now also enforces the map lifecycle boundary: `run_occupancy_grid_localization.sh` activates `/map_server` and waits for `/map`, while `run_nav2_navigation.sh` refuses to start Nav2 until `/map_server` is active and then waits for `/global_costmap/costmap` to resize from the static map. This prevents the planner from running against Nav2's default 5 m x 5 m costmap window.
- navigation and relocalization now keep the verified live JT128 ingress profile instead of switching the Hesai driver into the upstream `navigation` timestamp mode that leaves `/lidar_points` and `/lidar_imu` without live samples on the current Jetson stack
- `robot_localization_bridge` now runs as C++, latches a successful Isaac `localization_result`, and keeps publishing the derived `map -> odom` from live odometry instead of timing out one-shot occupancy-localizer results after one second
- Standard navigation keeps `RotationShimController` in front of MPPI so Ranger Mini 3 can perform direct heading alignment / turn-in-place behavior on large heading errors, while MPPI remains the primary path-following and obstacle-avoidance controller.
- occupancy localization sensing is now repository-owned as well: it reuses the same repo-owned `jt128_scan_slam2d.yaml` scan contract as live `slam_toolbox` mapping (`/lidar_points -> pointcloud_to_laserscan -> C++ scan_republisher_node -> /scan`) plus `laser_scan_to_flatscan`, instead of the older upstream `jt128_nav_sensing.launch.py` filter chain
- Isaac NITROS graph cache under `/tmp/isaac_ros_nitros/graphs` is prepared by the container launcher as `root:root` with `1777` permissions, so the `admin` runtime can create graph folders without handing directory ownership away from root
- live TF cutover status on Jetson:
  - the live 2D mapping path no longer starts any extra static TF publishers
  - raw `/lidar_points` and `/lidar_imu` now publish with `header.frame_id=lidar_link`
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
- `src/ranger_mini3_mode_controller`
- `src/robot_nav_config`
- `src/robot_bringup`
- `src/robot_system_tests`
- `scripts/jetson/njrh_container.sh`
- `scripts/jetson/Invoke-NJRHJetson.ps1`
- `docs/jetson_njrh_container_runtime.md`
