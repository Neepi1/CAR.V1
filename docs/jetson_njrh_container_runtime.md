# Jetson NJRH Runtime

This phase adds a temporary field runtime that runs from a repository-owned Jetson workspace while still reusing the validated `car` stack as an upstream asset source.

The control plane now comes from this repository's runtime overlay under `scripts/jetson/runtime_overlay`. The `car` workspace remains an upstream asset source for validated packages, parameters, and helper scripts, but it is no longer the intended owner of runtime orchestration.

## Runtime Goal

- Start a fixed Jetson container named `NJRH-car`
- Keep the runtime inside Docker
- Reuse the existing `web_dashboard` as the temporary operator frontend
- Support live mapping status, 3D map save, 2D map export/view, and navigation startup
- Saving a PGO map from the dashboard now performs a paired save: 3D first, then current `slam_toolbox /map` 2D save plus Isaac localizer asset generation

## Workspace Split

- project-owned Jetson workspace root: `/home/nvidia/workspaces/njrh-v3/workspace1`
- project-owned container mount: `/workspaces/njrh-v3/workspace1`
- upstream asset workspace root: `/home/nvidia/workspaces/isaac_ros-dev`
- upstream asset container mount: `/workspaces/isaac_ros-dev`
- upstream alias mount: `/workspaces/isaac_ros-dev-upstream`

The container now mounts both trees:

- the new repository workspace is the primary runtime root and contains `Dockerfile.car`, `scripts/jetson`, reports, and current package sources
- the historical `isaac_ros-dev` workspace remains mounted read-only as the upstream source of validated `car` scripts, dashboard assets, and installed packages
- the overlay carries `bringup_ranger_can.sh` and `shutdown_ranger_can.sh` compatibility helpers because the reused dashboard calls those filenames directly for CAN up/down operations from inside the container
- structured floor assets are written under `maps_release`. Container startup and common-service startup repair this bind-mounted directory to `admin:admin`, directory mode `2775`, and file mode `664`, so `robot_api_server` can save App-created bundles such as `maps_release/B1/F1/nav` and `maps_release/B1/F1/localizer`.

## Reused Assets

- Jetson host workspace: `/home/nvidia/workspaces/njrh-v3/workspace1`
- Dockerfile: `/home/nvidia/workspaces/njrh-v3/workspace1/Dockerfile.car`
- Dashboard entrypoint: `/home/nvidia/workspaces/njrh-v3/workspace1/scripts/jetson/njrh_container.sh`
- Mapping scripts:
  - `scripts/build_and_start.sh`
  - `scripts/run_fastlio_tf.sh`
  - `scripts/run_pgo.sh`
  - `scripts/export_pgo_map_2d.py`
- Navigation scripts:
  - `scripts/run_nav2_localization.sh`
  - `scripts/run_nav2_navigation.sh`

## Overlay Ownership

- Runtime launcher root: `scripts/jetson/runtime_overlay`
- Dashboard entrypoint owner: `scripts/jetson/runtime_overlay/scripts/run_web_dashboard.sh`
- Dashboard asset staging now uses writable project-owned `maps/`, `maps3d/`, and `waypoints/` directories under `runtime_overlay`, seeded from upstream assets instead of read-only symlinks
- Default Nav2 params owner: `scripts/jetson/runtime_overlay/config/nav2.yaml`
- Local obstacle runtime owner: `scripts/jetson/runtime_overlay/scripts/run_local_perception.sh`
- Local obstacle runtime now mirrors the validated `car` JT128 nav preprocessor contract in a repo-owned node: it transforms `/lidar_points` into `base_link`, applies range/height/azimuth/self/front masks, then publishes `/perception/obstacle_points` using mode profiles for `NORMAL`, `RAMP`, `ELEVATOR_WAIT`, and `DOORWAY`. NORMAL mode limits marking to the local Nav2 obstacle range (`5.50 m`), uses a marking height window of `0.40..1.30 m`, and enables voxel outlier filtering to suppress sparse far returns that otherwise remain as inflated local-costmap spots.
- `/perception/obstacle_points` and `/perception/clearing_points` are stamped from the latest available `odom -> base_link` TF when `restamp_to_latest_tf=true`. With `require_output_stamp_tf=true`, the node skips publishing until that TF exists, keeping the cloud stamp inside the local costmap TF buffer and preventing message-filter drops caused by perception processing latency or startup ordering.
- Local costmap clearing uses two sources: `/perception/obstacle_points` marks obstacles and clears free space up to real returns, while `/perception/clearing_points` publishes dense synthetic ray endpoints from the live `lidar_link` origin in `base_link`, so moved obstacles can clear even when the next JT128 scan has no real return behind them. The synthetic clearing fan includes near and far range steps (`0.50/1.00/2.00/3.50/5.50/8.00 m`) across dense low z layers; this avoids relying only on long slanted max-range rays, which can miss low near-field voxels close to the vehicle.
- The Nav2 local `VoxelLayer` keeps `z_resolution=0.10` and now uses `mark_threshold=2`, so a single marked voxel column is no longer enough to create a 2D occupied cell. This specifically reduces the small residual spots that remain after clearing when only one low voxel is left marked.
- Local-costmap-only debug mode is owned by `scripts/jetson/runtime_overlay/scripts/run_local_costmap_debug.sh` plus `src/robot_bringup/launch/local_costmap_debug.launch.py`. The dashboard button `只启动局部障碍地图` starts the JT128/chassis/TF prerequisites, `robot_local_perception`, and only Nav2 `controller_server` with `/local_costmap/costmap`; it intentionally does not start planner, BT navigator, velocity smoother, collision monitor, robot safety, or final `/cmd_vel`.
- Final command arbitration runtime owner: `scripts/jetson/runtime_overlay/scripts/run_robot_safety.sh`
- Final command arbitration runtime now exposes `/safety/status` and `/safety/motion_allowed` and publishes safe commands to `/cmd_vel_safe`, so field runtime can distinguish `ESTOP_ACTIVE`, `LOCALIZATION_INVALID`, and `COMMAND_STALE`
- Local odometry runtime owner: `scripts/jetson/runtime_overlay/scripts/run_local_state.sh`, which defaults to `robot_localization/ekf_node` named `robot_local_state`. The EKF fuses `/wheel/odom` planar velocity with `/lidar_imu` gyro yaw-rate and owns the only `odom -> base_link` TF. The JT128 driver runtime forces `use_timestamp_type=1`, so public `/lidar_points` and `/lidar_imu` headers use host/system time instead of the lidar sensor clock; this prevents mixed-clock EKF drift during left/right rotation. The IMU remap node replaces the Hesai driver's all-zero gyro covariance with a conservative diagonal covariance (`0.10, 0.10, 0.25`) before EKF fusion, keeping wheel odometry dominant while still using IMU yaw-rate for stabilization.
- Temporary navigation field mode defaults `NJRH_NAV_LOCAL_STATE_MODE=passthrough` in common services and occupancy localization startup, so navigation and relocalization keep the canonical `/local_state/odometry` interface but back it directly with Ranger `/wheel/odom`. The passthrough profile keeps `odom_yaw_offset_rad=0.0`; it only renames the SDK child frame to canonical `base_link` and does not rotate chassis yaw. The current JT128 install front/back correction is carried by `lidar_yaw=pi` and `imu_yaw=pi` in the static sensor TF, not by local odometry. Set `NJRH_NAV_LOCAL_STATE_MODE=ekf` to return navigation to wheel+JT128-IMU EKF.
- Ranger Mini 3 mode adaptation runtime owner: `scripts/jetson/runtime_overlay/scripts/run_ranger_mini3_mode_controller.sh`. It runs the C++ `ranger_mini3_mode_controller`, consumes `/cmd_vel_safe`, rejects lateral/crab commands during normal navigation, allows forced `crab` only for near-field docking, clamps reverse to zero except while `robot_docking_manager` asserts `/ranger_mini3/allow_reverse` during controlled undocking, converts steering requests above `0.698 rad` to pure spin commands, and publishes `/cmd_vel` for `ranger_base_node`.
- App API runtime owner: `scripts/jetson/runtime_overlay/scripts/run_robot_api_server_supervised.sh`. The common-service layer keeps `robot_api_server` supervised and `njrh_container.sh start-runtime` waits for `/api/v1/status` on port `8080`; if that health check does not pass, the runtime is not considered ready for App use.
- `run_ranger_chassis.sh` now owns a per-CAN single-instance guard. If another runtime helper has already started `ranger_base_node` on `can0`, a second helper stays alive as a monitor and does not open the same CAN device again.
- `run_ranger_chassis.sh` labels upstream Ranger odom with `base_frame=ranger_base_link`. `robot_local_state` consumes that native chassis odom and republishes the canonical `base_link`; no other node may publish `odom -> base_link`.
- Repository-owned bringup contract now exists in `src/robot_bringup/launch/localization_bringup.launch.py` and `navigation_bringup.launch.py`; Jetson shell helpers still wrap upstream scripts today, but they now have a repo launch target to converge to
- Web standard navigation owner: `scripts/jetson/runtime_overlay/scripts/run_nav2_navigation.sh`, which now launches `src/robot_bringup/launch/standard_navigation.launch.py` instead of the upstream `run_nav2_navigation.sh`, so `/api/navigation/start` enters repository-owned standard navigation without duplicating the separate localization startup
- The repo-owned standard navigation launch now instantiates `controller_server`, `behavior_server`, `velocity_smoother`, `collision_monitor`, and `lifecycle_manager_navigation` directly, with remaps that enforce the intended command path `cmd_vel_nav_raw -> cmd_vel_nav -> cmd_vel_collision_checked -> robot_safety -> /cmd_vel_safe -> ranger_mini3_mode_controller -> /cmd_vel -> ranger_base_node`
- Web standard navigation gating: `dashboard_server.py` now waits for `map -> odom` to become available before starting the standard Nav2 stack, and it no longer starts FAST-LIO2 during the localization handoff; `run_occupancy_grid_localization.sh` consumes canonical raw `/lidar_points` for stationary Isaac relocalization
- Web `restart localization` now mirrors the same ownership boundary: it restarts the Isaac localization stack plus canonical driver/TF prerequisites, and the localization script uses raw `/lidar_points` without inheriting or launching a dashboard-side FAST-LIO process
- Web navigation and `restart localization` no longer wait for `base_link -> lidar_link` before the localization stack starts; the dashboard now waits for that static TF only after `run_occupancy_grid_localization.sh` has launched `run_robot_description.sh`
- Web/App standard navigation now starts the localization stack, triggers Isaac relocalization, observes `localization_result` when it is still available, and treats `map -> odom` as the authoritative localization-ready signal before starting the standard Nav2 stack; this avoids failing when Isaac publishes its one-shot result before the backend wait subscribes
- The navigation startup guard now treats `/map_server` as a required lifecycle dependency. The localization helper activates `/map_server` and waits for `/map`; the Nav2 helper verifies that state and waits until `/global_costmap/costmap` has resized to the loaded static map before accepting navigation startup as healthy.
- JT128 ingress now forces the upstream Hesai helper into its system-time timestamp profile (`use_timestamp_type=1`) while preserving repository-owned topic/frame remaps, so `/lidar_points`, `/lidar_imu`, `/wheel/odom`, and EKF output share the same ROS time base
- `robot_localization_bridge` now latches a successful Isaac localization pose and keeps publishing `map -> odom` from live odometry until a newer global localization result arrives, which matches the one-shot behavior of occupancy-grid relocalization better than the previous 1-second pose timeout
- Occupancy localization sensing is now repository-owned as well: for navigation relocalization it consumes canonical raw `/lidar_points(lidar_link)` directly, then applies the same repo-owned `jt128_scan_slam2d.yaml` slicing geometry (`nav_cloud_preprocessor(lidar_level_link) -> /points_nav(lidar_level_link) -> pointcloud_to_laserscan(target_frame=lidar_level_link) -> C++ scan_republisher_node -> /scan`) and feeds `laser_scan_to_flatscan` to generate `/flatscan` for Isaac localizer
- The shared `jt128_scan_slam2d.yaml` slice now uses the leveled frame `lidar_level_link`, derived as `base_link -> lidar_level_link` with `xyz = lidar_link.xyz`, `yaw = lidar_link.yaw`, `roll = 0`, and `pitch = 0`; the current shared mapping/localization slice band in that frame is `min_height=-0.75m`, `max_height=0.35m`, selected as a thicker structural band to recover more long-range wall returns with the current tilted JT128 installation, full 360 degrees, `range_max=40m`
- `jt128_slam_toolbox_mapping.yaml` is tuned to consume the recovered high-rate scan stream during fast in-place rotation: `minimum_time_interval=0.05`, `minimum_travel_heading=0.02`, `minimum_travel_distance=0.04`, `scan_buffer_size=30`, and `transform_timeout=0.35`.
- Live 2D mapping owner: `scripts/jetson/runtime_overlay/scripts/run_projected_map.sh`, which now launches FAST-LIO2 first as a current-frame deskew source and feeds repository-owned `slam_toolbox` from `/cloud_registered_body` instead of raw `/lidar_points`; FAST-LIO2 TF stays remapped to `/tf_fastlio_internal` and the navigation main tree still comes only from `robot_local_state` plus static sensor TF
- Live 2D mapping currently starts `robot_local_state` with `LOCAL_STATE_MODE=passthrough` by default through `NJRH_SLAM2D_LOCAL_STATE_MODE`. The passthrough node republishes Ranger `/wheel/odom` as canonical `/local_state/odometry` with `odom_yaw_offset_rad=0.0` and `rotate_odom_position_with_yaw_offset=false`; Ranger SDK odometry is treated as the chassis truth. Public `/lidar_points` and `/lidar_imu` are canonical remap outputs from the JT128 driver path: raw axes are rotated as `x=raw_y`, `y=-raw_x`, `z=raw_z`, and static TF keeps only the physical installation pose. The current physical sensor relation uses `lidar_yaw=pi` and `imu_yaw=pi`; do not duplicate that correction in local odometry.
- Web `/api/mapping2d/start` now targets that same repository-owned `slam_toolbox` chain directly, instead of the historical `run_jt128_2d_mapping.sh` / cartographer path
- Fast-LIO runtime owner: `scripts/jetson/runtime_overlay/scripts/run_fastlio_tf.sh`, which now accepts only canonical `/lidar_points` + `/lidar_imu` with `sensor_frame_id=lidar_link` and rejects the previous Fast-LIO-only remap path
- 2D mapping directly reuses the shared `config/fastlio.yaml` FAST-LIO2 profile for deskewed current frames. Navigation relocalization does not start FAST-LIO2 by default because the intended field operation is a stationary global match; physical JT128 installation remains in static `base_link -> lidar_link/imu_link`, while FAST-LIO2 LiDAR-IMU extrinsic remains identity because the public `/lidar_points` and `/lidar_imu` are already canonicalized.
- The shared FAST-LIO2 profile is tuned for online mapping use: `point_filter_num=4`, `max_iteration=3`, `path_en=false`, and `map_en=false`. This keeps `/cloud_registered_body` available as the deskewed current-frame source while reducing input cloud load and avoiding avoidable Path/Laser-map publisher work during 2D mapping.
- Repository wrapper contract owner: `src/robot_fastlio_mapping/config/fastlio.yaml`, which now mirrors the same canonical-only input contract and no longer exposes `hesai_lidar_fastlio` as a repository-owned runtime path
- JT128 lidar web view owner: overlay-patched `web_dashboard/lidar_view.html`, which now defaults to `base_link` so operator inspection is chassis-frame first and raw sensor-frame display is debug-only
- JT128 ingress owner: `scripts/jetson/runtime_overlay/scripts/run_driver.sh`, which restores the validated project path: Hesai driver outputs vendor raw `/jt128/vendor/points_raw` and `/jt128/vendor/imu_raw`, then repository-owned remap helpers rotate them into canonical `/lidar_points(frame_id=lidar_link)` and `/lidar_imu(frame_id=imu_link)`.
- The canonical JT128 ingress remains repository-owned. Runtime now requires the compiled `robot_hesai_jt128` pointcloud and imu remap nodes; the Python remap fallbacks were removed so missing binaries fail fast.
- The C++ pointcloud remap includes an in-place fast path for the current JT128 canonical matrix (`x=-raw_y`, `y=-raw_x`, `z=raw_z`). This is required for keeping `/lidar_points`, FAST-LIO2 `/cloud_registered_body`, `/points_nav`, and `/scan` close to the live vendor stream rate under Jetson load.
- The canonical pointcloud remap publishes `/lidar_points` as a sensor stream (`best_effort`, queue depth `1`). This avoids DDS reliable-backpressure on 1.8 MB JT128 clouds and keeps the public `/lidar_points` rate aligned with the 20 Hz vendor source.
- Dashboard driver readiness now requires the full canonical ingress stack: live Hesai driver process, pointcloud/IMU remap helpers, and recent `/lidar_points` samples before `start_mapping` proceeds
- Dashboard driver readiness also accepts an already-fresh `/lidar_points` cache after transient probe errors. A `Timed out waiting for lidar driver output` message with `lidar_recent=True` used to indicate a false readiness failure in the Web wrapper, not a missing JT128 pointcloud stream.
- Dashboard `开始3D建图` now starts the formal backend too, not only the frontend: the live chain is JT128 canonical ingress + FAST-LIO2 + PGO + `slam_toolbox`
- Dashboard `停止底层感知` now stops that same canonical ingress stack as a unit, including the JT128 driver, canonical remap helpers, `run_projected_map.sh`, and mapping-side helper processes even after a dashboard restart
- The live `slam_toolbox` ready gate now allows up to 45 seconds for the first `/map` sample, which matches the observed Jetson bringup latency better than the previous 20-second guard
- The live `slam_toolbox` mapping chain keeps a repository-owned C++ `scan_republisher_node` between `/scan_raw` and `/scan`, defaulting to pass-through. Full scan reversal is only enabled when `NJRH_SLAM2D_FLIP_SCAN=1` is set for field debugging
- `run_projected_map.sh` and `run_occupancy_grid_localization.sh` now both kill stale `scan_republisher_node` processes before relaunch, so `/scan` stays single-publisher when switching between 2D mapping and Isaac relocalization
- The dashboard 2D page now renders OccupancyGrid in the normal y-up convention again; any orientation change should now come from the backend `/map`, not from custom front-end mirroring
- Release rebuild compatibility owner: `scripts/jetson/runtime_overlay/scripts/release_rebuild_compat.py`
- Upstream mapping and localization helpers are invoked only through the overlay wrappers
- Standard navigation now enters repository-owned `standard_navigation.launch.py` after starting the project-owned local perception and robot safety helpers; rapid navigation still hands control to the validated upstream rapid-avoidance helper.

## Canonical TF Cutover Status

Jetson live checks on `2026-04-20` confirmed the following:

- the live 2D mapping path no longer starts extra static TF publishers
- `/lidar_points.header.frame_id` is now `lidar_link`
- `/lidar_imu.header.frame_id` is now `imu_link`
- the mapping-side live `view_frames` graph is now `odom -> base_link -> lidar_mount_link -> lidar_link` plus `imu_link`, with no `hesai_lidar_fastlio` branch
- the localization-side `/tf` endpoint now includes `robot_localization_bridge`, but current `test-11` / `test-12` assets still leave `/localization/health=false`, so `map -> odom` has not yet been observed live

This means the live 2D chain is no longer a source of duplicate static TF. The remaining gap is not FAST-LIO TF pollution anymore; it is localization quality on the current 2D assets.

## CAN Prerequisite

Before starting mapping or occupancy localization with the overlay-owned canonical TF helpers, the Jetson host must have `can0` in the `up` state:

```bash
sudo bash /home/nvidia/workspaces/isaac_ros-dev/scripts/bringup_ranger_can_host.sh
```

The overlay now checks `/sys/class/net/can0/operstate` before killing any existing TF publishers, so a missing CAN link will fail fast instead of leaving the live graph half-switched.

## Localizer Asset Rule

Isaac Occupancy Grid Localizer does not accept Nav2 `pgm` images directly. The overlay now prepares a localizer-specific yaml that points at a `png` image before launching the localizer container:

- Nav2 map server still loads the original `*.yaml + *.pgm` asset
- Isaac localizer loads a generated `*.localizer.yaml` that references a sibling `*.png`
- the helper responsible for this conversion is `scripts/jetson/runtime_overlay/scripts/prepare_localizer_map.py`

This aligns the runtime with the repository rule that Nav2 map assets and localizer assets are related but not interchangeable.

The Isaac NITROS runtime also needs to create graph files under `/tmp/isaac_ros_nitros/graphs`. The container launcher keeps `/tmp/isaac_ros_nitros` and `graphs` owned by `root:root` with `1777` permissions, matching `/tmp` semantics: root owns the runtime directory, while the dashboard's `admin` user can still create per-run NITROS graph folders. If this directory is left root-owned without write permission for `admin`, the occupancy localizer exits before `/trigger_grid_search_localization` becomes available.

## Container Layout

- Host workspace mount: `/home/nvidia/workspaces/njrh-v3/workspace1 -> /workspaces/njrh-v3/workspace1`
- Upstream asset mount: `/home/nvidia/workspaces/isaac_ros-dev -> /workspaces/isaac_ros-dev`
- Upstream alias mount: `/home/nvidia/workspaces/isaac_ros-dev -> /workspaces/isaac_ros-dev-upstream`
- Container image default: `njrh-car:latest`
- Container name: `NJRH-car`
- Base image preference: `isaac_ros_dev-aarch64:latest`
- Dockerfile build network mode: `host` (avoids the current Jetson `iptables raw` bridge-endpoint failure during `docker build`)
- If `Dockerfile.car` cannot be rebuilt on the current Jetson Docker setup, the launcher falls back to `isaac_ros_dev-aarch64:latest` so the runtime can still start.
- Dashboard URL: `http://192.168.31.23:2048`

## Windows Entry

From `C:\Users\86236\Desktop\workspace1`:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\jetson\Invoke-NJRHJetson.ps1 -Action start-runtime
powershell -ExecutionPolicy Bypass -File .\scripts\jetson\Invoke-NJRHJetson.ps1 -Action start-dashboard
powershell -ExecutionPolicy Bypass -File .\scripts\jetson\Invoke-NJRHJetson.ps1 -Action open-dashboard
```

Useful follow-up actions:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\jetson\Invoke-NJRHJetson.ps1 -Action status
powershell -ExecutionPolicy Bypass -File .\scripts\jetson\Invoke-NJRHJetson.ps1 -Action shell
powershell -ExecutionPolicy Bypass -File .\scripts\jetson\Invoke-NJRHJetson.ps1 -Action dashboard-logs
powershell -ExecutionPolicy Bypass -File .\scripts\jetson\Invoke-NJRHJetson.ps1 -Action stop-container
```

## Frontend Flow

After opening the dashboard, the current field-default mapping flow is the existing 2D path:

1. Start 2D mapping to launch the repository-owned `slam_toolbox` chain.
2. Open the live 2D map view to inspect `/map`.
3. Use the standalone 2D save path to save the live `slam_toolbox` map and generate the matching Isaac localizer png/yaml beside it.
4. Open the saved 2D map view to inspect the navigation asset.
5. Select a 2D map and start standard navigation.

The 3D/PGO controls remain available for optional formal mapping: `开始3D建图` still launches JT128 + FAST-LIO2 + PGO + live `slam_toolbox`, and the paired 3D save still snapshots the current `slam_toolbox /map` after the PGO 3D asset is saved. This is not the current default field workflow.

For obstacle-layer debugging only, click `只启动局部障碍地图`. This opens the local costmap view (`source=local_costmap`) and should be used before full navigation when verifying `/perception/obstacle_points` marking and `/perception/clearing_points` clearing behavior.

Dynamic avoidance uses a two-stage policy. `/perception/obstacle_points` marks the local voxel costmap, while `/perception/clearing_points` clears stale voxels. Standard navigation wraps MPPI with `RotationShimController`: large heading errors are handled by direct heading alignment / turn-in-place behavior, while MPPI remains the primary controller for path following and obstacle avoidance. MPPI is still configured with an Ackermann motion model. The downstream C++ Ranger mode controller forbids crab/lateral output and reverse during navigation, converts steering requests above 40 degrees to spin, and otherwise passes forward Ackermann-style commands. Crab mode is reserved for `/docking/start` near-field charger alignment. `collision_monitor` should only hard-stop for close body-risk returns and should otherwise slow the robot rather than suppress forward motion.

The live MPPI envelope is constrained to the Ranger Mini 3 documented Ackermann radius: `AckermannConstraints.min_turning_r=0.81`, `vx_max=0.55`, `wz_max=0.70`, `vx_std=0.28`, `wz_std=0.45`. This avoids sampling trajectories that the chassis cannot execute while still sampling more turn candidates. The local planning window is `10m x 10m`, the MPPI horizon is `44 * 0.09s = 3.96s`, `ObstaclesCritic` is matched to `local_inflation_layer` (`inflation_radius=0.35`, `cost_scaling_factor=6.0`), and `PathAlignCritic.max_path_occupancy_ratio=0.05` so local dynamic obstacle avoidance can deviate from the global path earlier.

The balanced clearing profile is `clearing.virtual_rays.angular_resolution_deg=0.75`, `range_steps=[0.50, 1.00, 2.00, 3.50, 5.50, 8.00]`, 8 z endpoints, and `clearing.max_points=30000`. This keeps no-return clearing active over the enlarged local window without exploding point count. `collision_monitor` uses shorter timeouts (`source_timeout=0.6`, `stop_pub_timeout=0.3`) so stale perception affects velocity commands for less time.

## Saved Asset Locations

- 3D maps: `/home/nvidia/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/maps3d`
- 2D maps: `/home/nvidia/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/maps`
- Dashboard logs: `/home/nvidia/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/web_dashboard/runtime_logs`

## Paired Save Rule

- The dashboard save button now means `save 3D + save current slam_toolbox /map as paired 2D nav asset + generate Isaac localizer assets from the same occupancy result`
- Output names are coupled by default, for example `test-12.pcd/.ply`, `test-12.yaml/.pgm|.png`, `test-12.localizer.png`, and `test-12.localizer.yaml`
- The 2D nav asset now comes from the current live `slam_toolbox /map` occupancy grid, not from the legacy `export_pgo_map_2d.py` fallback chain
- The Isaac localizer image and yaml are generated from that same saved 2D nav occupancy result, so Nav2 and localizer assets stay source-aligned
- If 3D save succeeds but either the live `slam_toolbox` 2D save or localizer asset generation fails, the action reports failure instead of pretending the map is fully ready for navigation

Expected files:

- 3D map: `<name>.pcd`, `<name>.ply`
- live-saved 2D nav asset: `<name>.yaml`, `<name>.pgm`, `<name>.png`, optional `<name>.meta.json`
- live-saved localizer asset: `<name>.localizer.png`, `<name>.localizer.yaml`

## Known Limits

- This is a temporary runtime bridge built on top of the current `car` field stack.
- The dashboard runtime and the navigation-side perception/safety helpers are now started from this repository's overlay, but mapping, localization, chassis, and Nav2 data-plane launches still call validated upstream helper scripts because the final local wrappers are not fully production-ready yet.
- The live 2D map view has been cut over to a repository-owned `slam_toolbox` wrapper chain, but the dashboard still consumes it through the reused `/api/projected_map/latest` compatibility endpoint to avoid a larger frontend rewrite.
- The dashboard save button now targets operator-ready live `slam_toolbox` 2D assets; the formal `release_rebuild` chain still exists separately for offline production-grade assets built from `raw bag + optimized trajectory`.
- Live TF uniqueness for mapping has been rechecked successfully; localization still needs a healthy `localization_result` before `map -> odom` can be validated live.
- Current `test-11` / `test-12` localizer runs start successfully but remain unhealthy, which points to floor-map quality / alignment rather than TF ownership.
- Current Jetson Docker rebuild may fail with an `iptables raw table` error; the launcher therefore keeps a base-image fallback path enabled by default.

## Required Hardware Validation

- JT128 packet path on `eth1` with fallback to `eth0`
- Ranger Mini V3 CAN control inside the container
- FAST-LIO2 + PGO save service under real sensor load
- Occupancy localizer start, `map -> odom` uniqueness, and Nav2 startup in the same live graph
