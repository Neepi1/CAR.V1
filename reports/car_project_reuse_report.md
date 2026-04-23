# Car Project Reuse Report

- Workspace root: `C:\Users\86236\Desktop\workspace1`
- Local car repository: `D:\codespace\car`
- Jetson host context: `nvidia@192.168.31.23:/home/nvidia/workspaces/njrh-v3/workspace1`
- Jetson upstream asset workspace: `/home/nvidia/workspaces/isaac_ros-dev`
- Scan policy: local-first, network only as fallback
- Requested extra instruction file under `04_*OccupancyBuilder*.md` was not found in the current repository root during this scan; the design below therefore follows the user requirement in this thread plus the v3 spec files already present.

## Candidate Roots

- `D:\codespace\car`
- `D:\codespace\car\ros2_ws\src`
- `D:\codespace\car\nav2_test`
- `D:\codespace\car\scripts`
- `D:\codespace\car\web_dashboard`

## Direct Reuse

### JT128 network and driver parameters

- `D:\codespace\car\hesai_jt128\configs\jt128_network.yaml`
- `D:\codespace\car\hesai_jt128\configs\ros2_driver_params.yaml`
- `D:\codespace\car\ros2_ws\src\hesai_lidar_ros2\config\config.yaml`

### Sensor extrinsics and TF calibration references

- `D:\codespace\car\ros2_ws\src\car_description\launch\hesai_lidar_tf.launch.py`
- `D:\codespace\car\ros2_ws\src\car_description\urdf\hesai_lidar_mount.urdf.xacro`
- `D:\codespace\car\ros2_ws\src\jt128_nav_tools\config\jt128_fastlio_pointcloud_remap.yaml`
- `D:\codespace\car\ros2_ws\src\jt128_nav_tools\config\jt128_fastlio_imu_remap.yaml`

### FAST-LIO2 / PGO / localization / navigation configs

- `D:\codespace\car\ros2_ws\src\fast_lio\config\jt128.yaml`
- `D:\codespace\car\ros2_ws\src\fastlio_pgo\config\pgo.yaml`
- `D:\codespace\car\nav2_test\params\jt128_occupancy_grid_localizer.yaml`
- `D:\codespace\car\nav2_test\params\jt128_flatscan.yaml`
- `D:\codespace\car\nav2_test\params\jt128_map_to_odom_tf_bridge.yaml`
- `D:\codespace\car\nav2_test\params\nav2_jt128_rapid_avoidance.yaml`

### Chassis and platform assets

- `D:\codespace\car\ros2_ws\src\ranger_ros2`
- `D:\codespace\car\ros2_ws\src\ugv_sdk`
- `D:\codespace\car\docs\ranger_mini_v3_integration.md`
- `D:\codespace\car\Dockerfile.car`

### Runtime container and operator frontend assets

- `D:\codespace\car\scripts\run_web_dashboard.sh`
- `D:\codespace\car\web_dashboard\dashboard_server.py`
- `D:\codespace\car\web_dashboard\index.html`
- `D:\codespace\car\web_dashboard\map2d_view.html`
- `D:\codespace\car\web_dashboard\slam2d_view.html`
- `D:\codespace\car\scripts\build_and_start.sh`
- `D:\codespace\car\scripts\run_nav2_localization.sh`
- `D:\codespace\car\scripts\run_nav2_navigation.sh`
- `D:\codespace\car\scripts\run_pgo.sh`

## Occupancy Builder Reuse Candidates

### Live draft map reference logic

- `D:\codespace\car\scripts\projected_occupancy_mapper.py`
- `D:\codespace\car\scripts\run_projected_map.sh`

Reuse decision:

- Reuse the point-field parsing, Bresenham ray tracing, log-odds update, and `OccupancyGrid` publish pattern.
- Do not reuse its `accumulated_cloud_mode=true` path as the formal v3 release map path, because that path directly projects the current accumulated 3D cloud.
- Do not keep its historical topics as-is for the new package contract. The new package must expose `live_draft` on `/mapping/draft_map` and take `/mapping/frontend_pose` as the pose input.

### Ground / obstacle semantic filtering reference logic

- `D:\codespace\car\ros2_ws\src\jt128_nav_tools\src\terrain_map_builder_node.cpp`
- `D:\codespace\car\nav2_test\params\jt128_nav_cloud_preprocessor.yaml`
- `D:\codespace\car\nav2_test\jt128_nav_sensing.launch.py`

Reuse decision:

- Reuse the validated range, height, azimuth, self-mask, and front-mask parameter conventions for JT128.
- Reuse the terrain-grid style neighborhood-ground estimation idea to separate ground-like returns from obstacle-like returns.
- Extend it in the new package from a binary obstacle extractor into three classes required by the new chain: `ground`, `ramp`, and `obstacle`.

### Historical 2D export reference logic

- `D:\codespace\car\scripts\export_pgo_map_2d.py`

Reuse decision:

- Reuse only as a reference for post-processing and asset file emission.
- Do not treat it as the formal release path, because it consumes a saved PLY and directly rasterizes the final 3D result. The new requirement explicitly forbids using direct final-PCD or final-PLY projection as the official release 2D asset path.

### Existing topic naming and operator expectations

- `D:\codespace\car\nav2_test\params\jt128_nav_cloud_preprocessor.yaml`
- `D:\codespace\car\scripts\run_projected_map.sh`
- `D:\codespace\car\web_dashboard\dashboard_server.py`

Observed historical topics:

- Raw cloud: `/lidar_points`
- FAST-LIO accumulated cloud: `/Laser_map`
- Legacy odom input for 2D projected occupancy: `/Odometry`
- Legacy projected occupancy topic: `/projected_map`

New project decision:

- Keep raw JT128 compatibility through wrapper remap, but standardize the new builder input on the repository-owned JT128 wrapper output plus `/mapping/frontend_pose`.
- The new live draft topic is fixed to `/mapping/draft_map`.
- The draft map stays out of `robot_local_perception` and out of the local costmap obstacle source.

## Wrapped Reuse

- `hesai_lidar_ros2` remains the upstream driver source, but TF publication stays disabled in `robot_hesai_jt128`.
- `fast_lio` remains the upstream frontend source, but the internal FAST-LIO sensor frame stays outside the canonical navigation TF tree.
- `fastlio_pgo` remains the upstream backend source, but `slam_map` and `camera_init` remain internal backend frames only.
- `jt128_nav_tools` remains the source of validated remap, preprocessor, and semantic filtering references.
- `jt128_nav_tools` `pointcloud_axis_remap` and `imu_axis_remap` are now reused at the JT128 wrapper ingress so canonical `/lidar_points` and `/lidar_imu` share the same numeric truth as the main TF tree instead of a FAST-LIO-private correction path.
- The temporary Jetson `NJRH-car` runtime still provides the operator shell and web assets, but the new occupancy builder contract belongs to this repository and must be implemented as a repository-owned package.

## Key Parameter Decisions Imported From Car Repo

- JT128 lidar IP: `192.168.1.201`
- Jetson lidar host IP: `192.168.1.100`
- Preferred Jetson lidar NIC: `eth1`
- Observed fallback NIC in recent tests: `eth0`
- Hesai default vendor frame: `hesai_lidar`
- FAST-LIO2 upstream sensor frame: `hesai_lidar_fastlio`
- PGO upstream frames: `slam_map` and `camera_init`
- Nav sensing prefilter frame: `base_link`
- Nav sensing broad height gate: `[-0.20, 1.60]`
- Nav sensing azimuth gate: `[-110 deg, +110 deg]`

## Reuse Gaps And New Work Required In This Repository

- There is no existing package in the current repository that implements a repository-owned `JT128 + pose -> occupancy builder`.
- `robot_map_toolkit` is still a skeleton and does not yet implement `raw bag + optimized trajectory -> shared occupancy intermediate -> nav_map/localizer_map`.
- The current car-side projected occupancy path is suitable only as a live draft reference, not as the release asset builder.
- The current car-side PGO 2D export path is suitable only as a heuristic reference, not as the formal release asset builder.

## Immediate Conclusion

- Local car assets are sufficient to start the design and the first repository-owned wrapper package without downloading new third-party software.
- The correct reuse split is:
  - reuse JT128 network, TF, sensing masks, FAST-LIO2 and PGO wrappers, and occupancy update heuristics;
  - do not reuse direct accumulated-cloud projection as the release map path;
  - do not reuse direct saved-PCD or saved-PLY projection as the official release map path.
- The next implementation step, after design confirmation, is to add a new `robot_occupancy_builder` package with `live_draft` and `release_rebuild` modes while keeping the navigation main chain unchanged for now.
