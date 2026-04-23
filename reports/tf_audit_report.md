# TF Audit Report

- Audit mode: local repository scan plus explicit `D:\codespace\car` asset audit
- `view_frames` status: full `tf2_tools view_frames` dump still pending, but live Jetson checks were executed over SSH on `2026-04-19`

## Canonical Ownership
- `robot_local_state`: only `odom -> base_link`
- `robot_localization_bridge`: only `map -> odom`
- `robot_description`: only static sensor extrinsics

## Canonical TF Policy Files In This Repository
- `docs/tf_canonical_policy.md`
- `src/robot_nav_config/config/tf_policy.yaml`
- `src/robot_description/urdf/robot.urdf.xacro`

## Local Car Repo Findings That Conflict With The Canonical Tree

### FAST-LIO2 legacy publication behavior
- `D:\codespace\car\ros2_ws\src\fast_lio\config\jt128.yaml` sets `send_odom_base_tf: true`
- `D:\codespace\car\ros2_ws\src\fast_lio\config\jt128.yaml` uses `sensor_frame_id: hesai_lidar_fastlio`

### PGO legacy frame choices
- `D:\codespace\car\ros2_ws\src\fastlio_pgo\config\pgo.yaml` uses `map_frame: slam_map`
- `D:\codespace\car\ros2_ws\src\fastlio_pgo\config\pgo.yaml` uses `local_frame: camera_init`

### Legacy bridge and sensing stack references
- `D:\codespace\car\nav2_test\params\jt128_map_to_odom_tf_bridge.yaml` documents a legacy map-to-odom bridge
- `D:\codespace\car\docs\ranger_mini_v3_integration.md` documents the historical chain `map -> camera_init -> body -> base_link`
- `D:\codespace\car\nav2_test\jt128_nav_sensing.launch.py` publishes URDF-based lidar TF from the historical stack

### Jetson runtime evidence status
- Jetson SSH is reachable and the Humble environment is available.
- `/tf_static` publisher is now `robot_description_static_tf`; legacy `hesai_lidar_state_publisher` was removed from the live graph.
- `/lidar_points.header.frame_id` is now `lidar_link`.
- `/lidar_imu.header.frame_id` had historically been forced to match the FAST-LIO-private path; the repository runtime now owns ingress normalization and targets `imu_link` for canonical consumers.
- Jetson host `/sys/class/net/can0/operstate` is now `up`.
- The mapping-side live `view_frames` graph is `odom -> base_link -> {base_footprint, lidar_mount_link, lidar_link, imu_link}`.
- `hesai_lidar_fastlio` is absent from the mapping-side live TF graph after remapping FAST-LIO internal TF away from the main tree.
- The repository-owned Fast-LIO runtime now rejects the historical Fast-LIO-only remap path instead of keeping it as a supported fallback.
- The repository-owned JT128 runtime now stages vendor raw topics off-tree and applies the validated `car` axis-remap nodes before publishing canonical `/lidar_points` and `/lidar_imu`.
- Mapping and localization runtime cleanup no longer kill those canonical ingress remap nodes; only the historical Fast-LIO-private path remains suppressed.
- `robot_localization_bridge` now starts successfully in the localization stack, but no healthy `localization_result` has been observed yet on `test-11` / `test-12`, so `map -> odom` has not been validated live.
- Historical file `/home/nvidia/workspaces/isaac_ros-dev/frames_2026-04-18_14.53.17.gv` contains only `No tf data received`, so it is not valid TF evidence.

## Current Wrapper Suppression Decisions
- `src/robot_hesai_jt128/config/jt128.yaml` keeps `publish_vendor_tf: false`
- `src/robot_fastlio_mapping/config/fastlio.yaml` keeps `wrapper_override_publish_tf: false` and only allows canonical `/lidar_points` + `/lidar_imu` with `lidar_link`
- `src/robot_pgo_mapping/config/pgo.yaml` keeps `canonical_publish_tf: false`
- `src/robot_global_localization/config/global_localization.yaml` keeps `publish_tf: false`

## Occupancy Builder TF Boundary

### Ownership rule

- The new `robot_occupancy_builder` package must publish no TF.
- It may consume point clouds and a pose topic, but it must not create a new branch under the canonical navigation main tree.

### Live draft contract

- JT128 cloud input should arrive in the repository-owned sensor frame, not a third-party internal frame.
- The builder must take the repository-owned pose stream `/mapping/frontend_pose` instead of reading FAST-LIO internal TF directly.
- The builder publishes only `/mapping/draft_map` and optional debug topics.

### Release rebuild contract

- `release_rebuild` must use raw bag plus optimized trajectory files.
- Rebuild-time pose application must happen inside the package or its helper tools, not through ad-hoc TF publication.
- Release asset generation must not introduce extra `map -> odom` or `odom -> base_link` publishers.

### Canonical-tree protection

- `draft_map` is an `OccupancyGrid`, not a TF owner.
- `robot_local_perception` remains the only source for `/perception/obstacle_points`.
- The draft occupancy output must not be wired into the local costmap obstacle source.
- Mapping-internal frames such as FAST-LIO or PGO internal frames remain isolated from the canonical navigation tree.

## Current Conclusion
- The local car repository contains reusable parameters and source trees, but its historical internal frames are not canonical and must stay isolated.
- This repository now encodes the single canonical ownership for `map -> odom` and `odom -> base_link`, and the Jetson live graph already uses the project-owned static TF publisher plus canonical raw lidar frames.
- Canonical sensor ingress is now repository-owned: vendor raw JT128 data stays off the main tree, while repository wrappers publish the only normalized `/lidar_points` and `/lidar_imu` topics consumed by FAST-LIO, slam_toolbox, and operator tooling.
- FAST-LIO internal TF no longer contaminates the navigation main tree, and the repository wrapper contract now only permits canonical `lidar_link` input instead of the historical `hesai_lidar_fastlio` path.
- The remaining live gap is localization readiness: `robot_localization_bridge` is in the graph, but `map -> odom` still awaits a valid Isaac localization result from floor assets that match the current environment.
- The proposed `robot_occupancy_builder` addition is compatible with the canonical TF policy only if it remains a pure consumer of cloud plus pose topics and never becomes a TF publisher.
