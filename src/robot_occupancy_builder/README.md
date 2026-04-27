# robot_occupancy_builder

Repository-owned occupancy builder package for the v3 mapping workflow.

## Modes

- `live_draft`: subscribes to JT128 points plus `/mapping/frontend_pose` and publishes `/mapping/draft_map`
- `release_rebuild`: replays raw bag data against the optimized trajectory and rebuilds formal `nav_map` and `localizer_map` assets from the same occupancy intermediate
- `release_rebuild` now also emits empty filter mask placeholders and `poses.yaml` so the output satisfies the multi-floor asset contract consumed by `robot_floor_manager`

## Reuse Sources

- JT128 filtering and mask conventions reuse `D:/codespace/car/nav2_test/params/jt128_nav_cloud_preprocessor.yaml`
- free-space ray tracing and log-odds accumulation reuse ideas from `D:/codespace/car/scripts/projected_occupancy_mapper.py`
- ground-neighborhood estimation reuses ideas from `D:/codespace/car/ros2_ws/src/jt128_nav_tools/src/terrain_map_builder_node.cpp`
- direct saved-PLY projection from `export_pgo_map_2d.py` is intentionally not used as the formal release path

## Live Draft Contract

- `points_topic`: defaults to `/sensors/lidar/points_raw`
- `pose_topic`: fixed default `/mapping/frontend_pose`
- `map_topic`: fixed default `/mapping/draft_map`
- `base_frame`: defaults to `base_link`
- `use_tf_for_sensor_extrinsics`: defaults to `true`

The live draft output is for operator feedback and offline asset production only. It must not be wired into the local costmap obstacle source.

## Release Rebuild Contract

- `raw_bag_path`: rosbag2 directory containing JT128 point clouds
- `optimized_trajectory_csv`: repository-owned optimized trajectory export from `robot_pgo_mapping`
- `output_root`: floor asset root that receives `nav/`, `localizer/`, `reports/`, and `intermediate/`

The release rebuild path must consume raw scans plus the optimized trajectory. It does not accept a final PCD or PLY as the official source of truth.

## Algorithm Summary

- broad JT128 prefiltering with range, height, azimuth, self-mask, and front-mask
- per-scan classification into `ground`, `ramp`, and `obstacle`
- free-space ray tracing
- occupied endpoint accumulation
- bounded log-odds occupancy update
- post-processing before asset export

## Test Coverage

- package contract and file-layout tests
- topic and asset contract assertions
- static verification that `nav_map` and `localizer_map` are emitted from the same intermediate builder path
