# Phase 1.13/1.14 PointCloud Acceleration Profile

## Scope

The pointcloud acceleration profile is a reversible runtime wiring layer. It
does not change FAST-LIO2, Nav2 controller/planner plugins, EKF, App API,
timestamp policy, DDS defaults, or high-rate PointCloud2 QoS defaults.

Default remains:

```bash
export NJRH_POINTCLOUD_ACCEL_PROFILE="${NJRH_POINTCLOUD_ACCEL_PROFILE:-legacy}"
```

Ingress selection is separate:

```bash
export NJRH_POINTCLOUD_INGRESS_PROFILE="${NJRH_POINTCLOUD_INGRESS_PROFILE:-separate_process}"
```

`/lidar_points` is always the full-density/full-fields canonical trunk for
mapping, FAST-LIO2 mapping input, and diagnostics. Navigation profiles must not
compact, downsample, or add a second publisher to `/lidar_points`.

## Owner Contract

| Profile | Trunk owner | Obstacle owner | Scan owner | `/points_nav` role |
| --- | --- | --- | --- | --- |
| `legacy` | `pointcloud_axis_remap_node` / ROS node `pointcloud_axis_remap` | `robot_local_perception` | `scan_republisher` after `nav_cloud_preprocessor -> pointcloud_to_laserscan` | Production scan-chain hop |
| `ipc_worker` | `pointcloud_accel_axis_node` | `pointcloud_accel_axis_node` worker | `pointcloud_accel_axis_node` worker | Not production required |
| `nitros` | Guarded experimental skeleton; must not replace mapping trunk | ROS-compatible owner must be reported | ROS-compatible owner must be reported | Not production required |

### `legacy`

Legacy keeps the validated rollback path:

```text
/jt128/vendor/points_raw
  -> pointcloud_axis_remap_node
  -> /lidar_points

/_internal/lidar_points_local
  -> robot_local_perception
  -> /perception/obstacle_points
  -> /perception/clearing_points

/lidar_points_nav
  -> nav_cloud_preprocessor
  -> /points_nav
  -> pointcloud_to_laserscan
  -> /scan_raw
  -> scan_republisher
  -> /scan
  -> laser_scan_to_flatscan
  -> /flatscan
```

`run_pointcloud_accel_pipeline.sh` restores the minimal legacy scan chain through
`scripts/jetson/runtime_overlay/launch/jt128_localization_sensing.launch.py`.
That launch does not start the full localizer or
`occupancy_grid_localizer_container`.

The legacy scan launch supports these overrides:

- `NJRH_LEGACY_SCAN_PREPROCESSOR_PARAMS`
- `NJRH_LEGACY_SCAN_PARAMS`
- `NJRH_LEGACY_SCAN_FLATSCAN_PARAMS`
- `NJRH_LEGACY_SCAN_POINTS_TOPIC`, default `/lidar_points_nav`
- `NJRH_LEGACY_SCAN_NAV_POINTS_TOPIC`, default `/points_nav`
- `NJRH_LEGACY_SCAN_TOPIC`, default `/scan`
- `NJRH_LEGACY_SCAN_FLATSCAN_TOPIC`, default `/flatscan`

### `ipc_worker`

`ipc_worker` starts `pointcloud_accel_axis_node` as the only `/lidar_points`
publisher. The raw callback rotates `/jt128/vendor/points_raw`, publishes the
full trunk first, updates a latest normalized buffer, and returns. Same-process
workers then publish:

- `/perception/obstacle_points`
- `/perception/clearing_points`
- `/scan`

`laser_scan_to_flatscan` converts `/scan` to `/flatscan` for compatibility.
`/_internal/lidar_points_local`, `/lidar_points_nav`, and `/points_nav` are
debug/compat outputs only in this profile, not production-required DDS hops.
Profile restart stops legacy production obstacle and scan-chain owners unless an
explicit fallback environment variable is set for diagnosis.

Phase Z1 keeps this ROS graph unchanged but removes the worker-side full-cloud
reparse path. The accel node now stores the latest normalized points internally
as `LatestNormalizedBuffer` / `NormalizedPointView`; local and scan workers read
that buffer directly and only build the final existing ROS outputs
(`/perception/obstacle_points`, `/perception/clearing_points`, and `/scan`).
`/lidar_points` remains full-density/full-fields for mapping and diagnostics.
This is an internal copy/allocation cleanup, not RMW loaned messages and not a
new PointCloud2 topic.

### Ingress Profiles

`/jt128/vendor/points_raw` is the Hesai driver decoded ROS
`sensor_msgs/msg/PointCloud2` output. It is not the sensor UDP packet stream.

`separate_process` is the default and current production path:

```text
hesai_ros_driver_node
  -> /jt128/vendor/points_raw
  -> pointcloud_accel_axis_node
  -> PointCloudAccelCore
  -> /lidar_points + /perception/obstacle_points + /perception/clearing_points + /scan
```

`driver_integrated` builds the repo-owned
`src/third_party/hesai_lidar_ros2_overlay` source and starts
`hesai_accel_driver_node`. Hesai decode constructs one `PointCloud2` inside the
driver process, then moves it directly into `PointCloudAccelCore`; this removes
the production `/jt128/vendor/points_raw` DDS hop. `/jt128/vendor/points_raw`
may stay as an optional debug/compat topic, but it must not be the production
input to AccelCore. Typed decoded-frame input remains the next optimization
step.

In both `ipc_worker` and `driver_integrated`, `PointCloudAccelCore` owns the
accel local worker outputs. `/perception/obstacle_points` remains the VoxelLayer
marking source, and `/perception/clearing_points` is generated as bounded
virtual clearing rays by default. The clearing rays are in `base_link`, keep the
source cloud stamp, and are only a local costmap clearing input; they do not
change `/lidar_points`, TF ownership, DDS/QoS, Nav2 planner/controller settings,
or FAST-LIO2 mapping inputs.

Switch and rollback:

```bash
export NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker
export NJRH_POINTCLOUD_INGRESS_PROFILE=separate_process
bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile ipc_worker --ingress-profile separate_process --restart
```

Only use `driver_integrated` for explicit overlay tests until 5-minute and
20-minute navigation plus mapping validation pass.

### `nitros`

`nitros` remains a guarded experimental skeleton. `check_isaac_ros_nitros_env.sh`
must pass before the profile is written or started. NITROS must not replace
`/lidar_points`, must not become the mapping trunk, and must keep ROS-compatible
owner reporting for `/perception/*`, `/scan`, and `/flatscan`.

## Runtime Commands

Check NITROS environment:

```bash
bash scripts/jetson/runtime_overlay/scripts/check_isaac_ros_nitros_env.sh
```

Legacy baseline:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile legacy --restart
sleep 20
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
ros2 topic info -v /lidar_points
ros2 topic info -v /perception/obstacle_points
ros2 topic info -v /perception/clearing_points
ros2 topic info -v /points_nav
ros2 topic info -v /scan
ros2 topic info -v /flatscan
```

`ipc_worker`:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile ipc_worker --ingress-profile separate_process --restart
sleep 20
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
ros2 topic info -v /jt128/vendor/points_raw
ros2 topic info -v /lidar_points
ros2 topic info -v /perception/obstacle_points
ros2 topic info -v /perception/clearing_points
ros2 topic info -v /scan
ros2 topic info -v /flatscan
timeout 12 ros2 topic echo /lidar/axis_remap_status --field data
timeout 12 ros2 topic echo /lidar/pointcloud_accel_status --field data
```

A/B:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_ab.sh --profile legacy --ingress-profile separate_process --duration-sec 120 --apply --restart
bash scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_ab.sh --profile ipc_worker --ingress-profile separate_process --duration-sec 120 --apply --restart
```

Rollback:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile legacy --restart
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
```

## Verification

`verify_pointcloud_accel_profile.sh` reports:

- requested and resolved profile
- requested/resolved ingress profile and driver owner
- trunk, obstacle, clearing, points_nav, scan, and flatscan owners
- publisher/subscriber counts for `/jt128/vendor/points_raw`, `/lidar_points`, `/points_nav`, `/scan`, and `/flatscan`
- `/lidar/axis_remap_status`, `/lidar/pointcloud_accel_status`, and legacy `/perception/local_perception_status`
- accel status fields `accel_ingress_profile`, `input_path`,
  `vendor_raw_ros_hop_required`, `driver_integrated_process`,
  `accel_core_process_pointcloud2_count`, and
  `accel_core_process_decoded_view_count`
- internal zero-copy status fields, including latest internal buffer points,
  worker full-cloud-copy counters, intermediate PointCloud2 build counters,
  allocation counters, lock wait maxima, and worker processing averages
- FAST-LIO2 residual state
- Nav2 controller lifecycle state
- final summary flags: `PROFILE_OWNER_CONTRACT_OK`, `LEGACY_SCAN_CHAIN_OK`, `IPC_WORKER_OWNER_OK`, `TRUNK_FULL_DENSITY_OK`, and `NAV2_COMPAT_TOPICS_OK`

`run_pointcloud_accel_ab.sh` writes `reports/pointcloud_accel_ab_<timestamp>.md`
with requested profile, actual profile, ingress profile, actual driver process,
raw-topic pub/sub counts, topic owners, topic counts, status samples,
obstacle/scan/flatscan Hz, read-only socket/drop snapshots, CPU0/4/5/6/7
snapshot, FAST-LIO2 residual, Nav2 lifecycle, and PASS/WARN/FAIL.

## Acceptance

Legacy PASS:

- `/lidar_points` publisher count is `1`.
- `/lidar_points` owner is `pointcloud_axis_remap_node` or ROS node `pointcloud_axis_remap`.
- `/perception/obstacle_points` and `/perception/clearing_points` owner is `robot_local_perception`.
- `/points_nav` owner is `nav_cloud_preprocessor`.
- `/scan` owner is `scan_republisher`.
- `/flatscan` owner is `laser_scan_to_flatscan`.
- `/lidar/axis_remap_status` and `/perception/local_perception_status` exist.
- FAST-LIO2 is not resident during normal navigation.

`ipc_worker` PASS:

- `/lidar_points` publisher count is `1`.
- `/lidar_points` owner is `pointcloud_accel_axis_node`.
- `/lidar_points` remains full-density/full-fields and is not compact/downsampled.
- `/lidar/pointcloud_accel_status` exists.
- `internal_zero_copy_profile=true`.
- `latest_internal_buffer_points` is nonzero after the driver is publishing.
- `local_worker_full_cloud_copy_count=0` and `scan_worker_full_cloud_copy_count=0`.
- `local_worker_intermediate_pointcloud_build_count=0` and `scan_worker_intermediate_pointcloud_build_count=0`.
- `local_worker_enabled=true` and `scan_worker_enabled=true`.
- `/perception/obstacle_points`, `/perception/clearing_points`, and `/scan` owner is `pointcloud_accel_axis_node` or an explicitly documented accel worker.
- `/points_nav` is not a production hop.
- `robot_local_perception` and the legacy scan chain are not production owners unless an explicit fallback env is set.
- FAST-LIO2 is not resident during normal navigation.
