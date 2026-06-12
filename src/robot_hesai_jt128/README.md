# robot_hesai_jt128

JT128 wrapper that keeps vendor networking and frame IDs parameterized.

## Parameters

- `device_ip`, `host_ip`, `lidar_port`, `imu_port`: network config, intended for local car-project reuse
- `network_interface_preferred`: current preferred Jetson NIC is `eth1`
- `network_interface_fallbacks`: recent tests also observed the lidar on `eth0`
- `vendor_points_topic`, `vendor_imu_topic`: vendor raw ingress topics, default `/jt128/vendor/points_raw` and `/jt128/vendor/imu_raw`
- `points_topic`, `imu_topic`: canonical public outputs, default `/lidar_points` and `/lidar_imu`
- `nav_points_topic`: derived localization branch topic, default `/lidar_points_nav`
- `local_perception_points_topic`: optional diagnostic/compatibility branch topic, disabled by default
- `lidar_frame`, `imu_frame`: canonical sensor frames
- `mock_mode`, `bag_mode`: testing paths
- `publish_vendor_tf`: defaults to `false` so the vendor driver cannot pollute the main TF tree
- `local_vendor_driver_config`: reuses `D:/codespace/car/ros2_ws/src/hesai_lidar_ros2/config/config.yaml`
- `local_pointcloud_axis_remap_config`, `local_imu_axis_remap_config`: repository-owned canonical ingress remap configs reused by the Jetson runtime
- `scan_republisher_node`: C++ `/scan_raw -> /scan` restamp/pass-through helper shared by 2D mapping and Isaac flatscan localization. 2D mapping preserves scan acquisition stamps by default to avoid self-spin TF time skew; it does not subscribe to odometry and does not drop scans because field tests showed spin-related scan gating degraded the live map.
- `imu_axis_remap_node`: C++ canonical IMU remap helper. Field runtime enables gyro covariance override because the Hesai driver publishes zero covariance; `/lidar_imu` must be a weak yaw-rate input to EKF, not an exact heading source.
- `pointcloud_axis_remap_node`: C++ canonical point cloud remap helper and the only production publisher for `/lidar_points`. Runtime receives the vendor raw cloud with `input_reliable=false` and depth `1` even though the upstream Hesai publisher offers reliable QoS; a reliable publisher can satisfy a best-effort subscriber, and the remap node must not request reliable delivery that can backpressure the raw ingress path. Production starts this node as a standalone process, publishes the full-density `/lidar_points` trunk first, derives `/lidar_points_nav` at stride 4 with `nav_output_publish_every_n=2` for localization, and uses `NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=local_branch` to inject `/_internal/lidar_points_local` at stride 2 with `local_output_publish_every_n=1` for local obstacle filtering. The file-level YAML default keeps `local_output_topic: ""` as the safe trunk baseline; `NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=trunk` disables the hidden branch and routes `robot_local_perception` back to `/lidar_points`. It publishes `/lidar/axis_remap_status` at 1 Hz for source-side delivery diagnostics, including raw callback inter-arrival, `/lidar_points` publish interval, gap counters over 100/150/200 ms, trunk/branch timing, branch publish/skip/attempt rates, branch subscriber counts, and branch last points/bytes/duration.
- `PointCloudAccelCore`: shared C++ implementation used by standalone `pointcloud_accel_axis_node` and driver-integrated `hesai_accel_driver_node`. The local worker publishes `/perception/obstacle_points` for VoxelLayer marking and publishes `/perception/clearing_points` as bounded virtual clearing rays by default. The virtual clearing rays keep the local costmap from retaining stale near-robot voxels when the live JT128 obstacle cloud has no close returns, without changing pointcloud stamps, QoS, DDS, TF, Nav2 controller settings, or the `/lidar_points` mapping trunk.
- FAST-LIO2 subscribes to the canonical `/lidar_points(lidar_link)` stream directly in the default runtime. The older `pointcloud_fastlio_remap` identity branch is not started by `run_driver.sh`; it remains only as a diagnostic config because the extra full-size pointcloud copy can backpressure the estimator input path.
- `pointcloud_downsample_node`: C++ diagnostic mirror helper. Production runtime does not start it by default; if used, it must run after `/lidar_points` and must not publish another `/lidar_points` trunk.
- `nav_cloud_preprocessor`: upstream `jt128_nav_tools` helper used by localization/mapping scan slicing. The Jetson runtime applies `scripts/jetson/runtime_overlay/patches/jt128_nav_tools_pointcloud_qos.patch`, so its `/lidar_points_nav` input and `/points_nav` output are both best-effort/depth `1`.
- The pointcloud remap includes in-place fast paths for the validated JT128 canonical axis matrices, including the current `x=raw_y, y=-raw_x, z=raw_z` runtime matrix. Production runtime keeps `/lidar_points` as the full-density canonical trunk for mapping FAST-LIO2 and diagnostic consumers; localization scan preprocessing consumes the derived `/lidar_points_nav` branch, and local obstacle filtering consumes the controlled `/_internal/lidar_points_local` branch by default.

## TF Contract

- No static or dynamic TF is published here by default
- All extrinsics remain single-sourced by `robot_description`
- `/lidar_points` and `/lidar_imu` must keep the same numeric semantics that the validated Hesai driver already exposed before the relocalization debugging changes; installation pose adjustments belong in the canonical TF tree, not in a second ingress remap path
