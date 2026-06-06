# robot_hesai_jt128

JT128 wrapper that keeps vendor networking and frame IDs parameterized.

## Parameters

- `device_ip`, `host_ip`, `lidar_port`, `imu_port`: network config, intended for local car-project reuse
- `network_interface_preferred`: current preferred Jetson NIC is `eth1`
- `network_interface_fallbacks`: recent tests also observed the lidar on `eth0`
- `vendor_points_topic`, `vendor_imu_topic`: vendor raw ingress topics, default `/jt128/vendor/points_raw` and `/jt128/vendor/imu_raw`
- `points_topic`, `imu_topic`: canonical public outputs, default `/lidar_points` and `/lidar_imu`
- `nav_points_topic`: runtime navigation/localization branch topic, default `/lidar_points_nav`
- `local_perception_points_topic`: runtime local obstacle branch topic, default `/_internal/lidar_points_local`
- `lidar_frame`, `imu_frame`: canonical sensor frames
- `mock_mode`, `bag_mode`: testing paths
- `publish_vendor_tf`: defaults to `false` so the vendor driver cannot pollute the main TF tree
- `local_vendor_driver_config`: reuses `D:/codespace/car/ros2_ws/src/hesai_lidar_ros2/config/config.yaml`
- `local_pointcloud_axis_remap_config`, `local_imu_axis_remap_config`: repository-owned canonical ingress remap configs reused by the Jetson runtime
- `scan_republisher_node`: C++ `/scan_raw -> /scan` restamp/pass-through helper shared by 2D mapping and Isaac flatscan localization. 2D mapping preserves scan acquisition stamps by default to avoid self-spin TF time skew; it does not subscribe to odometry and does not drop scans because field tests showed spin-related scan gating degraded the live map.
- `imu_axis_remap_node`: C++ canonical IMU remap helper. Field runtime enables gyro covariance override because the Hesai driver publishes zero covariance; `/lidar_imu` must be a weak yaw-rate input to EKF, not an exact heading source.
- `pointcloud_axis_remap_node`: C++ canonical point cloud remap helper and the only raw pointcloud fan-out owner. Runtime receives the vendor raw cloud with `input_reliable=false` and depth `1` even though the upstream Hesai publisher offers reliable QoS; a reliable publisher can satisfy a best-effort subscriber, and the remap node must not request reliable delivery that can backpressure the raw ingress path. It publishes the full-density `/lidar_points` trunk for FAST-LIO2 plus the public `/lidar_points_nav` localization branch. In production it is loaded with `robot_local_perception` in `pointcloud_perception_pipeline.launch.py`, and the local obstacle branch is the hidden `/_internal/lidar_points_local` intra-process topic instead of a normal public full-size DDS stream. Full-density branch copies use a stride-1 whole-message fast path, and branches with no subscribers are skipped.
- FAST-LIO2 subscribes to the canonical `/lidar_points(lidar_link)` stream directly in the default runtime. The older `pointcloud_fastlio_remap` identity branch is not started by `run_driver.sh`; it remains only as a diagnostic config because the extra full-size pointcloud copy can backpressure the estimator input path.
- `pointcloud_downsample_node`: C++ diagnostic mirror helper. Production runtime does not start it by default because `/lidar_points_nav` and hidden `/_internal/lidar_points_local` are owned by the pointcloud perception pipeline.
- `nav_cloud_preprocessor`: upstream `jt128_nav_tools` helper used by localization/mapping scan slicing. The Jetson runtime applies `scripts/jetson/runtime_overlay/patches/jt128_nav_tools_pointcloud_qos.patch`, so its `/lidar_points_nav` input and `/points_nav` output are both best-effort/depth `1`.
- The pointcloud remap includes in-place fast paths for the validated JT128 canonical axis matrices, including the current `x=raw_y, y=-raw_x, z=raw_z` runtime matrix. Production runtime keeps FAST-LIO2 as the only default full-density `/lidar_points` consumer; navigation/localization preprocessing consumes `/lidar_points_nav`, and local obstacle filtering consumes `/_internal/lidar_points_local` inside the component container.

## TF Contract

- No static or dynamic TF is published here by default
- All extrinsics remain single-sourced by `robot_description`
- `/lidar_points` and `/lidar_imu` must keep the same numeric semantics that the validated Hesai driver already exposed before the relocalization debugging changes; installation pose adjustments belong in the canonical TF tree, not in a second ingress remap path
