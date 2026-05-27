# robot_hesai_jt128

JT128 wrapper that keeps vendor networking and frame IDs parameterized.

## Parameters

- `device_ip`, `host_ip`, `lidar_port`, `imu_port`: network config, intended for local car-project reuse
- `network_interface_preferred`: current preferred Jetson NIC is `eth1`
- `network_interface_fallbacks`: recent tests also observed the lidar on `eth0`
- `vendor_points_topic`, `vendor_imu_topic`: vendor raw ingress topics, default `/jt128/vendor/points_raw` and `/jt128/vendor/imu_raw`
- `points_topic`, `imu_topic`: canonical public outputs, default `/lidar_points` and `/lidar_imu`
- `lidar_frame`, `imu_frame`: canonical sensor frames
- `mock_mode`, `bag_mode`: testing paths
- `publish_vendor_tf`: defaults to `false` so the vendor driver cannot pollute the main TF tree
- `local_vendor_driver_config`: reuses `D:/codespace/car/ros2_ws/src/hesai_lidar_ros2/config/config.yaml`
- `local_pointcloud_axis_remap_config`, `local_imu_axis_remap_config`: repository-owned canonical ingress remap configs reused by the Jetson runtime
- `scan_republisher_node`: C++ `/scan_raw -> /scan` restamp/pass-through helper shared by 2D mapping and Isaac flatscan localization
- `imu_axis_remap_node`: C++ canonical IMU remap helper. Field runtime enables gyro covariance override because the Hesai driver publishes zero covariance; `/lidar_imu` must be a weak yaw-rate input to EKF, not an exact heading source.
- `pointcloud_axis_remap_node`: C++ canonical point cloud remap helper. Runtime publishes `/lidar_points` with `best_effort` reliability and depth `1` by default because each JT128 cloud is large; reliable DDS delivery can backpressure the remap node and drop the effective rate below the 20 Hz lidar source. Enable `output_reliable` only for short debugging sessions that explicitly require reliable transport.
- The pointcloud remap includes in-place fast paths for the validated JT128 canonical axis matrices, including the current `x=-raw_y, y=-raw_x, z=raw_z` runtime matrix. This keeps the public `/lidar_points` stream close to the vendor source rate and prevents the FAST-LIO2 deskew chain from being starved by remap overhead.

## TF Contract

- No static or dynamic TF is published here by default
- All extrinsics remain single-sourced by `robot_description`
- `/lidar_points` and `/lidar_imu` must keep the same numeric semantics that the validated Hesai driver already exposed before the relocalization debugging changes; installation pose adjustments belong in the canonical TF tree, not in a second ingress remap path
