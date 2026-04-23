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

## TF Contract

- No static or dynamic TF is published here by default
- All extrinsics remain single-sourced by `robot_description`
- `/lidar_points` and `/lidar_imu` must keep the same numeric semantics that the validated Hesai driver already exposed before the relocalization debugging changes; installation pose adjustments belong in the canonical TF tree, not in a second ingress remap path
