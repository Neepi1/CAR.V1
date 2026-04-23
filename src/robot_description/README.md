# robot_description

Single-source robot body and static sensor extrinsics for the canonical TF tree.

## Parameters

- `base_frame`: defaults to `base_link`
- `lidar_mount_frame`: defaults to `lidar_mount_link`
- `lidar_frame`: defaults to `lidar_link`
- `lidar_level_frame`: defaults to `lidar_level_link`
- `imu_frame`: defaults to `imu_link`
- `base_footprint_frame`: optional static child frame
- `lidar_xyz/rpy`, `lidar_axis_rpy`, `imu_xyz/rpy`: hardware extrinsics that must remain YAML or URDF sourced once
- `lidar_level_link`: derived horizontal slice frame with the same XYZ as the final `lidar_link`, the same final yaw, and zero roll/pitch
- current default lidar mount values are reused from `D:/codespace/car/ros2_ws/src/car_description`

## Rules

- Do not duplicate the static extrinsics in standalone `static_transform_publisher` nodes.
- This package is the only source for `base_link -> lidar_link`, `base_link -> lidar_level_link`, and `base_link -> imu_link`.
