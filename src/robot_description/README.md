# robot_description

Single-source robot body and static sensor extrinsics for the canonical TF tree.

## Parameters

- `base_frame`: defaults to `base_link`
- `lidar_mount_frame`: defaults to `lidar_mount_link`
- `lidar_frame`: defaults to `lidar_link`
- `lidar_level_frame`: defaults to `lidar_level_link`
- `imu_frame`: defaults to `imu_link`
- `gs2_frame`: defaults to `gs2_link`
- `charge_contact_frame`: defaults to `charge_contact_link`
- `base_footprint_frame`: optional static child frame
- `lidar_xyz/rpy`, `lidar_axis_rpy`, `imu_xyz/rpy`: hardware extrinsics that must remain YAML or URDF sourced once; `lidar_axis_rpy` stays zero because `/lidar_points` is canonicalized by `pointcloud_axis_remap`
- `lidar_level_link`: derived horizontal slice frame with the same XYZ as the final `lidar_link`, the same final yaw, and zero roll/pitch
- `gs2_xyz/rpy`: near-field docking lidar mount; current value is front-center, flush with the body front plane, `xyz=[0.36, 0.0, 0.290]`
- `charge_contact_xyz/rpy`: charging contact center; current value is `xyz=[0.398, 0.0, 0.255]`, 3.8 cm ahead of `gs2_link` on the same centerline
- current default lidar mount values are reused from `D:/codespace/car/ros2_ws/src/car_description`

## Rules

- Do not duplicate the static extrinsics in standalone `static_transform_publisher` nodes.
- This package is the only source for `base_link -> lidar_link`, `base_link -> lidar_level_link`, `base_link -> imu_link`, `base_link -> gs2_link`, and `base_link -> charge_contact_link`.
