# robot_global_localization

Wrapper for pointcloud-to-flatscan and Isaac Occupancy Grid Localizer responsibilities.

## Parameters

- `publish_tf`: defaults to `false`
- `pose_topic`: `/global_localization/pose`
- `health_topic`: `/global_localization/health`
- `default_floor_id`: mock floor asset binding
- `local_flatscan_config`: reuses the validated `jt128_flatscan.yaml`
- `local_localizer_config`: reuses the validated Isaac occupancy grid localizer parameters from the car repo
- Jetson verification found `isaac_ros_occupancy_grid_localizer` installed under `/home/nvidia/workspaces/isaac_ros-dev/install`
- Jetson verification found `PointCloudToFlatScanNode` inside `isaac_ros_pointcloud_utils`

## TF Contract

- Publishes global pose outputs, not `odom -> base_link`
- Leaves canonical `map -> odom` synthesis to `robot_localization_bridge`
