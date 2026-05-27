# robot_global_localization

Wrapper for pointcloud-to-flatscan and Isaac Occupancy Grid Localizer responsibilities.

## Parameters

- `publish_tf`: defaults to `false`
- `pose_topic`: `/global_localization/pose`
- `health_topic`: `/global_localization/health`
- `default_floor_id`: mock floor asset binding
- `grid_search_trigger_service`: Isaac relocalization trigger service, defaults to `/trigger_grid_search_localization`
- `require_grid_search_trigger`: fail `/global_localization/trigger` when Isaac trigger service is unavailable
- `local_flatscan_config`: reuses the validated `jt128_flatscan.yaml`
- `local_localizer_config`: reuses the validated Isaac occupancy grid localizer parameters from the car repo
- Jetson verification found `isaac_ros_occupancy_grid_localizer` installed under `/home/nvidia/workspaces/isaac_ros-dev/install`
- Jetson verification found `PointCloudToFlatScanNode` inside `isaac_ros_pointcloud_utils`

## TF Contract

- Publishes global pose outputs, not `odom -> base_link`
- Leaves canonical `map -> odom` synthesis to `robot_localization_bridge`

## Runtime Contract

`/global_localization/apply_floor_assets` must be available before `robot_floor_manager` applies a floor. `/global_localization/trigger` proxies the request into Isaac's `/trigger_grid_search_localization`; navigation resume must wait for the wrapper service and the Isaac trigger service before calling floor switch.
