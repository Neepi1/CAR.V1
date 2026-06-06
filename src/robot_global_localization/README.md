# robot_global_localization

Wrapper for pointcloud-to-flatscan and Isaac Occupancy Grid Localizer responsibilities.

The runtime wrapper node is the compiled C++ executable `global_localization_node`.
The former Python script is kept only as historical reference during migration and is not installed or launched by the runtime path.

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

The trigger service is intentionally non-blocking. It dispatches Isaac's `std_srvs/Empty` grid-search trigger and returns immediately once the request is handed off. A successful response means "trigger dispatched", not "localization completed". Runtime readiness must be proven by a fresh `/localization_result` or by the resulting `map -> odom` from `robot_localization_bridge`. This avoids a nested service wait in the wrapper callback from blocking navigation startup while Isaac has already accepted the one-shot relocalization.

Hardware validation still needs a cold navigation start while recording `/global_localization/health`, `/localization_result`, and `/tf` to confirm the wrapper does not hold the trigger service call open under Jetson startup load. Startup readiness should use fresh `/localization_result` or `map -> odom`; `/localization/health` is not used as a navigation startup probe.
