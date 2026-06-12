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
- `service_call_timeout_sec`, `result_wait_timeout_sec`, `bridge_accept_timeout_sec`, `map_to_odom_wait_timeout_sec`: staged `/global_localization/trigger` timeouts
- `bridge_status_topic`: defaults to `/localization/bridge_status`
- `bridge_force_accept_service`: defaults to `/robot_localization_bridge/force_accept_next_localization`
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

Phase L1.1 makes the trigger service a staged success gate. It arms `robot_localization_bridge` force-accept for the explicit triggered relocalization, calls Isaac's `std_srvs/Empty` grid-search trigger, waits for `/localization_result` or bridge processing, then requires bridge acceptance, `has_map_to_odom=true`, and a live `map -> odom` owned by `robot_localization_bridge`. A successful response now means the triggered localization result has been reflected into the canonical TF tree. Failure messages include `failure_code=` values for the failed stage.

Phase A2 keeps Isaac as triggered global relocalization only. Runtime continuous localization candidates come from AMCL on `/scan`; no runtime path forwards `/flatscan` into Isaac's trigger input for background updates.

Hardware validation still needs a cold navigation start while recording `/global_localization/health`, `/localization/bridge_status`, `/localization_result`, and `/tf` to confirm the wrapper reaches bridge acceptance without false timeouts. Startup readiness should use bridge acceptance plus `map -> odom`; `/localization/health` is not used as a navigation startup probe, and `/localization_result.header.stamp` being 2-3 seconds older than receive time is allowed in triggered mode.
