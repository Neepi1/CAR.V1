# Runtime Service Lifecycle

The runtime is split into common services and mode services.

The production target architecture is defined in
[`commercial_runtime_architecture.md`](commercial_runtime_architecture.md). That
document is the ownership contract: process lifetime, lifecycle state, and
mission task state are separate. App requests submit intent; they do not own
long-lived process startup or shutdown.

Common services should stay up during daily operation:

- JT128 driver plus canonical pointcloud/IMU remap
- Ranger chassis driver
- `robot_description` static TF publisher
- `robot_eai_gs2` GS2 near-field docking lidar driver (`/dock/gs2_scan`, `/dock/gs2_points`)
- `robot_local_state` canonical local odom owner (`LOCAL_STATE_MODE=ekf` by default: `/wheel/odom` + `/lidar_imu` -> EKF -> `/local_state/odometry`; FAST-LIO local-state remains an explicit diagnostic/mapping-aligned mode)
- `robot_local_perception`
- `robot_safety`
- `ranger_mini3_mode_controller`
- `robot_floor_manager`
- `robot_api_server`

`robot_api_server` is supervised inside the common-service layer. If the API process exits, `run_robot_api_server_supervised.sh` restarts it after a short delay. Before each restart, the supervisor clears stale orphan API processes so only one `robot_api_server_node` can own port `8080` and the fixed HTTP worker pool. `njrh_container.sh start-runtime` and `start-common` now also require `GET /api/v1/status` on port `8080` to become healthy before reporting common services as ready. That host-side HTTP wait defaults to `NJRH_ROBOT_API_READY_TIMEOUT_SEC=120` with `NJRH_ROBOT_API_READY_POLL_SEC=1`; it does not create ROS readiness participants. The API process uses a fixed HTTP worker pool controlled by `max_http_connections` and returns `503` when overloaded instead of creating unbounded detached request threads.

The API server treats `/tf` as a process-level localization input, not as page-scoped telemetry. It creates the `/tf` subscription once at startup and keeps it resident so high-rate `/api/v1/robot/pose` polling cannot repeatedly add and remove reliable Fast DDS endpoints on `/tf`; that endpoint churn can backpressure `robot_localization_bridge` while the bridge process still appears alive.

The API server's BMS charging-contact policy lives in `robot_api_server/bms_contact.hpp` and `src/bms_contact.cpp`, separate from HTTP routing. UTC timestamp formatting and generated map/current-pose IDs live in `robot_api_server/api_time_utils.hpp` and `src/api_time_utils.cpp`. Docking job state formatting lives in `robot_api_server/docking_job_model.hpp` and `src/docking_job_model.cpp`; docking/undocking status string classification lives in `robot_api_server/docking_status_utils.hpp` and `src/docking_status_utils.cpp`. Common text/binary file reads and writes, PGM output, and map YAML image-file rewrites live in `robot_api_server/file_utils.hpp` and `src/file_utils.cpp`. Floor asset completeness checks, active `current/` selection, `poses.yaml` fallback, and stored pose lookup live in `robot_api_server/floor_asset_resolver.hpp` and `src/floor_asset_resolver.cpp`. HTTP request/response structs, WebSocket accept-key helpers, and lightweight JSON parsing live in `robot_api_server/http_common.hpp` and `src/http_common.cpp`; localization result snapshots and relocalization diagnostic text live in `robot_api_server/localization_result_model.hpp` and `src/localization_result_model.cpp`. Map/pose models plus safe ID/name validation live in `robot_api_server/storage_models.hpp` and `src/storage_models.cpp`; grayscale PNG encoding, PGM dimension reads, and Nav map YAML metadata extraction live in `robot_api_server/map_asset_io.hpp` and `src/map_asset_io.cpp`; OccupancyGrid-to-image conversion, saved map YAML text, neutral costmap filter assets, and asset reports live in `robot_api_server/map_asset_writer.hpp` and `src/map_asset_writer.cpp`; released map directory paths, manifest traversal, and active/name/id lookup live in `robot_api_server/map_catalog.hpp` and `src/map_catalog.cpp`; `MapManifest` path derivation and `manifest.json` read/write formatting live in `robot_api_server/map_manifest_io.hpp` and `src/map_manifest_io.cpp`; navigation-cancel job state formatting lives in `robot_api_server/navigation_cancel_job_model.hpp` and `src/navigation_cancel_job_model.cpp`; `poses.yaml` parsing/writing lives in `robot_api_server/poses_io.hpp` and `src/poses_io.cpp`; runtime map context file formatting lives in `robot_api_server/runtime_map_context_io.hpp` and `src/runtime_map_context_io.cpp`; saved 2D PNG lookup and runtime flat-map companion file paths live in `robot_api_server/runtime_map_lookup.hpp` and `src/runtime_map_lookup.cpp`; robot pose snapshots and `/api/v1/robot/pose` response payloads live in `robot_api_server/robot_pose_model.hpp` and `src/robot_pose_model.cpp`; Linux child-process setup and pid/pgid helpers live in `robot_api_server/runtime_process_utils.hpp` and `src/runtime_process_utils.cpp`; keepout semantic JSON paths and response fragments live in `robot_api_server/semantic_layer_io.hpp` and `src/semantic_layer_io.cpp`; App subscription request parsing lives in `robot_api_server/subscription_api.hpp` and `src/subscription_api.cpp`; App page-scoped subscription leases and TTL expiry live in `robot_api_server/subscription_manager.hpp` and `src/subscription_manager.cpp`; frame ID normalization, yaw extraction, angle wrapping, and ROS timestamp helpers live in `robot_api_server/tf_pose_utils.hpp` and `src/tf_pose_utils.cpp`. These pure modules keep full-SOC dock-contact inference, timestamp/ID generation, docking job state formatting, docking status classification, file asset IO, floor asset resolution, gateway parsing, localization result diagnostics, map asset validation, saved-map asset generation, manifest catalog lookup, navigation-cancel state formatting, saved PNG lookup, robot pose payload formatting, semantic layer formatting, TF pose math, runtime process helpers, and subscription lease behavior testable without touching navigation, docking, or socket-loop code.

The Web dashboard is not part of the production runtime. It is only a manual observation/debug window.

Jetson CPU affinity is now a runtime policy, not a launch-file assumption. The default policy lives in `scripts/jetson/runtime_overlay/config/cpu_affinity.env` and can be disabled with:

```bash
NJRH_CPU_AFFINITY_ENABLED=false
```

The default 8-core split reserves CPU0 for lightweight API/map-filter work plus bursty Nav2 planner/BT work, CPU0/CPU1 for Nav2 lifecycle supervision, CPU1 for base/safety command arbitration, CPU2 for EKF local state, CPU3 for Nav2 controller/collision work, CPU4 for the JT128 UDP driver, CPU5 for full-density pointcloud remap ingress, CPU6 for IMU remap plus localization and local perception, and CPU7 for the `map -> odom` bridge plus mapping backend work. FAST-LIO2 is not resident during normal navigation; live 2D mapping re-derives its mode-local CPU sets at launch so FAST-LIO2 frontend/deskew defaults to CPU7 only while mapping is active and PGO/slam_toolbox mapping defaults to CPU7. During live 2D mapping, `run_projected_map.sh` also applies a temporary LiDAR NIC RPS/XPS profile (`NJRH_SLAM2D_LIDAR_RPS_XPS_INTERFACE=eth1`, `NJRH_SLAM2D_LIDAR_RPS_XPS_CPUSET=5` by default) and restores the previous queue masks on exit. `robot_api_server` also restores the same `mapping_lidar_rps_xps_state_dir` state on mapping stop/save, because API process-group termination can bypass the shell EXIT trap. It does not write IRQ affinity; the current Jetson eth1 IRQ rejects affinity writes, while RPS/XPS is sufficient to recover `/lidar_points` to the JT128 target cadence in mapping mode. Future arm control/planning should use CPU6/CPU7 only when mapping is not active. Existing processes can be retagged without restarting motion by running the helper below. It applies affinity to every Linux thread under `/proc/<pid>/task`, not only the process leader, because ROS 2 executors and DDS workers are multithreaded. The helper does not source `common_env.sh` and does not initialize ROS/DDS, so it cannot create extra participants or rewrite DDS profiles during a live navigation run:

```bash
bash scripts/jetson/runtime_overlay/scripts/apply_cpu_affinity.sh
```

Hardware validation still needs a loaded navigation run after a full restart to confirm `taskset -pc <pid>` matches the intended service groups: JT128 driver on CPU4, standalone pointcloud remap on CPU5, IMU remap/localization/local perception on CPU6, `robot_localization_bridge` on CPU7, planner/BT on CPU0, and both Nav2 lifecycle managers on CPU0/CPU1. Also confirm no `fastlio_mapping` process is resident during navigation, `/jt128/vendor/points_raw`, `/lidar_points`, `/_internal/lidar_points_local`, `/points_nav`, and `/perception/obstacle_points` remain close to the target cadence, `/perception/clearing_points` runs at the configured decimated cadence, the filter lifecycle manager reaches active before the core navigation lifecycle manager, `controller_server` no longer misses 12 Hz under JT128 load, and `/perception/obstacle_points` is still subscribed by local costmap plus collision monitor. Use `set_local_perception_input_profile.sh` for explicit `local_branch`/`trunk` switching, `verify_pointcloud_rates.sh` for sequential field rate checks, `verify_lidar_trunk_jitter.sh` for source-side trunk jitter checks, `diagnose_lidar_points_jitter.sh` for `/lidar_points` publish-side versus CLI subscriber-side classification, `diagnose_local_perception_pipeline.sh` for local obstacle CASE classification, `diagnose_nav_scan_pipeline.sh` for `/points_nav` and scan-chain CASE classification, `diagnose_pointcloud_cpu_pressure.sh` for CPU/thermal/fan-out pressure, `run_pointcloud_cpu_affinity_ab.sh --print` for reversible CPU A/B plans, `check_runtime_process_freshness.sh` after sync/build/restart, `inspect_pointcloud_subscribers.sh` for ROS graph fan-out checks without a PointCloud2 subscription, `verify_pointcloud_delivery_matrix.sh` for profile acceptance, `inspect_pointcloud_cpu_affinity.sh` for observation-only CPU/TID/thermal placement, `record_pointcloud_nav_acceptance.sh --duration-sec 1200` for the 20-minute lightweight report, `run_pointcloud_dds_transport_ab.sh` for controlled DDS transport A/B commands, and `run_lidar_trunk_pure_ab.sh --execute` only as a stationary diagnostic that temporarily disables derived branches and then restores production driver settings; they are manual verification tools, not background monitors.

Phase 1.12 extends the manual CPU diagnosis into IRQ/softirq placement without
making IRQ policy a default runtime setting. Use
`collect_cpu_irq_softirq_snapshot.sh --duration-sec 20` for a read-only report
covering tegrastats averages, per-thread CPU placement, `ksoftirqd`, NET_RX,
`/proc/interrupts`, `/proc/softirqs`, IRQ affinity, and RPS/XPS masks. Use
`identify_lidar_network_irq.sh` to infer the LiDAR NIC from the active Hesai
config and to check whether that NIC is also the SSH/default-route interface.
`run_cpu_core_allocation_ab.sh` writes only the temporary
`config/cpu_affinity_runtime_override.env` and can retag live threads with the
existing `taskset` helper; `run_lidar_irq_affinity_ab.sh` writes IRQ/RPS/XPS
only after explicit `--apply`, backs up the previous values, and restores with
`--restore`. The combined `run_pointcloud_cpu_irq_experiment.sh` collects a
baseline, applies the selected CPU/IRQ profile, collects the profile result, and
restores automatically unless `--keep-applied` is explicit. Start with
`--irq-profile irq_keep_default` so CPU migration is isolated before touching
network IRQs. These tools must not be used as justification to change PointCloud2
QoS reliability, DDS transport, timestamps, Nav2 controller/planner settings,
EKF, FAST-LIO2 navigation residency, or the App/mapping ownership boundary.

The full-size pointcloud trunk is latest-only after the upstream Hesai driver: `/lidar_points` is the high-density canonical trunk used by mapping-owned FAST-LIO2 and diagnostic consumers, not the default local-obstacle input during production navigation. Production `pointcloud_axis_remap` runs as a standalone process, publishes the only `/lidar_points` trunk, derives `/lidar_points_nav` at stride 4 with `nav_output_publish_every_n=2` for approximately 10 Hz localization scan preprocessing, and derives `/_internal/lidar_points_local` at stride 2 with `local_output_publish_every_n=1` when `NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=local_branch`. That profile is the production default and routes the separate `robot_local_perception` process to `/_internal/lidar_points_local`; `NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=trunk` disables the hidden branch and routes local perception back to `/lidar_points` for rollback/diagnosis. `ROBOT_LOCAL_PERCEPTION_INPUT_TOPIC` overrides the profile input topic, and explicit `NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_TOPIC`, `NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_STRIDE`, and `NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N` values override the runtime branch injection. `/lidar_points`, `/lidar_points_nav`, `/_internal/lidar_points_local`, `/points_nav`, `/perception/obstacle_points`, and `/perception/clearing_points` use best-effort QoS with depth `1`. Derived branches must not become additional canonical `/lidar_points` publishers. `/points_nav` depends on the upstream `jt128_nav_tools/nav_cloud_preprocessor` patch at `scripts/jetson/runtime_overlay/patches/jt128_nav_tools_pointcloud_qos.patch`; the patched build is sourced from `${NJRH_JT128_NAV_TOOLS_PATCHED_OVERLAY:-${PROJECT_ROOT}/.runtime/jt128_nav_tools_overlay/install}` by `common_env.sh`. Without that overlay the preprocessor falls back to its old mixed QoS behavior and can reintroduce large-message queues. Debug commands, RViz/Foxglove views, and rosbag captures should not stay attached to `/lidar_points`; use status topics or reduced/debug branches where possible, and require explicit env flags for bagging high-density clouds.

`robot_local_perception` is pinned to the localization/local perception CPU set instead of sharing the Nav2 control CPU set or the raw remap ingress CPU. The local costmap, MPPI controller, collision monitor, and localization bridge need the control/TF cores to service TF message filters and command callbacks with low latency; obstacle filtering is a lidar-derived perception workload and should not compete with those callbacks or with full-size raw remap copies.

The local perception hot path keeps `/perception/obstacle_points` ahead of clearing work: each frame fuses input rotation and TF into one 3x4 transform, filters directly from the PointCloud2 byte buffer, writes obstacle output directly as PointCloud2, and then hands the newest clearing job to a latest-only worker when the configured clearing cadence fires. Voxel speckle suppression uses packed keys in an unordered map rather than a tree map of tuple keys, so NORMAL-mode filtering does not add avoidable per-point allocation/comparison overhead. The clearing worker overwrites stale pending jobs instead of building a backlog, so `/perception/clearing_points` can fall behind briefly without delaying obstacle marking.

For field diagnosis, the same C++ node publishes `/perception/local_perception_status` every two seconds with received, accepted, process-timer, obstacle, clearing, no-new, empty-input, empty-obstacle, TF/stamp/publish-gating, active profile, filter bounds, point stride, clearing cadence, and processing-time counters. Use it to tell whether low `ros2 topic hz` readings come from the local perception loop itself, filter/gating emptiness, or CLI/DDS subscriber pressure; do not add Python runtime probes for this path.

Pointcloud source and localization-preprocessor diagnostics are also emitted from C++ nodes. `pointcloud_axis_remap` publishes `/lidar/axis_remap_status` at 1 Hz with raw input rate, `/lidar_points` publish rate, raw callback inter-arrival, `/lidar_points` publish interval and gap counters, raw callback duration, trunk/branch/total publish timing, cloud size, output subscriber count, branch flags, branch publish/skip/attempt rates, branch subscriber counts, branch last points/bytes/duration, and skipped-cloud count. The patched `nav_cloud_preprocessor` publishes `/lidar/nav_cloud_preprocessor_status` at 1 Hz with input callback/accept rate, `/points_nav` output rate, TF/empty/filter-empty skips, input inter-arrival, input stamp/message age, processing average/max, input/output point counts, lookup timeout, source/target frame, range/height filters, matched publisher/subscriber counts, and QoS. These status topics are the preferred field inputs for `diagnose_lidar_points_jitter.sh`, `diagnose_local_perception_pipeline.sh`, `diagnose_nav_scan_pipeline.sh`, `verify_lidar_trunk_jitter.sh`, and `verify_pointcloud_delivery_matrix.sh`; use direct `ros2 topic hz` on full-density clouds only when a status topic is unavailable or when deliberately comparing subscriber-side delivery with `--include-cli-hz` or a manual `NJRH_VERIFY_MATRIX_LIDAR_POINTS_CLI_HZ` value for CASE_G.

The default navigation local-state mode is wheel-odom + JT128-IMU EKF because Nav2 control needs low-latency, continuous `odom -> base_link` more than globally consistent lidar-inertial smoothing. When FAST-LIO local-state mode is explicitly selected, `/Odometry` can arrive behind wall time because the lidar-inertial frontend has processing latency; that latency must be diagnosed at the producer and transport layers, not hidden by changing local-obstacle cloud stamps. `robot_local_perception` publishes `/perception/obstacle_points` and `/perception/clearing_points` with the input pointcloud acquisition stamp preserved. The local costmap runs in `base_link`, so its obstacle layer consumes robot-frame clouds directly instead of depending on a historical `odom <- base_link` transform for every local obstacle update. The local perception hot path uses latest/static TF for the `lidar_link -> base_link` sensor extrinsic (`input_transform_use_latest=true`) while preserving the original cloud header stamp, so static extrinsic lookup cannot block every obstacle frame. Startup-time costmap MessageFilter drops are kept as diagnostics, not as shell-level startup blockers.

Canonical local-state is owned by common services, not by Nav2 startup. In the default EKF mode, common services start the wheel-odom preprocessor, IMU bias filter, and `robot_localization` EKF process, and process existence is the startup ownership check. In explicit FAST-LIO local-state mode the common layer requires both `fastlio_odom_bridge_node` and `robot_local_state/local_state_node`. Endpoint/topic/TF validation is manual diagnostics and API admission, not a shell launch gate. Runtime graph-probe misses under Nav2 startup load must not cause the canonical `odom -> base_link` owner to kill itself.

FAST-LIO2 itself is no longer a default navigation odom dependency and is no longer resident in normal navigation. `run_common_services.sh` defaults `NJRH_FASTLIO_AUTOSTART=false`; `run_projected_map.sh` starts a mapping-owned FAST-LIO2 frontend only while live mapping is active. FAST-LIO2 becomes a local-state dependency only when `NJRH_NAV_LOCAL_STATE_MODE=fastlio` is explicitly selected, and that mode must either enable `NJRH_FASTLIO_AUTOSTART=true` or attach to an already managed FAST-LIO runtime.

Nav2 preflight is process-first. `run_nav2_navigation.sh` starts or reuses floor-manager, `robot_safety`, Ranger mode-controller, and local perception by process ownership, then launches Nav2. It does not run local-state, TF, `/safety/status`, obstacle-cloud, or local-costmap probes as startup gates, and it does not kill/restart local perception just to warm a new costmap buffer.

The occupancy localization mode may start or reuse localization-specific services such as `robot_localization_bridge` and `robot_global_localization`, but it does not own FAST-LIO2 or canonical local-state. It consumes `/lidar_points` for stationary Isaac relocalization and checks only that the resident local-state process for the selected mode exists before launching the localization stack; FAST-LIO2 is required only for explicit `NJRH_NAV_LOCAL_STATE_MODE=fastlio`. On localization or Nav2 startup failure the mode script cleans the localization stack and overlay helpers only; `robot_local_state` remains a common/canonical service. This prevents Nav2 from staying active while `/local_state/odometry` and `/tf` publishers disappear during a stop/resume race.

Navigation resume gives the localization layer a short settle window (`NJRH_NAV_LOCALIZATION_START_SETTLE_SEC`, default 3 seconds), then runs a bounded deterministic startup chain before Nav2 is allowed to become ready. The chain waits for `/global_localization/trigger`, Isaac `/trigger_grid_search_localization`, `/map`, an observed `/flatscan` message, and a `/localization_result` publisher, sends one `/global_localization/trigger`, then requires a fresh `/localization_result` and live `map -> odom`. `/flatscan` is `isaac_ros_pointcloud_interfaces/msg/FlatScan`, not `sensor_msgs/msg/LaserScan`; `/scan` is the LaserScan intermediate. The FlatScan gate checks message flow rather than header freshness because the generic startup freshness tool intentionally supports only standard stamped ROS messages. This is the required localization sequence, not a high-frequency Python/rclpy graph watchdog.

Standard Nav2 startup uses two lifecycle managers. `lifecycle_manager_costmap_filters` owns only the keepout/speed mask map servers and filter-info servers. `lifecycle_manager_navigation` owns the core controller, smoother, planner, behavior tree, waypoint follower, velocity smoother, and collision monitor nodes, and starts after `NJRH_NAV_LIFECYCLE_START_DELAY_SEC` seconds so the filter services and map-mask files are already stable. This avoids aborting the whole Nav2 bringup because a filter-mask load or planner/global-costmap configuration burst delays a lifecycle service response.

The occupancy localization startup no longer waits on `/lidar_points` from shell. It launches the localization stack and leaves pointcloud freshness to explicit diagnostics, Isaac/localizer logs, and API goal admission. This prevents a manual navigation stop or short common-service recovery window from causing the resident navigation runtime to exit before Isaac localization and Nav2 are even launched.

Mode services are allowed to start and stop when switching between navigation and mapping:

- Navigation: Isaac localization stack, `robot_localization_bridge`, Nav2, velocity smoother, collision monitor.
- Mapping: mapping-owned FAST-LIO2, optional PGO, slam_toolbox 2D mapping, scan slicing helpers.
- Docking: `robot_docking_manager`, started for near-field charging alignment after Nav2 reaches the pre-dock pose, and reused for controlled `/docking/undock` so reverse motion stays behind `robot_safety`.
- Current field-default mapping: `run_projected_map.sh` starts FAST-LIO2 only for the active live mapping session, then feeds `slam_toolbox` 2D mapping. PGO remains optional formal mapping work.

The field runtime now has a selected-floor resident navigation entrypoint. It is
still script-supervised rather than a ROS 2 managed bringup package, but the App
no longer needs to own separate localization/Nav2 session scripts. Use the
read-only readiness check:

```bash
bash scripts/jetson/runtime_overlay/scripts/check_commercial_runtime_ready.sh
```

For a selected floor, use the resident navigation layer:

```bash
NJRH_BUILDING_ID=B1 NJRH_FLOOR_ID=F1 \
  bash scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh
```

Boot autostart defaults to `NJRH_RESIDENT_NAVIGATION_AUTOSTART=auto`. In this
mode `run_common_services.sh` reads
`maps_release/last_navigation_map.json`, verifies that it still matches the
selected `current/manifest.json` and required fixed assets, then resumes
navigation on that last manually selected map. If the file is missing or no
longer matches `current/`, common services stay alive in `NO_MAP` mode and Nav2
is not started. `NJRH_RESIDENT_NAVIGATION_AUTOSTART=false` disables this boot
resume path; `true` with an explicit `NJRH_FLOOR_ID` remains a diagnostic
override.

Navigation resume scripts do not restore broad ROS graph/topic/TF probe loops, but they do keep the critical localization startup chain as a gate. `run_navigation_runtime_services.sh` starts the selected-floor localization layer, verifies the initial localizer inputs/services, sends one bounded global-localization trigger request, waits for fresh `/localization_result` and `map -> odom`, starts Nav2, then marks the runtime context ready only after Nav2 lifecycle activation and the global costmap are available. Explicit readiness tools still exist for field diagnostics, but a transient Fast DDS discovery miss outside this critical chain must not keep the App in `starting`. The API server also polls the navigation resume child process while serving `/api/v1/status` and `/api/v1/navigation/state`; if the child process exits during startup or the runtime context records `state=failed`, the API reports navigation `failed` with the resume log path instead of leaving the App in `starting`.

The occupancy localization bridge watchdog follows the same process-first rule. It still rejects a real `robot_localization_bridge` process loss, but ROS graph probe misses during Nav2/map-server activation are diagnostics only while the bridge process is still alive.

Local perception stays resident across Nav2 restarts. `run_nav2_navigation.sh` no longer kills local perception, primes a probe-owned TF buffer, or blocks on local-costmap observation checks before launching Nav2. The C++ node keeps `restamp_to_now=false` for source timing diagnosis, and Nav2 builds its own costmap/TF buffers during normal lifecycle activation. Hardware validation should still inspect `/local_costmap/costmap`, `/perception/obstacle_points`, and TF logs after startup, but those checks are diagnostic rather than startup blockers.

Nav2's behavior tree is a coordinator, not the low-level controller loop. The field runtime uses `bt_loop_duration=50`, `default_server_timeout=1000`, and `wait_for_service_timeout=2000` so planner action acknowledgements, costmap clear services, and recovery behavior action servers are not falsely timed out by short Jetson CPU/DDS bursts. Motion safety remains downstream in the controller, velocity smoother, collision monitor, and `robot_safety`.

Return-to-dock uses Nav2 only up to the pre-dock approach pose. The backend prefers a manual point (`predock_pose_id`, `approach_pose_id`, or a saved pose such as `dock_main_predock`) and validates its yaw against the dock contact pose. Manual pre-dock distance checking is disabled by default (`docking_manual_predock_distance_check_enable=false`), so a close but intentionally saved point is not rejected only because it is below the old `0.50m` lower bound. If no manual point exists, the backend falls back to a geometric offset from the saved dock contact pose. After Nav2 reports success, `robot_api_server` triggers localization again and checks the refreshed `map -> base_link` pose against the approach pose before handing motion to `robot_docking_manager`. If the pose is outside the configured tolerance, GS2 fine docking is not started. Once fine docking succeeds, fails, or is stopped, the API triggers another localization refresh in `relocalize_after_fine_docking` before publishing the final docking state, so the next Nav2 action starts from a corrected `map -> odom` rather than odometry accumulated during non-Nav2 fine docking.

Normal point navigation also treats dock/contact state and localization freshness as gates. `robot_api_server` first builds one pre-navigation dock-contact snapshot from backend docking state, `/docking/status`, and fresh Ranger BMS charging contact. If the backend state is `docked`, `/docking/status` starts with `docked` or `charging`, or BMS contact is active, `POST /api/v1/navigation/goal` performs controlled `/docking/undock` and waits for post-undock relocalization before any Nav2 action is sent. Full batteries may report `current=0`, so this gate uses `power_supply_status=FULL/CHARGING`, BMS contact reason, valid contact voltage, and configured full-SOC contact inference rather than current alone. `GET /api/v1/navigation/pre_goal_check` and `scripts/jetson/runtime_overlay/scripts/verify_pre_navigation_undock_gate.sh` expose the same gate read-only for field diagnosis. After the dock/contact gate passes, `robot_api_server` checks the critical Nav2 lifecycle nodes, triggers `/global_localization/trigger`, waits for a fresh `/localization_result`, and verifies that `robot_localization_bridge` accepted the resulting `map -> odom` before sending `/navigate_to_pose`. If Nav2 lifecycle heartbeat loss has left controller/planner/BT/costmap nodes inactive, the API reports navigation as degraded and rejects new goals instead of exposing a false running state. Accepted goals are tracked as a background `navigation_goal`; the business completion rule is position-first. If the target position is reached but final yaw alignment is blocked by the safety chain, the API reports `position_reached_yaw_warning` with `final_yaw_align_blocked=true` and still marks the goal succeeded so a delivery does not fail only because the last in-place spin was unsafe.

By default, runtime scripts reuse common services:

```bash
NJRH_REUSE_COMMON_SERVICES=true
```

Force restart is explicit and should be used only for repair:

```bash
NJRH_FORCE_RESTART_DRIVER=true
NJRH_FORCE_RESTART_CANONICAL_TF=true
NJRH_FORCE_RESTART_NAV_HELPERS=true
```

Disable GS2 common-service startup only for bench tests or when the sensor is physically disconnected:

```bash
NJRH_GS2_AUTOSTART=false
```

When enabled through `njrh-runtime.service`, the host runner resolves `/dev/gs2` to its real tty device and passes that path into the container as `NJRH_GS2_SERIAL_PORT`.

Start common services:

```bash
NJRH_DASHBOARD_HOST=192.168.31.23 bash scripts/jetson/njrh_container.sh start-runtime
```

If the container already exists and you only need to restart the common layer:

```bash
bash scripts/jetson/njrh_container.sh start-common
```

Enable boot autostart on the Jetson host:

```bash
cd /home/nvidia/workspaces/njrh-v3/workspace1
bash scripts/jetson/install_njrh_autostart.sh install
```

The host `njrh-runtime.service` owns the production common-service process. It starts or reuses the container with `njrh_container.sh start`, then runs one foreground `run_common_services.sh` process for systemd supervision. Do not make the systemd runner call `start-runtime`, because that command also starts common services in the background.

Start the Web dashboard only when debugging:

```bash
bash scripts/jetson/njrh_container.sh start-dashboard
```

Daily navigation can then start or reuse the selected-floor resident navigation
layer:

```bash
docker exec -it NJRH-car bash -lc \
  'cd /workspaces/njrh-v3/workspace1 && NJRH_BUILDING_ID=building_1 NJRH_FLOOR_ID=floor_1 bash scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh'
```

`run_navigation_runtime_services.sh` resolves the selected floor assets, starts the resident occupancy-localization layer, starts or reuses the floor-manager process, sends one global-localization trigger request, waits for the initial `/localization_result` and `map -> odom`, then starts the Nav2 child script. It marks the runtime context ready only after Nav2 lifecycle activation and the global costmap are available. `run_nav2_navigation.sh` still starts or reuses floor-manager, robot_safety, ranger mode controller, and local perception by process ownership only, then launches the repository-owned standard Nav2 stack without blocking on `/safety/status`, `/perception/obstacle_points`, or local-costmap observation probes.

`run_floor_navigation.sh` is compatibility-only and delegates to `run_navigation_runtime_services.sh`.

If resident navigation startup exits before the runtime context reaches confirmed `ready`, `run_navigation_runtime_services.sh` writes the context as `failed` with the resume log path and tears down both Nav2 and the occupancy-localization helper layer. The App should show that failed state instead of waiting forever on `starting`.

Manual floor-navigation stop clears the runtime map context after killing Nav2/localization helper processes. This prevents `robot_api_server` from recovering a stale `ready` context after the stack has been stopped.

`robot_api_server` now treats a repeated same-floor resume as idempotent when the runtime context is already confirmed `ready` for the requested `map_id/building_id/floor_id` and the existing resume process is still alive. In that case it returns `navigation_runtime_reused` and does not signal the old process group, so the old cleanup trap cannot tear down an already-ready Nav2/localization stack.

Safety state is a core runtime input rather than an App page subscription. The
API keeps `/safety/status` and `/safety/motion_allowed` subscribed with
transient-local QoS for the lifetime of the process, so a late-started or
restarted API still receives the last safety arbitration state before it answers
status, docking, teleop, or navigation requests.

`GET /api/v1/status` and `GET /api/v1/navigation/state` must stay lightweight.
They refresh local process/context caches only and do not synchronously probe
Nav2 lifecycle services on every poll. Blocking lifecycle checks remain on
runtime resume, navigation goal admission, and explicit readiness diagnostics so
mobile polling cannot exhaust the API connection limit.

Default field mapping requires navigation mode services to be stopped first,
keeps common services alive, clears the transient runtime map context, then
starts the slam_toolbox 2D mapping chain. `robot_api_server` rejects App 2D
mapping start with `409` while the navigation runtime is active instead of
half-canceling a Nav2 task and then launching mapping helpers with overlapping
node names. Optional formal 3D mapping may start FAST-LIO2/PGO explicitly.
The Web dashboard is still a test UI; its stop-core path now keeps
driver/chassis/common services alive by default. Saving a 2D map writes the map
bundle under `maps_release/<building_id>/<floor_id>/maps/<map_id>/` but does not
activate it for navigation. The App must explicitly select the saved map with
`POST /api/v1/floors/switch` and `resume_navigation=true`; that selection writes
`last_navigation_map.json` for the next boot.

The API promotes 2D mapping from `starting` to `running` only after a live `/map` occupancy grid from the App-started `slam_toolbox` session is fresh and image-renderable. While 2D mapping is active, `robot_api_server` keeps its own `/map` cache subscription alive for startup readiness, `/api/v1/status`, and save operations; the App page lease is still required for live PNG rendering. `GET /api/v1/status` exposes `mapping.live_map_available`, `mapping.live_map_age_sec`, and the current live map dimensions so App-side delays can be distinguished from backend startup failures.

Live 2D mapping sets `slam_toolbox.scan_queue_size=30` and `transform_timeout=0.50` to tolerate short TF/scan timestamp jitter in the JT128 -> FAST-LIO -> flatscan chain. It also enables conservative 2D loop closing (`loop_search_maximum_distance=3.0`, `loop_match_minimum_response_coarse=0.40`, `loop_match_minimum_response_fine=0.50`, `loop_search_space_dimension=6.0`) so long 2D mapping routes can close planar drift without loose matches in repeated indoor geometry. The startup script waits up to `NJRH_SLAM2D_ODOM_READY_TIMEOUT=30` seconds for fresh resident `/local_state/odometry`, then starts a mapping-owned FAST-LIO2 frontend and waits up to `NJRH_SLAM2D_FASTLIO_POINTS_READY_TIMEOUT=60` seconds for `/cloud_registered_body`. It does not start, kill, or repair canonical TF/local-state; cleanup stops only the C++ mapping bridge and a FAST-LIO2 process carrying the `NJRH_SLAM2D_PRIVATE_FASTLIO=1` marker. It also compares `/local_state/odometry` with the resident local odom reference and refuses mapping if the difference exceeds `NJRH_SLAM2D_LOCAL_ODOM_MAX_WHEEL_DIFF_M=25.0`; this catches a diverged local-state process before `slam_toolbox` can create a corrupted map. If `Message Filter dropping message ... queue is full` continues after restart, treat it as a TF timing or producer-rate problem rather than simply increasing the queue again.

The API's mapping-stop residual sweep must keep this ownership boundary: FAST-LIO2 is cleaned only when both the command line matches the FAST-LIO2 mapping binary and `/proc/<pid>/environ` contains `NJRH_SLAM2D_PRIVATE_FASTLIO=1`. Residual cleanup must not match generic scan/localization helper names such as `nav_cloud_preprocessor`, `pointcloud_to_laserscan_node`, or `scan_republisher_node`; those names are also used by resident navigation localization. Hardware validation after an App mapping stop should confirm no mapping-owned `fast_lio`/`fastlio`/`laser_mapping` process remains, while the driver, chassis, `robot_local_state`, canonical TF publisher, `robot_safety`, `robot_api_server`, and any active resident navigation services are still alive.

The Windows Jetson helper refreshes `scripts/jetson/runtime_overlay` before remote actions. If a stale root-owned remote overlay prevents deletion, `Invoke-NJRHJetson.ps1` quarantines it as `runtime_overlay.stale.<timestamp>` and uploads a clean overlay so status/start commands are not blocked by old temporary logs.
