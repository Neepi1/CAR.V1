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

Phase 1.14 wires the pointcloud acceleration profile into the runtime driver
entrypoint without changing the canonical trunk. `NJRH_POINTCLOUD_ACCEL_PROFILE=legacy`
remains the default and keeps the validated standalone branch chain. In legacy,
`pointcloud_axis_remap_node` owns the only `/lidar_points` full-density trunk,
`robot_local_perception` owns `/perception/obstacle_points` and
`/perception/clearing_points`, and `jt128_localization_sensing.launch.py`
recovers the minimal scan chain
`/lidar_points_nav -> nav_cloud_preprocessor -> /points_nav ->
pointcloud_to_laserscan -> /scan_raw -> scan_republisher -> /scan ->
laser_scan_to_flatscan -> /flatscan`. The legacy restart path restores that
chain without starting the full localizer. `ipc_worker` starts
`pointcloud_accel_axis_node` as the single `/lidar_points` publisher; its raw
callback publishes full-density/full-fields `/lidar_points` and updates a latest
normalized buffer before returning. Worker threads named `pc_accel_local` and
`pc_accel_scan` then derive `/perception/obstacle_points`,
`/perception/clearing_points`, and `/scan`; `laser_scan_to_flatscan` converts
`/scan` to `/flatscan`. In this profile, `/_internal/lidar_points_local` and
`/lidar_points_nav` are compact XYZ/XYZI debug/compat branches, and `/points_nav`
is not a production hop. `nitros` is a guarded skeleton for future compact
navigation-branch acceleration only. The NITROS path must run future accelerated
nodes in one same-process component container and must not replace
`/lidar_points` or FAST-LIO2 mapping input.

Phase 1.15 treats `/flatscan` as a supervised localization-startup dependency,
not just a side effect of the scan chain. In `legacy`,
`jt128_localization_sensing.launch.py` owns the `laser_scan_to_flatscan` node and
`run_pointcloud_accel_pipeline.sh` supervises that launch or detects a missing
`/flatscan` publisher. In `ipc_worker`, `pointcloud_accel_axis_node` owns
`/scan`, while a bounded-restart standalone `laser_scan_to_flatscan`
compatibility helper owns `/flatscan`. Helper state is written to
`flatscan_helper_status.env` for `verify_pointcloud_accel_profile.sh` and
`run_pointcloud_accel_ab.sh`. If `/scan` has a publisher but `/flatscan` does
not, diagnostics report `CASE_FLATSCAN_HELPER_DEAD`. This does not add topics or
change PointCloud2 QoS, DDS/RMW, timestamp policy, FAST-LIO2, EKF, App API, or
Nav2 controller/planner behavior.

Phase 2.4a audits local costmap obstacle timestamp drops without changing
runtime behavior. `/perception/obstacle_points` frequency alone does not prove
the local costmap accepted the observation; the costmap also needs a compatible
`base_link` observation frame and TF cache coverage for the cloud header stamp.
`pointcloud_accel_axis_node` and `robot_local_perception` now report source
header age, latest-buffer age, and obstacle/clearing output header/source age
on their existing low-rate status topics. The read-only
`verify_local_costmap_observation_timestamp_root_cause.sh` script classifies
raw-stamp, internal-buffer, output-stamp, TF-cache, startup-warm-up, and frame
mismatch cases and writes a markdown report. It does not restamp clouds, change
`tf_filter_tolerance`, change observation persistence, or alter pointcloud,
DDS/RMW, EKF, FAST-LIO2, App API, or Nav2 controller/planner behavior.

Phase Z1 tightens the `ipc_worker` implementation without adding topics:
workers no longer keep or parse a latest `PointCloud2` snapshot. The raw
callback publishes the full trunk, refreshes a latest normalized in-process
buffer, and exits; obstacle, clearing, compact compatibility, and scan workers
read that buffer directly and only allocate/build the final existing outputs.
The existing `/lidar/pointcloud_accel_status` reports
`internal_zero_copy_profile=true`, latest internal buffer size, worker full
cloud copy counters, intermediate PointCloud2 build counters, allocation counts,
lock wait maxima, and processing averages.

Phase D1 separates pointcloud acceleration from ingress selection. The default
`NJRH_POINTCLOUD_INGRESS_PROFILE=separate_process` keeps the validated runtime:
`hesai_ros_driver_node` decodes JT128 UDP packets and publishes the decoded ROS
`PointCloud2` topic `/jt128/vendor/points_raw`; `pointcloud_accel_axis_node`
subscribes to that topic and calls the shared C++ `PointCloudAccelCore`. The
`driver_integrated` profile builds the repo-owned
`src/third_party/hesai_lidar_ros2_overlay` source and starts
`hesai_accel_driver_node`; Hesai decode constructs one in-process `PointCloud2`
and moves it directly into `PointCloudAccelCore`, so `/jt128/vendor/points_raw`
is no longer a production DDS input. `/jt128/vendor/imu_raw` remains available
for the existing IMU remap path. Rollback is
`NJRH_POINTCLOUD_INGRESS_PROFILE=separate_process`.
`/lidar_points` remains full-density/full-fields for FAST-LIO2 mapping, and no
new pointcloud or status topics are introduced.

The full-size pointcloud trunk is latest-only after the upstream Hesai driver:
`/lidar_points` is the high-density canonical trunk used by mapping-owned
FAST-LIO2 and diagnostic consumers. It must not be compacted or downsampled.
In `legacy`, production `pointcloud_axis_remap_node` runs as a standalone process,
publishes the only `/lidar_points` trunk, derives `/lidar_points_nav` at stride
4 with `nav_output_publish_every_n=2` for approximately 10 Hz localization scan
preprocessing, and derives `/_internal/lidar_points_local` at stride 2 with
`local_output_publish_every_n=1` when `NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=local_branch`.
That profile is the production default and routes the separate
`robot_local_perception` process to `/_internal/lidar_points_local`;
`NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=trunk` disables the hidden branch and routes
local perception back to `/lidar_points` for rollback/diagnosis. In `ipc_worker`,
`pointcloud_accel_axis_node` owns `/lidar_points` and same-process workers own
`/perception/obstacle_points`, `/perception/clearing_points`, and `/scan`;
`/_internal/lidar_points_local`, `/lidar_points_nav`, and `/points_nav` are not
production-required DDS hops. `ROBOT_LOCAL_PERCEPTION_INPUT_TOPIC` overrides the
legacy profile input topic, and explicit `NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_TOPIC`,
`NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_STRIDE`, and
`NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N` values override legacy branch
injection. `/lidar_points`, `/lidar_points_nav`, `/_internal/lidar_points_local`,
`/points_nav`, `/perception/obstacle_points`, and `/perception/clearing_points`
use best-effort QoS with depth `1`. Derived branches must not become additional
canonical `/lidar_points` publishers. `/points_nav` depends on the upstream
`jt128_nav_tools/nav_cloud_preprocessor` patch at
`scripts/jetson/runtime_overlay/patches/jt128_nav_tools_pointcloud_qos.patch`;
the patched build is sourced from
`${NJRH_JT128_NAV_TOOLS_PATCHED_OVERLAY:-${PROJECT_ROOT}/.runtime/jt128_nav_tools_overlay/install}`
by `common_env.sh`. Without that overlay the preprocessor falls back to its old
mixed QoS behavior and can reintroduce large-message queues. Debug commands,
RViz/Foxglove views, and rosbag captures should not stay attached to
`/lidar_points`; use status topics or reduced/debug branches where possible, and
require explicit env flags for bagging high-density clouds.

For accel-profile validation, use `set_pointcloud_accel_profile.sh`,
`verify_pointcloud_accel_profile.sh`, and `run_pointcloud_accel_ab.sh`. The
verify script reports requested/resolved profile, trunk/obstacle/clearing/
points_nav/scan/flatscan owners, `/lidar_points` publisher count, Nav2
subscribers, status topics, internal zero-copy counters, FAST-LIO2 residuals,
and final owner-contract summary flags. A valid `ipc_worker` run keeps
`/lidar_points` publisher count at one, keeps the trunk near vendor rate with
full fields, reports zero worker full-cloud copies and zero intermediate
PointCloud2 builds, leaves FAST-LIO2 non-resident during normal navigation, and
still feeds Nav2 through `/perception/obstacle_points`,
`/perception/clearing_points`, `/scan`, and `/flatscan`. Rollback is one command:
`bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile legacy --restart`.

`robot_local_perception` is pinned to the localization/local perception CPU set instead of sharing the Nav2 control CPU set or the raw remap ingress CPU. The local costmap, MPPI controller, collision monitor, and localization bridge need the control/TF cores to service TF message filters and command callbacks with low latency; obstacle filtering is a lidar-derived perception workload and should not compete with those callbacks or with full-size raw remap copies.

The local perception hot path keeps `/perception/obstacle_points` ahead of clearing work: each frame fuses input rotation and TF into one 3x4 transform, filters directly from the PointCloud2 byte buffer, writes obstacle output directly as PointCloud2, and then hands the newest clearing job to a latest-only worker when the configured clearing cadence fires. Voxel speckle suppression uses packed keys in an unordered map rather than a tree map of tuple keys, so NORMAL-mode filtering does not add avoidable per-point allocation/comparison overhead. The clearing worker overwrites stale pending jobs instead of building a backlog, so `/perception/clearing_points` can fall behind briefly without delaying obstacle marking.

For field diagnosis, the same C++ node publishes `/perception/local_perception_status` every two seconds with received, accepted, process-timer, obstacle, clearing, no-new, empty-input, empty-obstacle, TF/stamp/publish-gating, active profile, filter bounds, point stride, clearing cadence, and processing-time counters. Use it to tell whether low `ros2 topic hz` readings come from the local perception loop itself, filter/gating emptiness, or CLI/DDS subscriber pressure; do not add Python runtime probes for this path.

Pointcloud source and localization-preprocessor diagnostics are also emitted from C++ nodes. `pointcloud_axis_remap` publishes `/lidar/axis_remap_status` at 1 Hz with raw input rate, `/lidar_points` publish rate, raw callback inter-arrival, `/lidar_points` publish interval and gap counters, raw callback duration, trunk/branch/total publish timing, cloud size, output subscriber count, branch flags, branch publish/skip/attempt rates, branch subscriber counts, branch last points/bytes/duration, and skipped-cloud count. The patched `nav_cloud_preprocessor` publishes `/lidar/nav_cloud_preprocessor_status` at 1 Hz with input callback/accept rate, `/points_nav` output rate, TF/empty/filter-empty skips, input inter-arrival, input stamp/message age, processing average/max, input/output point counts, lookup timeout, source/target frame, range/height filters, matched publisher/subscriber counts, and QoS. These status topics are the preferred field inputs for `diagnose_lidar_points_jitter.sh`, `diagnose_local_perception_pipeline.sh`, `diagnose_nav_scan_pipeline.sh`, `verify_lidar_trunk_jitter.sh`, and `verify_pointcloud_delivery_matrix.sh`; use direct `ros2 topic hz` on full-density clouds only when a status topic is unavailable or when deliberately comparing subscriber-side delivery with `--include-cli-hz` or a manual `NJRH_VERIFY_MATRIX_LIDAR_POINTS_CLI_HZ` value for CASE_G.

The default navigation local-state mode is wheel-odom + JT128-IMU EKF because Nav2 control needs low-latency, continuous `odom -> base_link` more than globally consistent lidar-inertial smoothing. When FAST-LIO local-state mode is explicitly selected, `/Odometry` can arrive behind wall time because the lidar-inertial frontend has processing latency; that latency must be diagnosed at the producer and transport layers, not hidden by changing local-obstacle cloud stamps. `robot_local_perception` publishes `/perception/obstacle_points` and `/perception/clearing_points` with the input pointcloud acquisition stamp preserved. The local costmap rolling window runs in `odom` so controller progress checks see a robot pose that changes with motion; `robot_base_frame` remains `base_link`, and obstacle/clearing sources keep `sensor_frame=base_link` because those clouds are already expressed in the robot frame. Do not use `base_link` as the local costmap `global_frame`: it makes the pose returned to controller-side progress checking nearly constant in the robot frame and can cause false `Failed to make progress` aborts. The local perception hot path uses latest/static TF for the `lidar_link -> base_link` sensor extrinsic (`input_transform_use_latest=true`) while preserving the original cloud header stamp, so static extrinsic lookup cannot block every obstacle frame. Startup-time costmap MessageFilter drops are kept as diagnostics, not as shell-level startup blockers.

Canonical local-state is owned by common services, not by Nav2 startup. In the default EKF mode, common services start the wheel-odom preprocessor, IMU bias filter, and `robot_localization` EKF process, and process existence is the startup ownership check. In explicit FAST-LIO local-state mode the common layer requires both `fastlio_odom_bridge_node` and `robot_local_state/local_state_node`. Endpoint/topic/TF validation is manual diagnostics and API admission, not a shell launch gate. Runtime graph-probe misses under Nav2 startup load must not cause the canonical `odom -> base_link` owner to kill itself.

Phase 2.10 protects the local-state EKF DDS receive queues by bounding only the
EKF input branches. `/lidar_imu` remains the high-rate raw JT128 IMU stream for
FAST-LIO2 mapping, while `imu_gyro_bias_filter` publishes
`/lidar_imu_bias_corrected` at 100 Hz and `/local_state/imu_bias` at 10 Hz by
default. The wheel-odom EKF preprocessor publishes `/wheel/odom_ekf` at 50 Hz
from its timer with callback publishing disabled, matching the EKF output rate.
This does not change `frequency: 50.0`, PointCloud2 QoS, DDS/RMW defaults,
FAST-LIO2 inputs, Nav2 controller/planner settings, or runtime watchdog policy.
Use `verify_local_state_input_rates.sh` after a local-state restart to check
topic rates, ROS graph visibility, `/tf` ownership, EKF subscribers, and UDP
`RcvbufErrors` deltas.

FAST-LIO2 itself is no longer a default navigation odom dependency and is no longer resident in normal navigation. `run_common_services.sh` defaults `NJRH_FASTLIO_AUTOSTART=false`; `run_projected_map.sh` starts a mapping-owned FAST-LIO2 frontend only while live mapping is active. FAST-LIO2 becomes a local-state dependency only when `NJRH_NAV_LOCAL_STATE_MODE=fastlio` is explicitly selected, and that mode must either enable `NJRH_FASTLIO_AUTOSTART=true` or attach to an already managed FAST-LIO runtime.

Nav2 preflight is process-first. `run_nav2_navigation.sh` starts or reuses floor-manager, `robot_safety`, Ranger mode-controller, and local perception by process ownership, then launches Nav2. It does not run local-state, TF, `/safety/status`, obstacle-cloud, or local-costmap probes as startup gates, and it does not kill/restart local perception just to warm a new costmap buffer.

The occupancy localization mode may start or reuse localization-specific services such as `robot_localization_bridge` and `robot_global_localization`, but it does not own FAST-LIO2 or canonical local-state. It consumes `/lidar_points` for stationary Isaac relocalization and checks only that the resident local-state process for the selected mode exists before launching the localization stack; FAST-LIO2 is required only for explicit `NJRH_NAV_LOCAL_STATE_MODE=fastlio`. On localization or Nav2 startup failure the mode script cleans the localization stack and overlay helpers only; `robot_local_state` remains a common/canonical service. This prevents Nav2 from staying active while `/local_state/odometry` and `/tf` publishers disappear during a stop/resume race.

Navigation resume gives the localization layer a short settle window (`NJRH_NAV_LOCALIZATION_START_SETTLE_SEC`, default 3 seconds), then runs a bounded deterministic startup chain before Nav2 is allowed to become ready. The chain waits for `/global_localization/trigger`, Isaac `/trigger_grid_search_localization`, explicitly drives `/map_server` to active for the selected floor asset, requires the selected `/map` to be observable, waits for an observed `/flatscan` message and a `/localization_result` publisher, sends one `/global_localization/trigger`, then requires a fresh `/localization_result` and live `map -> odom`. `/flatscan` is `isaac_ros_pointcloud_interfaces/msg/FlatScan`, not `sensor_msgs/msg/LaserScan`; `/scan` is the LaserScan intermediate. The FlatScan gate checks message flow rather than header freshness because the generic startup freshness tool intentionally supports only standard stamped ROS messages. If this gate fails, `run_navigation_runtime_services.sh` now records the specific reason `FLATSCAN_MISSING` and logs `/scan` publisher presence, the `laser_scan_to_flatscan` process state, the active pointcloud accel profile, and the verify/restart command. Other admission failures are split into `GLOBAL_LOCALIZATION_TRIGGER_SERVICE_MISSING`, `GRID_SEARCH_LOCALIZATION_SERVICE_MISSING`, `MAP_SERVER_NOT_ACTIVE`, `MAP_TOPIC_MISSING`, and `LOCALIZATION_RESULT_PUBLISHER_MISSING`. This is the required localization sequence, not a high-frequency Python/rclpy graph watchdog.

The local costmap `MessageFilter` drops observed on `/perception/obstacle_points`
are separate from `/flatscan` startup admission. They require a later timestamp
source audit and must not be hidden by restamping or mixed into the flatscan
lifecycle fix.

Standard Nav2 startup uses two lifecycle managers. `lifecycle_manager_costmap_filters` owns only the keepout/speed mask map servers and filter-info servers. `lifecycle_manager_navigation` owns the core controller, smoother, planner, behavior tree, waypoint follower, velocity smoother, and collision monitor nodes, and starts after `NJRH_NAV_LIFECYCLE_START_DELAY_SEC` seconds (default 18 seconds) so the filter services and map-mask files are already stable. This avoids aborting the whole Nav2 bringup because a filter-mask load or planner/global-costmap configuration burst delays a lifecycle service response.

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

Navigation resume scripts do not restore broad ROS graph/topic/TF probe loops, but they do keep the critical localization startup chain as a gate. `run_navigation_runtime_services.sh` starts the selected-floor localization layer, verifies the initial localizer inputs/services, sends one bounded global-localization trigger request, waits for fresh `/localization_result` and `map -> odom`, starts Nav2, then marks the runtime context ready only after Nav2 lifecycle activation and the global costmap are available. The runtime context identity comes from the selected floor assets: `resolve_floor_assets` reads `asset_report.json`, exports `NJRH_NAV_MAP_ID`, and mirrors it into `NJRH_MAP_ID` so `/api/v1/robot/pose` can attach the confirmed `building_id` / `floor_id` / `map_id` instead of rejecting an otherwise fresh TF pose. Explicit readiness tools still exist for field diagnostics, but a transient Fast DDS discovery miss outside this critical chain must not keep the App in `starting`. The API server also polls the navigation resume child process while serving `/api/v1/status` and `/api/v1/navigation/state`; if the child process exits during startup or the runtime context records `state=failed`, the API reports navigation `failed` with the resume log path instead of leaving the App in `starting`.

The occupancy localization bridge watchdog follows the same process-first rule. It still rejects a real `robot_localization_bridge` process loss, but ROS graph probe misses during Nav2/map-server activation are diagnostics only while the bridge process is still alive.

Local perception stays resident across Nav2 restarts. `run_nav2_navigation.sh` no longer kills local perception, primes a probe-owned TF buffer, or blocks on local-costmap observation checks before launching Nav2. The C++ node keeps `restamp_to_now=false` for source timing diagnosis, and Nav2 builds its own costmap/TF buffers during normal lifecycle activation. Hardware validation should still inspect `/local_costmap/costmap`, `/perception/obstacle_points`, and TF logs after startup, but those checks are diagnostic rather than startup blockers.

Nav2's behavior tree is a coordinator, not the low-level controller loop. The field runtime uses `bt_loop_duration=50`, `default_server_timeout=1000`, and `wait_for_service_timeout=2000` so planner action acknowledgements, costmap clear services, and recovery behavior action servers are not falsely timed out by short Jetson CPU/DDS bursts. Motion safety remains downstream in the controller, velocity smoother, collision monitor, and `robot_safety`.

RotationShim startup alignment and progress checking are intentionally separated from terminal mission yaw. `RotationShimController` still wraps MPPI for large path-entry heading errors, but Nav2 uses `PoseProgressChecker` with `required_movement_radius=0.10`, `required_movement_angle=0.10`, and the unchanged `movement_time_allowance=12.0` so measurable in-place yaw progress is not falsely aborted as no XY movement. `FollowPath.rotate_to_goal_heading=false` because `robot_api_server` verifies the final position first and then runs bounded mission-layer `final_yaw_align` when needed. If a short goal still produces angular-only commands, use `diagnose_nav2_zero_linear_progress_failure.sh` to classify whether the zero-linear behavior originates in the controller, collision monitor, robot_safety, mode controller/chassis, or odom reflection before changing pointcloud, DDS, costmap, EKF, FAST-LIO2, or App API behavior.

Return-to-dock uses Nav2 only up to the pre-dock approach pose. The backend prefers a manual point (`predock_pose_id`, `approach_pose_id`, or a saved pose such as `dock_main_predock`) and validates its yaw against the dock contact pose. Manual pre-dock distance checking is disabled by default (`docking_manual_predock_distance_check_enable=false`), so a close but intentionally saved point is not rejected only because it is below the old `0.50m` lower bound. If no manual point exists, the backend falls back to a geometric offset from the saved dock contact pose. After Nav2 reports success, `robot_api_server` triggers localization again and checks the refreshed `map -> base_link` pose against the approach pose before handing motion to `robot_docking_manager`. If the pose is outside the configured tolerance, GS2 fine docking is not started. Once fine docking succeeds, fails, or is stopped, the API triggers another localization refresh in `relocalize_after_fine_docking` before publishing the final docking state, so the next Nav2 action starts from a corrected `map -> odom` rather than odometry accumulated during non-Nav2 fine docking.

After each docking relocalization gate, the API also waits for the live TF chain to settle: `map -> odom`, `odom -> base_link`, and the composed `map -> base_link` pose must all be fresh before the pre-dock Nav2 goal or GS2 fine-docking handoff proceeds. Docking cancel calls `/docking/stop` with the configured service wait and records that result instead of treating a short service-discovery miss as a clean stop.

Normal point navigation also treats dock/contact state and localization freshness as gates. `robot_api_server` first builds one pre-navigation dock-contact snapshot from backend docking state, `/docking/status`, and fresh Ranger BMS charging contact. If the backend state is `docked`, `/docking/status` starts with `docked` or `charging`, or BMS contact is active, `POST /api/v1/navigation/goal` performs controlled `/docking/undock` and waits for post-undock relocalization before any Nav2 action is sent. Full batteries may report `current=0`, so this gate uses `power_supply_status=FULL/CHARGING`, BMS contact reason, valid contact voltage, and configured full-SOC contact inference rather than current alone. `GET /api/v1/navigation/pre_goal_check` and `scripts/jetson/runtime_overlay/scripts/verify_pre_navigation_undock_gate.sh` expose the same gate read-only for field diagnosis. After the dock/contact gate passes, `robot_api_server` checks the critical Nav2 lifecycle nodes and evaluates a when-needed relocalization decision: confirmed same-map runtime context plus a fresh map-frame pose skips `/global_localization/trigger`; cold/unconfirmed context, stale or missing map-frame pose, undock, docking transitions, or explicit `force_relocalize` still trigger localization and require `robot_localization_bridge` to accept the resulting `map -> odom` before sending `/navigate_to_pose`. This prevents repeated normal goals from injecting avoidable `map -> odom` jumps while preserving recovery and startup safety. If Nav2 lifecycle heartbeat loss has left controller/planner/BT/costmap nodes inactive, the API reports navigation as degraded and rejects new goals instead of exposing a false running state. Accepted goals are tracked as a background `navigation_goal`; after Nav2 returns, the API enters `position_reached_verifying`, checks a fresh `map -> base_link` pose, and only then optionally enters `position_reached_yaw_aligning` for a bounded mission-layer final yaw correction. The command currently enters `/cmd_vel_collision_checked`, so `robot_safety` remains authoritative while `final_yaw_align_bypass_collision_monitor=true` is reported until a `/cmd_vel_nav` mux is added. It then enters `final_pose_verifying` and reports `final_pose_verified` or a specific warning/failure reason. The business completion rule remains position-first: if the target position is reached but final yaw alignment is blocked by the safety chain, the API reports `position_reached_yaw_warning` with `final_yaw_align_blocked=true` and still marks the goal succeeded so a delivery does not fail only because the last in-place spin was unsafe.

Phase 2.5 adds a non-position dock-contact latch as a second dock-state source. The latch is written by explicit events only: BMS contact, `/docking/status` docked/charging, docking success, and undock success. It is not inferred from the robot's current map pose. `pre_navigation_dock_check` exposes the latch as `dock_contact_snapshot`; if it is docked, normal navigation must run `/docking/undock` before Nav2. `final_yaw_align` rechecks the same gate and exits with `DOCKED_OR_CHARGING_CONTACT` instead of rotating. `robot_safety` also subscribes BMS and `/docking/status`, reads the same latch, and reports `DOCKED_CONTACT_BLOCK` while zeroing normal commands; `/cmd_vel_docking` remains allowed for controlled docking/undocking, including between watchdog timer ticks while the docking command is fresh. `/api/v1/status` and `/api/v1/navigation/state` expose safety status and `normal_motion_blocked_reason` so the App can display the blocker without inferring dock state from position.

Phase 2.6 extends that latch for full-charge and missing-contact recovery. `docking_contact_latch.json` now carries `latched_docked`, source, map/floor context, timestamps, clear reason, and note fields while retaining the old `docked` field for compatibility. Maintenance endpoints/scripts can confirm or clear the latch without sending velocity. BMS contact false does not clear the latch, because a full or signal-missing charger state can report no current/contact. `pre_navigation_dock_check.docked_state_class` reports `DOCKED_CONFIRMED`, `DOCKED_LATCHED`, `NOT_DOCKED`, or `UNKNOWN`; ordinary navigation auto-undocks for confirmed and latched docked states.

Phase 2.8 keeps the same docking ownership but splits undock progress timing into explicit phases. `robot_docking_manager` still owns near-field docking and controlled undocking; return-to-dock travel remains Nav2 up to the pre-dock pose. The undock path remains `/cmd_vel_docking -> robot_safety -> /cmd_vel_safe -> ranger_mini3_mode_controller -> /cmd_vel`, and ordinary Nav2 reverse remains disabled (`vx_min` non-negative). The retained calibrated speed is `undock.speed_mps=0.06`. `undock.command_settle_s` allows the Ranger park/forced-mode/reverse-enable state to settle before nonzero undock commands, `undock.motion_start_timeout_s` waits for first odometry-confirmed motion, and `undock.no_progress_timeout_s` is used only after first motion to detect a mid-undock stall. The total `undock.timeout_s` must cover command settle, first-motion wait, `distance / speed`, and margin. Use `scripts/jetson/runtime_overlay/scripts/diagnose_undock_logic_and_no_motion.sh --dry-run` for static/API checks, and `--execute-undock` only for a supervised controlled undock diagnostic.

Phase 2.7c tightens the motion-start phase so it cannot wait for odometry before sending the reverse command. After `command_settle_s`, every control tick in `waiting_first_motion` publishes `/ranger_mini3/docking_allow_reverse=true` and `/cmd_vel_docking.linear.x=-0.06` while waiting for `/local_state/odometry` to move by `progress_epsilon_m`. `/docking/status` includes `cmd_x`, `cmd_count`, `reverse_enable`, and timing fields. `undock_failed_motion_start_timeout ... cmd_count>0` means commands were sent and the next diagnosis should follow `robot_safety`, mode-controller, chassis execution, and odometry. `undock_failed_no_command_published` or `cmd_count=0` means the docking-manager state machine did not publish the undock command and must be treated as a software bug. This does not change Nav2, pointcloud, DDS/RMW, EKF, FAST-LIO2, Ranger CAN, App velocity ownership, or the final `robot_safety` speed chain.

Phase 2.7d keeps the same speed and ownership but makes the final safety arbiter continuous for push-in spring charging docks. Because the charging switch is mechanically engaged by pushing into the dock, undocking must drive at the controlled low speed through the switch travel rather than stopping on the DC contact. `robot_safety` stores the last fresh `/cmd_vel_docking` command and republishes it from its safety timer while `docking_cmd_priority_timeout_sec` is active; blocking states still publish zero, stale commands still expire, and ordinary Nav2 reverse remains disabled. `diagnose_undock_logic_and_no_motion.sh` now also treats API `cmd_count` evidence as command evidence, so reports with `cmd_count>0` are classified downstream of the docking manager instead of as no-command state-machine failures.

Phase 2.7d observability reconciliation keeps that control behavior unchanged and only separates command evidence. `/docking/status` now carries parseable `phase`, `cmd_count`, `reverse_enable_count`, `last_cmd_x`, command-age, command-start elapsed, and first-motion fields through running and terminal undock states. `/api/v1/docking/undock` and `/api/v1/docking/state` distinguish API acceptance from the underlying `/docking/undock` Trigger response using `api_accepted`, `already_running`, `docking_service_called`, `docking_service_success`, and `docking_service_message`. The undock diagnostic script arms all topic observers before calling the API and reports internal status counts separately from externally observed `/cmd_vel_docking`, `/cmd_vel_safe`, `/cmd_vel`, reverse-enable, and odometry evidence. This is observability only: no speed, timeout, Nav2 reverse, pointcloud, DDS/RMW, EKF, FAST-LIO2, or Ranger CAN behavior is changed.

Before any normal Nav2 goal is sent, `robot_api_server` waits for fresh `map -> odom`, `odom -> base_link`, and composed `map -> base_link` TF samples. If the localization bridge has accepted a new result but the TF stream has not caught up yet, the API returns a TF-not-ready admission failure rather than letting Nav2 immediately abort on future extrapolation.

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

`run_navigation_runtime_services.sh` resolves the selected floor assets, starts the resident occupancy-localization layer, starts or reuses the floor-manager process, then sends one `/global_localization/trigger` request. The wrapper calls Isaac's direct grid-search service but startup success is judged by `robot_localization_bridge` accepting the result, `/localization/bridge_status.has_map_to_odom=true`, and a live `map -> odom` TF owned by `robot_localization_bridge`. Resident startup trusts the wrapper's `map->odom ready owner=robot_localization_bridge` success detail and then still performs a live `map -> odom` TF check; if that strong detail is absent, it falls back to the older `/localization/bridge_status` wait. It no longer rejects triggered startup only because `/localization_result.header.stamp` is a few seconds older than receive time; triggered mode uses `triggered_max_result_age_ms` and the original stamp for historical TF lookup. The runtime context records `failure_code`, `localization_mode`, `last_triggered_relocalization_ok`, and `map_to_odom_age_ms` when available. It marks the runtime context ready only after Nav2 lifecycle activation and the global costmap are available. `run_nav2_navigation.sh` still starts or reuses floor-manager, robot_safety, ranger mode controller, and local perception by process ownership only, then launches the repository-owned standard Nav2 stack without blocking on `/safety/status`, `/perception/obstacle_points`, or local-costmap observation probes.

If `NJRH_AMCL_LOCALIZATION_MODE=shadow` or `gated`, the resident navigation runtime starts `run_amcl_shadow_localization.sh` only after the initial Isaac triggered relocalization has passed the bridge and `map -> odom` gates. AMCL is not part of the Nav2 controller/planner lifecycle and does not publish TF. The runner activates AMCL through the standard `/amcl/change_state` lifecycle service and still requires `/amcl` to report `active`. Startup waits for `/map`, `/scan`, `map -> odom`, `odom -> base_link`, and `base_link -> scan_frame`, then warms AMCL's process-local TF buffer before seeding `/initialpose` through `/robot_localization_bridge/seed_amcl_initial_pose`. The runner starts `/scan_amcl` only after seed and TF warmup; `/scan_amcl` is an AMCL production admission input derived from `/scan`, preserves the original stamp/frame/ranges, drops stale or non-TF-transformable scans, and defaults to 5 Hz. AMCL readiness requires seed success, fresh `/amcl_pose`, and healthy scan admission. `shadow` reports bridge candidates only, while `gated` accepts only small corrections. If AMCL is not ready, navigation continues on Isaac triggered plus odom baseline and exposes AMCL as WARN/not-ready rather than claiming continuous correction. Navigation stop calls the AMCL stop helper, which also stops the scan admission relay, before stopping the rest of the navigation stack.

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
