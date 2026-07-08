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
- `robot_local_state` canonical local odom owner (`LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_only` by default: `/wheel/odom_ekf` -> EKF -> `/local_state/odometry`; `/lidar_imu_bias_corrected` remains resident for safety-side spin-tail detection, not EKF fusion; FAST-LIO local-state remains an explicit diagnostic/mapping-aligned mode)
- `robot_safety`
- `ranger_mini3_mode_controller`
- `robot_floor_manager`
- `robot_api_server`

`ranger_mini3_mode_controller` remains resident after `robot_safety`, but it is
now official-passthrough-only. It preserves normal `/cmd_vel_safe` Twist values
for official AgileX `ranger_base_node` interpretation, while still protecting
timeout/startup/park zero behavior, reverse permits for docking and teleop,
normal-navigation lateral rejection, desired/actual `motion_mode` diagnostics,
and status publication. The legacy custom Ackermann shaping path and its A/B
profile switch have been removed so stale `mode_controller_profile=custom`
settings cannot reintroduce the wrong model.

`robot_api_server` is supervised inside the common-service layer. If the API process exits, `run_robot_api_server_supervised.sh` restarts it after a short delay. Before each restart, the supervisor clears stale orphan API processes so only one `robot_api_server_node` can own port `8080` and the fixed HTTP worker pool. `njrh_container.sh start-runtime` and `start-common` now also require `GET /api/v1/status` on port `8080` to become healthy before reporting common services as ready. That host-side HTTP wait defaults to `NJRH_ROBOT_API_READY_TIMEOUT_SEC=120` with `NJRH_ROBOT_API_READY_POLL_SEC=1`; it does not create ROS readiness participants. The API process uses a fixed HTTP worker pool controlled by `max_http_connections` and returns `503` when overloaded instead of creating unbounded detached request threads.

Canonical helper startup has a bounded failure path. During cold boot,
`robot_local_state_common` may briefly have its EKF process alive before ROS
graph discovery and the fresh `odom -> base_link` probe both pass. If the first
startup readiness check misses that window, `canonical_tf_helpers.sh` performs
one final readiness recheck before treating the helper as failed. A real helper
failure is stopped with bounded `SIGINT`, then `SIGTERM`, then a scoped kill of
that helper process tree. The common-service wrapper must not block forever in
`wait` on a helper that ignored `SIGINT`, because that prevents `robot_safety`,
the Ranger mode controller, docking manager, and `robot_api_server` from
starting.

The API server treats `/tf` as a process-level localization input, not as page-scoped telemetry. It creates the `/tf` subscription once at startup and keeps it resident so high-rate `/api/v1/robot/pose` polling cannot repeatedly add and remove reliable Fast DDS endpoints on `/tf`; that endpoint churn can backpressure `robot_localization_bridge` while the bridge process still appears alive.

`robot_localization_bridge` now separates accepted global corrections from the
fixed `map -> odom` publisher. Isaac trigger results, AMCL gated candidates, and
manual force-accept relocalization only update an internal `MapOdomState`; the
independent 50 Hz publisher callback group is the only code path that broadcasts
`map -> odom`. This keeps AMCL seed work, runtime status-file reads, service
callbacks, and correction logging off the transform broadcast path. The bridge
status topic exposes `map_odom_publish_loop_hz`,
`map_odom_publish_gap_ms`, accepted/published state sequence, pause/frozen
state, and `publisher_decoupled_from_correction=true`. The API
post-relocalization settle barrier refuses to release the next Nav2 goal or GS2
fine-docking stage until that publisher heartbeat is stable, the accepted state
has been published, `odom -> base_link` is fresh, local costmap has updated, and
no new local-costmap MessageFilter drop was seen. This is a runtime readiness
barrier; it does not change `transform_tolerance`, `max_odom_tf_age_ms`, Nav2
plugins, PointCloud2 QoS/DDS, FAST-LIO2, Ranger odom, or EKF fusion.

Phase V1 validation tools sit outside that runtime state machine. They observe
the D3/N2/R0-R2 contracts and write field reports, but the default combined
command does not start Nav2 goals, docking requests, relocalization triggers, or
velocity commands:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_v1_navigation_docking_validation.sh \
  --observe-only \
  --duration-sec 120
```

Manual relocalization verification and the small predock yaw probe are explicit
operator actions, not automatic startup or goal-admission gates.

Resident navigation startup activates the selected-floor localization
`/map_server` with an external `nav2_util/lifecycle_bringup map_server` helper
in the production non-legacy pointcloud profile. The launch file can still start
`lifecycle_manager_map` for compatibility, but the field runner disables it for
this path so a slow or dropped lifecycle `change_state` response cannot leave a
loaded map server inactive. During that external transition the shell readiness
helper only observes active state or the selected `/map` publication; it does
not send competing lifecycle transitions.

Before resident navigation autostart, common services now require the resident
wrapper process, the runtime context, and API goal-start safety to agree before
reusing an existing navigation runtime. If the wrapper is gone, the context is
not confirmed ready, or `/api/v1/status` reports `safe_for_goal_start=false`,
common services clear stale resident Nav2, localization, AMCL, and context
state before starting a new resident runtime. The same resident-layer sweep runs
from common-service cleanup before the common wrapper exits, so a systemd
timeout or restart cannot stack a new startup attempt on top of orphaned
controller, bridge, AMCL, or scan-admission processes.
`NJRH_RESIDENT_NAVIGATION_READY_TIMEOUT_SEC=120` is the soft startup SLA report
point. It is not the process-kill boundary while the resident runtime is still
progressing; `NJRH_RESIDENT_NAVIGATION_READY_HARD_TIMEOUT_SEC` is the hard
cleanup timeout. This prevents a few seconds of Nav2 activation or AMCL
readiness overrun from creating a self-inflicted restart loop.

The API server's BMS charging-contact policy lives in `robot_api_server/bms_contact.hpp` and `src/bms_contact.cpp`, separate from HTTP routing. UTC timestamp formatting and generated map/current-pose IDs live in `robot_api_server/api_time_utils.hpp` and `src/api_time_utils.cpp`. Docking job state formatting lives in `robot_api_server/docking_job_model.hpp` and `src/docking_job_model.cpp`; docking/undocking status string classification lives in `robot_api_server/docking_status_utils.hpp` and `src/docking_status_utils.cpp`. Common text/binary file reads and writes, PGM output, and map YAML image-file rewrites live in `robot_api_server/file_utils.hpp` and `src/file_utils.cpp`. Floor asset completeness checks, active `current/` selection, `poses.yaml` fallback, and stored pose lookup live in `robot_api_server/floor_asset_resolver.hpp` and `src/floor_asset_resolver.cpp`. HTTP request/response structs, WebSocket accept-key helpers, and lightweight JSON parsing live in `robot_api_server/http_common.hpp` and `src/http_common.cpp`; localization result snapshots and relocalization diagnostic text live in `robot_api_server/localization_result_model.hpp` and `src/localization_result_model.cpp`. Map/pose models plus safe ID/name validation live in `robot_api_server/storage_models.hpp` and `src/storage_models.cpp`; grayscale PNG encoding, PGM dimension reads, and Nav map YAML metadata extraction live in `robot_api_server/map_asset_io.hpp` and `src/map_asset_io.cpp`; OccupancyGrid-to-image conversion, saved map YAML text, neutral costmap filter assets, and asset reports live in `robot_api_server/map_asset_writer.hpp` and `src/map_asset_writer.cpp`; released map directory paths, manifest traversal, and active/name/id lookup live in `robot_api_server/map_catalog.hpp` and `src/map_catalog.cpp`; `MapManifest` path derivation and `manifest.json` read/write formatting live in `robot_api_server/map_manifest_io.hpp` and `src/map_manifest_io.cpp`; navigation-cancel job state formatting lives in `robot_api_server/navigation_cancel_job_model.hpp` and `src/navigation_cancel_job_model.cpp`; `poses.yaml` parsing/writing lives in `robot_api_server/poses_io.hpp` and `src/poses_io.cpp`; runtime map context file formatting lives in `robot_api_server/runtime_map_context_io.hpp` and `src/runtime_map_context_io.cpp`; saved 2D PNG lookup and runtime flat-map companion file paths live in `robot_api_server/runtime_map_lookup.hpp` and `src/runtime_map_lookup.cpp`; robot pose snapshots and `/api/v1/robot/pose` response payloads live in `robot_api_server/robot_pose_model.hpp` and `src/robot_pose_model.cpp`; Linux child-process setup and pid/pgid helpers live in `robot_api_server/runtime_process_utils.hpp` and `src/runtime_process_utils.cpp`; keepout semantic JSON paths and response fragments live in `robot_api_server/semantic_layer_io.hpp` and `src/semantic_layer_io.cpp`; App subscription request parsing lives in `robot_api_server/subscription_api.hpp` and `src/subscription_api.cpp`; App page-scoped subscription leases and TTL expiry live in `robot_api_server/subscription_manager.hpp` and `src/subscription_manager.cpp`; frame ID normalization, yaw extraction, angle wrapping, and ROS timestamp helpers live in `robot_api_server/tf_pose_utils.hpp` and `src/tf_pose_utils.cpp`. These pure modules keep full-SOC dock-contact inference, timestamp/ID generation, docking job state formatting, docking status classification, file asset IO, floor asset resolution, gateway parsing, localization result diagnostics, map asset validation, saved-map asset generation, manifest catalog lookup, navigation-cancel state formatting, saved PNG lookup, robot pose payload formatting, semantic layer formatting, TF pose math, runtime process helpers, and subscription lease behavior testable without touching navigation, docking, or socket-loop code.

The Web dashboard is not part of the production runtime. It is only a manual observation/debug window.

Jetson CPU affinity is now a runtime policy, not a launch-file assumption. The default policy lives in `scripts/jetson/runtime_overlay/config/cpu_affinity.env` and can be disabled with:

```bash
NJRH_CPU_AFFINITY_ENABLED=false
```

The default 8-core split reserves CPU0 for lightweight API/map-filter work plus bursty Nav2 planner/BT work, CPU0/CPU1 for Nav2 lifecycle supervision, CPU1 for base/safety command arbitration, CPU2 for EKF local state, CPU3 for Nav2 controller/collision work, CPU4 for the JT128 UDP driver, CPU5 for full-density pointcloud remap ingress, CPU6 for IMU remap plus AMCL scan/localization helpers, and CPU7 for the `map -> odom` bridge plus mapping backend work. FAST-LIO2 is not resident during normal navigation; live 2D mapping re-derives its mode-local CPU sets at launch so FAST-LIO2 frontend/deskew defaults to CPU7 only while mapping is active and PGO/slam_toolbox mapping defaults to CPU7. During live 2D mapping, `run_projected_map.sh` also applies a temporary LiDAR NIC RPS/XPS profile (`NJRH_SLAM2D_LIDAR_RPS_XPS_INTERFACE=eth1`, `NJRH_SLAM2D_LIDAR_RPS_XPS_CPUSET=5` by default) and restores the previous queue masks on exit. `robot_api_server` also restores the same `mapping_lidar_rps_xps_state_dir` state on mapping stop/save, because API process-group termination can bypass the shell EXIT trap. It does not write IRQ affinity; the current Jetson eth1 IRQ rejects affinity writes, while RPS/XPS is sufficient to recover `/lidar_points` to the JT128 target cadence in mapping mode. Future arm control/planning should use CPU6/CPU7 only when mapping is not active. Existing processes can be retagged without restarting motion by running the helper below. It applies affinity to every Linux thread under `/proc/<pid>/task`, not only the process leader, because ROS 2 executors and DDS workers are multithreaded. The helper does not source `common_env.sh` and does not initialize ROS/DDS, so it cannot create extra participants or rewrite DDS profiles during a live navigation run:

```bash
bash scripts/jetson/runtime_overlay/scripts/apply_cpu_affinity.sh
```

AMCL scan admission is part of the localization CPU budget. `NJRH_CPUSET_AMCL`
and `NJRH_CPUSET_AMCL_SCAN_ADMISSION` default to CPU6 through
`NJRH_CPUSET_LOCALIZATION`. Phase A1.4 keeps AMCL itself unchanged and starts
the C++ `robot_localization_bridge/amcl_scan_admission_node` by default through
`NJRH_AMCL_SCAN_ADMISSION_IMPL=cpp`. The node is launched with
`taskset -c ${NJRH_CPUSET_AMCL_SCAN_ADMISSION}` and startup fails if Linux
reports a different `Cpus_allowed_list`. This keeps `/scan -> /scan_amcl`
admission off EKF CPU2, Nav2 controller CPU3, and `robot_localization_bridge`
CPU7 without changing AMCL TF tolerance, bridge future-stamp gates, Nav2
controller/planner plugins, PointCloud2 QoS, DDS, Ranger odom, or EKF fusion.
The Python relay remains a rollback-only path selected explicitly with
`NJRH_AMCL_SCAN_ADMISSION_IMPL=python`; a missing C++ binary is a hard startup
failure, not a silent fallback. Use:

AMCL runtime readiness is a resident heartbeat, not a one-shot startup stamp.
`run_amcl_shadow_localization.sh --complete-readiness` seeds and validates the
resident AMCL path, then `run_navigation_runtime_services.sh` keeps
`run_amcl_shadow_localization.sh --heartbeat` alive while the resident
navigation runtime is alive. The heartbeat refreshes
`/tmp/njrh_amcl_runtime_status.env` from the AMCL and scan-admission PIDs plus
the accepted seed/static-standby state, so `robot_localization_bridge` and
`robot_api_server` do not mark AMCL stale a few seconds after cold start.

```bash
bash scripts/jetson/runtime_overlay/scripts/inspect_runtime_cpu_affinity.sh
bash scripts/jetson/runtime_overlay/scripts/observe_navigation_tf_jitter_180s.sh --duration-sec 180 --label nav_tf_jitter
```

Hardware validation still needs a loaded navigation run after a full restart to confirm `taskset -pc <pid>` matches the intended service groups: JT128 driver on CPU4, standalone pointcloud remap on CPU5, IMU remap/localization helpers on CPU6, `robot_localization_bridge` on CPU7, planner/BT on CPU0, and both Nav2 lifecycle managers on CPU0/CPU1. Also confirm no `fastlio_mapping` process is resident during navigation, `/lidar_points` remains the only high-density trunk, `/scan` and `/flatscan` are live, old `/perception/obstacle_points` and `/perception/clearing_points` publishers are zero, the filter lifecycle manager reaches active before the core navigation lifecycle manager, and `/scan` is subscribed by local costmap plus collision monitor. Use `verify_pointcloud_rates.sh`, `verify_lidar_trunk_jitter.sh`, `diagnose_lidar_points_jitter.sh`, `diagnose_nav_scan_pipeline.sh`, `diagnose_pointcloud_cpu_pressure.sh`, `run_pointcloud_cpu_affinity_ab.sh --print`, `check_runtime_process_freshness.sh`, `inspect_pointcloud_subscribers.sh`, `verify_pointcloud_delivery_matrix.sh`, `inspect_pointcloud_cpu_affinity.sh`, `record_pointcloud_nav_acceptance.sh --duration-sec 1200`, `run_pointcloud_dds_transport_ab.sh`, and `run_lidar_trunk_pure_ab.sh --execute` as manual verification tools, not background monitors.

Phase C1 adds a controller/local-costmap CPU-set A/B profile for the specific
case where external `/tf` still publishes at the expected rate but
`controller_server` reports a stale in-process TF buffer. The default
`NJRH_NAV2_CONTROLLER_CPU_PROFILE=current` keeps `controller_server` on CPU3.
`control_wide` sets only `controller_server` to CPU3,5, matching the fact that
Nav2 hosts the local costmap inside that process. Startup logs the selected
profile, CPU set, and PID, then fails if `/proc/<pid>/status` or any controller
thread does not match the expected CPU set. EKF/local-state CPU2, JT128 CPU4,
AMCL scan admission CPU6, and `robot_localization_bridge` CPU7 remain reserved.
Run the A/B as:

```bash
export NJRH_NAV2_CONTROLLER_CPU_PROFILE=current
bash scripts/jetson/runtime_overlay/scripts/run_nav2_controller_cpu_ab.sh \
  --profile current --duration-sec 180 --apply --restart-nav2

export NJRH_NAV2_CONTROLLER_CPU_PROFILE=control_wide
bash scripts/jetson/runtime_overlay/scripts/run_nav2_controller_cpu_ab.sh \
  --profile control_wide --duration-sec 180 --apply --restart-nav2
```

The reports compare controller requested/latest TF lag, local-costmap
MessageFilter drops, `map -> odom` and `odom -> base_link` gaps, command-chain
activity, and controller thread placement. This phase deliberately does not
change `transform_tolerance`, `max_odom_tf_age_ms`, Nav2 plugins, MPPI/progress
checker parameters, pointcloud QoS/DDS, FAST-LIO2, Ranger odom, or EKF fusion.
Rollback is `export NJRH_NAV2_CONTROLLER_CPU_PROFILE=current` followed by a
Nav2 restart.

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
entrypoint without changing the canonical trunk. `NJRH_POINTCLOUD_ACCEL_PROFILE`
now defaults to `ipc_worker`; `legacy` is removed from production. `ipc_worker`
starts `pointcloud_accel_axis_node` or the driver-integrated accel node as the
single `/lidar_points` publisher; its callback publishes full-density/full-fields
`/lidar_points` and updates a latest normalized buffer before returning. The
scan worker derives `/scan`, and `laser_scan_to_flatscan` converts `/scan` to
`/flatscan`. Nav2 local costmap and `collision_monitor` consume `/scan` for
standard LaserScan marking and clearing. The old custom PointCloud2 obstacle
topics `/perception/obstacle_points` and `/perception/clearing_points` are not
published or consumed in production. `nitros` is a guarded skeleton for future
navigation-branch acceleration only and must not replace `/lidar_points` or
FAST-LIO2 mapping input.

Phase 1.15 treats `/flatscan` as a supervised localization-startup dependency,
not just a side effect of the scan chain. In `ipc_worker`, the accel core owns
`/scan`, while a bounded-restart standalone `laser_scan_to_flatscan`
compatibility helper owns `/flatscan`. Helper state is written to
`flatscan_helper_status.env` for `verify_pointcloud_accel_profile.sh` and
`run_pointcloud_accel_ab.sh`. If `/scan` has a publisher but `/flatscan` does
not, diagnostics report `CASE_FLATSCAN_HELPER_DEAD`. This does not add topics or
change PointCloud2 QoS, DDS/RMW, timestamp policy, FAST-LIO2, EKF, App API, or
Nav2 controller/planner behavior.
Navigation stop and localization cleanup must preserve the common
`laser_scan_to_flatscan` helper. If an older manual mode transition or field
diagnostic still leaves `/scan` alive but `/flatscan` missing, resident
navigation startup restarts the current pointcloud accel profile once and waits
for `/flatscan` again before reporting `FLATSCAN_MISSING`. Common startup does
not duplicate that `/flatscan` gate by default; use
`NJRH_COMMON_REQUIRE_FLATSCAN_BEFORE_RESIDENT_AUTOSTART=true` only when isolating
the pointcloud helper before resident navigation is allowed to start.
Non-legacy common runtime startup also treats duplicate
`run_pointcloud_accel_pipeline.sh` or `laser_scan_to_flatscan` processes as
stale ownership and stops that pointcloud profile with targeted SIGINT/SIGTERM
before starting one replacement chain.

The local-state startup gate requires a fresh `odom -> base_link` TF before
common services continue to safety, mode control, API, and resident navigation.
If the C++ readiness probe prints success but does not exit because of an RMW
shutdown hang, the shell wrapper accepts that already-observed success only
after a bounded process timeout. This does not change the TF freshness threshold.

Phase 2.4a/2.4b and the later virtual-clearing path are superseded for
production local obstacles. Local-costmap obstacle admission now uses `/scan`
LaserScan only; standard Nav2 raytracing clears free space with
`inf_is_valid=true`. The old `/perception/obstacle_points` and
`/perception/clearing_points` topics must have zero production publishers and
zero Nav2/collision-monitor subscribers. The read-only legacy timestamp scripts
may still inspect old topics during cleanup, but they are not startup gates and
must not be used to restore the retired PointCloud2 obstacle path.
The production local-obstacle `/scan` worker uses a `lidar_level_link`
height slice of `-0.50m..0.35m`; this keeps the scan close enough to detect
low obstacles while reducing near-ground returns that can keep local-costmap
obstacles refreshed after a moved object leaves.
The read-only `verify_local_costmap_observation_timestamp_root_cause.sh` script
classifies raw-stamp, internal-buffer, output-stamp, TF-cache, startup-warm-up,
and frame
mismatch cases and writes a markdown report. The audit script remains read-only
and does not change `tf_filter_tolerance`, observation persistence,
pointcloud QoS, DDS/RMW, EKF, FAST-LIO2, App API, or Nav2 controller/planner
behavior.
The read-only `observe_local_costmap_scan_clearing.sh` script records `/scan`,
`/local_costmap/costmap`, and TF during a moved-obstacle reproduction. It does
not clear costmaps or mutate runtime state; it classifies occupied local cells
as current-scan endpoints, cells blocked behind current endpoints, or cells that
current clearing rays should have cleared.

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
Production navigation does not derive local obstacle PointCloud2 branches.
`/_internal/lidar_points_local`, `/lidar_points_nav`, `/points_nav`,
`/perception/obstacle_points`, and `/perception/clearing_points` are not
production outputs. `/scan` is the local obstacle observation output and is
derived by the accel scan worker from the same in-process normalized buffer.
Debug commands, RViz/Foxglove views, and rosbag captures should not stay
attached to `/lidar_points`; use status topics or `/scan` where possible, and
require explicit env flags for bagging high-density clouds.

For accel-profile validation, use `set_pointcloud_accel_profile.sh`,
`verify_pointcloud_accel_profile.sh`, and `run_pointcloud_accel_ab.sh`. The
verify script reports requested/resolved profile, trunk/scan/flatscan owners,
old `/perception/*` publisher counts, `/lidar_points` publisher count, Nav2
`/scan` subscribers, status topics, internal zero-copy counters, FAST-LIO2
residuals, and final owner-contract summary flags. A valid `ipc_worker` run
keeps `/lidar_points` publisher count at one, keeps the trunk near vendor rate
with full fields, leaves FAST-LIO2 non-resident during normal navigation, feeds
Nav2 through `/scan`, and reports zero publishers for
`/perception/obstacle_points` and `/perception/clearing_points`.

`robot_local_perception` is not a production local-obstacle runtime. Its runtime
script exits intentionally so stale profiles cannot reintroduce custom
PointCloud2 obstacle or clearing topics.

Pointcloud source and localization-preprocessor diagnostics are also emitted from C++ nodes. `pointcloud_axis_remap` publishes `/lidar/axis_remap_status` at 1 Hz with raw input rate, `/lidar_points` publish rate, raw callback inter-arrival, `/lidar_points` publish interval and gap counters, raw callback duration, trunk/branch/total publish timing, cloud size, output subscriber count, branch flags, branch publish/skip/attempt rates, branch subscriber counts, branch last points/bytes/duration, and skipped-cloud count. The patched `nav_cloud_preprocessor` publishes `/lidar/nav_cloud_preprocessor_status` at 1 Hz with input callback/accept rate, `/points_nav` output rate, TF/empty/filter-empty skips, input inter-arrival, input stamp/message age, processing average/max, input/output point counts, lookup timeout, source/target frame, range/height filters, matched publisher/subscriber counts, and QoS. These status topics are the preferred field inputs for `diagnose_lidar_points_jitter.sh`, `diagnose_local_perception_pipeline.sh`, `diagnose_nav_scan_pipeline.sh`, `verify_lidar_trunk_jitter.sh`, and `verify_pointcloud_delivery_matrix.sh`; use direct `ros2 topic hz` on full-density clouds only when a status topic is unavailable or when deliberately comparing subscriber-side delivery with `--include-cli-hz` or a manual `NJRH_VERIFY_MATRIX_LIDAR_POINTS_CLI_HZ` value for CASE_G.

The default navigation local-state mode is wheel-only EKF because the current Ranger field baseline keeps `/local_state/odometry` anchored to the official wheel odom while IMU fusion remains a controlled diagnostic profile. The corrected IMU helper still runs by default so `robot_safety` can wait for physical yaw-rate settle after pure spin. When FAST-LIO local-state mode is explicitly selected, `/Odometry` can arrive behind wall time because the lidar-inertial frontend has processing latency; that latency must be diagnosed at the producer and transport layers, not hidden by changing local-obstacle stamps. The local costmap rolling window runs in `odom` so controller progress checks see a robot pose that changes with motion; `robot_base_frame` remains `base_link`, and the LaserScan source uses `sensor_frame=lidar_level_link`. Do not use `base_link` as the local costmap `global_frame`: it makes the pose returned to controller-side progress checking nearly constant in the robot frame and can cause false `Failed to make progress` aborts. Startup-time costmap MessageFilter drops are kept as diagnostics, not as shell-level startup blockers.

Canonical local-state is owned by common services, not by Nav2 startup. In the default EKF mode, common services start the wheel-odom preprocessor, IMU bias filter, and `robot_localization` EKF process; wheel-only means the EKF params omit `imu0`, not that the corrected IMU topic disappears. Reuse now requires both the expected process set and the live ROS endpoint: `/robot_local_state`, `/local_state/odometry`, and a fresh `odom -> base_link` TF. In explicit FAST-LIO local-state mode the common layer requires both `fastlio_odom_bridge_node` and `robot_local_state/local_state_node` plus the same output endpoint/TF. If a mode switch leaves an EKF process alive but not visible in the ROS graph, common services restart only the local-state helper before Nav2 starts. Runtime graph-probe misses under Nav2 startup load must not cause the canonical `odom -> base_link` owner to kill itself after it has passed this bounded startup/reuse gate.

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

Nav2 preflight is process-first. `run_nav2_navigation.sh` starts or reuses floor-manager, `robot_safety`, and Ranger mode-controller by process ownership, then launches Nav2. It does not run local-state, TF, `/safety/status`, obstacle-cloud, or local-costmap probes as startup gates, and it does not start local perception just to warm a new costmap buffer.

The occupancy localization mode may start or reuse localization-specific services such as `robot_localization_bridge` and `robot_global_localization`, but it does not own FAST-LIO2 or canonical local-state. It consumes `/lidar_points` for stationary Isaac relocalization and checks only that the resident local-state process for the selected mode exists before launching the localization stack; FAST-LIO2 is required only for explicit `NJRH_NAV_LOCAL_STATE_MODE=fastlio`. On localization or Nav2 startup failure the mode script cleans the localization stack and overlay helpers only; `robot_local_state` remains a common/canonical service. This prevents Nav2 from staying active while `/local_state/odometry` and `/tf` publishers disappear during a stop/resume race.

Navigation resume gives the localization layer a short settle window (`NJRH_NAV_LOCALIZATION_START_SETTLE_SEC`, default 0.1 seconds), then runs a bounded deterministic startup chain before Nav2 is allowed to become ready. The critical path does not start resident AMCL before the first map correction. Instead, the chain waits for `/global_localization/trigger`, Isaac `/trigger_grid_search_localization`, explicitly drives `/map_server` to active for the selected floor asset, requires the selected `/map` to be observable, requires the `laser_scan_to_flatscan` publisher on `/flatscan`, sends one `/global_localization/trigger`, then requires bridge-accepted localization and live `map -> odom`. The `/localization_result` publisher pre-gate is optional through `NJRH_INITIAL_LOCALIZATION_REQUIRE_RESULT_PUBLISHER=true`; it is not the default because the wrapper service already waits for the actual result and bridge acceptance. `/flatscan` is `isaac_ros_pointcloud_interfaces/msg/FlatScan`, not `sensor_msgs/msg/LaserScan`; `/scan` is the LaserScan intermediate. The FlatScan gate checks publisher ownership rather than consuming a generic FlatScan message because the standard ROS CLI/generic probe path is unreliable for this Isaac interface on the Jetson runtime. If this gate fails, `run_navigation_runtime_services.sh` now records the specific reason `FLATSCAN_MISSING` and logs `/scan` publisher presence, the `laser_scan_to_flatscan` process state, the active pointcloud accel profile, and the verify/restart command. Other admission failures are split into `GLOBAL_LOCALIZATION_TRIGGER_SERVICE_MISSING`, `GRID_SEARCH_LOCALIZATION_SERVICE_MISSING`, `MAP_SERVER_NOT_ACTIVE`, `MAP_TOPIC_MISSING`, and `LOCALIZATION_RESULT_PUBLISHER_MISSING` only when that optional result-publisher gate is explicitly enabled. This is the required localization sequence, not a high-frequency Python/rclpy graph watchdog.

The local costmap `MessageFilter` drops observed on `/scan` are separate from
`/flatscan` startup admission. They require a producer/TF timing audit and must
not be hidden by restamping or mixed into the flatscan lifecycle fix.

Standard Nav2 startup separates filter lifecycle from core navigation lifecycle. `lifecycle_manager_costmap_filters` owns only the keepout/speed mask map servers and filter-info servers. The production resident cold-start path first prepares localization, triggers the bridge-owned `map -> odom` baseline, and only then launches Nav2. `NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=true` remains available only as an A/B optimization switch. After the trigger result is accepted and bridge-owned `map -> odom` is live, resident runtime starts the core lifecycle nodes through Nav2's lifecycle helper with a bounded runtime timeout. The default keeps deterministic TF ordering without changing Nav2 controller/planner plugins, TF tolerances, pointcloud QoS, FAST-LIO2, Ranger odom, or EKF policy.

The occupancy localization startup no longer waits on `/lidar_points` from shell. It launches the localization stack and leaves pointcloud freshness to explicit diagnostics, Isaac/localizer logs, and API goal admission. This prevents a manual navigation stop or short common-service recovery window from causing the resident navigation runtime to exit before Isaac localization and Nav2 are even launched.

Mode services are allowed to start and stop when switching between navigation and mapping:

- Navigation: Isaac localization stack, `robot_localization_bridge`, Nav2, velocity smoother, collision monitor.
- Mapping: mapping-owned FAST-LIO2, optional PGO, slam_toolbox 2D mapping, scan slicing helpers.
- Docking: `robot_docking_manager` is a common resident service, started by `run_common_services.sh` after `robot_safety` and `ranger_mini3_mode_controller`. It stays idle until `/docking/start` or `/docking/undock` is called, keeping `/docking/start`, `/docking/stop`, and `/docking/undock` discoverable before API return-to-dock jobs reach the fine-docking handoff.
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

Navigation resume scripts do not restore broad ROS graph/topic/TF probe loops, but they do keep the critical localization startup chain as a gate. `run_navigation_runtime_services.sh` starts the selected-floor localization layer, verifies the initial localizer inputs/services, sends one bounded global-localization trigger request, waits for bridge-accepted localization and `map -> odom`, starts Nav2, then marks the runtime context ready only after Nav2 lifecycle activation and the global costmap are available. The runtime context identity comes from the selected floor assets: `resolve_floor_assets` reads `asset_report.json`, exports `NJRH_NAV_MAP_ID`, and mirrors it into `NJRH_MAP_ID` so `/api/v1/robot/pose` can attach the confirmed `building_id` / `floor_id` / `map_id` instead of rejecting an otherwise fresh TF pose. Explicit readiness tools still exist for field diagnostics, but a transient Fast DDS discovery miss outside this critical chain must not keep the App in `starting`. The API server also polls the navigation resume child process while serving `/api/v1/status` and `/api/v1/navigation/state`; if the child process exits during startup or the runtime context records `state=failed`, the API reports navigation `failed` with the resume log path instead of leaving the App in `starting`.

The occupancy localization bridge watchdog follows the same process-first rule. It still rejects a real `robot_localization_bridge` process loss, but ROS graph probe misses during Nav2/map-server activation are diagnostics only while the bridge process is still alive.

Local perception does not run in production navigation. `run_nav2_navigation.sh` no longer starts local perception, primes a probe-owned TF buffer, or blocks on local-costmap observation checks before launching Nav2. Nav2 builds its own costmap/TF buffers during normal lifecycle activation. Hardware validation should inspect `/local_costmap/costmap`, `/scan` subscriber topology, old `/perception/*` publisher counts, and TF logs after startup, but those checks are diagnostic rather than startup blockers.

Nav2's behavior tree is a coordinator, not the low-level controller loop. The field runtime uses `bt_loop_duration=50`, `default_server_timeout=1000`, and `wait_for_service_timeout=2000` so planner action acknowledgements, costmap clear services, and recovery behavior action servers are not falsely timed out by short Jetson CPU/DDS bursts. MPPI includes `VelocityDeadbandCritic` with `deadband_velocities=[0.025, 0.0, 0.025]` and `cost_weight=90.0` to model the Ranger Mini 3 low-speed command deadband before commands reach the safety chain. The Ranger-matched MPPI profile keeps `vx_max=1.20` and `wz_max=0.70`, but uses a 4.0 s horizon and lower sampling noise (`vx_std=0.35`, `wz_std=0.38`); `velocity_smoother` owns the effective accel/decel limits (`max_accel=[0.55, 0.0, 0.90]`, `max_decel=[-0.95, 0.0, -1.10]`) so the command chain does not demand snap acceleration the chassis will not track. RotationShim owns large path-entry heading above `0.45rad` and disengages below `0.075rad`; residual spin tail is handled downstream by `robot_safety` using actual `/wheel/odom` yaw-rate settle before the next linear segment. Near-goal `GoalCritic`/`GoalAngleCritic` weights (`16.0`/`6.0`) enter at `1.5m` and keep terminal retries from selecting mathematically nonzero but physically ineffective tiny commands while biasing final convergence toward XY before heading closure. API normal navigation publishes `/speed_limit` from target distance to avoid carrying 1.2 m/s into the measured 0.58 m stop-distance zone; when a Nav2 task exits it restores the cruise limit instead of publishing `speed_limit=0`, which controller-server interprets as a stop limit. Ordinary Nav2 reverse is limited to terminal correction (`vx_min=-0.08`, `min_velocity=[-0.08, 0.0, -1.00]`) and guarded by `PreferForwardCritic` (`cost_weight=20.0`, `threshold_to_consider=0.50`) so long-range tracking remains forward-biased while MPPI can recover small forward overshoot at the goal. `PathFollowCritic.cost_weight=7.0` is intentionally higher than the initial terminal-reverse profile after report `20260629T085207Z` showed a long goal could otherwise fall into a low-speed local optimum and abort without making path progress. `TwirlingCritic.cost_weight=1.0` must use the live Humble key names (`cost_power`, `cost_weight`) rather than the older `twirling_cost_*` names, otherwise the plugin default can suppress large initial heading corrections. The Savitzky-Golay BT path smoother and BT wait/retry recovery were rolled back after the delivery-point loop showed they could introduce a mid-route pure-spin/replan chain. The active field BT now runs direct `ComputePathToPose -> FollowPath` / `ComputePathThroughPoses -> FollowPath`, while `controller_server.failure_tolerance=3.0` absorbs only very short no-trajectory intervals. Motion safety remains downstream in the controller, velocity smoother, collision monitor, and `robot_safety`.

The active repository navigation trees no longer run BT `Wait` recovery in the field path; blocked-corridor wait/retry policy should be explicit at the API/mission layer rather than hidden in the point-navigation BT.

The 2026-06-29 field validation used `record_navigation_goal_diagnostic.sh` for `delivery_675235` and the return to `delivery_512355`. The outbound run completed with `nav2_result_code=4`, `final_distance_m=0.042184`, and `final_yaw_error_rad=0.012823` without retry. The return completed with `nav2_result_code=4`, `final_distance_m=0.021524`, and `final_yaw_error_rad=0.008177` after one same-goal Nav2 retry. `/cmd_vel_nav_raw`, `/cmd_vel_nav`, `/cmd_vel_collision_checked`, and final `/cmd_vel` all showed the bounded terminal reverse path on the return, clamped at `-0.08m/s`; `/cmd_vel_api` stayed zero. AMCL accepted no bridge corrections during either run, so the result is attributable to the Nav2 MPPI/safety command path rather than continuous localization correction.

Extended same-day validation found one counterexample before the final weight adjustment: report `20260629T085207Z` sent `delivery_675235`, but FollowPath aborted with `nav2_result_code=6`, the robot remained about `10.93m` from the target, and `/cmd_vel_nav_raw` averaged only `0.009m/s`. The fix kept `vx_min=-0.08` but raised `PathFollowCritic.cost_weight` from `4.5` to `7.0` and `PreferForwardCritic.cost_weight` from `8.0` to `20.0`. After a full `njrh-runtime.service` restart, `20260629T090531Z` returned to `delivery_512355` with `nav2_result_code=4`, `final_distance_m=0.049267`, and no API velocity correction; `20260629T090746Z` then reached `delivery_675235` with `nav2_result_code=4`, `final_distance_m=0.003381`, and `final_yaw_error_rad=0.045486`. AMCL accepted no corrections in these validation legs.

RotationShim startup alignment and terminal goal heading now share the Nav2 controller path for ordinary `pose_required` goals. `RotationShimController` wraps MPPI, `FollowPath.rotate_to_goal_heading=true`, and Nav2 uses `PoseProgressChecker` with `required_movement_radius=0.03`, `required_movement_angle=0.05`, and `movement_time_allowance=12.0` so measurable terminal creep or in-place yaw progress is not falsely treated as a short no-progress wait. `SimpleGoalChecker` is `stateful=false` with `xy_goal_tolerance=0.06` and `yaw_goal_tolerance=0.05`, and `robot_api_server` treats Nav2 result success as input to commercial final verification rather than business completion by itself. This prevents a first near-goal XY hit from staying latched after a dynamic obstacle or avoidance maneuver moves the robot back outside tolerance. Docking fine alignment remains stricter and is handled by the docking/GS2 pipeline after predock staging. `robot_api_server` owns business admission, cancellation, state reporting, final-pose verification, bounded retry, and degraded reporting. `position_only` is only an explicit engineering opt-out. If a short goal still produces angular-only commands, use `diagnose_nav2_zero_linear_progress_failure.sh` or `observe_nav2_native_pose_required_goal.sh` to classify whether the zero-linear behavior originates in the controller, collision monitor, robot_safety, mode controller/chassis, or odom reflection before changing pointcloud, DDS, costmap, EKF, FAST-LIO2, or App API behavior.

Return-to-dock uses Nav2 only as the coarse owner up to the pre-dock approach area; final predock yaw and lateral centerline capture are owned by `robot_api_server`. The backend prefers a manual point (`predock_pose_id`, `approach_pose_id`, or a saved pose such as `dock_main_predock`) and validates its yaw against the dock contact pose. That predock target is treated as `goal_completion_policy=dock_staging`, so ordinary navigation `final_yaw_align` is not allowed to run after staging Nav2 succeeds. Manual pre-dock distance checking is disabled by default (`docking_manual_predock_distance_check_enable=false`), so a close but intentionally saved point is not rejected only because it is below the old `0.50m` lower bound. If no manual point exists, the backend falls back to a geometric offset from the saved dock contact pose and exposes whether a reverse yaw offset was applied. The default docking normal path no longer forces before-predock, after-predock, or after-fine-docking relocalization around this short route. Instead, `robot_api_server` checks bridge `safe_for_goal_start`, sends `NavigateToPose(predock x/y/expected_base_yaw_at_predock)`, and while Nav2 is running it cancels early once the current pose enters the docking recovery window. That early transition is exposed as `STAGING_NAV2_EARLY_HANDOFF`, `predock_nav_early_handoff`, and `predock_nav_handoff_detail`; it prevents RotationShim/MPPI terminal yaw behavior from competing with docking-owned yaw/lateral capture. If Nav2 finishes normally or aborts before the early window, the API still enters `PREDOCK_POSE_VERIFY` only after checking that the pose is recoverable. If XY is acceptable but yaw is not, `PREDOCK_YAW_ALIGN_RECOVERY` can run before GS2 handoff; if the pose is outside the handoff window it fails with `DOCK_FAILED_PREDOCK_NAV_OUTSIDE_HANDOFF_WINDOW`, `PREDOCK_NATIVE_GOAL_VERIFY_FAILED`, or `PREDOCK_YAW_NOT_ALIGNED_AFTER_NAV2` instead of blindly entering fine docking. Fine docking is not started unless `predock_pose_verified`, `dock_staging_handoff_ready`, `predock_yaw_aligned`, GS2 freshness, a bounded `FINE_DOCKING_BRIDGE_SETTLE` wait for bridge `map->odom` smoothing to finish, and global-correction pause are all satisfied. `PREDOCK_YAW_ALIGN_RECOVERY` remains available only when `predock_yaw_align_enabled=true` and `predock_yaw_align_fallback_enabled=true`; it is still docking-owned and publishes only through `/cmd_vel_docking`. Explicit localization recovery remains available when localization is degraded, but it is not mixed into the default predock path.

After each explicit docking recovery relocalization, the API can still use the post-relocalization settle barrier. Default docking normal path admission is lighter: it requires `robot_localization_bridge` to own `map -> odom` and report `safe_for_goal_start=true`, leaving correction timing and smoothing inside the bridge. Docking cancel calls `/docking/stop` with the configured service wait and records that result instead of treating a short service-discovery miss as a clean stop.

Normal point navigation also treats dock/contact state and localization readiness as gates, but those long operations are no longer performed inside the mobile HTTP request. `robot_api_server` first builds one pre-navigation dock-contact snapshot from backend docking state, `/docking/status`, and fresh Ranger BMS charging contact, creates a background `navigation_goal`, and returns `202` quickly. If the backend state is `docked`, `/docking/status` starts with `docked` or `charging`, or BMS contact is active, that background job performs controlled `/docking/undock` and waits for post-undock recovery before any Nav2 action is sent. Full batteries may report `current=0`, so this gate uses `power_supply_status=FULL/CHARGING`, BMS contact reason, valid contact voltage, and configured full-SOC contact inference rather than current alone. `GET /api/v1/navigation/pre_goal_check` and `scripts/jetson/runtime_overlay/scripts/verify_pre_navigation_undock_gate.sh` expose the same gate read-only for field diagnosis. After the dock/contact gate passes, the background job waits briefly for bridge `safe_for_goal_start` plus AMCL correction readiness, then sends the Nav2 goal; if readiness does not recover in that bounded wait, `/api/v1/navigation/state` reports `failed_goal_start_readiness` instead of making the App wait for a long HTTP response. Normal goal admission does not synchronously call Nav2 lifecycle `GetState` services because those service responses can time out while the controller-hosted costmaps and planner are actually active under Jetson/FastDDS load. The normal goal handler does not call `/global_localization/trigger`, force-accept, `/localization_result` wait, or the post-relocalization settle barrier. `force_relocalize=true` remains an immediate `LOCALIZATION_RECOVERY_REQUIRED` rejection. AMCL static standby while the robot is stopped remains visible in status and does not by itself require explicit recovery or block goal start when `map -> odom` is live and no-motion standby is clean. If the Nav2 action server is unavailable after admission, the accepted background job is marked failed instead of blocking the mobile HTTP request. `pose_required` is the normal default and depends on Nav2 native XY+yaw completion first. After Nav2 result, the API audits a fresh `map -> base_link` pose. If the robot is already inside the yaw-alignable XY window and yaw remains outside tolerance, the API runs one ordinary `final_yaw_align` through `/cmd_vel_api -> robot_safety -> /cmd_vel`; if that recovers the final pose, `task_complete=true`, otherwise the job remains failed with the yaw fallback diagnostics. The API does not retry the same goal, publish to collision_monitor's `/cmd_vel_collision_checked`, use `/cmd_vel_docking`, or let the App send chassis velocity. `position_only` remains available as an explicit opt-out while still relying on Nav2 as the motion completion owner. App clients must not display task success until `task_complete=true`.

Phase N6 supersedes the older N5 ordinary-completion behavior described above. Normal point navigation now uses `xy_goal_tolerance=0.06` and `yaw_goal_tolerance=0.05`; Nav2 success is not business completion by itself. After Nav2 returns, `robot_api_server` waits for bridge smoothing, reads fresh `map -> base_link`, and only sets `task_complete=true` when XY <= 0.06 m and yaw <= 0.05 rad, or when post-retry XY is inside the 0.08 m slack. A 0.06-0.12 m XY overrun or 0.05-0.15 rad yaw overrun triggers bounded same-goal retry or final yaw alignment; a 0.12-0.35 m XY overrun or 0.15-0.35 rad yaw overrun enters recovery retry. If Nav2 is still executing within 0.30 m and stops improving for 1.5 s after the 3 s minimum wait, API cancels only that Nav2 action and enters final verification/yaw/retry. Terminal pose correction is deterministic rather than heuristic: the API decomposes target error into signed yaw, body-frame lateral error, and body-frame forward error, then corrects yaw first with pure `angular.z`, lateral second with pure `linear.y`, and forward/reverse third with pure `linear.x`. If retry cannot satisfy the commercial gate, the job is marked `degraded` with `task_complete=false` instead of reporting false success. The command path remains `/cmd_vel_api -> robot_safety -> /cmd_vel` for API terminal correction and never uses collision_monitor's `/cmd_vel_collision_checked` as an API publisher.

Phase 2.5 adds a non-position dock-contact latch as a second dock-state source. The latch is written by explicit events only: BMS contact, `/docking/status` docked/charging, docking success, and undock success. It is not inferred from the robot's current map pose. `pre_navigation_dock_check` exposes the latch as `dock_contact_snapshot`; if it is docked, normal navigation must run `/docking/undock` before Nav2. `final_yaw_align` rechecks the same gate and exits with `DOCKED_OR_CHARGING_CONTACT` instead of rotating. `robot_safety` also subscribes BMS and `/docking/status`, reads the same latch, and reports `DOCKED_CONTACT_BLOCK` while zeroing normal commands; `/cmd_vel_docking` remains allowed for controlled docking/undocking, including between watchdog timer ticks while the docking command is fresh. If fresh BMS reports no charging contact and there is no current docked/charging status, `robot_safety` treats the persistent latch as contradicted safety memory instead of a permanent normal-motion block. `/api/v1/status` and `/api/v1/navigation/state` expose safety status and `normal_motion_blocked_reason` so the App can display the blocker without inferring dock state from position.

Phase 2.6 extends that latch for full-charge and missing-contact recovery. `docking_contact_latch.json` now carries `latched_docked`, source, map/floor context, timestamps, clear reason, and note fields while retaining the old `docked` field for compatibility. Maintenance endpoints/scripts can confirm or clear the latch without sending velocity. Phase D2 narrows BMS-derived latch use: a `source=bms` latch has a TTL, is written only after stable BMS contact, and cannot singly trigger pre-navigation auto-undock unless explicitly allowed by parameter. BMS contact false alone is not navigation permission, but the combined live contradiction of `/docking/state=undocked`, `/docking/status` not docked/charging, and stable BMS no-contact clears stale `source=bms` latch evidence with `clear_reason=stale_bms_latch_cleared_live_undocked_no_contact`. `pre_navigation_dock_check` reports `strong_live_docked`, `latch_valid_for_auto_undock`, latch source/age/stale/contradiction, and `docked_state_class`; ordinary navigation auto-undocks only for strong live dock evidence or valid non-stale latch evidence.

Phase D2.3 separates charging/contact telemetry from physical dock occupancy. New live BMS charging/contact/current evidence writes a strong `source=charging_session` latch. `source=charging_session` and `source=docking_job` represent dock/session evidence and cannot be cleared by BMS `no_contact`, `current=0`, or `present=false` alone while live docking context or full-charge-idle evidence still suggests the robot may physically remain on the charger. A legacy `source=bms` latch can be auto-cleared when fresh BMS reports stable no-contact and runtime/docking state has no docked, charging, or undocking context. A strong `source=charging_session` latch is not cleared by restart-time idle/no-contact context; it is auto-cleared only after confirmed live undock plus stable BMS no-contact, or by explicit maintenance/session clear. A full battery at charger idle can therefore still be `DOCKED_CHARGE_IDLE` when the context supports it. `dock_occupancy_state` is exposed by `/api/v1/navigation/state`, `/api/v1/docking/state`, and `pre_navigation_dock_check`; `CONFIRMED_DOCKED`, `DOCKED_CHARGING`, `DOCKED_CHARGE_IDLE`, and `UNCERTAIN_ON_DOCK` block direct Nav2 submission and require auto-undock first. SOC=100 without prior charging/session evidence remains insufficient dock proof.

Phase 2.8 keeps the same docking ownership but splits undock progress timing into explicit phases. `robot_docking_manager` still owns near-field docking and controlled undocking; return-to-dock travel remains Nav2 up to the pre-dock pose. The undock path remains `/cmd_vel_docking -> robot_safety -> /cmd_vel`, while ordinary Nav2 reverse is bounded to MPPI terminal correction only (`vx_min=-0.08` with `PreferForwardCritic`). `/cmd_vel_safe` is a robot_safety mirror for diagnostics. The retained calibrated speed is `undock.speed_mps=0.06`. `undock.command_settle_s` allows the Ranger park/forced-mode/reverse-enable state to settle before nonzero undock commands, `undock.motion_start_timeout_s` waits for first odometry-confirmed motion, and `undock.no_progress_timeout_s` is used only after first motion to detect a mid-undock stall. The total `undock.timeout_s` must cover command settle, first-motion wait, `distance / speed`, and margin. Use `scripts/jetson/runtime_overlay/scripts/diagnose_undock_logic_and_no_motion.sh --dry-run` for static/API checks, and `--execute-undock` only for a supervised controlled undock diagnostic.

Phase 2.7c tightens the motion-start phase so it cannot wait for odometry before sending the reverse command. After `command_settle_s`, every control tick in `waiting_first_motion` publishes `/ranger_mini3/docking_allow_reverse=true` and `/cmd_vel_docking.linear.x=-0.06` while waiting for `/local_state/odometry` to move by `progress_epsilon_m`. `/docking/status` includes `cmd_x`, `cmd_count`, `reverse_enable`, and timing fields. `undock_failed_motion_start_timeout ... cmd_count>0` means commands were sent and the next diagnosis should follow `robot_safety`, mode-controller, chassis execution, and odometry. `undock_failed_no_command_published` or `cmd_count=0` means the docking-manager state machine did not publish the undock command and must be treated as a software bug. This does not change Nav2, pointcloud, DDS/RMW, EKF, FAST-LIO2, Ranger CAN, App velocity ownership, or the final `robot_safety` speed chain.

Phase 2.7d keeps the same speed and ownership but makes the final safety arbiter continuous for push-in spring charging docks. Because the charging switch is mechanically engaged by pushing into the dock, undocking must drive at the controlled low speed through the switch travel rather than stopping on the DC contact. `robot_safety` stores the last fresh `/cmd_vel_docking` command and republishes it from its safety timer while `docking_cmd_priority_timeout_sec` is active; blocking states still publish zero, stale commands still expire, and ordinary Nav2 reverse is bounded to low-speed terminal correction instead of undock ownership. `diagnose_undock_logic_and_no_motion.sh` now also treats API `cmd_count` evidence as command evidence, so reports with `cmd_count>0` are classified downstream of the docking manager instead of as no-command state-machine failures.

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

`run_navigation_runtime_services.sh` resolves the selected floor assets, first reuses the fresh `local_state_ready` runtime-health snapshot produced by common services, and only falls back to direct `/local_state/odometry` plus `odom -> base_link` probes when that snapshot is unavailable. It then starts the resident occupancy-localization layer and runs the initial `/global_localization/trigger` only after selected-floor localization readiness and floor context selection by default. Nav2 starts after that initial baseline in production; `NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=true` is A/B only. The experimental `NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=true` path can still overlap the trigger with map/floor readiness for A/B, but it is not the production default because field startup showed it can increase stale Isaac cold-start results. A missing local-state endpoint now fails as `LOCAL_STATE_ENDPOINT_NOT_READY`, stale odometry as `LOCAL_STATE_ODOM_NOT_FRESH`, and stale local TF as `ODOM_BASE_TF_NOT_FRESH`, instead of being reported later as an AMCL warmup failure. The wrapper calls Isaac's direct grid-search service but startup success is judged by `robot_localization_bridge` accepting the result, `/localization/bridge_status.has_map_to_odom=true`, and a live `map -> odom` TF owned by `robot_localization_bridge`. AMCL resident warmup before the initial triggered baseline is disabled by default (`NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=false`) because AMCL lifecycle depends on stable map/seed context; enabling it is an A/B diagnostic path only. AMCL readiness still starts after the initial triggered baseline has been accepted, and the commercial navigation default is `NJRH_AMCL_LOCALIZATION_MODE=gated`, so bounded AMCL corrections can continuously update `map -> odom`; `shadow` remains the odom-only audit rollback. Nonfatal AMCL background failures are logged without poisoning the resident runtime context. The runtime context is marked ready after Nav2 lifecycle activation, `/global_costmap/global_costmap` is active, and `/global_costmap/costmap` has a publisher.

If `NJRH_AMCL_LOCALIZATION_MODE=shadow` or `gated`, AMCL is not part of the Nav2 controller/planner lifecycle and does not publish TF. After `/global_localization/trigger` is accepted by `robot_localization_bridge` and `map -> odom` is live, resident startup starts AMCL resident warmup in the background while Nav2 lifecycle activation proceeds. That warmup activates AMCL through the standard `/amcl/change_state` lifecycle service, warms AMCL's process-local TF buffer from `/map`, `/scan`, `odom -> base_link`, and `base_link -> scan_frame`, and starts the C++ `/scan_amcl` admission relay. After Nav2 lifecycle activation and global costmap readiness, the runtime joins the AMCL warmup if it is still running, then retries readiness completion for a bounded window: seed `/initialpose` through `/robot_localization_bridge/seed_amcl_initial_pose`, then wait for `/amcl_pose` only when explicit diagnostics disable static-standby skip. AMCL and the C++ scan-admission relay start through installed binaries (`/opt/ros/humble/lib/nav2_amcl/amcl` and `install/robot_localization_bridge/lib/robot_localization_bridge/amcl_scan_admission_node`) instead of `ros2 run` wrappers, and the seed service is called by an in-process rclpy client instead of `ros2 service call`, so startup avoids CLI package lookup and wrapper shutdown cost. `/scan_amcl` is an AMCL production admission input derived from `/scan`, preserves the original stamp/frame/ranges, drops stale or non-TF-transformable scans, defaults to 5 Hz, and is bound to `NJRH_CPUSET_AMCL_SCAN_ADMISSION` by `taskset` when started. AMCL readiness requires AMCL active, seed success, and healthy scan admission; while stopped or docked, stale `/amcl_pose` is not fatal and is exposed as `amcl_not_moving_no_update_ok`. During motion, stale `/amcl_pose` makes AMCL not tracking. `gated` is the commercial default and applies only bounded bridge-approved corrections; `shadow` reports bridge candidates only for diagnostics and odom-only audits. If AMCL readiness does not complete within `NJRH_AMCL_READINESS_COMPLETION_TIMEOUT_SEC`, resident startup fails instead of writing a ready context that the API would later reject. Navigation stop calls the AMCL stop helper, which also stops the scan admission relay, before stopping the rest of the navigation stack.

The AMCL runner writes `/tmp/njrh_amcl_runtime_status.env` on start, readiness completion, degraded startup, failure, and stop. The file records `AMCL_STATE`, `AMCL_READY`, `AMCL_DEGRADED`, `AMCL_FAILURE_REASON`, AMCL lifecycle/process state, stale PID cleanup, scan-admission PID/publisher state, `/amcl_pose` publisher count, seed status, the observed `map -> odom` owner, `AMCL_STATUS_STAMP_SEC`, no-motion probe fields, and split readiness fields for process/seed/static-standby/tracking/correction. `run_navigation_runtime_services.sh` captures the AMCL runner exit code under `set +e`, reads this status file, logs `AMCL_STATUS`, and only allows exit `10` to continue in `shadow` mode. Startup status writing is fast by default and uses the already validated AMCL/scan-admission PIDs plus the lifecycle action outcome instead of launching additional `ros2 node/topic` graph probes; set `NJRH_AMCL_STATUS_GRAPH_PROBE_ENABLED=true` only for explicit diagnostics. The file is a TTL-bound snapshot rather than a permanent authority: if it becomes stale, `robot_localization_bridge` reports `amcl_status_source=stale_file_ignored` and uses live AMCL graph/subscription evidence instead of letting an old `AMCL_FAILED` keep localization degraded. AMCL is event-driven while stationary, so a stale or absent fresh `/amcl_pose` immediately after seed is normal static standby, not a failure, when AMCL is active, scan admission is publishing, and the seed service succeeded. Startup defaults to `NJRH_AMCL_STATIC_STANDBY_SKIP_POSE_WAIT=true`, so successful seed immediately writes `AMCL_STATIC_STANDBY_ACCEPTED=true`, `AMCL_TRACKING_READY=true`, and `AMCL_CORRECTION_READY=false`; this means AMCL is ready as a resident candidate but has not yet supplied an applyable correction. If resident AMCL initially reports `AMCL_WAITING_SEED`, the bridge can resolve that transient state from live evidence once seed, scan admission, stationary robot state, and static standby are all true; normal goal admission then remains allowed when `map -> odom` is stable. Explicit diagnostics can disable the skip and use `amcl_nomotion_update_probe.py`, which subscribes to `/amcl_pose`, waits a short warmup, then calls `/request_nomotion_update`; the seed check accepts a pose received during that service window even if the header is older than the correction gate, while actual gated corrections still require a fresh header/TF-compatible pose. A seeded stationary AMCL that is tracking-ready but has not yet produced a fresh gated correction is reported as `amcl_correction_pending=true`; that is diagnostic and not `localization_degraded=true`. API goal admission allows clean no-motion static standby using structured bridge fields, but treats non-standby pending/not-ready correction as `LOCALIZATION_TRANSITION_ACTIVE` so Nav2 is not started while map correction is still entering `map -> odom`. `/localization/bridge_status`, `/api/v1/status`, and `/api/v1/navigation/state` expose `amcl_status_file_stale`, `amcl_status_source`, `amcl_seed_response_ok`, `amcl_nomotion_pose_received`, `amcl_process_ready`, `amcl_seeded`, `amcl_static_standby`, `amcl_tracking_ready`, `amcl_correction_ready`, `amcl_correction_pending`, `localization_degraded`, and `using_triggered_baseline_only`.

AMCL scan-admission readiness is based on the relay producing `/scan_amcl`,
not on the last observed drop reason being exactly `none`. The relay can be
publishing at a healthy rate while the most recent raw `/scan` sample is just
over the age cutoff and reports `AMCL_SCAN_STALE`; that must not permanently
block AMCL seeding. The readiness gate therefore requires `published_count > 0`
and a minimum output rate (`NJRH_AMCL_SCAN_ADMISSION_READY_MIN_HZ`, default
`0.5`) while still rejecting active TF, frame, future-stamp, and warmup errors.

## Post-Relocalization Settle Barrier

API-local TF readiness is not the same as controller/local-costmap readiness after an explicit map correction. Phase L2 therefore inserts a settle barrier after forced Isaac relocalization is accepted by `robot_localization_bridge` and before the next Nav2 goal or GS2 fine docking stage. The barrier checks `/localization/bridge_status.last_explicit_relocalization_sequence`, `map -> odom` owner/freshness, API-observed `odom -> base_link` freshness, `base_link -> lidar_level_link`, `/local_costmap/costmap` update heartbeat, and new local-costmap MessageFilter drops while publishing only zero commands through the existing safety path.

Default parameters live in `robot_api_server.yaml`: `post_relocalization_settle_min_ms=800`, `post_relocalization_settle_max_ms=3000`, `post_relocalization_stable_tf_samples=5`, `post_relocalization_tf_sample_period_ms=100`, `post_relocalization_required_local_costmap_updates=2`, and large-correction minimum settle time `1500 ms`. Failure codes are explicit: `POST_RELOCALIZATION_SETTLE_TIMEOUT`, `POST_RELOCALIZATION_STABLE_SAMPLE_TIMEOUT`, `POST_RELOCALIZATION_CORRECTION_ACTIVE`, `POST_RELOCALIZATION_MAP_ODOM_NOT_FRESH`, `POST_RELOCALIZATION_ODOM_BASE_NOT_FRESH`, `POST_RELOCALIZATION_TF_CHAIN_UNSTABLE`, `POST_RELOCALIZATION_LOCAL_COSTMAP_NOT_UPDATED`, `POST_RELOCALIZATION_LOCAL_COSTMAP_TF_DROPS`, `POST_RELOCALIZATION_SCAN_ADMISSION_TF_ERROR`, `POST_RELOCALIZATION_WRONG_MAP_ODOM_OWNER`, `POST_RELOCALIZATION_SEQUENCE_MISMATCH`, and `CANCELLED_BY_APP`. Roll back by setting `post_relocalization_settle_enabled: false` and restarting `robot_api_server`; no Nav2 controller/planner, TF tolerance, pointcloud, FAST-LIO2, Ranger odom, or EKF parameter is changed by this barrier.

Phase R0-R2 keeps this barrier for explicit recovery paths but removes it from ordinary navigation-goal admission and default predock docking. Bridge correction acceptance, rejection, and smoothing are owned by `robot_localization_bridge`; the API checks `safe_for_goal_start` instead of scheduling a hidden relocalization before `FollowPath`.

Phase U1 adds post-undock aliases for the same barrier so auto-undock can hold the pending point-navigation goal until the controller/local-costmap side is ready: `post_undock_relocalization_settle_enabled=true`, `post_undock_relocalization_settle_min_ms=800`, `post_undock_relocalization_settle_max_ms=5000`, `post_undock_stable_tf_samples=2`, `post_undock_tf_sample_period_ms=100`, `post_undock_required_local_costmap_updates=2`, `post_undock_reject_if_new_message_filter_drop=true`, and `post_undock_zero_cmd_during_settle=true`. A post-undock localization result accepted by `robot_localization_bridge` does not by itself prove that `controller_server` and the local costmap have consumed a stable TF chain, but post-undock goal release treats local-costmap and AMCL scan-admission transients as warnings once the hard bridge/TF conditions are clean. During this barrier the API keeps zero velocity active and exposes `post_undock_settle` in `/api/v1/navigation/state` plus post-undock fields in `/api/v1/docking/state`. If the hard bridge/TF conditions pass but the timer expires before all configured samples are collected, post-undock releases the pending goal with a warning instead of reporting a stale `POST_UNDOCK_MAP_ODOM_PUBLISH_SEQUENCE_LAG`. If a hard condition fails after odometry-confirmed undock, the API reports `post_undock_navigation_readiness_failed=true`, leaves the docking job `undocked`, and does not send the original `NavigateToPose` goal, avoiding a one-second Nav2 abort without misreporting the physical undock as failed. Observe without sending goals with `scripts/jetson/runtime_overlay/scripts/observe_post_undock_to_nav_goal.sh --duration-sec 180`.

`run_floor_navigation.sh` is compatibility-only and blocked by default. Daily
runtime restarts must use the host owner:

```bash
sudo systemctl restart njrh-runtime.service
```

The wrapper only delegates to `run_navigation_runtime_services.sh` when an
operator explicitly sets `NJRH_ALLOW_TRANSIENT_NAVIGATION_OWNER=1` for a
debug-only manual run. This prevents `docker exec -d` or `nohup` from starting a
second transient navigation owner whose EXIT cleanup can tear down Nav2,
localization, AMCL, and bridge children while readiness scripts continue waiting.

If resident navigation startup exits before the runtime context reaches confirmed `ready`, `run_navigation_runtime_services.sh` writes the context as `failed` with the resume log path. If the failure is Nav2 lifecycle activation, the script stops only the failed Nav2 layer, then runs one outer standard-Nav2 process sweep so stale `controller_server`, `planner_server`, `bt_navigator`, and lifecycle-manager processes cannot remain in the ROS graph. It keeps the selected-floor localization layer alive for diagnostics and retry. The runtime context includes `startup_stage` and `startup_elapsed_sec`, and the App should show that failed or starting state instead of waiting forever on generic TF pose timeouts.

Host `scripts/jetson/njrh_container.sh start-runtime` is a full runtime readiness command, not only a container/API probe. It first waits for `/api/v1/status`, then, when `NJRH_RESIDENT_NAVIGATION_AUTOSTART` is enabled and a selected or last navigation map exists, waits for `/tmp/njrh_runtime_map_context.json` to report `state=ready` and `confirmed=true`. By default that confirmed context means bridge-owned `map -> odom` plus Nav2 lifecycle readiness; AMCL tracking readiness is reported separately and continues in the background unless `NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY=true` is set. The default full runtime SLA window is `NJRH_ROBOT_NAV_READY_TIMEOUT_SEC=120` from the start of `start-runtime`. `stop-common` also clears stale runtime context and explicitly stops detached resident navigation, Nav2 lifecycle, occupancy localization, AMCL, scan admission, bridge, and API processes by fixed repository process patterns before the next start. This prevents an old ready context or orphaned Nav2 stack from making a cold restart look ready when the newly-started chain is not.

The boot-time systemd runner intentionally does not use resident navigation
readiness as a common-service startup gate. `run_common_services.sh` starts the
resident navigation runtime when a saved floor map exists, then prints
`common services are running` and enters its long-lived sleep loop without
waiting for `/tmp/njrh_runtime_map_context.json` to become confirmed `ready`.
The resident navigation runtime continues to own map/localization/Nav2
readiness and writes `starting`, `ready`, or `failed` to the context file for
the API and App to report. This keeps driver, chassis, TF, local state, safety,
docking, and API resident even when Nav2 or localization is still starting or
has failed, instead of letting a navigation-context timeout restart the whole
common-service layer.

Manual floor-navigation stop clears the runtime map context after killing Nav2/localization helper processes. This prevents `robot_api_server` from recovering a stale `ready` context after the stack has been stopped. The script sends INT/TERM/KILL to Nav2, localization bridge, occupancy localizer, AMCL, scan admission, and local-perception process patterns before the bounded AMCL shutdown helper runs; if AMCL lifecycle cleanup exceeds `NJRH_NAV_STOP_AMCL_TIMEOUT_SEC`, the script logs that diagnostic and continues with final lingering-process verification instead of letting AMCL block the API stop window.

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

Explicit readiness and field diagnostics must use the same DDS environment as
the production runtime. Every diagnostic script that starts a ROS participant
sources `common_env.sh`, records `RMW_IMPLEMENTATION`,
`FASTDDS_BUILTIN_TRANSPORTS`, and the Fast DDS profile path, and runs temporary
CLI probes in bounded process groups. Scripts must not leave naked `ros2`
inspection processes in the production domain. High-churn `/tf` and
`tf2_echo` captures are opt-in only; default field reports prefer API status,
topic metadata, low-rate status topics, and existing logs. This is a diagnostic
containment rule, not a replacement for fixing a half-dead TF owner.

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
