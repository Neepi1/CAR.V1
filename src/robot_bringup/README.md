# robot_bringup

Mapping startup reuses DDS contexts for conditional scan restore and exact
preflight, starts the private odom bridge alongside FAST-LIO initialization,
and combines scan-owner/freshness/original-stamp-TF confirmation. See
[mapping startup repair](../../docs/mapping_startup_latency.md).

`imu_pipeline_node` co-locates the canonical IMU remap and gyro-bias filter with
independent serialized executors and an intra-process IMU edge. It is owned by
sensor ingress, not EKF. See [design and validation](../../docs/imu_intra_process_pipeline.md).

完整导航冷启动支持临时使用 CPU0–7，结束后恢复既有五核逐模块分配；不改变
启动顺序或就绪判据。范围与硬件验收见 [启动调度说明](../../docs/startup_cpu_boost.md)。

Bringup composition for the repository-owned localization and navigation baseline.

## Runtime health

`runtime_health_guard` is the C++ resident observer: 1Hz latest-message sampling,
1Hz atomic JSON snapshots, and a separate 3-second no-fresh-update grace period.
`runtime_health_check` reads those snapshots without ROS/DDS or Python. Startup
freshness, independent failure confirmation and systemd whole-chain ownership
remain separate. See [runtime health design](../../docs/runtime_health_cpp.md)
for configuration, clock behavior, recovery and hardware acceptance.

One-shot readiness/lifecycle clients disable their own unused parameter
services and rosout (C++ also omits parameter-event publication), without
altering target-node services or readiness predicates. The network-isolated
`test/isolated_startup_clients_smoke.py` verifies endpoint and input behavior;
see [restart latency work](../../docs/restart_latency_40s.md) for measured runs.

Lifecycle confirmation retains a timed-out ChangeState future while querying
state, instead of discarding late success or repeating an unproven transition.
Non-trusting callers still require actual state. Trigger timing distinguishes
initialization, RPC/TF waiting and cleanup without changing readiness criteria.

The C++ `lifecycle-active` readiness check also retains one GetState request
across its 800 ms spin slices, up to the original overall deadline. It sends
another request only after a non-active reply, and removes an unresolved request
on timeout/interruption. `test/isolated_lifecycle_reply_smoke.py` exercises real
delayed, inactive-then-active, missing and interrupted replies in a private
network-none domain; it must not be run against production ROS services.

Whole-chain cleanup re-scans after old PID exit and reaps shutdown-created ROS
CLI children. Query-tool arguments cannot be mistaken for live runtime nodes;
actual residual nodes still fail the absence check.

The API launcher reuses prepared ROS/project setup and directly execs the
installed binary under the existing affinity and supervisor. Missing setup and
post-build setup are still loaded; API behavior and motion admission are unchanged.

Startup localization retains its trigger ROS node for actual TF confirmation,
but creates the volatile TF reader only after acceptance to exclude old queued
messages. Original TF waits and floor-handoff behavior remain; no transform or
velocity is published by this client. See the restart latency evidence above.

Production management also uses `runtime_process_check`, `runtime_amcl_status`
and `runtime_flatscan_check`. These native clients do not create ROS
participants; AMCL and FlatScan observation reuse the resident guard. See
[management design](../../docs/runtime_management_observer.md).

## Launches

- `mock_navigation.launch.py`: first-round lightweight stack for scaffold and mock validation
- `localization_bringup.launch.py`: canonical platform stack plus optional `nav2_map_server`
- `localization_bringup.launch.py`: also starts `robot_floor_manager` so floor switches can reuse the active map server and localization services
- `navigation_bringup.launch.py`: `localization_bringup` + repo-owned standard navigation chain
- `standard_navigation.launch.py`: repo-owned standard Nav2 stack only, for runtime paths that already started localization separately
- `local_costmap_debug.launch.py`: repo-owned local-costmap-only debug stack; it starts only `controller_server` and its local costmap lifecycle owner, with `cmd_vel` remapped away from the real control chain

## Parameters

- `use_sim_time`: default `false`
- `autostart`: default `true`
- `map_yaml`: Nav2 map asset used by the repo-owned map server entrypoint
- `params_file`: defaults to `robot_nav_config/config/nav2.yaml`
- `use_respawn`: passed to the repo-owned Nav2 node set
- `use_composition`: accepted for API compatibility; the field runtime still launches the stack non-composed
- `canonical_tf_policy`: points to `robot_nav_config/config/tf_policy.yaml`

## Notes

- Startup overlap and lifecycle-client reuse are described in
  [full restart latency work](../../docs/restart_latency_40s.md). Lifecycle
  transitions and readiness evidence stay unchanged; five-run timing acceptance
  is separate from unit tests and deployment.
- `runtime_readiness_probe amcl-inputs` combines AMCL's existing map, scan/frame
  and TF startup checks in one bounded context. It emits per-condition progress
  for the existing same-generation checkpoint and never publishes commands or
  changes parameters. See [AMCL startup](../../docs/amcl_startup_sequence.md)
  for isolated tests and separately authorized restart acceptance.
- `runtime_readiness_probe mapping-preflight` is the production 2D-mapping admission probe. One ROS 2 participant concurrently verifies `base_link -> lidar_level_link`, the exact canonical `/scan` owner and freshness, the resident `robot_local_state` endpoint, local odometry freshness, and the configured wheel/FAST-LIO reference-odometry consistency. It remains bounded and fail-closed; a failed composite probe does not fall back automatically to weaker checks.
- `run_projected_map.sh` emits `MAPPING_STARTUP_STAGE stage=<name> elapsed_sec=<n>` at each startup boundary. Use these records to distinguish legacy cleanup, mapping-pipeline cleanup, private FAST-LIO2 cleanup, DDS discovery, FAST-LIO2 readiness, odom-bridge readiness, and `slam_toolbox` launch time. Cleanup is candidate-driven (`pgrep`, followed by exact environment-marker validation where ownership requires it), so an idle start neither scans every `/proc` entry through external filters nor pays a fixed signal-settle sleep. Mapping-owned FAST-LIO2 is checked for immediate process survival and then admitted by the existing fresh-cloud, odometry, and same-instance gates, without a redundant fixed two-second wait. `NJRH_SLAM2D_COMPOSITE_PREFLIGHT_ENABLED=false` exists only as an operator-controlled rollback to the former serial readiness probes.
- `localization_bringup.launch.py` keeps the repository-owned canonical stack in front: description, chassis, JT128, local perception, local state, global localization, localization bridge, robot safety, and optional map server.
- `navigation_bringup.launch.py` reuses that same stack and then loads the repo-owned Nav2 chain from `standard_navigation.launch.py` with this repository's `robot_nav_config/config/nav2.yaml`.
- `standard_navigation.launch.py` explicitly launches `controller_server`, `behavior_server`, `velocity_smoother`, `collision_monitor`, `lifecycle_manager_costmap_filters`, and `lifecycle_manager_navigation` so every Nav2 velocity path is routed into the repository-owned safety chain while costmap filter servers have an isolated lifecycle owner. The field runner passes `navigation_lifecycle_autostart:=false` and activates the core navigation nodes with Nav2's `nav2_util/lifecycle_bringup` helper so Humble's fixed 2 second lifecycle-manager `get_state` wait does not abort startup while `planner_server` is loading the global costmap.
- `scripts/jetson/runtime_overlay/launch/occupancy_localization.launch.py` exposes `map_lifecycle_manager_enabled`. The production resident runtime passes `false` and activates `/map_server` with `nav2_util/lifecycle_bringup map_server`, so a map-server lifecycle-manager response timeout cannot leave a successfully loaded selected-floor map inactive.
- `standard_navigation.launch.py` and `local_costmap_debug.launch.py` read `NJRH_CPUSET_*` environment variables and add `taskset` prefixes when the Jetson runtime CPU affinity policy is enabled. The stack remains non-composed so critical nodes can be assigned different cores.
- The Jetson runtime wrapper around `standard_navigation.launch.py` now requires an active `/map_server`, waits for `/map`, and waits for `/global_costmap/costmap` to resize to the static map before considering the standard navigation stack ready.
- `local_costmap_debug.launch.py` is only for obstacle-layer verification. It does not start planner, BT navigator, velocity smoother, collision monitor, map server, or robot safety, so it must not be treated as the production navigation path.
- Jetson field runtime still uses the temporary `NJRH-car` container and shell helpers for operator workflows, but these launch files now define the repository-owned bringup contract that those helpers should converge toward.
# Restart camera ownership

Final startup service discovery and live bridge-state observation share a
short-lived client, preserving readiness semantics and adding phase timings.
For normal systemd startup with pending background Nav2 activation, the native
`startup_context_observer` opens that context early. It publishes nothing and
makes no service calls. At the parent's final commit marker it checks service
graph presence again, then requires a bridge message whose real DDS source
timestamp is no earlier than both commit and post-service confirmation. Old
queued samples cannot prove READY. Exact sequence/map commit remains in the
parent; missing/expired native preparation uses the existing Python path once.
Humble's public C++ MessageInfo is used; rclpy internals and DDS are unchanged.
The core test and `test/isolated_context_observer_smoke.py` cover old backlog,
fresh status, cancellation, owner exit and parent sequence rejection. Run the
latter only in its required private network/IPC fixture, never production ROS.
Optional background Nav2 activation now honors the requested after-stack phase
and joins its bounded worker without repeated cold lifecycle clients. Defaults
remain disabled; the restart experiment enables this through runtime.env.
Cleanup also covers the actual occupancy-localization launch owner so its
respawning child cannot outlive the stopped navigation generation.

Whole-runtime cleanup selects the canonical `camera336l` launch/namespace,
not every process using the Orbbec package. Independently managed cameras and
frame-name observers are outside navigation ownership. See
`../../docs/restart_latency_40s.md` for timing evidence and remaining acceptance.

For a bounded cold-start contention experiment,
`NJRH_NAV2_PRESTART_AFTER_LOCALIZATION_STACK=true` moves held Nav2 process
creation after existing map/Isaac/input initialization but before requesting
initial localization. It does not wait for localization success. The default
is false; non-systemd and legacy starts keep their original order. Stationary
restart timing must establish benefit before calling this an optimization.
