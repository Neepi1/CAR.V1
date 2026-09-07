# Phase S3 Fast Resident Navigation Startup

Phase S3 shortens cold-boot resident navigation startup without changing TF
ownership, pointcloud QoS, Nav2 plugins, FAST-LIO2 logic, or the Ranger speed
chain.

The startup contract is:

1. Common services must own the canonical `robot_local_state` process and ROS
   endpoint. The resident runtime always requires fresh
   `/local_state/odometry` and `odom -> base_link` before initial localization
   can be triggered or Nav2 can be activated.
2. Start-source policy controls only when the selected-floor localization
   process may be prewarmed:
   - systemd boot sets `NJRH_NAVIGATION_START_SOURCE=systemd_autostart` and
     keeps the measured cold-start overlap; localization may be spawned while
     the resident runtime completes its fresh local-state gate;
   - `POST /api/v1/navigation/start` sets
     `NJRH_NAVIGATION_START_SOURCE=api_resume` and first requires three
     consecutive, timestamp-advancing `/local_state/odometry` plus
     `odom -> base_link` observations. Only then may localization start.
3. The selected-floor localization layer and exact floor context must be ready
   before the initial global-localization transaction is dispatched.
4. The initial `/global_localization/trigger` must be accepted by
   `robot_localization_bridge`, which evaluates the result at its original
   timestamp and publishes the canonical `map -> odom`. The bridge retains
   30 seconds of `odom -> base_link` TF history for the wrapper's 20-second
   result window; it never restamps the result or substitutes latest TF.
5. The production path starts the held Nav2 process preload only after that
   bridge-owned baseline is accepted. Lifecycle activation remains held until
   the preload is complete.
   `NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=true` remains
   available for A/B diagnostics, but it is not the default because field
   startup showed lifecycle helpers and localization repair can race when Nav2
   is prestarted and activated too early.
6. Startup does not repeat the floor-manager source-preflight service after it
   verifies the already committed exact `current/` asset context. The
   child-process
   switch `NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=true` remains
   available for A/B, but it does not bypass localization-stack or exact
   floor-context readiness.
   Each trigger-client attempt retains its existing rclpy `--timeout-sec`
   budget and is additionally wrapped by an operating-system process timeout:
   the wrapper budget plus 5 seconds for result/shutdown output, then `SIGTERM`
   with a fixed 2-second `SIGKILL` escalation. A stuck rclpy shutdown therefore
   cannot leave the resident runtime indefinitely in `starting`; the existing
   return-code handling and bridge-owned `map -> odom` fallback remain unchanged.
7. AMCL resident warmup before the initial triggered bridge baseline is disabled
   by default. The AMCL readiness path starts after the bridge baseline is
   accepted so lifecycle failures from missing map/seed context cannot poison the
   resident runtime. Set `NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=true`
   only for A/B diagnostics. AMCL seed and readiness completion still continue
   in the background by default and are reported through runtime status; set
   `NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY=true` to restore the older hard
    startup gate.

The App resume preflight is an observation gate, not a sleep and not a second
startup implementation. It uses one `runtime_readiness_probe` participant,
validates the canonical `odom` / `base_link` frame contract, checks the existing
age limits, and requires both odometry and TF timestamps to advance. Failure is
reported as `LOCAL_STATE_ODOM_TF_NOT_STABLE`; localization and Nav2 are not
started on top of a stale local-state boundary. This branch normally costs only
the DDS discovery time plus three 50 Hz samples and does not affect the systemd
cold-boot ordering.

This is not a return to pure odom. Nav2 is not started until a global
localization result has produced a bridge-owned `map -> odom`; after that,
Nav2 lifecycle nodes must become active and the global costmap must be
available. AMCL remains the continuous correction source after its candidate
path becomes ready, but AMCL tracking readiness is not a default cold-start
hard gate because the bridge-owned triggered baseline is already the TF
authority. AMCL no longer starts before the first triggered Isaac baseline
unless explicitly requested by A/B environment. The runtime context advertises
ready after Nav2 lifecycle activation has succeeded and the global costmap
lifecycle plus `/global_costmap/costmap` publisher are observable. The full
large OccupancyGrid message gate remains available for diagnostics with
`NJRH_GLOBAL_COSTMAP_FULL_MESSAGE_GATE=true`, but it is not the default startup
gate because field measurements showed the next full costmap frame can lag the
already-active static layer by several seconds. AMCL status continues to converge in the background
unless `NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY=true` is set. The resident
runtime then starts an AMCL runtime status heartbeat. That heartbeat refreshes
`/tmp/njrh_amcl_runtime_status.env` from the resident AMCL and scan-admission
PIDs plus the accepted seed state. The bridge/API read that file with a short
TTL, so readiness must remain a live heartbeat rather than a one-shot startup
write.
Resident cleanup stops orphan AMCL heartbeat processes before deleting the
status file. A stale heartbeat is not allowed to survive a failed startup and
write `AMCL_HEARTBEAT_PROCESS_NOT_ALIVE` into the next startup attempt.
Both `run_common_services.sh` and `run_navigation_runtime_services.sh` perform
that exact cleanup at startup, before the runtime context can be reused or AMCL
readiness can be evaluated.
Common autostart also stops stale `run_navigation_runtime_services.sh`
processes before relaunching resident navigation. Systemd restarting the outer
`docker exec` must not leave an in-container resident runtime orphan that is
later mistaken for a healthy reusable navigation runtime.
If that exact stale PID ignores INT/TERM, common startup kills the enumerated
PID directly; it still does not use broad `pkill -9` against ROS processes.
AMCL stop performs the same exact-PID cleanup for stale
`run_amcl_shadow_localization.sh` start/readiness/heartbeat runner processes so
old seed or readiness helpers cannot overlap the next resident startup.
When AMCL static standby is enabled, startup seeds AMCL from the bridge-accepted
`map -> base_link` pose without first blocking on `/scan` freshness
(`NJRH_AMCL_STATIC_STANDBY_SKIP_SCAN_FRESH_WAIT=true`). It still starts the C++
scan admission relay, but with
`NJRH_AMCL_STATIC_STANDBY_SKIP_SCAN_ADMISSION_READY_WAIT=true` it does not make
runtime ready wait for the relay's first admitted scan; AMCL correction remains
pending until real scans are admitted. This keeps startup from waiting tens of
seconds for the scan producer's startup header age to fall below the AMCL-only
1000 ms admission window, while the local costmap and diagnostics continue to
validate live `/scan` separately.
`run_common_services.sh` treats
`NJRH_RESIDENT_NAVIGATION_READY_TIMEOUT_SEC` as the soft startup SLA report
point. It does not kill a resident runtime that is still making progress at
that boundary; the hard cleanup boundary is
`NJRH_RESIDENT_NAVIGATION_READY_HARD_TIMEOUT_SEC`. Field measurements therefore
show when startup misses the two-minute target without creating a restart loop
that hides the real slow stage.
The external lifecycle helper activates `planner_server` first so the selected
floor global costmap subscribes to `/map` and starts its static-layer resize as
early as possible. For normal point navigation, `waypoint_follower`,
`waypoint_follower` is not part of the resident startup
hard gate. The field BT XMLs execute only `ComputePathToPose` /
`ComputePathThroughPoses`, `FollowPath`, and `PipelineSequence`, so behavior
recovery action servers are not required before the App can send a point goal.
This only changes lifecycle ordering and BT library loading; it does not change
the Nav2 planner/controller plugins or the command-chain readiness criteria.

AMCL resident warmup normally starts before the initial global localization
result has been accepted by `robot_localization_bridge`, but the warmup is
limited to AMCL lifecycle activation and scan-admission setup. The seed step
still waits until the bridge-owned `map -> odom` baseline exists, so AMCL does
not compete for TF ownership or replace the startup Isaac trigger.

Common-service startup keeps the driver-integrated JT128 accel pipeline alive
when `hesai_accel_driver_node`, the IMU remap, the pipeline supervisor, and the
`laser_scan_to_flatscan` helper are already unique and running. Fixed helper
settle sleeps default to short values (`NJRH_COMMON_PROCESS_START_SETTLE_SEC=0.2`,
`NJRH_OVERLAY_HELPER_START_SETTLE_SEC=0.2`) while the real readiness gates remain
the explicit local-state TF check, API/context check, Nav2 lifecycle state, and
AMCL readiness status.
When a valid last navigation map is available, common startup starts resident
navigation as soon as the pointcloud pipeline, chassis process, static TF
helper, and `robot_local_state` are ready
(`NJRH_RESIDENT_NAVIGATION_EARLY_AUTOSTART=true`). This keeps localization/Nav2
startup parallel with the remaining common API/safety services, but it no
longer lets Nav2/local-costmap prestart while `odom -> base_link` is absent.
The resident script still verifies `robot_local_state` before triggering initial
global localization or confirming the runtime context ready.

The runtime map context now carries `startup_stage` and `startup_elapsed_sec`.
`/api/v1/navigation/state` exposes the same context so the App can show whether
startup is waiting in local-state readiness, localization stack readiness, Nav2
prestart, initial global localization, AMCL resident warmup, Nav2 activation,
or AMCL background readiness.
Initial global localization is requested by
`call_global_localization_trigger.py`, a small rclpy client for
`/global_localization/trigger`. Startup no longer creates a fresh `ros2 service
call` process for every retry. The default process timeout is 75 seconds within
a 90 second outer window, but another wrapper request is allowed only when the
response proves `dispatch_state=not_dispatched`. The wrapper owns one immutable
force-accept/Isaac transaction. A queued result from a previous trigger is
drained inside that request; it does not move the arm time, end the request, or
cause startup to issue another trigger. Explicit results are never restamped:
they must pass historical odom TF and latest-odom freshness, but delivery age
alone is not a 5-second hard rejection.
The trigger is normally launched after localization-stack readiness and floor
context selection, so the first cold Isaac request runs with selected-floor
assets and input topics already ready. Localization-stack readiness requires the
wrapper service, Isaac grid-search service, selected `/map`, and `/flatscan`.
It no longer waits by default for a `/localization_result` publisher before
triggering; that graph probe is optional via
`NJRH_INITIAL_LOCALIZATION_REQUIRE_RESULT_PUBLISHER=true`. The trigger wrapper
is the hard gate because it waits for the real localization result, bridge
acceptance, and bridge-owned `map -> odom`. The background trigger path remains
as `NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=true` for controlled A/B,
and still joins before `initial_global_localization_ready` if enabled.
Common startup no longer blocks resident navigation autostart on a duplicate
`/flatscan` precheck by default. Set
`NJRH_COMMON_REQUIRE_FLATSCAN_BEFORE_RESIDENT_AUTOSTART=true` only for strict
field isolation. The resident localization gate remains the authoritative
publisher-owner confirmation
(`NJRH_INITIAL_LOCALIZATION_FLATSCAN_WAIT_SEC=20` by default). Phase S4 makes
that gate consumer-only: it trusts a fresh healthy supervisor status or waits
for the publisher, but never starts or restarts the common pointcloud owner. It
still does not trigger global localization until `/flatscan` is observable.
Common startup treats `robot_local_state` direct readiness as the hard local
state gate. The EKF starts in the background while the common sensors
initialize; `runtime_health_guard` starts after docking-sensor readiness and
then observes the already-starting local-state stream. A fresh-but-empty health
snapshot cannot overrule the direct gate by itself: while the EKF process is
alive, an independent bounded fresh-odom probe must also fail before the
producer-failure budget can advance.
`/api/v1/robot/pose` fails fast when that context is still `starting` or
`failed`; it no longer waits for the generic TF timeout before telling the App
that resident navigation is not ready.

The resident owner now treats the confirmed runtime-map context as part of the
ready commit, not as best-effort telemetry. It atomically writes
`state=ready, confirmed=true`, reads the file back through the exact-current-map
matcher, and only then sets its internal `runtime_ready=1` cleanup guard. A
write or read-back failure exits nonzero while the guard is still false. The
EXIT handler attempts to record `failed`, but that secondary write is
best-effort so an unwritable context path cannot prevent full Nav2,
localization, localizer, AMCL, and bridge cleanup.

If Nav2 lifecycle activation fails, the resident runtime writes context
`failed` for the navigation layer, stops the `run_nav2_navigation.sh` wrapper,
and then sweeps the standard Nav2 process names from the outer runtime as a
second cleanup pass. It then exits nonzero so the existing trap also tears down
the incomplete localization/localizer/bridge layer. A later
`POST /api/v1/navigation/start` therefore starts from a clean process set
instead of inheriting stale controller/planner/BT/lifecycle or localization
participants. Failure evidence remains in the runtime log and the failed
runtime-map context until the next controlled start or stop operation.

The selected-floor localization layer also avoids relying on
`lifecycle_manager_map` autostart for `/map_server` in the production
driver-integrated path. It now uses the repository-owned
`nav2_lifecycle_sequence.py` for the single `/map_server` lifecycle transition
instead of `nav2_util/lifecycle_bringup`, reducing cold ROS CLI polling while
keeping the same configure/activate contract. Field logs showed `/map_server`
could finish loading `nav_map.pgm` quickly but fail to return the lifecycle
`change_state` response
to the manager, leaving `/map` unpublished and causing resident startup to fail
at `localization_layer_started`. The runtime now launches `/map_server` without
the map lifecycle manager and activates it with the repository-owned one-shot
lifecycle helper. Its fallback state reads retain late Futures across retry
intervals instead of replacing them, so ordinary Fast DDS response latency is
not amplified into a 30-second configure wait. While that helper owns the
transition, the readiness gate only observes lifecycle active state or a
selected `/map` publication; it does not concurrently send configure/activate
requests.

The Jetson field runner does not use Humble `lifecycle_manager_navigation`
autostart for the core Nav2 nodes. That manager's lifecycle client uses a fixed
2 second `get_state` wait, which is too short while `planner_server` is
configuring the selected-floor global costmap and filters on Orin NX. The
runtime keeps `lifecycle_manager_navigation` present but starts the core nodes
with the repo-owned `nav2_lifecycle_sequence.py` helper and a bounded outer
timeout. The production default keeps the point-navigation core lifecycle
sequence serial (`NJRH_NAV2_LIFECYCLE_PARALLEL_CORE=false`): `planner_server`,
`controller_server`, `velocity_smoother`, `collision_monitor`, `behavior_server`,
`smoother_server`, and `bt_navigator` transition in a deterministic order. A parallel core sequence is
kept as an explicit A/B switch only because field cold-start testing showed it
can leave the resident startup stuck in lifecycle activation on the current
Orin NX image. `NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST=true` remains an
explicit A/B experiment only because it can delay ready time or time out under
startup graph load. Already-active nodes remain success, and transient
service-call failures are retried inside the bounded per-node deadline instead
of letting one rclcpp lifecycle helper abort the startup. When this external
bringup mode is enabled, the resident runtime uses a 210 second default Nav2
lifecycle-ready window so the outer supervisor does not kill the helper before
its own 180 second timeout. Cleanup explicitly terminates any in-flight lifecycle
helper process before sweeping standard Nav2 nodes.
The production helper also defaults
`NJRH_NAV2_LIFECYCLE_TRUST_CHANGE_STATE_RESPONSE=true`: a successful
`ChangeState` response is treated as the authoritative transition result instead
of immediately issuing another `GetState` poll. Field logs showed
`bt_navigator` could already be active and bonded while the extra `GetState`
response lagged tens of seconds under startup graph load. The initial state read
and the `ChangeState` success/failure response remain hard checks.

Host autostart uses `NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE=once` by default.
The runtime asset/log directory permission sweep is still available, but it is
not repeated on every boot after
`${NJRH_WORKSPACE_CONTAINER}/.njrh_runtime_permissions_ready` exists. This saves
the cold-start path from a recursive `find ... chmod/chown` pass while keeping
`NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE=always` as the exact old behavior.

Common startup uses `NJRH_COMMON_LOCAL_STATE_START_READY_MODE=endpoint` by
default. The common layer waits until `robot_local_state` has its required
processes and ROS endpoint, then lets resident navigation perform the final
fresh `/local_state/odometry` and `odom -> base_link` checks. This overlaps
local-state TF warmup with localization process loading without changing the
resident ready contract. `NJRH_COMMON_LOCAL_STATE_START_READY_MODE=fresh_tf`
restores the older common-layer fresh-TF wait.

`NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=true` starts the resident
navigation wrapper after driver/GS2/runtime-health startup and before the common
layer has waited for local-state endpoint/fresh TF. This is only a scheduling
overlap: the resident script still calls its local-state readiness gate before
initial triggered localization and before Nav2 lifecycle activation. Set the
flag to `false` to return to the older common sequence where resident navigation
starts only after common local-state readiness.

The production path retains the compatibility flag
`NJRH_NAV2_HELD_PRESTART_AFTER_LOCAL_STATE=true`, but local-state readiness is
only its first prerequisite. The resident runtime proves the complete
selected-floor localization stack and exact floor context, completes the
initial triggered localization, and requires the bridge-owned `map -> odom`
baseline before it starts the held Nav2 process tree. This keeps Nav2 process
and DDS-discovery load out of the map-server/Isaac request window and prevents
the result from aging out of the bridge's historical TF cache. Controller,
local-costmap, and BT lifecycle activation still wait until the held process
tree is ready. The foreground production path and optional background path both
consume the current wrapper's atomic held-ready record before starting the
lifecycle client; neither may infer launch readiness from the wrapper process
alone.
`NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=false` is the production
default. A/B testing showed that starting lifecycle activation immediately after
the localization stack is ready can make the `global_costmap` wait on the
missing `base_link -> map` transform before the bridge baseline is complete,
which is slower and less deterministic on the current Orin NX image. The
overlap remains available only as an explicit field experiment with
`NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=true`; final runtime
readiness still requires the trigger wrapper to accept localization, the
bridge-owned `map -> odom` to be live, and the Nav2 core lifecycle nodes to be
active. It does not send a goal, publish velocity, skip localization readiness,
or make App success visible early.
The serial production path keeps one repo-owned lifecycle helper for the startup
critical path rather than spawning competing lifecycle clients. Its node order
brings up the point-navigation core first: planner, controller/local costmap,
velocity smoother, collision monitor, `behavior_server`, `smoother_server`, and `bt_navigator`. The older
`nav2_util/lifecycle_bringup` path remains available with
`NJRH_NAV2_USE_REPO_LIFECYCLE_SEQUENCE=false` for A/B rollback.
`waypoint_follower` remains available
as launched Nav2 processes and are activated by a background repo lifecycle
helper, but they are not part of the resident startup ready gate for the current
point-navigation behavior trees.
`NJRH_NAV2_LIFECYCLE_PARALLEL_CORE=false` is the held-launch production default.
Enable `NJRH_NAV2_LIFECYCLE_PARALLEL_CORE=true` only for a controlled A/B trace
after the serial path is healthy. Roll back BT-only parallel behavior with
`NJRH_NAV2_LIFECYCLE_PARALLEL_BT=false`.
The parallel helper treats the lifecycle node's actual `active` state as
authoritative: if a per-node rclpy lifecycle helper hangs after the node is
already active, the resident script stops that helper and continues to
`bt_navigator` instead of failing an otherwise usable Nav2 stack.
The background lifecycle join path applies the same rule at the group level: if
planner, controller, velocity smoother, collision monitor, and BT navigator are
already active, resident startup stops the background helper and proceeds to the
global-costmap gate instead of waiting for the helper process to exit on its own.
`bt_navigator.plugin_lib_names` is explicit for the field runtime: it keeps a
bounded field list instead of loading Humble's full sample plugin list. This
does not change planner/controller plugin types or the command chain; it reduces
`bt_navigator` configure-time library loading without using the slower
ultra-minimal 4-plugin experiment measured on Jetson.

Production global costmap consumes only `KeepoutFilter` by default. The
selected-floor `filters/speed_mask.yaml` and dormant `SpeedFilter` parameter
block remain staged for explicit rollback/A-B, but the default global costmap
`filters` list excludes `speed_filter` and `standard_navigation.launch.py` does
not start `speed_filter_mask_server` or `speed_costmap_filter_info_server`
unless `NJRH_ENABLE_SPEED_FILTER=true`. Jetson startup traces showed
`/speed_filter_mask` lifecycle and delivery could delay Nav2 readiness, while
the current field maps do not require speed zones for startup validation.

Common startup also keeps the canonical `robot_local_state` endpoint/fresh-TF
probe bounded (`LOCAL_STATE_START_READY_TIMEOUT_SEC=12`,
`LOCAL_STATE_READY_RECHECK_TIMEOUT_SEC=4`). Resident navigation still performs
its own local-state and `map -> odom` gates; the common phase does not spend
tens of seconds waiting on a transient fresh-TF sample before resident startup.

Initial global localization is still a hard startup gate. The bounded outer
window (`NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT`, default 90s;
per-process default 75s) may create another wrapper request only for a proven
pre-dispatch readiness/service race. After dispatch, a result stamped before
the immutable force-accept arm is drained without ending the request; the
wrapper waits for the current-arm result and never creates a one-result-behind
trigger loop. Any post-dispatch failure is terminal unless exact newer bridge
evidence reconciles it. Other bridge rejections remain terminal. Common
startup also removes stale diagnostics plus exact AMCL lifecycle
`ros2 service call /amcl/(change_state|get_state)` clients before resident
navigation starts, so one-shot CLI clients cannot survive into the next boot and
perturb DDS graph discovery.

Hardware validation:

- Reboot the Jetson and compare `STARTUP_STAGE` timestamps in
  `scripts/jetson/runtime_overlay/web_dashboard/runtime_logs/resident_navigation_runtime.log`.
- Confirm `/api/v1/robot/pose` returns quickly with a context detail while the
  runtime context is `starting` or `failed`.
- Confirm AMCL can warm up during Nav2 activation and reach `AMCL_READY`
  after or before the runtime context becomes confirmed `ready`, without
  changing `map -> odom` ownership.
- Confirm `nav2_layer_prestarted` does not set runtime context `ready` by
  itself. `nav2_layer_ready` may confirm runtime readiness when bridge
  `map -> odom` is fresh and `NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY=false`;
  with that rollback flag set to `true`, `amcl_tracking_ready` must appear
  before the confirmed ready context.
- Confirm `/api/v1/status.localization.amcl_status_file_stale=false` remains
  stable after startup; AMCL ready must not decay a few seconds after the
  startup script writes the context.
- Confirm the external Nav2 lifecycle node list activates `planner_server`
  before `controller_server`, starts in the background after
  `nav2_layer_prestarted` only after the matching
  `NJRH_NAV2_HOLD_READY_FILE` is written by the current `run_nav2_navigation.sh`
  wrapper, and does not include `waypoint_follower` in the point-navigation
  startup hard gate.
- Confirm common startup logs `runtime health confirms local_state_ready before
  resident navigation autostart` before `starting resident_navigation_runtime`.
- Confirm the localization layer logs `localization map_server
  lifecycle_bringup: map_server active` and `/map` matches the selected
  `NAV2_MAP_YAML` before initial global localization.
- Confirm default startup logs show `skipping /localization_result publisher
  pre-gate`; if `NJRH_INITIAL_LOCALIZATION_REQUIRE_RESULT_PUBLISHER=true` is
  explicitly set for A/B, startup may wait for that publisher before triggering.
- Confirm common startup logs show `/flatscan publisher ready before resident
  navigation autostart`; resident readiness should normally only perform the
  second publisher-owner confirmation.
- Confirm startup logs show either one accepted global localization transaction,
  same-transaction pre-arm drain messages before the current-arm accept, or
  bounded retries only for failures marked `dispatch_state=not_dispatched`.
- Confirm default startup reaches `localization_stack_ready`, followed by
  `floor_asset_context_verified`; then it logs `initial global localization
  trigger is starting after full localization-stack and floor-context
  readiness`. `initial_global_localization_ready` must appear only after the
  serial trigger has produced a bridge-owned `map -> odom`, and before
  `nav2_layer_prestarted_held`. When
  `NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=true` is explicitly set for
  A/B, `initial_global_localization_trigger_started` must appear after
  `floor_asset_context_verified` and before the background trigger is joined.
- Confirm default startup logs do not launch `speed_filter_mask_server`; enable
  `NJRH_ENABLE_SPEED_FILTER=true` only for a deliberate speed-zone A/B test.
- If Nav2 activation fails, confirm the incomplete occupancy-localization,
  `robot_localization_bridge`, `controller_server`, `planner_server`,
  `bt_navigator`, and lifecycle-manager processes are all gone before retrying.
- On a normal full-service hardware restart, confirm the exact selected-map
  context reaches `state=ready, confirmed=true` before the API reports the
  runtime ready. In an isolated non-motion test container, inject an unwritable
  `NJRH_RUNTIME_MAP_CONTEXT_FILE` and confirm startup exits nonzero and leaves
  no Nav2, occupancy-localization, localizer, AMCL, or bridge child process.
