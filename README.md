# multi_floor_delivery_robot

ROS 2 Humble multi-floor indoor/outdoor delivery robot navigation stack scaffold for Jetson Orin + JT128 + Ranger Mini 3.

## Current Status

BMS docking-contact evidence now distinguishes received docking commands from
internally generated stop commands. The isolated candidate preserves confirmed
dock protection and existing release rules; it was activated on 2026-09-17.
See [scope, evidence and validation](src/robot_safety/docs/bms_contact_history.md).

Elevator adapter executor recovery is scoped to its ROS worker and internal
failure/stop handling. See [audit and tests](docs/elevator_adapter_executor_recovery.md);
no whole-service restart is implied by the source change.

Fixed-distance auto-undock accepts an unknown dock ID without inventing one.
The existing departure, post-undock localization and original navigation-goal
sequence is retained; no-motion dock-zone reconciliation is unchanged. See
[scope and deployment record](docs/pre_navigation_dock_interlock_recovery.md#fixed-distance-undock-identity-correction-2026-09-16).

Ordinary MPPI now enables native Humble `ConstraintCritic`; the local costmap
consumes the existing keepout mask with matched post-filter inflation. Motion
limits, footprint, goal tolerances and elevator test policy are unchanged.
Configuration activation still needs a user-controlled navigation-service
restart. See [scope and isolated validation](src/robot_nav_config/docs/local_keepout_and_constraints.md).

App navigation resume now keeps slow map/Isaac initialization in `starting`,
owns a fresh startup CPU session, and stages Nav2 after map/Isaac initialization
while overlapping configure-all/activation with the localization request.
Terminal App localization failures finish instead of waiting indefinitely.
Optional pre-trigger baseline observation defaults to off; localization
acceptance and final readiness predicates are unchanged. See
[navigation resume repair and pending hardware timing](docs/navigation_resume_startup.md).

API release builds must use one complete approved source/header snapshot and
a fresh build directory. Mixing historical object files can corrupt private
mapping state and stall all HTTP workers. See
[coherent API build and isolated query regression](docs/api_coherent_build.md).

Ordinary-navigation terminal verification preserves correction failure history
but revalidates the current strict pose and stop evidence after successful
same-goal recovery. See [scope and isolated tests](docs/navigation_terminal_revalidation.md).
The 2026-09-17 candidate is activated after the authorized whole-service restart;
binary identity and stationary readiness are verified, not physical acceptance.

Relocalization completion no longer waits for navigation admission. A current,
accepted and settled canonical TF result completes localization even while a
floor transaction keeps `safe_for_goal_start=false`. Navigation's own checks
remain unchanged. See [scope and validation](docs/relocalization_completion_responsibility.md).

Local-state cold startup now shares one 30-second readiness deadline across
IMU preparation and the owned EKF endpoint check. Process checks verify the
actual executable; failed-attempt logs are retained before retry. No sensor,
EKF, CPU or motion policy changes. See [scope and acceptance](docs/local_state_startup_budget.md).

Mapping startup has a scoped orchestration repair: reuse DDS discovery, start
the private odom bridge early, combine scan/TF checks, and keep status responsive
during shutdown. Algorithm/CPU/sensor parameters are unchanged. The API candidate
requires a separately authorized activation; see
[mapping startup repair](docs/mapping_startup_latency.md).

The CPU pointcloud candidate fuses the validated canonical axis remap with
normalized XYZI cache filling, preserving the full-density/full-fields trunk,
scan behavior, QoS and publish-before-cache-exchange ordering. This stage is
candidate deployment, not a full-runtime restart or hardware acceptance. Twelve
targeted tests passed; isolated algorithm CPU median fell from 417 to 247 us per
40000 points, not whole-process savings. See [scope and acceptance](docs/pointcloud_cpu_fused_normalization.md).

The IMU conversion/filter chain now has a driver-owned, same-process transport
option with independent executors; external interfaces and 100 Hz navigation
output are unchanged. See [IMU IPC scope and acceptance](docs/imu_intra_process_pipeline.md).

IMU remap/filter arithmetic now avoids overwritten covariance calculations and
reuses unchanged rotation/covariance results. Per-sample TF lookup, bias learning,
frequencies and timestamps stay unchanged. See
[IMU optimization scope](src/robot_local_state/docs/imu_arithmetic_reuse.md);
whole-process CPU benefit requires separate post-restart measurement.

Isaac request admission now checks original FlatScan header age as well as
receipt age, and never dispatches after failed post-arm input validation.
Startup frequency fluctuation is not a new failure criterion; existing input
windows and pre-dispatch startup retries remain unchanged. See
[fresh-input dispatch](src/robot_global_localization/docs/fresh_input_dispatch.md)
for isolated regression coverage and separate activation/acceptance status.

完整导航冷启动支持临时八核调度，结束后恢复原五核逐模块分配，不改变导航算法、
启动顺序或就绪判据。见 [启动 CPU 调度](docs/startup_cpu_boost.md)；实机提速待授权重启验收。

Final startup context observation can prewarm its native client during Nav2
activation. READY still requires a post-commit, post-service DDS status sample
and the original exact-map/sequence commit; the Python compatibility path stays.
See [context observer overlap](docs/restart_latency_40s.md#final-context-observer-overlap).

Held Nav2 launch-receipt waiting now runs inside the existing lifecycle worker,
so it no longer serializes the initial Isaac trigger. Launch ownership and final
navigation readiness are unchanged; whole-restart timing still needs acceptance.
See [startup overlap scope](docs/restart_latency_40s.md#held-launch-wait-overlap).

Full-runtime restart optimization now overlaps map/bridge and early AMCL
initialization, and reuses lifecycle service endpoints. The operator's target
is under 40 seconds including shutdown, verified over five runs; it is not yet
accepted. Early AMCL progress reporting no longer blocks initialization when
the status observer is cold; final READY still requires committed evidence.
One-shot startup clients omit unused parameter-service and log endpoints,
while keeping their existing service calls, TF checks and ROS clocks.
See [restart latency work](docs/restart_latency_40s.md).

The one-shot lifecycle-active check retains late GetState replies within the
existing total budget, avoiding repeated requests that discard valid active
responses. Stationary full-restart acceptance remains pending.

AMCL input preparation now combines map/scan/TF observation in one bounded
context, retaining partial checkpoints; seed requests defer when their full
client budget does not fit. No readiness condition or CPU allocation changes.
See [AMCL startup verification](docs/amcl_startup_sequence.md); full-restart
timing acceptance remains separate from isolated tests and deployment.

Systemd cold startup now waits for the canonical scan owner with one continuous
observer, avoiding a scan-enable request caused by an undiscovered graph.
Mapping/API resume behavior is unchanged; restart timing acceptance is pending.
See [cold-start scan ownership](docs/navigation_scan_cold_start.md).

AMCL startup now resumes confirmed preparation for the same process/map/startup
owner and uses one bounded completion path; seed retries do not repeat warmup.
Offline regression passed; full-service restart timing acceptance is pending.
See [AMCL startup sequence](docs/amcl_startup_sequence.md).

The runtime health observer and snapshot queries now have C++ implementations
with 1Hz sampling and a separate 3-second no-update grace period. Control-loop
rates and safety watchdogs are unchanged. Deployment acceptance is recorded in
[the runtime health design](docs/runtime_health_cpp.md).

Management checks now use native process audits and the existing C++ health
observer. Stationary 300-second measurement, including reaped child CPU, fell
from 54.89% to 4.70% of one core. AMCL status and FlatScan metadata retain their
public contracts without steady-state shell/DDS polling. See
[runtime management design](docs/runtime_management_observer.md) for scope and
deployment acceptance; navigation and safety control rates are unchanged.

The official-source Fast DDS 2.6.12 **system-package** upgrade is installed in
NJRH-car, without a second runtime prefix or changes to Humble/RMW. Isolated
regression/transport tests passed, but full startup acceptance failed when the
existing startup TF check caused localization teardown. Navigation recovery is
not yet accepted. See [evidence and status](docs/fastdds_2612_system_upgrade.md).

`robot_api_server` now defaults unspecified single-config builds to
`RelWithDebInfo`, preserving explicit build-type selections. This is a build-only
optimization; no API contracts or robot control policies change. Candidate
deployment and live acceptance are tracked separately in
[the API build optimization record](docs/robot_api_optimized_build.md).

JT128 vendor point parsing supports the Orin CUDA production build and defaults
to GPU in the runtime-generated driver configuration. This does not change
pointcloud/IMU transport, timestamps or the five-core CPU allocation. See
[CUDA scope, build and acceptance](docs/jt128_cuda_parsing.md).

An opt-in five-core navigation affinity profile is available; the default remains
the existing site allocation. It preserves field overrides for restoration and
does not modify the arm black box or navigation algorithms. Synchronizing the
profile is not activation or hardware acceptance. See
[five-core scope, selection and tests](docs/navigation_five_cpu_profile.md).
The active field trial keeps navigation on CPU0-4; it is not rolled back to eight
cores. The grouped candidate separates driver/pointcloud work, the control/TF
chain and navigation computation, including sensor-wrapper initialization.
Earlier grouped trials failed sustained startup/frequency acceptance, even when
one bounded capture approached the configured rates. Candidate T places the
official JT128 driver on CPU3 and pointcloud/scan workers on CPU1,4. Both sensor
startup entries use CPU3 before child-specific worker masks take effect; the
whole lidar chain does not share CPU3. Other compute/helpers use
CPU0,1,4; IMU/EKF/bridge remain on CPU2 and control on CPU1. System/API/supervision
also uses CPU0,1,4, while CPU2 is excluded from that pool. This is process
placement, not kernel/IRQ isolation. O completed AMCL seeding but scan remained
12.04 Hz with substantial driver runqueue wait. P's added driver CPU3 access was
removed after repeated startup failures; it was not accepted. The measured IMU
input arrived after its readiness deadline. Q moves only the non-legacy sensor
bootstrap off the congested CPU1,3 pool; startup order and waits are unchanged.
Q restored first-start readiness, but scan measured 12.606 Hz. R keeps that
bootstrap correction and tests driver runtime headroom only; it was not accepted.
S swaps Q's physical CPU3/4 roles to compare cluster placement, without changing
CPU frequency or power settings. Sustained rates and bridge continuity remain unverified.
T also places existing FlatScan graph/rate CLI checks in the system pool instead
of the driver core. T is not accepted: scan reached 15 Hz in a capture without
working map-to-odom or AMCL, and Nav2 lifecycle activation failed. This is not
a full-load result or proof of five-core whole-chain stability.
The checks' timing and result handling are unchanged.
Five-core sustained rates, repeat startup and moving control remain unaccepted.
Read-only evidence identified concentrated eth1 IRQ work on CPU0; no IRQ/RPS/XPS
changes have been made. See the profile document for placement and evidence.

The API-only current-state floor interlock is deployed on Jetson. Historical
`FAILED_LOCKED` no longer permanently overrides current typed evidence;
active-switch exclusion and current-invalid navigation rejection remain.
This does not change undock map admission or other floor/elevator cleanup code.
See [deployment scope and verification](docs/floor_interlock_deployment_20260910.md).

Manual relocalization now distinguishes its own settled Isaac result from
unrelated AMCL candidate rejections. Request-scoped late completion prevents
false 503 responses from becoming permanent 409 admission locks; the App no
longer labels every 409 as a floor switch. See
[confirmation protocol and validation](docs/relocalization_trigger_confirmation.md).
Source changes, isolated verification and production activation are separate steps.

Controlled undock now accepts the existing fresh safety memory latch. Failed or
cancelled reverse attempts no longer count as successful undock when clearing it.
See [dock-memory recovery](docs/pre_navigation_dock_interlock_recovery.md#memory-only-undock-admission-and-completion).

336L common startup always enters its driver wrapper; the existing owner lock
prevents duplicate camera instances. API frame names no longer skip the camera.
Docking still starts last. See [ownership and validation](docs/orbbec_gemini_336l_container.md#336l-startup-ownership).

Cabin center -> cabin panel has an isolated 0.20 m/s lateral speed profile;
other elevator legs retain 0.40 m/s. See
[scope and validation](src/robot_nav_config/docs/elevator_scoped_motion.md#cabin-panel-speed-isolation).
Activation requires a separately authorized full-chain restart.

The [floor-switch failure lifecycle correction](docs/floor_failure_terminal_cleanup.md)
separates terminal transaction cleanup from map localization readiness. It
addresses the retained bridge/API/elevator failure coupling that the preceding
offline asset-edit admission change did not solve. Candidate verification and
deployment are tracked separately; no hardware acceptance is implied.

The [floor-failure asset admission fix](docs/floor_failure_asset_admission.md)
separates offline point/configuration editing from retained runtime failures.
It preserves motion protection and active-switch exclusion; 16 isolated C++
regressions and the real isolated API admission test pass. The precise two-object
API update has been deployed and loaded after a complete runtime restart.

Cold-start map loading and Isaac initialization now start alongside the common
sensor chain. Initial relocalization waits for the current Isaac process's
post-GXF startup event, target map and FlatScan owner; the existing wrapper
still validates fresh input before dispatch. See
[parallel localization startup](docs/isaac_parallel_startup.md) for the exact
scope, isolated tests and pending full-runtime restart acceptance.

Nav2 process preload now overlaps Isaac startup and initial localization;
the systemd runner, installed runtime.env and installation defaults must all
set `NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=true` for this to take effect.
lifecycle activation still follows accepted localization. An unsuccessful first
localization keeps both processes alive and waits for a later explicit result,
without automatically retriggering Isaac. See
[startup decoupling and pending hardware acceptance](docs/phase_s3_fast_resident_navigation_startup.md).

Common startup overlaps static TF, JT128, Ranger and EKF. Common alone owns
safety/floor/mode helpers; API starts before camera initialization. The docking
camera, perception and manager start last, after HTTP is responsive and the
first navigation startup finishes or enters explicit localization waiting.
Dock observation timeout is docking-only degradation, not a whole-chain exit.
No GPU/CPU allocation, localization algorithm or motion policy is changed.
Full-service restart/timing acceptance is pending. See
[docking-last startup and acceptance](docs/startup_docking_last.md).

The [project gate inventory](docs/project_gate_inventory.md) documents each
audited admission, motion, localization and handoff mechanism: its purpose,
blocking conditions, release behavior, configuration status and source entry.
It distinguishes enabled defaults, conditional paths and retired mechanisms;
the inventory is not a snapshot of the robot's live blocking state.

The [gate audit remediation record](docs/gate_audit_remediation_plan.md) tracks
the eight findings. The first fix separates BEGIN from target-request dispatch:
proven pre-dispatch failure restores the source without a retained failure lock.
The floor-manager-only candidate is deployed on Jetson; full-runtime restart
has loaded the verified executable. Real floor-switch fault/retry acceptance
is still pending; no movement or switch was triggered during deployment.
Restart verification found an initial Isaac result timeout: the API is online,
but navigation remains unready until localization establishes `map -> odom`.

Manual map switching no longer requires the source map to be localized or ready.
Target loading/localization remains authoritative; no navigation start/stop was
added. See [source-independent switching](docs/map_switch_source_independence.md)
for scope, isolated verification and the separate runtime-startup limitation.

Ordinary persistent navigation recovery is an **offline candidate**, not yet
activated on the robot. It retains the original task after verified progress
failures, backs off repeated attempts, and communicates goal-correlated waiting
to the API. No physical acceptance or production deployment is claimed. See
[candidate scope and acceptance](src/robot_nav_config/docs/ordinary_navigation_recovery.md).

Ordinary navigation now reconciles the final BMS docking memory interlock before
submitting a Nav2 goal. Live contact, dock-near, and uncertain cases reuse the
existing standard controlled-undock path; only a fresh, exact-map proof that the
robot is at least 1.5 m outside the commissioned dock can clear stale memory
without motion. Missing safety state and stale evidence retain their existing
checks; unresolved dock identity blocks only no-motion reconciliation, not
fixed-distance physical departure. See
[pre-navigation dock interlock recovery](docs/pre_navigation_dock_interlock_recovery.md).

The Ranger FollowPath candidate now predicts measured chassis response inside
Humble MPPI, in `robot_nav_config/chassis_dynamics`: existing smoother dynamics,
CAN-fitted longitudinal response and steering response precede trajectory scoring.
It adds no stop gate or micro-motion policy and does not change terminal accuracy,
elevator-specific control or docking contact control. See
[chassis response model](src/robot_nav_config/docs/chassis_response_model.md)
for source-data hashes, held-out replay, tests and pending authorized activation.

MPPI output finalization also restores active speed and motion-model constraints
after Humble's signed filter, before command selection/history commit/horizon
shift. This closes filter-generated reverse and speed-limit overshoot without
changing chassis stop-centering or adding a low-speed stop gate. See
[post-filter constraints](src/robot_nav_config/docs/chassis_response_model.md#post-filter-output-constraints).

The 2026-09-08 navigation clearance candidate separates the unchanged
`0.39/0.28 m` scan mask / padded footprint, `0.47/0.36 m` StopZone, and
`0.52/0.41 m` finite MPPI planning preference. Local repair shares the preference
with an original-clearance fallback. No new velocity gate or expanded footprint
clearing is introduced. See [planning clearance](src/robot_nav_config/docs/planning_clearance.md)
for scope, isolated tests and pending authorized restart / hardware verification.

`robot_safety` now recognizes spin-to-drive handoff from fresh actual chassis
mode feedback, not low-speed Twist thresholds. Slow Ackermann obstacle bypass
cannot arm spin settling; real spin-tail checks and the existing timeout remain.
See [actual-mode handoff](src/robot_safety/docs/spin_to_drive_actual_mode.md)
for isolated regression tests and the separately authorized hardware acceptance.

Ordinary Ranger navigation now has one internal progress-failure recovery:
retain the App/NavigateToPose goal, replan, explicitly re-evaluate startup
alignment, and retry FollowPath once. Predock/elevator trees, terminal accuracy,
obstacle maps and the velocity chain are unchanged. Deployment requires the
next authorized complete-runtime restart; supervised hardware acceptance is
separate from isolated software tests. See
[ordinary navigation recovery](src/robot_nav_config/docs/ordinary_navigation_recovery.md).

This repository currently contains the phase-ordered baseline required by `02_实现任务清单.yaml`:

- `P0`: local car-project reuse scanning, TF audit tooling, canonical TF policy
- `P1`: workspace skeleton, dependency resolution baseline, initial package scaffolds
- `P2/P4/P5`: first-round wrapper and bringup scaffolds for the critical path packages
- `P6`: elevator topology/FSM safety core plus a versioned building-level
  elevator configuration manager in `robot_api_server`. The manager saves
  drafts, validates exact floor/map assets, publishes immutable releases, and
  rolls back by creating a new auditable release. A fail-closed runtime loader
  now pins one immutable release, verifies all six release files, and freezes
  the selected elevator and exact source/target map bindings. Schema v2 freezes
  the legacy `hall_call`/`landing`/`cabin` poses; schema v3 freezes
  `hall_call`/`landing`/`cabin`/`cabin_panel` plus both explicit panel sides.
  Schema-v1 five-point/threshold releases
  remain integrity-checkable history but cannot execute or become rollback
  targets. Map bindings use a persistent positive `asset_epoch` plus a
  canonical `sha256:<64 lowercase hex>` under the fixed
  `njrh-map-asset-bundle-v1` contract.
  Publishing remains asset-only (`runtime_applied=false`). The production
   overlay now has a manual-confirmation commissioning adapter that can bind a
   release, run elevator-owned Nav2 effects, and execute the strict atomic
   floor-switch transaction. Nearby (`<=2.5 m`) elevator-owned hall-call goals
   now reuse the scoped four-wheel planner/controller, so a blocked startup
   spin can become checked translation/reverse to a clear staging pose; far
   hall-call goals and all ordinary App navigation remain on ordinary Nav2.
   Elevator recovery is phase-aware: a pre-transition failure never depends on
   the floor Action server, while any durable floor-transition intent retains
   the full cancel/terminal proof barrier. If a complete-runtime restart has
   already rebound a pre-transition failure to the other frozen elevator
   endpoint, cleanup accepts that endpoint only after its exact map identity,
   localization/TF health, runtime-idle state, and dual-odometry stop are all
   proven; unrelated floors remain rejected.
   The production elevator execution wrapper now runs with the persistent
   recovery latch permanently disabled. A failed/cancelled elevator task still
   commands stop and reconciles owned runtime resources, but cleanup continues
   automatically and can no longer terminate as `LOCKED`, require an App
   recovery acknowledgement, or install
   `ELEVATOR_EXECUTION_RECOVERY_REQUIRED` as a global API interlock. Historical
   retained journals are converted to automatic cleanup on the next complete
   runtime start. Active-task exclusion, estop, watchdog, `robot_safety`, and
   the final command arbitration chain are unchanged.
   Automatic arm/vision evidence and unattended
  cross-floor missions remain disabled.
- current local reuse source: `D:\codespace\car`
- current Jetson host context: `nvidia@192.168.31.23:/home/nvidia/workspaces/njrh-v3/workspace1`

The implementation intentionally prioritizes:

1. Local car-project reuse before any network fetch
2. Canonical TF tree governance
3. Wrapper isolation for JT128 / FAST-LIO2 / PGO / Isaac localizer
4. Single `map->odom` and single `odom->base_link`

The repository now also carries the occupancy-builder extension required by the v3 update:

- `reports/occupancy_builder_design.md`
- `src/robot_occupancy_builder`
- `docs/occupancy_builder_workflow.md`
- live draft contract: `JT128 + /mapping/frontend_pose -> /mapping/draft_map`
- release rebuild contract: `raw bag + optimized trajectory -> nav_map + localizer_map`

## Jetson Runtime

The current field runtime path reuses the validated Jetson `car` stack inside a dedicated `NJRH-car` container instead of introducing a second temporary operator UI.

Production provisioning is now release-driven and fail-closed. A clean golden
Jetson/CI build generates deterministic runtime/site payloads, a release lock,
and an Ed25519-signed device manifest. After cloning the matching release tag,
the target command is:

```bash
bash scripts/jetson/provision_njrh.sh deploy \
  https://github.com/Neepi1/CAR.V1/releases/download/<release-tag>
```

Deployment verifies the fixed Orin NX/JetPack identity, passive Ranger CAN and
JT128 traffic, the unique USB3 Orbbec camera, NVIDIA runtime, all artifact/tree
digests, 28 ROS packages, and the reviewed test allowlist before one atomic
release switch. Downloaded metadata is verified against the out-of-band factory
key and the exact cloned commit before it is persisted or any device API secret
is generated. Payloads over 1900 MiB are deterministically split into
individually signed-size/hash-bound GitHub Release assets and verified again
after streaming reassembly. The signed manifest may pin an exact Orbbec serial or request
one-time `AUTO_ENROLL`; after `/etc/njrh/device-identity.json` is created, a
serial or platform identity change fails closed and is never rebound
automatically. A release with validated site assets finishes in `READY_LOCKED`;
without them it finishes in `READY_NO_MAP_LOCKED`. Both states keep services
disabled and motion locked, and the no-map state cannot pass hardware
acceptance or activation. The generated runtime remains pinned to
`ranger_lattice + wheel_spin_imu + gated AMCL` and keeps
`Nav2 -> velocity_smoother -> collision_monitor -> robot_safety -> /cmd_vel`.

The scoped exception covers every post-call elevator leg: source landing/entry
staging, cabin entry, cabin-center to panel, panel back to cabin-center, and
exit to the target landing. These exact elevator intents use an unchecked
bounded Nav2 path and disable controller costmap clearance checks. Under an
exact, fresh elevator transaction permit in `ELEVATOR_WAIT` (post-call staging)
or `DOORWAY` (cabin entry/exit) mode, `/cmd_vel_nav`
goes from `velocity_smoother` directly to `robot_safety`. Only the pre-call
hall approach and ordinary navigation keep the canonical collision-monitor
chain. `robot_safety` still enforces the hold/lease, estop, localization,
watchdog, speed, reverse, and lateral gates.
See [post-call collision scope](src/robot_safety/docs/elevator_post_call_collision_bypass.md)
for the mode-mismatch fix, isolated regressions, and remaining hardware checks.
The scoped safety patch has been deployed and restarted; the exact artifact,
rollback backup, and startup verification are recorded in
[deployment record](reports/elevator_post_call_collision_bypass/README.md).
See [docs/production_jetson_provisioning.md](docs/production_jetson_provisioning.md).

- Jetson host workspace: `/home/nvidia/workspaces/njrh-v3/workspace1`
- Jetson upstream asset workspace: `/home/nvidia/workspaces/isaac_ros-dev`
- upstream compatibility mount inside container: `/workspaces/isaac_ros-dev`
- runtime container: `NJRH-car`
- runtime image default: `njrh-car:latest`
- runtime image build network mode: `host`
- runtime image fallback when Jetson rebuild is blocked: `isaac_ros_dev-aarch64:latest`
- dashboard URL: `http://192.168.31.23:2048` when explicitly started for debug only
- operator guide: `docs/jetson_njrh_container_runtime.md`
- runtime orchestration owner: this repository's `scripts/jetson/runtime_overlay`
- CAN up/down helpers are now present in the overlay as `scripts/jetson/runtime_overlay/scripts/bringup_ranger_can.sh` and `shutdown_ranger_can.sh`, matching the reused dashboard's expected filenames
- live 2D mapping owner: `slam_toolbox` launched by this repository's `scripts/jetson/runtime_overlay/scripts/run_projected_map.sh`, exposed to the reused dashboard through the compatibility `/api/projected_map/latest` endpoint and explicitly reusing the existing TF tree without injecting extra static sensor TF
- web `/api/mapping2d/start` now enters that same repository-owned `slam_toolbox` chain directly and no longer falls back to the historical `run_jt128_2d_mapping.sh` / cartographer path
- current live 2D source for the operator view: mapping-owned FAST-LIO2 publishes its deskewed/body-frame cloud on `/mapping/fastlio/cloud_registered_body`; `nav_cloud_preprocessor` levels and self-masks it, and `pointcloud_to_laserscan` slices it directly onto canonical `/scan`. The same FAST-LIO2 instance owns mapping odometry and the private `/tf_slam2d` edge (`mapping_odom -> base_link`).
- repository-owned 2D scan ownership contract: canonical `/scan` always has exactly one publisher. Navigation uses `pointcloud_accel_axis_node`; mapping unregisters only that node's LaserScan publisher while leaving `/lidar_points` alive, then admits `pointcloud_to_laserscan` as the sole owner. Mapping exit reverses and verifies the handoff. There is no `/mapping/scan`, `/mapping/scan_raw`, scan relay, or timestamp rewrite.
- `slam_toolbox` stays on canonical `/scan` with local scan matching enabled. After the SM1 no-loop field run produced visibly worse accumulated yaw distortion, `do_loop_closing=true` was restored as a single-variable correction; Huber loss and the conservative loop thresholds remain enabled. The other graph-admission settings are intentionally unchanged by this correction and require separate evidence before any further edit. The file value is loaded by the next mapping start; see `src/robot_fastlio_mapping/docs/slam_toolbox_direct_scan.md`.
- Mapping startup proves the resident scan owner, TF and local odometry, then starts and pair-validates mapping FAST-LIO2 cloud/odom. It unregisters resident `/scan`, proves zero publishers, starts the corrected-cloud slice, and proves exactly one `pointcloud_to_laserscan` publisher before the original-stamp TF gate passes. Cleanup stops the mapping slice before restoring and proving the navigation owner; navigation startup repeats restoration as a crash-recovery guard.
- Canonical `/scan` remains a single direct topic. The JT128 vendor publisher and resident `/lidar_points` owner now share a stable Fast DDS `UDPv4 + SHM` participant profile, while mapping-owned FAST-LIO2, `nav_cloud_preprocessor`, and `pointcloud_to_laserscan` retain the mapping-session `UDPv4 + SHM` profile with 128 MiB segments. Same-container full-size clouds, including `/jt128/vendor/points_raw`, therefore negotiate SHM; UDP remains available for remote/legacy readers and small-message consumers. IMU remap, `slam_toolbox`, TF/odom bridges, API, Nav2, and probes keep their normal UDP participant settings. This changes transport only, not point density, QoS, timestamps, TF ownership, scan geometry, or the unique `/scan` handoff. The 2026-08-20 stationary field check measured FAST-LIO corrected-cloud input at `17.95 Hz` and mapping `/scan` at `17.10 Hz` over 60 seconds, with zero duplicate/regressed stamps; a second 30-second check measured `17.56 Hz` with a maximum wall gap of `0.252 s`. See `src/robot_fastlio_mapping/docs/slam_toolbox_direct_scan.md`.
- Fast-LIO runtime now accepts only the canonical sensor topics `/lidar_points` and `/lidar_imu` with `sensor_frame_id=lidar_link`; the repository-owned runtime no longer permits the old Fast-LIO-only remap path
- The shared FAST-LIO2 profile keeps `/cloud_registered_body` and `/Odometry` enabled for mapping, uses full JT128 input density (`point_filter_num=1`) with `max_iteration=4`, consumes canonical `/lidar_points` directly with best-effort/depth `1`, and disables Path/Laser-map publisher work (`path_en=false`, `map_en=false`) to reduce frontend load without changing the canonical sensor installation TF. Common services default `NJRH_FASTLIO_AUTOSTART=false`; FAST-LIO2 is started by `run_projected_map.sh` only while mapping is active, unless an explicit diagnostic FAST-LIO local-state mode is selected.
- The patched upstream FAST-LIO2 build is deployed as a writable overlay at `${NJRH_FASTLIO_PATCHED_OVERLAY:-/workspaces/njrh-v3/workspace1/.runtime/fast_lio_overlay/install}` and sourced after the reused upstream installs, so `ros2 run fast_lio fastlio_mapping` resolves to the transport-hardened build without overwriting the root-owned upstream install.
- The patched upstream `jt128_nav_tools` build is deployed the same way at `${NJRH_JT128_NAV_TOOLS_PATCHED_OVERLAY:-/workspaces/njrh-v3/workspace1/.runtime/jt128_nav_tools_overlay/install}`. `common_env.sh` sources it after the reused upstream installs, so `nav_cloud_preprocessor` uses the best-effort/depth `1` pointcloud QoS patch without overwriting the root-owned upstream install.
- repository wrapper contract now matches the Jetson runtime: `robot_fastlio_mapping` is canonical-only and no longer exposes `hesai_lidar_fastlio` as a runtime fallback
- JT128 point-cloud web view now defaults to the chassis frame `base_link`; raw sensor coordinates remain available only as an explicit debug toggle
- JT128 ingress follows the validated project path: the driver publishes vendor raw `/jt128/vendor/points_raw` and `/jt128/vendor/imu_raw`, then repository-owned remap helpers rotate raw axes into the canonical sensor frames and publish `/lidar_points(lidar_link)` and `/lidar_imu(imu_link)`. FAST-LIO2 now subscribes to `/lidar_points` directly; the old `/lidar_points_fastlio` identity alias is not part of the default runtime because it adds another full-size pointcloud copy and can create backpressure under load. The static TF chain keeps only the physical installation pose from `base_link` to `lidar_link` / `imu_link`.
- the canonical JT128 ingress remains repository-owned, and runtime now requires the compiled `robot_hesai_jt128` pointcloud and imu remap nodes; Python remap fallbacks were removed so a missing binary fails fast instead of degrading runtime performance
- The C++ pointcloud remap publishes `/lidar_points` as the production full-density trunk. Production navigation no longer derives local obstacle PointCloud2 branches (`/_internal/lidar_points_local`, `/lidar_points_nav`, `/points_nav`, `/perception/obstacle_points`, or `/perception/clearing_points`). The accel scan worker derives `/scan`, and Nav2 local costmap plus `collision_monitor` consume that LaserScan for standard marking and clearing. FAST-LIO2 subscribes to `/lidar_points` only during mapping/diagnostic runs. `pointcloud_downsample_node` is retained only as an opt-in diagnostic mirror.
- Pointcloud delivery diagnostics are C++ status topics, not Python probes: `ipc_worker` `pointcloud_accel_axis_node` and driver-integrated accel publish `/lidar/axis_remap_status` and `/lidar/pointcloud_accel_status`, while the patched `nav_cloud_preprocessor` remains diagnostic-only for localization/mapping preprocessor visibility. Production local obstacle handling does not use `robot_local_perception` or `/perception/local_perception_status`; Nav2 consumes `/scan`.
- Pointcloud rate diagnosis now treats `/scan` and `/flatscan` as the production navigation branches. The `/lidar_points` trunk has been validated near the JT128 target rate in pure-trunk A/B; a low `ros2 topic hz /lidar_points` reading can be subscriber-side delivery loss and is not enough evidence to change timestamps, QoS reliability, DDS middleware, or the canonical matrix. Use `diagnose_nav_scan_pipeline.sh` for `/scan -> /flatscan`, and `diagnose_pointcloud_cpu_pressure.sh` for Jetson CPU/thermal placement. Run `run_pointcloud_cpu_affinity_ab.sh --print` first; only use `--apply`/`--restart` after a diagnostic CASE points to CPU contention.
- Phase 1.12 adds reversible CPU/IRQ/softirq field tooling for the current CPU6/CPU7 pointcloud-placement question. Start with `collect_cpu_irq_softirq_snapshot.sh --duration-sec 20` and `identify_lidar_network_irq.sh` to capture per-core load, ksoftirqd, NET_RX, LiDAR IRQs, and RPS/XPS masks without subscribing to full-density PointCloud2 topics. `run_cpu_core_allocation_ab.sh` writes a temporary `cpu_affinity_runtime_override.env` only with `--apply` and retags live PIDs only with `--restart`; `run_lidar_irq_affinity_ab.sh` changes IRQ/RPS/XPS only with `--apply` and refuses a LiDAR NIC that is also the SSH/default-route interface unless `--allow-ssh-interface-risk` is explicit. `run_pointcloud_cpu_irq_experiment.sh` combines the CPU and IRQ plans, restores by default, and should first be run as CPU-only with `--irq-profile irq_keep_default`. These tools do not change QoS, DDS, timestamps, Nav2 planner/controller settings, EKF, FAST-LIO2 logic, App API, or mapping cleanup ownership.
- Phase 1.14 wires `NJRH_POINTCLOUD_ACCEL_PROFILE` into the runtime driver entrypoint. Production now defaults to `ipc_worker`; `legacy` is removed. `ipc_worker` starts `pointcloud_accel_axis_node` or the driver-integrated accel node as the single `/lidar_points` trunk owner: the callback publishes full-density/full-fields `/lidar_points`, updates a latest normalized buffer, and returns; the scan worker publishes `/scan`, while `laser_scan_to_flatscan` keeps `/flatscan` compatible. NITROS is a guarded navigation-branch skeleton only; it never replaces the `/lidar_points` mapping trunk. See [docs/phase_1_13_pointcloud_accel_profile.md](docs/phase_1_13_pointcloud_accel_profile.md).
- Phase 1.15 hardens `/flatscan` lifecycle for navigation localization startup. `/scan` existing is not enough: Isaac occupancy localization waits on `/flatscan`. In the production `ipc_worker`/driver-integrated path, the accel core owns `/scan` and a supervised `laser_scan_to_flatscan` compatibility helper owns `/flatscan`. `run_pointcloud_accel_pipeline.sh` starts the real helper binary directly, records its PID, and supervises ROS publisher liveness. A live helper is never killed from one short Fast DDS graph miss: three consecutive misses require a message-rate confirmation before restart. Cold startup also warms up and retries the bounded rate probe; an inconclusive or slightly low startup sample records `startup_degraded` but keeps both the helper and parent supervisor alive, so shell `set -e` cannot tear down a working `/flatscan` path. Restarts use a rolling cooldown window, and stable health restores the budget, so exhausting one window cannot terminate the supervisor or clean up the healthy `/scan` owner. `verify_pointcloud_accel_profile.sh` reports `FLATSCAN_OWNER_OK`, `FLATSCAN_HZ_OK`, `FLATSCAN_NAV_STARTUP_GATE_OK`, and `CASE_FLATSCAN_HELPER_DEAD` when `/scan` exists but `/flatscan` is missing. `run_navigation_runtime_services.sh` reports `FLATSCAN_MISSING` instead of a generic localization failure. Local dynamic-obstacle marking and clearing are handled by Nav2's `/scan` ObstacleLayer.
- Navigation stop and mode cleanup preserve the common `laser_scan_to_flatscan` helper instead of treating `/flatscan` as a private Nav2 process. The common runtime is the only pointcloud owner and the only layer allowed to replace its supervised helper. Resident localization reads `flatscan_helper_status.env` and performs a bounded publisher check, but never starts or restarts the pointcloud profile. A missing dependency fails the resident startup so the systemd owner can recover the complete chain. Common runtime startup also requires a single non-legacy pointcloud supervisor and a single `laser_scan_to_flatscan` helper; duplicate stale helpers are stopped with targeted SIGINT/SIGTERM before starting the replacement chain.
- Engineering navigation stop (`/api/v1/navigation/stop` or `/stop_runtime`) now tears down Nav2/localization process patterns before bounded AMCL shutdown cleanup. `stop_floor_navigation.sh` includes AMCL and scan-admission process patterns in the INT/TERM/KILL sweep, then runs `run_amcl_shadow_localization.sh --stop` under `NJRH_NAV_STOP_AMCL_TIMEOUT_SEC`, so AMCL lifecycle waits cannot consume the API stop window before Nav2 has been cleaned up. It explicitly preserves the common resident `robot_floor_manager`, keeping the next `/floors/switch` preflight available after navigation stops. User-facing task cancel remains `/api/v1/navigation/cancel`, which keeps the resident runtime alive.
- Phase 2.4a/2.4b/D2 are superseded for production local obstacle handling by the standard `/scan` path: Nav2 local costmap and `collision_monitor` no longer consume or publish derived `/perception/obstacle_points` or `/perception/clearing_points` clouds. Those old cloud topics must have zero production publishers/subscribers.
- Phase Z1 keeps the same `ipc_worker` ROS graph and removes the worker-side latest-`PointCloud2` reparse path. `pointcloud_accel_axis_node` now stores a latest in-process `LatestNormalizedBuffer` of `NormalizedPointView` points; production uses that buffer only to build `/scan` for Nav2 standard LaserScan marking and clearing. The retired local obstacle/clearing PointCloud2 workers are disabled. `/lidar/pointcloud_accel_status` reports `internal_zero_copy_profile`, latest internal buffer size, worker full-cloud-copy counters, intermediate PointCloud2 build counters, allocation counts, lock wait maxima, and worker processing averages. This is not RMW loaned-message zero-copy and does not add a new PointCloud2 topic.
- Phase D1 adds the reversible `NJRH_POINTCLOUD_INGRESS_PROFILE` selector. The `separate_process` path publishes decoded ROS `PointCloud2` on `/jt128/vendor/points_raw`, then `pointcloud_accel_axis_node` calls the shared C++ `PointCloudAccelCore` and publishes `/lidar_points` plus `/scan`. `driver_integrated` builds the repo-owned `src/third_party/hesai_lidar_ros2_overlay` source and starts `hesai_accel_driver_node`; Hesai decode constructs one in-process `PointCloud2` and moves it directly into `PointCloudAccelCore`, so `/jt128/vendor/points_raw` is no longer a production DDS input. `/jt128/vendor/imu_raw` remains available for the existing IMU remap path. `/jt128/vendor/points_raw` is a decoded ROS pointcloud topic, not JT128 UDP packets, and in the integrated path it is optional debug/compat only. Roll back ingress with `NJRH_POINTCLOUD_INGRESS_PROFILE=separate_process`.
- Phase D2 virtual PointCloud2 clearing is retired for production. Stale dynamic obstacles are cleared by Nav2's standard LaserScan raytracing from `/scan` with `inf_is_valid=true`; no custom clearing cloud is published.
- Phase 2.10 bounds local-state EKF input pressure without touching mapping inputs: `/lidar_imu` stays raw/high-rate for FAST-LIO2, `/lidar_imu_bias_corrected` is published at 100 Hz by `imu_gyro_bias_filter`, `/local_state/imu_bias` is published at 10 Hz, `/wheel/odom_ekf` is timer-published at 50 Hz, and the EKF output stays at `frequency: 50.0`. The corrected IMU helper is resident even when `LOCAL_STATE_EKF_PROFILE=wheel_only`, so `robot_safety` can use physical yaw-rate settle for spin-to-drive handoff without fusing IMU into `/local_state/odometry`. Use `scripts/jetson/runtime_overlay/scripts/verify_local_state_input_rates.sh` after restarting local-state helpers to verify ROS graph visibility, EKF subscribers, `/tf` ownership, topic rates, and UDP `RcvbufErrors`.
- Production `ipc_worker` topology is `/jt128/vendor/points_raw -> pointcloud_accel_axis_node -> /lidar_points + /scan`, or driver-integrated decode directly into the same accel core. During mapping the accel core keeps `/lidar_points` but relinquishes only `/scan`; FAST-LIO2 consumes full `/lidar_points + /lidar_imu`, and its corrected cloud is sliced by the mapping-owned chain onto the same canonical `/scan`. `/lidar_points` is not compacted or downsampled.
- The canonical `/lidar_points` publisher uses best-effort QoS with depth `1`, and mapping-owned FAST-LIO2 consumes that stream directly with best-effort depth `1`. Removing the former `/lidar_points -> /lidar_points_fastlio` identity hop keeps estimator input continuity during mapping without adding a second large-message copy on the critical path, while best-effort transport prevents full-size pointcloud DDS backpressure from lowering the live mapping input rate.
- Dashboard driver readiness now keys off the live Hesai driver plus the canonical pointcloud/IMU remap helpers and `/lidar/axis_remap_status`, not a long-running `/lidar_points` full-density subscription or `ros2 topic hz /lidar_points` probe. The web lidar view may still create a short lease when an operator opens it, but readiness and driver repair use the status topic.
- Dashboard `slam_toolbox` startup now allows a longer first-map warmup window before declaring failure, so slow first `/map` publication no longer aborts an otherwise healthy mapping start
- The live `slam_toolbox` mapping chain subscribes directly to canonical `/scan`; the former private scan converter and TF gate are retired from the launch path. No mapping process republishes, restamps, reverses, or re-slices LaserScan data.
- Mapping startup now proves `robot_fastlio_mapping` is visible through the ament index and verifies the canonical scan owner/freshness before starting the mapping backend. Mapping launch children retain an ownership marker for bounded abnormal-exit cleanup; the legacy converter/gate names remain only in one-cycle residual cleanup so an old running session cannot leave orphan processes after its next stop.
- the 2D mapping and occupancy-localization launch scripts now both clear stale `scan_republisher_node` processes before relaunch, preventing duplicate `/scan` publishers from blocking the flatscan path
- The 2D web view is back to rendering OccupancyGrid in its normal y-up convention and no longer carries a web-only orientation workaround
- Dashboard `开始3D建图` now starts the formal mapping backend as well: JT128 canonical ingress + FAST-LIO2 frontend + PGO backend + live `slam_toolbox` 2D view
- dashboard save behavior: paired `PGO 3D save -> current slam_toolbox /map snapshot -> Isaac localizer asset generation`, and the standalone 2D save path now follows the same `slam_toolbox nav map + localizer png/yaml` rule
- Current field default is the Web 2D mapping path: use `/api/mapping2d/start` and the standalone 2D save action for `slam_toolbox` maps. The 3D/PGO path is retained for optional formal mapping, not the default daily mapping flow.
- dashboard runtime asset directories `maps/`, `maps3d/`, and `waypoints/` are now writable project-owned mirrors seeded from the upstream `car` workspace instead of read-only symlinks
- dashboard `停止底层感知` now targets the direct JT128 driver plus FAST-LIO2, slam_toolbox live 2D chain, and related mapping-side helpers as a unit
- default navigation params owner: this repository's `scripts/jetson/runtime_overlay/config/nav2.yaml`
- runtime CPU affinity owner: `scripts/jetson/runtime_overlay/config/cpu_affinity.env` and `scripts/jetson/runtime_overlay/scripts/cpu_affinity.sh`; the default 8-core Jetson split isolates base/safety, TF/local odom, Nav2 control, JT128 driver, JT128 pointcloud ingress, localization scan admission, and mapping-only FAST-LIO. Pointcloud/IMU remap helpers stay on their assigned CPU sets, AMCL scan admission uses CPU6 by default, and `robot_localization_bridge` runs on CPU7 so `map -> odom` timers do not share CPU2 with EKF local-state. FAST-LIO2 deskew/frontend defaults to CPU7 only while mapping is active; live `slam_toolbox` mapping uses CPU3 and CPU7, while PGO stays on CPU7. Live 2D mapping also applies a temporary LiDAR RPS/XPS profile on eth1 to CPU5 and restores the previous queue masks when mapping exits; API stop/save also restores the same state file so process-group termination cannot leave mapping RPS/XPS masks behind. Use `NJRH_SLAM2D_FASTLIO_CPUSET`, `NJRH_SLAM2D_SLAM_TOOLBOX_CPUSET`, or `NJRH_SLAM2D_LIDAR_RPS_XPS_*` only for deliberate field overrides.
- Phase A1.3 pins the AMCL `/scan_amcl` admission relay to the localization CPU set through `NJRH_CPUSET_AMCL_SCAN_ADMISSION` (default CPU6) so the Python relay does not contend with EKF CPU2, Nav2 controller CPU3, or `robot_localization_bridge` CPU7. Use `inspect_runtime_cpu_affinity.sh` to check live `Cpus_allowed_list`, and `observe_navigation_tf_jitter_180s.sh --duration-sec 180 --label <run>` to capture odom/TF/AMCL timing without publishing goals or subscribing to pointclouds. This phase does not change `max_odom_tf_age_ms`, bridge future stamp offset, AMCL `transform_tolerance`, Nav2 plugins, pointcloud QoS, or DDS settings.
- runtime DDS owner: `scripts/jetson/runtime_overlay/scripts/common_env.sh`; all overlay helpers now default to `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, avoiding Fast DDS shared-memory port lock failures that can leave a process alive but invisible to the ROS graph. It also generates a default Fast DDS profile that whitelists only `lo` and `wlan0` for DDS discovery/data traffic, preventing `eth1`, `docker0`, USB, and link-local interfaces from participating in the ROS graph. Override with `NJRH_FASTDDS_ALLOWED_INTERFACES` or disable with `NJRH_FASTDDS_PROFILE_ENABLED=false` for field diagnostics. Re-sourcing it in the same shell is now a no-op after a complete load; each child process still performs one correct setup. Common services resolve the selected-floor asset context once and export it to resident localization and Nav2.
- Phase S4 hardens the boot-to-navigation critical path without weakening readiness. Common services exclusively own Ranger, static sensor TF, and the pointcloud pipeline; localization is a read-only consumer and cannot restart those dependencies after a transient DDS graph miss. Ranger admission reuses the common generation's long-lived runtime-health snapshot before falling back to a direct wheel-odom probe. The global-localization wrapper remains a child of the active occupancy-localization generation, and systemd removes any older orphan before a complete restart. Runtime logs rotate to one bounded 32 MiB history at owner startup, Hesai per-frame stdout timing is disabled, and Nav2 lifecycle readiness uses an atomic PID/timestamp status file instead of rescanning cumulative logs. See [docs/phase_s4_boot_to-navigation_critical_path.md](docs/phase_s4_boot_to_navigation_critical_path.md).
- Phase S4 field timing separates lifecycle service-response loss from real node convergence: `ChangeState` responses get a 5-second wait before `GetState` confirmation, while genuine configure/activate work retains its 60-second budget. A bridge-confirmed pre-arm Isaac result is drained within the same immutable trigger transaction. The bridge keeps the pre-arm stamp gate, but a currently armed explicit result is validated by original-stamp TF history rather than rejected solely by the 5-second wall-age value; the outer loop retries only proven pre-dispatch failures.
- Phase S4 startup probes now preserve Ranger, IMU, localization, and global-costmap hard conditions while grouping each related set into one DDS participant. Held Nav2 preload waits for both the Isaac service and bridge `has_odom=true`; global localization requires two consecutive valid post-arm FlatScan samples. Three identical-build full restarts measured `59.32–69.68s` start-to-ready with a `66.05s` median, and resident navigation measured `43s`, `43s`, and `52s`. Formal P95 acceptance still requires the documented five-run sample.
- production diagnostics must also source `common_env.sh` before starting any ROS participant. Field capture scripts must not use naked `ros2` CLI commands in the production domain: they inherit the Fast DDS profile, run each temporary CLI in a bounded process group, and verify cleanup. High-churn `/tf` and `tf2_echo` captures are opt-in only, because batch creation/teardown of many CLI participants can perturb Fast DDS discovery on a live robot.
- Predock SPINNING-without-rotation diagnosis uses the single-participant, read-only `record_predock_spin_command_chain_light.sh`. After its DDS warmup prints `READY`, it records only the four low-bandwidth command boundaries, Ranger mode feedback, wheel/local odom, and corrected LiDAR IMU; `Ctrl+C` finalizes an automatic stage diagnosis without rosbag, scan, pointcloud, or residual probe processes. See [docs/phase_ranger_spin_settle_handoff.md](docs/phase_ranger_spin_settle_handoff.md).
- Return-to-dock `rotate -> stop -> rotate` diagnosis uses `record_docking_rotation_trace.sh`. It is an SSH-friendly, single-participant, read-only recorder that correlates `/cmd_vel_nav_raw`, `/cmd_vel_docking`, the collision/safety/final velocity boundaries, wheel/local odometry, Ranger mode, safety state, and authenticated read-only docking/navigation phase snapshots. It creates no command publisher and never subscribes to scan or pointcloud; reports are written below `/tmp/njrh_reports`. See [docs/docking_rotation_trace.md](docs/docking_rotation_trace.md).
- Elevator segmented-spin diagnosis is available directly from the Jetson SSH host through `record_elevator_spin_chain_ssh.sh`. The wrapper starts exactly one bounded ROS participant inside `NJRH-car`, records the five velocity boundaries plus mode/odom/IMU/map/elevator state, and reduces `/scan` to footprint/StopZone/SlowZone counters at 5 Hz without storing ranges or subscribing to PointCloud2. Reports are copied to `/tmp/njrh_reports`; the recorder auto-exits, supports clean `Ctrl+C`, and never publishes a command, goal, parameter, or service request.
- Straight-line controller diagnosis uses the existing single-participant `record_navigation_odom_goal_closure.sh --assert-straightness` loop. It records Nav2 path shape, MPPI command reversals, wheel/local odometry, and Ranger feedback, then writes `straightness.md`; see [docs/navigation_straightness_diagnostic.md](docs/navigation_straightness_diagnostic.md). It changes no runtime parameter and does not publish velocity itself.
- runtime health snapshot writer: C++ `robot_bringup/runtime_health_guard`, launched by `scripts/jetson/runtime_overlay/scripts/run_runtime_health_guard.sh`. It samples persistent local-odom and docking-observation subscriptions at 1Hz, writes one atomic JSON snapshot per second, and checks ROS graph metadata every 5 seconds. `robot_bringup/runtime_health_check` reads the snapshot without Python or DDS. Optional scan/map/TF observation remains disabled by default; strong startup topic/TF readiness remains owned by `robot_bringup/runtime_readiness_probe`. See [runtime health design](docs/runtime_health_cpp.md) for sampling, compatibility, and deployment verification. Normal navigation startup still verifies the selected map, FlatScan input, localization result, bridge acceptance, canonical TF, and Nav2 activation.
- local-state runtime health allows 3 seconds without fresh advancing odometry stamps before classifying a producer-fault candidate. Observer staleness, malformed snapshots and clock jumps never authorize recovery. The common owner requires three distinct fault snapshots and independent fresh-odom confirmation, even if process-name inspection reports a missing process; only confirmed failures can exit the owner for a complete systemd restart. Startup freshness and real-time safety watchdogs are unchanged.
- startup readiness probes are diagnostic tools except for the deterministic navigation startup chain above. The compiled `robot_bringup/runtime_readiness_probe` binary remains available for explicit service/topic/TF/local-state checks, and `run_navigation_runtime_services.sh` uses it only for bounded one-shot gates that prove the initial localization and Nav2 activation are usable. It must not restore high-frequency rclpy/Python graph polling.
- local perception PointCloud2 obstacle runtime is disabled. `scripts/jetson/runtime_overlay/scripts/run_local_perception.sh` exits intentionally so `/perception/obstacle_points` and `/perception/clearing_points` cannot reappear through a stale helper/profile. Use `/scan` diagnostics and `verify_pointcloud_accel_profile.sh` to confirm local costmap and `collision_monitor` subscribe to `/scan`, and that old `/perception/*` publisher counts are zero.
- Run `diagnose_lidar_points_jitter.sh` first when `ros2 topic hz /lidar_points` looks low: its default mode does not subscribe to the full-density trunk and classifies publish-side low rate, stale binaries, excessive trunk subscribers, and CLI-only delivery loss from status topics and graph metadata. Use `--include-cli-hz` only for a short subscriber-side comparison. Run `run_lidar_trunk_pure_ab.sh --execute` only while stationary for source-side trunk isolation. Run `verify_lidar_trunk_jitter.sh`, `inspect_pointcloud_subscribers.sh`, `verify_pointcloud_delivery_matrix.sh`, `check_runtime_process_freshness.sh`, `inspect_pointcloud_cpu_affinity.sh`, and `record_pointcloud_nav_acceptance.sh --duration-sec 1200` for field diagnostics. These tools must not restore the retired `/perception/*` local obstacle path.
- navigation startup no longer starts or pauses local perception and no longer pre-warms a probe-owned TF buffer before launching Nav2. Nav2 is allowed to build its own TF/costmap buffers during normal lifecycle activation. Local dynamic obstacles come from `/scan` LaserScan marking+clearing.
- the `odom -> base_link` freshness gate now directly observes live `/tf` messages for that dynamic transform before falling back to tf2 lookup. This avoids declaring local odom stale only because a brand-new tf2 buffer has not finished discovery or has not accumulated current samples yet; the 0.25 s freshness threshold remains unchanged.
- local-state runtime defaults to `LOCAL_STATE_EKF_PROFILE=wheel_spin_imu`: upstream Ranger `/wheel/odom` remains unchanged, while the preprocessor replaces only actual `SPINNING`-mode yaw integration with corrected JT128 IMU yaw-rate, freezes spin x/y, includes the physical stop tail, and preserves a continuous offset when normal wheel odom resumes. Missing/stale IMU falls back to raw wheel odom. The paired EKF fuses corrected wheel `x/y/yaw/vx` plus corrected IMU yaw-rate and excludes wheel yaw-rate to avoid a duplicate dynamic input. On 2026-07-14, corrected `+180/-180deg` shadow spins matched IMU within `0.031deg`; production `+90/-90deg` canonical odom matched within `0.114deg`. A correction-frozen `delivery_675235 <-> delivery_512355` round trip finished at `5.00cm/0.22deg` and `4.78cm/0.55deg` with zero accepted AMCL corrections. `wheel_imu` is the immediate rollback profile and `wheel_only` remains the isolation profile. `robot_local_state` still owns the only `odom -> base_link` TF; FAST-LIO2 and raw JT128 topics are unchanged.
- Ranger stop-latency tests use a DDS readiness barrier before motion: fresh wheel odom, raw and corrected IMU, the `robot_safety` command subscription, and verified zero commands on `/cmd_vel_safe` plus `/cmd_vel` are required. Readiness failure is fail-closed. Event-level receipt timestamps now split yaw at the first final `/cmd_vel` zero callback into pre-zero integration error and post-zero physical tail; the 2026-07-14 `+/-180deg @ 0.6rad/s` test found that the dominant wheel/IMU mismatch was already present before zero receipt, while wheel odom observed the physical 4.2-4.7deg stop tail within 0.45deg.
- Common startup reuses `robot_local_state` only when the process, `/local_state/odometry`, and fresh `odom -> base_link` TF are all present. The EKF is launched through the `robot_localization` `ekf_node` binary directly rather than a `ros2 run` wrapper, so a live PID maps more closely to the actual ROS node. The local-state fresh-TF probe is process-time bounded: if the C++ probe has already printed `fresh TF ready` but hangs during exit, common startup continues after the bounded timeout instead of blocking safety/API startup.
- Ranger odom now enters the canonical tree directly as `base_link`: `run_ranger_chassis.sh` starts the upstream driver with `base_frame=base_link` and `publish_odom_tf=false`, while `robot_local_state` remains the only owner that publishes `odom -> base_link`. The static `ranger_base_link` alias remains available only as a compatibility sensor-description frame.
- Ranger spinning odom now keeps the navigation `base_link` at the chassis motion center by launching the upstream driver with `spinning_base_to_center_x=0.0` and `spinning_base_to_center_y=0.0`. The upstream Ranger SDK can model a nonzero spin-center offset, but carrying that offset into `/wheel/odom` makes `base_link` translate during pure yaw and conflicts with the canonical Nav2 base-frame contract. Static sensor/contact TFs must be calibrated against this centered `base_link`.
- Ranger spin-stop handoff now separates braking from mode exit in the patched `ranger_base` overlay. All-zero Twist commands remain in SPINNING with `angular=0` while wheel odom yaw-rate still reports spin motion or chassis feedback reports `mode_changing`, then later zero commands may return to DUAL_ACKERMAN. Disable with `RANGER_SPINNING_ZERO_CMD_HOLD_ENABLED=false` for rollback.
- Ranger near-straight linear odometry uses `dual_ackermann_linear_odom_scale=0.991`. A correction-frozen `calibration2 <-> calibration3` round trip kept `map->odom` unchanged, generated 297 AMCL shadow candidates with zero accepted corrections, and produced independent physical-marker fits of `0.99095` and `0.99110`. A post-apply shadow round trip measured only `9mm/13.915m` (`0.067%`) and `30mm/13.951m` (`0.218%`) distance residuals. The rounded common value changes only DUAL_ACKERMAN feedback integration while `abs(yaw_rate) <= 0.060rad/s`; yaw, lateral, and Ackermann arc calibration remain separate variables.
- JT128 static translation is calibrated against that centered `base_link`: `base_link -> lidar_level_link` and `base_link -> imu_link` use the current field candidate `x=0.3450, y=0.0000, z=0.85`, with yaw `3.1764992386296798`. This supersedes the earlier approximate `x=0.25`, the intermediate `x=0.38, y=0.0`, and the first fit candidate `x=0.34152, y=-0.040216`; it still requires post-apply four-heading relocalization validation before treating it as final.
- Field recalibration of `base_link -> lidar_level_link` XY/yaw uses `scripts/jetson/runtime_overlay/scripts/run_lidar_level_extrinsic_calibration.sh` plus the fitter in `fit_lidar_level_extrinsic_from_relocalize_samples.py`; the procedure is documented in [docs/lidar_level_extrinsic_calibration.md](docs/lidar_level_extrinsic_calibration.md). It changes only static sensor extrinsics after review and must be followed by a full `njrh-runtime.service` restart.
- local Nav2 dynamic-obstacle handling now uses the standard 2D LaserScan flow: `local_costmap` runs `ObstacleLayer + InflationLayer`, and the single observation source is `/scan` with `marking=true`, `clearing=true`, and `inf_is_valid=true`. `collision_monitor` also consumes `/scan`. The previous custom `/perception/obstacle_points` marking cloud and `/perception/clearing_points` synthetic clearing cloud are disabled in the default accel config, so moved people are cleared by LaserScan free-space rays instead of a separate virtual PointCloud2 clearing model.
- StopZone uses half extents `0.47 x 0.36m`, leaving an 8cm visible band outside the unchanged `0.39 x 0.28m` scan mask. MPPI separately prefers `0.52 x 0.41m` without enlarging the shared physical footprint or its clearing area. The two-return threshold (`max_points=1`), 2s FootprintApproach and velocity chain are unchanged. See [collision monitor geometry](src/robot_nav_config/docs/collision_monitor_geometry.md) for scope and outstanding low-speed hardware verification.
- Local-costmap clearing uses `raytrace_min_range=0.20m` while obstacle marking keeps the `/scan` cutoff at `0.25m`. With the `0.05m` costmap resolution, the earlier `0.25m` clearing start produced repeatable integer-raytracing blind cells that could retain lethal costs after an obstacle left; the one-cell inward start cleared every observed stale cell. This does not make the lidar observe the `0.20..0.25m` annulus and does not replace near-field safety sensing.
- The production local-obstacle `/scan` worker slices `lidar_level_link` at `-0.50m..0.35m` to reduce near-ground returns that can keep refreshing local-costmap obstacles after a moved object leaves. This is a navigation/local-costmap slice and does not change the separate `jt128_scan_slam2d.yaml` mapping/localization slice.
- The navigation-owned `/scan` worker now removes chassis/mechanical-arm self returns by transforming each candidate endpoint to `base_link` and filtering only the padded Ranger footprint (`x=-0.39..0.39m`, `y=-0.28..0.28m`). It does not increase `range_min` or clear a surrounding radius, so real obstacles immediately outside the robot remain available to both Nav2 and `collision_monitor`. The scan status reports the active bounds, per-scan/total filtered-point counts, and TF-unavailable count; an unavailable mask TF fails closed for that scan.
- local dynamic-obstacle avoidance is tuned for Ranger Mini 3 four-wheel-drive/four-wheel-steering with the project-maintained `ranger_base_node` interpreting each post-safety Twist exactly once. `robot_safety` owns timeout/stop, reverse permits, and normal-navigation lateral rejection; `ranger_base` is the sole CAN and motion-mode owner and publishes desired/actual feedback on `/ranger_base/status`. The former `ranger_mini3_mode_controller`, custom Ackermann shaping path, and profile switch are removed.
- Phase A2 removes the earlier plan to use Isaac as the continuous AMCL replacement. Isaac Occupancy Grid Localizer is kept for triggered global relocalization only through `/global_localization/trigger`; it consumes `/flatscan` and no runtime path forwards `/flatscan` into `/flatscan_localization`. AMCL is the continuous localization candidate source through `/scan_amcl`, which is a production AMCL admission input derived from `/scan` only in AMCL shadow/gated mode. Phase A1.4 keeps Nav2 AMCL itself unchanged and replaces only the scan admission relay with the C++ `robot_localization_bridge/amcl_scan_admission_node` by default (`NJRH_AMCL_SCAN_ADMISSION_IMPL=cpp`). AMCL and the C++ relay start through their installed binaries instead of `ros2 run` wrappers. The relay preserves the original scan stamp/frame/ranges, drops stale or non-TF-transformable scans, defaults to 5 Hz, and is pinned to `NJRH_CPUSET_AMCL_SCAN_ADMISSION` (CPU6 by default). Python `amcl_scan_admission_relay.py` remains an explicit rollback path with `NJRH_AMCL_SCAN_ADMISSION_IMPL=python`; a missing C++ binary fails fast instead of silently falling back. AMCL runs resident with `tf_broadcast=false` and now defaults to `gated`, where `robot_localization_bridge` accepts bounded covariance-gated AMCL corrections into `map -> odom`. See [docs/phase_a2_amcl_continuous_localization.md](docs/phase_a2_amcl_continuous_localization.md).
- Phase A1/A2 AMCL support keeps `robot_localization_bridge` as the sole `map -> odom` owner. `gated` is the current field default: corrections up to `0.12 m` and `0.20 rad` are applied directly; corrections up to `0.28 m` and `0.35 rad` require 3 consecutive consistent AMCL candidates. Candidates outside the medium gate but no greater than `0.60 m` and `0.80 rad` are large corrections that require explicit Isaac recovery; only corrections above `0.60 m` or `0.80 rad` are hard-rejected. `shadow` mode remains the odom-audit rollback profile, where AMCL candidates are reported without applying them to `map -> odom`. Isaac triggered relocalization remains responsible for initial/global recovery and seeds AMCL through `/initialpose`; after the initial trigger is accepted and `map -> odom` is live, AMCL resident warmup and readiness completion run in the background while Nav2 becomes usable. Static `/amcl_pose` staleness is normal when the robot is stopped; moving stale pose is reported as AMCL not tracking. Use `verify_amcl_runtime_readiness.sh --mode gated` for a stationary seed/no-motion check and `record_navigation_amcl_odom_correlation.sh` while sending a short goal. The runtime context is confirmed ready when the bridge-owned `map -> odom` baseline and Nav2 lifecycle are ready; AMCL tracking readiness remains visible in status and can be restored as a hard startup gate with `NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY=true`. See [docs/phase_a1_amcl_shadow_localization.md](docs/phase_a1_amcl_shadow_localization.md).
- Phase A2.1/A2.2 makes AMCL readiness explicit without making AMCL a TF owner. `run_amcl_shadow_localization.sh` writes `/tmp/njrh_amcl_runtime_status.env` with `AMCL_STATE`, process/lifecycle flags, scan-admission publisher counts, `/amcl_pose` publisher count, seed result, stale-PID cleanup flags, failure reason, a status epoch, and split readiness fields for process/seed/static-standby/tracking/correction. The file is a TTL-bound snapshot: stale `AMCL_FAILED` is diagnostic only and `robot_localization_bridge` falls back to live ROS graph and AMCL subscription evidence instead of treating it as authoritative. Startup status writing is fast by default and avoids extra ROS graph probes; set `NJRH_AMCL_STATUS_GRAPH_PROBE_ENABLED=true` only for explicit diagnostics. AMCL configure/activate now uses the same bounded one-process `rclpy` lifecycle helper as resident Nav2 instead of spawning separate `ros2 lifecycle get` and `ros2 service call` clients; the status heartbeat preserves a bounded `AMCL_STARTING` state while lifecycle activation and scan admission are in progress. Startup also defaults to `NJRH_AMCL_STATIC_STANDBY_SKIP_POSE_WAIT=true`: after a successful seed, AMCL can enter static standby immediately with `AMCL_STATIC_STANDBY_ACCEPTED=true`, `amcl_tracking_ready=true`, and `amcl_correction_ready=false`; this finishes boot without pretending AMCL has already supplied an applyable correction. AMCL seed is called by a lightweight rclpy client rather than `ros2 service call`; explicit diagnostics can disable the standby skip and use `scripts/jetson/runtime_overlay/scripts/amcl_nomotion_update_probe.py`, which subscribes to `/amcl_pose` before calling `/request_nomotion_update`. The helper does not restamp, publish TF, publish `/initialpose`, or use `ros2 service call`. Gated correction still requires `amcl_correction_ready=true`, but lack of a fresh correction while stationary is reported as `amcl_correction_pending=true`, not `localization_degraded=true`. `/localization/bridge_status`, `/api/v1/status`, and `/api/v1/navigation/state` expose `amcl_status_file_stale`, `amcl_status_source`, `amcl_seed_response_ok`, `amcl_nomotion_pose_received`, `amcl_process_ready`, `amcl_seeded`, `amcl_static_standby`, `amcl_tracking_ready`, `amcl_correction_ready`, `amcl_correction_pending`, `localization_degraded`, and `using_triggered_baseline_only`.
- If resident AMCL is launched before its background readiness completion, the status file can temporarily report `AMCL_WAITING_SEED`. `robot_localization_bridge` treats that state as resolved once live evidence shows AMCL is seeded, scan admission is ready, the robot is stationary, and static standby is valid. In that case normal goal admission remains allowed when `map->odom` is stable; the API uses these structured fields instead of a fixed degraded-reason string.
- Phase C1 adds a reversible controller/local-costmap CPU-set A/B profile without changing Nav2 parameters or TF gates. `NJRH_NAV2_CONTROLLER_CPU_PROFILE=current` keeps `controller_server` on CPU3; `control_wide` expands only that process to CPU3,5 because Nav2 hosts the local costmap inside controller_server. EKF/local-state remains CPU2, the multithreaded JT128 driver may run on CPU4,6 so unrelated arm/vision load cannot backlog its UDP decoder, AMCL scan admission uses CPU6, and `robot_localization_bridge` uses CPU7. `run_nav2_navigation.sh` now fails startup if the live controller PID and threads do not match the selected CPU set. Use `inspect_nav2_controller_threads.sh`, `observe_controller_tf_backlog_180s.sh`, and `run_nav2_controller_cpu_ab.sh --profile current|control_wide --duration-sec 180 --apply --restart-nav2` to compare controller-side TF requested/latest lag and local-costmap MessageFilter drops. Roll back with `export NJRH_NAV2_CONTROLLER_CPU_PROFILE=current` and restart Nav2.
- Phase L1.1 fixes the `/global_localization/trigger` success contract. The wrapper now treats triggered relocalization as a staged operation: arm bridge force-accept, call Isaac `/trigger_grid_search_localization`, wait for bridge acceptance, require `/localization/bridge_status.has_map_to_odom=true`, and verify `map -> odom` from `robot_localization_bridge`. Resident startup trusts the wrapper's `map->odom ready owner=robot_localization_bridge` success detail, then still performs a live `map -> odom` TF check; if that strong detail is absent it falls back to the older `/localization/bridge_status` wait. Triggered results use `triggered_max_result_age_ms=5000.0` to tolerate Isaac processing latency while still using the original result stamp for historical TF lookup.
- The global-localization wrapper now restores its authoritative source-map identity after a wrapper restart only through a fail-closed durable proof. A confirmed/ready runtime context with exact building/floor/map/epoch/digest and positive generation must match the active `current/` manifest and all typed live Isaac map parameters on the unique component. Success is latched as `LocalizerAssetState.code=BOOTSTRAP_READY`; missing legacy fields or any mismatch leave `active_identity_valid=false`, perform no component mutation, and retry at low rate.
- Phase L2 adds a post-relocalization settle barrier for explicit business relocalization. After the bridge accepts a forced Isaac relocalization, `robot_api_server` waits for the expected bridge sequence, `map -> odom` owner/freshness, `odom -> base_link` freshness, `base_link -> lidar_level_link`, local costmap heartbeats, and local-costmap MessageFilter stability before sending the next Nav2 goal or starting GS2 fine docking. The barrier keeps zero command active and returns explicit `POST_RELOCALIZATION_*` or `CANCELLED_BY_APP` failure codes instead of allowing the next stage to fail as a generic Nav2 abort. This does not change Nav2 controller/planner parameters, TF tolerances, pointcloud QoS/DDS, FAST-LIO2, Ranger odom, or EKF.
- Phase TF1 hardens that same relocalization handoff without changing TF tolerances or timestamp policy. `robot_localization_bridge` now separates the correction engine from the `map -> odom` publisher: Isaac/AMCL/manual correction callbacks only update bridge state, while an independent 50 Hz callback group is the only path that calls `sendTransform()`. `/localization/bridge_status` exposes the publish heartbeat (`map_odom_publish_loop_hz`, `map_odom_publish_gap_ms`, published/accepted sequence, paused/frozen state, and `publisher_decoupled_from_correction=true`). Use `verify_bridge_map_odom_publisher.sh` for a live contract check and `observe_tf_stability_after_relocalization.sh --duration-sec 180 --label <run>` around manual relocalization or undock transitions.
- Phase R0-R2 removes high-risk implicit relocalization from normal motion paths and adds bridge current/target smoothing. Normal `POST /api/v1/navigation/goal` no longer calls `/global_localization/trigger`, `/robot_localization_bridge/force_accept_next_localization`, `/localization_result` wait, or the post-relocalization settle barrier; if localization has a goal-start-blocking degraded state, bridge smoothing is still active, or AMCL reports a non-standby pending/not-ready correction, it returns `LOCALIZATION_DEGRADED` or `LOCALIZATION_TRANSITION_ACTIVE` and leaves recovery to the explicit localization path when recovery is actually required. AMCL static standby while stopped is not a recovery requirement and does not block a goal when `map -> odom` is live and no-motion standby is clean. Docking normal path also defaults `docking_relocalize_before_predock=false`, `docking_relocalize_after_predock=false`, and `docking_relocalize_after_fine_docking=false`. The bridge now accepts corrections into a target `map -> odom` and slews the current output at bounded rates, reporting `correction_active`, `safe_for_goal_start`, remaining error, current/target sequence, and large-correction rejection counters. This does not change Nav2 plugins, MPPI/progress checker, TF tolerances, `max_odom_tf_age_ms`, pointcloud QoS/DDS, FAST-LIO2, Ranger odom, or EKF. See [docs/phase_r0_r2_runtime_force_accept_bridge_smoothing.md](docs/phase_r0_r2_runtime_force_accept_bridge_smoothing.md).
- Phase R3 keeps that smoothing model but separates correction intent. AMCL gated and ordinary online corrections still use the default `0.20 m/s` and `0.25 rad/s` rates, while force-accepted explicit Isaac relocalization corrections above `1.0 m` or `0.35 rad` use a per-correction active rate sized to finish within `3.0 s`. `/localization/bridge_status` reports `smoothing_policy`, active rates, and configured rates so large manual/post-undock/floor-switch relocalization does not spend minutes in a half-updated `map -> odom` state. See [docs/phase_r3_explicit_relocalization_fast_smoothing.md](docs/phase_r3_explicit_relocalization_fast_smoothing.md).
- MPPI now uses Ranger Mini 3's documented Ackermann radius envelope (`min_turning_r=0.81`, `wz_max=0.70`) instead of sampling sub-physical 0.35 m turns; its obstacle critic matches the local inflation layer (`inflation_radius=0.60`, `cost_scaling_factor=6.0`). The 0.60 m radius covers the padded rectangular footprint's approximately 0.480 m circumscribed radius plus the 0.08 m collision margin after rounding up to the 0.05 m costmap grid. `VelocityDeadbandCritic` encodes the field-calibrated low-speed command deadband (`deadband_velocities=[0.025, 0.0, 0.025]`, `cost_weight=90.0`) so MPPI avoids selecting tiny linear/yaw commands that the chassis will not execute. The live controller keeps the 1.2 m/s field speed target but matches the measured chassis response with a 4.0 s horizon (`time_steps=48`, `model_dt=0.0833333333`), velocity-smoother acceleration limits (`max_accel=[0.55, 0.0, 0.90]`, `max_decel=[-0.95, 0.0, -1.10]`), and lower MPPI sampling noise (`vx_std=0.35`, `wz_std=0.38`). RotationShim enters path-entry heading at `0.45rad` and disengages at `0.075rad`, while residual spin tail is handled downstream by `robot_safety` using actual `/wheel/odom` yaw-rate settle. Near-goal convergence is biased toward XY before terminal yaw by `GoalCritic`/`GoalAngleCritic` weights (`16.0`/`6.0`) and a `1.5 m` critic window. Normal API navigation also publishes `/speed_limit` from the measured remaining map-frame distance to the target: `>2.0m=1.20m/s`, `1.2..2.0m=0.70m/s`, `0.6..1.2m=0.40m/s`, `0.15..0.6m=0.25m/s`, and `<0.15m=0.10m/s`, then clears the limit with `speed_limit=0` when the Nav2 task exits. Ordinary Nav2/MPPI tracking is now forward-only (`vx_min=0.0`); the existing controller-native terminal handoff owns bounded overshoot correction instead of letting mid-route MPPI select reverse commands that safety rejects. Velocity-smoother negative-X support and terminal reverse permits remain available. Existing critic weights are unchanged. `TwirlingCritic` uses Humble's actual `cost_weight=1.0` key so large initial heading corrections are not suppressed by the plugin default. The local window is `10m x 10m`, and PathAlign is moderately reinforced (`cost_weight=2.4`, `max_path_occupancy_ratio=0.05`) so dynamic obstacles can be skirted without arriving laterally offset from the goal path. Single-pose navigation uses the repository stable behavior tree instead of Nav2's default periodic replanning tree so MPPI is not reset by unchanged global path updates every second. If Ackermann MPPI stalls near a pose-required goal, the API handoff reuses the canonical executable recovery envelope (`distance<=0.40m`, body-frame `|forward|<=0.15m`) after at least `3s` task time and `1.5s` without `0.02m` improvement, then cancels only that Nav2 goal and enters the guarded axis-staged correction path through `/cmd_vel_api -> robot_safety -> /cmd_vel`. A target outside that geometry remains owned by Nav2. Final commercial verification is stricter than goal admission: after Nav2 or API final yaw, the API requests one stationary AMCL `/request_nomotion_update` when gated AMCL is active and waits for `robot_localization_bridge` to clear pending/smoothing before `task_complete=true`, so a final-yaw pause cannot hide a delayed `map -> odom` correction.
- Terminal reverse execution remains explicitly scoped: the controller-native handoff admits a target behind the robot and outside the `0.06m` XY tolerance within the existing `distance<=0.40m`, `|forward|<=0.15m` envelope, even with no lateral residual. It reuses settling and bounded reverse with its existing permit. The legacy API near-goal permit (`0.30m` enter / `0.35m` exit) and safety reverse rejection remain unchanged; they no longer need to rescue ordinary MPPI reverse samples. See [ordinary forward and terminal reverse](src/robot_nav_config/docs/ordinary_forward_terminal_reverse.md).
- local-costmap-only debug mode is repository-owned: Web `只启动局部障碍地图` starts JT128 driver prerequisites, chassis odometry, canonical TF helpers, and `src/robot_bringup/launch/local_costmap_debug.launch.py`; it publishes `/local_costmap/costmap` from the same `/scan` ObstacleLayer used by production navigation, without planner/BT/robot_safety control output.
- Current Ranger velocity-smoother calibration supersedes the older acceleration values in the MPPI field summary above: Nav2 runs `velocity_smoother` in open-loop ramp mode (`feedback=OPEN_LOOP`, `smoothing_frequency=30.0`, `odom_duration=0.2`), uses `max_velocity=[1.20, 0.05, 0.70]`, `min_velocity=[-0.08, -0.05, -0.70]`, `max_accel=[0.55, 0.20, 0.90]`, and `max_decel=[-0.95, -0.30, -1.10]`. MPPI remains Ackermann with no lateral samples; the narrow smoother Y channel carries only a controller-native terminal side-slip protected by a fresh safety permit. The ordinary-navigation profile runs the controller at 15 Hz with `model_dt=0.0666666667`, uses `vx_std=0.30` and `wz_std=0.32`, and starts terminal `/speed_limit` reduction at `2.4m/1.5m/0.9m/0.35m` with `1.20/0.65/0.32/0.16m/s`, then `0.08m/s` inside the final band. Closed-loop velocity-smoother feedback remains disabled because odom=0 plus the chassis motion-mode deadband can pin low angular output before the chassis starts moving.
- The non-Ackermann terminal residual is now controller-owned instead of API-cancel-owned. Inside the same `FollowPath` action, `GoalScopedRotationShimController` starts a bounded terminal state machine when distance is at most `0.40m`, body-frame forward error is at most `0.15m`, lateral error is at least `0.06m`, and either the residual is lateral-dominant or the remaining path is an Ackermann hairpin. It serializes yaw, side-slip, forward/reverse, and physical-stop settle; every translation is local-costmap checked and remains on `velocity_smoother -> collision_monitor -> robot_safety -> ranger_base`. Fresh lifecycle permits gate the `0.05m/s` lateral and `0.08m/s` reverse capabilities. `navigation_near_goal_stalled_handoff_enabled=false`, so the API no longer proactively cancels a live Nav2 goal and remains only the true-abort fallback.
- final safety arbitration runtime owner: this repository's C++ `robot_safety_node`, launched by `scripts/jetson/runtime_overlay/scripts/run_robot_safety.sh`
- robot safety runtime now publishes explicit arbitration state on `/safety/status` and `/safety/motion_allowed`, publishes safe motion commands on `/cmd_vel_safe`, and no longer has a Python fallback path
- Ranger Mini 3 chassis runtime owner: the project-maintained `src/ranger_base`, built with the pinned UGV SDK under `external_sources/jetson_ugv_sdk_20260713`. It is the only CAN and motion-mode owner, waits for chassis mode confirmation before releasing nonzero commands, and publishes `/ranger_base/status`. The former `ranger_mini3_mode_controller` shadow process and profile switch have been removed; `/cmd_vel_safe` remains a read-only safety mirror.
- GS2 is retained only as an explicit rollback backend. `src/robot_eai_gs2` and `run_gs2_driver.sh` remain available when `NJRH_DOCKING_SENSOR_BACKEND=gs2`, but the product runtime sets `NJRH_DOCKING_SENSOR_BACKEND=orbbec_336l` and `NJRH_GS2_AUTOSTART=false`, so no GS2 process or `/dock/gs2_scan` publisher is started during normal operation.
- Docking contact geometry is now explicit: `base_link -> charge_contact_link` is single-sourced by `robot_description` at `xyz=[0.398, 0.0, 0.255]`, which is 3.8 cm ahead of `gs2_link` on the same centerline. Docking config lives in `src/robot_nav_config/config/docking.yaml` and the Jetson overlay copy.
- Near-field charging alignment owner: `src/robot_docking_manager`, started as a resident common service by `run_common_services.sh` through `run_docking_manager.sh`. The production path reads the sensor-independent `/dock/target_observation` from Orbbec depth, corrects lateral/yaw error with a `0.15m/s` forward cap, and hands off at a `0.34m` fixed-face gap to a straight `0.05m/s` contact crawl. BMS contact is the success stop; stale observation/odom, loss of hard alignment, the bounded crawl distance, or contact timeout stops the robot and starts the configured retry path. Commands remain `/cmd_vel_docking -> robot_safety -> /cmd_vel -> ranger_base_node`; App clients never publish chassis velocity.
- Common services start Orbbec depth and `robot_docking_perception` after canonical static TF, then require a fresh `/dock/target_observation` before startup continues. `njrh_systemd_runtime.sh` explicitly passes the selected backend into the container, preventing an `/etc/njrh/runtime.env` choice from silently falling back to GS2. `robot_docking_manager` remains resident so `/docking/start`, `/docking/stop`, and `/docking/undock` are available before App/API requests.
- The Orbbec Gemini 336L depth-only fine-docking backend is the product default. `robot_docking_perception` converts live depth plus factory `CameraInfo` into `/dock/target_observation` and has no velocity publisher. Common startup identifies this owner by the commissioned `camera336l` process identity, not the generic `orbbec_camera` token, so a simultaneously running arm/vision Orbbec cannot mask a missing docking camera. The mount is single-sourced as `base_link -> camera336l_link`, `xyz=[0.394, 0.000, 0.350]`, `rpy=[0.000, 0.000, 0.000]`. Final-mount testing accepts the telescoping head and fixed face from about 0.8m through 0.3m; the fixed-face fit remains the yaw/lateral reference when the head enters the near-depth blind zone. At the measured `~0.313m` depth floor, control switches only after settled alignment to the bounded BMS-supervised contact crawl described above. See [docs/orbbec_gemini_336l_container.md](docs/orbbec_gemini_336l_container.md).
- Supervised Orbbec motion fitting uses `run_orbbec_dock_bidirectional_fit.sh`. It performs an explicitly enabled, low-speed fixed-face sweep through `/cmd_vel_docking -> robot_safety -> /cmd_vel -> ranger_base_node`, records camera range against wheel motion separately for forward and reverse legs, and confirms wheel stop between legs. Two complete 2026-07-16 rounds passed: all four fits had `R^2 >= 0.99766`, the combined camera-range/wheel-motion scale was `1.08146`, the forward/reverse scale difference was `0.664%`, and the maximum endpoint residual was `5.7mm`. The scale is diagnostic only until checked against an independent physical distance datum. The camera and perception launchers hold independent single-owner locks so a duplicate launch is rejected before entering the Orbbec SDK.
- Supervised lateral fitting uses `run_orbbec_dock_lateral_fit.sh` and the same safety-owned command path. Ranger Mini3 reports pure lateral translation as `MOTION_MODE_PARALLEL`; the fit admits only matched lateral-mode samples with actual wheel `|vy| >= 0.005m/s`, while retaining transition/stop samples separately. Across 11 admitted legs, left scale averaged `0.99320`, right scale `0.98811`, and the direction difference was about `0.51%`, with less than `1mm` forward cross-coupling. No lateral odometry scale is applied. The remaining 11--19mm endpoint residual is associated with wheel-mode transition/stop tail and the static fixed-face center estimate has about `3.4mm` standard deviation, so final docking requires settled multi-frame confirmation rather than a chassis-scale patch.
- Expanded Orbbec coverage at an approximately 0.81m fixed-face gap passed a 15cm-amplitude left/right/return sweep while retaining valid target observations from -12.0cm through +19.8cm lateral error, with less than 0.7mm forward cross-coupling and a 5.1mm return residual. Centerline acquisition remained dynamically consistent at 1.0m, 1.2m, and 1.4m (`R^2=0.9940..0.9960` after valid-frame filtering); 1.2m is the recommended camera-control ceiling and 1.4m is an early-lock region. The 2.0m depth input limit is not a control guarantee, and geometry remains capped at 1.50m. Bidirectional-fit reports now discard unhealthy, invalid, wrong-source, non-finite, and zero-gap frames before regression and expose fit/discarded sample counts so fail-closed zero geometry cannot corrupt calibration metrics.
- Ranger chassis startup is single-instance guarded per CAN interface by `run_ranger_chassis.sh`; if the dashboard already owns `ranger_base_node` on `can0`, localization attaches as a monitor instead of starting a second driver against the same CAN bus.
- The common-runtime API uniqueness health check now uses canonical `/proc/<pid>/exe` identity for `robot_api_server_node` and exact NUL-delimited supervisor arguments. Diagnostic commands that only mention the installed API path are no longer counted as API processes, while a genuinely missing or duplicate API executable still triggers fail-closed full-chain recovery. Cold startup first gives that exact executable up to `NJRH_ROBOT_API_PROCESS_READY_TIMEOUT_SEC=120` to complete ROS/Fast DDS construction; the grace applies only before `robot_api_server_ready`, not to later ownership loss.
- `robot_api_server` source, public headers, and tests now mirror a domain-oriented `features/`, `application/`, and `infrastructure/` tree. Existing map, floor-switch, navigation, elevator, docking, power, safety, teleop, subscription, HTTP, and process modules moved without changing interfaces, ROS wiring, parameters, or runtime ownership. Mapping is one deep `features/mapping/mapping_module` boundary: it owns its HTTP routes, async start transaction, launcher/process-group lifetime, marked residual cleanup, temporary LiDAR RPS/XPS restoration, live `/map` cache, exact navigation `/scan` owner proof, save/stop flow, and status snapshot. Teleop is one deep `features/teleop/teleop_module` boundary: it owns `/ws/v1/teleop`, mapping/elevator/charging admission, session and subscription lifetime, `/cmd_vel_api` and reverse-permit publishers, watchdog repetition, state JSON, and stop cleanup. The safety feature owns only the resident `/safety/status`, `/safety/motion_allowed`, and `/safety/estop` gateway edge plus its status policy; final arbitration remains in `robot_safety`. The top-level `robot_api_server_node.cpp` is now a six-line `main`; `infrastructure/process/robot_api_process` owns ROS process bootstrap, and `application/composition/ApplicationCompositionModule` is the sole cross-module object graph and lifecycle root. All feature handlers and domain logic live in their own modules.
- repository-owned bringup now includes `src/robot_bringup/launch/localization_bringup.launch.py` and `navigation_bringup.launch.py`, wiring the canonical stack to the repo-owned standard navigation chain with `robot_nav_config/config/nav2.yaml`
- repository-owned standard navigation now explicitly launches `behavior_server`, `velocity_smoother`, and `collision_monitor`, with remaps that force all Nav2 motion outputs through `cmd_vel_nav_raw -> cmd_vel_nav -> cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base_node`. `robot_safety` also publishes `/cmd_vel_safe` as a read-only diagnostic mirror; no second Ranger mode-controller node exists. Nav2 lifecycle bond timeout is disabled for the field runtime because repeated false bond misses on Jetson were deactivating the safety command chain; filter mask/info servers plus the lifecycle manager are kept off the planner/BT CPU core, while API goal admission and `robot_safety` remain the runtime safety gates. Navigation startup now reuses or starts resident helper processes by process ownership only; it does not run helper readiness probes for canonical local-state, local perception, safety, floor-manager, or global-localization before launching Nav2.
- Nav2 BT timing is tuned for the Jetson runtime instead of the desktop/default 20 ms action-server ack window: `bt_loop_duration=50ms`, `default_server_timeout=1000ms`, and `wait_for_service_timeout=2000ms`. `bt_navigator.plugin_lib_names` is also explicit so startup loads a bounded field list instead of Humble's full sample list. This prevents short CPU/DDS bursts from making `compute_path_to_pose` or `follow_path` fail immediately while the normal controller, collision monitor, and `robot_safety` chain remain authoritative for motion.
- Navigation goal diagnostics now record the full Nav2 action handoff (`/navigate_to_pose`, `/compute_path_to_pose`, and `/follow_path` status/feedback) together with the command chain. Use `record_navigation_goal_diagnostic.sh --post-goal-file <json>` when tuning MPPI so controller failures can be separated from planner/BT/API admission failures before changing parameters.
- Field validation on 2026-06-29 after enabling bounded MPPI terminal reverse passed a two-point run: `delivery_675235` finished with `final_distance_m=0.042184`, `final_yaw_error_rad=0.012823`, no retry; the return to `delivery_512355` finished with `final_distance_m=0.021524`, `final_yaw_error_rad=0.008177`, one same-goal Nav2 retry, and `/cmd_vel_nav -> /cmd_vel` showed bounded reverse samples clamped at `-0.08m/s`. AMCL accepted no corrections during either run, so this validates the MPPI/safety command path rather than localization masking.
- Extended MPPI validation then exposed one long-goal low-speed local optimum in report `20260629T085207Z`: `delivery_675235` stayed about `10.93m` from target and `/cmd_vel_nav_raw` averaged only `0.009m/s`. The follow-up tuning raised `PathFollowCritic.cost_weight` to `7.0` and `PreferForwardCritic.cost_weight` to `20.0`. After a full `njrh-runtime.service` restart, the same direction recovered: `20260629T090746Z` reached `delivery_675235` with `nav2_result_code=4`, `final_distance_m=0.003381`, and `final_yaw_error_rad=0.045486`; the intervening return to `delivery_512355` also completed with `nav2_result_code=4`, `final_distance_m=0.049267`, and no API velocity correction.
- local-costmap health is diagnosed after startup instead of being a pre-launch blocking gate. A live-but-blind costmap is still a real field fault, but the navigation process is allowed to start so logs, lifecycle state, and App/API failure reporting can show the actual failing component instead of leaving the system stuck in `starting`.
- multi-floor runtime assets now have a repository-owned multi-map contract: immutable bundles live under `maps_release/<building_id>/<floor_id>/maps/<map_id>/`, while `current/` is only the backend-owned compatibility mirror selected for a later controlled startup. The offline `/floor_manager/switch_floor` preflight requires and echoes the exact `building_id/floor_id/map_id/asset_epoch/asset_digest`, validates the immutable source bundle even when `current/` does not exist, and never starts Nav2. The API verifies that proof before atomically creating `current/`; ordinary single-floor startup then uses `POST /api/v1/navigation/start` with the exact identity. Legacy `resume_navigation=true` remains disabled. Resident elevator floor changes use the separate strict `/floor_manager/floor_switch` Action through the API transaction, with stopped-motion, map/filter/localizer, bridge, AMCL tracking/static-standby, costmap, runtime-context, and fresh stable target-map robot-pose barriers. The handoff never compares source- and target-floor `cabin_panel` coordinates: after target localization is proven, the following target-floor `cabin` goal starts from the live target-map `map -> base_link` pose supplied to Nav2.
- `scripts/jetson/njrh_container.sh` now prepares `maps_release` as an App/API-writable bind-mounted asset root on container start/common-service start (`root:root`, directory mode `2775`, file mode `664`). This keeps map, pose, keepout, runtime preview, and dashboard-generated assets under one container ownership model.
- Nav2 costmap filters are wired with a startup-safe production default. `standard_navigation.launch.py` starts the keepout mask map server plus keepout `costmap_filter_info_server`; speed-mask assets are still staged, but `speed_filter_mask_server` and `speed_costmap_filter_info_server` launch only when `NJRH_ENABLE_SPEED_FILTER=true`. The global costmap consumes only `KeepoutFilter` by default because field startup showed delayed `/speed_filter_mask` lifecycle and delivery can block Nav2 ready; the dormant `SpeedFilter` block remains in config for explicit rollback/A-B when speed zones are required. Filter mask/info servers are managed by `lifecycle_manager_costmap_filters`, while the controller/planner/BT/velocity/collision nodes are managed by a delayed `lifecycle_manager_navigation`; this keeps large map/filter activation from aborting the core Nav2 lifecycle chain under startup load.
- On the Jetson field runtime, core Nav2 lifecycle startup is driven by the repo-owned retrying `nav2_lifecycle_sequence.py` helper with `lifecycle_manager_navigation` present but navigation autostart disabled. This avoids Humble lifecycle manager's fixed 2 second `get_state` service wait and avoids `nav2_util/lifecycle_bringup` aborting on transient service-client failures while `planner_server`, `behavior_server`, and `bt_navigator` are loading selected-floor resources. The helper now defaults `NJRH_NAV2_LIFECYCLE_TRUST_CHANGE_STATE_RESPONSE=true`, so a successful `ChangeState` response is not followed by another `GetState` poll that can lag tens of seconds under startup graph load. When a fallback `GetState` response arrives after the two-second retry interval, every still-pending request remains eligible for completion; issuing a newer read-only retry no longer discards the older Future and stretches a small Fast DDS delay to the per-node deadline. Unresolved requests are removed when one response succeeds or the deadline expires. It does not change TF ownership, pointcloud transport, Nav2 planners/controllers, or the speed chain.
- Resident navigation autostart is now launched early from common startup once the pointcloud pipeline, chassis process, and static TF helper are alive. The production path gives the selected-floor localization transaction an uncontended window: it first proves the wrapper service, Isaac grid-search service, active exact map, supervised FlatScan owner, and exact floor context, then completes the initial global-localization transaction and bridge-owned `map->odom` baseline before starting the held Nav2 process preload. Nav2 lifecycle activation remains after that accepted baseline; `NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=true` remains an earlier A/B-only path. The bridge explicitly retains 30 seconds of `odom->base_link` history so an Isaac result arriving near the configured 20-second result window can still be evaluated at its original scan timestamp. It never restamps a result, substitutes latest TF, accepts a cached/unaccepted sequence, or skips TF ownership. AMCL no longer prewarms before the initial triggered bridge baseline by default; its tracking readiness starts after that baseline and continues in the background by default, reported through `/localization/bridge_status`, `/api/v1/status`, and `/api/v1/navigation/state`. Set `NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=true` only for A/B diagnostics, and set `NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY=true` only for rollback/diagnostics.
- Resident startup calls `/global_localization/trigger` through the lightweight `call_global_localization_trigger.py` rclpy client instead of `ros2 service call`. The default outer window is `NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT=90s` with a `75s` process bound, but another wrapper request is allowed only when the response proves `dispatch_state=not_dispatched`. Busy/post-reload/input/service-discovery races remain retryable; a result, bridge, TF, or map-to-odom timeout after Isaac dispatch is reconciled against the newer explicit sequence and never authorizes another automatic Isaac trigger.
- `/global_localization/trigger` is the sole owner of bridge force-accept, Isaac dispatch, and completion waiting. One request arms once and keeps that arm time immutable. A result from an older request is drained inside the same transaction and cannot end it or move the arm window. An explicitly armed result keeps its original timestamp: wall-clock delivery age alone no longer rejects it at 5 seconds, while historical `odom -> base_link` at that timestamp, fresh latest odom, TF ownership, bridge acceptance, and settled canonical `map -> odom` remain mandatory. Unarmed Isaac results are diagnostic-only and cannot change `map -> odom`. See [docs/global_localization_trigger_transaction.md](docs/global_localization_trigger_transaction.md).
- A live floor reload no longer treats the component-manager `load_node` acknowledgement as Isaac readiness. The wrapper keeps one trigger client for its full lifetime so DDS rediscovery does not mutate a Reentrant callback group during executor spin, then requires a post-reload fresh `/flatscan`, a 1 second minimum settle interval, and three stable service-ready samples before force-accept/trigger. The floor manager keeps ordinary services at 10 seconds, gives component asset apply 30 seconds and reconciles a delayed/lost Apply response only from the exact fresh terminal typed state, then gives the composite explicit-localization transaction 75 seconds and reconciles delayed trigger responses only against an exact newer target generation and explicit-localization sequence. `/api/v1/robot/pose` reports `FLOOR_CONTEXT_NOT_READY` plus the actual runtime state when a floor transaction is still unconfirmed.
- Live floor switching retries the wrapper only for failures that explicitly prove Isaac was not dispatched: wrapper busy, post-reload/input readiness, Isaac service unavailable, or bridge-arm service unavailable/timeout. After dispatch, the same wrapper transaction owns old-result draining and convergence; result/bridge/TF/map-to-odom failures are reconciled against exact target evidence but never create another Isaac request. The target identity, localizer generation, safety hold, and original `75s` Action budget remain unchanged.
- Cross-floor explicit localization treats independent map origins as independent coordinates. The ordinary 20 m forced-correction guard remains unchanged for startup, manual, and same-map recovery, but it is not applied to the first explicit result of an active `FloorSwitch` after the bridge proves the exact pending transaction and target Localizer requested/active identity, reload, generation, and readiness. The 2026-08-27 B15/F1-to-F2 failure reproduced at `29.671 m > 20.0 m`; the regression keeps that value accepted only under the exact floor transaction and rejected everywhere else.
- Manual live floor switching is admitted only from a confirmed `ready` source context with a nonzero explicit-localization sequence. A cold-start request is rejected before any pause/hold/bridge mutation, and an exact same-map request completes as an idempotent no-op instead of reloading identical assets through a different canonical projection. If a later transition fails after bridge BEGIN, exact ABORT plus fresh source health, durable source-context restoration, and exact lease release returns an ordinary `FAILED` terminal without a persistent lock; incomplete recovery proof remains stopped and explicit.
- Resident AMCL readiness now survives a transient lifecycle `get_state` timeout: the background worker continues through the bounded full-readiness retry, while the resident supervisor reaps and restarts a failed readiness worker until scan admission and AMCL seeding become ready. This repair does not restart Nav2 and does not change AMCL thresholds.
- API goal admission no longer synchronously polls Nav2 lifecycle `GetState` services. Runtime startup still verifies lifecycle activation, but ordinary goal start uses the confirmed resident context, `/navigate_to_pose` action-server readiness, and `robot_localization_bridge.safe_for_goal_start`, avoiding false `/local_costmap` or `/planner_server:state_timeout` rejections while Nav2 is actually active under Jetson/FastDDS load.
- Navigation startup stages the selected keepout/speed/binary masks into a per-run `runtime_nav2` snapshot before launching Nav2. `standard_navigation.launch.py` reads only that stable snapshot, so a concurrent floor switch or active-map mirror refresh cannot expose a half-written `current/filters/*.yaml` file to the Nav2 lifecycle manager. If staging cannot validate a source mask against the active Nav2 map dimensions, the helper falls back to a same-size neutral mask and logs the reason.
- App-authored keepout lines are now a complete backend transaction instead of JSON-only metadata. `POST /api/v1/maps/filters/keepout/save` validates every map-frame line/polygon against both map bounds and free nav cells, applies rotated map origins and PGM Y inversion, bounds raster work, and writes canonical semantic/YAML/PGM projections with durable atomic file replacement, readback, rollback, and a final commit marker. Active-map success also updates Nav2's actual runtime staging copy and requires lifecycle/plugin checks, `LoadMap`, a fresh matching `/keepout_filter_mask` digest, global costmap clear, and a strictly post-clear full-costmap whose added samples are lethal and removed samples are no longer lethal. `GET` returns a revision and `expected_revision` prevents stale full-layer overwrites. Neutral keepout servers and the global `KeepoutFilter` remain resident by default so the first line can take effect without restarting Nav2; a selected invalid managed mask now fails startup closed instead of silently becoming neutral. Mapping, docking, floor/map mutation, API or Nav2 action goals, and non-stationary wheel odometry block editing before any file write.
- saved flat Web maps can be promoted into a floor bundle with `scripts/jetson/runtime_overlay/scripts/promote_map_to_floor.sh <map_name> <building_id> <floor_id>`, and runtime localization/navigation can select the bundle through `NJRH_BUILDING_ID` + `NJRH_FLOOR_ID`.
- Web dashboard floor controls are test-only: list floor assets, promote a saved map into a floor bundle, select a floor bundle for a later controlled startup, and call the selection-only `/floor_manager/switch_floor`. Selection does not require a live `/map_server`; live cross-floor map/localizer/Nav2 loading through the legacy service is disabled. These controls exercise asset catalog behavior and are not the production mission UI.
- Android / external App integration now has a separate production gateway package, `src/robot_api_server`. It exposes a narrow HTTP API for status, safety stop/resume, floor switching, localization trigger, map listing, `POST /api/v1/mapping/2d/start` for repository-owned `slam_toolbox` mapping startup, `POST /api/v1/mapping/2d/stop` / `POST /api/v1/mapping/stop` for stopping the App-started 2D mapping process group, `POST /api/v1/mapping/2d/save` / `POST /api/v1/mapping/save` for saving the current slam_toolbox occupancy into runtime previews plus `maps_release/<building>/<floor>/maps/<map_id>/manifest.json` business map assets, and live `GET /api/v1/mapping/2d/map` PNG rendering from the current `slam_toolbox /map`, plus `WS /ws/v1/teleop` for low-speed App mapping movement. Activated maps are copied into `maps_release/<building>/<floor>/current/` as fixed role files (`nav_map.yaml`, `localizer_map.png`) so Nav2/Isaac/floor_manager do not depend on user-visible names; stale root-owned `current/` directories are quarantined before the backend recreates the runtime mirror. Editable App overlays are backend-owned through `GET /api/v1/maps/semantic_layer`, `GET /api/v1/maps/poses`, `GET /api/v1/maps/filters/keepout`, `POST /api/v1/maps/poses`, `PUT /api/v1/maps/poses/{pose_id}`, `DELETE /api/v1/maps/poses/{pose_id}`, `PUT /api/v1/maps/poses/batch`, legacy `POST /api/v1/maps/poses/save`, and `POST /api/v1/maps/filters/keepout/save`; Android must not restore points or keepout lines from local files. `GET /api/v1/status` now also subscribes Ranger `/battery_state` and returns `bms.soc`, BMS power-supply fields, and `bms.charging_contact` as the real chassis power state for the App. `GET /api/v1/navigation/pre_goal_check` exposes the same pre-navigation undock gate used by `POST /api/v1/navigation/goal` without moving the robot. Page-scoped `POST /api/v1/subscriptions/acquire|heartbeat|release` controls `status`, `live_map`, `scan`, `tf`, and `teleop`; `robot_api_server` keeps the `/map` cache subscribed while 2D mapping is active for startup readiness and save, page rendering still acquires `live_map`, high-rate `/scan` remains TTL-released after App disconnect/crash, and `/tf` is kept resident as a process-level localization input to avoid reliable Fast DDS endpoint churn during `/api/v1/robot/pose` polling. Saved map PNG preview remains explicit through `?source=saved` or `?name=<map>`. WebSocket teleop publishes only to `/cmd_vel_api`, so it still goes through `robot_safety` instead of sharing collision_monitor's `/cmd_vel_collision_checked` publisher; reverse is enabled only during active mapping teleop via `/ranger_mini3/teleop_allow_reverse`, docking undock uses `/ranger_mini3/docking_allow_reverse`, and navigation keeps `allow_reverse:false`.
- App-selected saved-map previews use the exact immutable identity form `GET /api/v1/mapping/2d/map?source=saved&map_id=<map_id>&building_id=<building_id>&floor_id=<floor_id>`. The gateway reads only that bundle's `localizer/<safe_map_name>.png`; missing assets and building/floor mismatches fail closed without falling back to a newer runtime preview or `current/` map.
- The Orbbec Gemini 336L depth-only launcher has a single-owner lock, enables the device heartbeat, and injects bounded SDK stream retry settings. This prevents duplicate SDK ownership and recovers many stream failures while preserving the `/cmd_vel_docking -> robot_safety` docking command path; it does not mask or resolve a physical USB disconnect.
- Starting App 2D mapping is now an asynchronous backend-owned mode transition. `POST /api/v1/mapping/2d/start` returns `202` immediately, then `robot_api_server` serially cancels the active navigation task, publishes zero velocity, stops the navigation/localization mode services, clears the transient navigation map context, and only then launches the mapping-owned `run_projected_map.sh` chain. `GET /api/v1/status` exposes `mode_transition` and `mapping.start_job` phases so the App does not need to race `/navigation/stop` against mapping startup. New navigation, docking, and navigation stop/cancel requests are rejected while mapping is active or starting, preventing overlapping mode owners. A plain `/navigation/cancel` keeps the resident navigation runtime alive and now reports `navigation_stack_stopped=false` unless `stop_stack=true` actually completed.
- App 2D mapping stop/save closes the marked mapping process group, corrected-cloud slice, slam_toolbox, C++ odom bridge, and private FAST-LIO2 while common services remain alive. After mapping `nav_cloud_preprocessor` and `pointcloud_to_laserscan` leave the graph, the runner restores the resident navigation `/scan` publisher. The stop transaction now reserves a 30-second graceful-cleanup window before signal escalation and the API independently requires the exact resident `/scan` owner before returning success; an interrupted runner is recovered with one idempotent service request and graph proof instead of leaving a zero-publisher scan gap.
- Runtime service ownership is now split into long-lived common services and mode services. Common services can be started with `scripts/jetson/runtime_overlay/scripts/run_common_services.sh`; navigation and mapping scripts reuse them by default instead of killing/restarting driver, chassis, TF, local-state, local-perception, safety, floor-manager, or App API processes.
- The production ownership model is documented in [docs/commercial_runtime_architecture.md](docs/commercial_runtime_architecture.md): process lifetime, lifecycle state, and mission task state are separate concerns. `scripts/jetson/runtime_overlay/scripts/check_commercial_runtime_ready.sh` reports whether the resident stack is actually ready, and `scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh` is the selected-floor resident navigation entrypoint. `run_floor_navigation.sh` is now compatibility-only and delegates to that resident runtime. `robot_api_server` keeps `/safety/status` and `/safety/motion_allowed` as process-level resident subscriptions, so App page subscription release no longer clears the safety cache used by status and motion admission.
- Phase S3 resident startup is documented in [docs/phase_s3_fast_resident_navigation_startup.md](docs/phase_s3_fast_resident_navigation_startup.md): production cold start is deterministic across the critical TF boundary. After `robot_local_state` and the exact selected-floor localization stack are ready, the initial trigger must produce the bridge-owned `map -> odom` baseline before the held Nav2 process tree starts. Field testing showed both early lifecycle activation and concurrent Nav2 process discovery can interfere with the initial localization transaction, so `NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=false` is the production default and pre-baseline Nav2 startup remains A/B only. AMCL resident pre-baseline warmup remains disabled by default; AMCL readiness starts after the accepted bridge baseline and continues in the background. Nav2 uses the repo-owned lifecycle helper for the point-navigation core: planner, controller, velocity smoother, collision monitor, and `bt_navigator`; noncritical smoother/behavior/waypoint lifecycle activation does not block the resident ready context. AMCL static-standby seed runs after the bridge baseline, and the context is confirmed `ready` after Nav2 core is active and the bridge-owned `map -> odom` baseline is usable; AMCL tracking/standby readiness continues in status unless `NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY=true` is set.
- Startup SLA tightening keeps the same ownership model: common startup reuses a healthy driver-integrated JT128 accel pipeline instead of restarting it, skips empty resident Nav2/localization/AMCL cleanup sweeps after systemd has already stopped those processes, and shortens only fixed helper settle sleeps. Reliability takes priority at the critical boundary: initial `/global_localization/trigger` completion is serialized before held Nav2 preload, and the bridge keeps a 30-second TF history window for the original-stamp lookup required by a result that may take up to 20 seconds. `NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=true` remains an execution-model A/B switch but is joined before Nav2 starts; it cannot bypass map/FlatScan/floor readiness or overlap the trigger with Nav2 discovery.
- Host autostart defaults `NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE=once`, so the recursive runtime asset/log permission preparation runs once and then uses `${NJRH_WORKSPACE_CONTAINER}/.njrh_runtime_permissions_ready` as a marker on later boots. Set it to `always` to restore the old every-boot permission sweep, or `skip` only for controlled diagnosis.
- Common startup defaults `NJRH_COMMON_LOCAL_STATE_START_READY_MODE=endpoint`: `robot_local_state` must have its process and ROS endpoint before resident navigation autostart begins. The boot child is tagged `systemd_autostart` and preserves localization prewarm overlap, while the resident runtime still requires fresh `/local_state/odometry` and `odom -> base_link` before it can trigger localization or activate Nav2. Set the mode to `fresh_tf` to restore the older common-layer fresh-TF wait.
- App `POST /api/v1/navigation/start` uses the same resident script tagged `api_resume`, but first requires three consecutive advancing local-state odometry and TF observations before selected-floor localization starts. A stale boundary fails as `LOCAL_STATE_ODOM_TF_NOT_STABLE`; the check adds no fixed sleep and does not alter the systemd cold-boot fast path. `NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=false` remains the default, and the held Nav2 process tree is not started until bridge-owned `map -> odom` is accepted.
- The resident navigation owner resets its own CPU affinity to `NJRH_CPUSET_NAVIGATION_RUNTIME_OWNER=0-7` before any asset work or child startup. This is idempotent for systemd autostart and prevents an App resume forked by the CPU0-pinned API server from accidentally keeping the entire unpinned startup tree on CPU0; individual ROS nodes still apply their existing service-specific CPU sets.
- Resident startup treats global costmap readiness as lifecycle active plus a live `/global_costmap/costmap` publisher by default. The full large OccupancyGrid message wait is still available with `NJRH_GLOBAL_COSTMAP_FULL_MESSAGE_GATE=true`, but is diagnostic-only because it can lag the active static layer and delay API readiness without improving command safety.
- AMCL startup now uses a fast static-standby seed path: after the bridge-accepted `map -> odom` baseline, AMCL is seeded from the current `map -> base_link` pose once the C++ scan-admission relay is running, but startup does not block on `/scan` header age or the relay's first admitted scan before confirming standby. Real AMCL corrections still require admitted scans; no extra `map -> odom` owner is introduced.
- Live 2D mapping defaults `NJRH_SLAM2D_ODOM_SOURCE=fastlio`: `run_projected_map.sh` starts a mapping-owned FAST-LIO2 frontend, converts its `/Odometry` through the C++ bridge to private `/tf_slam2d` as `mapping_odom -> base_link`, and only the 2D mapping launch remaps `slam_toolbox` to that private TF. This keeps the 2D backend odom consistent with FAST-LIO2 `/cloud_registered_body` after self-spin without publishing FAST-LIO2 odom into the canonical navigation `/tf` tree. Set `NJRH_SLAM2D_ODOM_SOURCE=local_state` to make `slam_toolbox` consume canonical local-state odom. `/api/v1/status` advances mapping from `starting` to `running` once the live `/map` is fresh and renderable; the API keeps this `/map` cache active for the lifetime of the mapping process instead of waiting for an App page lease.
- `scripts/jetson/njrh_container.sh start-runtime` now starts the container plus production common ROS services and, when a selected or last navigation map exists, waits for resident navigation context `state=ready, confirmed=true` plus API goal-start safety before reporting runtime ready. API HTTP readiness alone is not treated as full Nav2 readiness. Before resident autostart, common services clear stale resident Nav2/localization/AMCL state unless the resident wrapper process, context, and API safe-for-goal state all agree that the existing runtime is healthy. Use `start-dashboard` only when the debug Web observation window is needed.
- Resident navigation startup keeps a 120 second soft SLA report point, but that boundary is no longer the process-kill timeout while startup is still progressing. A separate hard cleanup timeout prevents slow Nav2 activation or AMCL readiness from creating a restart loop that hides the real slow stage.
- Runtime permission policy is explicit: new `NJRH-car` containers default to `root`, production ROS services and `robot_api_server` run as `root`, runtime writes use `umask 0002`, and released/runtime assets must stay `root:root` (`2775` directories, `664` files). See [reports/runtime_permission_audit.md](reports/runtime_permission_audit.md).
- Jetson boot autostart is installed through `scripts/jetson/install_njrh_autostart.sh`; it enables host `systemd` unit `njrh-runtime.service`, which starts the container and common ROS services but not the Web dashboard.
- web `标准导航` now enters repository-owned `src/robot_bringup/launch/standard_navigation.launch.py` for the Nav2 stack itself, while preserving the dashboard's existing separate localization startup step
- web navigation startup no longer starts a second FAST-LIO2 for the localization handoff; `run_occupancy_grid_localization.sh` consumes the derived `/lidar_points_nav` branch for stationary Isaac relocalization and only requires a managed FAST-LIO2 process when `NJRH_NAV_LOCAL_STATE_MODE=fastlio` is explicitly requested. The default navigation local-state mode is `ekf`, so localization only checks that the resident EKF local-state process exists. Stamped odom/cloud freshness is checked by explicit diagnostics and API goal admission after the runtime is up, not by shell startup gates. Floor navigation resume can reuse an already-running Nav2 stack for the same active map instead of killing and cold-starting it again
- web `重启定位` now follows the same ownership rule: it restarts the navigation localization stack and canonical sensing chain while preserving resident common services instead of killing the local odom owner
- web navigation and `重启定位` no longer wait for `base_link -> lidar_link` before the localization stack starts; that static TF is now waited on only after `run_occupancy_grid_localization.sh` has launched the canonical TF helpers
- web standard navigation now starts the localization stack, waits for `/global_localization/trigger`, Isaac `/trigger_grid_search_localization`, `/map`, and `/flatscan`, sends one bounded `/global_localization/trigger` request, then requires bridge-accepted localization and live `map -> odom` before launching standard Nav2. The old `/localization_result` publisher pre-gate is disabled by default because the trigger wrapper already verifies the real result path. This is a startup sequence gate, not a high-frequency diagnostic loop.
- Navigation resume treats initial localization and Nav2 lifecycle activation as the startup boundary. `/global_localization/trigger` is a bounded dispatch wrapper around Isaac's grid-search service; a delayed service response can be tolerated only if a fresh `/localization_result` and `map -> odom` are observed before Nav2 is marked ready.
- Local perception startup is retired for production. `run_local_perception.sh` intentionally exits and Nav2/local-costmap diagnostics observe the standard `/scan` obstacle source directly.
- Standard navigation no longer enforces map/topic/TF/costmap readiness from shell before Nav2 starts. `run_occupancy_grid_localization.sh` owns map/localizer loading, while `run_nav2_navigation.sh` launches the repository-owned Nav2 stack and lets Nav2 lifecycle plus API goal admission expose any real map/TF/costmap failure.
- Nav2 startup now reuses or starts common helper processes by process ownership, while the canonical local-state helper has an extra reuse gate: `robot_local_state` must expose its ROS endpoint, publish `/local_state/odometry`, and provide a fresh `odom -> base_link` TF. If a mode switch leaves an EKF process alive but the ROS graph/TF endpoint is dead, common services restart only that local-state helper before Nav2 starts. The resident runtime mirrors the selected floor asset `NJRH_NAV_MAP_ID` into `NJRH_MAP_ID` before writing `/tmp/njrh_runtime_map_context.json`, so a ready Nav2 stack exposes a confirmed map identity for `/api/v1/robot/pose`. If the resident runtime writes `state=failed` to that context file, `robot_api_server` reports navigation `failed` on `/api/v1/status` and `/api/v1/navigation/state` instead of leaving the App in `starting`.
- `scripts/jetson/njrh_container.sh start-runtime` waits for the App API HTTP health endpoint for up to `NJRH_ROBOT_API_READY_TIMEOUT_SEC=120` seconds by default. This absorbs resident navigation auto-start time without adding ROS topic/TF readiness probes or creating extra DDS participants.
- The repository retains idempotent same-floor navigation-runtime reuse for controlled startup and docking recovery. Ordinary App startup is an explicit two-step transaction: offline-select the exact map with `/api/v1/floors/switch` and `resume_navigation=false`, then call `POST /api/v1/navigation/start`; the App may report that map as current only after `/api/v1/navigation/state` confirms the same identity and goal-admission readiness. The public `/api/v1/floors/switch` endpoint does not invoke live switching: `resume_navigation=true` is rejected with `LIVE_FLOOR_SWITCH_DISABLED` until the strict cross-floor transaction is complete. App editor-map selection is local UI state and must not call this endpoint. For older App compatibility only, `resume_navigation=false` targeting the exact confirmed `ready` runtime map returns `runtime_map_already_selected` as a zero-side-effect 200 response; it does not call the floor service, rewrite `current/`, clear runtime context, reload localization, or change processes. Any different map remains blocked while a motion runtime is active.
- JT128 ingress now forces the upstream Hesai helper into its system-time timestamp profile (`use_timestamp_type=1`) while preserving repository-owned topic/frame remaps, so `/lidar_points`, `/lidar_imu`, `/wheel/odom`, and EKF output share the same ROS time base
- `robot_localization_bridge` now runs as C++, latches a successful Isaac `localization_result`, and keeps publishing the derived `map -> odom` from live odometry instead of timing out one-shot occupancy-localizer results after one second. Runtime publishes `map -> odom` at 50 Hz with `tf_future_stamp_offset_sec=0.0`, keeping TF timestamps measurement-time truthful while the independent publisher loop absorbs ordinary scheduling jitter. `/localization/health` is no longer subscribed by startup probes, avoiding durability-QoS false negatives. A real bridge process loss still stops the localization layer; navigation startup also refuses to mark the runtime ready until the initial `map -> odom` and Nav2 global costmap are available.
- Explicit business relocalization remains available for startup, floor switch, manual recovery, localization-degraded recovery, and post-undock recovery. It is no longer executed inside ordinary point-navigation goals or the default predock path. Auto-undock before navigation is still blocked until the post-undock recovery correction is reflected in `map -> base_link`. The stationary AMCL refinement immediately after an accepted Isaac pose uses a `10.0 m` / `0.872664626 rad` (`50 deg`) maximum candidate gate while retaining the `10 s` window and two-candidate agreement check; ordinary moving AMCL gates are unchanged.
- `POST /api/v1/docking/start` now treats the navigation-to-dock switch as a fast backend mode transition without default force-accept relocalization around the short predock path. It cancels the cached API navigation goal without stopping the Nav2/localization stack, checks bridge `safe_for_goal_start`, and then sends the pre-dock Nav2 goal. If the map has a manual predock pose named `dock_id_predock`, `dock_id_pre_dock`, `dock_id_approach`, `predock_dock_id`, `pre_dock_dock_id`, or `approach_dock_id`, or the request passes `predock_pose_id`, the backend uses that pose as the Nav2 target after checking yaw sanity. Manual pre-dock distance checking is disabled by default (`docking_manual_predock_distance_check_enable=false`), so intentionally close points are not rejected by the old `0.50m..1.20m` range; otherwise it falls back to the configured geometric offset from the saved dock contact pose. After Nav2 reports the pre-dock goal reached, the API verifies the predock pose. Fine docking remains protected by GS2 freshness, staging handoff readiness, a bounded `FINE_DOCKING_BRIDGE_SETTLE` wait for bridge `map->odom` smoothing to finish, and global-correction pause; explicit localization recovery is separate from the default predock flow. `POST /api/v1/docking/undock` still uses post-undock recovery when configured so navigation-triggered auto-undock can wait for a corrected pose before releasing the pending Nav2 goal.
- Return-to-dock has an explicit Nav2/fine-docking ownership boundary. `navigate_to_predock.xml` keeps its one-plan stable-path behavior but now selects the ordinary `goal_checker`, so native Nav2 success requires the commissioned predock pose to satisfy both `0.06 m` XY and `0.05 rad` yaw. The Ranger Lattice wrapper preserves the searched route but restores the exact commissioned endpoint across only a bounded (`0.08 m / 0.21 rad`), full-footprint-checked quantization residue; a blocked or larger residue fails planning instead of replacing the requested yaw with the nearest 5 cm / 16-bin state. The legacy asymmetric `DockStagingGoalChecker` remains registered only for compatibility/diagnostics and is not selected by any active tree. After the predock action becomes terminal, `PREDOCK_NAV2_STOP_VERIFY` holds zero and requires wheel/local-odometry stop proof before `/docking/start`; failure is fail-closed as `DOCK_FAILED_PREDOCK_NAV_STOP_UNPROVEN`. The resulting owner sequence is Nav2 full-pose predock -> proven stop -> `robot_docking_manager` monotonic yaw/vector/final/contact phases. No planner search, MPPI, progress-checker, AMCL, EKF, TF, or downstream velocity-chain parameter is changed.
- Phase D3/N2/N3-D separates delivery-goal completion from return-to-dock staging. `POST /api/v1/navigation/goal` exposes `goal_completion_policy`: ordinary stored `delivery_point` goals default to `pose_required`, while return-to-dock keeps the internal `dock_staging` label so ordinary API fallback motion is not mixed into the docking job. The predock behavior tree nevertheless uses the normal Nav2 `goal_checker`; it must reach `0.06 m / 0.05 rad` natively before stopped-state handoff. `PREDOCK_POSE_VERIFY` remains read-only. After `FINE_DOCKING_BRIDGE_SETTLE` and correction pause, `robot_docking_manager` becomes the single near-field motion owner: it requires three distinct sub-`0.5deg` yaw observations in `spinning`, latches that successful capture through translation/contact handoff, moves forward and laterally as one bounded `side_slip` vector, ignores yaw noise below the `3deg` recapture threshold, permits at most one large-yaw recapture, and locks lateral motion only in the final `0.06m`. An incomplete translation state can never silently remain active with an all-zero command. The contact phase remains straight-only with fresh odometry and BMS evidence, using distance-derived timeout and adaptive retry backoff. `/api/v1/navigation/state` and `/api/v1/docking/state` retain task, pose-evidence, command-owner, and Nav2 result diagnostics. `/api/v1/localization/trigger` semantics and canonical TF ownership are unchanged. See `src/robot_docking_manager/README.md`, `src/robot_api_server/src/features/docking/predock_alignment/README.md`, and `docs/docking_near_field_controller.md`.
- Pre-dock coarse navigation now uses a dedicated stable-path BT: one SmacPlanner2D plan is followed for each bounded Nav2 attempt. Ordinary delivery goals retain 1 Hz obstacle-aware global replanning, and the Humble-compatible `GoalScopedRotationShimController` forwards those replacement paths to MPPI without re-arming startup rotation for the same navigation goal. A genuinely changed target arms startup alignment again, while terminal goal-yaw rotation remains enabled. A stable BMS contact during the pre-dock Nav2 action now cancels that action immediately, holds zero through `robot_safety`, records wheel-odometry stop confirmation, and finishes as charging without any post-contact yaw or lateral command.
- The GS2 fine-docking global-correction pause is lifecycle-owned by the docking job. If a fine docking attempt is canceled, fails, stops, or is preempted after `/docking/start`, `robot_api_server` releases the `docking_fine` pause on job finish. Auto-undock-before-navigation and post-undock relocalization also clear stale `correction_pause_reason=docking_fine` before triggering localization, so a previous docking attempt cannot freeze `map->odom` and reject the next correction as `GLOBAL_CORRECTION_PAUSED`.
- Phase V1 adds validation-only field tooling around the D3/N2 contract. Use `scripts/jetson/runtime_overlay/scripts/observe_pose_required_navigation.sh`, `verify_manual_relocalization_api.sh`, `observe_predock_yaw_alignment_trace.sh`, `run_predock_yaw_alignment_probe.sh`, `verify_fine_docking_entry_gate.sh`, and `run_v1_navigation_docking_validation.sh --observe-only --duration-sec 120` before any full docking attempt. The default V1 runner does not send goals, docking requests, relocalization triggers, velocity commands, or heavy pointcloud subscriptions; manual relocalization and the small predock yaw probe require explicit opt-in. See `docs/phase_v1_navigation_docking_validation.md`.
- For ordinary navigation abort triage, use `scripts/jetson/runtime_overlay/scripts/observe_navigation_failure_minimal.sh --duration-sec 180 --label <run>` before sending the App goal. It creates one temporary `rclpy` participant, records API state, Nav2 action status, bridge/safety strings, the small Twist command chain, local-costmap summaries, and filtered `/rosout`, while deliberately avoiding `/tf`, PointCloud2, and LaserScan. For a reproducible moving-obstacle run, add `--store-costmap-snapshots --costmap-snapshot-period-sec 0.5 --output-dir /tmp/njrh_reports/<run>`; this stores exact sampled OccupancyGrid payloads plus PGM previews without changing navigation state.
- Recorder v3 retains terminal-permit/progress evidence, `/ranger_base/status`, actual published footprint, changed repair-path geometry, startup parameter-file snapshots, unique report directories and orderly Ctrl+C completion. It adds stamped Nav2 feedback with approximate odometry pairing, original-stamped controller warnings with command context, sparse publisher inventory and process counters. Collision state is recorded only if actually published; `EVIDENCE_LIMITS` and `evidence_availability.json` explicitly report missing diagnostics. MPPI rejection reasons and optimizer computation duration are **not collected**; process counters are not a substitute. No controller instrumentation, live parameter changes or visualization are enabled. See [navigation obstacle capture](docs/navigation_obstacle_capture.md).
- The September 7 dynamic-obstacle traces exposed missing local reference updates and defects in the subsequent repair loop. Ordinary `FollowPath` repairs its prefix on a detached local-costmap snapshot using forward-only Dubins Hybrid A* with the Ranger's `0.81m` radius and expanded rectangle. A blocked reference up to 4m ahead now requests background repair, not an automatic stop. Rejoin search extends within the local map, tries separated exits with independent search graphs and one shared budget, and rechecks attachment from the latest robot pose before calling MPPI `setPlan()`. Only the exact Humble MPPI no-valid-control exception becomes zero-command waiting with same-action retries; other faults still report errors. Background repair retains normal progress checking. Global planning, goal tolerances, elevator/fine-docking control, and the command chain are unchanged; predock travel shares ordinary FollowPath. Accepted paths are visible on `/ranger_mini3/ordinary_local_repair_path`; see `src/robot_nav_config/docs/ordinary_local_path_repair.md` for software regressions and remaining supervised hardware validation.
- For slow terminal convergence triage, use `scripts/jetson/runtime_overlay/scripts/observe_navigation_terminal_adjustment.sh --duration-sec 40 --label <run> --stop-when-terminal` before sending the App goal. It is read-only and quantifies when the robot enters the 1.5 m / 0.25 m near-goal windows, how long Nav2 stays there with tiny `/cmd_vel_nav_raw`, the `/speed_limit` sequence, and when API `final_yaw_align` or `/cmd_vel_api` takes over. The observer only starts timing after it first sees a new `running` navigation goal id, so stale `succeeded` state from the previous App goal is preserved in raw samples but excluded from near-goal/task-complete timing. Report `20260630T023038Z_nav_terminal_01_420s` showed the `delivery_987692` path reached about `0.31m` from the target with roughly `145deg` yaw error, then spent more than a minute in tiny MPPI commands before API yaw took over; this is the reason the near-goal stalled handoff window is `0.35m` instead of the previous `0.12m`. A later `delivery_230891 -> delivery_987692` run exposed a separate 8s idle gap before yaw-first recovery and a `0.35rad/s` API yaw timeout on a `2.39rad` residual yaw; ordinary API final yaw is now capped at `0.60rad/s` and the yaw-first recovery path skips that idle salvage wait. Follow-up field state then showed `distance=0.041m` and yaw near `0.050rad` being marked `degraded` only because gated AMCL remained in stationary standby with `amcl_correction_pending=true`; final verification now tolerates that clean standby condition and API yaw internally targets `0.045rad` before the unchanged `0.05rad` commercial gate.
- `20260630T061311Z_nav_230891_to_987692_fast_params_forward_xy_fix_60s` is the previous `delivery_230891 -> delivery_987692` terminal-convergence baseline: the goal completed in about `33s` from API accept to success with `final_distance_m=0.050476`, `final_yaw_error_rad=0.043281`, no Nav2 retry, and terminal XY correction taking about `1.0s`. Later field evidence showed mixed `linear.x`/`linear.y` terminal correction could enter the Ranger Mini3 official driver's parallel mode and increase XY error. Ordinary API terminal correction is now deterministic terminal-pose servo: compute signed yaw, forward, and lateral error from fresh `map -> base_link`; correct yaw first with pure `angular.z`, then lateral with pure `linear.y` in `side_slip`, then forward/reverse with pure `linear.x`. It does not mix angular and translation commands in the same terminal correction step.
- A July 22 pre-dock reproduction exposed a deterministic policy gap: Nav2 stopped at `0.354192m` with `0.126303m` forward and `0.330907m` lateral residual, while the old direct-correction, retry, and failed-Nav2 gates ended independently at `0.30m`/`0.35m`. These paths now share `navigation_terminal_recovery_max_distance_m=0.40`; direct axis-staged correction admits at most `0.15m` forward residual and has a `20s` maximum budget for the largest admitted error. Each nonzero terminal translation also requires a fresh `/local_costmap/costmap` centerline corridor below cost `50`, then remains routed through `/cmd_vel_api -> robot_safety`. The commercial success gate remains `0.06m` / `0.05rad`, and errors outside `0.40m` still fail closed instead of being hidden by terminal servo.
- July 22 field validation used the normal API path for three consecutive `predock -> delivery_512355 -> predock` round trips after one full `njrh-runtime.service` restart. All six goals reached `task_complete=true`; final errors were `3.06cm`, `5.43cm`, `2.50cm`, `3.73cm`, `2.53cm`, and `5.36cm`, with yaw error below `2.04deg`. The fifth goal reproduced the old hard failure exactly: its initial terminal lateral residual was `36.62cm`, above the removed `35cm` gate, and the costmap-guarded terminal recovery reduced it to `2.53cm` in `11.05s`. This validates the recovery-envelope fix, but all six Nav2 actions still ended non-successfully before commercial final verification; native Nav2 terminal convergence remains a separate open diagnosis rather than being declared fixed by the API recovery result.
- The fifth goal's controller log reported `Failed to make progress` twice. Its first Lattice plan was usable, but the terminal replan had a `0.403m` chord, `1.370m` path length, and `0.272m` cross-track loop: an Ackermann hairpin for a residual that Ranger side-slip can correct directly. The stalled-handoff check therefore no longer owns a separate `0.30m` gate. It reuses the canonical recovery eligibility (`distance<=0.40m`, body-frame `|forward|<=0.15m`, pose-required terminal correction available) and cancels Nav2 only after the existing `3s` minimum / `1.5s` no-progress evidence. Targets outside that executable recovery geometry remain with Nav2.
- After deploying that canonical handoff policy, two more `predock <-> delivery_512355` round trips completed at `3.13cm/1.87deg`, `5.36cm/1.89deg`, `4.19cm/0.59deg`, and `2.17cm/1.82deg`. The observed post-Nav2 lateral residuals included `14.83cm` and `19.60cm`; all four reached `task_complete=true`. In these samples Nav2 aborted after roughly `1s` of near-goal stall, before the API accumulated the configured `1.5s` handoff evidence, so they validate the shared post-abort recovery path rather than claiming the proactive handoff timer fired. Together with the earlier `36.62cm` field reproduction, they show that the former `35cm` unreachable boundary is removed while native Lattice/MPPI terminal convergence remains separately visible.
- July 22 controller-native terminal validation supersedes the earlier open native-convergence diagnosis above. Normal routes finished natively at `5.12cm/0.97deg` and `2.25cm/0.42deg` without terminal handoff. A deliberately induced opposite lateral residual triggered inside the same `FollowPath` action at `0.151m` total distance with `0.128m` lateral error. The normal Nav2 command chain moved about `0.103m` laterally at no more than `0.05m/s`, returned Nav2 result code `4` (`SUCCEEDED`), and finished at `0.020m/2.26deg`; `/cmd_vel_api.linear.y` remained zero. The first A/B run also proved the permit is fail-closed: an inactive lifecycle publisher caused `robot_safety` to clear the lateral command, while explicit publisher activation restored the command through raw, smoother, collision, safety, and final chassis topics.
- Resident startup lifecycle activation uses a one-shot DDS client that exits immediately after confirmed lifecycle transitions. This prevents Fast DDS participant cleanup from hanging after a lost service response and turning an already-active `map_server` into a false 90-second timeout that tears down the localization owner, Nav2, and AMCL. The initial global-localization Python client is now independently bounded at the process level as well: each existing wrapper timeout gets 5 seconds of shutdown grace, then GNU `timeout` sends `SIGTERM` and force-kills after 2 seconds, so an rclpy shutdown hang cannot leave runtime mode stuck in `starting`; the bridge-readiness fallback and all navigation/localization parameters are unchanged.
- Foreground and background held-Nav2 activation now share the same launch-readiness boundary: lifecycle requests start only after the current `run_nav2_navigation.sh` wrapper atomically reports its own live `controller_server` PID. This removes the race where lifecycle service calls began while the Nav2 child processes were still being created; it adds no new business or safety gate and changes no Nav2, localization, TF, DDS, pointcloud, or motion parameter.
- Final field validation after the axis-staged sign, `0.60rad/s` RotationShim/API yaw cap, and terminal-lateral-before-salvage fix:
  `20260630T080403Z_verify_delivery_987692_salvage_skip_rotshim06_no_post`
  reached `delivery_987692` in `30.61s` with `final_distance_m=0.048962`,
  `final_yaw_error_rad=0.042367`, `mixed_xy=0`, and no
  `final_pose_salvage_waiting`; `20260630T080506Z_verify_delivery_230891_salvage_skip_rotshim06_no_post`
  reached `delivery_230891` in `13.78s` with `final_distance_m=0.056584`,
  `final_yaw_error_rad=0.002038`, `mixed_xy=0`, and no salvage wait.
- Diagnostic field validation from the temporary `delivery_point=position_only`
  A/B config with Nav2 approach-heading yaw:
  `20260630T090030Z_verify_delivery_987692_api_default_after_binary_reload`
  resolved to `position_only`, used `nav2_goal_yaw_source=approach_heading`
  with `nav2_goal_yaw_rad=-1.364025`, completed in `22.96s`, returned
  `nav2_succeeded=true`, skipped API final yaw alignment, and finished with
  `final_distance_m=0.024671`. This is retained as root-cause evidence for
  the stale-yaw terminal delay, not as the default `delivery_point` policy.
  Repeat validation
  `20260630T090740Z_repeat_verify_delivery_987692_api_default` completed in
  `20.40s` with `final_distance_m=0.036479`, the same `position_only` policy,
  `nav2_goal_yaw_source=approach_heading`, `nav2_succeeded=true`, and no API
  final yaw alignment.
- For local-costmap `/scan` marking/clearing triage, use `scripts/jetson/runtime_overlay/scripts/observe_local_costmap_scan_clearing.sh --duration-sec 90 --label <run>` while reproducing a moved-obstacle case. It is read-only and classifies occupied cells as current-scan endpoints, cells blocked behind current endpoints, or cells that current clearing rays should have cleared.
- `POST /api/v1/navigation/goal` no longer performs hidden pre-goal relocalization and no longer blocks the HTTP request on controlled undock, post-undock relocalization, bridge readiness, or Nav2 action acceptance. It evaluates one pre-navigation dock-contact snapshot from backend docking state, `/docking/status`, and fresh BMS charging contact, creates a background `navigation_goal`, and returns `202` quickly. If the backend is docked, `/docking/status` starts with `docked` or `charging`, or BMS reports charging contact, the background job performs controlled undocking first and holds Nav2 submission until undock plus post-undock recovery succeeds. The same job then waits briefly for bridge `safe_for_goal_start` plus AMCL correction readiness before sending Nav2; if readiness does not recover, `/api/v1/navigation/state` reports `failed_goal_start_readiness` instead of making the App wait for a long HTTP response. `force_relocalize=true` remains an immediate recovery-required rejection, and normal goals still do not call `/global_localization/trigger` inside the goal handler. AMCL static standby while stopped is exposed in status but does not require explicit recovery and does not block a goal when `map -> odom` is live and no-motion standby is clean. `GET /api/v1/navigation/pre_goal_check` remains the read-only diagnostic form of the dock/contact gate, and `scripts/jetson/runtime_overlay/scripts/verify_pre_navigation_undock_gate.sh` checks it on the Jetson. This does not change FAST-LIO2, pointcloud, Nav2 planner/controller plugins, EKF, or DDS behavior.
- Phase U1 holds the original pending Nav2 goal after controlled auto-undock until post-undock relocalization has been accepted and the post-undock settle barrier passes. A bridge-accepted `map -> odom` correction only proves that the API has a new global correction; it does not prove that `controller_server` and its hosted local costmap have warmed their internal TF buffers. The barrier keeps zero velocity active, checks fresh `map -> odom`, fresh `odom -> base_link`, the static `base_link -> lidar_level_link` transform, local-costmap updates, local-costmap MessageFilter drops, and AMCL scan admission status before releasing the goal. If this readiness barrier fails after odometry-confirmed undock, the docking job remains `state=undocked` and `ok=true`; the original navigation goal stays held with `post_undock_navigation_readiness_failed=true` and `pending_goal_released_after_post_undock_settle=false`. `/api/v1/navigation/state` and `/api/v1/docking/state` expose `post_undock_settle`, including `pending_goal_held_for_post_undock_settle`, `pending_goal_released_after_post_undock_settle`, `post_undock_settle_failure_reason`, `post_undock_navigation_readiness_failure_code`, `amcl_ready`, and `localization_degraded`. Observe a live run with `scripts/jetson/runtime_overlay/scripts/observe_post_undock_to_nav_goal.sh --duration-sec 180`.
- `robot_api_server` gates normal Nav2 and predock submission on bridge status rather than scheduling its own correction timing. Before normal goals, docking pre-dock goals, and GS2 fine docking handoff, the API requires `robot_localization_bridge` to own `map -> odom`, have a live correction state, report `safe_for_goal_start=true`, and not report non-standby AMCL correction pending/not-ready while tracking. Stale or in-progress correction windows are rejected before Nav2 receives a goal, but correction acceptance, rejection, and smoothing policy live in `robot_localization_bridge`. Docking cancel waits for `/docking/stop` using the configured service wait instead of a 500 ms probe, then records the stop result in the docking job.
- Docked motion interlock is explicit and non-position-based. `robot_api_server` and `robot_docking_manager` maintain a persistent `docking_contact_latch.json` from stable charging-session evidence, `/docking/status`, dock success, and undock success. The latch is safety memory, not permanent docked truth: `pre_navigation_dock_check` separates `strong_live_docked` from `latch_valid_for_auto_undock`, reports latch source/age/stale/contradiction fields, and prevents stale latch evidence from singly blocking normal navigation once live BMS reports stable no-contact and runtime/docking state has no docked, charging, or undocking context. `final_yaw_align` refuses to rotate under live dock/contact, and `robot_safety` zeros normal `/cmd_vel_collision_checked` and API `/cmd_vel_api` commands while preserving controlled, continuous `/cmd_vel_docking` for docking/undocking. If fresh BMS says no contact and there is no current docked/charging status, old latch memory is cleared or treated as contradicted instead of permanently blocking ordinary point navigation.
- The final `robot_safety` BMS docking latch is event-order independent without adding another gate: release still requires an observed explicit reverse session, reverse permit disabled, and fresh BMS no-contact, but the shared predicate is evaluated from both the reverse-permit and battery callbacks. Thus `no_contact -> reverse_disable` and `reverse_disable -> no_contact` converge identically, while contact present or a still-enabled permit continues to force zero.
- Phase D2.3 preserves physical dock occupancy after full charge. New BMS charging/contact/current evidence is stored as strong `source=charging_session` instead of weak `source=bms`; `source=docking_job` and `source=charging_session` are not cleared by BMS no-contact alone while docking context or full-charge-idle evidence still suggests the robot may be physically on the charger. If BMS later reports stable no-contact and the runtime has no docked/charging/undocking context, an old `source=charging_session` latch is auto-cleared so ordinary terminal yaw and the next point goal are not misclassified as docked. SOC=100 alone never creates dock evidence. `/api/v1/navigation/state` and `/api/v1/docking/state` expose `dock_occupancy_state`, `dock_occupancy_evidence`, `charging_session_latched`, `full_charge_idle_on_dock`, latch source strength, and BMS live fields. Use `scripts/jetson/runtime_overlay/scripts/verify_full_charge_dock_session_gate.sh --dry-run --mock-charging-observed --mock-full-charge-idle --mock-bms-no-contact --expect-auto-undock --expect-docked-charge-idle --expect-no-latch-clear` for the non-moving dock-retained check.
- Phase N3 makes ordinary `pose_required` yaw completion native to Nav2, and Phase N3-D applies the same completion contract to docking predock staging. Phase N5 keeps Nav2 as the primary ordinary completion owner, with one same-goal Nav2 retry when Nav2 aborts inside the near-goal window but outside the yaw-alignable XY gate, and an API bounded yaw fallback after Nav2 abort/success when the robot is already inside the yaw-alignable XY window but heading is still outside tolerance. `GoalScopedRotationShimController` remains in front of MPPI for startup path alignment and has `rotate_to_heading_once=true` plus `rotate_to_goal_heading=true`: 1 Hz replans for an unchanged target update MPPI without restarting startup spin, while final goal yaw remains native to Nav2. Its pure-yaw shim command is limited to `0.60 rad/s` with `1.2 rad/s^2` acceleration so Ranger Mini 3 heading closure remains smooth while reducing long 160-degree startup turns. `SimpleGoalChecker` is `stateful=false` with `xy_goal_tolerance=0.06` and `yaw_goal_tolerance=0.05`, so ordinary point navigation and predock must re-satisfy XY and yaw instead of relying on a latched first XY hit; `PoseProgressChecker` uses `required_movement_radius=0.03` and `movement_time_allowance=12.0` so terminal low-speed creep is not aborted as no progress. The API sends target yaw to `NavigateToPose`, waits for the action result, and audits ordinary navigation with the same 6cm tolerance. Predock does not use the ordinary API yaw fallback: its Nav2 action must meet the normal checker first, and the later manager camera-capture tolerances are a separate fine-docking contract, not a substitute for the commissioned predock yaw. Normal path following stays Ackermann-first: the Ranger mode controller no longer converts moving high-curvature requests into spin by default, and instead clamps yaw rate unless the upstream command is pure yaw or the robot is below `0.08 m/s`. See `docs/phase_n3_nav2_native_goal_completion.md`, `docs/phase_n5_single_nav2_completion_owner.md`, and `src/robot_nav_config/docs/goal_scoped_rotation_shim.md`.
- Phase N6 commercializes ordinary point completion. `SimpleGoalChecker.xy_goal_tolerance` and API `navigation_goal_position_success_tolerance_m` are now `0.06`; Nav2 success is only input to API final verification. The API waits for bridge smoothing, rechecks fresh `map -> base_link`, retries the same Nav2 goal for bounded overruns, accepts only <=6 cm normally or <=8 cm after retry slack, and marks `degraded` with `task_complete=false` instead of reporting a false arrival when the commercial gate is still not met. Inside the near-goal API handoff window, terminal pose correction is explicit rather than trial-and-error: yaw first, lateral second, forward/reverse third, with each step publishing only the one required Twist axis through `/cmd_vel_api -> robot_safety -> /cmd_vel`. The API is not a publisher of `/cmd_vel_collision_checked`.
- Phase N6.1 fixes a downstream continuity fault found during elevator hall-call navigation: while a permitted normal Nav2 side-slip stream was fresh, the 10 Hz `robot_safety` idle mode-exit timer could alternate it with exact zero because the Ranger still reported PARALLEL mode. The timer now defers idle mode exit until the existing normal-command watchdog declares the stream stale; upstream loss still produces fail-safe zero. The isolated regression reproduced `8` zero interruptions among `24` pre-fix outputs and, after the fix, repeatedly observed `16/16` lateral outputs with `0` inserted zeros plus watchdog zeros after the stream stopped. This is an isolated non-moving result; the next supervised elevator run remains the hardware acceptance gate.
- occupancy localization sensing is now repository-owned as well: it uses `/lidar_points(lidar_link) -> nav_cloud_preprocessor(lidar_level_link, best-effort/depth 1) -> /points_nav(lidar_level_link, best-effort/depth 1) -> pointcloud_to_laserscan(target_frame=lidar_level_link) -> C++ scan_republisher_node -> /scan -> laser_scan_to_flatscan -> /flatscan`, instead of launching FAST-LIO2 or the older upstream `jt128_nav_sensing.launch.py` filter chain
- Isaac NITROS graph cache under `/tmp/isaac_ros_nitros/graphs` is prepared by the container launcher as `root:root` with `1777` permissions, so the root runtime can create graph folders without mixed ownership
- live TF cutover status on Jetson:
  - the live 2D mapping path no longer starts any extra static TF publishers
  - raw `/lidar_points` now publishes with `header.frame_id=lidar_link`; `/lidar_imu` publishes with `header.frame_id=imu_link`
  - mapping live graph no longer exposes `hesai_lidar_fastlio` in the canonical tree
  - localization entrypoint now prepares a PNG-backed localizer yaml instead of feeding Nav2 `pgm` directly into Isaac localizer
  - live `map -> odom` still depends on successful localizer matching; current `test-11` / `test-12` assets start the localizer stack but do not yet yield healthy localization

Windows entrypoint:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\jetson\Invoke-NJRHJetson.ps1 -Action start-runtime
powershell -ExecutionPolicy Bypass -File .\scripts\jetson\Invoke-NJRHJetson.ps1 -Action open-dashboard
```

`Invoke-NJRHJetson.ps1` refreshes the remote runtime overlay before each action. If a stale root-owned remote overlay cannot be removed, the helper quarantines it as `runtime_overlay.stale.<timestamp>` and publishes a clean overlay instead of leaving status/start commands blocked by old temporary files.

Manual pointcloud rate validation on the Jetson:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_local_perception_input_profile.sh --profile local_branch --print
bash scripts/jetson/runtime_overlay/scripts/set_local_perception_input_profile.sh --profile local_branch --restart
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_rates.sh
bash scripts/jetson/runtime_overlay/scripts/verify_lidar_trunk_jitter.sh
bash scripts/jetson/runtime_overlay/scripts/diagnose_lidar_points_jitter.sh
bash scripts/jetson/runtime_overlay/scripts/check_runtime_process_freshness.sh
bash scripts/jetson/runtime_overlay/scripts/check_pointcloud_topology_contract.sh
bash scripts/jetson/runtime_overlay/scripts/inspect_pointcloud_subscribers.sh
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_delivery_matrix.sh
bash scripts/jetson/runtime_overlay/scripts/inspect_pointcloud_cpu_affinity.sh
bash scripts/jetson/runtime_overlay/scripts/record_pointcloud_nav_acceptance.sh --duration-sec 1200
bash scripts/jetson/runtime_overlay/scripts/run_pointcloud_dds_transport_ab.sh --transport UDPv4
bash scripts/jetson/runtime_overlay/scripts/run_lidar_trunk_pure_ab.sh --duration-sec 20
bash scripts/jetson/runtime_overlay/scripts/check_isaac_ros_nitros_env.sh
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
ros2 topic info -v /lidar_points
ros2 topic info -v /scan
ros2 topic info -v /flatscan
ros2 topic info -v /perception/obstacle_points
ros2 topic info -v /perception/clearing_points
bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile ipc_worker --restart
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
bash scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_ab.sh --profile ipc_worker --duration-sec 120 --apply --restart
```

The rate script measures heavy PointCloud2 topics sequentially, while the trunk jitter and lidar jitter scripts prefer `/lidar/axis_remap_status` and only use raw-topic or `/lidar_points` `ros2 topic hz` as temporary checks. The subscriber inspector reads ROS graph metadata only, the CPU script only observes PID/TID/core/thermal placement, the process-freshness script checks build mtimes and status fields, the acceptance script samples lightweight status topics by default, and the DDS/pure-trunk A/B helpers print candidate actions unless an explicit restart/execute flag is requested. These are field verification tools, not resident monitors.

## Canonical TF

AMCL/Isaac correction gates use the timestamp-aligned physical `map -> base_link` pose innovation. Raw `map -> odom` x/y changes are transform parameters and may be amplified by yaw times the distance from the odom origin; they are diagnostic values, not robot travel. See [docs/localization_bridge_pose_innovation.md](docs/localization_bridge_pose_innovation.md).

```text
map
 └── odom                      (only robot_localization_bridge)
      └── base_link            (only robot_local_state)
           ├── lidar_link      (static)
           ├── imu_link        (static)
           ├── base_footprint  (optional static)
           └── other static frames
```

See [docs/tf_canonical_policy.md](docs/tf_canonical_policy.md) for ownership, suppression rules, and wrapper-level TF constraints.

## Local Reuse

- Windows car repository: `D:\codespace\car`
- Jetson host workspace: `/home/nvidia/workspaces/njrh-v3/workspace1`
- Jetson container workspace: `/workspaces/njrh-v3/workspace1`
- Jetson upstream asset workspace: `/home/nvidia/workspaces/isaac_ros-dev`
- Orbbec Gemini 336L host-cache driver reuse and container validation: [docs/orbbec_gemini_336l_container.md](docs/orbbec_gemini_336l_container.md)
- JT128 lidar IP: `192.168.1.201`
- Jetson lidar host IP: `192.168.1.100`
- preferred Jetson interface: `eth1`
- fallback interface seen in recent tests: `eth0`

## Bootstrapping

```bash
source scripts/env.sh
export CAR_PROJECT_ROOT=/path/to/car
./scripts/bootstrap.sh
python3 scripts/scan_car_project.py --root .
python3 scripts/tf_audit.py --root .
python3 scripts/resolve_third_party.py --root .
```

## First-Round Deliverables

- `reports/car_project_reuse_report.md`
- `reports/tf_audit_report.md`
- `reports/third_party_resolution_report.md`
- `reports/occupancy_builder_design.md`
- `src/robot_interfaces`
- `src/robot_description`
- `src/robot_chassis_bridge`
- `src/robot_hesai_jt128`
- `src/robot_fastlio_mapping`
- `src/robot_pgo_mapping`
- `src/robot_map_toolkit`
- `src/robot_local_state`
- `src/robot_global_localization`
- `src/robot_localization_bridge`
- `src/robot_local_perception`
- `src/robot_occupancy_builder`
- `src/robot_map_asset_identity`
- `src/robot_safety`
- `src/robot_floor_manager`
- `src/robot_mode_manager`
- `src/robot_elevator_manager`
- `src/robot_mission_manager`
- `src/ranger_base`
- `src/robot_nav_config`
- `src/robot_bringup`
- `src/robot_system_tests`
- `scripts/jetson/njrh_container.sh`
- `scripts/jetson/Invoke-NJRHJetson.ps1`
- `docs/jetson_njrh_container_runtime.md`
- `docs/phase_s2_navigation_runtime_ownership.md`

## P6 multi-floor/elevator work

2026-09-09 deployed candidate: floor switching can hand an exact target to an
unlocalized resident startup after explicit inactive-lifecycle and stopped-robot
proof. Normal hot switching keeps its existing path. The docking-observation
startup teardown was isolated and the runtime restarted with fresh docking data;
see [startup failure scope](docs/docking_startup_failure_isolation.md).
Real floor-switch/localization hardware acceptance is still pending;
see [unready-startup handoff, tests and limits](docs/floor_switch_unready_startup.md).

P6 now contains the pure-core scaffold for mission sequencing, elevator
schema-v1/v2/v3 topology/YAML loading, schema-v2 compatibility sequencing,
schema-v3 four-point reverse-entry sequencing, retryable
failure cleanup, and the atomic floor-transition readiness contract. The owner-scoped
operating-mode, safety-hold, and localization correction-pause arbiters are
also implemented. The safety execution-lease service remains available for
legacy/non-elevator clients, but new elevator-test transactions do not acquire
it. A non-moving cross-floor GTest composes these cores and verifies the
successful path leaves no Nav2 goal, motion hold, operating-mode lease, or
correction pause.

The manual commissioning path is implemented through
`/api/v1/elevator-test/*`. `robot_api_server` owns the production ROS adapter,
exact frozen release, manual confirmation gates, Nav2 goals, operating-mode/
correction leases, owner hold, and strict `FloorSwitch.action`. Landing/cabin
effects select an elevator-only Nav2 behavior tree that serializes yaw,
lateral, and forward/reverse motion with global and local footprint checks;
nearby hall-call also selects its dedicated elevator profile, while ordinary
navigation retains the existing Ranger planner/controller. `source_landing`
preserves the frozen, commissioned
landing-pose yaw. `enter_cabin` starts from that proven live ingress yaw but
keeps the cabin pose's independently commissioned final yaw. Its bounded
16-heading search can compose forward/reverse/lateral/spin primitives and
insert multiple footprint-checked intermediate turns; it never derives either
endpoint heading from the
landing-to-cabin chord. `target_landing` retains the directed
cabin-to-landing exit heading.

The production elevator-test adapter now consumes the hall-call and cabin
floor-button effects through the loopback arm black box on
`127.0.0.1:8083`. In the current feature-validation phase, each operation is
the direct `ready -> physical task -> release` sequence with task polling;
health, busy, ready, avoidance-pose, and `safe_to_drive` fields do not gate the
workflow. Source-door open, target-floor
arrival, and target-door open remain operator observations. The deployed hall
call endpoint currently reports `501 capability_unavailable`, so that one step
returns to the existing `CALL_BUTTON_PRESSED` operator step after the release
task; cabin floor selection is automatic.
The start request already pins the selected building, elevator, source floor,
target floor, exact maps, and release identity.

Each elevator-scoped `NavigateToPose` attempt now has an explicit controller
session keyed by `transaction_id:effect_sequence`. Starting a new attempt resets
stale terminal timers, failure state, route progress, cached paths, blockage
history, and pending replan work, while 1 Hz plan updates within the same action
preserve motion. Matching completion is idempotent and a late completion from an
older action cannot clear the newer session. The command-producing footprint
check waits for the local costmap's standard mutex instead of treating temporary
contention as an obstacle, so a map update cannot inject a false zero-command
pulse. Real transform, bounds, lethal, keepout, and unknown failures remain
fail-closed. Ordinary navigation, the frozen goal tolerances, and the final
velocity chain are unchanged.

Motion admission now follows the FSM's ownership boundary without creating an
elevator execution lease. The initial `hall_call` leg uses
an ordinary hall-approach contract: after its owner hold is released, the
interlock must be clear, no execution session/lease may already be engaged, and
the normal estop/localization/dock/watchdog gates remain authoritative.
`COMMAND_STALE` at that point is only first-command warmup. Every later elevator
navigation intent requires the exact transaction-owned operating-mode contract,
owner, mission, mode lease ID, and normal Nav2 source. A retained or foreign
legacy execution session blocks a new hall approach even if a
separate fresh `motion_allowed=true` sample exists.

The current schema-v3 commissioning contract adds explicit hall/cabin panel
sides and four per-floor poses: `hall_call`, `landing`, `cabin`, and
`cabin_panel`. After call confirmation the robot adopts the commissioned
reverse-entry yaw, moves laterally onto the landing axis, closes the remaining
forward/reverse error at `landing`, then
reverses straight to `cabin` and moves laterally to `cabin_panel`. Floor switch
anchors on the target `cabin_panel`, returns laterally to target cabin center,
then exits forward. Source staging selects an intent-specific deterministic
planner/controller; later schema-v3 effects retain the separate reverse-docking
planner/controller. Neither can be selected by ordinary Nav2 goals. Existing
schema-v2 releases remain executable but are not silently upgraded. The
operator-selected panel side is authoritative during commissioning; publication
does not reject a measured `cabin_panel` pose merely because its map-frame
vector from `cabin` also contains a forward/reverse component.
The atomic floor-switch correction-pause handoff now uses exact transaction
identity, a monotonic transition sequence, and a sticky level-ready flag rather
than a transient stage string. This prevents a back-to-back
`CALLER_PAUSE_HANDOFF_READY -> VERIFY_PAUSE_HANDOFF` feedback burst from leaving
the caller pause retained until the floor action aborts. See
`docs/floor_switch_pause_handoff.md`.
The floor-switch Nav2-idle gate also handles a newly started action server that
has never accepted a goal: after `/navigate_to_pose` and its status publisher
remain discovered for a bounded two-second grace period with no retained or
live status, the state is initialized as idle. Active/canceling status still
blocks immediately, and loss of the action graph returns the evidence to
unknown; no motion, localization, map, or safety barrier is relaxed.
The Isaac occupancy localizer now runs in Humble's
`component_container_isolated` with bounded process respawn. Its dedicated
single-threaded executor is canceled and joined before component destruction,
preventing the multi-threaded unload race that crashed the container during the
2026-08-06 B11/F2-to-F1 floor transaction. Unload confirmation polls within one
bounded operation window and succeeds only after the unload response is
successful, all component-manager services are live, and an exact component
listing proves the old ID absent. A lost response remains a failed request even
when absence is later proven, allowing only source-component rollback; a
permanently ambiguous graph still fails closed. The first stationary App floor
switch after deployment remains the real-runtime acceptance gate.
The elevator route now evaluates the filled, padded Ranger
footprint across the complete swept yaw/lateral/forward corridor. Global and
local inflation parameters remain unchanged: derived non-lethal costs
`1..253` do not by themselves reject the elevator-only route, while any intersecting lethal,
keepout, or unknown cell (`254/255`) still fails closed. Mechanical-arm/vision
evidence and unattended mission orchestration are still not connected.

The schema-v2/general elevator behavior trees wrap only `ComputePathToPose` in
an initial attempt plus at most three 0.5 s-separated retries. Their single
`FollowPath` remains outside the retry node, so a controller action is never
periodically replaced and no recovery clears a costmap. The detailed planner
uses a hard-obstacle distance wavefront before 16-heading expansion: a pose
outside the goal's inscribed-footprint connected component returns `no_path`
quickly instead of exhausting the one-second budget or crossing `254/255`.
The two schema-v3 prescribed trees instead retry their own commissioned
sequence for up to 30 seconds so a temporary person/door obstruction may clear;
neither replaces the sequence with an arbitrary detour. Translation/yaw command
clearance is capped by the active segment's remaining distance, so the fixed
0.15 m/0.10 rad projection cannot inspect beyond a short segment endpoint.

Elevator planning now keeps a completely free direct route, but when a
hard-clear direct route crosses soft inflation it also searches and compares a
clearance-improving alternative instead of returning immediately. Live route
revision runs on an immutable costmap snapshot in a dedicated worker with a
bounded 60,000-expansion/1-second four-wheel-steer search; it never runs inside
the 15 Hz controller callback or holds the live costmap during search. A
hard-blocked command first becomes exact
zero, then the worker searches a complete forward/reverse/lateral/spin route
from the current pose to the one canonical final goal. A successful result
atomically replaces the whole unfinished execution route; it never rejoins or
splices into a historical route. Identical costmap/start/segment/block-evidence
signatures are searched once per action, including A/B/A observation
oscillation, while changed evidence may supersede an unfinished revision in the
same `FollowPath`. Before commit, the returned swept path is rechecked against
the latest live costmap with the same filled-footprint rule used by planning and
command execution. Three seconds of continuous blockage remains a diagnostic
notice threshold, not an abort or widened motion permission.

Every accepted route revision is converted back to the canonical `map` frame
before it replaces the controller's execution route. The explicit final-pose segment
always targets the original map-frame goal, never the odom-frame endpoint of a
search snapshot. This closes the 2026-08-05 cabin-entry failure where a route
was accepted in `odom`, later AMCL corrections moved `map->odom` by up to
0.426 m, the controller stopped at its stale repaired endpoint, and Nav2 still
measured about 0.125 m to the real goal. A cumulative map-to-odom change of at
least 0.08 m or 0.08 rad is now diagnostic evidence only. The active segment
and canonical goal remain map-anchored and are transformed through the latest
TF on every control cycle, so AMCL/map-to-odom corrections update the remaining
error without resetting the controller, revoking permits, or inserting a
controller-owned zero interval. A compatible same-goal path refresh atomically
replaces only the remaining suffix and preserves the active motion phase; a
different goal still performs the full reset, while an incompatible same-goal
refresh cannot overwrite motion already in progress. Route revision remains
event-driven by a real hard blockage, with at most one background search in
flight and exact-evidence deduplication. The elevator behavior tree still owns
one `ComputePathToPose` followed by one `FollowPath`; no 1 Hz
`PipelineSequence`/`RateController` was added. The controller publishes typed
`TRACKING`, `REPLANNING`, and `WAIT_CLEAR` states. Only the two real blocked
states pause the elevator progress timer; ordinary navigation keeps its
existing checker limits and replanning behavior.

The elevator controller now treats the searched route phase as executable
intent rather than using the route only as a list of endpoints. A forward or
reverse edge ignores lateral endpoint residual, a lateral edge ignores forward
endpoint residual, and the edge direction cannot flip after overshoot. Only
after every planned edge settles does an explicit final-pose stage close the
remaining bounded XY/yaw error. This fixes the field failure in which a
five-segment detour was accepted repeatedly but the controller reconstructed
its own `yaw -> lateral -> forward` line and hit the projected-footprint gate.

The adapter does not publish Twist or TF and does not alter odometry, EKF,
AMCL, MPPI, ordinary tolerances, or the fixed velocity chain. Real entry/exit
still requires an empty-elevator supervised hardware gate. See
`docs/p6_elevator_without_arm_vision.md` for the remaining integration and
hardware acceptance gates.

The App-facing
`/api/v1/elevator-test/start|state|confirm|cancel|recover` contract now treats
ordinary navigation, confirmation, cancellation, doorway, ride, and floor
transition failures as retryable task failures. Before `SwitchFloor` succeeds,
cleanup is bound to the exact journaled source-floor runtime identity; after it
succeeds, cleanup is bound to the exact target-floor identity. Successful
cleanup releases the transaction-owned hold and resources and terminates as
`FAILED` or `CANCELLED`, so a new manual test can start without an on-site
unlock. A current-schema historical `ELEVATOR_EXECUTION_ON_SITE_SERVICE_REQUIRED`
snapshot created by the former phase-lock policy is converted on service
startup to the matching source/target cleanup when its journaled current
floor/map identity is exact. A bounded startup-recovery window that previously
ended as `ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN` is also finalized on
the next production full-chain cold start, but only for a schema-v3 ordinary
stage whose exact current source/target identity matches its outside cleanup
disposition. The failure remains audited, no motion/odometry proof is claimed,
and prepare, retained-lock, identity-mismatch, journal, and storage/audit
failures remain locked. `recover` remains a maintenance path for corrupt,
unsupported, or genuinely unreconciled runtime evidence; `active.yaml` is
never deleted to clear it. This simplification does not bypass Nav2 collision
checks, `collision_monitor`, emergency stop, or `robot_safety` command
arbitration. See `docs/elevator_test_http_fail_closed_deployment.md`.

Floor-transition evidence also distinguishes event state from live telemetry.
The latest `/navigate_to_pose/_action/status` active/terminal state is latched
until a new action event arrives; the 0.75 second freshness window remains only
for heartbeat-like motion-hold and wheel/local-odometry evidence. A human
confirmation delay therefore no longer turns an already terminal Nav2 action
into `ELEVATOR_FLOOR_SWITCH_PREMATURE_TERMINAL`.

Elevator-test no longer creates, renews, or releases a safety execution lease.
Its operating mode has one transaction-lifetime keepalive worker, independent
from Nav2 and `FloorSwitch`. A failed mode renewal is recoverable by reapplying
the exact owner/mission contract and, after confirmed expiry, rotating to a new
mode lease ID. Motion effects renew the mode synchronously before releasing the
owner hold. While the owner hold proves the robot stopped, a mode-renewal error
is diagnostic only: it cannot cancel an admitted `FloorSwitch` or turn a
successful map transition into `FAILED_LOCKED`. The next motion boundary must
re-establish the exact mode before any nonzero command can pass.

After a complete runtime restart, a confirmed healthy runtime on a newly
selected unrelated map may retire the stale failed transaction and release only
its owner hold once actions, resources, and dual-odometry stop are proven. This
restart-only rule does not relax exact-map cleanup during a live elevator task.

The post-call move from `hall_call` to the source-floor landing/entry-staging
pose now uses the same obstacle-unchecked elevator contract as the later cabin
legs. This covers schema-v2 `SOURCE_LANDING_FACE_CABIN` and schema-v3
`REVERSE_ENTRY_STAGING`: their bounded planners no longer read costmap cells,
their controllers do not run command-clearance or obstacle-driven route
revision, and the exact transaction refreshes the existing `/cmd_vel_nav`
collision-monitor bypass permit until that Nav2 action terminates. Schema-v3
still executes commissioned yaw -> lateral -> longitudinal staging, and all
existing speed, operating mode, estop, localization,
reverse/lateral permit, watchdog, stop and cleanup behavior remains unchanged.
Only the approach to `hall_call` and ordinary navigation continue to use live
obstacle checking.

Production API authentication reads the device token from the inherited
`ROBOT_API_TOKEN` environment when the ROS parameter is empty; runtime scripts
pass only the environment-variable name and never expand the secret into the
process command line.

The non-moving runtime preflight is implemented by
`robot_elevator_manager::load_elevator_release()`. It resolves
`.elevator_config/current` once (or an explicit release ID), then reads only
that immutable release. On Jetson/Linux it pins the release directory by file
descriptor and uses `openat(O_NOFOLLOW)` with regular-file, single-link, size,
schema, lineage, digest, topology, and internal-pose checks. A successful load
returns a frozen plan containing the exact authoritative source/target
`asset_epoch + asset_digest`; it does not imply that floor switching is
executable. Historical schema-v1 releases return `LEGACY_READ_ONLY` rather
than an executable frozen plan. No epoch is invented and no `FloorSwitchGoal`, Nav2 goal, ROS
service, Twist, or TF is produced.

`robot_map_asset_identity` owns the canonical 11-file digest and a
process-safe persistent epoch registry below
`maps_release/.map_asset_registry`. Epochs are globally monotonic, survive
process restart, and are never reused when content returns to an older digest.
Strict `MapManifest v2` records persist the same identity atomically. API-side
identity commits are serialized across processes, pin every source directory
and file without following symlinks, stream at most a bounded 1 GiB bundle,
revalidate the whole snapshot before and after registry binding, and commit the
manifest through the pinned bundle directory. Configuration GET/catalog lookup
paths only verify existing identities and never allocate an epoch or rewrite a
manifest; legacy import is confined to the single-threaded startup migration.
`robot_floor_manager` also provides a pure read-only source-bundle snapshot
loader that verifies the registry, manifest, path/file safety, and digest
without touching the localizer or runtime. It remains disconnected from the
disabled Action until real reload/generation evidence exists.

Runtime health recovery distinguishes observer availability from a proven
robot-local-state failure. A missing, invalid, clock-invalid, or stale resident
JSON snapshot, or graph metadata contradicted by fresh odometry delivery, is
diagnostic-only and cannot latch estop or restart the complete
runtime. Only three consecutive fresh snapshots that concretely report a
missing endpoint/publisher, unseen or stale odometry, or an inconsistent
summary preserve the fail-closed complete-chain recovery. Each decision records
snapshot age, publisher/message counts, odometry header/receive age, and
required-process liveness.

The same evidence boundary protects the Orbbec docking stream. A missing or
stale health JSON is an observer failure and resets the camera-fault retry
budget without restarting the robot. Fresh snapshots that explicitly report
`docking_sensor_healthy=false` raise a bounded diagnostic alarm, but they do
not terminate Nav2, localization, chassis, or the common runtime. Docking
motion remains fail-closed in `robot_docking_manager` when its observation is
missing, stale, unhealthy, or invalid.

Dock occupancy is now a persistent three-state contract: `ON_DOCK`,
`UNCERTAIN_ON_DOCK`, or `OFF_DOCK`. A completed dock/contact-stop or explicit
operator confirmation creates strong on-dock evidence; a successful controlled
undock clears it. Battery `FULL` is supporting evidence only and cannot create
the latch by itself. Both `ON_DOCK` and `UNCERTAIN_ON_DOCK` block ordinary Nav2
motion while retaining the dedicated controlled-undock path. An undock request
may safely retire a stale/running docking owner before it starts, preventing a
failed docking job from permanently deadlocking the only recovery action.

# 2026-09-14 建图保存协议补充

建图不再使用电梯测试准入锁。新增显式异步保存与请求 ID 查询，分别报告资产
提交和建图退出；客户端须同步更新。范围、兼容性和实机验收见
[建图保存协议](docs/mapping_save_async.md)。

## Explicit relocalization application (2026-09-14)

Explicit stationary Isaac relocalization applies the accepted `map -> odom`
target immediately, including small corrections; ordinary AMCL smoothing is
unchanged. Completion still uses the published target. See
[application contract and validation](docs/explicit_relocalization_immediate.md).
