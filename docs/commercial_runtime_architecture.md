# Commercial Runtime Architecture

This document defines the target production runtime model for the Jetson
delivery robot. It is the architecture contract for future refactors; field
scripts may still contain transitional compatibility paths until each phase is
retired.

## Principles

The production system separates three different states:

```text
process state      service process exists
lifecycle state    managed ROS 2/Nav2 nodes are configured or active
task state         a business mission is currently executing
```

App requests must change task state only. They must not directly own long-lived
process startup or shutdown and must never publish velocity commands.

## API Process Composition Boundary

The API executable has three explicit layers:

```text
robot_api_server_node.cpp                         six-line main only
  -> infrastructure/process/robot_api_process    ROS init/node/executor/shutdown
    -> application/composition/                   object graph and lifecycle
      -> application + feature + infrastructure modules
```

`ApplicationCompositionModule` is the only cross-module object graph. It asks
each dedicated configuration module to declare its existing parameters,
constructs one top-level aggregate for every feature family, connects narrow
ports, completes the deliberate Navigation/Docking and Localization/Navigation
late dependencies, starts the API gateway, and destroys modules in dependency
order. It owns no navigation, mapping, localization, floor-switch, elevator,
docking, teleop, power, safety, status, subscription, or HTTP business
implementation. The process bootstrap owns no feature dependency graph, and
the top-level main owns neither ROS state nor modules. This is an ownership-only
refactor: runtime parameters, gates, topic/QoS contracts, TF ownership, command
paths, and mission ordering are unchanged.

## Resident Services

These services are expected to be started by the boot supervisor or the
container common-service layer and kept alive during normal operation:

| Service | Owner | Production role |
| --- | --- | --- |
| JT128 driver and canonical remap | `robot_hesai_jt128` / runtime overlay | Sensor ingress |
| Ranger chassis core | project-maintained `ranger_base` under the `robot_chassis_bridge` boundary | Wheel odom, confirmed mode transitions, and the only CAN command sink |
| Static robot TF | `robot_description` | Sensor extrinsics |
| FAST-LIO2 runtime | `robot_fastlio_mapping` wrapper | Resident mapping/diagnostic frontend; optional explicit FAST-LIO local-state source |
| Local state | `robot_local_state` | Only `odom->base_link` publisher, default wheel-only EKF with corrected IMU kept resident for safety-side spin-tail detection |
| Global localization service | `robot_global_localization` | Asset reload and relocalization trigger |
| Localization bridge | `robot_localization_bridge` | Only `map->odom` publisher |
| Local perception | `robot_local_perception` | `/perception/obstacle_points` and clearing cloud |
| Safety arbiter | `robot_safety` | Final command gate |
| Floor manager | `robot_floor_manager` | Atomic floor asset switching |
| Map server | Nav2 `map_server` | Continuously running lifecycle node, map loaded by service |
| Nav2 stack | Nav2 lifecycle nodes | Resident navigation capability, not always executing a task |
| API gateway | `robot_api_server` | Narrow App API and mission admission |

The resident navigation runtime now owns selected-floor localization and Nav2
process activation. It does not use shell-level topic/TF/costmap readiness
probes as startup gates; those checks are explicit diagnostics after startup,
while API goal admission reports user-facing failures. `run_floor_navigation.sh`
is kept only as a compatibility wrapper for older callers.

## Canonical TF Contract

The runtime tree remains:

```text
map
  odom                 only robot_localization_bridge
    base_link          only robot_local_state
      lidar_link       static
      imu_link         static
      base_footprint   optional static
      other static frames
```

FAST-LIO2, PGO, and Isaac localizer frames stay internal or are wrapped before
they enter the canonical tree.

## Modes

Mode is a business and safety state, not a process-start command.

| Mode | Navigation services | Mission admission |
| --- | --- | --- |
| `BOOTING` | Starting or checking resident services | Reject all motion |
| `NO_MAP` | Resident services may be up; no valid active map | Reject navigation |
| `MAPPING` | Nav2 may stay resident but must not accept App goals | Allow mapping teleop only through safety |
| `LOCALIZING` | Map loaded, relocalization in progress | Reject App navigation goals |
| `NAV_READY` | Nav2 active, costmaps valid, TF fresh | Accept navigation goals |
| `NAVIGATING` | Nav2 active with an accepted goal | Track goal result |
| `DOCKING` | Nav2 used only to reach pre-dock pose, then paused for fine docking | Reject normal goals |
| `UNDOCKING` | Docking manager controls controlled reverse through safety | Reject normal goals |
| `CHARGING` | Services may stay resident | Reject normal goals until undock |
| `FAULT` / `ESTOP` | Services may stay resident or deactivate | Reject all motion |

## Startup Contract

Production boot should do this once:

```text
start common resident services
start map/localization services
start Nav2 lifecycle services
load selected floor map when available
trigger relocalization when a valid floor is selected
wait for:
  odom->base_link fresh
  map->odom fresh
  /map selected and valid
  /global_costmap/costmap resized from static map
  /perception/obstacle_points fresh
  robot_safety healthy
transition mode to NAV_READY
```

If no released map exists, the robot remains in `NO_MAP` with resident services
alive. Starting mapping does not require killing driver, local odom, safety, or
API services.

## Navigation Goal Admission

`robot_api_server` or a future `robot_mission_manager` must verify all gates
before sending `NavigateToPose`:

```text
mode == NAV_READY
critical Nav2 lifecycle nodes active
/navigate_to_pose action server ready
map->odom fresh
odom->base_link fresh
/local_state/odometry fresh
/perception/obstacle_points fresh enough for local costmap/collision monitor
robot_safety healthy and not estopped
selected map_id/building_id/floor_id matches runtime context
```

Goal execution remains:

```text
App
  -> robot_api_server / mission manager
  -> Nav2 NavigateToPose
  -> planner_server
  -> controller_server
  -> velocity_smoother
  -> collision_monitor
  -> robot_safety
  -> ranger_base
  -> pinned UGV SDK / chassis
```

## Mapping Contract

Mapping is a maintenance/commissioning mode. It may reuse resident sensor,
FAST-LIO2, local-state, safety, and API services. It must not let App navigation
goals run against an unstable live map.

```text
MAPPING mode:
  keep resident services alive
  verify resident static TF, FAST-LIO2, and local-state; do not repair them here
  cancel or gate active navigation tasks without tearing down resident runtime
  run mapping frontend/backend
  save map assets
  validate nav/localizer/filter assets
  load the released map
  trigger relocalization
  return to NAV_READY after resident localization/Nav2 processes are launched
```

The default 2D mapping path starts a mapping-owned FAST-LIO2 frontend, consumes
its `/cloud_registered_body` and `/Odometry`, then publishes only a
mapping-private `mapping_odom->base_link` TF on `/tf_slam2d` for
`slam_toolbox`. It must not start or kill canonical static TF or
`robot_local_state`; stopping mapping only cleans the mapping-owned FAST-LIO2
process and the mapping bridge.

## Docking Contract

Docking is a mission mode:

```text
Dock:
  cancel normal Nav2 goal
  relocalize
  Nav2 to manual or computed pre-dock pose
  relocalize and validate approach pose
  docking_manager fine alignment
  robot_safety remains final command gate
  contact/charging confirmation
  mode -> CHARGING or FAULT

Undock:
  cancel normal Nav2 goal
  docking_manager controlled reverse through safety
  confirm movement with /local_state/odometry
  trigger relocalization
  mode -> NAV_READY only after map->odom is fresh
```

The process-level ownership boundary is
`features/docking/docking_feature_module`. One aggregate owns contact
classification, the canonical task store, correction-pause lifecycle,
docking-manager runtime, automatic undock, predock control, task execution,
pose resolution, HTTP, status reconciliation, its deferred-work queue, and
start serialization. Construction is explicitly split into a core phase and a
completion phase only to resolve the real Navigation/Docking dependency cycle;
both phases finish before the API gateway starts. The composition root owns no
individual docking submodule or docking Ports object. This is an ownership
change only: phases, thresholds, BMS semantics, retries, TF ownership, command
topics, and the final safety chain are unchanged.

The API-side runtime edge for this contract is contained behind
`features/docking/lifecycle/docking_runtime_module`. It owns docking-manager
process supervision, `/docking/start|stop|undock` clients, `/docking/status`
and the selected observation subscription, cached observation evidence,
predock `/cmd_vel_docking` plus Ranger forced-mode publishers, and the single
docking worker. `DockingFeatureModule` supplies status/job callbacks and
cross-domain runtime-state projection. This extraction preserves the
existing service budgets, undock charging retry, target-source filtering, QoS,
and command path; it introduces no new admission or motion gate.

The executor-facing application edge is contained behind
`features/docking/lifecycle/docking_job_execution_module`. It is the single
concrete `DockingJobExecutionPort`, binds the canonical docking job store,
serializes pre-dock goal submission on the shared Nav2 action mutex, and keeps a
timed-out submission classified as an unresolved side effect until independent
evidence resolves it. Existing localization-settle, BMS-contact, terminal-stop,
speed-limit, reverse-permit, teleop-zero and runtime-state effects remain
explicit ports owned by the docking aggregate. This move adds no phase,
tolerance, retry, command path, or gate.

The parameter-composition edge is contained behind
`features/docking/configuration/docking_configuration_module`. It alone
declares the 100 established docking, predock, fine-entry, dock-contact and
undock parameters, preserves their defaults and dependent clamp order, and
returns ready-to-consume configs for the existing ten docking units. Shared
service timeout, Nav2 action name, map frame, pose freshness and BMS thresholds
remain explicit neighboring-domain inputs. The composition root retains no
parallel docking scalars, and this extraction changes no runtime value, TF
owner, command owner, admission rule, or safety path.

The navigation parameter-composition edge is contained behind
`features/navigation/configuration/navigation_configuration_module`. It is the
single declaration owner for the 112 established API-side navigation
parameters and constructs the existing navigation, terminal-runtime,
goal-execution, and goal-executor configuration graphs. Shared action names,
map/TF context, pose freshness, service timeout, and predock lateral-control
settings enter as explicit neighboring-domain inputs. Defaults, clamps,
compatibility parameters, goal acceptance, terminal correction, TF ownership,
and the final velocity chain are unchanged.

The API-side localization construction graph is contained behind
`features/localization/localization_configuration_module`. It uniquely
declares the 48 localization, TF-observation, AMCL-refinement,
bridge-acceptance, and settle parameters, normalizes the configured frame IDs,
and returns the existing localization and post-relocalization-settle configs.
The floor-health topic is an explicit floor-switch projection; the shared
service timeout and navigation-owned AMCL no-motion endpoint remain explicit
cross-domain values. This ownership move changes neither Isaac/AMCL sequencing
nor result freshness, bridge acceptance, settle policy, TF publishers, or
motion admission.

Runtime ownership for that graph is contained behind
`features/localization/localization_feature_module`. The aggregate owns the
localization facade and post-relocalization settle barrier and performs one
explicit second-phase attachment after navigation is constructed. This
resolves the localization-to-navigation costmap dependency before HTTP starts;
it does not add a runtime fallback, gate, timeout, topic, process, or TF
publisher. The API composition root owns one localization aggregate rather
than the two submodules and their port wiring.

The elevator runtime construction graph is contained behind
`features/elevator/configuration/elevator_runtime_configuration_module`. It
uniquely declares the 15 established adapter, external-arm-client, scoped-BT,
collision-bypass permit, and recovery-service parameters, and combines them
with explicit map, navigation, FloorSwitch, and safety inputs. It does not
implement or modify the mechanical-arm black box, execute an arm request,
change the elevator state machine, alter motion admission, or add a gate.

Commissioning-time manual predock selection and validation are contained behind
`features/docking/configuration/docking_predock_pose_resolver`. It preserves the
existing explicit-ID, conventional-ID, name-match, and unique-type precedence,
including conflict status codes, then applies the same optional distance and
mandatory yaw sanity checks. The composition root injects read-only pose-catalog
ports; this module writes no map asset and commands no motion.

The `docking_fine` bridge correction-pause lifecycle is contained behind
`features/docking/lifecycle/docking_correction_pause_module`. It applies and
releases the configured pause, updates the one canonical docking job and frozen
display pose, protects a running fine-docking owner, and clears only stale job
or bridge pause evidence. The original phase set, reason strings, timeout, and
logging severity are preserved; the module publishes neither TF nor velocity.

Persistent dock occupancy policy is contained behind
`features/docking/lifecycle/dock_contact_interlock_module`. It owns the latch
file schema/read-write transaction, evidence-source strength, stable BMS
charging-session update, weak-latch TTL, strong-session full-charge-idle
retention, confirmed-undock contradiction clear, final occupancy decision,
and the exact App JSON projection. The composition root injects immutable
runtime/BMS/navigation-job snapshots only. This extraction preserves every
existing threshold and clear condition and does not start undock, publish
velocity, or add an admission rule.

## Migration Phases

1. Document and test the target ownership contract.
2. Add read-only health checks for resident service readiness.
3. Add an opt-in resident navigation runtime entrypoint.
4. Move `map_server` and Nav2 ownership from `run_floor_navigation.sh` into the
   common resident layer. Done for the field runtime through
   `run_navigation_runtime_services.sh`.
5. Change `run_floor_navigation.sh` to compatibility-only. Done: it is blocked
   by default and delegates to the resident navigation runtime only with the
   explicit debug override.
6. Move task admission from script state into `robot_mode_manager` and
   `robot_mission_manager`.
7. Make App endpoints call intent APIs only: map, localize, navigate, dock,
   undock, cancel, stop.

## Current Runtime Entrypoints

`run_navigation_runtime_services.sh` is the selected-floor resident navigation
entrypoint. It verifies the already committed exact floor assets, launches
localization, sends one bounded global-localization trigger request, and
launches Nav2. It marks the runtime context ready only after fresh
`odom->base_link`, bridge-accepted `map->odom`, required Nav2 lifecycle states,
and `/global_costmap/costmap` are confirmed; configured AMCL tracking readiness
is also enforced. A missing required proof fails startup and tears down the
incomplete navigation/localization process set. Local perception and
`/safety/status` are not shell startup gates; safety remains authoritative at
API task admission and on the final velocity chain.
FAST-LIO2, `fastlio_odom_bridge`, `robot_local_state`, `robot_safety`, and the
Ranger chassis core are common resident services. Lower-level localization
and Nav2 scripts may start missing helper processes, but they must not kill or
repair canonical odom owners as part of navigation startup.

`run_floor_navigation.sh` remains for compatibility but is blocked by default.
Daily restarts must use `sudo systemctl restart njrh-runtime.service`, which
owns the foreground `run_common_services.sh` process and resident navigation
autostart. The wrapper only delegates to the resident entrypoint when
`NJRH_ALLOW_TRANSIENT_NAVIGATION_OWNER=1` is set for a debug-only manual run.
`run_occupancy_grid_localization.sh` and `run_nav2_navigation.sh` remain
lower-level repair/building blocks used by the resident runtime; they should not
be App-owned process lifetimes.

The API-facing mapping lifecycle is contained behind
`features/mapping/mapping_module`. It owns mapping-route dispatch, the one
asynchronous start transaction, mapping-owned process cleanup, live `/map`
cache lifetime, exact restoration proof for the canonical navigation `/scan`
publisher, and inactive immutable map-bundle saving. The API composition root
only supplies explicit cross-domain operations: floor/elevator admission,
navigation cancel/stop, and runtime-map-context clearing. This code boundary
does not move FAST-LIO2/JT128 ownership into the API and does not change TF,
DDS, pointcloud, LaserScan, or final velocity-chain contracts.

Aggregate ownership follows the same domain boundaries. `features/maps/
maps_feature_module` owns the runtime map-context store, cross-asset commit
mutex and complete maps module; `features/floor_switch/
floor_switch_feature_module` owns floor-switch configuration projections and
the atomic floor-switch module; and `features/mapping/mapping_feature_module`
owns the mapping module plus its navigation handoff ports. Their late provider
functions only resolve construction cycles before HTTP admission starts. No
map identity rule, switch timeout, recovery behavior, process command, scan
owner, or obstacle/TF contract changes with this ownership extraction.

The mapping-only App teleoperation lifecycle is contained behind
`features/teleop/teleop_module`. It owns `/ws/v1/teleop`, token and WebSocket
admission, the session/frame loop, command and reverse-permit ROS publishers,
watchdog repetition, subscription lease refresh, state JSON, and disconnect or
interlock stop publication. The composition root supplies only read-only
mapping/pose/BMS/elevator observations plus motion-admission and subscription
ports. Commands still enter `/cmd_vel_api -> robot_safety -> /cmd_vel ->
ranger_base`; this boundary move adds no new gate and gives the App no direct
chassis publisher.

The process-resident BMS input lifecycle is contained behind
`features/power/power_module`. It is the single owner of the
`/battery_state` subscription, SOC normalization, charging-contact evaluation,
contact/no-contact stability timers, freshness expiry, and the immutable BMS
snapshot. After committing a message and releasing its mutex, it forwards the
same evidence first to docking-latch policy and then to the teleop charging
guard. Navigation, docking, status, and teleop consume this one snapshot; the
API composition root keeps no parallel raw battery cache. This boundary move
adds no new gate and does not move charger, docking, estop, or velocity-chain
ownership into the API.

The read-only App status lifecycle is contained behind
`features/system_status/system_status_module`. It owns
`GET /api/v1/status`, `GET /api/v1/robot/pose`, the stable aggregate JSON
projection, and the single process-resident `/floor_manager/status`
subscription. Runtime mode, maps, navigation, docking, localization, safety,
power, subscriptions, and HTTP transport remain their own domains. The
`system_status_wiring` adapter owns the complete immutable observation
projection into the status ports, so the composition root no longer assembles
status, pose-runtime-context, or map-identity snapshots. The module does not
reinterpret those domains or publish any command; moving the response assembly here adds no
admission gate and leaves TF, safety, and final velocity ownership unchanged.

The API-facing navigation lifecycle is contained behind
`features/navigation/navigation_module`. It dispatches the complete
`/api/v1/navigation/*` surface and owns runtime start/reuse, goal admission,
pre-goal dock inspection, cancel/stop orchestration, and the lightweight
navigation-state response. The state projection keeps the established
bridge-over-AMCL precedence and distinguishes a transient localization
transition from a true recovery requirement; it combines immutable dock,
safety, post-relocalization, post-undock, goal, and cancel snapshots supplied
through explicit ports. Polling the state endpoint performs no synchronous
Nav2 lifecycle probe, no relocalization, and no motion. This extraction changes
no goal tolerance, controller, TF owner, docking owner, or velocity chain.

The navigation family is composed as one deep runtime module by
`features/navigation/navigation_feature_module`. That aggregate owns the
navigation lifecycle/HTTP module, terminal ROS runtime, goal-execution
transaction, goal executor, their delayed callback cycle, the localization
bridge-readiness projection, and the pre-navigation undock request adapter.
The process entry point supplies neighboring feature references once and keeps
no parallel navigation runtime pointers. This is an ownership-only change: the
existing admission decisions, stop acknowledgements, relocalization waits,
commercial pose gate, and command topics remain unchanged.

App page-subscription lifecycle is contained behind
`application/subscriptions/subscription_module`. It owns all three
`/api/v1/subscriptions/*` routes, compatibility client identity, TTL clamping,
lease/refcount expiry, resource transition semantics, and the page-scoped
high-rate `/scan` subscription/cache. The composition root provides narrow
ports only: keep status and TF resident, toggle the page-owned live map, and
clear teleop command state after the final lease. Consequently an App page
release cannot tear down safety/floor health or localization TF, while a lost
client still releases `/scan` and teleop through the same bounded TTL behavior.

The final shared bootstrap seam is contained behind
`application/runtime_configuration/runtime_configuration_module`. It uniquely
declares the Nav2 action name, action-status topic, and base ROS service
timeout, then projects the same construction-time values into navigation,
docking, elevator, localization, floor switch, and system status. The
composition root therefore declares no ROS parameter and retains no parallel
shared scalar. This changes no action/topic name, timeout, request, gate, or
motion owner.
This is an ownership move only; it changes no scan geometry/QoS/timestamp,
DDS, TF authority, mapping/navigation behavior, or final velocity chain.

Authenticated robot endpoint precedence is contained behind
`application/routing/application_router_module`. It captures exactly one
motion-admission epoch for each request, applies the global elevator execution
interlock first, and then preserves the established system-status, maps,
elevator, mapping, metadata, subscriptions, safety, floor-switch, localization,
navigation, and docking handler order. The routing module also owns the
reserved mapping/navigation `501` and final `404` responses. Its separate
wiring adapter binds concrete modules while keeping robot feature dependencies
out of HTTP transport infrastructure. No endpoint matching, response body,
authentication rule, motion gate, or feature algorithm changes in this move.

The public HTTP lifecycle is contained behind
`infrastructure/http/api_gateway_module`. It owns host/port/token resolution,
the `ROBOT_API_TOKEN` fallback, unauthenticated `OPTIONS`, shared token
validation, bounded transport startup/shutdown, WebSocket-session shutdown
ordering, access/event logging, connection counters, and the exact OpenAPI
catalog. The application router receives only authenticated normal requests;
socket sessions remain owned by the teleop module. This extraction changes no endpoint result, authentication
rule, elevator interlock, ROS publisher, TF authority, DDS setting, or velocity
path.

The ordinary-navigation terminal ROS edge is contained behind
`features/navigation/terminal_control/navigation_terminal_runtime_module`.
It implements the controller's runtime port and exclusively owns the final
command, terminal speed-limit and reverse-permit publishers together with the
Ranger mode-status, wheel-odometry, local-costmap and `/rosout` subscriptions.
It also owns their synchronized evidence caches, reverse-permit hysteresis,
physical-stop waits and MessageFilter-drop accounting. The composition root
only injects mission, localization, safety, docking-contact and drive-mode
callbacks, plus thin forwarding methods required by neighboring executor
interfaces. This is an ownership-only extraction: topic names, QoS, refresh
periods, stop thresholds/timeouts, zero-command cadence, costmap checks and
terminal success rules are unchanged, and no admission or motion gate is
introduced.

Within that aggregate, the post-localization motion handoff is contained behind
`features/localization/post_relocalization_settle_module`. It owns the full
accepted-sequence settle state machine: bridge publisher ownership and
sequence checks, canonical TF freshness, static LiDAR transform proof,
local-costmap update/drop evidence, large-correction minimum time,
zero-command cadence, cancellation and timeout reporting. Post-undock keeps
the existing warning-only treatment for transient costmap/drop/AMCL scan
evidence while all bridge/TF conditions remain hard. The localization
aggregate wires snapshots and effects. No threshold, error code, TF publisher, Nav2
parameter, safety decision, or velocity-chain edge changes with this move.

The controlled handoff from dock occupancy to normal navigation is contained
behind `features/docking/lifecycle/pre_navigation_undock_module`. It consumes
the already-classified immutable dock decision, then owns the whole transaction:
teleop zeroing, one docking-start mutex, concurrent-job rejection or active
undock reuse, stale fine-pause cleanup, manager readiness, canonical undock-job
creation, Trigger-service evidence, runtime/status observation, bounded wait,
and optional post-undock localization-readiness proof. The pending Nav2 goal is
released only after the established proof succeeds. This is an ownership move;
it preserves phase names, the 28-second undock budget plus configured
relocation allowance, charging retry, failure text, and the existing command
chain.

`robot_api_server` keeps core health subscriptions resident. `/safety/status`
and `/safety/motion_allowed` are reliable transient-local state topics and are
not released when an App page lease expires, so `/api/v1/status`, docking,
navigation, and teleop admission all read the same process-level safety cache.
That cache and the `/safety/estop` publisher live behind the complete
`features/safety/safety_module` boundary. Its configuration module uniquely
owns the three safety topic parameters; the module also owns both safety HTTP
routes, the floor/elevator-atomic resume transaction, and the ordered emergency
stop fan-out used for an unproven Nav2 terminal result. Cross-domain checks and
zero-command effects are injected ports, so the composition root contains no
parallel safety route or stop sequence. This remains a gateway boundary only
and does not move final command arbitration, watchdogs, or interlocks out of
`robot_safety`. The pure `safety_state` policy preserves the existing behavior:
missing motion evidence defers to the final arbiter, `COMMAND_STALE` is a
first-command warmup rather than a hard block, and other explicit denials are
hard blocks.

Ordinary goal execution is contained behind
`features/navigation/mission/navigation_goal_execution_module`. It is the one
concrete implementation of both the goal-executor and bridge-wait runtime
ports, so Nav2 submission/result evidence, final verification and bounded
recovery, correction-pause lifetime, and predock command-owner exclusion form
one transaction. `NavigationFeatureModule` composes its floor, undock,
localization, safety, docking and runtime-state ports; the application
composition module owns only that aggregate. This boundary adds no gate and changes
no goal tolerance, speed, TF authority, DDS setting or command-chain edge.
