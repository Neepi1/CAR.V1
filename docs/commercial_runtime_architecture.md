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

## Resident Services

These services are expected to be started by the boot supervisor or the
container common-service layer and kept alive during normal operation:

| Service | Owner | Production role |
| --- | --- | --- |
| JT128 driver and canonical remap | `robot_hesai_jt128` / runtime overlay | Sensor ingress |
| Ranger chassis driver | `robot_chassis_bridge` / Ranger driver wrapper | Wheel odom and command sink |
| Static robot TF | `robot_description` | Sensor extrinsics |
| FAST-LIO2 runtime | `robot_fastlio_mapping` wrapper | Resident mapping/diagnostic frontend; optional explicit FAST-LIO local-state source |
| Local state | `robot_local_state` | Only `odom->base_link` publisher, default wheel-only EKF with corrected IMU kept resident for safety-side spin-tail detection |
| Global localization service | `robot_global_localization` | Asset reload and relocalization trigger |
| Localization bridge | `robot_localization_bridge` | Only `map->odom` publisher |
| Local perception | `robot_local_perception` | `/perception/obstacle_points` and clearing cloud |
| Safety arbiter | `robot_safety` | Final command gate |
| Ranger mode controller | `ranger_mini3_mode_controller` | Chassis-mode adaptation |
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
  -> ranger_mini3_mode_controller
  -> chassis
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
entrypoint. It resolves the floor assets, launches localization, sends one
bounded global-localization trigger request, launches Nav2, and marks the
runtime context ready once both child processes survive the initial settle
window. It does not report startup failure solely because a shell probe missed
`map->odom`, `odom->base_link`, `/global_costmap/costmap`,
`/perception/obstacle_points`, `/safety/status`, or Nav2 lifecycle state.
FAST-LIO2, `fastlio_odom_bridge`, `robot_local_state`, `robot_safety`, and the
Ranger mode controller are common resident services. Lower-level localization
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

`robot_api_server` keeps core health subscriptions resident. `/safety/status`
and `/safety/motion_allowed` are reliable transient-local state topics and are
not released when an App page lease expires, so `/api/v1/status`, docking,
navigation, and teleop admission all read the same process-level safety cache.
