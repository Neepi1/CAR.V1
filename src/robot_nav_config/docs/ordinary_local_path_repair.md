# Ordinary local path repair

The [body geometry candidate](body_geometry_alignment.md) changes the derived
preferred first-pass envelope to `0.63/0.52 m` and hard fallback to
`0.58/0.47 m`, while preserving both margin parameters and search logic.
The parameter shared with the MPPI critic is its separate 0.05 m
preference width, not the native inflation-derived collision penalty. Search
passes split the existing budget; final attachment retains hard clearance.

## Why this exists

The production `ranger_lattice` behavior tree deliberately retains a globally
valid path. The global costmap contains static, inflation, and keepout data; the
rolling local costmap is the map that contains people and other transient scan
obstacles. Therefore a local-only obstacle can make MPPI's current reference
path unusable without causing `ComputePathToPose` to publish a replacement.

The field trace at `2026-09-07T13:09:13.851+08:00` recorded 24 lethal local
cells inside the center 1 m while `/plan` had been published only once, at
`2026-09-07T13:08:59.535+08:00`. MPPI continued resampling velocities against
the same reference path. This module repairs that missing path-update layer; it
does not replace MPPI.

## Runtime contract

`OrdinaryLocalPathRepairRuntime` is owned by
`GoalScopedRotationShimController`, but its planning, worker, state, parameters,
and progress heartbeat are isolated from that wrapper.

1. While ordinary `FollowPath` is active and farther than 0.75 m from the final
   goal, the runtime inspects the next 4.0 m of the active reference path every
   0.20 s.
2. Inspection uses an immutable copy of the rolling local costmap and the
   preferred rectangle when enabled; otherwise it uses the padded rectangular
   Ranger footprint expanded by the retained 0.08 m hard repair margin.
3. A blocked result starts a 0.20 s persistence interval, **without commanding
   a stop**. The 4 m reference inspection is not a braking distance. MPPI still
   evaluates the current local trajectories; the smoother/collision/safety
   chain remains authoritative. Background search does not pause progress.
4. Persistent blockage is searched on the worker thread with forward-only
   Dubins Hybrid A*, 0.81 m minimum turning radius, and no reverse, lateral, or
   spin primitives.
5. Rejoin candidates extend beyond the inspection horizon within the current
   local map, with the entire footprint in bounds. Up to three spatially
   separated exits (far/middle/near) are tried with independent search graphs
   and a shared request time/iteration budget. Humble's existing deadline is
   checked every 64 expansions, instead of every 5000. The worker reuses the
   expensive Dubins lookup table for identical resolution/kinematics, never
   the visited graph. One-time cold table construction is outside the 0.40 s
   shared budget; subsequent setup and all candidate attempts share it. This
   is a soft wall-time budget, not a hard real-time guarantee: an individual
   expansion cannot be preempted. All table construction is on the worker.
6. A result is checked again against the latest pose and local costmap. If the
   original path has cleared, the detour is discarded. Old clear responses are
   also reinspected. A result older than inspection period + twice the search
   budget is discarded. A bounded forward Dubins connector (the already
   installed OMPL implementation, same 0.81 m radius) attaches the current
   pose to the returned path; its entire sampled footprint must be clear.
   Long loop-around connectors are rejected and a fresh search is requested.
   The original exact goal and unaffected suffix are retained.
7. The wrapper calls the existing MPPI `setPlan()` without rearming startup
   RotationShim and without ending or replacing the Nav2 action.
8. No local rejoin means a background retry every 0.50 s, not automatic zero.
   Only the configured MPPI plugin's exact Humble 1.1.19 error
   `Optimizer fail to compute path` becomes zero-command `WAIT_CLEAR` while
   retaining FollowPath. MPPI is called again on every control cycle, even in
   this wait. A valid command resumes immediately; persistent no-control does
   not consume the ordinary 12 s progress allowance. This error alone does not
   prove a physical obstacle exists. Unknown controller errors and search-worker
   exceptions propagate rather than masquerading as obstacle waiting.

Near-goal terminal handoff remains the owner inside 0.75 m. Entering that scope
invalidates any outstanding local-repair result so an old worker response
cannot block terminal completion. Explicit action/lifecycle resets retire old
control-wait state. No new navigation action, recovery motion, lease or gate is
introduced. The existing BT/action server still handles user cancellation.

## Fixed invariants

- Global costmap contents and `ranger_lattice` BT semantics are unchanged.
- Ordinary goal tolerances remain 0.06 m / 0.05 rad.
- Elevator and docking controllers are unchanged.
- The final command chain remains Nav2 -> velocity smoother -> collision
  monitor -> robot safety -> `/cmd_vel` -> Ranger base.
- The App neither publishes chassis velocity nor creates a replacement goal.
- No costmap clear, recovery spin, reverse motion, or relocalization is used.

## Parameters and observability

All parameters are under `controller_server.ros__parameters.FollowPath` and use
the `ordinary_local_repair_*` prefix. The source and Jetson runtime-overlay
`nav2.yaml` files must remain identical for these values.

Accepted replacement paths are published on
`/ranger_mini3/ordinary_local_repair_path`. Controller logs include the first
blocked index, result status, selected rejoin index, search iterations, pose
count, and maximum lateral deviation. Progress states share the existing
`/ranger_mini3/nav_elevator_scoped_progress_state` heartbeat with distinct
ordinary-local-wait enum value 6 only for actual MPPI no-control. Background
repair publishes ordinary state 0 and keeps normal progress checking; enum 5
remains reserved for compatibility and is no longer emitted by this runtime.

## Verification

`test_ordinary_local_path_repair` reproduces a straight path blocked by a
person-width lethal barrier. It requires a successful Ackermann detour, an
exact original endpoint, and no intersection between any returned pose's
expanded rectangle and any lethal cell. It also proves that a clear reference
path is not rewritten. `test_ordinary_local_path_repair_worker` verifies plan
generation and unexpected exceptions survive the background boundary, preventing
a stale result from being mistaken for the current goal. Additional tests cover
the 4 m boundary, a failed far exit followed by a reachable alternative, moving
pose attachment and a newly obstructed connector. Geometry uses the production
0.47/0.36 m repair half-envelope, including Nav2 padding and MPPI margin.
`test_ordinary_local_path_repair_runtime` runs the actual runtime on an isolated
synthetic costmap: distant blockage does not hold or pause progress, 300
consecutive no-control cycles retain retry capability and recover, unknown
errors propagate, other plugins are excluded, and action/lifecycle resets clear
wait state. These are deterministic software tests, not a moving-robot success
rate. The footprint/inflation contract test
binds the repair margin and turning radius to MPPI and Ranger values.

A supervised moving-person hardware run is still required before declaring
field acceptance. Record the repair-path topic, local-costmap summaries,
`/cmd_vel_nav_raw`, `/cmd_vel_collision_checked`, and Nav2 result while avoiding
PointCloud2 and LaserScan subscriptions.
