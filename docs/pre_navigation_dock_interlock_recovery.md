# Pre-navigation dock interlock recovery

## Purpose

Every ordinary navigation job now resolves the final `robot_safety` BMS docking
memory interlock before a Nav2 goal is submitted. This prevents a robot that was
manually driven away from the charger from remaining blocked by an old in-memory
contact latch, while retaining the conservative controlled-undock behavior when
the physical dock state is not proven.

This flow reuses the existing `PreNavigationUndockModule` and the existing
`/docking/undock` motion path. It does not create a second reverse-motion owner,
does not publish velocity from the API, and does not weaken `robot_safety` as the
final `/cmd_vel` arbiter.

## Decision flow

```text
ordinary navigation request
  -> require fresh /safety/dock_interlock_state
  -> BMS memory latch clear
       -> continue through the existing dock checks
  -> BMS memory latch set
       -> live charging contact
            -> standard controlled undock -> normal readiness checks -> Nav2
       -> robot at or within 1.0 m of the commissioned dock pose
            -> standard controlled undock -> normal readiness checks -> Nav2
       -> fresh map pose at least 1.5 m from the exact commissioned dock pose
            -> constrained no-motion reconciliation -> Nav2
       -> 1.0--1.5 m hysteresis band or position/map/dock identity uncertain
            -> standard fixed-distance controlled undock (dock_id optional)
```

The no-motion reconciliation is allowed only when all of these are true:

- the API proves the robot is outside the exact dock zone on the confirmed
  building/floor/map;
- the robot has a fresh `map`-frame pose;
- BMS feedback is fresh and has continuously reported no contact for at least
  3 seconds;
- `/docking/status` does not report docked or charging;
- no controlled-undock reverse permit is active;
- no fresh docking command is active.

Any failed no-motion reconciliation proof blocks the pending navigation goal.
It never clears a latch by timeout alone. Physical fixed-distance undocking is
a separate path: once existing occupancy evidence requires it, missing
`dock_id` does not block the existing odometry-confirmed departure.

## Fixed-distance undock identity correction (2026-09-16)

The field navigation job failed at 21:42:18 UTC+8 because the API required a
resolved dock identity before calling `/docking/undock`, even though occupancy
already required departure. Manual departure then completed with 0.610 m of
measured travel. The Trigger service has no dock-ID argument; its distance and
speed come from the existing manager configuration and its completion uses
odometry, not a commissioned map pose.

`PreNavigationUndockModule` now retains a known dock ID only as metadata and
accepts an empty one. No placeholder identity or map/nearest-dock lookup is
introduced. Existing status/job completion clears on-dock memory with an empty
ID, and post-undock localization already uses `unknown_dock` only in its reason
label. Physical completion, configured relocalization and pending-goal release
retain their existing semantics. The separate geometry-based, no-motion stale
interlock reconciliation still requires an exact dock identity.

Only the automatic departure's empty-ID rejection changes. Occupancy admission,
ownership cleanup, velocity/distance, safety arbitration, manual departure,
return-to-dock and all elevator policy are unchanged. Isolated regressions cover
missing/known identity, service failure, post-undock localization failure and
unchanged no-motion reconciliation. Production activation completed on
2026-09-16 through the user-authorized complete runtime restart. The running API
matches the isolated-tested candidate SHA256
`63e5d4893130998d2b390172e27fda594685692c50dea3c6a350be48a522ffae`.
Navigation and AMCL are ready, and docking sensor health is confirmed. A
supervised navigation-from-dock motion test remains required; no motion test
was performed during deployment. Evidence is under
`/tmp/njrh_reports/undock_optional_id_20260916TdqmyUD`.

## Interfaces and ownership

- `/safety/dock_interlock_state`
  (`robot_interfaces/msg/DockSafetyInterlockState`) is a reliable,
  transient-local state snapshot published by `robot_safety`.
- `/safety/reconcile_dock_interlock`
  (`robot_interfaces/srv/ReconcileDockInterlock`) is the constrained request for
  reconciling an already-proven remote-undock state.
- `robot_safety` alone clears its private in-memory latch after validating live
  BMS, docking-command, status, and reverse-permit conditions.
- `robot_api_server` alone evaluates the commissioned dock-zone geometry and,
  after safety accepts reconciliation, clears the existing persistent
  `docking_contact_latch.json` through its current latch owner.
- Standard physical undocking remains owned by `robot_docking_manager` through
  `/cmd_vel_docking -> robot_safety -> /cmd_vel`.

The state topic is required and must be no older than 1 second at preflight.
The reconciliation service timeout is 3 seconds. These values, the 0.5-second
map-pose freshness bound, and the 1.0/1.5 m dock-zone hysteresis are configured
in both `robot_api_server.yaml` copies. The required stable BMS no-contact period
is configured in both `robot_safety.yaml` copies.

## Diagnostics

`GET /api/v1/navigation/pre_goal_check` and navigation state now expose:

- `safety_interlock.available`, `fresh`, `memory_latched`, `active`, and BMS
  evidence;
- `dock_zone.state`, reason, dock identity, and distance;
- `pre_navigation_recovery_action`, which is `NONE`, `CONTROLLED_UNDOCK`,
  `CLEAR_STALE_INTERLOCK`, or `BLOCK`;
- `pre_navigation_block_reason` and `resolved_dock_id`.

The safety reconciliation response uses explicit result codes for missing
outside-zone proof, stale BMS, live contact, insufficient no-contact duration,
docked runtime status, active reverse permit, and active docking command.

## Memory-only undock admission and completion

The 2026-09-10 field case exposed a split contract: the API selected
`CONTROLLED_UNDOCK` from fresh safety memory plus `NEAR`, while the docking
manager accepted only `Docked`, live contact, or the persistent latch. The
manager now consumes the existing `DockSafetyInterlockState` topic directly
and accepts its enabled, active, fresh memory latch as an additional explicit
undock admission source. Source and receipt ages must both be within the
existing API freshness default (1.0 s). No map query, lease, new service, synthetic
Docked state, or velocity publisher is introduced. Docked/contact/persistent-latch
admission and ContactStopping remain unchanged; the new topic does not become
a prerequisite for those existing paths.

Safety clearing now distinguishes a successful undock from a failed/cancelled
reverse session. It consumes the existing ordered `undocking ...` then
`undocked phase=succeeded ...` status progression, resets success evidence for
each new reverse attempt, and requires reverse disabled plus fresh BMS
no-contact before clearing. Either status-first or permit-first delivery works.
Failure/stop does not impersonate successful departure. The separate proven
outside-dock reconciliation service remains unchanged.

Hardware-free regression:
`src/robot_system_tests/test/test_dock_memory_undock_isolated.py` runs the real
candidate executables in a separate network namespace, ROS domain 183, with all
motion outputs remapped to `/dock_memory_test`. It covers each admission source,
stale/missing/clear memory, full-battery-only rejection, failed/cancelled cleanup,
both success callback orders, old-success rejection, live contact retention, and
both nodes together with fake odometry success/failure. It refuses normal host
networking and does not launch any chassis or camera driver.

Candidates must be deployed together during an authorized full restart window.
Only the two changed translation units are rebuilt; existing supporting link
inputs are retained. Never replace a running executable under the exact-exe
health guard: stage it first, then activate while its owner is stopped.

## Required hardware validation

Run these as separate supervised cases before commercial release:

1. Memory latch clear: a normal goal reaches Nav2 without any undock command.
2. Live charger contact: the existing standard undock completes before the held
   goal is released.
3. No contact but near the dock: the same standard undock path is selected.
4. Manually driven at least 1.5 m from the exact dock on the confirmed map: the
   stale latch is reconciled without chassis motion, then the original goal is
   released.
5. Missing dock identity with retained on-dock evidence: perform the standard
   fixed-distance departure, clear occupancy after success, relocalize, and
   continue the original goal. A rejected or failed departure must not release
   the goal. Missing identity still cannot authorize no-motion latch clearing.
6. Other existing stale-evidence and map/ownership checks retain their current
   behavior; this correction does not relax them.
