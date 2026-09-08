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
            -> standard controlled undock when dock identity is known
            -> fail closed when a safe undock cannot be identified
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

Any failed proof blocks the pending navigation goal. It never clears a latch by
timeout alone and never starts a blind reverse without a resolved `dock_id`.

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

## Required hardware validation

Run these as separate supervised cases before commercial release:

1. Memory latch clear: a normal goal reaches Nav2 without any undock command.
2. Live charger contact: the existing standard undock completes before the held
   goal is released.
3. No contact but near the dock: the same standard undock path is selected.
4. Manually driven at least 1.5 m from the exact dock on the confirmed map: the
   stale latch is reconciled without chassis motion, then the original goal is
   released.
5. Stale BMS, stale map pose, wrong map identity, hysteresis-band position, and
   missing dock identity: each case remains stopped and reports its exact
   fail-closed reason.
