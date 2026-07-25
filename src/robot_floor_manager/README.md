# robot_floor_manager

`robot_floor_manager` owns only floor asset switching. It does not publish TF and does not alter FAST-LIO2, PGO, local perception, or Nav2 controller behavior.

## Services

- `/floor_manager/switch_floor` (`robot_interfaces/srv/SwitchFloor`)
- `/floor_manager/floor_switch` (`robot_interfaces/action/FloorSwitch`)
- `/floor_manager/transition_status` (`robot_interfaces/msg/FloorSwitchStatus`,
  reliable + transient local)

The legacy service is selection-only:

1. Validate `maps_root/<building_id>/<floor_id>` assets.
2. With `resume_navigation=false`, record the selected floor assets for the next navigation start without requiring `/map_server`, Isaac localization, or Nav2 to be running.
3. With `resume_navigation=true`, reject before asset validation or any ROS
   side effect with `LEGACY_RESUME_NAVIGATION_DISABLED`.

The Action endpoint is currently a non-mutating strict preflight adapter.
`live_floor_switch_enabled` defaults to `false`; every accepted goal publishes
typed `PREFLIGHT`/`BLOCKED` status and aborts with a structured failure code.
Each goal must identify one immutable target with
`building_id/floor_id/map_id/expected_asset_epoch/expected_asset_digest`.
`expected_asset_epoch` must be nonzero and the digest must be canonical
lowercase `sha256:<64 hex>`. The epoch and digest form one indivisible identity:
neither a digest match from another epoch nor a nonzero-but-different epoch is
accepted.
The adapter intentionally has no map, filter, localizer, bridge, or costmap
mutation port. It is not a live cross-floor switch.
Because this preflight is short and non-blocking, the accepted callback executes
it synchronously instead of detaching a thread that can outlive the node.
Legacy selection state, Action arbitration, selected-asset snapshots, and typed
status generations share one mutex; a late cancel is rechecked before the
terminal Action transition.

The old helper methods for map/filter/localizer operations remain internal
implementation inventory only. They are unreachable from the enabled legacy
path and must not be treated as a production transaction.

## Read-only source bundle snapshot

`FloorAssetSnapshotLoader` is a pure C++ boundary for exact, non-mutating
source-bundle verification. Its request contains
`maps_root/building_id/floor_id/map_id/expected_asset_epoch/expected_asset_digest`.
It first requires an exact tuple match from the authoritative persistent epoch
registry, then opens only
`maps_root/<building>/<floor>/maps/<map_id>`. A successful result returns the
verified identity, exact role paths, and file fingerprints.

On Linux the loader walks the bundle with `openat(..., O_NOFOLLOW)`, requires
single-link regular files, holds descriptors through digest calculation, and
revalidates inode, size, mtime, ctime, directory identity, and registry
identity before returning. It rejects unsafe IDs, path-component symlinks,
required-file symlinks/hardlinks, ambiguous Nav/localizer filenames, malformed
or duplicate-key manifest JSON, non-canonical epochs, and all epoch/digest or
content drift. The 11 canonical digest roles are each capped at 512 MiB and
their aggregate is capped at 1 GiB. They are hashed from held file descriptors
in 64 KiB chunks through `CanonicalMapAssetDigestStream`; their bodies are
never retained in `OpenFile` objects or assembled as `DigestEntry` strings.
The Windows fallback likewise fingerprints all roles first and then feeds one
file at a time to the same streaming digest contract.

`manifest.json` is the only retained source file and is capped at 1 MiB because
its strict JSON identity fields must be parsed. `poses.yaml` remains capped at
64 MiB but is only opened, validated, fingerprinted, and revalidated; its body
is neither retained nor hashed. Both files are intentionally excluded from the
11-role canonical content digest.

The loader has no bind/write method and no ROS, localizer, map-server, bridge,
or costmap port. It is not connected to the Action callback yet, so it cannot
block a ROS executor or authorize a live switch.

## Required source assets

```text
maps_release/<building_id>/<floor_id>/maps/<map_id>/
  manifest.json                    # njrh.map_manifest.v2
  nav/<safe_map_name>.yaml
  nav/<safe_map_name>.pgm
  localizer/<safe_map_name>.png
  localizer/<safe_map_name>.yaml
  filters/keepout_mask.yaml
  filters/keepout_mask.pgm
  filters/speed_mask.yaml
  filters/speed_mask.pgm
  filters/binary_mask.yaml
  filters/binary_mask.pgm
  reports/asset_report.json
  poses.yaml
```

The old floor-level/current projection remains a legacy compatibility input
for selection and existing runtime startup. It is not authoritative input to
the strict snapshot loader or a future atomic `FloorSwitch` transaction.

## Field Validation Still Required

- Prove the Isaac localizer actually reloads the target PNG/parameters instead
  of only caching paths.
- Connect the read-only exact bundle snapshot to a non-blocking transaction
  adapter; do not run its filesystem verification on a ROS executor callback.
- Wire the Action to owner-scoped hold/pause, bridge begin/commit/abort, typed
  Nav2 map/filter reload, explicit target localization, fresh costmaps, and
  final readiness.
- Only after those gates pass, validate the whole transaction under one
  supervised runtime restart and then perform controlled hardware acceptance.

## P6 transaction core

`floor_transition_core` is a pure C++ safety contract for the future live
`FloorSwitch.action` adapter. The node now exposes the Action name, but only
the non-mutating preflight is wired. The core is not connected to the legacy
service and does not authorize live asset mutation.

Its ordered barriers are:

1. prove motion hold, Nav2 idle, and fresh stopped evidence;
2. acquire a floor-manager-owned correction pause;
3. invalidate the ordinary source runtime context;
4. report the begin barrier, then verify the caller released only its own pause
   while the floor pause remains effective;
5. load Nav map, enabled filters, and actual localizer assets from one exact
   digest/epoch; every target evidence barrier requires
   `asset_epoch == expected_asset_epoch` and an exact digest match;
6. release the floor pause and prove no other pause remains before target
   explicit localization;
7. require a nonzero explicit-localization sequence and bridge-ready target
   building/floor/map/digest/epoch;
8. clear both costmaps with typed Nav2 adapters and wait for fresh target-epoch
   global and local costmaps;
9. commit `runtime_context_valid=true` and `safe_for_goal_start=true` only at
   the final barrier.

Failure or cancellation produces a retryable `HOLD_AND_LOCK` cleanup effect.
After source-context invalidation, every failure keeps the context invalid and
sets `recovery_required=true`; the core never claims that a physical
cross-floor move was rolled back.

The core GTests cover the ordered success barrier, pre- and post-mutation
failures, idempotent transaction replay, zero/incorrect epoch rejection,
non-canonical and stale digest rejection, exact epoch propagation to effects,
and retry with a stable cleanup ID. The cross-package
`robot_elevator_manager/test/test_nonmoving_cross_floor_scenario.cpp` composes
the core with the elevator, mission, safety, mode, and correction-pause cores
and verifies a synthetic success path.

These tests do not change the live runtime. A preflight-only
`FloorSwitch.action` server exists, and `robot_localization_bridge` exposes the
separate `BeginFloorTransition` fence, but the floor-manager Action is not yet
connected to it or to real asset reload, bridge readiness, or typed costmap
evidence. The bridge also keeps live BEGIN/COMMIT disabled by default. Real
cross-floor switching and elevator exit remain disabled until those adapters
pass isolated integration and supervised hardware acceptance.
