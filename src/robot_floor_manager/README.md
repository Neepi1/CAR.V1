# robot_floor_manager

`robot_floor_manager` owns only floor asset switching. It does not publish TF and does not alter FAST-LIO2, PGO, local perception, or Nav2 controller behavior.

Failure cleanup now separates released transaction resources from localization
validity: a failed switch can terminate as `FAILED` with an unready map, without
turning that unready map into a retained transaction lock. Pending writes must
still settle before a new conflicting switch. See
[`floor_failure_terminal_cleanup.md`](../../docs/floor_failure_terminal_cleanup.md)
for the corrected scope and verification boundary.

## Services

- `/floor_manager/switch_floor` (`robot_interfaces/srv/SwitchFloor`)
- `/floor_manager/floor_switch` (`robot_interfaces/action/FloorSwitch`)
- `/floor_manager/transition_status` (`robot_interfaces/msg/FloorSwitchStatus`,
  reliable + transient local)

The legacy service is source-preflight-only:

1. Require the exact
   `building_id/floor_id/map_id/expected_asset_epoch/expected_asset_digest`.
2. Validate only the immutable
   `maps_root/<building_id>/<floor_id>/maps/<map_id>/` source bundle through
   `FloorAssetSnapshotLoader`; `current/` may be absent or still point at a
   different map.
3. With `resume_navigation=false`, return the exact verified identity and
   source paths without changing or publishing `active`/`selected` runtime
   state. The API owns the later `current/` activation; a failed API commit
   therefore cannot leave a split floor-manager selection hint behind.
4. With `resume_navigation=true`, reject before asset validation or any ROS
   side effect with `LEGACY_RESUME_NAVIGATION_DISABLED`.

The Action endpoint now has a strict live transaction adapter, but
`live_floor_switch_enabled` deliberately remains `false` by default. With that
default, every accepted goal publishes typed `PREFLIGHT`/`BLOCKED` status and
aborts without a ROS or filesystem mutation. Enabling the adapter is a
deployment decision after isolated runtime validation; it is not enabled by
installing this package.
Each goal must identify one immutable target with
`building_id/floor_id/map_id/expected_asset_epoch/expected_asset_digest`.
`expected_asset_epoch` must be nonzero and the digest must be canonical
lowercase `sha256:<64 hex>`. The epoch and digest form one indivisible identity:
neither a digest match from another epoch nor a nonzero-but-different epoch is
accepted.
When explicitly enabled, the Action uses one managed worker thread so ROS
subscriptions and service responses keep progressing. It never detaches a
thread that can outlive the node.
Legacy selection state, Action arbitration, selected-asset snapshots, and typed
status generations share one mutex; a late cancel is rechecked before the
terminal Action transition.

The legacy service remains source-preflight-only and cannot reach the live
mutation path.

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
or costmap port. The offline legacy service uses it only as an exact
source-bundle preflight. It is not connected to the live Action callback, and
its service success is not a runtime/current commit. The API must still verify
the echoed identity and complete its own serialized activation transaction.
The gateway holds the cross-process map-asset commit lock from its first exact
snapshot through the service proof, a second exact snapshot, and activation.

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

The old floor-level/current projection remains a compatibility input for
existing runtime startup only. Offline selection preflight reads the immutable
source bundle, not `current/`, and neither projection is authoritative input
to a future atomic live `FloorSwitch` transaction.

## Live Action evidence contract

When `live_floor_switch_enabled=true`, one `FloorSwitch.action` goal executes
these barriers in order:

1. acquire the exact `robot_floor_manager:<transaction_id>` safety hold and
   prove fresh Nav2-idle plus stable stopped evidence from both `/wheel/odom`
   and `/local_state/odometry`;
2. acquire the exact floor-manager correction pause and call bridge `BEGIN`,
   which must report `runtime_context_valid=false`;
3. publish feedback stage `CALLER_PAUSE_HANDOFF_READY` with the exact
   transaction ID, monotonic effect sequence, and level-triggered
   `caller_pause_handoff_ready=true`. The flag remains true during
   `VERIFY_PAUSE_HANDOFF`, so a following feedback sample cannot erase the
   handoff. The elevator caller may then release only its own pause. The Action
   does not continue until fresh state proves the floor-manager lease is the
   sole remaining pause;
4. revalidate the immutable source snapshot, load the target Nav map and the
   keepout filter, load the speed filter only when
   `speed_filter_enabled=true`, then call the strong `ApplyFloorAssets`
   contract;
5. normally require both the Apply response and
   `/global_localization/asset_state` to echo the same transaction, target
   identity, newer localizer generation, and `reloaded=true`; if the RPC
   outcome is delayed or lost, reconcile only the exact fresh terminal typed
   state with the same requested and active identity;
6. release the floor pause, prove no pause remains, trigger explicit target
   localization, and require a newer exact-target localization sequence;
7. require typed pending-target bridge readiness and `amcl_ready=true`
   (`seeded/tracking-ready`, including valid stationary static standby), clear
   both costmaps, and observe a fresh post-clear message from each costmap;
8. call bridge `COMMIT`, require valid/safe target context, atomically publish
   `/tmp/njrh_runtime_map_context.json`, then release only the floor-manager
   motion hold.

The Action proves target asset and localization identity; it does not compare
numeric waypoint coordinates across floor maps. In particular, a target-floor
`cabin_panel` coordinate is never treated as a localization equality check.
The elevator caller must obtain a fresh target-map robot pose after completion
and let the next Nav2 action plan from that live pose.

Ordinary service calls retain the `10 s` timeout. Isaac component asset apply
has an independent `localizer_apply_timeout_sec` budget (default `30 s`)
because capture, unload, load, and exact component verification are one
composite operation. A late/lost Apply response is reconciled against the same
transaction, exact requested and active identity, `reloaded=true`, a newer
generation, and `localizer_ready=true`; the request is not repeated. The
explicit localization transaction has an independent
`localization_trigger_timeout_sec` budget
(default `75 s`) because its wrapper contains ordered post-reload readiness,
Isaac result, bridge acceptance, and canonical-TF barriers. A response timeout
is reconciled against the same exact target identity, newer localizer
generation, and newer explicit-localization sequence. Exact evidence may prove
a delayed/lost response as `OK_RECONCILED`; without that evidence the
transaction still fails and the existing recovery policy applies.

The live Action retries `/global_localization/trigger` only when the wrapper
proves `dispatch_state=not_dispatched`: localizer busy/post-reload/input
readiness, Isaac service unavailable, or bridge-arm service unavailable/timeout.
Once Isaac is dispatched, the wrapper itself owns old-result draining and
completion. Result, bridge, TF, and `map->odom` failures do not authorize a new
Isaac request; the Action instead reconciles a delayed/lost response against
the exact target generation and explicit sequence. Every safe pre-dispatch
retry keeps the same target identity, localizer generation, correction state,
motion hold, and original `75 s` deadline. Cancellation is checked throughout.
Wrong TF ownership, missing TF history, malformed/unknown failure codes, and
all post-dispatch failures retain the fail-closed recovery path.
The API and elevator runtime adapter use the same `120 s` outer Action window,
so neither caller can cancel the floor manager while its bounded `75 s`
localization sub-transaction is still valid.

Every `SetMotionHold` and owner-scoped `SetCorrectionPause` request uses one
process-generation sequence block reserved durably through
`motion_hold_sequence_state_file` (default
`/tmp/njrh_floor_manager_hold_sequence.state`). The node fails to start if that
absolute state file cannot be opened and reserved, fails closed if its block is
exhausted, and synchronizes to `applied_sequence` after a stale-command
response. A replacement process therefore cannot emit a sequence below any
still-in-flight command from the process it replaced.

Service timeout is an unknown outcome, not a rejected command. Pre-mutation
cleanup therefore always submits higher-sequence exact RELEASE commands for
both transaction keys, even when the local acquire flag was never set, and
requires later `MotionInterlockState` and `CorrectionPauseState` samples to
prove the exact keys absent. If that proof fails, it establishes a newer
motion hold and reports a recovery lock instead of a clean failure.

BEGIN invalidates the runtime context, but is not a target asset dispatch.
The worker records `target_effect_dispatched_` immediately before sending a
NavMap/filter load, localizer apply, explicit localization or costmap-clear
request. The record is monotonic within one transaction: timeout, exception,
cancellation and a failed response never reset it. It is not another lock.

If no target request was dispatched, cleanup after an accepted or unknown
BEGIN sends a higher-sequence exact
`OP_ABORT_PREMUTATION`; if it arrives first, its transaction tombstone rejects
the delayed lower-sequence BEGIN. If BEGIN arrived first, the bridge restores
the source context only after proving the exact source asset, unchanged
map-to-odom/localizer state, and the floor-manager pause. A later
`LocalizationHealth` sample must then prove the same building/floor/map,
epoch/digest, valid runtime context, ready localizer/bridge, and unique
canonical TF. Its explicit sequence must match the restore response, not the
possibly older preflight health cache. The proven fresh source sample supplies
the generation/sequence persisted to disk. A known successful BEGIN is no
longer sufficient by itself to retain a permanent lock. If exact pre-mutation
ABORT is acknowledged, the newer source
health sample is valid, the source runtime context is durably written back,
and both exact transaction leases are proven absent, the failed transaction
terminates as ordinary `FAILED` on the proven source floor. Any unproven
ABORT, source identity, durable write, or lease release retains the motion
hold, writes an unconfirmed failed context, and reports
`recovery_required=true`.

Once any target request may have been dispatched, ordinary `OP_ABORT` retains
invalid context and safety protection; it never claims to restore the source.
A delayed NavMap request cannot safely be undone just because its response was
lost. No automatic post-dispatch rollback or live recovery-lock reset was added.

Cleanup proves correction-pause release before it sends motion-hold release,
and releases only this transaction's keys. A proven recovered failure reports
the source identity and an explicit retryable-failure explanation, not target
success; it does not resume a navigation goal. Tests exercise the production
cleanup selector with the real bridge state machine and separately check the
node's dispatch/release wiring. No ROS interface or normal target COMMIT
requirement changed. See [audit repair record](../../docs/gate_audit_remediation_plan.md).

The runtime context writer uses a same-directory durable rename and preserves
the v1 compatibility fields while adding transaction ID, epoch/digest,
localizer generation, and explicit-localization sequence.

`speed_filter_enabled` defaults to `false`, matching the deployed Jetson
profile (`NJRH_ENABLE_SPEED_FILTER=false`). This flag changes only the live
Nav2 server/lifecycle requirement: keepout is always required and hot-loaded,
while speed is queried and hot-loaded only when enabled. It does not weaken
the immutable bundle contract; `speed_mask.yaml` and `speed_mask.pgm` remain
required and digest-verified members of every exact floor snapshot.

## Field Validation Still Required

- Run an isolated multi-node transaction with fake motion only and prove the
  exact `CALLER_PAUSE_HANDOFF_READY` handoff.
- Verify the bridge publishes pending-target `bridge_ready=true` before
  COMMIT while keeping `safe_for_goal_start=false` until COMMIT.
- Verify the Nav2 map and keepout server publish the new maps after their typed
  load acknowledgements. Repeat with `speed_filter_enabled=true` only in a
  profile that actually launches the speed mask server.
- Inject delayed motion-hold, correction-pause, and bridge-BEGIN responses in
  an isolated graph. Verify higher-sequence releases defeat late ACQUIRE,
  and verify a BEGIN timeout cannot become an ordinary failure without exact
  ABORT plus a newer source-identity health sample.
- Only after those gates pass, enable both live flags for a supervised runtime
  restart and controlled stationary validation. Real vehicle motion remains a
  separate acceptance step.

## P6 transaction core

`floor_transition_core` is the pure C++ safety contract used by the live
`FloorSwitch.action` adapter. It remains disconnected from the legacy service.

Its ordered barriers are:

1. prove a fresh motion hold, the latest latched Nav2 action state is idle, and
   fresh stopped evidence;
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
The runtime port may clear recovery after source-context invalidation only
when exact ABORT, fresh source identity, durable source-context restoration,
and exact lease absence have all been proven. That path terminates as
`FAILED`, not `FAILED_LOCKED`. An incomplete proof remains `FAILED_LOCKED`;
the core never infers physical floor location from a timeout.

The core GTests cover the ordered success barrier, pre- and post-mutation
failures, idempotent transaction replay, zero/incorrect epoch rejection,
non-canonical and stale digest rejection, exact epoch propagation to effects,
and retry with a stable cleanup ID. The cross-package
`robot_elevator_manager/test/test_nonmoving_cross_floor_scenario.cpp` composes
the core with the elevator, mission, safety, mode, and correction-pause cores
and verifies a synthetic success path.

The executor, typed evidence tracker, and atomic runtime-context writer have
isolated GTests. The live adapter still does not change the deployed runtime
while its flag is false. The bridge also keeps live BEGIN/COMMIT disabled by
default. Real cross-floor switching and elevator exit remain disabled until
isolated integration and supervised hardware acceptance pass.

`/navigate_to_pose/_action/status` is event-driven rather than a heartbeat.
The evidence tracker therefore latches the latest active/terminal action state
until the next status event. `evidence_max_age_sec` continues to expire the
motion-hold and wheel/local-odometry samples, but it no longer expires an
already observed terminal Nav2 result during a manual door confirmation.

A newly started NavigateToPose action server does not publish an initial empty
`GoalStatusArray` when it has never accepted a goal. The floor manager now
probes both the `/navigate_to_pose` action server and the discovered status
publisher. If both remain continuously available for
`nav_idle_bootstrap_grace_sec` (default `2.0 s`) and no retained or live status
sample arrives, that bounded absence is accepted as the cold-start idle state.
An `ACCEPTED`, `EXECUTING`, or `CANCELING` sample overrides it immediately;
loss of the action graph clears both bootstrap and terminal idle evidence.
This removes false `NAV_IDLE_UNPROVEN` failures without treating a missing
Nav2 server or status writer as idle.

### Unlocalized startup handoff (2026-09-09 deployment)

When both `/bt_navigator/get_state` and `/controller_server/get_state` have
unique expected owners and fresh explicit unconfigured/inactive responses,
the node can prove Nav idle without an action-status publisher. An observed
active goal still vetoes this path; graph absence alone never proves idle.
The same safety hold and fresh stopped wheel/local odometry remain required.

A cold-start switch first hands the exact immutable target to the resident
startup owner. Only after old workers have settled does it perform BEGIN,
load target maps/masks, reload Isaac assets, and explicitly localize. Target
pending TF may start Nav2/AMCL without falsely granting ordinary navigation.
Full target AMCL, post-clear costmaps and bridge COMMIT are still mandatory.
The request/ack files carry transaction, nonce, digest/epoch and localization
generation; they do not grant motion. Cancellation publishes a failed request
before transaction cleanup to prevent new startup steps. Old startup writers
cannot overwrite the selected target.

Implementation and evidence: `docs/floor_switch_unready_startup.md`. The candidate
was deployed, but restart acceptance is blocked by the common startup's missing
fresh `/dock/target_observation`; automatic restarts were stopped. Hardware
floor-switch acceptance remains pending. The change
does **not** automatically clear an unknown RPC outcome or a failed transition
after target mutation. Those cases retain the existing stop/recovery protection.

The Nav2 graph probe is deliberately graph-only: it checks the three hidden
`NavigateToPose` action services plus the status publisher and does not create
an Action Client. The node uses a single-threaded ROS executor because the
bounded floor transaction already runs on its managed worker thread. The exact
Humble wait-set exception `Taking data from action client but no ready event`
is classified as retryable at the process boundary; unrelated exceptions
remain fatal. This prevents a read-only readiness probe from aborting the
floor-manager process while preserving strict failure behavior elsewhere.
