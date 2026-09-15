# FlatScan Management Phase 3

## Ownership

The pipeline shell remains the child owner. Confirmed child exit still uses its
original helper restart function, limits, backoff and cooldown. The C++ observer
supplies evidence, never recovery commands. Missing observations cannot prove
producer failure. No production connection, restart, movement or ROS test was
performed by this task. Parent owns build integration and deployment.

## Parent integration API

Header: `robot_bringup/runtime_flatscan_monitor.hpp`.
Class: `robot_bringup::RuntimeFlatScanMonitor`.

```cpp
// Guard constructor, after Node is constructed:
flatscan_ = std::make_unique<robot_bringup::RuntimeFlatScanMonitor>(*this);

// Once per existing sampling cycle. No executor or spin call is needed:
flatscan_->tick(m);

// Once per existing graph refresh, inside its try block, on the same Node:
// Reuse /scan's count and add the other two topics to the existing pass.
flatscan_->update_graph(m, true, scan_count, flatscan_count, metadata_count);

// Graph exception must invalidate evidence, not appear as zero publishers:
flatscan_->update_graph(m, false, 0, 0, 0);

// In write_snapshot(), before the existing single atomic commit:
put_json(d, "flatscan_monitor", flatscan_->snapshot(a), a);
```

Graph topics: `/scan`, `/flatscan`, `/global_localization/flatscan_input_status`.
Do not add a timer, executor or graph-query loop for this module. The defaults
are 3 s metadata age and 15 s graph-cache age; these classify observer
availability, not producer failure or navigation readiness. Optional constructor
age arguments should track non-default guard sampling/graph periods if used.
`snapshot()` retains original metadata emission/receive times, never refreshing
them simply because the guard copied the evidence.

Build integration (deliberately not edited by this task):

- Compile `src/runtime_flatscan_monitor.cpp` into `runtime_health_guard`.
- Existing rclcpp, std_msgs and RapidJSON dependencies cover the monitor.
- Add/install executable `runtime_flatscan_check` from
  `src/runtime_flatscan_check.cpp`, with the same RapidJSON include setup as
  `runtime_health_check`. Do NOT link ROS/rclcpp into this reader.
- Register `test/test_runtime_flatscan_snapshot.cpp` as a C++17 executable test
  with RapidJSON and the package include directory.
- Run `python -m pytest src/robot_bringup/test/test_runtime_flatscan_management.py`.

## Metadata source

Global localization publishes a separate 1 Hz String JSON topic
`/global_localization/flatscan_input_status`. The legacy health String is
unchanged. Added fields include boot identity, process generation, timer
sequence, actual input receive sequence, steady receive/emission times and
header stamp. One extra steady clock sample is taken in the existing FlatScan
callback. There is no extra FlatScan/LaserScan/pointcloud/odom/IMU subscription.

IMPORTANT: The existing FlatScan subscription is lazy. Before the first trigger
or post-reload input wait it may not exist; this change does not create it.
Early startup, disabled input freshness, mock mode, alternate input topic,
mapping without global localization, absent samples and missing/duplicate
metadata publishers all mean observer-unavailable, not healthy or failed.

This metadata describes transport reception, NOT successful matching or input
suitability. Existing header age, FOV, point-count and freshness checks remain
unchanged and authoritative for localization requests.

The monitor independently tracks a strictly increasing FlatScan header stamp
watermark, scoped to the localizer process generation. The first sample only
establishes a baseline; a later received frame must advance its header before
health is available. Timer heartbeats do not renew the last stamp advancement
time. New receive sequences with equal/regressed frame stamps mark observation
UNKNOWN immediately, even when receive timestamps and heartbeat sequences keep
advancing. Recovery requires a frame above the previous watermark; a new source
generation must establish its own baseline and advancement. This observer-only
bookkeeping does not change any original localizer input or trigger gate.

The shared monitor object adds `stamp_progress`: `generation`, `seen_advancing`,
`replay_detected`, `header_watermark_sec`, `last_advance_monotonic_sec`, and
`last_advance_input_sequence`. Existing snapshots without it are UNKNOWN rather
than implicitly healthy. Native/ROS fixtures must publish a positive progressing
`header_stamp_sec`; the former constant-zero smoke fixture is no longer valid.

## Checker and supervisor contract

```
runtime_flatscan_check FILE MAX_SNAPSHOT_AGE STREAM_TIMEOUT
```

- Exit 0: recent FlatScan reception with proven frame-stamp advancement observed.
- Exit 40: unavailable/stale/malformed observer, wrong boot, invalid clock,
  absent input or duplicate/missing metadata publisher. No CLI fallback/restart.
  This also includes repeated/regressed frame stamps with continuing receives.
- Exit 50: fresh observer reports no reception beyond the existing
  `NJRH_FLATSCAN_MESSAGE_CONFIRM_TIMEOUT_SEC` (default 10 s). Candidate only.
- Output: `status=... source_id=guard:localizer evidence_kind=input|graph
  evidence_sequence=N`. The shell never evaluates this output as code.

The shell keeps the original 10 s supervision sleep and three-confirmation
default. Only distinct increasing metadata sequences (or graph sequences when
the publisher is also absent) accrue candidates. Guard JSON rewrites around
the same source sample cannot increment the failure streak. Source generation
changes and observer unavailability reset it. Replayed healthy evidence cannot
maintain the continuous healthy interval used to reset the restart budget.

After three candidates, the owner runs the existing bounded
`ros2 topic hz /flatscan --window 3`. Flowing data keeps the helper. Failed flow
confirmation still requires the original bounded `/scan` publisher check
before existing recovery can run. Startup topic/rate probes remain unchanged.

The legacy status env path and fields are retained with atomic replacement.
Added observer fields distinguish status-file activity from actual observation.
New health state `observer_unavailable` must NOT be interpreted as producer
failure by downstream consumers. Reader override: `NJRH_RUNTIME_FLATSCAN_CHECK_BIN`.
Shared snapshot path/age: `NJRH_RUNTIME_HEALTH_FILE`, `NJRH_RUNTIME_HEALTH_MAX_AGE_SEC`.

## Deployment and review

Byte-identical originals of the two existing edited files were captured before
editing under `reports/runtime_management_20260912/local_before/`, retaining
their full repository-relative paths. Derive deployment deltas against those
files, NOT HEAD; both originals already contained unrelated changes.

The two obsolete FlatScan log assertions in shared `test_workspace_contracts.py`
were updated with a two-line patch only. The earlier shared original backup was
not overwritten; the immediate pre-edit snapshot is additionally stored beside
it as `test_workspace_contracts.py.flatscan-pre-edit`. Other agents' API identity
test changes were not replaced or edited.

Parent acceptance still required:

1. Compile monitor/checker/global-localization in an isolated Humble environment;
   native snapshot tests must pass, not merely skip.
2. Isolated ROS domain: metadata source restart/exit, lazy subscription, callback
   stall with timer alive, replay, graph failure, duplicate metadata publishers,
   observer restart/exit. Verify no new full-stream subscriber or routine ROS CLI.
3. Verify startup/mapping/readiness consumers distinguish unknown observation
   from producer failure and preserve whole-chain ownership.
4. Synchronized deployment, full-chain restart only, then passive CPU measurement.
   No motion needed. End all probes/test processes afterward.

Direct child-exit recovery works even without metadata. A live but stalled
helper before any localization input subscriber exists cannot be diagnosed
from absent metadata alone; startup probes cover startup, and later missing
metadata remains explicitly unknown rather than authorizing a restart.

Startup behavior verified: `run_occupancy_grid_localization.sh` still accepts
only `FLATSCAN_HELPER_HEALTH_STATE=healthy` for its status-file fast path. UNKNOWN
uses the original one-off publisher verification with its default 20 s upper
bound (not a mandatory 20 s sleep). Isolated shell tests exercise successful and
failed verification, assert exactly one verification call for `/flatscan` from
`laser_scan_to_flatscan`, and confirm UNKNOWN stays UNKNOWN. Verification success
allows startup; failure remains a startup failure without isolated helper restart.
The startup reader and its publisher checks were not modified.

## Phase 3 file manifest

Modified existing files (deploy only our delta against preserved originals):

- `scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_pipeline.sh`
- `src/robot_global_localization/src/global_localization_node.cpp`
- `src/robot_system_tests/test/test_workspace_contracts.py` (two FlatScan assertions only)

New implementation and test files:

- `src/robot_bringup/include/robot_bringup/runtime_flatscan_monitor.hpp`
- `src/robot_bringup/include/robot_bringup/runtime_flatscan_snapshot.hpp`
- `src/robot_bringup/src/runtime_flatscan_monitor.cpp`
- `src/robot_bringup/src/runtime_flatscan_check.cpp`
- `src/robot_bringup/test/test_runtime_flatscan_snapshot.cpp`
- `src/robot_bringup/test/test_runtime_flatscan_management.py`
- `src/robot_bringup/test/isolated_flatscan_management_smoke.py`

New documentation files (include both in the parent deployment/document list):

- `docs/runtime_flatscan_management.md`
- `src/robot_bringup/test/flatscan/README.md`
