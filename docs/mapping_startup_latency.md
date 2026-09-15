# Mapping startup: warm discovery and non-blocking status

2026-09-14 scope: startup orchestration and mapping-process status locking only.
FAST-LIO math, four OpenMP threads, CPU1–4 eligibility, sensor/DDS settings,
navigation parameters and canonical TF ownership are unchanged.

## Evidence before the change

`/tmp/njrh_reports/mapping_startup_20260914T1304Z` contains the captured logs.
The 13:02:25 UTC mapping start took 51 seconds to finish original-stamp
scan/TF proof: preflight 18 s, FAST-LIO ready 28 s, odom bridge 31 s,
resident scan release 37 s, mapping scan owner 48 s, stamped TF 51 s.
FAST-LIO logged initialization complete at 13:02:45.528 UTC and initial kdtree
at 13:02:47.295 UTC. Not all 51 seconds were spent initializing the algorithm.

The resident log showed scan enabled/registered at 13:01:47.164 UTC, and
enabled again at 13:02:34.658 without an intervening disable. A cold 1-second
owner client timed out; a second cold client immediately accepted count=0.
That empty discovery cache incorrectly triggered redundant restoration.

The previous stop HTTP request took 31,098 ms; a simultaneous status GET took
30,421 ms. Shutdown and snapshots held the same mutex. The logs do not identify
the exact child responsible for the graceful-stop wait; its 30-second allowance
is retained.

## Implementation

- `mapping-preflight` optionally restores resident scan in its existing DDS
  participant while checking the existing TF, scan and local-odom conditions.
  Exact publisher count is checked here, without another cold client.
- `ScanHandoff` reuses the existing low-rate resident status and SetBool service.
  Cold graph zero is unknown. Enable is requested only after fresh, correctly
  sourced status proves disabled and no other owner is discovered. Already
  enabled scan waits for discovery; it is not restarted. Release observes
  acknowledgement and disappearance on the same graph. Actual unique ownership
  also proves enable success if the RPC reply arrives late.
- FAST-LIO's private odom bridge starts immediately and naturally waits for
  input. The existing cloud-freshness check remains. One small-message client
  then checks the cloud/odom graph pair and raw/bridged odometry; it never
  subscribes to PointCloud2. UNKNOWN names cannot prove a source pair.
- Pre-armed `mapping-scan-ready` checks unique owner, scan freshness and three
  consecutive transformable ORIGINAL scan stamps together. The common budget
  is the maximum of existing owner/scan/TF budgets (default 30 s). There is no
  fixed sleep, relay or restamping.
- Separate operation and state mutexes keep start/stop serialized while status
  reads remain responsive. Snapshots do not concurrently reap or rediscover a
  stopping process. An empty escalation set no longer sleeps another 800 ms.

Legacy probe subcommands remain available to running scripts and non-mapping
callers. The shared shell restore helper also uses the native handoff.

## Verification and activation

Reports/candidates: `/tmp/njrh_reports/mapping_startup_fix_20260914`.

- `test_mapping_scan_restore.py` replays cold discovery through the real Bash
  helper; it failed before and passed after the fix.
- `test_mapping_stop_concurrency.cpp` links the real runtime with process-OS
  fakes. Status must return during stop, but a concurrent start must wait.
  The fixture never enumerates or signals production PIDs.
- `isolated_mapping_startup_smoke.py` exercises native DDS behavior with
  synthetic inputs: late discovery/reply, release/rejection, duplicate owners,
  stale scan, stamp/TF mismatch, source-pair mismatch and composite preflight.
  Run via `unshare --net --ipc` with loopback only, ROS_DOMAIN_ID=226 and
  ROS_LOCALHOST_ONLY=1. It publishes no pointcloud data or motion commands.
- The API candidate starts from a byte-identical deployed link baseline;
  both the private MappingModule implementation and process runtime are rebuilt.
  Other pending source changes are excluded. HTTP construction/status smoke
  additionally runs in private PID and mount namespaces.

Scripts/probe apply on the next mapping invocation. The API binary must remain
staged until an authorized whole-runtime restart: replacing a live executable
previously caused a supervisor `(deleted)` path check to trigger an unrequested
restart. Do not replace it during active mapping.

Hardware acceptance remains: user-operated next start/stop, comparison of the
same `MAPPING_STARTUP_STAGE` boundaries, responsive status during stop, and
canonical scan restoration. Synthetic tests do not establish actual startup
time or moving-map quality. Restart/movement need separate authorization.
