# Phase S4 Boot-to-Navigation Critical Path

## Problem

The same resident navigation path previously reached `nav2_layer_ready` in 46
seconds but later required 82 seconds. The dominant regression was not Nav2
compute time. A transient Fast DDS graph miss on `/motion_state` caused the
localization child to kill and restart the common-owned Ranger process. The
same child also had authority to restart the pointcloud profile after a
`/flatscan` graph miss. Repeated environment setup, repeated floor-asset
parsing, cumulative log scans, and per-frame Hesai stdout added variable I/O.

## Runtime Contract

- `run_common_services.sh` exclusively owns Ranger, static sensor TF, and the
  pointcloud pipeline.
- Localization checks those dependencies but never starts, kills, or restarts
  them.
- A missing common dependency fails resident startup. Recovery belongs to the
  systemd owner through a complete `njrh-runtime.service` restart.
- `common_env.sh` is idempotent only inside one shell. Child processes still
  perform one complete environment setup.
- Localization first consumes the common generation's long-lived runtime
  health snapshot for Ranger-to-local-state admission. It creates a bounded
  direct `/wheel/odom` probe only when that snapshot is unavailable.
- Ranger cold-start admission uses one DDS participant while preserving all
  four checks: `/wheel/odom` owner, fresh wheel-odom stamp, `/motion_state`
  owner, and an actual motion-state sample. The IMU-bias gate likewise uses one
  participant for corrected-IMU and bias publisher/message checks.
- The selected floor is resolved once in common services. Matching child
  layers reuse the exported paths and map identity.
- Nav2 lifecycle readiness is an atomic status record bound to a live owner
  PID and timestamp, with direct lifecycle checks retained as fallback.
- Lifecycle transition response loss is separated from node convergence. A
  missing `ChangeState` response falls back to `GetState` after 5 seconds,
  while the existing 60-second per-node convergence budget remains available
  for a node that is genuinely configuring or activating.
- A pre-trigger Isaac result remains rejected, but it no longer ends the
  startup sequence ambiguously. Isaac emits one result per explicit service
  call, so a bridge-confirmed pre-trigger or over-age result returns
  `FRESH_LOCALIZATION_RETRY_REQUIRED` immediately and the bounded outer loop
  issues a new trigger. No result-age or pre-trigger gate is relaxed.
- The normal trigger order is input-health check, bridge force-accept arm,
  first post-arm `/flatscan`, trigger baseline capture, then Isaac service
  call. This removes false pre-arm rejects caused by bridge service latency
  while retaining the existing one-second pose-stamp slack.
- The wrapper performs the same 115-degree minimum-FOV admission reported by
  Isaac before triggering. The post-arm gate requires two consecutive good,
  uniquely sequenced FlatScan samples. A narrow transient resets that count and
  is skipped within the one-second input window instead of consuming a
  20-second result timeout.
- The global-localization wrapper belongs to the current occupancy-localization
  generation. It is never disowned or reused from an older generation, and
  systemd cleanup also removes any legacy orphan before a complete restart.
- Runtime logs retain at most 32 MiB from the previous owner run by default.
- The systemd owner checks the container process table before entering the
  expensive in-container cleanup sweep. A clean cold start or the start half
  of a completed systemd restart only clears status files; it does not repeat
  the process sweep already completed by `ExecStop`.
- Container user readiness, NITROS `/tmp` preparation, and GS2 device repair
  are consolidated into one retrying `docker exec` when the container already
  exists.
- Host systemd startup uses non-login `bash -c` inside the already-running
  container. It does not repeat `/etc/profile` and workspace setup through a
  nested login shell before common services begin.
- The systemd cleanup set includes the pointcloud supervisor and FlatScan
  helper, so common startup does not inherit them and wait through a second
  signal escalation sequence.
- FAST-LIO stale-process isolation starts from matching PIDs instead of reading
  every `/proc/*/cmdline` entry. Mapping-owned private FAST-LIO processes retain
  the same environment-based exclusion.
- Resident-before-local-state overlap remains disabled. A field A/B that loaded
  MapServer/Isaac concurrently with local state produced fresh odometry but
  prevented both a new TF probe and the long-lived health guard from observing
  `odom -> base_link` under cold DDS load. The bounded final recheck remains in
  place for local-state generations that finish just after the first window.
- The local-state final recheck now repeats the same bounded process/endpoint
  readiness contract. Previously it performed an instantaneous process test,
  so a generation completing just after the first 12-second window caused a
  full service restart even though a second 12-second budget was configured.
- Child wrappers in one systemd generation inherit the already-resolved
  ROS/DDS/profile environment. They still source `common_env.sh` to define its
  shell functions, but skip repeated interface discovery, profile reads, ROS
  setup, and directory preparation. Driver entry points that explicitly reset
  their environment also clear the generation marker and perform the full
  setup.
- Normal localization-stack readiness uses one participant for the wrapper
  trigger service, Isaac grid-search service, selected map, and supervised
  FlatScan owner. Global-costmap readiness similarly combines lifecycle-active
  and costmap-publisher checks without relaxing either condition.
- Held Nav2 preload waits until Isaac exposes
  `/trigger_grid_search_localization` and the bridge status reports
  `has_odom=true`. This gives the bridge time to consume canonical odom before
  Nav2 DDS load begins. Nav2 still preloads before the first localization result
  and remains lifecycle-inactive until the bridge-owned `map -> odom` baseline
  is accepted.

## Preserved Behavior

This phase does not change JT128 pointcloud content, QoS, DDS middleware,
timestamps, FAST-LIO2 logic, localization gates, Nav2 plugins, Ranger motion
parameters, TF ownership, or the command chain.

## Measured Result

Three complete restarts of the same final build were measured with Docker
already available:

| Run | systemd start to ready | resident navigation |
|---|---:|---:|
| 1 | 66.055 s | 43 s |
| 2 | 59.321 s | 43 s |
| 3 | 69.676 s | 52 s |
| Median | 66.055 s | 43 s |

The previous stable observation was about 87.7 seconds full-chain and 56
seconds resident. These three runs confirm the regression is removed and all
full-chain samples are below 70 seconds. They are an engineering confirmation,
not the formal five-run/P95 sample required below.

## Validation

Use only the complete service owner:

```bash
sudo systemctl restart njrh-runtime.service
```

For five complete restarts, verify:

1. No localization log contains `existing ranger_chassis_localization process
   will be restarted` or a pointcloud profile restart.
2. Exactly one `ranger_base_node`, pointcloud supervisor, and
   `laser_scan_to_flatscan` process exist. Exactly one
   `global_localization_node` exists and its start time is newer than the
   current occupancy-localization owner.
3. `flatscan_helper_status.env` is fresh and healthy.
4. `STARTUP_STAGE` reaches `localization_stack_ready` without a chassis restart
   and reaches `nav2_layer_ready` with a median resident time at or below 50
   seconds and P95 at or below 60 seconds.
5. The full systemd start-to-ready median is at or below 70 seconds after the
   Docker service is already available. Cold boot timing must be reported
   separately because Docker startup is outside the resident timer.
