# Low-frequency C++ runtime health

## Scope

`robot_bringup/runtime_health_guard` replaces the resident Python health writer.
`robot_bringup/runtime_health_check` replaces the Python snapshot readers. The
reader is a plain C++ executable: it neither links rclcpp nor creates a DDS
participant. Shell remains the process owner and systemd remains the only
complete-runtime restart owner.

No change is made to wheel/local odometry production, EKF, TF ownership, Nav2,
collision monitoring, or the robot_safety watchdog. This observer never
publishes velocity or invokes a navigation/relocalization action.

The production wrapper additionally enables AMCL status maintenance and
FlatScan metadata observation in this same process. These use bounded IPC,
latest-message takes and the existing graph sampling, not an extra executor or
sensor stream subscription. See [management design](runtime_management_observer.md).

## Sampling and evidence

- One persistent BEST_EFFORT KEEP_LAST(1) local-odometry reader.
- One persistent reliable KEEP_LAST(1) docking-observation reader.
- No executor runs their callbacks. Each reader is explicitly taken once per
  `NJRH_RUNTIME_HEALTH_SAMPLE_PERIOD_SEC` (default 1.0).
- `message_count` now counts sampled messages, not the publisher's rate. The
  snapshot includes `message_count_semantics` to make this explicit.
- The graph is inspected every 5 seconds. Point clouds are never subscribed;
  extra scan/map/TF observation remains opt-in and disabled by default.
- Snapshots are written immediately after sampling, by atomic replacement of
  `/tmp/njrh_runtime_health.json`. Temporary files are removed on write errors;
  historical snapshots are not accumulated.

`NJRH_RUNTIME_HEALTH_ODOM_NO_UPDATE_TIMEOUT_SEC=3.0` is a runtime observation
grace period. Only increasing, non-future, sufficiently recent header stamps
refresh the steady-clock liveness timer. Repeated old stamps do not prove
recovery. The first sample has a bounded warmup window; an empty first snapshot
does not immediately prove a producer failure.

The previous `NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC=0.75` remains the separate
startup/direct-readiness sample-age check. It is not the steady-state guard's
fault deadline. Startup does not wait an additional 3 seconds after a good
sample appears.

Clock discontinuities invalidate the observer evidence and reset a bounded
sampling warmup. Snapshot validity uses a monotonic timestamp plus boot ID when
available, so an old file from another boot cannot authorize recovery. JSON
still includes wall/ROS times for diagnosis and compatibility.

Graph/serialization/write delays cannot turn an old sample into new fault
evidence: before atomic replacement, the writer rechecks the full iteration
budget and clock continuity. A delayed iteration discards its temporary file.
The reader also advances cached sample ages by snapshot age for startup/TF
freshness checks; it does not treat the cached age as a live age.

## Recovery contract

`runtime_health_local_state_diagnostic` distinguishes:

- 0: healthy runtime observation;
- 40-49: unavailable, stale, malformed, clock-invalid, graph-inconsistent or
  warming observer; never authorizes whole-runtime recovery;
- 50-59: producer-fault candidate requiring independent confirmation.

The common owner keeps its 5-second check period and three-consecutive-fault
budget. Each C++ snapshot has a generation and increasing sequence. Re-reading
the same fault evidence cannot increment that budget twice. Independent fresh
odometry contradicts the candidate and resets the budget even if the process
name check reports a missing process. A new writer generation resets the
budget, and missing/non-increasing evidence cannot increment it. A confirmed fault
may exit the owner so systemd restarts the full chain; no individual navigation
service restart is introduced. Three seconds is not a promise of restart latency.

Camera health remains diagnostic at the common-owner layer; docking_manager
retains its own motion admission rules. A missing observer must not be turned
into a camera fault.

## Validation and deployment

Compile `robot_bringup` in an isolated build prefix. Run pure C++ clock/stamp
policy tests, native snapshot-reader fixtures, the shell recovery regression,
and isolated-DDS tests with synthetic 50Hz odometry. Fault injection must not
stop or alter production EKF, TF or safety nodes.

Deploy the matching binaries and owner/helper scripts together, then restart
only through `sudo systemctl restart njrh-runtime.service`. Confirm selected
map identity, service readiness, one C++ guard, no legacy Python guard, and
fresh snapshots. Measure at least five minutes at steady state; the engineering
target is below 3% of one CPU core on average, not a guaranteed result.

All test processes must exit. Any remaining stage directory is a build artifact,
not a running monitor. Hardware motion is not part of this change's validation.

## Recorded acceptance

The 2026-09-12 deployment passed 168 CLI/shell regressions, the native policy
test, and isolated ROS dropout/replay/recovery tests. The production guard
averaged 1.15% of one core over 300 seconds, with 300 sampled odometry messages,
300 snapshots and 60/60 healthy checks. Whole-chain restart reached confirmed
ready on the retained B10/F1 map; no motion command was issued.
See [measurement and remaining limits](../reports/runtime_health_cpp_20260912/summary.md).
