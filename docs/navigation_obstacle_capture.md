# Navigation obstacle capture (recorder v3)

Run on the Jetson SSH host. This starts only one temporary observer in the
existing container; the operator owns the App navigation goal.

```bash
docker exec -it NJRH-car bash /workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/observe_navigation_failure_minimal.sh \
  --duration-sec 240 --sample-period-sec 0.5 \
  --label sidepass_v3 \
  --store-costmap-snapshots --costmap-snapshot-period-sec 0.5
```

Wait for `READY` after the six-second discovery warmup, then issue **one** normal
App navigation goal. Observe the existing supervised obstacle scenario. Once
the obstacle has moved away and the robot has either resumed or stayed stopped
for 15 seconds, press Ctrl+C once. Do not start docking or a second navigation
goal in the same capture. Ctrl+C stops only recording, not the navigation task.
The 240 seconds includes warmup. READY means the recording window is open,
not that every optional topic has a publisher or has delivered a message.
Read the preceding `EVIDENCE_LIMITS` line: absent collision-state messages and
uncollected MPPI internals are explicit limitations, not a robot motion gate.

Each run creates a randomized, unique directory under `/tmp/njrh_reports`.
The current NJRH-car bind-mounts host `/tmp`, so the printed report is available
on both the host and in the container. An explicit existing `--output-dir` is
rejected; past reports are never reused or overwritten.

## Evidence

- All seven command sources/boundaries: Nav2 raw, smoother, collision checked,
  safety mirror, final `/cmd_vel`, API fallback, and docking. Every command has
  receive time, monotonic elapsed time, latest chassis-mode feedback, and latest
  reverse/lateral permits plus receive ages.
- `/ranger_base/status` replaces the retired mode-controller topic.
- The shared UInt8 progress heartbeat records the actual numeric value: ordinary
  `0`, elevator tracking/replanning/wait/localization `1..4`, ordinary local
  replanning `5`, ordinary local wait `6`. Names follow the current project enum.
- `path_frames.jsonl.gz` stores changed reference/repaired geometry, including
  x/y/yaw and frame/header/receive times. High-rate transformed-path writes are
  capped at 2 Hz; sparse original/repaired path changes are kept. Snapshot
  metadata also stores the latest received path geometry.
- Exact 2 Hz OccupancyGrid payloads, rolling origin, odometry and the actual
  `/local_costmap/published_footprint` polygon allow path/obstacle reconstruction
  without guessing the chassis outline from a square statistics window.
- API status/navigation state and filtered ROS logs retain goal IDs, terminal
  results, phases, blockage/repair errors, and timestamps. HTTP is off the ROS
  callback thread, with only one timeout-bounded batch in flight.
- `runtime_snapshot.json` records exact executable process counts and the
  controller/smoother/collision startup parameter files and hashes. This is
  file evidence, not a live ROS parameter query; later dynamic parameter changes
  are not proven by this snapshot. No token or process environment is saved.
- `action_feedback.jsonl` stores full goal UUIDs and existing NavigateToPose /
  FollowPath feedback (at most 5 Hz per action; changed goals bypass throttling).
  NavigateToPose retains the original map-pose stamp and nearest buffered odom
  sample. Approximate alignment requires map/odom/base_link frames and a stamp
  difference no greater than 100 ms; the actual difference is saved. This is
  **not exact TF**. Odom pairing requires the costmap snapshot option above.
- `diagnostic_events.jsonl` preserves controller/collision/BT warning source
  timestamps, source file/function/line, latest commands with receive times,
  odometry, progress, and action feedback. Current command context means latest
  received, not necessarily simultaneous. Available CollisionMonitorState
  topics are discovered and recorded as change events; unknown numeric action
  types remain unknown. State age is last-event age, not heartbeat validity.
- `diagnostic_graph.jsonl` inventories a bounded set of existing publishers at
  READY and every 10 seconds. `evidence_availability.json` distinguishes no
  observed publisher, publisher without messages, and actually received state.
  Merely installing a message definition does not prove a publisher exists.
- `process_samples.jsonl` reads at most two controller PIDs from the initial
  snapshot, at most once per second. It records process CPU ticks (with tick
  units/start identity) and **main-thread** runqueue counters. It does not scan
  every thread, profile the optimizer, or infer MPPI duration from receive gaps.

## Interpretation limits

Missing permit/path samples mean **unobserved**, not permission=false or no
replanning. The repair-path publisher may be absent from an older controller
binary; use logs and received/transformed paths as additional evidence. No MPPI
candidate trajectories or critic scores are enabled by this recorder.
The checked Jetson Humble 1.1.19 collision-monitor header/library has no state
publisher despite the installed CollisionMonitorState message type. Installed
MPPI visualization exposes MarkerArray/path data, not validated rejection
reasons. V3 therefore inventories these interfaces but never enables or
subscribes to the large candidate MarkerArray. A script-only capture **cannot
fully reconstruct why MPPI rejected its candidates**. A later, separately
authorized controller diagnostic change would be needed for that evidence.

The legacy summary classifications are capture-wide hints and may contain old
action statuses: correlate timestamps and goal IDs before assigning a cause.
The `center_0_5m` and `center_1_0m` windows are map-axis squares, not the robot's
footprint. Legacy `lethal` counters combine OccupancyGrid values 99 and 100;
inspect exact grids to separate inscribed inflation from lethal obstacle cells.
Do not infer physical contact from one aggregate counter or mix map/odom frames.

## Validation and scope

The observer creates no command publisher, action client or ROS service client,
changes no parameters, clears no costmap and restarts nothing. It subscribes to
neither `/tf`, PointCloud2 nor LaserScan. SIGINT/SIGTERM request orderly capture
completion without destroying the ROS context in the middle of message decoding;
unexpected errors preserve a partial report and remain errors, not success.

Offline tests exercise actual callbacks, bounded graph discovery, source-stamp
retention, approximate pose pairing, and the entire Python entrypoint against
fake ROS. Both orderly interruption and callback failure must save reports and
close files; callback failure still exits nonzero. No live capture is launched
as part of this script-only change. Import-only checks on the Jetson validate
installed message types without initializing ROS. A separately operator-run
capture is needed for actual data; recorder tests do not validate navigation
or prove the reported smoothness issue fixed.

Deploy `navigation_observer_evidence.py` beside the shell recorder; both files
are required. No build or service restart is needed.
