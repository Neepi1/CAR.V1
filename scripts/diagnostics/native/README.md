# NAVLITE native raw-message helper

One temporary C++ ROS participant, one generic serialized subscription per selected
topic, one bounded queue and one existing rosbag2/sqlite3 writer. No publishers,
Action requests, movement/parameter services, pointcloud/image subscriptions or
production process management. The Python log recorder owns operator marks.

The recording path does not deserialize payloads. Header/frame, full Scan ranges,
paths, costmap and signed velocities are decoded from the original CDR offline.
This helper replaces, rather than runs alongside, the older rclpy main recorder.

## Standalone build (only this diagnostic)

Use the existing ROS environment; never build into a running production target:

```bash
source /opt/ros/humble/setup.bash
cmake -S scripts/diagnostics/native -B /tmp/njrh_reports/<new_candidate>/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/njrh_reports/<new_candidate>/build -j1
```

For existing project-specific message types, source the actual runtime workspace
before recording so their installed type-support libraries can be resolved.

Optional offline trajectory integrator (does not initialize ROS):

```bash
cmake -S scripts/diagnostics/native -B /tmp/njrh_reports/<new_candidate>/build \
  -DNAVLITE_BUILD_MPPI_REPLAY=ON \
  -DNAVLITE_NAV_CONFIG_DIR=/absolute/candidate/src/robot_nav_config
cmake --build /tmp/njrh_reports/<new_candidate>/build --target navlite_mppi_replay -j1
```

## CLI and manifest contract

```text
navlite_raw_capture --output NEW_OR_EMPTY_DIRECTORY --manifest JSON_FILE
                    --duration 300 --disk-budget-mb 512 --queue-mb 16
                    [--discovery-sec 3] [--min-free-mb 128]
```

Keep the manifest **outside** the new output directory. Existing evidence is never
overwritten. Exit 0 means recording complete by the helper's quality checks,
3 means retained partial/incomplete evidence, 2 means startup/fatal failure.

```json
{"schema":1,"topics":[
  {"name":"/scan","role":"collision_scan","critical":true,
   "cadence":"continuous","expected_type":"sensor_msgs/msg/LaserScan"},
  {"name":"/cmd_vel","role":"chassis_input","critical":true,"cadence":"task"},
  {"name":"/tf_static","role":"static_tf","critical":true,"cadence":"latched"}
]}
```

- Explicit selection only, maximum 64 unique absolute topics. These example names
  are not a live mapping assertion: the caller must audit current remaps.
- Actual graph type must match `expected_type` when provided. Missing/ambiguous
  types are recorded, not guessed. An explicit expected type permits subscribing
  while a publisher is still undiscovered.
- Optional `roles`, `binding_origin`, and `endpoint_expected.publisher/subscriber`
  are preserved. Compare expected vs observed endpoints offline; no vehicle gate.
- Optional diagnostic `qos` accepts `reliability`, `durability`, `depth`; it changes
  only this subscriber. By default volatile streams use best_effort; retained
  reliable publishers use reliable/transient_local. `/tf_static` retains this
  policy even if initially undiscovered. This was validated with a late subscriber.
- Only `continuous` topics get the diagnostic two-second stale check. `task`,
  `event`, `latched` do not get false gap alarms after a task stops publishing.
  Missing critical first messages still show INCOMPLETE rather than fake READY.

## Evidence

- `bag/`: existing SQLite3 rosbag, original CDR payloads.
- `receive_index.csv`: receive `seq`, topic/ordinal, unique `bag_timestamp_ns`,
  callback-entry steady and wall times, recorder ROS-clock time, per-message GID,
  RMW source/receive timestamps, payload size, available RMW sequence numbers.
- `topic_map.json`: observed types, endpoints/GIDs/QoS, requested subscription QoS,
  selected roles and missing items. Changed endpoint snapshots are also retained
  in `health.jsonl` (checked every five seconds for selected topics only).
- `quality.json`: atomically replaced once per second and at close;
  `ready`, `incomplete`, `incomplete_reasons`, per-topic message/written/drop/gap
  counts, queue drops, CPU/RSS and final reason. Stdout only prints quality changes.
- `health.jsonl`: at most one health sample/second, plus graph and clock changes.

Join bag and index by `(topic, bag_timestamp_ns)`. Bag timestamp is a unique
wall-based storage key `max(previous+1, receive_wall)`; it is **not** a source stamp.
Missing unsupported MessageInfo values are empty/null, not fabricated zero. Source
clock synchronization remains unverified. ROS receive clock is the recorder node's
default system-backed ROS clock, not proof of another node's use_sim_time setting.

The 16 MiB queue default has a second 8192-message cap; overflow records counts and
first/last dropped capture sequence, never silently discards evidence. SQLite's
additional internal message cache is disabled. Disk budget/free space are sampled
at 1 Hz with queue+1 MiB drain reserve, not a filesystem hard quota. Writer failure
marks incomplete and retains partial bag/index for offline consistency checks.

SIGINT/SIGTERM stop receiving and drain the finite queue, close the bag and record
the interruption. No navigation cancel or speed/TF publish occurs. Kernel-level
indefinitely blocked disk I/O or uncatchable process kill cannot guarantee clean
closure; the supervising recorder must preserve the partial evidence.

## Tests

```bash
source /opt/ros/humble/setup.bash
bash scripts/diagnostics/native/run_isolated.sh \
  /tmp/njrh_reports/<candidate>/build/navlite_raw_capture \
  /tmp/njrh_reports/<new_test_directory> --perf-seconds 60
```

The wrapper requires unshare/mount permission and creates private network/IPC/mount
namespaces, private `/dev/shm`, loopback only, domain 181 and a test-only Fast DDS
profile. It never joins the real robot's DDS domain, never uses CAN, and installs
nothing. Test code refuses to run outside that environment.

Tests exercise real mock publishers -> native subscriptions -> original bag/CDR
and receive index; both odoms, TF, Scan, four command stages, 200x200 map, paths and
footprint compete continuously. They cover intermediate zero commands, GID for two
publishers, repeated source header stamps, late retained TF, QoS mismatch, missing
topics, a gap after readiness that later recovers, Ctrl+C, disk reserve/budget and
queue overflow. Test markers are checked after initial discovery, not only at start.

An initial red run caught lost historical `/tf_static` when using best-effort with
a reliable retained publisher. Matching reliable/transient_local fixed that actual
installed-library test; this is not a guessed QoS workaround applied to production.

This is not zero overhead. Performance numbers apply only to the stated fixture,
not to all real workloads. Complete real-car motion capture remains unverified
until the user elects to record. Published maps still do not prove the controller's
internal per-cycle map, and wheel odom still does not prove fresh CAN samples.

### 2026-09-20 candidate validation

Candidate-only directory (not deployed):
`/tmp/njrh_reports/navlite_native_20260920_aP5s8a`.

- Independent C++17 Release build succeeded with `-j1` using installed Humble.
- `green4.log`: **9/9** isolated CLI tests passed in 95.836 seconds, including a
  60-second continuous recording. The mock publisher process is not included in
  recorder CPU measurements.
- Normal recording: **23,365 received = 23,365 indexed/written**, queue drops 0;
  queue peak 47,756 bytes / 11 messages. Four command stages approximately 3001
  each, both odoms 3001 each, TF 3000, Scan 857 (1440 ranges each), 200x200 map 500,
  path/footprint 500 each, startup-only retained TF 1. Evidence size 37,490,529 bytes.
- Recorder: CPU 6.43025 s / elapsed 61.1551 s = **10.5147% of one CPU core**;
  max RSS 60,024 KiB. This includes its discovery/open/close, not the Python log
  reader or producer diagnostic overhead; not a like-for-like speedup claim.
- Source SHA256 `656729191c20d59be4547d5e79b98211d3bfda6ae4fe739d15e955e0b043a6c3`;
  binary SHA256 `c78a519f0e97954ba22869d193eadb5eb062ca0521ddc513e52f1c88f941091a`.
  Local source and temporary Jetson candidate were hash-matched.
- Ctrl+C fixture originally used fixed sleep and once interrupted before DDS
  delivered data. It now waits for >20 written samples before testing stop during
  recording; it does not relax the recorder's readiness or message assertions.
- Still unverified in this helper: forced physical ENOSPC midway through SQLite
  write, kernel I/O permanently blocked, SIGKILL/power loss, an actual system-clock
  jump during native recording, real-car full selected-topic load. Repeated ROS
  source header stamps and multi-publisher GIDs were tested; they are not a
  substitute for a system-clock-jump integration test.
