# NAVLITE evidence candidate — 2026-09-20

Scope: recording/diagnostic code only. Not deployed or restarted. No navigation
goal, velocity, TF, recovery call, parameter change or robot motion was issued.
The old log-only mode remains available. Existing dirty workspace changes are
not the baseline for claiming deployed binary equivalence.

## Runtime binding check

Read-only `/proc`/launch-file inspection on the current Jetson verified:

| Role | Topic |
| --- | --- |
| Controller / smoother input | `/cmd_vel_nav_raw` |
| Smoother output / collision input | `/cmd_vel_nav` |
| Collision output / safety input | `/cmd_vel_collision_checked` |
| Safety output / chassis input | `/cmd_vel` |
| Controller odometry | `/local_state/odometry` |
| Wheel odometry | `/wheel/odom` |
| Collision scan | `/scan` |

Evidence: `/tmp/njrh_reports/navlite_parent_9Fv9T5/actual_bindings_final/` (no binding gaps).
This was not a production ROS recording or a graph probe. Graph/type/QoS and
actual received coverage are checked by the native helper on each recording.
The current launched Nav2 file was `/tmp/launch_params_qaq7lroq`.

## Tests already executed

| Layer | Test/result | Evidence |
| --- | --- | --- |
| Existing log CLI | 21/21 Linux fake-log tests pass | `navlite_parent_9Fv9T5/delivery_tests.log` |
| Parent integration | 10/10 stdlib tests pass | same combined log |
| Raw collector | 9/9 isolated native ROS/CDR tests pass | `navlite_native_20260920_aP5s8a/` |
| Offline report | 19/19 CDR/index/window/early-failure tests pass | same combined log (50 total in 30.118 s) |
| Internal writer | 5 scenario groups pass, including a new session after prior overflow | `navlite_snapshot_20260920/transport_final.log` |
| Real MPPI | diagnostic off/on exact-command, original-error and model-replay checks pass | `navlite_snapshot_20260920/integration_final.log` |

Native tests use private network/IPC/mount namespaces and private `/dev/shm`,
localhost-only domain 181. They use mock publishers, not the actual chassis or
navigation tasks. They cover late-join transient-local TF, best-effort/reliable,
missing topics, incompatible QoS, multiple publishers, repeated source stamps,
disk budget, queue overflow, Ctrl+C and lossless CDR/index association.

The late-join static-TF test initially failed and exposed a real capture gap:
on the installed Fast DDS, a best-effort late joiner did not obtain the reliable
retained frame. The helper now requests reliable/transient-local when all offered
publishers support it. Production QoS is not changed.

### Measured native recorder cost (isolated, not real navigation)

60-second publisher workload plus discovery/close: 61.1551 s elapsed,
6.43025 CPU seconds = **10.5147% of one core**, max RSS **60024 KiB**.
23,365 received and 23,365 written; recorder queue drops 0; peak queue 47,756 bytes
/ 11 messages. Output 37.49 MB. Each velocity and odometry stream was about 50 Hz,
Scan about 15 Hz, and the 200×200 costmap about 8 Hz, with TF, path and footprint.
This excludes mock-publisher load and is not a guarantee of production CPU cost,
zero DDS/source loss, or full vehicle acceptance.

Offline decoding of a real native mock bag produced 3,128 decoded/indexed messages
with no CDR or index-association errors. Its 8-second duration covers the tested
central ±2-second window but deliberately **fails** the required ±5-second window.
It also reports missing MPPI snapshots instead of calling the raw bag full evidence.
Evidence: `/tmp/njrh_reports/navlite_offline_real_hdwr1cyb/`.

### Real algorithm and producer checks

With the same input and fixed optimizer seed, five calls to the actual optimizer
return exactly identical six-axis command arrays with diagnostic snapshots off/on.
The actual native obstacle critic/fallback's all-lethal-map error is unchanged.
The true controller's empty-path `transformPath` exception retains its type and
original `Received plan with zero length` text with diagnostics off/on.
Final pre-shift selector output is checked against the captured returned command;
offline predicted velocity and installed native integration results match point by
point (bitwise equality in the isolated fixture).

The real controller hook generated five frames: three successful calculations,
one optimizer failure and one failure before map processing. The last is correctly
reported as an original computation error with **no available map or trajectory**,
not invented geometry and not a corrupt transport frame. No diagnostic drops or
write errors. Schema check: `/tmp/njrh_reports/navlite_premap_report_y_4fdwr1/report`.
Offline replay of these files produced 3 trajectories, 2 expected no-return cases,
0 invalid skips: `/tmp/njrh_reports/navlite_parent_9Fv9T5/replay_final/`.

Final transport-only synthetic 40KB/15Hz test: 0.667717% of one CPU core;
producer copy p50 12.256µs / max 17.6µs, preallocated queue 12,952,064 bytes,
no drops. This is not real whole-controller CPU/deadline acceptance.

An existing visualization-symbol allocator ABI difference in the deployed build
prevents strictly linking the standalone integration executable's unused visualizer
branch. Its **test-only** visualization substitute throws if invoked; production
candidate contains no substitute. The tested live configuration has visualization
disabled. This test does not cover `visualize=true`, and no system library was changed.

Only the MPPI translation unit was compiled/linked into a temporary candidate;
there was no API rebuild. Runtime controller PID 1581383 and its deployed MPPI
library remain unchanged, SHA256
`0771f1e653314228dc520539259f7a6d950f6a8973eb6d17c69a3eef4bb05314`.
The diagnostic candidate is
`/tmp/njrh_reports/navlite_snapshot_20260920/libranger_dynamics_mppi_controller.so`,
SHA256 `60329a47863795b8249c2c564cb69bead5a2a9697457bb4bdea227af891b5801`.
Its delta against the audited deployed source is `mppi_incremental.patch` in that
directory, not a diff against an assumed clean main branch.

Repeat tests using [the isolated snapshot entry point](run_navlite_snapshot_tests.sh)
and [native CLI tests](native/README.md). Test assertions explicitly remain enabled
in Release builds. Dependencies are the installed Humble libraries; nothing is installed.

## Acceptance boundaries

- No new production binary is loaded yet; real simultaneous MPPI/collision/chassis
  capture and the real robot's CPU/deadline impact remain unverified.
- Diagnostic tests do not prove that obstacle avoidance is fixed. No navigation,
  safety, geometry, weights, sample counts or retry behavior was changed.
- A raw scan and TF can be compared with the exact MPPI grid, but they do not
  identify collision_monitor's internally consumed scan/TF cache with certainty.
- Arrival time is not source time or proof of internal latency. Original stamps,
  raw CDR, receive monotonic time and unique bag index remain distinct.
- `EVIDENCE_READY` is an observation status. Final completeness requires per-mark
  window/index/closed-snapshot checks. It never gates vehicle behavior.
- Physical ENOSPC during SQLite write, SIGKILL/power loss and native recording
  during an actual system clock jump have not been validated; partial-file checks
  do not stand in for those tests.
