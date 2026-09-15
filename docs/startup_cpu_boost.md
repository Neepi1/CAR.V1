# Eight-core cold startup, five-core runtime

The full common-services owner creates a private CPU scheduling session only
for `navigation_5cpu`, normal EKF navigation, with FAST-LIO autostart disabled.
`NJRH_STARTUP_CPU_BOOST_ENABLED=false` disables this optional startup placement.
The navigation CPU profile and all `NJRH_CPUSET_*` steady masks remain unchanged.

During this session navigation launch wrappers may use CPU0–7. Exec-time
registration preserves each process's original steady target. Existing readiness
conditions, timeouts, launch order and algorithms are unchanged. An intermediate
`waiting_for_initialization` receipt does not end the session. Existing `ready`,
`reused`, or `waiting_for_localization` completion receipts restore steady
placement; common completion covers no-map/exited startup, and existing
EXIT/TERM cleanup covers failure and cancellation. Docking retains its last-start
order and normally starts after restoration.

Restoration covers surviving threads and unregistered descendants of owned
launchers, using the closest explicitly registered ancestor. Records contain
PID plus process start time, not process-name guesses. API processes form an
ownership boundary so mapping/arm business children are not swept into navigation
restoration. Explicitly registered navigation children receive their own placement.
Later launches read the closed session and start on their steady masks.
After restoration completes, short-lived launches do not append to the session
record or rewrite its JSON file; unfinished restoration still retains recovery records.

This is a scheduling scope, not a navigation lease or motion gate. Registration
and phase changes share a bounded file lock. Restoration reports errors without
adding navigation rejection criteria. No persistent polling process, ROS
subscription, cgroup, IRQ/RPS or mechanical-arm changes are introduced. Startup
can contend with other users of CPU5–7; no particular startup time is guaranteed.

Linux affinity calls address numeric TIDs. Start-time checks reject observed PID
reuse but cannot make a stat/syscall sequence kernel-atomic. SIGKILL cannot execute
a shell cleanup trap; full service teardown still relies on existing systemd/
container cleanup. This module does not control unrelated orphan processes.

## Validation

Run startup CPU core, shell-wiring, launch-prefix and navigation CPU profile pytest
suites, then common startup, AMCL and Nav2 regressions. Core tests use an OS adapter;
Linux-only validation creates an isolated multithreaded process, not robot nodes.
Deploy only changed files after comparing pre-edit hashes, replacing atomically.

Activation requires an authorized full `njrh-runtime.service` restart. Record
full restart T0, Nav2/AMCL readiness, session finish reason and thread masks
before/after restoration. Confirm steady navigation uses CPU0–4 with the prior
per-module allocation and late processes remain bound. No movement is needed.
Timing improvement and hardware activation remain unverified until that restart.
Reports belong under `/tmp/njrh_reports`.
