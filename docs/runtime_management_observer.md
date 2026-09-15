# Runtime management overhead

## Scope

Management checks must not compete with navigation by repeatedly spawning ROS
CLI participants or walking /proc through hundreds of shell subprocesses.
The runtime control, localization, TF and safety algorithms are unchanged.

1. `runtime_process_check` reads /proc natively once per existing common-service
   audit. It verifies executable identity and exact NUL-delimited supervisor
   arguments together. Missing/duplicate owners are distinct from unreadable
   or racing observations; an observer failure cannot authorize a restart.
2. The existing C++ `runtime_health_guard` owns the public AMCL status file.
   Startup submits process/map/owner-scoped evidence through a bounded native
   local socket. Lifecycle events, actual graph counts and pose samples update
   status without a separate shell heartbeat or DDS participant. Static standby
   remains permitted; graph presence alone is not proof of lifecycle activation.
3. The existing global-localization FlatScan receiver emits tiny input metadata.
   The guard samples that metadata at 1Hz and reuses its 5-second graph query.
   `runtime_flatscan_check` reads the shared snapshot without ROS. Cold-start
   validation and bounded independent fault confirmation remain in the helper.

The production wrapper enables these integrations through
`NJRH_RUNTIME_MANAGEMENT_ENABLED=true`. Snapshot files are atomically replaced;
they are not a growing history. Missing/stale observer evidence is UNKNOWN,
not a successful observation and not permission to restart a producer.
See [FlatScan details](runtime_flatscan_management.md) and
[runtime health](runtime_health_cpp.md).

## Ownership and validation

Process lifetime, lifecycle activation and mission status remain separate.
systemd is the sole complete-chain restart owner. Deployment requires matched
binaries and scripts, followed by `systemctl restart njrh-runtime.service`.
Do not restart individual navigation services to apply this change.

For the existing default that does not require AMCL tracking before runtime
ready, finish the live bridge/context proof before launching AMCL background
readiness. This separates competing temporary DDS clients. The strict AMCL
startup option and all bridge sequence/owner/TF-age gates remain unchanged.
The 2026-09-12 stationary deployment reproduced a cold TF observer failure
while an established observer still saw 10-20ms-old map->odom messages. This
does not establish a deadlock or justify relaxing the TF freshness limit.

Stage only the task delta against the deployed sources; unrelated local and
remote changes must survive. Compile and fault-test in an isolated prefix and
DDS domain. Production fault injection and robot movement are not required.
Compare at least 300 seconds before/after, counting shell child CPU as well as
parent CPU. A combined management budget below 5% of one CPU core is a target,
not an assumed result. Full-machine utilization is affected by other workloads.

Required checks include observer death/staleness, PID reuse, duplicate owners,
AMCL mode/static-standby transitions, graph loss, FlatScan replay/non-advancing
stamps, startup readiness and cleanup of every temporary test process.
On-hardware navigation, floor switching and mapping transitions remain separate
field validation; no motion is commanded during this management deployment.

Implementation and measured acceptance are recorded in
`reports/runtime_management_20260912/summary.md`.

Stationary acceptance completed: management CPU fell from 54.892% to 4.703%
of one core over separate 300-second windows, including reaped child CPU.
The full runtime and AMCL were ready and all temporary probes exited. This
is not a total-navigation CPU figure or moving-robot acceptance. The successful
start still took about 125 seconds for runtime context and 297 seconds for
AMCL readiness; reducing those cold-start waits remains separate work.
