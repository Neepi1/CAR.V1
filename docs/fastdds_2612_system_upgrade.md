# Fast DDS 2.6.12 system upgrade

## Scope and evidence

The 2026-09-11 bridge captures proved a circular wait between the Fast DDS
StatefulWriter reader-removal path and the flow-controller send path. The
specific temporary AMCL TF reader is a candidate trigger, not a proven GUID
match. eProsima's official [PR 6463](https://github.com/eProsima/Fast-DDS/pull/6463)
fixes this lock-order inversion and is included in
[v2.6.12](https://github.com/eProsima/Fast-DDS/releases/tag/v2.6.12).

The user selected a **system package replacement for the entire NJRH-car ROS
runtime**, not a bridge-only override, second install prefix, or global OS
upgrade. ROS remains Humble, RMW remains rmw_fastrtps_cpp, and Fast CDR remains
the installed 1.0.24. Preserve SECURITY, TLS, SQLite, statistics and SHM build
support. Do not change transport profiles, QoS, timestamps, TF ownership,
CPU placement, navigation/mapping policies, or arm-black-box source.

The local package is built from the unmodified official release archive; it is
not an official ROS build-farm binary. The installed package name remains
`ros-humble-fastrtps`, version `2.6.12-1njrh20260911`, so dpkg replaces the old
headers, tools and library and removes its superseded 2.6.10 file. No
LD_LIBRARY_PATH override or dual-version selector is introduced. Test build
artifacts and recovery archives below `/tmp/njrh_reports` are not runtime
installations.

## Build and validation

`scripts/jetson/fastdds_2612/build_system_package.sh` accepts an offline official
archive and an existing report directory. It verifies the archive SHA256,
builds on CPU0,1,4 at reduced priority with two jobs, then generates a staged
Debian package. It **does not install or restart anything**.

Archive URL:
`https://codeload.github.com/eProsima/Fast-DDS/tar.gz/refs/tags/v2.6.12`

Archive SHA256:
`53476bcee331f28fe83e8387122fb7f84ab39209c459bd2cc227c71e4bd21c9b`

The test CMake target compiles the official unmodified
`BlackboxTestsPubSubFlowControllers.cpp` with a minimal UDP-only test main.
`run_isolated_regression.sh` enforces a private network, IPC and /dev/shm,
bounded execution and cleanup. It selects
`AsyncPubSubReaderRemovalWhileDeliveringDoesNotDeadlock`: each repeat removes
20 readers while a reliable asynchronous writer is sending. This checks the
actual fault path, not merely successful process construction. A passing
finite stress run is not proof that all possible DDS deadlocks are eliminated.

`run_ros_transport_smoke.sh` runs 20 rounds of reliable topic reception,
transient-local late joining, service calls and reader removal, with normal
writer shutdown. Run both UDPv4-only and SHM-only cases in private namespaces.
`run_bridge_smoke.sh` checks the installed bridge with the package's existing
correction-pause service test, in another private namespace with TF publication
and AMCL input disabled. Neither test can reach the production DDS domain.

Before installation, validate library dependencies/exported symbols, unchanged
Humble RMW loading, small ROS topic/service exchanges and reader teardown in
isolation. Preserve package metadata for incident recovery. Install with dpkg
only after those checks pass, then use only the separately authorized full
`njrh-runtime.service` restart. No navigation goal, localization trigger,
single-node restart or arm command is permitted by this procedure.

`audit_loaded_libraries.py` reads /proc without creating a DDS participant or
exposing command-line arguments. After restart, verify every navigation DDS
process loads 2.6.12, no old/deleted mappings remain in that runtime, critical
processes are unique, and existing health/status logs continue advancing.
Record external/arm-owned processes separately; do not restart them or alter
their private dependencies.

## Acceptance status

System replacement completed on 2026-09-11 UTC; **whole-navigation startup
acceptance failed**. Do not describe this deployment as a healthy navigation
recovery.

Report directory on Jetson: `/tmp/njrh_reports/fastdds_2612_MRgCrsY6`.

- Installed package: `ros-humble-fastrtps 2.6.12-1njrh20260911`.
- Debian SHA256: `423c8369fb206d6aadead5b4ac66b17f91b9cbd5a22f47f047c6d6d71016bc2a`.
- Installed library SHA256: `c7efcac6a6b3429f5bdb38d84104c12879a1e7d4cac6f67e0c4e39149235176f`.
- Source `tar --compare` passed. Library SONAME and NEEDED dependencies match
  the old package. Fast CDR and both RMW library hashes are unchanged.
- 17 old exported symbols are absent (internal TCP signatures, destructor
  adjustment thunks and RTTI). None is referenced by the 502 loaded ELF objects
  scanned across 42 pre-upgrade container DDS processes. This is scoped binary
  compatibility evidence, not a guarantee for every possible external plugin.
- Unmodified official deadlock regression: old 2.6.10 hung in the first repeat
  and hit the 150-second timeout; 2.6.12 passed 20 repeats / 400 reader removals.
- UDPv4-only and SHM-only ROS topic/service tests each passed 20 reader rounds
  with normal writer shutdown. Installed bridge isolated service test passed.
- The old `.so.2.6.10` system file was removed by dpkg. Installed package verify
  passed. The container's doc-exclusion rule initially omitted `njrh-build.txt`;
  that exact package-owned provenance file was restored from the verified stage.
- The old ROS CLI daemon was stopped normally. A Domain-173 unit-test process
  left from September 7 was verified and terminated with SIGTERM; no navigation
  node was individually restarted. Later audit found no old/deleted DDS mapping
  in NJRH-car. Other containers and host-owned dependencies were not changed.

Authorized restart timeline (UTC):

| Time | Actual event |
| --- | --- |
| 17:57:05 | Full service restart requested |
| 17:57:12 | New service main process started |
| 17:58:28 | Bridge accepted Isaac result, explicit sequence 1; wrapper reported map-to-odom age 23.231 ms |
| 17:58:58 | Nav2 lifecycle ready (113 seconds after restart request) |
| 17:59:43 | Runtime context failed after its fresh map-to-odom TF check timed out |
| 18:00:52 | Bridge received SIGINT during the existing startup-failure cleanup |

The resident startup log says the TF observer had no `map` frame, then explicitly
tears down the incomplete runtime. That establishes the failure/cleanup chain,
but does not establish whether TF publication stopped or the temporary observer
failed to receive it. The bridge was already gone before a guarded GDB capture
could attach. A separate bounded 12-second continuity observer hit its outer
20-second limit without producing a sample report, so it is not valid continuity
or deadlock evidence. Local-state odometry was still updating in the existing
health guard after localization cleanup; navigation/AMCL was not ready.

No automatic extra restart, rollback, map switch, localization trigger, arm
command or robot motion was performed. Another full restart with early capture
needs renewed operator authorization. Moving navigation, map switching,
elevator/docking and longer continuity acceptance remain outstanding.
