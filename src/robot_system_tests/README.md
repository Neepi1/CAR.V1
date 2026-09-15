# robot_system_tests

`test_startup_cpu_affinity*.py` 覆盖八核冷启动、逐线程恢复原五核分配、进程身份、
子树边界、延后启动及 shell/launch 接线。Linux 实测只操作测试自建进程；
整链提速仍需授权重启验收，不连接 ROS、不移动小车。

Centralized baseline tests for the first scaffold iteration.

## Coverage

- `test/test_local_state_startup_budget.py` runs the actual local-state/common
  startup flow with inert executables and simulated ROS readiness. It covers
  late IMU input, one deadline, exact process ownership, early child failure,
  client shutdown and retained attempt logs. Run in a private PID/network
  namespace; no robot node, motion or sensor subscription is used.

- `test/test_startup_trigger_tf_warmup.py` covers same-node TF confirmation with
  a post-acceptance reader, legacy output fallback, timeout accounting, handoff,
  rejected/failed RPCs and cleanup. Real isolated TF queue tests supplement it.

- `test/test_local_state_package_lookup.py` preserves ament overlay precedence,
  missing-package/binary errors and space-containing paths with one lookup.

- `test/test_robot_api_launcher.py` runs the real launcher with isolated setup,
  executable and OS adapters to preserve argv, affinity, cold/inherited setup,
  build fallback and failure propagation while removing redundant CLI startup.

- `test/test_runtime_cleanup_races.py` executes the real cleanup flow against
  a synthetic process table: late CLI/node children, query-tool exclusion,
  read-only checks, and mandatory failure for unkillable real-node identities.

- Full-restart optimization: early AMCL, nonblocking resident join and slow-map
  overlap tests preserve readiness and cleanup. Cold status-observer tests
  preserve early preparation, owner checks and mandatory final READY commits.
  Lifecycle endpoint reuse tests
  preserve ordered transitions and late responses. See
  [acceptance scope](../../docs/restart_latency_40s.md).
- Client auxiliary-endpoint constructor contracts are tested separately from
  the real private-domain smoke fixture in `robot_bringup/test`; neither is a
  substitute for full-runtime latency acceptance.
- AMCL composite startup: `test/test_amcl_composite_startup.py` covers shared
  observation, actual-frame checkpoints, partial timeout and evidence completeness.
  `test/test_amcl_startup_sequence.py` also covers short-budget seed deferral
  and pending propagation without duplicate calls under the same budget.

- Cold-start scan ownership: `test/test_navigation_scan_startup.py` exercises
  the actual systemd branch and scan helpers with a simulated ROS boundary.
  Late discovery must not trigger a scan-enable service, while missing,
  duplicate and foreign publishers cannot report ready. See
  [scope and restart acceptance](../../docs/navigation_scan_cold_start.md).

- Five-core navigation placement: `test/test_navigation_cpu_profile.py` exercises
  the real Bash resolver, field-override precedence, inherited child environments,
  restoration, exact per-group masks, non-navigation scope, opt-in startup-parent
  placement and sensor-wrapper placement before initialization. OS
  scheduling is a fixture boundary; no robot node or live process is operated.
  See [profile scope and hardware acceptance](../../docs/navigation_five_cpu_profile.md).

- Parallel Isaac cold start: `test/test_isaac_parallel_startup.py` covers
  current-process GXF completion, dependency ordering and delayed startup,
  with all shell state isolated from production `/tmp`. Run startup tests
  locally or in a separate network/device-isolated container, never in the
  live robot container. Hardware cold-start acceptance remains separate.

- Docking-last startup: `test/test_common_startup_docking_last.py` exercises
  the actual common sequence with fake producers, missing docking observations,
  single-owner helper startup, startup receipts, and a local HTTP fixture.
  These tests never start a robot node or send motion/localization requests.

- Production Nav2 preload environment: `test/test_nav2_prestart_environment.py`
  covers real outer-launcher Docker arguments, env/override precedence, and
  fresh/repeated installation env writes using isolated fake boundaries.

- Nav2 / Isaac startup decoupling: `test/test_navigation_localization_startup.py`
  exercises early process preload, retained processes after a first localization
  failure, later bridge acceptance, stop cleanup and launch-record ownership.
  It uses fake processes/ROS boundaries and does not operate the robot.

- Common startup overlap, readiness failure propagation, process ownership,
  cleanup and reuse: `test/test_common_startup_parallel.py` runs fake processes
  and simulated ROS boundaries, including the real common startup sequence.
  It does not restart services or send robot commands. Hardware startup timing
  still requires separate acceptance.
- Required reports exist
- Nav2 fixed defaults remain intact
- Canonical TF ownership stays single-sourced
- Navigation recorder v3: callback evidence, goal/pose/odom timestamps, bounded
  publisher discovery, explicit diagnostic gaps, and report preservation on
  interruption/decoder error. Tests use fake ROS, no robot or ROS context.

Run the recorder-only suite with:

```bash
python3 -m pytest src/robot_system_tests/test/test_navigation_obstacle_recorder.py src/robot_system_tests/test/test_navigation_observer_evidence.py -q
```

These tests do not validate real obstacle avoidance. Operator-run hardware
capture remains necessary, and missing optimizer diagnostics remain explicit.

## Follow-Up

- AMCL startup behavior coverage: `test_amcl_startup_sequence.py` runs real Bash
  orchestration functions with isolated ROS/process fixtures. It checks resumable
  preparation, process/map/owner invalidation, seed retry, and short-client budget
  expiry without killing a resident process. See
  [startup verification](../../docs/amcl_startup_sequence.md).

- Add runtime ROS graph tests for TF uniqueness once the Humble environment and launch stack are available.
- Extend coverage to mapping_result artifacts, floor assets, localizer reload, odom anomalies, and failure handling in later phases.
# Camera cleanup regression

`test/test_nav2_background_startup.py` exercises environment forwarding,
after-stack scheduling, single-worker join, failure fallback, TERM cleanup and
floor handoff. It uses fixture-owned children and makes no production ROS calls.
The real background worker is also exercised with a blocked launch receipt:
localization proceeds first, lifecycle waits for the matching owner, and final
READY waits for completion. Missing/wrong/dead owners cannot use active-state
fallback; pending-receipt reuse, TERM cleanup and floor handoff are covered.
The same suite covers optional cold-start process staging after existing
localization-stack initialization, before the trigger, with unchanged opt-out,
non-systemd ordering and floor-handoff ownership.

`test_nav2_lifecycle_sequence_behavior.py` covers retained late ChangeState
responses, read-only state confirmation, no timeout redispatch, trust modes,
handoff and pending-request cleanup. `test_trigger_startup_timing.py` verifies
that stage timing preserves existing dispatch counts, returns and TF markers.

`test/test_startup_context_observer.py` covers one-node final service/bridge
observation, exact sequence, missing inputs, handoff, fallback and durable-write
failure. It also exercises actual startup ordering and the prewarmed native
worker's parent-owned join, single-worker reuse, TERM/IPC cleanup, adopted floor,
late commit and unchanged cold compatibility path. Native source-timestamp
filtering has pure C++ tests plus private-network real-DDS backlog acceptance in
`robot_bringup/test/isolated_context_observer_smoke.py`.
`test_runtime_cleanup_races.py` also covers the real occupancy launch
parent and its respawning component; transport/process boundaries are synthetic.

`test/test_runtime_cleanup_races.py` exercises the real cleanup flow against a
synthetic process table: it stops navigation's camera336l wrapper, launch and
orphan container, but leaves another Orbbec camera, frame-name observer and
similarly prefixed namespace intact. It sends no real process signals.
