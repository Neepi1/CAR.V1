# Local-state cold-start budget and ownership

## Reason and scope

The September 14 startup trace showed the first IMU publication after the
local-state preprocessor had already been stopped. The previous launcher waited
only 8 seconds for IMU outputs before creating EKF. Independently, the common
owner waited for a command-line substring and then started a 12-second endpoint
timeout followed by a new recheck budget. Cleanup commands and unrelated
processes could satisfy that process substring, masking the actual failure.

The retained logs strongly support this failure path, but failed helper stderr
was overwritten by the next attempt; the exact originally matched PID was not
captured. Isolated replay confirms both code defects. Hardware restart
acceptance of the repair remains separate.

## Behavior

- `LOCAL_STATE_STARTUP_TIMEOUT_SEC` defaults to 30 seconds. It is a deadline,
  not a fixed sleep. Normal inputs complete immediately when ready.
- The common launcher creates one monotonic deadline and passes it to the
  local-state child and its startup waiter. A standalone EKF launcher creates
  its own deadline if none is supplied. The deadline is process-local startup
  state, not a persisted lease or motion gate.
- Existing wheel preprocessor and IMU ownership remain unchanged. The existing
  corrected-IMU and bias publisher/message requirements still precede EKF
  creation. They receive only the remaining shared budget.
- The new EKF-owned path replaces, rather than adds to, the old process/endpoint
  timeouts and final recheck. Legacy individual IMU/process/start-ready timeout
  overrides do not shorten or reset this shared cold-start budget. Reuse checks
  and explicit FAST-LIO/passthrough readiness retain their existing behavior.
- The startup waiter scans only until the owned EKF is found, using executable
  identity, complete remap arguments and descendant ownership; it rejects
  zombies, deleted executables, foreign processes and duplicate owned EKFs.
  Jetson lacks `/proc/PID/task/PID/children`, so descendants are resolved from
  one process snapshot, without spawning recursive pgrep subprocesses.
- The existing native endpoint check then receives the remaining time. The
  existing `fresh_tf` caller mode still checks `odom -> base_link` with
  `LOCAL_STATE_TF_READY_MAX_AGE_SEC`. It does not add that check to endpoint mode.
- No repeated cold ROS participants are created while waiting for the EKF
  process. Once started, the native readiness client is monitored for owner
  exit and deadline expiry. Only this disposable client is terminated by the
  observer. Producers remain owned/cleaned by the existing common launcher.
- Child failure is reported with its helper log immediately. A child exiting
  during endpoint checking cannot be accepted as ready. Explicit native success
  output still permits bounded client-shutdown cleanup, as before.
- Readiness timeout does not include the existing bounded INT/TERM/KILL cleanup
  duration. The budget is not reset by a final recheck. No new respawn policy is
  introduced; existing systemd whole-chain policy is unchanged.

## Logs

Before a local-state helper log is reused, its previous content is copied to a
unique UTC/random-suffix file under `/tmp/njrh_reports/local_state_startup`.
`NJRH_LOCAL_STATE_STARTUP_REPORT_DIR` can select a diagnostic report directory.
The current attempt remains at the existing `robot_local_state_common.log` path,
so existing tools continue to work. Failures print phase, reason and that path,
plus the original log tail. Retained reports require normal operator retention
management; this repair does not delete old reports.

## Validation

`test_local_state_startup_budget.py` executes the real common/child startup
flow. Only OS executables, ament package lookup and ROS readiness are fixtures.
Use a private PID/mount/network namespace **with a separately mounted private
`/tmp`**, or an isolated test container without production runtime mounts.
`unshare --mount` alone does not isolate existing filesystem contents. Never run
the broader production-runtime stop/cleanup shell tests against a live shared
`/tmp`: mocked process/service calls do not prevent real status-file cleanup.
The regression includes IMU delays of 8.5 and 12 seconds, absent inputs, child
exit, no new endpoint budget, explicit success before stuck client shutdown,
exact process/remap matching and previous-log preservation. Existing startup,
package lookup, ownership and cleanup tests must also pass.

After deployment and separate authorization, restart only the full
`njrh-runtime.service`. Record first-attempt EKF/endpoint readiness, Nav2 and
AMCL readiness, service restart count, process uniqueness and the existing
odometry health snapshot. No motion is required. This change does not claim a
30-second whole-navigation startup time or guarantee success with missing IMU.

No API/ROS interfaces, fusion parameters, JT128/IMU arithmetic or rates,
FAST-LIO2, CPU affinity, DDS version, TF ownership or velocity chain are changed.
