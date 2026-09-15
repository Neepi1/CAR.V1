# Common startup overlap — phase 1

## Scope and current acceptance

This change overlaps common startup work; it does not declare navigation ready
from a process PID. Hardware restart/timing acceptance is still pending. It does
not fix the separate Isaac cold-start internal-graph readiness issue.

The service entry remains `njrh-runtime.service`. The systemd wrapper completes
old-runtime cleanup before starting `run_common_services.sh`. No additional
daemon, persistent lease, navigation retry, or motion admission is introduced.

## Scheduling

After the existing CAN interface check, common startup:

1. launches static TF and its existing readiness check asynchronously;
2. launches the unchanged JT128/scan/FlatScan chain;
3. launches Ranger and wheel+IMU EKF with asynchronous readiness checks;
4. launches the existing docking camera/perception and starts its original
   fresh-observation check immediately, retaining the same timeout;
5. starts the health observer, then joins static TF, Ranger and EKF readiness;
6. starts the existing selected-map resident navigation branch (when enabled);
7. joins docking observation readiness before continuing to the existing
   floor/safety/mode/docking/API startup sequence.

Thus a slow camera no longer serializes the resident navigation launch. The
localization runner still owns its original local-state/scan/map checks and
initial localization. An invalid or absent map is not replaced with another map.
The late-autostart opt-out and the explicit FAST-LIO diagnostic prerequisites
are retained. Normal navigation does not start FAST-LIO or alter its parameters.

`pointcloud_started`, `static_tf_started`, `ranger_chassis_started` and
`docking_sensor_started` mean launch initiated, not data readiness. Existing
helper completion logs record when each check finishes; the common `*_ready`
stage records when that result is joined. Use both timestamps when measuring
overlap, rather than adding all stage durations together.

## Process ownership and failure semantics

`canonical_tf_helpers.sh` separates launch from completion. Its existing
`start_canonical_helper` entry remains synchronous for other callers and keeps
the existing reuse and mode-cleanup behavior.

`common_startup_helpers.sh` is private to the common owner. Launch happens in
the parent shell, so new producer PIDs are registered there. Only bounded
readiness checks run in child shells. A successful readiness worker cannot
remove the parent's producer registration. Reused processes are not adopted.

Startup checks are joined through their exit status and producer liveness.
Repeated start requests within this owner do not replace a running producer.
Cleanup cancels pending check descendants and the owned producers; the parent's
live job table prevents an old PID record from authorizing a signal to an
unrelated process. No broad process-name kill is added.

Existing startup failures still propagate. The runtime health loop still starts
after common startup completes; initializing streams are not fed into its
running-producer failure counters. This change does not add automatic recovery
or change the current docking failure policy.

## Tests

`src/robot_system_tests/test/test_common_startup_parallel.py` executes shell
startup with dummy processes and simulated ROS observations. It uses no ROS,
Docker control, navigation goals, velocities, or hardware services. It covers:

- slow readiness overlapping another helper launch;
- ownership retained after readiness succeeds;
- readiness failure propagation and pending-start cleanup;
- slow docking checks overlapping independent work;
- repeated start and reused-process semantics;
- unrelated PID protection;
- compatibility of synchronous helper callers;
- the actual common main sequence reaching navigation launch before the docking
  check joins, using fake driver scripts and ROS boundaries.

Run with `python3 -m pytest src/robot_system_tests/test/test_common_startup_parallel.py -q`.
Update the two existing scheduling assertions in `test_workspace_contracts.py`
without relaxing the unrelated runtime contracts.

## Hardware acceptance still required

After separate authorization, restart only the complete `njrh-runtime.service`.
Record current script hashes and service/process identities, journal/helper
timestamps, actual localization acceptance, Nav2 lifecycle readiness, and unique
TF/driver ownership. No motion goal is needed. Use bounded existing readiness
evidence; do not add high-frequency point-cloud subscriptions.

The acceptance measure is successful Nav2 readiness, not systemd `active` or an
open API port. Compare repeated starts to the pre-change evidence, including
startup failures and cleanup behavior. Keep reports under `/tmp/njrh_reports`.

## Deferred phase 2

Moving MapServer/Isaac asset loading ahead of sensor readiness is not implemented
here. First validate this common scheduling change on hardware and establish a
reliable initialization-complete signal for the current Isaac instance. A
visible trigger service or result publisher does not prove that GXF is running.
Do not substitute a fixed sleep or repeated trigger calls. Nav2/AMCL preload,
map switching, localization result validation and business workflows remain
unchanged in this phase.
