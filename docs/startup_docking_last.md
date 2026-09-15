# Main-stack first, docking last

## Scope

This supersedes the docking scheduling in `startup_parallel_phase1.md`.
The three unchanged-build restart measurements are under
`/tmp/njrh_reports/restart_triplet_20260909T1620Z_1mozjU`. They showed two real
safety processes in round 3, docking-observation timeouts tearing down common
startup, and 65–104 seconds between API process ownership and HTTP listening.
GPU measurements included an operator arm test and are not Isaac evidence.

## Scheduling and ownership

1. Static TF, JT128, Ranger and local state retain their existing parallel
   launch and checks. Common starts floor/safety/mode before navigation can
   create any competing helper; `NJRH_COMMON_SERVICES_MANAGED=true` makes the
   Nav2 helper path consumer-only. Standalone launch compatibility remains.
2. API launches before camera work and overlaps remaining main-stack startup.
   `robot_api_server_process_ready` means exact process ownership only.
   `robot_api_server_ready` now means a successful authenticated, one-second
   bounded HTTP metadata GET. Checks use the existing five-second common loop
   only until HTTP is available; no extra ROS participant is created.
3. Docking launches once, last: camera, perception, the existing observation
   check, then manager. A per-invocation temporary receipt distinguishes main
   initialization from a merely live wrapper. Navigation reports `ready`,
   `reused`, or `waiting_for_localization` after the bounded first attempt.
   No-map mode and an exited startup also permit this final initialization.
   Localization success is not required; its existing runtime readiness
   semantics, later explicit-result waiting and lifecycle order are unchanged.

The common owner continues normal health supervision while waiting for the
receipt and HTTP. It adds no fixed settle sleep, localization retry or startup
kill deadline. If HTTP never responds or initialization never finishes, docking
stays deferred; existing process failures/timeouts remain visible. A later
explicit localization or user-requested mode start is not serialized against
the already-running camera: this policy controls cold startup, not all future
business operations.

During common-managed startup the API's existing `docking_manager_start_command`
is an empty string. This disables only the fallback process creator, preventing
an early App request from starting a second manager ahead of the common owner.
The service client path, controlled undock and docking algorithms are unchanged.
Before the manager is available a docking/undock request can still report
service unavailable; it cannot bypass the existing controlled-undock behavior.
Standalone API and explicit `NJRH_DOCKING_MANAGER_AUTOSTART=false` retain the
prior on-demand command.

## Failure behavior

A missing fresh `/dock/target_observation` keeps camera/perception alive and
logs `docking_sensor_degraded`; the manager still starts and applies its existing
observation checks. Driver launch/configuration failure logs
`docking_startup_failed` and preserves the core. This does not add camera
respawn, silently retry a failed motion task or claim the camera is healthy.
There is one final initialization attempt, not a recovery loop.

## Tests and deployment

The offline tests cover real shell scheduling with fake producers, failed
observations, no map, first localization timeout, absent HTTP, malformed
receipts, exited owners, one-time launch, actual HTTP response validation and
standalone compatibility. Existing localization/preload/floor-handoff tests
remain in the regression set. No test moves hardware or calls Isaac.

Deployment uses the freshly inspected Jetson baseline after the operator paused
the concurrent writer. The newer floor-startup handoff code is retained, as is
the intervening fix that degrades docking after an observation timeout instead
of terminating common services. Only this startup delta is merged; unrelated
remote README and test changes are not replaced with local versions.

Runtime scripts are replaced atomically, so a live shell keeps its old inode.
No restart is performed by deployment. Software tests do not prove cold-start
timing or physical docking success. After explicit authorization restart only
the complete `njrh-runtime.service`; record actual HTTP time, first localization
outcome, Nav2 lifecycle time, docking startup timestamps, unique helper processes
and automatic restart count. Do not move the robot or subscribe to pointclouds.

API constructor optimization and the cause of Isaac's missing result are not
implemented here. This phase supplies earlier API launch and truthful timing;
it does not promise a specific startup-time reduction.
