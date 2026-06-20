# Phase S1 Startup Readiness Semantics

## Problem

A cold navigation start could report `AMCL_READY` in the AMCL runner while
`/api/v1/status` and `/api/v1/navigation/state` still showed:

- `localization_degraded=true`
- `localization_degraded_reason=AMCL_CORRECTION_NOT_READY`
- `using_triggered_baseline_only=true`

This was misleading for a stationary robot. After the initial triggered
localization seeds AMCL and `map -> odom` is live, AMCL may not publish a fresh
correction until motion or a no-motion update produces a new sample. That is a
normal startup/standby state, not a reason to block Nav2 if
`robot_localization_bridge.safe_for_goal_start=true`.

## Change

- `robot_localization_bridge` now treats gated AMCL as ready when the AMCL
  process is active, seeded, scan admission is healthy, and tracking readiness
  is satisfied. Static standby counts as tracking-ready while the robot is not
  moving.
- `amcl_correction_ready` remains the strict signal for a fresh correction that
  may be accepted into `map -> odom`.
- `amcl_correction_pending=true` reports the normal gap between startup seed
  readiness and the first fresh continuous correction.
- `localization_degraded` is reserved for actual missing/not-tracking states,
  such as AMCL upstream missing, AMCL runtime not ready, or stale AMCL pose
  while the robot is moving.
- `run_navigation_runtime_services.sh` now logs `STARTUP_STAGE` markers with
  elapsed seconds for cold-start timing diagnosis.

## What This Does Not Change

- AMCL still does not publish TF.
- `map -> odom` remains owned only by `robot_localization_bridge`.
- `odom -> base_link` remains owned only by `robot_local_state`.
- Normal navigation still does not trigger hidden relocalization.
- AMCL corrections still use the existing bridge correction gates and smoothing.
- JT128 pointcloud QoS/DDS, FAST-LIO2, Nav2 plugins, and the speed chain are
  unchanged.

## Hardware Validation Needed

On the Jetson after a cold reboot:

1. Start the normal runtime and collect the resident navigation log.
2. Confirm `STARTUP_STAGE` entries identify the slow stage.
3. Before motion, confirm `/localization/bridge_status` can show
   `amcl_ready=true`, `amcl_correction_ready=false`, and
   `amcl_correction_pending=true` without `localization_degraded=true`.
4. Confirm `/api/v1/status` and `/api/v1/navigation/state` no longer report
   localization recovery required solely because AMCL has not produced a fresh
   correction while stationary.
5. Send a short Nav2 goal and confirm AMCL correction readiness becomes fresh
   during motion without a `map -> odom` ownership conflict.
