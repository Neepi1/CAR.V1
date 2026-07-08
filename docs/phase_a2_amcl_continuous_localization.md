# Phase A2 AMCL Continuous Localization

Phase A2 makes AMCL the only continuous localization candidate source and
removes the earlier Isaac continuous replacement path.

## Runtime Order

1. Start canonical local state first so `odom -> base_link` is owned only by
   `robot_local_state`.
2. Start Isaac Occupancy Grid Localizer in `triggered` mode only.
3. Start `robot_localization_bridge`; it remains the only `map -> odom`
   publisher.
4. Start AMCL resident when `NJRH_AMCL_LOCALIZATION_MODE=shadow` or `gated`,
   activate the lifecycle node, warm its local TF buffer without requiring
   `map -> odom`, and start the C++ `/scan_amcl` admission relay.
5. Call `/global_localization/trigger` and require bridge acceptance plus a live
   `map -> odom`.
6. Complete AMCL readiness by waiting for `map -> odom`, then seed AMCL through
   `/robot_localization_bridge/seed_amcl_initial_pose`, which
   publishes `/initialpose` from the current bridge-approved `map -> base_link`.
7. Wait for a fresh `/amcl_pose`; if the robot is stationary, subscribe to
   `/amcl_pose` first and then request one `/request_nomotion_update` so the
   bridge sees AMCL's single no-motion response without moving the chassis.
8. Report shadow readiness only after AMCL is active, seeded, scan admission is
   healthy, and AMCL has produced at least one pose sample. During motion, that
   sample must remain fresh; while stopped or docked, stale `/amcl_pose` is
   reported as `amcl_not_moving_no_update_ok` rather than a fatal localization
   failure.

## Topic Ownership

- Isaac triggered input: `/flatscan`
- Isaac triggered result: `/localization_result`
- AMCL production input: `/scan_amcl`, derived from `/scan` only in AMCL mode
- AMCL result: `/amcl_pose`
- AMCL TF: disabled with `tf_broadcast=false`
- Canonical TF: `map -> odom` from `robot_localization_bridge` only

The `/scan_amcl` relay is C++ by default:
`robot_localization_bridge/amcl_scan_admission_node`. It does not restamp scans
and does not change ranges, intensities, frame id, angular metadata, or scan
timing fields. It drops stale scans and scans without `odom <- scan_frame` at
the original scan stamp, reducing AMCL MessageFilter pressure without hiding TF
delay. Python `amcl_scan_admission_relay.py` remains an explicit fallback with
`NJRH_AMCL_SCAN_ADMISSION_IMPL=python`; the default C++ path fails fast if the
binary is missing. If seed, moving-state fresh `/amcl_pose`, or scan admission
fails, the navigation runtime keeps the Isaac triggered plus odom baseline
active and exposes AMCL as not ready or degraded rather than pretending
continuous correction is active.

## Readiness Status

`run_amcl_shadow_localization.sh` writes
`/tmp/njrh_amcl_runtime_status.env` as a TTL-bound startup/readiness snapshot.
It includes `AMCL_STATUS_STAMP_SEC`, process/lifecycle state, scan-admission
state, seed response state, static-standby state, tracking readiness, and gated
correction readiness. A stale `AMCL_FAILED` in this file is diagnostic only:
`robot_localization_bridge` reports `amcl_status_source=stale_file_ignored` and
uses live AMCL graph/subscription evidence instead of keeping localization
permanently degraded. Static `/amcl_pose` staleness is normal when AMCL is
seeded and the robot is not moving; gated correction still requires
`amcl_correction_ready=true`, which means a fresh AMCL pose is available for
the correction gate.

The removed Isaac continuous path no longer starts a repository flatscan
forwarder. Isaac may still expose internal `/flatscan_localization` endpoints,
but there is no repository process forwarding `/flatscan` into that topic for
background localization.

## Modes

`NJRH_AMCL_LOCALIZATION_MODE` controls AMCL:

- `gated`: commercial navigation default. Bridge accepts only small
  covariance-gated AMCL corrections into `map -> odom`.
- `shadow`: field audit / odom-only rollback mode. AMCL runs and bridge reports
  candidates, but AMCL cannot change `map -> odom`.
- `disabled`: AMCL is stopped; only Isaac triggered relocalization is active.

The runtime profile defaults to `gated` for commercial navigation after the
two-point field loop showed odom-only/shadow runs could finish within the stale
map frame while the post-goal triggered relocalization still moved `map -> odom`
by about 0.28-0.29 m. Large AMCL offsets still require Isaac triggered
relocalization, and `shadow` remains the odom-only audit rollback.

## Verification

Use the resident readiness check without moving the robot:

```bash
export NJRH_AMCL_LOCALIZATION_MODE=gated
bash scripts/jetson/runtime_overlay/scripts/verify_amcl_readiness_status.sh \
  --mode gated --request-nomotion-update --expect-static-standby
```

For a dynamic navigation window, start the observer and then send a short
operator-selected goal:

```bash
export NJRH_AMCL_LOCALIZATION_MODE=gated
bash scripts/jetson/runtime_overlay/scripts/observe_amcl_navigation_shadow_180s.sh \
  --duration-sec 180
```
