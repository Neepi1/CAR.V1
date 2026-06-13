# Phase A1 AMCL Shadow Localization

Phase A1 adds AMCL as an opt-in continuous localization candidate. It does not
replace Isaac triggered relocalization and it does not make AMCL a TF owner.

## Modes

Configure the mode with:

```bash
export NJRH_AMCL_LOCALIZATION_MODE=disabled
```

Supported values:

- `disabled`: AMCL is not started and the existing Isaac triggered path is unchanged.
- `shadow`: AMCL runs on `/scan_amcl` and publishes `/amcl_pose`; `robot_localization_bridge` computes candidate corrections but does not update `map -> odom`.
- `gated`: default active integration mode. AMCL runs on `/scan_amcl`; `robot_localization_bridge` accepts only small, covariance-gated corrections into `map -> odom`.

`AMCL gated` is the active Phase A2 default. Use `shadow` to observe without
correction, or `disabled` to return to Isaac-triggered-only localization.

## TF Contract

AMCL uses:

- `scan_topic: /scan_amcl`
- `map_topic: /map`
- `tf_broadcast: false`

`/scan_amcl` is an AMCL production admission input, not a debug topic. It is
derived from `/scan` only when AMCL mode is `shadow` or `gated`; the relay
preserves the original `LaserScan.header.stamp`, frame id, angular metadata, and
ranges. It drops scans older than `NJRH_AMCL_SCAN_MAX_AGE_MS` and scans whose
`odom <- scan_frame` transform is unavailable at the original scan stamp, then
limits the admitted stream to `NJRH_AMCL_SCAN_RATE_HZ` (default 5 Hz). `/scan`
remains available to other consumers, and `/flatscan` remains an Isaac
`FlatScan` topic, not an AMCL input. The only `map -> odom` publisher remains
`robot_localization_bridge`; the only `odom -> base_link` publisher remains
`robot_local_state`.

## Source Arbitration

`robot_localization_bridge` receives:

- Isaac triggered `/localization_result`: highest priority, allowed to perform explicit large correction when the trigger wrapper arms force-accept.
- AMCL `/amcl_pose`: continuous candidate source; shadow records candidates only, gated accepts only small corrections.
- Isaac triggered `/localization_result` only. Phase A2 removes the Isaac continuous replacement path; AMCL is the only continuous localization candidate source.

After an Isaac triggered correction is accepted, the bridge publishes
`/initialpose` to seed AMCL. If AMCL starts after that accepted result, the AMCL
runner calls `/robot_localization_bridge/seed_amcl_initial_pose` to publish a
seed from the current reliable `map -> base_link`.

Phase A1.2 makes readiness explicit. AMCL has its own TF buffer, so startup
waits for `/map`, `/scan`, `map -> odom`, `odom -> base_link`, and
`base_link -> scan_frame`, then warms that AMCL-local TF cache for
`NJRH_AMCL_TF_WARMUP_SEC` before seeding. Seed is retried
`NJRH_AMCL_SEED_RETRY_COUNT` times, and the bridge refuses AMCL corrections
until `amcl_seed_succeeded=true`. AMCL is not ready unless `/amcl_pose` remains
fresh and the `/scan_amcl` admission status is healthy.

The runner drives AMCL lifecycle transitions by calling the standard
`/amcl/change_state` service directly, then verifies `/amcl` reaches `active`.
This avoids depending on `ros2 lifecycle set` transition discovery during
Jetson startup.

## Run

```bash
export NJRH_AMCL_LOCALIZATION_MODE=shadow
bash scripts/jetson/runtime_overlay/scripts/run_amcl_shadow_localization.sh --restart
bash scripts/jetson/runtime_overlay/scripts/verify_amcl_shadow_localization.sh --mode shadow --seed --tf-warmup-sec 3 --scan-admission --duration-sec 60 --check-logs
```

For gated observation:

```bash
export NJRH_AMCL_LOCALIZATION_MODE=gated
bash scripts/jetson/runtime_overlay/scripts/run_amcl_shadow_localization.sh --restart
bash scripts/jetson/runtime_overlay/scripts/verify_amcl_shadow_localization.sh --mode gated --seed --tf-warmup-sec 3 --scan-admission --duration-sec 120 --check-logs
```

Rollback:

```bash
export NJRH_AMCL_LOCALIZATION_MODE=disabled
bash scripts/jetson/runtime_overlay/scripts/run_amcl_shadow_localization.sh --stop
```

## Field Notes

AMCL may be unstable on oversized or highly symmetric maps. In that case keep
the mode at `shadow`, inspect covariance and candidate corrections from
`/localization/bridge_status`, and use Isaac triggered relocalization for large
pose recovery.

If AMCL is not ready, the runtime stays on the Isaac triggered plus odom
baseline and reports the failure through runner warnings and
`/localization/bridge_status.amcl_ready=false`; it does not let AMCL publish TF
or add a second `map -> odom` owner.
