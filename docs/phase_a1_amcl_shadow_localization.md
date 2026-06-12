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
- `shadow`: AMCL runs on `/scan` and publishes `/amcl_pose`; `robot_localization_bridge` computes candidate corrections but does not update `map -> odom`.
- `gated`: default active integration mode. AMCL runs on `/scan`; `robot_localization_bridge` accepts only small, covariance-gated corrections into `map -> odom`.

`AMCL gated` is the active Phase A2 default. Use `shadow` to observe without
correction, or `disabled` to return to Isaac-triggered-only localization.

## TF Contract

AMCL uses:

- `scan_topic: /scan`
- `map_topic: /map`
- `tf_broadcast: false`

`/flatscan` is an Isaac `FlatScan` topic and is not an AMCL input. The only
`map -> odom` publisher remains `robot_localization_bridge`; the only
`odom -> base_link` publisher remains `robot_local_state`.

## Source Arbitration

`robot_localization_bridge` receives:

- Isaac triggered `/localization_result`: highest priority, allowed to perform explicit large correction when the trigger wrapper arms force-accept.
- AMCL `/amcl_pose`: continuous candidate source; shadow records candidates only, gated accepts only small corrections.
- Isaac triggered `/localization_result` only. Phase A2 removes the Isaac continuous replacement path; AMCL is the only continuous localization candidate source.

After an Isaac triggered correction is accepted, the bridge publishes
`/initialpose` to seed AMCL. If AMCL starts after that accepted result, the AMCL
runner calls `/robot_localization_bridge/seed_amcl_initial_pose` to publish a
seed from the current reliable `map -> base_link`.

## Run

```bash
export NJRH_AMCL_LOCALIZATION_MODE=shadow
bash scripts/jetson/runtime_overlay/scripts/run_amcl_shadow_localization.sh --restart
bash scripts/jetson/runtime_overlay/scripts/verify_amcl_shadow_localization.sh --mode shadow --duration-sec 60
```

For gated observation:

```bash
export NJRH_AMCL_LOCALIZATION_MODE=gated
bash scripts/jetson/runtime_overlay/scripts/run_amcl_shadow_localization.sh --restart
bash scripts/jetson/runtime_overlay/scripts/verify_amcl_shadow_localization.sh --mode gated --duration-sec 120
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
