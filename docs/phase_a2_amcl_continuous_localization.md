# Phase A2 AMCL Continuous Localization

Phase A2 makes AMCL the only continuous localization candidate source and
removes the earlier Isaac continuous replacement path.

## Runtime Order

1. Start canonical local state first so `odom -> base_link` is owned only by
   `robot_local_state`.
2. Start Isaac Occupancy Grid Localizer in `triggered` mode only.
3. Start `robot_localization_bridge`; it remains the only `map -> odom`
   publisher.
4. Call `/global_localization/trigger` and require bridge acceptance plus a live
   `map -> odom`.
5. Start AMCL when `NJRH_AMCL_LOCALIZATION_MODE=shadow` or `gated`.
6. Seed AMCL through `/robot_localization_bridge/seed_amcl_initial_pose`, which
   publishes `/initialpose` from the current bridge-approved `map -> base_link`.

## Topic Ownership

- Isaac triggered input: `/flatscan`
- Isaac triggered result: `/localization_result`
- AMCL input: `/scan`
- AMCL result: `/amcl_pose`
- AMCL TF: disabled with `tf_broadcast=false`
- Canonical TF: `map -> odom` from `robot_localization_bridge` only

The removed Isaac continuous path no longer starts a repository flatscan
forwarder. Isaac may still expose internal `/flatscan_localization` endpoints,
but there is no repository process forwarding `/flatscan` into that topic for
background localization.

## Modes

`NJRH_AMCL_LOCALIZATION_MODE` controls AMCL:

- `shadow`: AMCL runs and bridge reports candidates, but AMCL cannot change
  `map -> odom`.
- `gated`: default active Phase A2 mode. Bridge accepts only small
  covariance-gated AMCL corrections.
- `disabled`: AMCL is stopped; only Isaac triggered relocalization is active.

Use `shadow` as the one-command rollback when field data shows AMCL candidates
are too sparse or too large for active correction.
