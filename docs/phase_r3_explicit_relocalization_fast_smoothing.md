# Phase R3 Explicit Relocalization Fast Smoothing

Phase R3 keeps `robot_localization_bridge` as the only `map -> odom` TF owner
and narrows the bridge smoothing policy by correction source.

## Behavior

- AMCL gated corrections and ordinary online corrections continue to use the
  default map-odom smoothing rates:
  - translation: `0.20 m/s`
  - yaw: `0.25 rad/s`
- Force-accepted explicit Isaac relocalization keeps smoothing enabled, but
  large corrections use a per-correction active rate sized to finish within
  `explicit_relocalization_fast_max_duration_sec`.
- The production threshold is `1.0 m` or `0.35 rad`, with a target duration of
  `3.0 s`.
- Initial map lock still snaps immediately, matching the previous startup
  behavior.

## Rationale

Continuous AMCL corrections are expected to be small and should not create TF
jumps during motion. Explicit business relocalization is different: the robot is
expected to be stopped or in a controlled localization handoff. Applying an
11 m correction at the normal `0.20 m/s` rate can leave Nav2, docking, and the
App seeing a half-updated `map -> odom` transform for roughly a minute or more.

The fast policy keeps the same single publisher and current/target state model,
but makes large explicit relocalization converge in seconds instead of minutes.

## Status Fields

`/localization/bridge_status` reports:

- `smoothing_policy`
- `smoothing_translation_rate_mps`
- `smoothing_yaw_rate_radps`
- `configured_smoothing_translation_rate_mps`
- `configured_smoothing_yaw_rate_radps`
- `explicit_relocalization_fast_smoothing_enabled`
- `explicit_relocalization_fast_max_duration_sec`

When `smoothing_policy=explicit_relocalization_fast`, the active smoothing rates
are correction-specific and may be higher than the configured default rates.

## Rollback

Set:

```yaml
explicit_relocalization_fast_smoothing_enabled: false
```

and restart `robot_localization_bridge`.
