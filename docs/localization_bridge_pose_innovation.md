# Localization Bridge Pose-Innovation Contract

## Problem

`map -> odom` is a coordinate-transform parameter, not a robot pose. Its x/y change cannot be interpreted as robot travel when its yaw also changes.

For a localization result stamped at `t`, the bridge now uses:

```text
predicted_map_base(t) = current_map_odom * odom_base(t)
measured_map_base(t)  = AMCL or Isaac pose
robot_pose_innovation = measured_map_base(t) - predicted_map_base(t)

target_map_odom = measured_map_base(t) * inverse(odom_base(t))
```

Small, medium, hard-reject, forced-relocalization and consecutive-consistency decisions use the robot-pose innovation. `target_map_odom` remains the transform that the sole canonical bridge publishes.

## Why The Old Value Was Amplified

The 2026-07-14 post-navigation sample had:

- odom-origin radius: about `9.03 m`
- actual `map -> base_link` correction: `0.0924 m`
- yaw correction: `1.564 deg` (`0.0273 rad`)
- reported raw `map -> odom` x/y parameter change: about `0.3285 m`

A yaw correction at radius `r` requires a compensating transform translation of approximately:

```text
r * abs(delta_yaw) = 9.03 * 0.0273 = 0.246 m
```

That compensation combines vectorially with the real 9.24 cm pose correction. Therefore a roughly 32.9 cm `map -> odom` parameter delta is mathematically possible while the robot pose moves only 9.24 cm. Treating the parameter delta as physical drift caused valid AMCL corrections to be rejected by the 30 cm hard gate.

## Runtime Semantics

- `last_*_correction_translation_m` and `last_*_correction_yaw_rad`: physical `map -> base_link` innovation used by gates.
- `last_*_correction_dx_map_m`, `dy_map_m`, `dyaw_rad`: signed physical innovation in map coordinates.
- `last_*_map_odom_parameter_translation_m` and `yaw_rad`: raw target-transform parameter delta, diagnostics only.
- `remaining_translation_error_m` and `remaining_yaw_error_rad`: estimated physical correction still being applied.
- `remaining_map_odom_parameter_*`: transform-space remainder, diagnostics only.

Smoothing uses one progress factor for target x, y and shortest-path yaw. Its duration is selected from the physical translation/yaw correction rates. This keeps yaw and its lever-arm translation compensation coupled throughout an update.

## Post-Isaac Ordering

`/initialpose` and `/amcl_pose` are independent asynchronous channels. A queued AMCL pose can therefore arrive immediately after the bridge publishes a new Isaac seed even though that pose was computed from the previous particle-filter generation. Candidate acceptance alone is not proof that a correction reached `map -> odom`.

The runtime now enforces this order:

1. accept the explicit Isaac result and record its map-odom target sequence;
2. keep AMCL from replacing that target until smoothing completes;
3. publish the AMCL seed and reject poses stamped at or before the seed generation;
4. after the Isaac target settles, issue bounded `/request_nomotion_update` calls so stationary AMCL produces samples from the new generation;
5. wait at least `0.25 s`, then require two consistent, genuinely post-seed AMCL candidates;
6. finish the selected target so current and target sequences match before reporting relocalization success.

No-motion calls are rate-limited to one every `0.5 s` and capped at four per explicit sequence. If the robot begins moving before the optional static refine completes, that refine generation is abandoned and normal gated AMCL continues. This keeps manual relocalization responsive without allowing a stale callback or an unbounded service loop to undo or delay the global correction.

The manual relocalization HTTP path treats this refine as required when gated AMCL is active. It observes the bridge-owned no-motion sequence instead of issuing duplicate calls, and returns success only after the AMCL correction is accepted, fully smoothed, and published. Normal runtime and startup triggers do not inherit this HTTP wait. Typical static refinement adds about one second; the failure bound is four seconds.

`capture_relocalize_correction_compare.sh` reports `map -> base_link` first. If direct TF lookup is momentarily unavailable, it composes `map -> odom` with `odom -> base_link` from the same snapshot. The `map -> odom` row is explicitly labeled as a parameter delta.

## Verification

The package regression suite covers:

1. a robot 10 m from the odom origin with only 2 deg yaw innovation: physical translation is zero while parameter translation is about 0.349 m;
2. a physical 9.2 cm medium correction whose raw parameter translation exceeds 0.30 m;
3. consecutive transform hypotheses compared at one common robot reference pose.
4. queued pre-seed AMCL poses cannot preempt an unsettled Isaac target, and a fresh post-seed pose becomes eligible only after the generation guard.
5. stationary no-motion requests are not issued before the seed and settled Isaac target, are rate-limited, and stop after the configured retry bound.

Real-hardware acceptance still requires a full runtime restart followed by one 8-10 m two-point navigation and an explicit relocalization comparison. The primary result is `map -> base_link` correction; `map -> odom` parameter translation is not an acceptance metric.
