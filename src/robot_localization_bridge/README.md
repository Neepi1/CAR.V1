# robot_localization_bridge

Bridge that synthesizes the only canonical `map -> odom` transform.

## Parameters

- `publish_tf`: defaults to `true`
- `map_frame`: `map`
- `odom_frame`: `odom`
- `localization_topic`: global pose input, defaults to `/global_localization/pose`
- `local_odom_topic`: local odom input, defaults to `/local_state/odometry`
- `health_topic`: Bool output, defaults to `/localization/health`
- `jump_threshold_m`, `timeout_sec`: active runtime gating controls
- `two_d_mode`: defaults to `true`

## TF Contract

- Sole publisher of `map -> odom`
- Consumes `robot_global_localization` pose and `robot_local_state` odometry
- The current implementation computes `map -> odom` from planar `map -> base_link` and `odom -> base_link`, and rejects large jumps
