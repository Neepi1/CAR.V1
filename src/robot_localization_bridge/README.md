# robot_localization_bridge

Bridge that synthesizes the only canonical `map -> odom` transform.

## Parameters

- `publish_tf`: defaults to `true`
- `map_frame`: `map`
- `odom_frame`: `odom`
- `localization_topic`: global pose input, defaults to `/global_localization/pose`
- `local_odom_topic`: local odom input, defaults to `/local_state/odometry`
- `health_topic`: Bool output, defaults to `/localization/health`
- Startup supervision does not subscribe to `health_topic`; bridge readiness is checked with graph endpoints plus live `map -> odom` to avoid QoS-durability probe false negatives.
- `jump_threshold_m`, `timeout_sec`: active runtime gating controls
- `forced_jump_threshold_m`: maximum one-shot correction accepted after the API arms `force_accept_service`
- `force_accept_service`: `std_srvs/Trigger` service, defaults to `/robot_localization_bridge/force_accept_next_localization`
- `publish_rate_hz`: `map -> odom` publish cadence; Jetson runtime uses `20.0`
- `tf_future_stamp_offset_sec`: small future-dating offset for Nav2 lookup jitter; Jetson runtime uses `0.05`
- `two_d_mode`: defaults to `true`

## TF Contract

- Sole publisher of `map -> odom`
- Consumes `robot_global_localization` pose and `robot_local_state` odometry
- The current implementation is C++, computes `map -> odom` from planar `map -> base_link` and `odom -> base_link`, latches one-shot localization results, republishes at the configured rate, and rejects large jumps
- Explicit business relocalization calls, such as post-undock and pre-navigation localization, arm `force_accept_service` first. That lets the next localization result correct a drifted `map -> odom` once, while normal background updates still use `jump_threshold_m`.
- Field runtime publishes at 20 Hz with a 50 ms future stamp so Nav2 controller lookups have headroom when Jetson scheduling briefly delays TF delivery; this does not change the canonical TF owner.
