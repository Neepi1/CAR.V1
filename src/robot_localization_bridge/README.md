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
- `publish_rate_hz`: `map -> odom` publish cadence; Jetson runtime uses `50.0`
- `map_odom_publish_gap_warn_ms`, `map_odom_publish_gap_fail_ms`: publisher heartbeat thresholds exposed in `/localization/bridge_status`; Jetson runtime uses `100.0` and `250.0`
- `tf_future_stamp_offset_sec`: small future-dating offset for Nav2 lookup jitter; Jetson runtime uses `0.10`
- `two_d_mode`: defaults to `true`
- `continuous_localization_mode`: legacy compatibility parameter. Phase A2 supports `triggered` only; any other value is ignored and reset to `triggered`.
- `status_topic`: JSON status output, defaults to `/localization/bridge_status`
- `triggered_max_result_age_ms`: bounded Isaac grid-search latency gate for service-triggered `/localization_result`; defaults to `5000.0`
- `max_odom_tf_age_ms`: freshness gate for the latest `odom -> base_link`, while candidate correction lookup uses `odom -> base_link` at the localization result stamp
- `triggered_allow_large_correction`: keeps explicit trigger relocalization eligible for the force-accept path; normal triggered updates still obey jump gating
- `amcl_input_enabled`: defaults to `false`; enables `/amcl_pose` as a continuous candidate input only when `NJRH_AMCL_LOCALIZATION_MODE` is `shadow` or `gated`
- `amcl_pose_topic`: defaults to `/amcl_pose`
- `amcl_runtime_status_file`: defaults to `/tmp/njrh_amcl_runtime_status.env`; read-only AMCL runtime contract exported by `run_amcl_shadow_localization.sh`
- `amcl_gate_mode`: `shadow` or `gated`; shadow records candidates only, gated accepts bounded AMCL corrections
- `amcl_max_result_age_ms`, `amcl_small_correction_translation_m`, `amcl_small_correction_yaw_rad`: AMCL-specific freshness and direct small-correction gates. The field profile directly accepts translation corrections up to `0.30 m`.
- `amcl_medium_correction_translation_m`, `amcl_medium_correction_yaw_rad`, `amcl_medium_correction_consistency_count`: medium AMCL gate. The field profile accepts corrections up to `0.70 m` only after 3 consecutive consistent candidates.
- `amcl_hard_reject_translation_m`, `amcl_hard_reject_yaw_rad`: hard reject / Isaac recovery gate. The field profile hard-rejects AMCL translation corrections above `1.20 m`.
- `amcl_max_xy_covariance`, `amcl_max_yaw_covariance`: covariance gates for AMCL pose input
- `amcl_seed_service`: defaults to `/robot_localization_bridge/seed_amcl_initial_pose` and publishes `/initialpose` from the current reliable `map -> base_link`

## TF Contract

- Sole publisher of `map -> odom`
- Consumes `robot_global_localization` pose and `robot_local_state` odometry
- The current implementation is C++, computes `map -> odom` from planar `map -> base_link` and the TF `odom -> base_link` at the localization result stamp, latches one-shot localization results, republishes at the configured rate, and rejects large jumps
- Correction handling and TF broadcasting are separate. Isaac explicit relocalization, AMCL gated corrections, and manual force-accept update a locked `MapOdomState`; the independent publisher callback group is the only code path that calls `sendTransform()`. Correction pause rejects new global corrections but keeps broadcasting the last accepted `map -> odom`.
- Explicit business relocalization calls, such as post-undock, startup, and pre-navigation localization, arm `force_accept_service` first. That lets the next localization result use the triggered gate and correct a drifted `map -> odom` once.
- Each accepted explicit Isaac force-accept relocalization increments `last_explicit_relocalization_sequence` on `/localization/bridge_status` and records `last_explicit_relocalization_accept_time` plus `last_explicit_relocalization_source`. AMCL small/medium gated corrections do not increment this sequence, so runtime settle barriers are triggered only by business relocalization, not every continuous AMCL correction.
- Field runtime publishes at 50 Hz with a 100 ms future stamp so Nav2 controller lookups have headroom when Jetson scheduling briefly delays TF delivery; this does not change the canonical TF owner.
- AMCL is a continuous candidate source only. It must run with `tf_broadcast=false`; this bridge computes AMCL candidates from `/amcl_pose` and historical `odom -> base_link`.
- Isaac triggered relocalization has the highest priority and can seed AMCL through `/initialpose`. AMCL gated corrections directly accept small covariance-gated updates, accept medium corrections only after consecutive consistency, and report large corrections for Isaac recovery instead of applying one-frame TF jumps.
- `/localization/bridge_status` reports `gate_mode`, result age and gate limit, original-stamp TF lookup state, latest odom TF freshness, accept/reject reasons, triggered/AMCL counters, `active_correction_source`, `last_accepted_source`, `last_rejected_source`, `last_explicit_relocalization_sequence`, `last_explicit_relocalization_accept_time`, `last_accepted_correction_translation_m`, `last_accepted_correction_yaw_rad`, `has_map_to_odom`, `map_to_odom_age_ms`, and `map_to_odom_publisher_owner`. It also reports `map_odom_publish_loop_hz`, `map_odom_publish_gap_ms`, `map_odom_publish_gap_max_ms`, `map_odom_publish_callback_duration_us`, `map_odom_latest_accepted_sequence`, `map_odom_last_published_sequence`, `map_odom_latest_source`, `map_odom_state_valid`, `map_odom_correction_paused`, `map_odom_frozen_due_to_pause`, `map_odom_publish_missed_count`, and `publisher_decoupled_from_correction=true`. Default `localization_settle_*` fields remain for compatibility; the live settle barrier is owned by `robot_api_server`.
- AMCL bridge readiness also uses the runtime status file. If AMCL input is enabled, `amcl_ready` cannot become true when the AMCL process, lifecycle, `/amcl_pose` publisher, scan-admission process, or `/amcl_scan_admission/status` publisher is missing. The status JSON exposes `amcl_state`, `amcl_process_alive`, `amcl_lifecycle_active`, `amcl_scan_admission_alive`, `amcl_pose_publisher_count`, `amcl_scan_admission_status_publisher_count`, `amcl_upstream_missing`, `localization_degraded`, and `amcl_degraded_reason`.
