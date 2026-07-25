# robot_localization_bridge

Bridge that synthesizes the only canonical `map -> odom` transform.

The Jetson field profile defaults to `NJRH_AMCL_LOCALIZATION_MODE=gated`, so
bounded AMCL corrections remain active during navigation. Apply profile changes
with a full `njrh-runtime.service` restart; do not restart AMCL or this bridge
independently from the navigation chain.

## Parameters

- `publish_tf`: defaults to `true`
- `map_frame`: `map`
- `odom_frame`: `odom`
- `localization_topic`: global pose input, defaults to `/global_localization/pose`
- `local_odom_topic`: local odom input, defaults to `/local_state/odometry`
- `health_topic`: Bool output, defaults to `/localization/health`
- `floor_health_topic`: typed `robot_interfaces/msg/LocalizationHealth`,
  defaults to `/localization/floor_health`; this is separate from the legacy
  Bool topic and carries floor transaction/runtime-context state.
- `begin_floor_transition_service`: typed
  `robot_interfaces/srv/BeginFloorTransition`, defaults to
  `/robot_localization_bridge/begin_floor_transition`.
- `live_floor_transition_service_enabled`: defaults to `false`. While disabled,
  BEGIN and COMMIT are rejected with `LIVE_FLOOR_TRANSITION_DISABLED`; ABORT
  remains available to converge an already-active transaction to a locked safe
  state. Isolated transaction tests opt in explicitly, but the Jetson runtime
  profile stays disabled.
- `floor_transition_pause_owner`: exact owner required for the transaction
  pause record, defaults to `robot_floor_manager`.
- Startup supervision does not subscribe to `health_topic`; bridge readiness is checked with graph endpoints plus live `map -> odom` to avoid QoS-durability probe false negatives.
- `jump_threshold_m`, `timeout_sec`: active runtime gating controls
- `forced_jump_threshold_m`: maximum one-shot correction accepted after the API arms `force_accept_service`
- `force_accept_service`: `std_srvs/Trigger` service, defaults to `/robot_localization_bridge/force_accept_next_localization`
- `correction_pause_service`: legacy `std_srvs/SetBool` endpoint, defaults to `/robot_localization_bridge/set_correction_paused`
- `correction_pause_lease_service`: owner-scoped `robot_interfaces/srv/SetCorrectionPause` endpoint, defaults to `/robot_localization_bridge/set_correction_pause_lease`
- `correction_pause_state_topic`: transient-local `robot_interfaces/msg/CorrectionPauseState`, defaults to `/localization/correction_pause_state`
- `publish_rate_hz`: `map -> odom` publish cadence; Jetson runtime uses `50.0`
- `map_odom_publish_gap_warn_ms`, `map_odom_publish_gap_fail_ms`: publisher heartbeat thresholds exposed in `/localization/bridge_status`; Jetson runtime uses `100.0` and `250.0`
- `map_odom_smoothing_enabled`: defaults to `true`; accepted corrections update a target transform and the publisher slews current `map -> odom` toward it
- `map_odom_smoothing_translation_rate_mps`, `map_odom_smoothing_yaw_rate_radps`: physical `map -> base_link` correction-rate limits used to choose one shared smoothing duration; Jetson runtime uses `0.20 m/s` and `0.25 rad/s`. The resulting `map -> odom` x/y/yaw parameters advance with one progress factor so yaw and its lever-arm translation compensation cannot separate transiently.
- `map_odom_smoothing_snap_translation_epsilon_m`, `map_odom_smoothing_snap_yaw_epsilon_rad`: small remaining-error snap thresholds
- `explicit_relocalization_fast_smoothing_enabled`, `explicit_relocalization_fast_correction_translation_m`, `explicit_relocalization_fast_correction_yaw_rad`, `explicit_relocalization_fast_max_duration_sec`: force-accepted explicit Isaac relocalization keeps smoothing enabled, but large business corrections use a per-correction active rate sized to finish within the configured duration. AMCL gated corrections and ordinary online corrections continue to use the normal smoothing rates.
- `map_odom_large_correction_translation_m`, `map_odom_large_correction_yaw_rad`, `map_odom_large_correction_requires_recovery`: online large-correction policy metadata and recovery contract
- `map_odom_online_hard_reject_translation_m`, `map_odom_online_hard_reject_yaw_rad`: hard reject thresholds for non-forced online corrections
- `tf_future_stamp_offset_sec`: optional future-dating offset; Jetson runtime keeps this at `0.0` so TF timestamps remain measurement-time truthful
- `two_d_mode`: defaults to `true`
- `continuous_localization_mode`: legacy compatibility parameter. Phase A2 supports `triggered` only; any other value is ignored and reset to `triggered`.
- `status_topic`: JSON status output, defaults to `/localization/bridge_status`
- `triggered_max_result_age_ms`: bounded Isaac grid-search latency gate for service-triggered `/localization_result`; defaults to `5000.0`
- `force_accept_min_pose_stamp_slack_sec`: when `force_accept_service` is armed, ignore `/localization_result` messages whose header stamp predates the force-accept request by more than this slack instead of counting them as rejected stale results
- `max_odom_tf_age_ms`: freshness gate for the latest `odom -> base_link`, while candidate correction lookup uses `odom -> base_link` at the localization result stamp
- `triggered_allow_large_correction`: keeps explicit trigger relocalization eligible for the force-accept path; normal triggered updates still obey jump gating
- `amcl_input_enabled`: defaults to `false`; enables `/amcl_pose` as a continuous candidate input only when `NJRH_AMCL_LOCALIZATION_MODE` is `shadow` or `gated`
- `amcl_pose_topic`: defaults to `/amcl_pose`
- `amcl_runtime_status_file`: defaults to `/tmp/njrh_amcl_runtime_status.env`; read-only AMCL runtime contract exported by `run_amcl_shadow_localization.sh`
- `amcl_gate_mode`: `shadow` or `gated`; shadow records candidates only, gated accepts bounded AMCL corrections
- `amcl_max_result_age_ms`, `amcl_small_correction_translation_m`, `amcl_small_correction_yaw_rad`: AMCL-specific freshness and direct small-correction gates. Translation and yaw are the measured-versus-predicted `map -> base_link` pose innovation at the localization result timestamp, not raw `map -> odom` parameter deltas. The field profile directly accepts physical translation corrections up to `0.07 m` and yaw corrections up to `0.20 rad`.
- `amcl_medium_correction_translation_m`, `amcl_medium_correction_yaw_rad`, `amcl_medium_correction_consistency_count`: medium AMCL gate. The field profile accepts corrections up to `0.15 m` and `0.20 rad` only after 3 consecutive consistent candidates.
- `amcl_accept_corrections_while_moving`, `amcl_moving_linear_speed_mps`, `amcl_moving_angular_speed_radps`: field runtime accepts bounded AMCL corrections while moving in `gated` mode, so navigation can continuously correct `map -> odom`. Set `NJRH_AMCL_ACCEPT_CORRECTIONS_WHILE_MOVING=false` or `NJRH_AMCL_LOCALIZATION_MODE=shadow` for odometry audit / observe-only rollback.
- `amcl_hard_reject_translation_m`, `amcl_hard_reject_yaw_rad`: hard reject / Isaac recovery gate. The field profile hard-rejects AMCL translation corrections above `0.30 m` or yaw corrections above `0.8 rad`.
- `amcl_max_xy_covariance`, `amcl_max_yaw_covariance`: covariance gates for AMCL pose input
- `amcl_post_isaac_refine_*`: after a force-accepted Isaac relocalization seeds AMCL, gated AMCL may apply one stationary, short-window residual correction before the normal post-Isaac suppression resumes. The Isaac `map -> odom` target must finish smoothing first. A refine candidate must be received after the seed, carry a pose stamp newer than the seed, and pass `amcl_post_isaac_refine_min_delay_sec` (`0.25 s` in the field profile) before it can enter the `2`-candidate consistency gate. Because AMCL is event-driven and may stay silent while the robot is stationary, the bridge then requests `/request_nomotion_update` every `0.5 s`, at most `4` times. Requests stop after refine acceptance, window expiry, or robot motion. This prevents queued pre-seed callbacks from replacing the active Isaac target while still giving the new particle generation a bounded chance to publish. The field profile allows one correction within `10.0 s`, capped at `0.12 m` and `0.10 rad`, after candidates that agree within `0.08 m` and `0.08 rad`.
- `amcl_seed_service`: defaults to `/robot_localization_bridge/seed_amcl_initial_pose` and publishes `/initialpose` from the current reliable `map -> base_link`

## Correction Pause Ownership

The owner-scoped service and the legacy Bool service share one internal
arbiter. Effective pause is the logical OR of all active records. While paused,
new Isaac/AMCL correction candidates are rejected, but the bridge continues
broadcasting the last accepted canonical `map -> odom`; pausing does not create
a second TF owner.

### Owner-scoped API

`SetCorrectionPause(OP_ACQUIRE)` requires a non-empty
`owner + transaction_id + reason`. Records from different transactions compose.
An exact repeated acquire is idempotent, and only the exact owner/transaction
can release its record. Reusing one transaction ID under another owner is a
conflict.

The owner-scoped pause currently has no TTL. It remains until exact release or
bridge process restart. Consequently an elevator recovery path must inspect
`CorrectionPauseState.lease_keys`; it must not guess that a failed task or a
destroyed client released its record.

### Legacy `SetBool` compatibility

The legacy endpoint is represented internally by the synthetic record
`legacy_set_bool:legacy_set_bool`:

- `data=true` acquires or idempotently keeps only that legacy record.
- `data=false` releases only that legacy record.
- If the legacy record is already absent, `data=false` is a successful no-op.
- A legacy `false` never removes an elevator, mission, docking, or other
  owner-scoped record.
- Releasing an owner-scoped record never changes the legacy record.

Callers must use the API through which they acquired the pause. New P6 code
must use `SetCorrectionPause`; it must not issue legacy `false` as a global
cleanup operation.

### Elevator and floor-switch rule

During elevator ride, the elevator transaction owns a correction-pause record
while `robot_safety` keeps the chassis stopped. A live target-floor
relocalization cannot succeed while any correction pause remains, because the
bridge intentionally rejects the new correction.

The release must be fenced inside the floor transition; releasing it before the
transition starts would briefly admit source-floor corrections while source
assets are still active. The required logical order is:

1. acquire/confirm the owner-scoped motion hold;
2. confirm the target-floor and door evidence;
3. begin the floor transition; the floor manager must first acquire its own
   owner-scoped pause and invalidate the ordinary source runtime context;
4. after that acknowledgement, release only the elevator-owned pause under
   hold. Effective pause remains true because the floor-manager record owns the
   transition fence;
5. run `FloorSwitch`, load and identify the target asset epoch, then release
   only the floor-manager pause at the explicit target-localization stage;
6. admit only target-epoch evidence, trigger and settle target-floor
   localization, then commit the target runtime context;
7. keep the hold and an invalid runtime context on every failure.

The current pure elevator FSM now emits
`BeginFloorTransition -> ResumeLocalizationCorrections -> SwitchFloor`, which
encodes this ownership handoff. The bridge now implements the typed
`BeginFloorTransition` BEGIN/COMMIT/ABORT fence and publishes typed floor
health. BEGIN requires the exact floor-manager pause and invalidates the source
runtime context. COMMIT requires every pause released, a newer explicit
localization sequence, and a settled/published `map -> odom`. ABORT keeps the
context invalid as `FAILED_LOCKED`; force-accept cannot bypass that lock.
Production BEGIN/COMMIT remain gated off by
`live_floor_transition_service_enabled=false`, so a direct ROS client cannot
bypass the preflight-only FloorManager and invalidate runtime context.

This remains a non-production seam because the floor-manager Action is still
preflight-only and is not connected to the bridge service or the real asset and
costmap adapters. The typed health deliberately reports
`localizer_ready=false`, `tf_unique=false`, and generation zero until another
authoritative component can prove them; consumers must not reinterpret these
conservative fields as success.

If a legacy or unrelated owner still holds correction pause, floor switching
must fail locked. It must not clear that record or release motion. After the
physical elevator has moved, a partially applied target-floor transaction must
not automatically roll back to the source-floor map and claim a valid runtime
context.

### Validation

The pure arbiter tests in `test/test_correction_pause_arbiter.cpp` cover
multi-owner composition, exact-owner release, idempotence, transaction
collision, and rejected malformed requests.

The only recommended command for the isolated correction-pause ROS smoke is:

```bash
bash /workspaces/njrh-v3/workspace1/src/robot_localization_bridge/test/run_isolated_correction_pause_smoke.sh
```

Do not invoke `isolated_correction_pause_smoke.py` directly. The wrapper fixes
`ROS_DOMAIN_ID=182`, starts an isolated bridge with TF publication and AMCL
input disabled, and runs the Python assertions against that process. Its
`EXIT` trap terminates and waits for the bridge and removes the temporary log
on success, failure, or interruption. Never run this test against the live
navigation process.

The smoke acquires an elevator record, rejects the reserved legacy identifiers,
sends legacy `false`, verifies the elevator record and effective pause remain,
then releases the exact elevator record.

The cross-package `robot_elevator_manager` non-moving scenario also composes
this pure arbiter with the elevator and floor-transition cores to verify the
pause handoff leaves no residual record. It does not instantiate this ROS node,
reload a real floor asset, publish TF, or authorize robot motion.

The isolated floor-transition fence smoke is:

```bash
bash /workspaces/njrh-v3/workspace1/src/robot_localization_bridge/test/run_isolated_floor_transition_smoke.sh
bash /workspaces/njrh-v3/workspace1/src/robot_localization_bridge/test/run_disabled_floor_transition_smoke.sh
```

The disabled-gate smoke proves the production default rejects BEGIN without
invalidating runtime context. The opt-in transaction smoke uses a separate ROS
domain, proves BEGIN rejects a missing exact pause,
proves BEGIN invalidates the runtime context, checks typed health does not
overstate localizer/TF readiness, then proves ABORT enters `FAILED_LOCKED` and
force-accept remains rejected. Its wrapper terminates the probe and bridge on
every exit path; never point it at the live navigation process.

## TF Contract

- Sole publisher of `map -> odom`
- Consumes `robot_global_localization` pose and `robot_local_state` odometry
- The current implementation is C++, computes `map -> odom` from planar `map -> base_link` and the TF `odom -> base_link` at the localization result stamp, latches one-shot localization results, republishes at the configured rate, and rejects large jumps
- Correction handling and TF broadcasting are separate. Isaac explicit relocalization, AMCL gated corrections, and manual force-accept update a locked `MapOdomState`; the independent publisher callback group is the only code path that calls `sendTransform()`. Correction pause rejects new global corrections but keeps broadcasting the last accepted `map -> odom`.
- Explicit business relocalization calls, such as startup, floor switch, manual recovery, localization-degraded recovery, and post-undock recovery, arm `force_accept_service` first. Ordinary point-navigation goals and default predock docking do not arm force-accept in their normal paths.
- Each accepted explicit Isaac force-accept relocalization increments `last_explicit_relocalization_sequence` on `/localization/bridge_status` and records `last_explicit_relocalization_accept_time` plus `last_explicit_relocalization_source`. AMCL small/medium gated corrections do not increment this sequence, so runtime settle barriers are triggered only by business relocalization, not every continuous AMCL correction.
- Field runtime publishes at 50 Hz with `tf_future_stamp_offset_sec=0.0`, so the canonical transform remains measurement-time truthful; this does not change the canonical TF owner.
- AMCL is a continuous candidate source only. It must run with `tf_broadcast=false`; this bridge computes AMCL candidates from `/amcl_pose` and historical `odom -> base_link`.
- Candidate gating first predicts `map -> base_link` as `current(map -> odom) * odom -> base_link(t)` and compares that pose with the AMCL/Isaac measurement at the same timestamp. Only this robot-pose innovation drives small/medium/hard gates. The separately reported `map_odom_parameter_*` values are diagnostic transform-parameter changes; they can be much larger because a yaw correction at odom radius `r` requires approximately `r * delta_yaw` of compensating `map -> odom` translation.
- Isaac triggered relocalization has the highest priority and can seed AMCL through `/initialpose`. While its correction is active, all AMCL target changes are held. Once the Isaac target is current, at most one genuinely post-seed stationary AMCL refine correction may remove a small scan-map residual. If navigation starts first, the static refine generation is abandoned and normal gated AMCL resumes; the refine state cannot block moving corrections. Outside that short refine window, AMCL gated corrections directly accept small covariance-gated updates, accept medium corrections only after consecutive consistency, and report large corrections for Isaac recovery instead of applying one-frame TF jumps.
- `/localization/bridge_status` reports `gate_mode`, result age and gate limit, force-accept arm time, pre-arm ignored result count/reason, original-stamp TF lookup state, latest odom TF freshness, accept/reject reasons, triggered/AMCL counters, `active_correction_source`, `last_accepted_source`, `last_rejected_source`, `last_explicit_relocalization_sequence`, `last_explicit_relocalization_accept_time`, `last_accepted_correction_translation_m`, `last_accepted_correction_yaw_rad`, `has_map_to_odom`, `map_to_odom_age_ms`, and `map_to_odom_publisher_owner`. `correction_metric_frame=map_base_link` makes the gate metric explicit; signed `*_correction_dx_map_m`, `*_dy_map_m`, and `*_dyaw_rad` expose the physical innovation, while `*_map_odom_parameter_translation_m` and `*_map_odom_parameter_yaw_rad` expose the transform representation separately. It also reports `map_odom_publish_loop_hz`, `map_odom_publish_gap_ms`, `map_odom_publish_gap_max_ms`, `map_odom_publish_callback_duration_us`, `map_odom_current_sequence`, `map_odom_target_sequence`, `map_odom_last_accepted_sequence`, `map_odom_last_published_sequence`, `map_odom_current_source`, `map_odom_target_source`, physical `remaining_translation_error_m` / `remaining_yaw_error_rad`, parameter-space `remaining_map_odom_parameter_*`, `smoothing_total_duration_sec`, `smoothing_remaining_duration_sec`, `smoothing_progress`, `last_step_translation_m`, `last_step_yaw_rad`, `smoothing_policy`, active and configured smoothing rates, `smoothing_enabled`, `correction_active`, `safe_for_goal_start`, `large_correction_rejected_count`, `online_correction_smoothed_count`, `online_correction_snap_count`, `map_odom_correction_paused`, `map_odom_frozen_due_to_pause`, `map_odom_publish_missed_count`, and `publisher_decoupled_from_correction=true`. Post-Isaac diagnostics include the no-motion service readiness, per-sequence and total request counts, request sequence/time, and request state. Default `localization_settle_*` fields remain for compatibility; the live settle barrier is owned by `robot_api_server`.
- AMCL bridge readiness also uses the runtime status file. If AMCL input is enabled, `amcl_ready` cannot become true when the AMCL process, lifecycle, `/amcl_pose` publisher, scan-admission process, or `/amcl_scan_admission/status` publisher is missing. In gated mode, `amcl_ready=true` means AMCL is seeded and tracking-ready for startup; `amcl_correction_ready=true` remains the stricter signal that a fresh correction can be applied. A stationary seeded robot may therefore report `amcl_correction_pending=true` without `localization_degraded=true`. The status JSON exposes `amcl_state`, `amcl_process_alive`, `amcl_lifecycle_active`, `amcl_scan_admission_alive`, `amcl_pose_publisher_count`, `amcl_scan_admission_status_publisher_count`, `amcl_upstream_missing`, `amcl_correction_pending`, `localization_degraded`, and `amcl_degraded_reason`.
