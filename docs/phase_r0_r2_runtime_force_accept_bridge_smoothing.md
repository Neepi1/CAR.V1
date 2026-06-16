# Phase R0-R2 Runtime Force-Accept Reduction And Bridge Smoothing

## Scope

Phase R0-R2 reduces runtime complexity on the normal navigation and docking paths.

- R0 writes the read-only audit report:
  `reports/runtime_force_accept_and_bridge_smoothing_audit_20260615T204650Z.md`
- R1 removes implicit force-accept / Isaac relocalization from normal point-navigation goals and default predock docking.
- R2 changes `robot_localization_bridge` from immediate accepted-correction output replacement to current/target `map -> odom` smoothing.

This phase does not change Nav2 planner/controller plugins, MPPI/progress-checker parameters, TF tolerances, `max_odom_tf_age_ms`, pointcloud QoS/DDS, JT128, FAST-LIO2, Ranger odom, EKF, or the safety speed chain.

## Normal Navigation

`POST /api/v1/navigation/goal` no longer calls these in the normal goal handler:

- `/global_localization/trigger`
- `/robot_localization_bridge/force_accept_next_localization`
- `/localization_result` wait
- `wait_for_post_relocalization_settle_barrier()`

The normal path still performs dock/contact admission, Nav2 lifecycle admission, and bridge status admission. If a request asks for `force_relocalize=true`, or bridge status reports a goal-start-blocking degraded localization state or an active correction transition, the API returns recovery-required detail instead of injecting a relocalization before `NavigateToPose`. AMCL static standby while the robot is stopped is not a goal-start blocker when `map -> odom` is live and bridge `safe_for_goal_start=true`.

Relevant status fields:

- `navigation_normal_path_relocalization_enabled=false`
- `force_accept_allowed_in_normal_path=false`
- `ordinary_navigation_triggered_relocalization=false`
- `localization_recovery_available=true`
- `localization_recovery_required`
- `removed_redundant_gates`

## Docking Normal Path

Default predock relocalization is disabled:

- `docking_relocalize_before_predock: false`
- `docking_relocalize_after_predock: false`
- `docking_relocalize_after_predock_required: false`
- `docking_relocalize_after_fine_docking: false`

Docking still uses the saved predock pose, Nav2 staging, predock pose verification, predock yaw alignment, GS2 freshness, and global correction pause before fine docking. Explicit localization recovery remains available, but it is not mixed into the default short predock travel path.

Relevant status fields:

- `docking_normal_path_relocalization_enabled=false`
- `docking_predock_triggered_relocalization=false`

## Bridge Smoothing

`robot_localization_bridge` remains the only `map -> odom` publisher. Accepted candidates now update a target transform. The 50 Hz publisher advances a current transform toward that target using bounded translation and yaw slew rates.

Default parameters:

- `map_odom_smoothing_enabled: true`
- `map_odom_smoothing_publish_rate_hz: 50.0`
- `map_odom_smoothing_translation_rate_mps: 0.20`
- `map_odom_smoothing_yaw_rate_radps: 0.25`
- `map_odom_smoothing_snap_translation_epsilon_m: 0.005`
- `map_odom_smoothing_snap_yaw_epsilon_rad: 0.005`
- `map_odom_large_correction_translation_m: 0.50`
- `map_odom_large_correction_yaw_rad: 0.35`
- `map_odom_large_correction_requires_recovery: true`
- `map_odom_online_hard_reject_translation_m: 0.80`
- `map_odom_online_hard_reject_yaw_rad: 0.80`

Bridge status now exposes current/target sequence, source, remaining translation/yaw error, last smoothing step, correction-active state, `safe_for_goal_start`, and large-correction rejection counters.

## Verification

Static/read-only contract checks:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_runtime_force_accept_reduction.sh --expect-pass
bash scripts/jetson/runtime_overlay/scripts/verify_bridge_map_odom_smoothing.sh --duration-sec 120 --expect-pass
```

Observe one normal App navigation without sending goals from the script:

```bash
bash scripts/jetson/runtime_overlay/scripts/observe_normal_navigation_minimal_path.sh \
  --duration-sec 180 \
  --label normal_nav_r0_r2
```

## Rollback

Set the changed defaults back and restart the affected services:

- `navigation_relocalize_before_goal: true`
- `navigation_relocalize_before_goal_required: true`
- `docking_relocalize_before_predock: true`
- `docking_relocalize_after_predock: true`
- `docking_relocalize_after_predock_required: true`
- `docking_relocalize_after_fine_docking: true`
- `map_odom_smoothing_enabled: false`

Rollback should be used only for controlled A/B diagnosis because it restores implicit force-accept timing near Nav2 `FollowPath`.
