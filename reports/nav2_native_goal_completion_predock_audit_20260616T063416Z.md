# Phase N3-D Nav2 Native Predock Completion Audit

Timestamp: 2026-06-16T06:34:16Z

## Scope

Phase N3-D reuses the Phase N3 ordinary navigation contract:

- `FollowPath` is `nav2_rotation_shim_controller::RotationShimController`.
- `FollowPath.primary_controller` remains `nav2_mppi_controller::MPPIController`.
- `FollowPath.rotate_to_goal_heading=true`.
- `goal_checker` remains `nav2_controller::SimpleGoalChecker`.
- `goal_checker.stateful=true`.
- `xy_goal_tolerance=0.20` and `yaw_goal_tolerance=0.15` are unchanged.
- RPP remains as `FollowPathFallback`.

No Nav2 planner, MPPI tuning, progress checker, transform tolerance, bridge gate,
AMCL/Isaac strategy, pointcloud QoS/DDS, FAST-LIO2, Ranger odom, EKF, or speed
chain change is part of this phase.

## Audit Table

| Area | Current Finding | N3-D Decision |
| --- | --- | --- |
| RotationShim support | Existing N3 config already uses `nav2_rotation_shim_controller::RotationShimController`. | Reuse as-is. |
| `rotate_to_goal_heading` | Already `true` in repo and runtime overlay `nav2.yaml`. | Reuse as-is. |
| Primary controller | Existing MPPI remains under `FollowPath.primary_controller`. | Reuse as-is. |
| SimpleGoalChecker | Existing `stateful=true`, `xy_goal_tolerance=0.20`, `yaw_goal_tolerance=0.15`. | Reuse as-is. |
| Ordinary API final yaw | `api_final_yaw_align_fallback_enabled=false` and `navigation_final_yaw_align_enable=false`. | Reuse as-is. |
| Docking predock yaw source | Predock goal yaw is `job.approach_yaw`; manual predock poses use the saved yaw, computed poses use saved dock yaw. | Keep math unchanged, expose `expected_base_yaw_at_predock`, `dock_insertion_yaw_map`, and `reverse_yaw_offset_applied` so semantics are no longer implicit. |
| Docking predock normal path | Previously Nav2 success could be followed by default `PREDOCK_YAW_ALIGN` on `/cmd_vel_docking`. | Change default to native Nav2 XY+yaw completion plus read-only `PREDOCK_POSE_VERIFY`. |
| Predock yaw fallback | Existing code can publish `/cmd_vel_docking` and checks actual SPINNING mode. | Retain only behind explicit `predock_yaw_align_enabled=true` and `predock_yaw_align_fallback_enabled=true`. Defaults are false. |
| Fine docking entry | Existing gate required `dock_staging_handoff_ready`, yaw alignment, GS2 freshness, correction pause, distance/lateral/yaw limits. | Add explicit `predock_pose_verified=true` requirement before `/docking/start`. |
| Status observability | Existing docking JSON exposed `predock_expected_*` and `predock_current_*` fields. | Add clearer aliases: `expected_base_yaw_at_predock`, `expected_contact_yaw_at_predock`, `current_base_yaw_map`, `current_contact_yaw_map`, `base_yaw_error`, `contact_yaw_error`, `reverse_yaw_offset_applied`, `contact_frame_available`, and `predock_yaw_verified_by_nav2`. |

## Failure Semantics

If Nav2 predock action succeeds but read-only `PREDOCK_POSE_VERIFY` fails:

- `/docking/start` is not called.
- Default path does not publish `/cmd_vel_docking`.
- Failure code is `PREDOCK_NATIVE_GOAL_VERIFY_FAILED` for pose/XY failures.
- Failure code is `PREDOCK_YAW_NOT_ALIGNED_AFTER_NAV2` for yaw failures.

`PREDOCK_YAW_ALIGN_RECOVERY` is available only as explicit fallback/recovery.

## Hardware Validation Still Required

- Run a return-to-dock attempt with
  `scripts/jetson/runtime_overlay/scripts/observe_docking_predock_native_nav2.sh --duration-sec 240 --label dock_n3d_1`.
- Confirm `nav_goal_succeeded=true`, `predock_pose_verified=true`, and
  `predock_yaw_verified_by_nav2=true` before `FINE_DOCKING_ENTRY_CHECK`.
- Confirm `predock_yaw_align_attempted=false` on the normal path.
- If predock verify fails, confirm no `/docking/start` call occurs and the failure
  code is `PREDOCK_NATIVE_GOAL_VERIFY_FAILED` or `PREDOCK_YAW_NOT_ALIGNED_AFTER_NAV2`.
