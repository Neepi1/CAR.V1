# Phase N2 Goal Completion Semantics

Phase N2 separates three concepts that were previously easy to mix together:

- `position_reached`: the robot's fresh `map -> base_link` pose is within the XY tolerance.
- `final_pose_verified`: the completion policy has been satisfied.
- `task_complete`: the API may report the business task as complete.

## Policies

`goal_completion_policy` is exposed in `/api/v1/navigation/state` and accepted on normal navigation requests when appropriate.

- `pose_required`: default for normal delivery points. The API may run mission-layer `final_yaw_align`, then succeeds only after final pose verification. If final yaw alignment exceeds the existing XY drift guard, the API may run one `REPOSITION_AFTER_YAW_DRIFT` retry to the original target.
- `position_only`: explicit opt-out for engineering cases where final heading is irrelevant. A fresh XY verification completes the goal. The API does not run mission-layer `final_yaw_align`.
- `dock_staging`: reserved for `/api/v1/docking/start`. A predock Nav2 result only means the robot reached the staging pose. Ordinary `final_yaw_align` is forbidden; docking owns yaw through `PREDOCK_POSE_VERIFY` and `PREDOCK_YAW_ALIGN`.

## App State Rules

The App should show arrival success only when `task_complete=true`.

- During `position_reached_yaw_aligning`, show a heading-alignment state, not task success.
- During `REPOSITION_AFTER_YAW_DRIFT`, show a short reposition/reverify state.
- For `position_only`, `position_reached=true` and `final_pose_verified=true` can happen immediately after Nav2 result verification.
- For `pose_required`, `position_reached=true` alone is not enough.

The navigation goal JSON includes:

- `goal_completion_policy`
- `position_reached`
- `yaw_align_required`
- `yaw_align_active`
- `yaw_align_succeeded`
- `yaw_align_failed`
- `final_pose_verified`
- `task_complete`
- `final_yaw_align_retry_count`
- `reposition_after_yaw_drift_retry_count`

## Command Ownership

Ordinary final yaw and docking predock yaw are separate owners.

- Ordinary `final_yaw_align` may publish only to `/cmd_vel_nav` or `/cmd_vel_collision_checked`.
- `PREDOCK_YAW_ALIGN` must publish to `/cmd_vel_docking`.
- The two owners must not be active at the same time.
- If docking starts while ordinary final yaw is active, the API requests cancellation, sends a zero burst on the ordinary final-yaw stream, waits briefly for the owner to release, and then lets docking proceed.

The API exposes:

- `ordinary_final_yaw_align_active`
- `predock_yaw_align_active`
- `cmd_owner_conflict_detected`
- `final_yaw_align_blocked_by_docking`
- `docking_blocked_by_final_yaw_align`

## Docking Gate

Fine docking is allowed only after all staging conditions are true:

- `goal_completion_policy == dock_staging`
- predock XY is verified
- `predock_yaw_aligned == true`
- post-predock handoff settle is complete; post-predock relocalization is only part of this gate when explicitly enabled
- GS2 scan is fresh
- global correction pause is applied

If fine-entry yaw is too large, the state machine retries `PREDOCK_YAW_ALIGN` or restaging according to the existing retry budget instead of calling GS2 fine docking directly.

## Verification

Static contract:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_goal_completion_semantics.sh
```

Live observation during a normal navigation:

```bash
bash scripts/jetson/runtime_overlay/scripts/observe_navigation_final_yaw_align.sh \
  --duration-sec 180 \
  --label nav_goal_1
```

Rollback is a code/config revert of Phase N2 files. This phase does not change Nav2 controller/planner plugins, MPPI, progress checker, TF tolerances, pointcloud QoS/DDS, FAST-LIO2, Ranger odom, EKF, or the `robot_safety` speed chain.
