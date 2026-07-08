# Phase N2 Goal Completion Semantics

Phase N2 separates three concepts that were previously easy to mix together:

- `position_reached`: the robot's fresh `map -> base_link` pose is within the XY tolerance.
- `final_pose_verified`: the completion policy has been satisfied.
- `task_complete`: the API may report the business task as complete.

## Policies

`goal_completion_policy` is exposed in `/api/v1/navigation/state` and accepted on normal navigation requests when appropriate.

- `pose_required`: default for ordinary stored `delivery_point` poses, direct coordinate goals, and stored pose types that explicitly require heading (`delivery_pose_required`, `precise_pose`, `orientation_required`, `require_yaw`). Phase N3 makes Nav2 native `RotationShimController + SimpleGoalChecker` the primary XY/yaw completion path. The API then audits a fresh final pose; if Nav2 failed near the target but outside the yaw-alignable XY window, one same-goal Nav2 retry may run first. If Nav2 failed or returned with the robot already inside the yaw-alignable XY window but yaw remains outside tolerance, one bounded API `final_yaw_align` fallback may run through `/cmd_vel_api -> robot_safety -> /cmd_vel`. That fallback first waits for bridge smoothing to settle, then pauses `robot_localization_bridge` correction intake, so AMCL/Isaac cannot update `map->odom` during the final spin window.
- `position_only`: explicit opt-out for diagnostics or special XY-only targets where final heading is not a business requirement. A fresh XY verification completes the goal. The API does not run mission-layer `final_yaw_align`. For position-only requests, the Nav2 action goal keeps the saved XY but uses the current approach heading as its yaw by default, so Nav2 is not asked to perform an unnecessary terminal turn toward a stale saved pose yaw.
- `dock_staging`: reserved for `/api/v1/docking/start`. The predock Nav2 result must complete XY plus yaw before GS2 fine docking can start. Ordinary `final_yaw_align` is forbidden; the API performs read-only `PREDOCK_POSE_VERIFY`, and `PREDOCK_YAW_ALIGN_RECOVERY` is only an explicit fallback.

## App State Rules

The App should show arrival success only when `task_complete=true`.

- During `position_reached_yaw_aligning`, show a heading-alignment state, not task success.
- If a legacy emergency fallback explicitly enables `REPOSITION_AFTER_YAW_DRIFT`, show a short reposition/reverify state. It is not part of the default ordinary navigation path.
- For `position_only`, `position_reached=true` and `final_pose_verified=true` can happen immediately after Nav2 result verification.
- For `pose_required`, `position_reached=true` alone is not enough.

The navigation goal JSON includes:

- `goal_completion_policy`
- `nav2_goal_yaw_rad`
- `nav2_goal_yaw_source`
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

- Ordinary `final_yaw_align` publishes through `/cmd_vel_api -> robot_safety -> /cmd_vel` in production and must not share collision_monitor's `/cmd_vel_collision_checked` publisher.
- Explicit `PREDOCK_YAW_ALIGN_RECOVERY` must publish only to `/cmd_vel_docking`.
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
- `predock_pose_verified == true`
- `predock_yaw_aligned == true`
- post-predock handoff settle is complete; post-predock relocalization is only part of this gate when explicitly enabled
- GS2 scan is fresh
- bridge `map->odom` smoothing has settled in `FINE_DOCKING_BRIDGE_SETTLE`
- global correction pause is applied

If fine-entry yaw is too large, the state machine fails by default instead of calling GS2 fine docking directly. It may enter `PREDOCK_YAW_ALIGN_RECOVERY` only when the explicit fallback config is enabled.

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

Phase V1 adds the current combined validation path for normal delivery
semantics. It observes an already-running navigation and checks that
`pose_required` completion is not reported until `task_complete=true`:

```bash
bash scripts/jetson/runtime_overlay/scripts/observe_pose_required_navigation.sh \
  --duration-sec 180

bash scripts/jetson/runtime_overlay/scripts/run_v1_navigation_docking_validation.sh \
  --observe-only \
  --duration-sec 120
```

Rollback is a code/config revert of Phase N2 files. This phase does not change Nav2 controller/planner plugins, MPPI, progress checker, TF tolerances, pointcloud QoS/DDS, FAST-LIO2, Ranger odom, EKF, or the `robot_safety` speed chain.
