# Phase N3: Nav2 Native Goal Completion

Ordinary delivery goals use Nav2 native yaw completion as the terminal heading controller. For `goal_completion_policy=pose_required`, the target yaw is part of the Nav2 `NavigateToPose` goal, and Nav2 must complete XY plus yaw before the action succeeds. Phase N5 makes Nav2 the single ordinary completion owner; API-owned `final_yaw_align` is disabled by default and retained only as legacy emergency code.

## Runtime Contract

- `FollowPath` uses `nav2_rotation_shim_controller::RotationShimController`.
- `FollowPath.primary_controller` remains `nav2_mppi_controller::MPPIController`; existing MPPI tuning parameters stay under `FollowPath`.
- `FollowPath.rotate_to_goal_heading=true`.
- `goal_checker` remains `nav2_controller::SimpleGoalChecker`.
- `goal_checker.stateful=false`, so XY remains checked while terminal yaw is being satisfied.
- `xy_goal_tolerance=0.20` and `yaw_goal_tolerance=0.0873` keep Nav2 terminal heading within about 5 degrees before action success.
- `FollowPathFallback` remains the RPP fallback entry and is not removed.

## API Semantics

Normal `pose_required` navigation:

```text
API sends NavigateToPose with x/y/yaw
-> Nav2 RotationShim + MPPI + SimpleGoalChecker completes XY and yaw
-> API receives Nav2 action result
-> API waits for bridge smoothing and performs final map->base_link verification
-> task_complete=true only when final_pose_verified=true
```

If the Nav2 action fails, the API records `phase=nav2_failed`. If Nav2 succeeds, the API records a final pose audit but does not retry the same goal, does not run ordinary API `final_yaw_align`, does not reposition after yaw drift, and does not convert the Nav2 success into `failed_final_pose_verify`. Any residual correction must occur inside the Nav2 action before success.

Phase N4 extends this boundary: after Nav2 success, the API waits for bridge
`map->odom` smoothing before final verification. If a recoverable AMCL/bridge
correction leaves a small XY or yaw residual, the API resends the same
`NavigateToPose` goal once. If only yaw remains outside tolerance after that
retry, API final yaw alignment is allowed as a bounded residual correction.

## Docking Predock Boundary

`dock_staging` is not ordinary `pose_required` delivery navigation, but it uses the same native Nav2 XY+yaw completion mechanism for the staging pose. Docking passes `expected_base_yaw_at_predock` to `NavigateToPose`, and fine docking requires verification after Nav2 action success and after bridge smoothing has settled. The API predock/fine-entry yaw gate is `0.105 rad`, about 6 degrees, so Nav2 aims tighter than the API handoff gate:

```text
Nav2 NavigateToPose(predock x/y/expected_base_yaw_at_predock)
-> Nav2 RotationShim + MPPI + SimpleGoalChecker completes XY and yaw
-> STAGING_NAV2_GOAL_SUCCEEDED
PREDOCK_POSE_VERIFY
-> FINE_DOCKING_BRIDGE_SETTLE
-> PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE
-> PREDOCK_YAW_ALIGN_AFTER_BRIDGE_SETTLE if needed
-> FINE_DOCKING_ENTRY_CHECK
```

If `PREDOCK_POSE_VERIFY` fails, `/docking/start` is not called and the job fails with `PREDOCK_NATIVE_GOAL_VERIFY_FAILED` or `PREDOCK_YAW_NOT_ALIGNED_AFTER_NAV2`. If bridge `map->odom` smoothing is still active at the fine docking boundary, the API waits in `FINE_DOCKING_BRIDGE_SETTLE`; timeout fails with `DOCK_FAILED_FINE_LOCALIZATION_TRANSITION_TIMEOUT` before `/docking/start`. After smoothing, the API rechecks the staging pose; if yaw changed outside tolerance while XY remains valid, docking-owned yaw recovery uses `/cmd_vel_docking` through `robot_safety` before the fine-entry gate.

## Validation

Use:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_nav2_native_goal_completion.sh
bash scripts/jetson/runtime_overlay/scripts/observe_nav2_native_pose_required_goal.sh --duration-sec 180
```

The observer is read-only. It does not send goals, docking requests, relocalization requests, velocity commands, or pointcloud subscriptions.
