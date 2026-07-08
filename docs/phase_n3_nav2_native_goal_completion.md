# Phase N3: Nav2 Native Goal Completion

Pose-required navigation uses Nav2 native yaw completion as the primary terminal heading controller. For `goal_completion_policy=pose_required`, the target yaw is part of the Nav2 `NavigateToPose` goal, and Nav2 should complete XY plus yaw before the action succeeds. Ordinary stored `delivery_point` goals default to `pose_required`. Explicit `position_only` requests are reserved for diagnostics or special XY-only targets; for those requests the API sends Nav2 the saved XY with an approach-heading yaw so Nav2 is not asked to spend terminal time turning toward a saved pose yaw that the request explicitly declared irrelevant. Phase N5 keeps Nav2 as the primary pose-required completion owner, but enables one bounded API `final_yaw_align` fallback when Nav2 aborts or returns with the robot already inside the yaw-alignable XY window and heading still outside tolerance.

## Runtime Contract

- `FollowPath` uses `nav2_rotation_shim_controller::RotationShimController`.
- `FollowPath.primary_controller` remains `nav2_mppi_controller::MPPIController`; existing MPPI tuning parameters stay under `FollowPath`.
- `FollowPath.rotate_to_goal_heading=true`.
- `FollowPath.rotate_to_heading_angular_vel=0.60` and
  `FollowPath.max_angular_accel=1.2` keep RotationShim pure-yaw commands inside
  the Ranger Mini 3 range that has not shown terminal spin overrun in field
  tests.
- `goal_checker` remains `nav2_controller::SimpleGoalChecker`.
- `goal_checker.stateful=false`, so terminal XY and yaw are rechecked together after dynamic-obstacle interruption or near-goal avoidance drift.
- Current commercial ordinary navigation uses `xy_goal_tolerance=0.06` and `yaw_goal_tolerance=0.05`; Nav2 pose-required completion is followed by API commercial final verification and bounded retry/degraded handling.
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

Explicit position-only navigation:

```text
API receives goal_completion_policy=position_only
-> API sends NavigateToPose with saved x/y and approach-heading yaw
-> Nav2 completes XY without a forced terminal turn toward the saved pose yaw
-> API verifies fresh map->base_link XY
-> task_complete=true when final_pose_verified=true
```

If the Nav2 action fails before the robot is near the target, the API records `phase=nav2_failed`. If Nav2 aborts after getting near the target but still outside the yaw-alignable XY gate, `navigation_nav2_failed_near_goal_retry_enabled=true` allows a yaw-first recovery inside the configured near-goal window, default `0.35m`: the API enters `nav2_failed_near_goal_yaw_aligning`, aligns heading once through `/cmd_vel_api`, then sends one same-goal Nav2 retry so Nav2 can close XY plus yaw instead of leaving a 20cm terminal miss. If Nav2 fails or succeeds while the robot is close enough for safe heading correction and yaw remains outside the trigger threshold, the API enters `nav2_failed_yaw_aligning` or `position_reached_yaw_aligning` and runs one bounded `final_yaw_align` through `/cmd_vel_api -> robot_safety -> /cmd_vel`. Before any bounded yaw fallback starts, `robot_api_server` waits for `robot_localization_bridge.safe_for_goal_start=true` so an already accepted `map->odom` smoothing update is not still moving underneath the spin controller. During the spin, it pauses bridge global correction intake, so AMCL/Isaac candidates can still run but cannot update `map->odom` until the final spin exits. The API does not publish to collision_monitor's `/cmd_vel_collision_checked`, does not use `/cmd_vel_docking`, and does not let the App publish chassis velocity.

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
