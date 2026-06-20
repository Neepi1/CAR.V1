# Phase N5 Single Nav2 Completion Owner

Phase N5 removes the ordinary-navigation double completion judge. For normal
`pose_required` goals, Nav2 owns XY plus yaw completion before the
`NavigateToPose` action can succeed. `robot_api_server` owns business
admission, cancellation, state reporting, and final pose audit only.

Runtime sequence for normal goals:

```text
App goal
-> robot_api_server dock/contact/localization/Nav2 readiness gates
-> NavigateToPose(x/y/yaw)
-> RotationShimController + MPPI + SimpleGoalChecker(stateful=false)
-> Nav2 action result
-> if SUCCEEDED: API task_complete=true and records final pose audit fields
-> if ABORTED/CANCELED/TIMEOUT: API reports the Nav2/action failure
```

Key contracts:

- `goal_checker.stateful=false`, so XY remains checked while terminal yaw is
  being satisfied.
- `FollowPath.rotate_to_goal_heading=true`, so Nav2 owns terminal yaw.
- Ordinary `api_final_yaw_align_fallback_enabled=false`.
- Ordinary post-Nav2 bridge wait, same-goal retry, and acceptance slack are
  disabled by default.
- API final pose audit records `final_distance_m`, `final_yaw_error_rad`,
  `final_verify_xy_error_m`, `final_verify_yaw_error_rad`, and
  `final_pose_verify_reason`; it does not turn a Nav2 success into
  `failed_final_pose_verify`.

Docking predock remains stricter than ordinary delivery handoff. Predock can
still reject the handoff if its docking-specific pose/yaw/bridge/GS2 gates are
not satisfied, but ordinary point navigation no longer runs API-owned velocity
recovery after Nav2 success.

Validation:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_nav2_native_goal_completion.sh
bash scripts/jetson/runtime_overlay/scripts/verify_post_nav2_final_verify_recovery.sh \
  --mock-nav2-succeeded \
  --mock-final-distance 0.269 \
  --mock-yaw-error 0.018 \
  --mock-tolerance 0.2 \
  --expect-task-complete
```

This phase does not change Nav2 planner/controller plugin types, MPPI critics,
costmap source policy, TF tolerances, `max_odom_tf_age_ms`, AMCL/Isaac/bridge
correction policy, pointcloud QoS/DDS, FAST-LIO2, Ranger odom, EKF, or the
`robot_safety` speed chain.
