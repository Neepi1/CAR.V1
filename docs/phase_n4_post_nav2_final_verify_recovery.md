# Phase N4 Post-Nav2 Final Verify Recovery

Superseded by `docs/phase_n5_single_nav2_completion_owner.md` for production
ordinary navigation. N4 was a bounded field stop-gap that retried the same Nav2
goal and allowed a small post-retry XY slack. N5 disables that ordinary
post-Nav2 recovery path and makes Nav2 the only XY+yaw completion owner.

Phase N4 keeps ordinary navigation completion primarily owned by Nav2 native
`RotationShimController + SimpleGoalChecker`. API-owned `final_yaw_align` is
enabled only as a bounded post-bridge residual yaw correction after Nav2 success
and one same-goal retry.

Runtime sequence for normal `pose_required` goals:

```text
NavigateToPose(x/y/yaw)
-> Nav2 action result
-> wait for bridge map->odom smoothing before final verify
-> final_pose_verify
-> if verified: task_complete=true
-> if recoverable XY/yaw error: resend the same NavigateToPose goal once
-> if XY is only slightly outside tolerance after retry: accept within post_nav2_final_verify_acceptance_slack_m
-> if only yaw remains outside tolerance: API final_yaw_align through safety chain
-> final verification
```

The bridge wait is bounded by
`post_nav2_final_verify_bridge_wait_timeout_ms` and defaults to `2000ms`. A
timeout is recorded in `/api/v1/navigation/state` and final verification still
runs against the latest available `map -> base_link` pose.

Retry policy:

- `post_nav2_final_verify_max_retry_count: 1`
- post-retry XY acceptance slack: `0.03m`
- XY retry range: `0.20m..0.60m`
- yaw retry: enabled for `pose_required`
- retry target: the same Nav2 goal, including the original target yaw
- API velocity correction: disabled for XY; final yaw fallback enabled for residual yaw

If the retry succeeds and final verification passes, `task_complete=true`. If
the retry is exhausted and the remaining XY error is within the small
post-Nav2 slack, ordinary delivery is accepted while preserving the measured
`final_verify_xy_error_m` for field logs. If the retry is exhausted, disabled,
rejected, canceled, or the final error remains outside tolerance plus slack, the
goal enters terminal `failed_final_pose_verify` with `task_complete=false`; it
must not continue to show as running.

State fields exposed on `navigation_goal`:

- `post_nav2_final_verify_enabled`
- `post_nav2_final_verify_wait_bridge_smoothing`
- `post_nav2_final_verify_bridge_wait_elapsed_ms`
- `post_nav2_final_verify_bridge_wait_timeout`
- `final_verify_retry_count`
- `final_verify_retry_reason`
- `final_verify_retry_max_count`
- `final_verify_retry_goal_sent`
- `final_verify_xy_error_m`
- `final_verify_yaw_error_rad`
- `final_verify_failure_is_terminal`
- `api_velocity_correction_enabled`

Validation:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_post_nav2_final_verify_recovery.sh \
  --mock-nav2-succeeded \
  --mock-final-distance 0.269 \
  --mock-yaw-error 0.018 \
  --mock-tolerance 0.2 \
  --expect-retry-xy

bash scripts/jetson/runtime_overlay/scripts/observe_post_nav2_final_verify_recovery.sh \
  --duration-sec 180
```

This phase does not change Nav2 planner/controller plugins, MPPI, progress
checker, TF tolerances, `max_odom_tf_age_ms`, AMCL/Isaac/bridge correction
policy, pointcloud QoS/DDS, FAST-LIO2, Ranger odom, EKF, or the `robot_safety`
speed chain.
