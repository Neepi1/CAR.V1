# Phase N5 Single Nav2 Completion Owner

Phase N5 removes the ordinary-navigation double completion judge. For normal
`pose_required` goals, Nav2 owns the primary XY plus yaw completion before the
`NavigateToPose` action can succeed. `robot_api_server` owns business
admission, cancellation, state reporting, final pose audit, and one bounded
ordinary yaw fallback when Nav2 aborts or succeeds with the robot already close
enough for safe heading correction.

Phase N6 updates this contract for commercial operation: Nav2 action success is
no longer business completion by itself. The API now performs commercial final
verification, bounded same-goal retry, and degraded reporting after Nav2 result.

Runtime sequence for normal goals:

```text
App goal
-> robot_api_server dock/contact/localization/Nav2 readiness gates
-> NavigateToPose(x/y/yaw)
-> RotationShimController + MPPI + SimpleGoalChecker(stateful=false)
-> Nav2 action result
-> if close to target but yaw is still outside tolerance: API final_yaw_align
   through /cmd_vel_api -> robot_safety -> /cmd_vel
-> if commercially verified: API task_complete=true
-> if final verify overrun is bounded: same-goal retry/recovery
-> if final verify remains outside gate: state=degraded task_complete=false
-> if ABORTED/CANCELED/TIMEOUT: API reports the Nav2/action failure
```

Key contracts:

- `goal_checker.stateful=false`, so ordinary point navigation rechecks XY and
  yaw together after a near-goal obstacle or avoidance maneuver instead of
  accepting a latched first XY hit.
- Ordinary pose-required completion uses `xy_goal_tolerance=0.06` and
  `yaw_goal_tolerance=0.05`.
- `FollowPath.rotate_to_goal_heading=true`, so Nav2 owns primary terminal yaw.
- `FollowPath.rotate_to_heading_angular_vel=0.45` and
  `FollowPath.max_angular_accel=1.2` are the production heading-closure
  dynamics for Ranger Mini 3.
- Ordinary `api_final_yaw_align_fallback_enabled=true`, but only as a bounded
  fallback after Nav2 result when XY is inside the yaw-alignable window.
- If Nav2 aborts near the target but outside that yaw-alignable XY gate,
  `navigation_nav2_failed_near_goal_retry_enabled=true` first runs one bounded
  yaw alignment, then sends one same-goal Nav2 retry inside the `0.35m`
  near-goal window before the API decides whether strict final verification,
  final yaw fallback, or failure is appropriate.
- During that ordinary final-yaw fallback,
  `navigation_final_yaw_align_wait_bridge_smoothing=true` first waits for
  `robot_localization_bridge.safe_for_goal_start=true`, then
  `navigation_pause_global_correction_during_final_yaw=true` pauses correction
  intake so AMCL/Isaac localization candidates cannot move `map->odom` while
  the robot is spinning in place.
- Ordinary post-Nav2 bridge wait, same-goal retry, and acceptance slack are
  enabled by default. Normal completion is <=0.06 m XY and <=0.05 rad yaw;
  post-retry XY slack accepts <=0.08 m.
- API final pose audit records `final_distance_m`, `final_yaw_error_rad`,
  `final_verify_xy_error_m`, `final_verify_yaw_error_rad`, and
  `final_pose_verify_reason`; it does not turn a Nav2 success into false
  `task_complete=true` when commercial final verification is outside the gate.

Docking predock remains stricter than ordinary delivery handoff. Predock can
still reject the handoff if its docking-specific pose/yaw/bridge/GS2 gates are
not satisfied, but ordinary point navigation no longer runs API-owned velocity
recovery after Nav2 success.

Validation:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_nav2_native_goal_completion.sh
bash scripts/jetson/runtime_overlay/scripts/verify_post_nav2_final_verify_recovery.sh \
  --mock-nav2-succeeded \
  --mock-final-distance 0.070 \
  --mock-yaw-error 0.018 \
  --mock-tolerance 0.06 \
  --expect-retry-xy
```

This phase does not change Nav2 planner/controller plugin types, MPPI critics,
costmap source policy, TF tolerances, `max_odom_tf_age_ms`, pointcloud QoS/DDS,
FAST-LIO2, Ranger odom, EKF, or the `robot_safety` speed chain. The only
localization interaction is the bounded correction-intake pause around API
final-yaw fallback; AMCL itself is not stopped.
