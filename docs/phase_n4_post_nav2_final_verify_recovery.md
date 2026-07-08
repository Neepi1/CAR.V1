# Phase N4 Post-Nav2 Final Verify Recovery

Reinstated for commercial ordinary navigation after N5. Nav2 remains the
primary approach owner, but API-side commercial final verification now retries
bounded final pose overruns, can run bounded terminal XY correction for
short side-slip residuals, and reports `degraded` instead of false success.

Phase N4 keeps ordinary navigation completion primarily owned by Nav2 native
`RotationShimController + SimpleGoalChecker`. API-owned terminal correction is
enabled only after the Nav2 result/final verification path: final yaw uses
`/cmd_vel_api`, and short lateral residuals use `/cmd_vel_api` plus a temporary
`/ranger_mini3/forced_mode=side_slip` request.

Runtime sequence for normal `pose_required` goals:

```text
NavigateToPose(x/y/yaw)
-> Nav2 action result
   or near-goal stalled handoff if Nav2 keeps executing without terminal progress
-> wait for bridge map->odom smoothing before final verify
   and request one AMCL /request_nomotion_update while stationary when AMCL is gated
-> final_pose_verify
-> if verified: task_complete=true
-> if recoverable XY/yaw error: terminal correction if gated, then same-goal retry
-> if XY is only slightly outside tolerance after retry: accept within post_nav2_final_verify_acceptance_slack_m
-> if only yaw remains outside tolerance: API final_yaw_align through safety chain
-> wait for AMCL/bridge to settle again, then final verification
```

The bridge wait is bounded by
`post_nav2_final_verify_bridge_wait_timeout_ms` and defaults to `2000ms`. A
timeout is recorded in `/api/v1/navigation/state`, but commercial completion
now also requires the final-verify bridge gate to be ready. Active/frozen
`map -> odom` correction and unfinished smoothing still block
`task_complete=true`; however, the final gate now matches the goal-start gate
for clean stationary AMCL standby. When `amcl_static_standby=true` and
`amcl_not_moving_no_update_ok=true`, a resident gated AMCL
`amcl_correction_pending=true` status is treated as "no stationary update was
needed", not as an active localization transition. When AMCL is resident and
gated, the API still requests one `/request_nomotion_update` before final
verification so AMCL can emit a fresh stationary correction candidate when one
is available.

Retry policy:

- `post_nav2_final_verify_max_retry_count: 3`
- post-retry XY acceptance slack: `0.02m`
- XY retry range: `0.06m..0.35m`
- yaw retry: enabled for `pose_required`
- retry target: the same Nav2 goal, including the original target yaw
- API velocity correction: enabled only for bounded final yaw/terminal XY correction
- terminal XY correction: enabled when yaw is already within `0.05rad`, XY is within `0.30m`, and the forward error is within `0.12m`; the correction uses the terminal lateral configuration and side-slip forced mode, but it is axis-staged. While lateral error is above `0.03m`, it publishes only `linear.y` with the field-validated positive command sign. Only after lateral is inside target does it publish a pure `linear.x` forward/backward correction. This avoids the Ranger Mini3 official driver's mixed x/y parallel-mode behavior, which can move the robot away from the target near the goal. Side-slip direction still reverses at most once if the lateral error diverges, then exits to retry/degraded instead of pushing indefinitely.
- terminal XY correction is checked before the yaw-only salvage wait. If the robot is already within the terminal lateral gate, the API goes directly to bounded axis-staged correction instead of waiting up to the final-yaw timeout for a yaw-only candidate that can never solve the remaining XY error.
- near-goal stalled handoff: enabled by `navigation_near_goal_stalled_handoff_enabled`; when Nav2 is still executing within `0.30m` of the target for at least `3s` and has not improved by `0.02m` over `1.5s`, the API cancels only that Nav2 goal and enters the same final verification/correction path; this does not loosen the `0.06m` commercial acceptance gate
- near-goal yaw-first recovery does not wait through the generic final-pose salvage loop when the robot is already inside the `0.35m` recovery window but outside the yaw-only XY gate. This avoids an 8s idle gap seen on the `delivery_230891 -> delivery_987692` route before the API can either align yaw or retry the same Nav2 goal.
- ordinary API final yaw uses a conservative `0.60rad/s` cap, matching the Nav2 RotationShim cap. Field data showed a `2.39rad` residual yaw could time out at `0.35rad/s` with only about `0.053rad` remaining, forcing an extra same-goal retry. The API yaw loop now stops at `navigation_final_yaw_align_success_tolerance_rad=0.045`, while commercial verification remains `0.05rad`, so normal pose jitter after the zero command does not turn a good final yaw into `degraded`.
- reverse is not globally enabled; ordinary MPPI/Nav2 only has a bounded low-speed terminal reverse envelope (`vx_min=-0.08`) guarded by `PreferForwardCritic`, while `/ranger_mini3/allow_reverse` is retained as a bounded final-verify permit path for future/dedicated recovery commands

If the retry succeeds and final verification passes, `task_complete=true`. If
the retry is exhausted and the remaining XY error is within the small
post-Nav2 slack, ordinary delivery is accepted while preserving the measured
`final_verify_xy_error_m` for field logs. If retries cannot satisfy the
commercial gate, the goal enters `degraded_final_pose_verify` with
`task_complete=false`; safety blocks, unavailable localization, action timeout,
or explicit cancel still use terminal failure/cancel states as appropriate.

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
  --mock-final-distance 0.070 \
  --mock-yaw-error 0.018 \
  --mock-tolerance 0.06 \
  --expect-retry-xy

bash scripts/jetson/runtime_overlay/scripts/observe_post_nav2_final_verify_recovery.sh \
  --duration-sec 180
```

For field MPPI tuning or Nav2 abort diagnosis, capture a real goal with:

```bash
bash scripts/jetson/runtime_overlay/scripts/record_navigation_goal_diagnostic.sh \
  --duration-sec 60 \
  --post-goal-file /tmp/nav_goal.json
```

The report records `/navigate_to_pose`, `/compute_path_to_pose`, and
`/follow_path` action status/feedback plus the command chain, so a failed run
can distinguish planner/BT handoff failures from controller output or safety
chain failures before changing MPPI weights.

Field validation:

- `20260630T061132Z_verify_delivery_230891_forward_terminal_xy_fix` verified
  the forward-only terminal XY path: an earlier `9.6cm` mostly-forward residual
  no longer degraded, and the final pose completed at `0.048m` XY and
  `0.00054rad` yaw with `/cmd_vel_api` linear correction.
- `20260630T061311Z_nav_230891_to_987692_fast_params_forward_xy_fix_60s`
  completed `delivery_230891 -> delivery_987692` in about `33s`, with
  `final_distance_m=0.050476`, `final_yaw_error_rad=0.043281`, no same-goal
  Nav2 retry, API yaw capped at `0.60rad/s`, and terminal XY correction
  completing in about `1.0s`.
- `20260630T080403Z_verify_delivery_987692_salvage_skip_rotshim06_no_post`
  and `20260630T080506Z_verify_delivery_230891_salvage_skip_rotshim06_no_post`
  verified the current field profile after positive terminal-lateral sign,
  `0.60rad/s` RotationShim/API yaw, and terminal-lateral-before-salvage gating:
  `delivery_987692` completed in `30.61s` at `0.048962m` / `0.042367rad`,
  `delivery_230891` completed in `13.78s` at `0.056584m` / `0.002038rad`,
  `/cmd_vel_api` had `mixed_xy=0` in both runs, and neither run entered
  `final_pose_salvage_waiting`.

This phase does not change Nav2 planner/controller plugins, progress checker,
TF tolerances, `max_odom_tf_age_ms`, AMCL/Isaac/bridge correction policy,
pointcloud QoS/DDS, FAST-LIO2, Ranger odom, EKF, or the `robot_safety` speed
chain. Ordinary MPPI/Nav2 reverse is limited to low-speed terminal correction
and remains forward-biased by `PreferForwardCritic`; docking/teleop reverse
continues to use explicit permit paths instead of the normal MPPI sampler.
