# IMU EKF Optimization Standard

Scope: Ranger Mini3 point navigation between `delivery_512355` and
`delivery_675235` on map `B10/F1`.

Primary baseline is the wheel-pose-anchored `wheel_imu` EKF run:

- Navigation leg:
  `delivery_512355 -> delivery_675235`
- Report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T043132Z_delivery_675235_imu_ekf_observe_only_after_safety_relocalize/summary.md`
- Runtime AMCL mode: `gated`
- Moving AMCL candidates: `70`
- Moving AMCL accepted corrections: `0`
- Moving AMCL rejected corrections: `70`
- Rejection reason: `AMCL_ROBOT_MOVING_OBSERVE_ONLY`
- API/Nav2 final audit before post-goal relocalize:
  `0.139 m` xy error, `0.010 rad` yaw error
- Post-goal explicit relocalize correction:
  `0.858 m` map->odom translation, `0.065 rad` yaw
- AMCL candidate correction distribution during the run:
  median about `0.847 m`, mean about `0.739 m`, max about `0.889 m`
- Yaw-rate alignment during the run:
  high-rate signs match, but IMU yaw-rate is about `0.92x` wheel yaw-rate for
  stronger turns.

This baseline means AMCL gating is working. The optimization target is no longer
"reduce accepted AMCL count" during movement, because moving corrections are now
observe-only. The target is to reduce the correction that an explicit
post-goal relocalization has to apply after a normal point-to-point run.

Optimization target for an IMU-biased local-state EKF:

- Moving AMCL accepted corrections: must remain `0`.
- Post-goal map->odom translation correction:
  target `< 0.30 m`, first acceptable improvement `<= 0.50 m`.
- Post-goal map->odom yaw correction:
  target `< 0.035 rad`, first acceptable improvement `<= 0.050 rad`.
- Post-goal explicit relocalize true `map->base_link` target error:
  target `< 0.30 m`, first acceptable improvement `<= 0.50 m`.
  Do not accept a profile on map->odom correction alone, because yaw correction
  around a distant odom origin can move `map->base_link` laterally by nearly
  a meter even when map->odom translation looks smaller.
- Moving AMCL candidate P95 translation:
  target `< 0.40 m`, first acceptable improvement `<= 0.60 m`.
- API/Nav2 final audit before post-goal relocalize:
  must still be within the existing navigation tolerance.
- TF contract:
  `robot_local_state` remains the only `odom->base_link` publisher, and
  `robot_localization_bridge` remains the only `map->odom` publisher.

Implementation rule:

- Prefer the conservative `wheel_xy_imu_yaw` experiment first. It keeps chassis
  x/y pose, but removes chassis yaw pose from EKF fusion so heading is governed
  by wheel yaw-rate plus corrected JT128 IMU yaw-rate.
- If `wheel_xy_imu_yaw` still follows the wheel yaw-rate too closely, test
  `wheel_xy_imu_vyaw`: wheel x/y pose and wheel vx remain fused, but corrected
  JT128 IMU yaw-rate is the only yaw-rate measurement.
- `twist_imu` fuses chassis feedback twist plus corrected JT128 IMU yaw-rate and
  intentionally does not fuse the chassis-integrated wheel pose x/y/yaw. The
  first A/B attempt below rejected it as a safe default.
- `wheel_imu` remains available as a one-line rollback by setting
  `LOCAL_STATE_EKF_PROFILE=wheel_imu`.
- Do not change Nav2 planner/controller plugins, JT128 pointcloud/QoS/timestamps,
  FAST-LIO2, or the Ranger SDK while evaluating this EKF profile.

First `twist_imu` A/B attempt:

- Report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T045747Z_delivery_512355_twist_imu_observe_only_after_safety_relocalize/summary.md`
- Result: rejected as default.
- Nav2/API final audit: failed, final distance `11.147 m`, yaw error `1.990 rad`.
- Post-goal relocalize correction: `1.697 m` map->odom translation and
  `2.759 rad` yaw.
- Conclusion: removing wheel pose entirely is not safe for this current
  Nav2/bridge runtime without additional startup alignment and controller
  validation. Keep `wheel_imu` as the stable default and treat `twist_imu` as
  an experimental profile only.

Recovery status after rejecting `twist_imu`:

- Runtime default was returned to `LOCAL_STATE_EKF_PROFILE=wheel_imu`.
- `robot_local_state` runtime parameters again fuse wheel x/y/yaw pose, wheel
  forward/yaw velocity, and corrected IMU yaw-rate.
- Safety relocalization after rollback reported `0.0 m / 0.0 rad` correction and
  `bridge_safe_for_goal_start=True`.
- A recovery navigation attempt to `delivery_512355` did not move the robot:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T050704Z_delivery_512355_wheel_imu_recovery_baseline/summary.md`
- Nav2 failed after about 7 seconds with result code `6`; runtime logs show
  `Robot is out of bounds of the costmap` and a timeout waiting for
  `compute_path_to_pose` acknowledgement.
- Conclusion: do not continue automatic A/B navigation until the current map
  pose is externally confirmed or recovered to a known valid start region.

Later recovery and additional A/B results:

- The robot was recovered through explicit relocalization and AMCL gated
  readiness. The API server was confirmed on `http://127.0.0.1:8080`; `8010`
  was a stale probe port.
- `wheel_xy_imu_yaw` was tested from the `delivery_675235` side toward
  `delivery_512355`.
  Report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T052608Z_delivery_512355_wheel_xy_imu_yaw_observe_only/summary.md`
- Its API POST timed out, but Nav2 did move to the goal region. Manual
  post-goal relocalize:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T052608Z_delivery_512355_wheel_xy_imu_yaw_observe_only/relocalize_compare_20260621T053005Z/summary.md`
- Result: rejected as default. The explicit relocalize correction was
  `0.253 m` map->odom translation and `0.068 rad` yaw, but true
  `map->base_link` moved `0.917 m`; the corrected pose was about `0.714 m`
  from `delivery_512355` with about `0.098 rad` yaw error.
- Interpretation: removing wheel yaw pose while still fusing wheel yaw-rate
  does not reduce the real terminal error.

`wheel_xy_imu_vyaw` A/B attempt:

- New profile added for controlled diagnostics only:
  `LOCAL_STATE_EKF_PROFILE=wheel_xy_imu_vyaw`.
- Config behavior: fuse wheel `x`, `y`, and `vx`; do not fuse wheel yaw pose
  or wheel yaw-rate; fuse corrected JT128 IMU `vyaw`.
- Report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T054824Z_delivery_675235_wheel_xy_imu_vyaw_observe_only_retry/summary.md`
- Runtime AMCL mode: `gated`.
- API/Nav2 final audit before post-goal relocalize:
  `0.152 m` xy error, `0.000567 rad` yaw error.
- Moving AMCL candidates during the 180 s window: `59`.
- Moving AMCL accepted corrections: `0`.
- Post-goal explicit relocalize correction:
  `0.461 m` map->odom translation and `0.051 rad` yaw.
- True corrected `map->base_link` moved `1.013 m`. Against
  `delivery_675235`, the post-relocalize pose was about `0.984 m` from target
  and about `0.041 rad` yaw error.
- Same-target baseline (`wheel_imu`) post-relocalize true target error was
  about `0.785 m` xy and `0.053 rad` yaw, using
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T043132Z_delivery_675235_imu_ekf_observe_only_after_safety_relocalize/post_relocalize_compare/summary.md`.
- Interpretation: `wheel_xy_imu_vyaw` reduced yaw correction (`3.70 deg` to
  `2.92 deg`) and map->odom translation (`0.858 m` to `0.461 m`), but it
  increased true terminal xy error. It is not acceptable as a default profile.

Current runtime safety state after A/B:

- Runtime was restored to `LOCAL_STATE_EKF_PROFILE=wheel_imu`.
- Confirmed process:
  `/opt/ros/humble/lib/robot_localization/ekf_node --params-file .../local_state_ekf.yaml`.
- Confirmed parameters:
  wheel `x/y/yaw/vx/vyaw` plus corrected IMU `vyaw`.
- Safety relocalization after restoring default:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260621T055242Z_wheel_imu_restore_safety_relocalize/relocalize_compare_20260621T055243Z/summary.md`
- AMCL readiness was completed after restore. Final state:
  `localization_degraded=false`, `safe_for_goal_start=true`,
  `amcl_state=AMCL_READY`.

Decision:

- Keep `wheel_imu` as the runtime default.
- Keep `twist_imu`, `wheel_xy_imu_yaw`, and `wheel_xy_imu_vyaw` as diagnostic
  profiles only.
- Do not promote an IMU EKF profile until it improves both:
  post-goal map->odom correction and post-relocalize true target error.
- The next likely fix should focus on the odom/motion-model yaw source or
  geometric calibration, not just EKF sensor selection.

Soft-yaw covariance tuning follow-up:

- A softer wheel yaw/yaw-rate covariance route was tested because deleting wheel
  yaw pose or wheel yaw-rate was too disruptive. This keeps the production EKF
  fusion fields but changes only the `/wheel/odom_ekf` covariance floors.
- Default wheel yaw/yaw-rate covariance floor: `0.08`.
- Aggressive soft-yaw profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw`, floor `0.25`.
- Moderate soft-yaw profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_015`, floor `0.15`.
- A temporary uncommitted runtime-only `0.12` floor was also tested through
  `LOCAL_STATE_WHEEL_ODOM_EKF_PARAMS_FILE=/tmp/local_state_wheel_odom_ekf_soft_yaw_012.yaml`.

Same-direction default baseline to `delivery_512355`:

- Report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T055657Z_delivery_512355_wheel_imu_same_direction_baseline/summary.md`
- API/Nav2 final audit: succeeded, `0.163 m` xy, `0.0058 rad` yaw.
- Moving AMCL candidates: `75`; accepted: `0`.
- Post-goal explicit relocalize:
  `0.224 m` map->odom, `0.043 rad` yaw.
- True corrected target error after relocalize:
  about `0.714 m` xy and `2.11 deg` yaw.

Aggressive `0.25` soft-yaw result:

- Report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T060511Z_delivery_675235_wheel_imu_soft_yaw_observe_only/summary.md`
- Result: rejected.
- It produced a low post-relocalize true target error (`0.679 m` xy,
  `0.31 deg` yaw) on the `delivery_675235` side, but Nav2/API failed the run
  with result code `6` and believed the robot remained about `11.0 m` from the
  goal before relocalization. That means the online control-frame pose drifted
  too much during navigation.

Moderate `0.15` soft-yaw result:

- `delivery_675235 -> delivery_512355` report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T061234Z_delivery_512355_wheel_imu_soft_yaw_015_observe_only/summary.md`
- API/Nav2 final audit: succeeded, `0.161 m` xy, `0.0265 rad` yaw.
- Moving AMCL candidates: `68`; accepted: `0`.
- Post-goal explicit relocalize:
  `0.326 m` map->odom, `0.012 rad` yaw.
- True corrected target error after relocalize:
  about `0.416 m` xy and `0.70 deg` yaw.
- This is the best successful one-way improvement so far versus the same
  direction default baseline (`0.714 m` xy and `2.11 deg` yaw).

- `delivery_512355 -> delivery_675235` reverse report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T061753Z_delivery_675235_wheel_imu_soft_yaw_015_reverse_observe_only/summary.md`
- API/Nav2 final audit: succeeded, but with a warning: `0.180 m` xy and
  `0.0518 rad` yaw, just outside the `0.05 rad` yaw audit tolerance.
- Moving AMCL candidates: `76`; accepted: `0`.
- Post-goal explicit relocalize:
  `0.544 m` map->odom, `0.056 rad` yaw.
- True corrected target error after relocalize:
  about `0.422 m` xy and `6.11 deg` yaw.
- Interpretation: `0.15` is the strongest current candidate for reducing xy
  drift, but yaw consistency is not good enough to promote it as the default.

Temporary `0.12` soft-yaw result:

- Report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260621T062325Z_delivery_512355_wheel_imu_soft_yaw_012_observe_only/summary.md`
- API/Nav2 final audit: succeeded with warning, `0.154 m` xy and `0.0587 rad`
  yaw.
- Moving AMCL candidates: `74`; accepted: `0`.
- Post-goal explicit relocalize:
  only `0.116 m` map->odom translation, but `0.087 rad` yaw.
- True corrected target error after relocalize:
  about `1.114 m` xy and `1.41 deg` yaw.
- Result: rejected. It again confirms that small map->odom translation alone is
  not a sufficient acceptance metric.

Updated decision after soft-yaw tests:

- Runtime was restored to the stable `wheel_imu` default after testing.
- Confirmed restored runtime:
  wheel yaw/yaw-rate covariance floor `0.08`, wheel `x/y/yaw/vx/vyaw` plus
  corrected IMU `vyaw`, `safe_for_goal_start=true`, `AMCL_READY`.
- `wheel_imu_soft_yaw_015` remains the best diagnostic candidate, but it is not
  promoted to default because reverse yaw is outside the target.
- Do not continue broad EKF-only tuning without a yaw-specific acceptance rule:
  any candidate must pass both directions with API success, moving AMCL accepted
  count `0`, post-relocalize true xy `<=0.50 m`, and post-relocalize true yaw
  `<=0.050 rad`.

Next narrow EKF candidate:

- New diagnostic profile: `LOCAL_STATE_EKF_PROFILE=wheel_imu_pose_soft_yaw_015`.
- Hypothesis: the previous `wheel_imu_soft_yaw_015` improved terminal XY but
  degraded reverse terminal yaw because it softened both wheel yaw pose and
  wheel yaw-rate. This profile keeps the same production EKF fusion fields and
  the same `0.15` wheel yaw-pose covariance floor, but keeps wheel yaw-rate
  covariance at the stable `0.08` default.
- Expected effect: retain the XY improvement from reducing wheel yaw-pose
  dominance while preserving low-latency wheel yaw-rate feedback for Nav2 final
  heading convergence.
- Acceptance rule remains unchanged: both `delivery_675235 -> delivery_512355`
  and `delivery_512355 -> delivery_675235` must finish through the normal API
  path, moving AMCL accepted count must stay `0`, post-relocalize true XY must
  be `<=0.50 m`, and post-relocalize true yaw must be `<=0.050 rad`.

`wheel_imu_pose_soft_yaw_015` field result:

- First guarded leg:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T114314Z_pingpong_leg1_delivery_675235/summary.md`
- Guard summary:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pingpong_guarded_pose_soft_yaw_015/20260622T114313Z_delivery_pingpong_guarded/summary.md`
- API/Nav2 final audit: succeeded with warning, `0.079 m` xy and `0.075 rad`
  yaw. This exceeds the `0.050 rad` yaw target.
- Post-goal explicit relocalize correction:
  `0.238 m` map->odom translation and `0.105 rad` yaw.
- True `map->base_link` jump after relocalize:
  `0.904 m` and `6.00 deg`.
- Decision: reject. Softening wheel yaw pose while keeping wheel yaw-rate at
  the default made the `delivery_675235` terminal error worse than the previous
  `wheel_imu_soft_yaw_015` reverse result.

Next narrow EKF candidate:

- New diagnostic profile: `LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_015`.
- Hypothesis: the `0.15/0.15` result may have improved XY mainly by reducing
  wheel yaw-rate dominance, while its weaker wheel yaw-pose anchor hurt terminal
  heading. This profile keeps wheel yaw pose at the stable `0.08` floor and
  raises only wheel yaw-rate covariance to `0.15`.
- Acceptance rule remains unchanged.

`wheel_imu_twist_soft_yaw_015` field result:

- `delivery_675235 -> delivery_512355` report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T115056Z_pingpong_leg1_delivery_512355/summary.md`
- Guard summary:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pingpong_guarded_twist_soft_yaw_015/20260622T115054Z_delivery_pingpong_guarded/summary.md`
- API/Nav2 final audit: succeeded, `0.144 m` xy and `0.0048 rad` yaw.
- Post-goal explicit relocalize: `0.093 m` map->odom and `0.041 rad`.
- True `map->base_link` jump after relocalize: `0.515 m` and `2.32 deg`.
  This misses the `0.50 m` target by about `1.5 cm`, but yaw is acceptable.

- `delivery_512355 -> delivery_675235` report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T115217Z_twist_soft_yaw_015_leg2_delivery_675235/summary.md`
- API/Nav2 final audit: succeeded, `0.152 m` xy and `0.030 rad` yaw.
- Post-goal explicit relocalize: `0.732 m` map->odom and `0.059 rad`
  (`3.38 deg`). The correction capture did not include a valid
  `map_base_link_delta`, but the map->odom correction alone already exceeds
  the acceptance envelope.
- Decision: reject. Softening wheel yaw-rate while keeping wheel yaw pose
  improves online final yaw but does not produce symmetric two-direction
  map-frame consistency.

Updated EKF tuning conclusion:

- `wheel_imu_pose_soft_yaw_015`: rejected.
- `wheel_imu_twist_soft_yaw_015`: rejected.
- `wheel_imu_soft_yaw_015` remains the best diagnostic candidate seen so far,
  but still is not safe to promote because reverse yaw exceeded the acceptance
  target.
- Runtime should be restored to stable `LOCAL_STATE_EKF_PROFILE=wheel_imu`
  after these A/B runs.

Next narrow EKF candidate:

- New diagnostic profile: `LOCAL_STATE_EKF_PROFILE=wheel_pose_imu_vyaw`.
- Hypothesis: default wheel yaw pose may still be a useful local heading anchor,
  while the chassis-reported wheel yaw-rate may be the part competing with the
  corrected JT128 IMU during terminal yaw convergence. This profile keeps wheel
  `x/y/yaw` pose and wheel `vx`, removes wheel `vyaw`, and keeps corrected IMU
  `vyaw` as the only EKF yaw-rate input.
- Acceptance rule remains unchanged.

`wheel_pose_imu_vyaw` field result:

- Guard summary:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pingpong_guarded_wheel_pose_imu_vyaw/20260622T121154Z_delivery_pingpong_guarded/summary.md`
- First leg report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T121156Z_pingpong_leg1_delivery_512355/summary.md`
- API/Nav2 final audit: succeeded, `0.153 m` xy and `0.0218 rad` yaw.
- Post-goal explicit relocalize correction:
  `0.329 m` map->odom translation and `3.63 deg` yaw.
- True `map->base_link` jump after relocalize:
  `0.996 m` and `3.63 deg`; lateral component was about `0.988 m`.
- Decision: reject. Removing wheel yaw-rate while keeping wheel yaw pose made
  the online controller believe it reached the target, but the true post-
  relocalize body pose was almost `1.0 m` away. This is worse than both the
  stable `wheel_imu` baseline and the `wheel_imu_soft_yaw_015` diagnostic run.

Updated EKF tuning conclusion after this run:

- `wheel_pose_imu_vyaw`: rejected.
- `wheel_imu_pose_soft_yaw_015`: rejected.
- `wheel_imu_twist_soft_yaw_015`: rejected.
- `wheel_imu_soft_yaw_015` remains the best diagnostic candidate, but it still
  fails reverse-yaw acceptance and must not be promoted.
- Runtime was restored to stable `LOCAL_STATE_EKF_PROFILE=wheel_imu`; AMCL
  gated was restored to `AMCL_READY` with scan admission alive.

Next narrow EKF candidate:

- New diagnostic profile: `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_018`.
- Hypothesis: `0.15/0.15` is the best observed direction for terminal XY but
  misses reverse yaw, while `0.25/0.25` is too soft and lets online control
  drift. A `0.18/0.18` covariance floor is the smallest next step above `0.15`
  that may improve reverse heading without losing the online stability seen at
  `0.15`.
- Acceptance rule remains unchanged.

`wheel_imu_soft_yaw_018` field result:

- Guard summary:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pingpong_guarded_soft_yaw_018/20260622T123101Z_delivery_pingpong_guarded/summary.md`
- First leg report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T123103Z_pingpong_leg1_delivery_675235/summary.md`
- API/Nav2 final audit: failed with result code `6`; online final distance was
  `11.099 m` and yaw error was `3.060 rad`.
- Post-goal relocalize capture also failed to accept the explicit trigger
  (`relocalize_exit_code=1`), but the captured metrics still showed:
  `0.513 m` map->odom translation, `1.779 m` map->base_link translation, and
  `2.69 deg` map->base_link yaw.
- Decision: reject. Raising both wheel yaw and yaw-rate covariance floors from
  `0.15` to `0.18` loses the online stability that made `0.15` usable for
  one-way testing. This confirms that the viable covariance window is very
  narrow and that simply increasing soft-yaw trust toward the IMU is not a
  robust fix.

Updated EKF tuning conclusion after `0.18`:

- `wheel_imu_soft_yaw_018`: rejected.
- `wheel_pose_imu_vyaw`: rejected.
- `wheel_imu_pose_soft_yaw_015`: rejected.
- `wheel_imu_twist_soft_yaw_015`: rejected.
- `wheel_imu_soft_yaw_015` remains the only profile with a meaningful one-way
  improvement, but it still fails reverse-yaw acceptance and must not be
  promoted.
- Runtime was restored again to stable `LOCAL_STATE_EKF_PROFILE=wheel_imu`;
  AMCL gated was restored to `AMCL_READY` with scan admission alive.

Next narrow EKF candidate:

- New diagnostic profile: `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_014`.
- Hypothesis: `0.15/0.15` is the only setting that produced a meaningful
  one-way target improvement, but reverse yaw exceeded the target. `0.18/0.18`
  is too soft and causes online Nav2 failure. A slightly firmer `0.14/0.14`
  setting tests the lower side of the narrow viable window: preserve most of
  the XY benefit while reducing reverse heading overshoot.
- Acceptance rule remains unchanged.

`wheel_imu_soft_yaw_014` field result:

- Guard summary:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pingpong_guarded_soft_yaw_014/20260622T125527Z_delivery_pingpong_guarded/summary.md`
- First leg report:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T125528Z_pingpong_leg1_delivery_512355/summary.md`
- API/Nav2 final audit: failed with result code `6`; online final distance was
  `11.155 m` and yaw error was `1.030 rad`.
- Post-goal explicit relocalize accepted and reset `map->odom` by `0.516 m`
  and `1.32 deg`. The capture did not include a valid `map->base_link` delta,
  so the guard stopped on missing true-pose metrics (`nan`) even before a
  second leg could be sent.
- Immediate stable-profile sanity check after restoring `wheel_imu`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pingpong_guarded_wheel_imu_after_014_sanity/20260622T130157Z_delivery_pingpong_guarded/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T130200Z_pingpong_leg1_delivery_512355/summary.md`.
  The restored stable profile also failed Nav2 completion in the terminal area:
  online final distance was `0.473 m`, yaw error was `0.756 rad`, and post-goal
  relocalize reported `0.265 m` true `map->base_link` translation with
  `5.87 deg` yaw correction.
- Decision: do not promote. The `0.14` run failed at the Nav2/API completion
  level, but the immediate restored `wheel_imu` sanity check also failed around
  the same endpoint. This makes the `0.14` run unsuitable as a clean EKF-only
  rejection; the dominant observed issue in this sequence is terminal yaw/start
  state contamination. A clean A/B must start from a verified endpoint after
  explicit relocalization and then send the opposite delivery target.

Updated EKF tuning conclusion after `0.14`:

- `wheel_imu_soft_yaw_014`: not promoted; current field run is confounded by
  terminal-yaw/start-state failure and must not be used as proof of improvement.
- `wheel_imu_soft_yaw_018`: rejected.
- `wheel_pose_imu_vyaw`: rejected.
- `wheel_imu_pose_soft_yaw_015`: rejected.
- `wheel_imu_twist_soft_yaw_015`: rejected.
- `wheel_imu_soft_yaw_015` remains the only profile with a meaningful one-way
  improvement, but it still fails reverse-yaw acceptance and must not be
  promoted.
- Runtime was restored to stable `LOCAL_STATE_EKF_PROFILE=wheel_imu`; live
  parameters were verified as `pose_covariance_floor_yaw=0.08` and
  `twist_covariance_floor_vyaw=0.08`. AMCL gated was restored to `AMCL_READY`
  with scan admission alive.

Clean endpoint follow-up after the contaminated `0.14` run:

- Stable `wheel_imu` opposite-direction baseline from the current
  `delivery_512355` area to `delivery_675235`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pingpong_guarded_wheel_imu_clean_after_014/20260622T130518Z_delivery_pingpong_guarded/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T130519Z_pingpong_leg1_delivery_675235/summary.md`.
- API/Nav2 final audit succeeded: `0.134 m` xy and `0.013 rad` yaw.
- Post-goal relocalize accepted a small `map->odom` translation correction
  (`0.119 m`), but yaw correction was `3.97 deg`, exceeding the `3.0 deg`
  guard. The capture did not include a valid `map->base_link` delta.
- This confirms the current dominant endpoint issue is yaw/global-pose
  consistency after Nav2 believes it has arrived, not gross online XY failure.

`wheel_imu_soft_yaw_015` clean follow-up:

- Diagnostic profile was re-run from the current `delivery_675235` area toward
  `delivery_512355`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pingpong_guarded_soft_yaw_015_clean_after_014/20260622T131548Z_delivery_pingpong_guarded/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T131550Z_pingpong_leg1_delivery_512355/summary.md`.
- API/Nav2 final audit succeeded but with a yaw warning:
  `0.063 m` xy and `0.0845 rad` yaw.
- Post-goal relocalize showed `0.141 m` map->odom translation, `4.74 deg`
  yaw correction, and a `1.037 m` true `map->base_link` translation jump,
  mostly lateral (`-1.035 m` in the before-frame left axis).
- Decision: do not promote `wheel_imu_soft_yaw_015`. It can make the online
  final pose look excellent, but the post-relocalize true body pose can be much
  worse than stable `wheel_imu`. The next work should shift from further EKF
  covariance sweeping to terminal yaw/controller/localization timing diagnosis,
  while keeping stable `wheel_imu` as the runtime default.
- Runtime was restored again to stable `LOCAL_STATE_EKF_PROFILE=wheel_imu`;
  live parameters were verified as `pose_covariance_floor_yaw=0.08` and
  `twist_covariance_floor_vyaw=0.08`. AMCL gated was restored to `AMCL_READY`
  with scan admission alive.

Clean-start guard and stable recovery follow-up:

- Added diagnostic entrypoint:
  `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_navigation_delivery_clean_start_guarded.sh`.
  It checks the current API `map` pose against `delivery_512355` or
  `delivery_675235` before sending the opposite endpoint through the existing
  guarded ping-pong script. Default clean-start gates are `0.50 m` XY and
  `5.0 deg` yaw.
- First clean-start check from the current `delivery_512355` side refused to
  send navigation:
  `/workspaces/njrh-v3/workspace1/reports/navigation_clean_start_guarded/20260622T132700Z_wheel_imu_clean_entry_after_015_expected_512355/summary.md`.
  The robot was nearest to `delivery_512355` but still `1.014 m` away in XY;
  yaw was only `0.70 deg` from target.
- Stable `wheel_imu` recovery navigation to `delivery_512355`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T132744Z_wheel_imu_recover_512_after_clean_guard/summary.md`.
  Nav2/API failed with result code `6`: final distance `0.385 m`, yaw error
  `1.030 rad`. Post-goal relocalize corrected `0.288 m` true
  `map->base_link` translation and `3.53 deg` yaw.
- Second clean-start check after the recovery attempt also refused to continue:
  `/workspaces/njrh-v3/workspace1/reports/navigation_clean_start_guarded/20260622T132908Z_wheel_imu_after_recover_512_guard_only_expected_512355/summary.md`.
  XY was now acceptable at `0.158 m`, but yaw was still `62.58 deg` from the
  saved `delivery_512355` heading.
- Decision: stop EKF A/B here. The two-point workflow is now blocked by terminal
  heading convergence at the endpoint, not by a missing EKF covariance profile.
  Stable `wheel_imu` remains the only runtime-safe default. The next technical
  work should inspect Nav2 terminal yaw behavior, RotationShim/goal-checker
  interaction, and the source of the yaw command at the endpoint before any more
  EKF profile experiments.

Terminal yaw and final-audit trace follow-up:

- Added diagnostic script:
  `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_navigation_terminal_yaw_trace.sh`.
  It records the Nav2/safety/mode-controller command chain, wheel/local odom,
  corrected IMU yaw-rate, motion mode, AMCL status, and API map pose while still
  sending goals only through the normal robot_api_server/Nav2 path.
- `delivery_512355` terminal-yaw repro before API final-audit gating:
  `/workspaces/njrh-v3/workspace1/reports/navigation_terminal_yaw_trace/20260622T134153Z_wheel_imu_terminal_yaw_delivery_512355_from_bad_yaw/summary.md`.
  Nav2/API reported success even though API final yaw error was `0.349774 rad`
  (`20.0 deg`). The full command chain did receive the pure spin command:
  `/cmd_vel_nav_raw -> /cmd_vel_nav -> /cmd_vel_collision_checked ->
  /cmd_vel_safe -> /cmd_vel`, max `angular.z=0.85`, and the chassis switched
  through motion mode `2` (`SPINNING`). This makes the immediate endpoint
  failure a completion-semantics issue, not an EKF covariance issue.
- Minimal API change applied and built on Jetson: Nav2 remains the single action
  completion owner and the API still does not publish corrective velocity, but a
  `pose_required` goal is no longer marked complete when the post-Nav2 final
  audit exceeds the XY tolerance or the required yaw gate
  (`navigation_final_yaw_align_trigger_rad`, currently `0.08 rad`). Such cases
  now finish as `failed_final_pose_verify`.
- Post-change same-target 512 test:
  `/workspaces/njrh-v3/workspace1/reports/navigation_terminal_yaw_trace/20260622T135616Z_wheel_imu_terminal_yaw_delivery_512355_after_final_audit_gate/summary.md`.
  Nav2 itself returned result code `6` without publishing nonzero commands, so
  this did not exercise the new final-audit gate.
- Post-change opposite-leg 512 -> 675 test:
  `/workspaces/njrh-v3/workspace1/reports/navigation_terminal_yaw_trace/20260622T135729Z_wheel_imu_terminal_yaw_delivery_675235_after_final_audit_gate/summary.md`.
  The online Nav2/API terminal audit was good (`0.197 m`, `0.026 rad` yaw), and
  the command chain was continuous through `/cmd_vel` with max linear
  `1.196 m/s` and max angular `0.85 rad/s`. However post-goal relocalization
  still showed a true `map->base_link` jump of `0.881 m` and `3.20 deg`,
  mostly lateral. This keeps stable `wheel_imu` safe as runtime default but
  shows that the remaining point-to-point error must be judged against
  post-relocalize true pose, not API online success alone.
- Recovery attempt to `delivery_675235` after that lateral jump:
  `/workspaces/njrh-v3/workspace1/reports/navigation_terminal_yaw_trace/20260622T140022Z_wheel_imu_recover_delivery_675235_after_lateral_jump/summary.md`.
  Nav2 failed with result code `6`; online final distance remained `0.978 m`
  and yaw error was `0.060 rad`. The command chain did issue commands through
  `/cmd_vel` for about 22 seconds, with max linear `0.482 m/s` and max angular
  `0.85 rad/s`. Post-relocalize reported `0.646 m` map->odom correction and
  `3.69 deg` yaw. The current state is therefore not a clean endpoint start for
  another EKF A/B leg.

Spin-center SDK fix follow-up:

- Runtime Ranger SDK was rebuilt with the `MOTION_MODE_SPINNING` base-to-center
  offset correction and verified live with
  `spinning_base_to_center_x=-0.14`, `spinning_base_to_center_y=0.06`.
- After restarting Ranger odom and `robot_local_state`, a safety
  relocalization was required because local odom was re-anchored to zero:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260622T143356Z_wheel_imu_spin_center_safety_relocalize/relocalize_compare/summary.md`.
  The large `11.342 m / 158.9 deg` map->odom update is the expected reset
  realignment, not a new navigation leg error.
- Stable `wheel_imu` leg from the recovered `delivery_675235` area toward
  `delivery_512355`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T143752Z_pingpong_leg1_delivery_512355/summary.md`.
  API/Nav2 final audit succeeded under `pose_required` with `0.159 m` XY and
  `0.052 rad` yaw. Post-goal relocalize still required `0.268 m / 3.03 deg`
  map->odom correction, which became a `0.842 m` true `map->base_link` lateral
  jump because the odom origin was about 11 m from the robot. This keeps the
  remaining dominant error as local yaw consistency over the long leg, not a
  missing velocity command or Nav2 action failure.
- Diagnostic `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_015` was re-tested
  after the spin-center SDK fix and then rejected for this state:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T144157Z_soft_yaw_015_spin_center_recover_512/summary.md`.
  The short recovery goal to `delivery_512355` failed Nav2 with result code `6`,
  online final error `0.351 m / 0.361 rad`. Post-goal relocalize was moderate
  (`0.218 m / 1.70 deg`), but the profile did not pass normal endpoint
  convergence and must not be used for a full two-point run.
- Runtime was restored to `LOCAL_STATE_EKF_PROFILE=wheel_imu` and explicitly
  relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260622T144148Z_wheel_imu_restore_after_soft_yaw_test/relocalize_compare/summary.md`.

Current decision after the spin-center follow-up:

- Do not promote `wheel_imu_soft_yaw_015`.
- Do not retry `twist_imu` automatically; the previous `twist_imu` run had an
  11 m class failure and remains a high-risk diagnostic only.
- Further EKF-only covariance/profile changes are unlikely to solve the current
  two-point behavior by themselves. The stronger next target is terminal yaw
  convergence and localization timing around Nav2 completion, while keeping
  stable `wheel_imu` as the runtime rollback profile.

Post spin-center `wheel_xy_imu_yaw` retry:

- The robot could not be recovered to a clean `delivery_512355` start with
  stable `wheel_imu`. A `pose_required` recovery goal failed with result code
  `6` and final online error `0.973 m / 1.103 rad`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T144659Z_wheel_imu_restore_recover_512_terminal_trace_nav_pose_error/summary.md`.
  A `position_only` recovery still ended at `0.829 m / 1.180 rad` online:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T144849Z_wheel_imu_position_only_recover_512/summary.md`.
- Despite the imperfect start, `LOCAL_STATE_EKF_PROFILE=wheel_xy_imu_yaw` was
  retried after the spin-center SDK fix. This profile keeps wheel `x/y` pose and
  wheel+IMU yaw-rate, but drops wheel yaw pose:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260622T144815Z_wheel_xy_imu_yaw_spin_center_relocalize/relocalize_compare/summary.md`.
- Test leg from the current 512 area toward `delivery_675235`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T145141Z_wheel_xy_imu_yaw_spin_center_512area_to_675_nav_pose_error/summary.md`.
  Nav2 failed with result code `6`; online final error was
  `0.304 m / 0.174 rad`. Post-goal relocalize required
  `1.064 m / 6.68 deg` map->odom correction, while true `map->base_link`
  shifted `0.312 m / 6.68 deg`.
- Decision: reject `wheel_xy_imu_yaw` again. Removing wheel yaw pose does not
  fix the remaining long-leg behavior and worsens map->odom correction in this
  post spin-center run.
- Runtime was restored to `LOCAL_STATE_EKF_PROFILE=wheel_imu` and explicitly
  relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260622T145147Z_wheel_imu_restore_after_xy_yaw/relocalize_compare/summary.md`.

Trace-based EKF yaw-source analysis:

- In the stable `wheel_imu` traces, `local_odom` yaw matched `/wheel/odom_ekf`
  yaw exactly at the end of the run. The corrected IMU yaw-rate did not move
  the final local heading away from the chassis-integrated wheel pose when wheel
  yaw pose and wheel yaw-rate were both fused.
- In the `wheel_xy_imu_yaw` trace, dropping wheel yaw pose made final
  `local_odom` yaw differ from `/wheel/odom_ekf` by about `0.105 rad`
  (`6.0 deg`), and that candidate worsened the post-goal map->odom correction.
- Integrated corrected IMU yaw-rate was not a stable constant scale relative to
  wheel yaw-rate across the tested legs: about `0.93x`, `1.03x`, and `0.94x`.
  That makes a simple EKF yaw-rate substitution or static gain an unsafe default
  without a broader calibration pass.

Post spin-center `wheel_pose_imu_vyaw` retry:

- Rationale: this is more conservative than `wheel_xy_imu_yaw`; it keeps wheel
  `x/y/yaw` pose and wheel forward speed, but removes wheel yaw-rate so the
  corrected JT128 IMU gyro is the only EKF yaw-rate input.
- Runtime was switched to `LOCAL_STATE_EKF_PROFILE=wheel_pose_imu_vyaw` and
  relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260622T145629Z_wheel_pose_imu_vyaw_spin_center_relocalize/relocalize_compare/summary.md`.
- Test leg from the current `delivery_675235` area toward `delivery_512355`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T145939Z_wheel_pose_imu_vyaw_spin_center_675area_to_512_nav_pose_error/summary.md`.
  Online API/Nav2 final audit succeeded (`0.167 m / 0.037 rad`), but post-goal
  relocalize showed a true `map->base_link` jump of `0.937 m / 3.43 deg` and a
  `0.336 m / 3.43 deg` map->odom correction. The online success therefore did
  not correspond to true endpoint accuracy.
- Decision: reject `wheel_pose_imu_vyaw`. Keeping wheel yaw pose while replacing
  wheel yaw-rate with IMU yaw-rate does not fix the long-leg true lateral error.
- Runtime was restored again to `LOCAL_STATE_EKF_PROFILE=wheel_imu` and
  explicitly relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260622T145911Z_wheel_imu_restore_after_pose_vyaw/relocalize_compare/summary.md`.

Current EKF decision:

- Rejected after post spin-center testing: `wheel_imu_soft_yaw_015`,
  `wheel_xy_imu_yaw`, and `wheel_pose_imu_vyaw`.
- Previously rejected or high-risk: `twist_imu`, `wheel_xy_imu_vyaw`,
  `wheel_imu_pose_soft_yaw_015`, `wheel_imu_twist_soft_yaw_015`,
  `wheel_imu_soft_yaw_018`, and `wheel_imu_soft_yaw_014`.
- Keep `wheel_imu` as the rollback runtime. The remaining two-point problem is
  not solved by EKF profile selection alone; the next engineering work should
  target terminal goal convergence/completion semantics and the global
  localization timing used for the final map pose.

Post spin-center `wheel_imu_twist_soft_yaw_015` retry:

- Rationale: this is the least invasive remaining yaw-rate candidate. It keeps
  the stable `wheel_imu` EKF fusion fields and wheel yaw-pose covariance, but
  raises only wheel yaw-rate covariance from `0.08` to `0.15` so corrected JT128
  gyro can have a small turn-rate influence.
- Runtime was switched to `LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_015`
  and relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260622T150145Z_twist_soft_yaw_015_spin_center_relocalize/relocalize_compare/summary.md`.
- Test leg from the current 512 area toward `delivery_675235`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T150512Z_twist_soft_yaw_015_spin_center_512area_to_675_nav_pose_error/summary.md`.
  The API/Nav2 goal record failed almost immediately with result code `6` and
  final error `10.703 m / 3.063 rad`.
- The terminal trace for the same leg shows that command output continued for
  about 20 seconds after the API goal record had already entered failed state,
  and the sampled API map pose later appeared close to `delivery_675235`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_terminal_yaw_trace/20260622T150509Z_twist_soft_yaw_015_spin_center_512area_to_675/summary.md`.
  This makes the run internally inconsistent as an EKF acceptance sample.
- The post-goal relocalize trigger returned nonzero (`trigger_accepted=false`,
  `BRIDGE_REJECTED_RESULT`), although bridge metrics still showed a small
  `0.255 m / 2.15 deg` map->odom delta. Because the normal goal lifecycle did
  not remain coherent, this profile cannot be promoted.
- Decision: reject `wheel_imu_twist_soft_yaw_015` for runtime use. It does not
  provide a clean, reproducible two-point improvement and introduced a
  goal-state/command-trace inconsistency in this field run.
- Runtime was restored again to `LOCAL_STATE_EKF_PROFILE=wheel_imu` and
  explicitly relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260622T150440Z_wheel_imu_restore_after_twist_soft/relocalize_compare/summary.md`.

Final EKF-profile status after this round:

- Rejected after post spin-center testing: `wheel_imu_soft_yaw_015`,
  `wheel_xy_imu_yaw`, `wheel_pose_imu_vyaw`, and
  `wheel_imu_twist_soft_yaw_015`.
- Keep `wheel_imu` as the only field-safe runtime profile for now.
- Do not continue automatic EKF profile A/B runs without first fixing the
  endpoint lifecycle/terminal convergence issue; otherwise online goal records,
  command traces, and post-relocalize truth can disagree enough to make EKF
  conclusions unsafe.

Offline EKF A/B acceptance gate:

- Added
  `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/validate_ekf_ab_report.py`.
  It is offline-only: it reads saved `summary.md` files and never calls ROS,
  the API server, relocalization, or velocity topics.
- Integrated the same gate into
  `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_navigation_delivery_pingpong_guarded.sh`.
  Each ping-pong leg now writes `legN_delivery_TARGET_ekf_ab_validation.json`
  and stops before the next target unless the saved report passes the full EKF
  A/B acceptance rule.
- Added read-only replay helper
  `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/replay_navigation_pingpong_ekf_gate.py`.
  It re-scores saved delivery ping-pong reports with the same gate without
  sending navigation goals, triggering relocalization, or publishing velocity.
- Added guarded apply wrapper
  `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_ekf_profile_delivery_ab_guarded.sh`.
  Its default mode is dry-run. With `--apply`, it restarts local_state with the
  requested EKF profile, explicitly relocalizes, runs the guarded two-point
  delivery A/B path, then always restores `wheel_imu` and relocalizes again.
  The apply path now first checks the current API map pose against the expected
  delivery start (`delivery_512355` or `delivery_675235`) and refuses before
  any EKF switch if the robot is not within `0.50 m / 5 deg`, if a navigation
  goal is active, or if localization is not safe for goal start.
- Acceptance requires all of the following: API/Nav2 `state=succeeded`,
  `nav2_result_code=4`, `final_pose_verified=True`, online final error within
  `0.20 m / 0.08 rad`, post-goal relocalize exit code `0`,
  `post_relocalize_compare` `trigger_accepted=true`, true
  `map->base_link` correction within `0.50 m / 2.0 deg`, and `map->odom`
  correction within `0.30 m / 2.0 deg`. Optional terminal traces must also end
  with API state `succeeded`, phase `final_pose_verified`, and Nav2 result `4`.
- Field validation on the latest reports:
  - Stable `wheel_imu`
    `20260622T143752Z_pingpong_leg1_delivery_512355`: rejected because true
    `map->base_link` was `0.842 m / 3.03 deg` and `map->odom` yaw was
    `3.03 deg`, despite online API success.
  - `wheel_pose_imu_vyaw`
    `20260622T145939Z_wheel_pose_imu_vyaw_spin_center_675area_to_512_nav_pose_error`:
    rejected because true `map->base_link` was `0.937 m / 3.43 deg` and
    `map->odom` was `0.336 m / 3.43 deg`, despite online API success.
  - `wheel_imu_twist_soft_yaw_015`
    `20260622T150512Z_twist_soft_yaw_015_spin_center_512area_to_675_nav_pose_error`
    with terminal trace
    `20260622T150509Z_twist_soft_yaw_015_spin_center_512area_to_675`: rejected
    because Nav2/API failed (`result_code=6`, `final_pose_verified=False`),
    post-relocalize was not accepted (`BRIDGE_REJECTED_RESULT`), and the trace
    also ended in `nav2_failed`.
- Verification:
  `python3 -m py_compile scripts/jetson/runtime_overlay/scripts/replay_navigation_pingpong_ekf_gate.py scripts/jetson/runtime_overlay/scripts/validate_ekf_ab_report.py`
  passed locally and inside the Jetson container.
  `bash -n scripts/jetson/runtime_overlay/scripts/run_ekf_profile_delivery_ab_guarded.sh`
  passed inside the Jetson container.
  `python3 -m pytest src/robot_system_tests/test/test_workspace_contracts.py -k ekf_ab_report_validator_rejects_dirty_navigation_samples`
  passed both locally and inside the Jetson container.
- Latest replay output:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pingpong_ekf_gate_replay/20260622_latest_replay/summary.md`.
  It is read-only and rejects all three latest reports for the same reasons
  listed above.
- Latest dry-run A/B wrapper output:
  `/workspaces/njrh-v3/workspace1/reports/ekf_profile_delivery_ab_guarded_dryrun/20260622T153244Z_wheel_imu_soft_yaw_015_delivery_ab_guarded/summary.md`.
  It did not restart local_state or send navigation goals; live runtime stayed
  on stable `wheel_imu`. The dry-run summary includes the apply-time start
  guard plan for `delivery_512355 -> delivery_675235`.

Preflight-only guarded start check:

- Added `--preflight-only` to
  `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_ekf_profile_delivery_ab_guarded.sh`.
  This mode reads `/api/v1/robot/pose`, `/api/v1/navigation/state`, and
  `/api/v1/status`, writes the same start-guard report used by `--apply`, then
  exits before any local_state restart, relocalization, navigation goal, or
  velocity-producing path.
- The first read-only preflight from the current field pose used the default
  `0.50 m / 5 deg` start gate for `delivery_675235` and correctly refused:
  `/workspaces/njrh-v3/workspace1/reports/ekf_profile_delivery_ab_guarded_preflight/20260622T154113Z_wheel_imu_soft_yaw_015_delivery_ab_guarded/start_guard/summary.md`.
  API pose was `(-6.142628, -2.535219, -1.558524)`, which is nearest to
  `delivery_675235` but `0.544805 m / 2.596530 deg` away, so the default
  start XY gate failed by about `4.5 cm`.
- A second read-only preflight with only `--max-start-xy-m 0.60` passed:
  `/workspaces/njrh-v3/workspace1/reports/ekf_profile_delivery_ab_guarded_preflight/20260622T154141Z_wheel_imu_soft_yaw_015_delivery_ab_guarded/start_guard/summary.md`.
  This proves the wrapper can safely identify the current endpoint as the
  `delivery_675235` side of the two-point route, but the stricter default still
  blocks automatic apply from this exact pose.
- No `--apply` A/B run was executed in this step. The robot was not commanded
  to move, local_state was not restarted, and no relocalization was triggered by
  the preflight-only checks.

Guarded apply result for `wheel_imu_soft_yaw_015`:

- Command executed through the guarded wrapper, not direct chassis velocity:
  `run_ekf_profile_delivery_ab_guarded.sh --profile wheel_imu_soft_yaw_015 --start-target 512355 --expected-start 675235 --max-start-xy-m 0.60 --cycles 1 --timeout-sec 180 --apply --output-root reports/ekf_profile_delivery_ab_guarded_apply`.
- Report root:
  `/workspaces/njrh-v3/workspace1/reports/ekf_profile_delivery_ab_guarded_apply/20260622T154358Z_wheel_imu_soft_yaw_015_delivery_ab_guarded/summary.md`.
- Start guard passed from the `delivery_675235` side. The script switched
  local_state to `wheel_imu_soft_yaw_015`, ran the pre-A/B relocalization, then
  sent the first guarded leg to `delivery_512355`.
- The business/navigation result for leg 1 was successful:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T154421Z_pingpong_leg1_delivery_512355/summary.md`.
  API/Nav2 ended `state=succeeded`, `phase=final_pose_verified`,
  `nav2_result_code=4`, with online final error `0.152056 m / 0.037616 rad`.
- The EKF A/B gate still rejected the profile after post-goal relocalization:
  `/workspaces/njrh-v3/workspace1/reports/ekf_profile_delivery_ab_guarded_apply/20260622T154358Z_wheel_imu_soft_yaw_015_delivery_ab_guarded/pingpong/20260622T154420Z_delivery_pingpong_guarded/leg1_delivery_512355_ekf_ab_validation.json`.
  Rejection metrics were true `map->base_link = 0.634293 m / 2.130352 deg`
  and `map->odom = 0.327314 m / 2.130352 deg`, exceeding the current
  acceptance gate of `0.50 m / 2.0 deg` and `0.30 m / 2.0 deg`.
- Decision: do not promote `wheel_imu_soft_yaw_015`. It is closer than the
  earlier stable `wheel_imu` replay sample, but it still leaves a real
  relocalization correction above the acceptance threshold.
- The wrapper restored the stable `wheel_imu` profile and ran restore
  relocalization successfully:
  `/workspaces/njrh-v3/workspace1/reports/ekf_profile_delivery_ab_guarded_apply/20260622T154358Z_wheel_imu_soft_yaw_015_delivery_ab_guarded/restore_relocalize/summary.md`.
  Runtime process inspection after the run showed
  `local_state_wheel_odom_ekf.yaml` feeding `robot_local_state` with
  `local_state_ekf.yaml`, i.e. the stable `wheel_imu` runtime was restored.
- A follow-up read-only preflight for `wheel_imu_soft_yaw_018` from the
  `delivery_512355` side was intentionally not applied:
  `/workspaces/njrh-v3/workspace1/reports/ekf_profile_delivery_ab_guarded_preflight/20260622T154627Z_wheel_imu_soft_yaw_018_delivery_ab_guarded/start_guard/summary.md`.
  After true relocalization, the current API map pose was
  `(-6.842629, 7.664781, 1.570796)`, which is nearest to `delivery_512355` but
  `0.701971 m` away. Running the next EKF candidate from that offset would mix
  endpoint start error into the EKF comparison, so it was stopped before any
  local_state restart or navigation goal.

Endpoint recenter blocker before the next EKF candidate:

- Stable `wheel_imu` recenter to `delivery_512355` was attempted through the
  normal API/Nav2 path:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T154845Z_wheel_imu_recenter_512_before_soft_yaw018/summary.md`.
  It failed with Nav2 result code `6`; API final error remained
  `0.619636 m / 0.058166 rad`. The post-goal relocalization correction was only
  `0.086235 m / 0.183 deg` in `map->base_link`, so the remaining endpoint
  error was not explained by a large global localization correction.
- Stable `wheel_imu` recenter to the opposite endpoint `delivery_675235` was
  also attempted:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260622T155018Z_wheel_imu_recenter_675_before_soft_yaw018/summary.md`.
  This failed after about two seconds with Nav2 result code `6` and no robot
  map-pose movement; before and after API poses were both
  `(-6.742629, 7.964781, 1.509437)`. Post-goal relocalization correction was
  effectively zero.
- Because stable `wheel_imu` could no longer obtain a clean endpoint start, no
  further moving EKF A/B candidate was run. Continuing from a `0.71 m` endpoint
  offset would mix terminal navigation/start-condition error into the EKF
  result.
- Added the next candidate profile
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_016`. It keeps the production
  `local_state_ekf.yaml` fusion fields and uses
  `local_state_wheel_odom_ekf_soft_yaw_016.yaml` to raise wheel yaw and
  yaw-rate covariance floors to `0.16`, just above the `0.15` candidate that
  nearly passed but still exceeded the true relocalization gate.
- The profile is wired in both source and Jetson runtime overlay:
  `src/robot_local_state/config/local_state_wheel_odom_ekf_soft_yaw_016.yaml`
  and
  `scripts/jetson/runtime_overlay/config/local_state_wheel_odom_ekf_soft_yaw_016.yaml`.
  `run_local_state.sh` and `run_ekf_profile_delivery_ab_guarded.sh` both accept
  `wheel_imu_soft_yaw_016` / `soft_yaw_016`.
- Read-only preflight for the new profile succeeded only with a deliberately
  loose `0.80 m` start gate:
  `/workspaces/njrh-v3/workspace1/reports/ekf_profile_delivery_ab_guarded_preflight/20260622T155417Z_wheel_imu_soft_yaw_016_delivery_ab_guarded/start_guard/summary.md`.
  This validates the wrapper/profile wiring but is not a valid moving A/B start;
  the actual pose was still `0.713798 m / 3.515612 deg` from `delivery_512355`.
- Verification after adding `wheel_imu_soft_yaw_016`:
  `python3 -m pytest src/robot_system_tests/test/test_workspace_contracts.py -k "local_state_uses_robot_localization_ekf_with_system_time_driver or ekf_ab_report_validator_rejects_dirty_navigation_samples"` passed locally and inside the Jetson container.
  `bash -n scripts/jetson/runtime_overlay/scripts/run_local_state.sh` and
  `bash -n scripts/jetson/runtime_overlay/scripts/run_ekf_profile_delivery_ab_guarded.sh`
  passed inside the Jetson container.

2026-06-23 per-leg EKF continuation:

- The test rule was tightened to one navigation leg at a time. Every leg must
  finish with post-goal relocalization before any next target is sent.
- `LOCAL_STATE_EKF_PROFILE=twist_wheel_yaw_imu` was verified to publish
  `/local_state/odometry` at about 50 Hz, but the first leg to
  `delivery_512355` failed Nav2/API completion:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T035821Z_twist_wheel_yaw_imu_delivery_512355_per_leg_01_nav_pose_error/summary.md`.
  Result code was `6`; post-goal relocalization required
  `0.6117 m / 4.30 deg` true `map->base_link` correction. Decision: reject.
- Stable `wheel_imu` was restored and relocalized after that failure:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T040327Z_wheel_imu_restore_after_twist_wheel_yaw_imu/relocalize_compare/summary.md`.
- `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_016` was then tested per leg.
  Leg 1 to `delivery_512355` passed online and truth checks:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T041136Z_wheel_imu_soft_yaw_016_delivery_512355_per_leg_01_nav_pose_error/summary.md`.
  Online final error was `0.174833 m / 0.010248 rad`; post-goal true
  `map->base_link` correction was `0.1716 m / 0.60 deg`.
- Leg 2 to `delivery_675235` showed the same failure pattern as the larger
  soft-yaw candidates:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T041328Z_wheel_imu_soft_yaw_016_delivery_675235_per_leg_02_nav_pose_error/summary.md`.
  Online final audit passed (`0.131088 m / 0.034753 rad`), but post-goal
  relocalization required `0.9245 m / 5.05 deg` true `map->base_link`
  correction, mostly lateral. Decision: reject `wheel_imu_soft_yaw_016`.
- Runtime was restored to stable `wheel_imu` and explicitly relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T041811Z_wheel_imu_restore_after_soft_yaw_016/relocalize_compare/summary.md`.
  Restore correction was `0.0 m / 0.0 deg`.
- Added `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_010` as the smallest
  default-off soft-yaw step above the stable `0.08` covariance floor. It keeps
  the production `local_state_ekf.yaml` fusion fields and uses
  `local_state_wheel_odom_ekf_soft_yaw_010.yaml` to set wheel yaw and yaw-rate
  covariance floors to `0.10`.
- Verification after adding `wheel_imu_soft_yaw_010`:
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_local_state_uses_robot_localization_ekf_with_system_time_driver -q`
  and
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_ekf_ab_report_validator_rejects_dirty_navigation_samples -q`
  passed locally. On Jetson, `bash -n` passed for
  `run_local_state.sh` and `run_ekf_profile_delivery_ab_guarded.sh`, and live
  params confirmed `pose_covariance_floor_yaw=0.10`,
  `twist_covariance_floor_vyaw=0.10`, with `/local_state/odometry` at about
  50 Hz.
- The first moving leg with `wheel_imu_soft_yaw_010` to `delivery_512355`
  failed:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T043050Z_wheel_imu_soft_yaw_010_delivery_512355_per_leg_01_nav_pose_error/summary.md`.
  Nav2 result code was `6`; post-goal relocalization required
  `0.5965 m / 0.41 deg` true `map->base_link` correction, with about
  `0.488 m` lateral correction in the before-frame left axis. Decision: reject
  `wheel_imu_soft_yaw_010`.
- Runtime was restored again to stable `wheel_imu` and explicitly relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T044116Z_wheel_imu_restore_after_soft_yaw_010/relocalize_compare/summary.md`.
  Restore correction was `0.0 m / 0.0 deg`.
- Current conclusion: `wheel_imu` remains the only runtime-safe default.
  Covariance-only EKF soft-yaw changes from `0.10` through `0.18` either fail
  online Nav2/API completion or make the online pose look good while increasing
  post-relocalization true lateral/yaw correction. Further improvement should
  not continue as blind covariance sweeping; it needs either a better motion
  model calibration input or terminal localization/controller timing work while
  keeping `wheel_imu` as rollback.

2026-06-23 yaw-rate-only `0.10` follow-up:

- Added `LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_010` as a narrower
  alternative to the rejected `wheel_imu_soft_yaw_010`. It keeps the stable
  wheel yaw-pose covariance floor at `0.08` and raises only wheel yaw-rate
  covariance to `0.10`.
- Wiring added in both source and runtime overlay:
  `src/robot_local_state/config/local_state_wheel_odom_ekf_twist_soft_yaw_010.yaml`
  and
  `scripts/jetson/runtime_overlay/config/local_state_wheel_odom_ekf_twist_soft_yaw_010.yaml`.
  `run_local_state.sh` and `run_ekf_profile_delivery_ab_guarded.sh` accept
  `wheel_imu_twist_soft_yaw_010` / `twist_soft_yaw_010`.
- Verification:
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_local_state_uses_robot_localization_ekf_with_system_time_driver src/robot_system_tests/test/test_workspace_contracts.py::test_ekf_ab_report_validator_rejects_dirty_navigation_samples -q`
  passed locally. On Jetson, `bash -n` passed for `run_local_state.sh` and
  `run_ekf_profile_delivery_ab_guarded.sh`.
- Live candidate params were confirmed before the moving test:
  `pose_covariance_floor_yaw=0.08`, `twist_covariance_floor_vyaw=0.10`, and
  `/local_state/odometry` was about 50 Hz. AMCL was gated and ready.
- Single required per-leg test from the `delivery_512355` area to
  `delivery_675235`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T045020Z_wheel_imu_twist_soft_yaw_010_delivery_675235_per_leg_01_nav_pose_error/summary.md`.
  Online final audit passed with `0.167107 m / 0.018808 rad`, but post-goal
  relocalization required `0.6258 m / 1.62 deg` true `map->base_link`
  correction, including about `0.542 m` lateral correction in the before-frame
  left axis. Decision: reject `wheel_imu_twist_soft_yaw_010`; do not run the
  reverse leg.
- Runtime was restored to stable `wheel_imu` and explicitly relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T045529Z_wheel_imu_restore_after_twist_soft_yaw_010/relocalize_compare/summary.md`.
  The restore `map->odom` correction was `0.0 m / 0.0 deg`.
- Updated conclusion: yaw-rate-only covariance changes also do not solve the
  two-point truth error. At this point, the tested EKF covariance variants
  either fail the normal navigation leg or produce unacceptable
  post-relocalization lateral correction. Keep `wheel_imu` as the runtime
  default.

2026-06-23 differential wheel-XY follow-up:

- Added default-off `LOCAL_STATE_EKF_PROFILE=wheel_xy_diff_yaw_imu`. This is
  not a covariance sweep. It keeps wheel yaw pose, wheel forward/yaw twist, and
  corrected IMU yaw-rate, but feeds wheel `x/y` only through a second
  robot_localization odom input with `odom1_differential: true`. The intent was
  to prevent absolute wheel `x/y` pose from hard-anchoring the EKF while still
  retaining incremental wheel translation.
- Wiring added in both source and runtime overlay:
  `src/robot_local_state/config/local_state_ekf_wheel_xy_diff_yaw_imu.yaml`
  and
  `scripts/jetson/runtime_overlay/config/local_state_ekf_wheel_xy_diff_yaw_imu.yaml`.
  `run_local_state.sh` accepts `wheel_xy_diff_yaw_imu` / `xy_diff_yaw_imu`.
  The guarded ping-pong wrapper was intentionally not used for the moving test;
  the active rule is one leg at a time, with post-goal relocalization before
  any next leg.
- Verification:
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_local_state_uses_robot_localization_ekf_with_system_time_driver src/robot_system_tests/test/test_workspace_contracts.py::test_ekf_ab_report_validator_rejects_dirty_navigation_samples -q`
  passed locally and inside the Jetson container. Jetson `bash -n` passed for
  `run_local_state.sh`. Live params confirmed the candidate EKF process was
  using `local_state_ekf_wheel_xy_diff_yaw_imu.yaml`, with
  `odom0_config=[false,false,false,false,false,true,true,false,false,false,false,true,false,false,false]`,
  `odom1_config=[true,true,false,false,false,false,false,false,false,false,false,false,false,false,false]`,
  `odom1_differential=true`, and `/local_state/odometry` at about 50 Hz.
- Single required per-leg test from the `delivery_675235` area to
  `delivery_512355`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T051152Z_wheel_xy_diff_yaw_imu_delivery_512355_round01_nav_pose_error/summary.md`.
  The online Nav2/API final audit passed with `0.134883 m / 0.010161 rad`, but
  post-goal relocalization required `0.8211 m / 4.10 deg` true
  `map->base_link` correction. The correction was mostly lateral:
  `left_m=-0.8062 m` in the before-frame. Decision: reject
  `wheel_xy_diff_yaw_imu` after round 1 and do not run rounds 2-3.
- Runtime was restored to stable `wheel_imu` through the full navigation chain,
  not by restarting a single service. The first restore attempt left API in
  `idle` after a local_state readiness timeout even though local_state itself
  was publishing at 50 Hz; a second full-chain stop/run restored the complete
  navigation runtime. Explicit restore relocalization then succeeded:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T052232Z_wheel_imu_restore_after_xy_diff_round01/relocalize_compare/summary.md`.
  Restore correction was `0.0 m / 0.0 deg`, `/tmp/njrh_runtime_override.env`
  was absent, and the live EKF process was back on
  `scripts/jetson/runtime_overlay/config/local_state_ekf.yaml`.
- Current conclusion remains unchanged: `wheel_imu` is the only runtime-safe
  default. Removing absolute wheel `x/y` anchoring through a differential input
  made the online goal audit look acceptable but produced an unacceptable true
  lateral correction after relocalization.

2026-06-23 wheel-only EKF control leg:

- `LOCAL_STATE_EKF_PROFILE=wheel_only` was run as a control sample, not as a
  promotion candidate. It keeps `robot_localization` as the sole
  `odom->base_link` publisher but removes IMU fusion entirely, so it tests
  whether the two-point error is introduced by the corrected IMU input.
- The full navigation chain was restarted with `wheel_only`; no single service
  was restarted in isolation. Live process inspection confirmed
  `robot_local_state` was using
  `scripts/jetson/runtime_overlay/config/local_state_ekf_wheel_only.yaml`, with
  no `imu_gyro_bias_filter` process, and `/local_state/odometry` published at
  about 50 Hz.
- Single required per-leg test from the `delivery_512355` area to
  `delivery_675235`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T052954Z_wheel_only_delivery_675235_round01_nav_pose_error/summary.md`.
  Online Nav2/API final audit passed with `0.113259 m / 0.044599 rad`, but
  post-goal relocalization required `0.7372 m / 3.52 deg` true
  `map->base_link` correction. The correction was mostly lateral:
  `left_m=-0.7134 m` in the before-frame. Decision: reject `wheel_only` as a
  solution and do not run a second wheel-only leg.
- Runtime was restored to stable `wheel_imu` through the full navigation chain
  and explicitly relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T053519Z_wheel_imu_restore_after_wheel_only_round01/relocalize_compare/summary.md`.
  Restore correction was `0.0 m / 0.0 deg`, `/tmp/njrh_runtime_override.env`
  was absent, and the live EKF process was back on
  `scripts/jetson/runtime_overlay/config/local_state_ekf.yaml`.
- Updated conclusion: the two-point lateral truth error is not caused only by
  IMU fusion. Removing IMU fusion still leaves a large true lateral correction
  while the online goal audit passes, so the remaining problem is likely at the
  boundary between odom-frame terminal convergence and map-frame truth, not a
  simple EKF sensor weighting fix.

2026-06-23 planar yaw-offset calibration candidate:

- Added default-off `LOCAL_STATE_EKF_PROFILE=wheel_imu_yaw_offset_m061`. It
  keeps the production `local_state_ekf.yaml` fusion fields, but starts the
  wheel odom preprocessor with
  `local_state_wheel_odom_ekf_yaw_offset_m061.yaml`. That config applies
  `odom_yaw_offset_rad=-0.061` and
  `rotate_odom_position_with_yaw_offset=true` after anchoring wheel odom. This
  was derived from the previous `wheel_only` control leg, where the true
  post-goal yaw correction was about `-3.52 deg`; over a roughly 10 m leg that
  magnitude can explain a 0.6-0.7 m lateral projection error.
- Verification:
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_local_state_uses_robot_localization_ekf_with_system_time_driver src/robot_system_tests/test/test_workspace_contracts.py::test_ekf_ab_report_validator_rejects_dirty_navigation_samples -q`
  passed locally and inside the Jetson container. Jetson `bash -n` passed for
  `run_local_state.sh`. Live candidate params confirmed
  `odom_yaw_offset_rad=-0.061`,
  `rotate_odom_position_with_yaw_offset=true`,
  `anchor_pose_to_first_sample=true`, and `/local_state/odometry` at about
  50 Hz.
- Single required per-leg test from the `delivery_675235` area to
  `delivery_512355`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T054555Z_yaw_offset_m061_delivery_512355_round01_nav_pose_error/summary.md`.
  Online Nav2/API final audit improved to `0.071297 m / 0.032515 rad`, but
  post-goal relocalization still required `0.6967 m / 3.27 deg` true
  `map->base_link` correction. The correction remained mostly lateral:
  `left_m=-0.6662 m` in the before-frame. Decision: reject
  `wheel_imu_yaw_offset_m061` after round 1 and do not run a second leg.
- Runtime was restored to stable `wheel_imu` through the full navigation chain
  and explicitly relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T055550Z_wheel_imu_restore_after_yaw_offset_m061_round01/relocalize_compare/summary.md`.
  Restore correction was `0.0 m / 0.0 deg`, `/tmp/njrh_runtime_override.env`
  was absent, and the live EKF process was back on
  `scripts/jetson/runtime_overlay/config/local_state_ekf.yaml`.
- Updated conclusion: the constant planar yaw offset hypothesis improves the
  online audit slightly but does not remove the true lateral correction. The
  remaining error is not solved by a single static odom yaw rotation; continuing
  EKF work should shift toward identifying whether the final map-frame truth
  discrepancy comes from terminal controller convergence, goal completion timing,
  or a path-dependent motion model bias that requires more than one scalar yaw
  offset.

2026-06-23 planar XY shear calibration candidate:

- Added default-off `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062`. It keeps
  the production `local_state_ekf.yaml` fusion fields, but starts the wheel odom
  preprocessor with `local_state_wheel_odom_ekf_xy_shear_p062.yaml`. That config
  applies `odom_position_y_to_x_shear=0.062` after anchoring wheel odom, with
  `odom_position_scale_x=1.0`, `odom_position_scale_y=1.0`, and
  `odom_position_x_to_y_shear=0.0`. This was a diagnostic planar correction
  candidate for the repeated roughly 0.6-0.7 m lateral error over about 10 m.
- Verification:
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_local_state_uses_robot_localization_ekf_with_system_time_driver src/robot_system_tests/test/test_workspace_contracts.py::test_ekf_ab_report_validator_rejects_dirty_navigation_samples -q`
  passed locally and inside the Jetson container. Jetson `bash -n` passed for
  `run_local_state.sh`, and `colcon build --packages-select robot_local_state`
  completed successfully after adding the C++ planar calibration parameters.
  Live candidate params confirmed `odom_position_y_to_x_shear=0.062` and
  `/local_state/odometry` at about 50 Hz.
- Runtime activation note: an accidental container-side write created a root
  owned `/tmp/njrh_runtime_override.env`; it was removed through container root
  before candidate testing. The reliable candidate start path was direct host
  environment export:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062` and
  `NJRH_LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062` on the full
  `njrh_systemd_runtime.sh stop/run` chain.
- Required single-leg discipline was followed. Before the leg, AMCL gated was
  `AMCL_WAITING_SEED`, so one explicit static relocalization was run only to
  seed the gated localizer:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T061742_wheel_imu_xy_shear_p062_preseed_relocalize/summary.md`.
  The correction was `0.0 m / 0.0 deg`, then readiness showed
  `AMCL_READY`, `safe_for_goal_start=true`, no duplicate critical nodes, and
  the live shear param still `0.062`.
- Single attempted leg from the `delivery_512355` area to `delivery_675235`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T062135Z_xy_shear_p062_delivery_675235_round01_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_odom_xy_trace/20260623T062132Z_xy_shear_p062_delivery_675235_round01/summary.md`.
  This leg is invalid for odom evaluation: Nav2 returned result code `6`
  after BT Navigator timed out waiting for `compute_path_to_pose` action-server
  acknowledgement. The robot did not move: all command-chain nonzero counts were
  `0`, `max_abs_imu_wz=0.0`, odom deltas were `0`, and final distance remained
  `10.908163 m`. The required post-leg relocalization still ran and accepted
  with `0.0 m / 0.0 deg` correction:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T062135Z_xy_shear_p062_delivery_675235_round01_nav_pose_error/post_relocalize_compare/summary.md`.
- Runtime was restored to stable `wheel_imu` through the full navigation chain
  and explicitly relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T062438_wheel_imu_restore_after_xy_shear_p062_invalid_relocalize/summary.md`.
  Restore correction was effectively `0.0 m / 0.0 deg`, AMCL returned
  `AMCL_READY`, `safe_for_goal_start=true`, no duplicate nodes were reported,
  and the live wheel odom preprocessor parameter was back to
  `odom_position_y_to_x_shear=0.0`.
- Decision: `wheel_imu_xy_shear_p062` is not accepted or rejected by motion
  evidence yet, because the attempted leg produced no motion. It remains a
  default-off diagnostic candidate only. The next valid test must first confirm
  planner action-server readiness, then run exactly one leg and end with
  post-leg relocalization before any next leg.

2026-06-23 planar XY shear retry and startup guard:

- Before moving the robot again, a planner-only preflight was run on the stable
  `wheel_imu` runtime. It sent a `ComputePathToPose` action goal to the farther
  endpoint, `delivery_675235`, without publishing velocity. The action server
  acknowledged the goal and returned success (`status=4`) with `218` path poses
  and about `0.068 s` planning time. This confirms the previous
  `compute_path_to_pose` acknowledgement timeout was not a persistent planner
  action-server outage at the time of preflight.
- A new `wheel_imu_xy_shear_p062` retry was attempted under the strict
  single-leg rule, but no navigation leg was executed. Direct environment
  profile passing was unreliable in this startup sequence, so the runtime was
  restarted with the supported `/tmp/njrh_runtime_override.env` mechanism:
  `NJRH_LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062` and
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062`. Process inspection showed
  those values did reach the host `docker exec` environment.
- The candidate startup did not reach a valid runtime state. API was unavailable
  during the failed candidate run, `/wheel_odom_ekf_input` was not present, and
  the resident navigation log showed startup failure around selected-map
  activation (`/map_server did not appear within 45s`) plus common local-state
  availability not being established. Because the robot was never moved, this
  retry provides no odom evidence and does not change the EKF conclusion.
- The override was removed and the stable `wheel_imu` runtime was restored
  through the full navigation chain, not by restarting individual services.
  Duplicate safety-chain nodes observed during an intermediate restore were
  cleared by another full `njrh_systemd_runtime.sh stop/run`.
- Explicit restore relocalization succeeded:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T064516_wheel_imu_restore_after_xy_shear_startup_fail_relocalize/summary.md`.
  Correction was `0.0 m / 0.0 deg`. Final live state after restore:
  `/tmp/njrh_runtime_override.env` absent on host and container,
  `AMCL_READY`, `safe_for_goal_start=true`, no duplicate nodes,
  `/wheel_odom_ekf_input odom_position_y_to_x_shear=0.0`,
  `/local_state/odometry` about `50 Hz`, and `robot_local_state` using
  `local_state_ekf.yaml`.
- Decision: do not attempt another moving EKF leg until the runtime can be
  switched to the candidate profile and pass three preflight checks in the same
  started runtime: candidate param live, no duplicate critical nodes, and
  planner-only `ComputePathToPose` success. The strict field rule remains one
  navigation leg at a time followed by post-leg relocalization before any next
  leg.

2026-06-23 planar XY shear valid single-leg sample:

- The candidate startup issue was isolated to resident navigation starting
  before the common local-state EKF during profile switching. The valid retry
  used the same full `njrh_systemd_runtime.sh stop/run` path, but set the
  runtime override to:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062`,
  `NJRH_LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062`, and
  `NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=false`.
  This keeps the test full-chain while letting `/wheel_odom_ekf_input` and
  `robot_local_state` come up before resident navigation.
- In the same candidate runtime, preflight passed before any movement:
  live `odom_position_y_to_x_shear=0.062`, no duplicate critical nodes,
  `AMCL_READY`, `safe_for_goal_start=true`, and a planner-only
  `ComputePathToPose` action to `delivery_675235` succeeded with `218` path
  poses (`status=4`, about `0.102 s` planning time).
- Single required leg from the `delivery_512355` area to `delivery_675235`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T070201Z_xy_shear_p062_delivery_675235_round02_localstate_first_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_odom_xy_trace/20260623T070158Z_xy_shear_p062_delivery_675235_round02_localstate_first/summary.md`.
  This was a valid moving sample: Nav2 succeeded (`result_code=4`), final online
  audit was `0.10312 m / 0.061338 rad`, and the command chain produced normal
  nonzero commands with max `/cmd_vel` through the safety chain of `1.2 m/s`
  and `0.85 rad/s`.
- Required post-leg relocalization was executed immediately:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T070201Z_xy_shear_p062_delivery_675235_round02_localstate_first_nav_pose_error/post_relocalize_compare/summary.md`.
  True `map->base_link` correction was `0.4415 m / 1.826 deg`
  (`forward_m=-0.1715`, `left_m=0.4069`). `map->odom` correction was
  `0.1722 m / 1.826 deg`. This passes the current single-leg acceptance gates
  (`map_base <= 0.50 m`, `map_odom <= 0.30 m`, yaw <= `2 deg`), but it is only
  one direction and cannot promote the candidate by itself.
- Runtime was restored to stable `wheel_imu` through the full navigation chain,
  override removed, and explicit restore relocalization succeeded:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T070447_wheel_imu_restore_after_xy_shear_valid_round02_relocalize/summary.md`.
  Restore correction was `0.1000 m / 0.0 deg`. Final live state after restore:
  override absent on host and container, `AMCL_READY`,
  `safe_for_goal_start=true`, no duplicate nodes,
  `/wheel_odom_ekf_input odom_position_y_to_x_shear=0.0`, and
  `/local_state/odometry` at about `50 Hz`.
- Decision: `wheel_imu_xy_shear_p062` is the first EKF candidate in this series
  to pass a valid 10 m single leg against the post-relocalization truth gates.
  It remains default-off until the reverse leg (`delivery_675235` to
  `delivery_512355`) is tested under the same one-leg-then-relocalize rule.

2026-06-23 planar XY shear reverse-leg rejection:

- The reverse leg was run as a separate single-leg check under the same
  candidate runtime method: full `njrh_systemd_runtime.sh stop/run` with
  `/tmp/njrh_runtime_override.env` setting
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062`,
  `NJRH_LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062`, and
  `NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=false`.
  Candidate preflight passed in that same runtime: live
  `odom_position_y_to_x_shear=0.062`, no duplicate critical nodes,
  `AMCL_READY`, `safe_for_goal_start=true`, and planner-only
  `ComputePathToPose` to `delivery_512355` succeeded with `216` path poses
  (`status=4`, about `0.140 s` planning time).
- Single reverse leg from the `delivery_675235` area to `delivery_512355`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T071416Z_xy_shear_p062_delivery_512355_round03_reverse_localstate_first_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_odom_xy_trace/20260623T071413Z_xy_shear_p062_delivery_512355_round03_reverse_localstate_first/summary.md`.
  This was a valid moving sample: Nav2 succeeded (`result_code=4`), final online
  audit was `0.162446 m / 0.069628 rad`, and the command chain produced normal
  nonzero commands (`174` safety-chain samples, max `1.2 m/s` and
  `0.85 rad/s`).
- Required post-leg relocalization was executed immediately:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T071416Z_xy_shear_p062_delivery_512355_round03_reverse_localstate_first_nav_pose_error/post_relocalize_compare/summary.md`.
  True `map->base_link` correction was `0.9470 m / -3.952 deg`
  (`forward_m=-0.1975`, `left_m=-0.9262`). `map->odom` correction was
  `0.2887 m / -3.952 deg`. The result fails the current gates because
  `map_base` is greater than `0.50 m` and yaw is greater than `2 deg`, even
  though `map_odom` translation is still within `0.30 m`.
- Runtime was restored to stable default `wheel_imu` through the full navigation
  chain, override removed, and explicit restore relocalization succeeded:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T072329Z_wheel_imu_restore_after_xy_shear_reverse_fail_relocalize/summary.md`.
  Restore relocalization translation was `0.0 m`; yaw adjustment was about
  `0.703 deg`. Final live state after restore: override absent in the
  container, `AMCL_READY`, `amcl_mode=gated`, `safe_for_goal_start=true`, no
  duplicate nodes, `/wheel_odom_ekf_input odom_position_y_to_x_shear=0.0`, and
  `/local_state/odometry` at about `50 Hz`.
- Decision: reject `wheel_imu_xy_shear_p062` as a promotable correction. It
  improved one direction but failed the reverse direction with a large lateral
  post-relocalization correction, so it is an asymmetric one-direction patch,
  not a stable odom fix.

2026-06-23 body-lateral XY shear first valid leg:

- Added default-off `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m061`. It
  keeps the production `local_state_ekf.yaml` fusion fields, but starts the
  wheel odom preprocessor with
  `local_state_wheel_odom_ekf_xy_lateral_m061.yaml`. That config applies
  `odom_position_x_to_y_shear=-0.061` after anchoring the wheel odom pose.
  The hypothesis is path-length-proportional body-frame lateral projection:
  stable `wheel_imu` runs repeatedly showed about `0.5-0.9 m` lateral truth
  correction over 10 m class legs, while covariance-only and yaw-only changes
  did not solve the two-point drift.
- Local and Jetson verification passed before field movement:
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_local_state_uses_robot_localization_ekf_with_system_time_driver -q`
  and Jetson `colcon build --packages-select robot_local_state --cmake-args -DCMAKE_BUILD_TYPE=Release`.
- The first candidate startup reached live profile params
  (`odom_position_x_to_y_shear=-0.061`, `odom_position_y_to_x_shear=0.0`) but
  did not become navigation-safe because AMCL reported
  `AMCL_HEARTBEAT_PROCESS_NOT_ALIVE` and `/amcl_scan_admission/status` had no
  publisher. No navigation goal was sent. The runtime was restored to stable
  `wheel_imu` through the full chain and explicitly relocalized:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T084156Z_wheel_imu_restore_after_xy_lateral_startup_fail_relocalize/summary.md`.
  Restore correction was `0.0 m / 0.0 deg`.
- The second candidate startup succeeded with `AMCL_READY` and
  `safe_for_goal_start=true`. Live candidate checks showed no persistent
  duplicate nodes, `/local_state/odometry` about `50 Hz`, and
  `/wheel_odom_ekf_input odom_position_x_to_y_shear=-0.061`.
  A hand-written planner-only preflight used the wrong Humble action field and
  then a hand-filled coordinate, so it was discarded as a non-motion setup
  error; the actual navigation test used the saved API pose id.
- Single required leg from the `delivery_512355` area to `delivery_675235`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T084826Z_xy_lateral_m061_delivery_675235_round01_localstate_first_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_odom_xy_trace/20260623T084824Z_xy_lateral_m061_delivery_675235_round01_localstate_first/summary.md`.
  This was a valid moving sample: Nav2 succeeded (`result_code=4`), final online
  audit was `0.08848 m / 0.07773 rad`, with the API explicitly noting a yaw
  audit warning but still `final_pose_verified=True` under the current
  `pose_required` gate. The command chain was normal: `185` safety-chain
  nonzero samples with max `1.2 m/s` and `0.85 rad/s`; motion modes seen were
  `0` and `2`.
- Required post-leg relocalization was executed immediately:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T084826Z_xy_lateral_m061_delivery_675235_round01_localstate_first_nav_pose_error/post_relocalize_compare/summary.md`.
  True `map->base_link` correction was `0.2007 m / -1.974 deg`
  (`forward_m=-0.1872`, `left_m=0.0723`). This is the best true-base result
  seen in the two-point delivery tests so far and clears the current single-leg
  truth gate. `map->odom` correction was `0.4711 m / -1.974 deg`, which is below
  the first-improvement `0.50 m` level but still above the desired `0.30 m`
  target. The trace also shows the preprocessor doing the intended correction:
  final `/wheel/odom` y was about `-0.9197 m`, while `/wheel/odom_ekf` and
  `/local_state/odometry` y were about `-0.2677 m`.
- Runtime was restored to stable default `wheel_imu` through the full navigation
  chain, override removed, and explicit restore relocalization succeeded:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T085249Z_wheel_imu_restore_after_xy_lateral_valid_round01_relocalize/summary.md`.
  Restore correction was `0.0 m / 0.0 deg`. Final live state after restore:
  override absent, `AMCL_READY`, `safe_for_goal_start=true`, no duplicate nodes,
  `/wheel_odom_ekf_input odom_position_x_to_y_shear=0.0`,
  `/wheel_odom_ekf_input odom_position_y_to_x_shear=0.0`, and
  `/local_state/odometry` at about `50 Hz`.
- Decision: keep `wheel_imu_xy_lateral_m061` default-off, but continue testing.
  It is not promotable from one direction only. The next required field check is
  the reverse leg, `delivery_675235 -> delivery_512355`, with the same
  one-leg-then-post-relocalize rule.

2026-06-23 body-lateral XY shear reverse-leg result:

- The reverse leg was run as a separate single-leg check after the previous leg
  had completed post-relocalization and the runtime had been restored to stable
  `wheel_imu`. The candidate was started again through the full
  `njrh_systemd_runtime.sh stop/run` chain with
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m061`,
  `NJRH_LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m061`, and
  `NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=false`.
  Candidate startup reached `AMCL_READY` and `safe_for_goal_start=true`; live
  checks showed `/wheel_odom_ekf_input odom_position_x_to_y_shear=-0.061`,
  `odom_position_y_to_x_shear=0.0`, and `/local_state/odometry` about `50 Hz`.
- Single reverse leg from the `delivery_675235` area to `delivery_512355`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T085747Z_xy_lateral_m061_delivery_512355_round02_reverse_localstate_first_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_odom_xy_trace/20260623T085746Z_xy_lateral_m061_delivery_512355_round02_reverse_localstate_first/summary.md`.
  This was a valid moving sample: Nav2 succeeded (`result_code=4`), final online
  audit was `0.091353 m / 0.042337 rad`, and the command chain was normal
  (`181` safety-chain nonzero samples, max `1.2 m/s` and `0.85 rad/s`).
- Required post-leg relocalization was executed immediately:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T085747Z_xy_lateral_m061_delivery_512355_round02_reverse_localstate_first_nav_pose_error/post_relocalize_compare/summary.md`.
  True `map->base_link` correction was `0.5359 m / -3.823 deg`
  (`forward_m=-0.0654`, `left_m=-0.5319`). `map->odom` correction was
  `0.2043 m / -3.823 deg`. This is a meaningful improvement over the earlier
  reverse `xy_shear_p062` result (`0.9470 m / -3.952 deg`) and over the stable
  wheel-anchored reverse samples in translation, but it still fails the current
  truth gate because `map_base` is above `0.50 m` and yaw is above `2 deg`.
  The trace shows the preprocessor correction was active: final `/wheel/odom`
  y was about `-1.4269 m`, while `/wheel/odom_ekf` and `/local_state/odometry`
  y were about `-0.7702 m`.
- Runtime was restored to stable default `wheel_imu` through the full navigation
  chain, override removed, and explicit restore relocalization succeeded:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T090350Z_wheel_imu_restore_after_xy_lateral_reverse_round02_relocalize/summary.md`.
  Restore correction was `0.1000 m / 0.0 deg`. Final live state after restore:
  override absent, `AMCL_READY`, `safe_for_goal_start=true`, no duplicate nodes,
  `/wheel_odom_ekf_input odom_position_x_to_y_shear=0.0`,
  `/wheel_odom_ekf_input odom_position_y_to_x_shear=0.0`, and
  `/local_state/odometry` at about `50 Hz`.
- Decision: do not promote `wheel_imu_xy_lateral_m061`. It is the best
  directionally useful calibration candidate so far, but it does not pass the
  reverse-leg truth gate. The next candidate should keep the physically
  meaningful body-lateral correction family, but it must also address the
  reverse yaw residual; a pure `x -> y` coefficient alone is unlikely to make
  both directions pass.

2026-06-23 body-lateral plus signed yaw-scale candidate result:

- Candidate `wheel_imu_xy_lateral_yaw_p979_n1011` was added default-off. It
  keeps the body-lateral preprocessor correction
  `odom_position_x_to_y_shear=-0.061` and adds signed yaw scaling in the same
  `/wheel/odom_ekf` input preprocessor:
  `odom_yaw_scale_positive=0.979`, `odom_yaw_scale_negative=1.011`, and
  `scale_odom_twist_with_yaw_scale=true`. The intent was to reduce the
  direction-dependent yaw residual observed after the valid `m061` forward and
  reverse legs.
- Local and Jetson contract tests passed:
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_local_state_uses_robot_localization_ekf_with_system_time_driver -q`.
  Jetson `robot_local_state` was clean-built after clearing only
  `build/robot_local_state` and `install/robot_local_state`; the installed
  runtime binary was verified to contain the new yaw-scale parameters before
  movement. The candidate full-chain startup reached `AMCL_READY` and
  `safe_for_goal_start=true`; live checks showed
  `/wheel_odom_ekf_input odom_position_x_to_y_shear=-0.061`,
  `odom_yaw_scale_positive=0.979`, `odom_yaw_scale_negative=1.011`, no
  duplicate ROS nodes, and `/local_state/odometry` about `50 Hz`.
- A single forward leg was run from the `delivery_512355` area to
  `delivery_675235`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T093204Z_xy_lateral_yaw_p979_n1011_delivery_675235_round01_localstate_first_nav_pose_error/summary.md`.
  Nav2 succeeded (`result_code=4`) and the API online final audit was
  `0.101435 m / 0.037784 rad`. The companion terminal-yaw trace report
  `/workspaces/njrh-v3/workspace1/reports/navigation_odom_xy_trace/20260623T093202Z_xy_lateral_yaw_p979_n1011_delivery_675235_round01_localstate_first/summary.md`
  is not usable because the wrapper command accidentally exported a literal
  non-numeric `ROS_DOMAIN_ID`, so its ROS sampler produced `sample_count=0` and
  skipped automatic post-goal relocalization. The navigation leg itself was
  completed before that failure.
- Required post-leg relocalization was therefore executed immediately as a
  manual correction capture with a valid `ROS_DOMAIN_ID=0`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T093204Z_xy_lateral_yaw_p979_n1011_delivery_675235_round01_localstate_first_nav_pose_error/post_relocalize_compare/summary.md`.
  True `map->base_link` correction was `0.9109 m / 3.954 deg`
  (`forward_m=-0.1828`, `left_m=0.8923`). `map->odom` correction was
  `0.1910 m / 3.954 deg`. This is much worse than the previous
  `wheel_imu_xy_lateral_m061` forward truth result (`0.2007 m / -1.974 deg`),
  so the signed yaw-scale candidate failed on the first required leg. The
  reverse leg was intentionally not run.
- Runtime was restored to stable default `wheel_imu` through the full
  navigation chain, override removed, and explicit restore relocalization
  succeeded:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T093907Z_wheel_imu_restore_after_xy_lateral_yaw_round01/relocalize_compare/summary.md`.
  Restore correction was `0.0 m / 0.0 deg`. Final live state after restore:
  override absent, `AMCL_READY`, `safe_for_goal_start=true`, no duplicate
  nodes, `/wheel_odom_ekf_input odom_position_x_to_y_shear=0.0`,
  `odom_yaw_scale_positive=1.0`, and `odom_yaw_scale_negative=1.0`.
- Decision: reject `wheel_imu_xy_lateral_yaw_p979_n1011`. Do not promote or
  continue with reverse testing. Directly scaling the integrated wheel yaw in
  this form makes the forward truth result much worse even though the online
  Nav2 audit still looks acceptable. Keep the current runtime on stable
  `wheel_imu`; keep `wheel_imu_xy_lateral_m061` only as a default-off diagnostic
  reference.

2026-06-23 body-lateral plus 0.16 soft-yaw candidate result:

- Candidate `wheel_imu_xy_lateral_soft_yaw_016` was added default-off. It
  combines the useful body-lateral preprocessor correction from
  `wheel_imu_xy_lateral_m061` (`odom_position_x_to_y_shear=-0.061`) with the
  0.16 wheel yaw and yaw-rate covariance floors from `wheel_imu_soft_yaw_016`.
  The intent was to preserve the lateral correction evidence while testing
  whether the reverse-leg yaw residual could be reduced without directly
  scaling integrated wheel yaw.
- Local and Jetson contract tests passed:
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_local_state_uses_robot_localization_ekf_with_system_time_driver -q`.
  Jetson `robot_local_state` was rebuilt and the new config was verified in the
  installed package share directory before movement. The candidate full-chain
  startup reached `AMCL_READY` and `safe_for_goal_start=true`; live checks
  showed `/wheel_odom_ekf_input odom_position_x_to_y_shear=-0.061`,
  `pose_covariance_floor_yaw=0.16`, `twist_covariance_floor_vyaw=0.16`,
  yaw scales `1.0/1.0`, no duplicate ROS nodes, and `/local_state/odometry`
  about `50 Hz`.
- A single reverse leg was run from the current `delivery_675235` area to
  `delivery_512355`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T100417Z_xy_lateral_soft_yaw_016_delivery_512355_round01_reverse_localstate_first_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_odom_xy_trace/20260623T100416Z_xy_lateral_soft_yaw_016_delivery_512355_round01_reverse_localstate_first/summary.md`.
  Nav2 succeeded (`result_code=4`), online final audit was
  `0.141888 m / 0.057167 rad`, and the command chain was normal with `196`
  safety-chain nonzero samples, max `1.2 m/s` and `0.85 rad/s`.
- Required post-leg relocalization ran automatically:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T100417Z_xy_lateral_soft_yaw_016_delivery_512355_round01_reverse_localstate_first_nav_pose_error/post_relocalize_compare/summary.md`.
  True `map->base_link` correction was `0.2999 m / -5.391 deg`
  (`forward_m=0.0691`, `left_m=-0.2918`). `map->odom` correction was
  `0.7181 m / -5.391 deg`. Translation is better than
  `wheel_imu_xy_lateral_m061` on the same reverse side (`0.5359 m`), but yaw is
  much worse than both `m061` (`-3.823 deg`) and the earlier soft-yaw-only
  reverse sample (`-0.599 deg`). This fails the truth gate and should not be
  promoted.
- Runtime was restored to stable default `wheel_imu` through the full
  navigation chain, override removed, and explicit restore relocalization
  succeeded:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T101045Z_wheel_imu_restore_after_xy_lateral_soft_yaw_016_round01/relocalize_compare/summary.md`.
  Restore correction was `0.0 m / 0.0 deg`. Final live state after restore:
  override absent, `AMCL_READY`, `safe_for_goal_start=true`, no duplicate nodes,
  `/wheel_odom_ekf_input odom_position_x_to_y_shear=0.0`,
  `pose_covariance_floor_yaw=0.08`, `twist_covariance_floor_vyaw=0.08`, and yaw
  scales `1.0/1.0`.
- Decision: reject `wheel_imu_xy_lateral_soft_yaw_016` as a promotable profile.
  It improves reverse translation but makes yaw too poor. The next useful
  search direction is not more yaw covariance on top of `m061`; instead test a
  stronger pure body-lateral shear step while keeping the stable yaw covariance,
  because the `xy_shear_p062 -> xy_lateral_m061` sequence improved lateral
  error in both directions.

2026-06-23 stronger pure body-lateral shear candidate result:

- Candidate `wheel_imu_xy_lateral_m120` was added default-off. It keeps the
  stable EKF fusion fields and stable wheel yaw/yaw-rate covariance floors
  (`0.08`), but changes the anchored wheel odom preprocessor to
  `odom_position_x_to_y_shear=-0.120`. This tests whether the lateral trend
  that improved from `+0.062 y->x` to `-0.061 x->y` continues with a stronger
  pure body-lateral correction, without changing yaw fusion behavior.
- Local and Jetson contract tests passed:
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_local_state_uses_robot_localization_ekf_with_system_time_driver -q`.
  Jetson `robot_local_state` was rebuilt and the new config was verified in the
  installed package share directory before movement. The candidate full-chain
  startup reached `AMCL_READY` and `safe_for_goal_start=true`; live checks
  showed `/wheel_odom_ekf_input odom_position_x_to_y_shear=-0.120`,
  `pose_covariance_floor_yaw=0.08`, `twist_covariance_floor_vyaw=0.08`, yaw
  scales `1.0/1.0`, no duplicate ROS nodes, and `/local_state/odometry` about
  `50 Hz`.
- A single forward leg was run from the current `delivery_512355` area to
  `delivery_675235`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T102536Z_xy_lateral_m120_delivery_675235_round01_forward_localstate_first_nav_pose_error/summary.md`.
  Nav2 succeeded (`result_code=4`) and the API online final audit was very good
  at `0.138432 m / 0.001328 rad`. The terminal-yaw trace wrapper got stuck in
  its post-goal motion-settle subprocess after the goal had already completed,
  so the wrapper was killed and its raw trace directory was moved to the normal
  path:
  `/workspaces/njrh-v3/workspace1/reports/navigation_odom_xy_trace/20260623T102533Z_xy_lateral_m120_delivery_675235_round01_forward_localstate_first`.
  No completed trace summary should be used for this run.
- Required post-leg relocalization was executed manually immediately after
  clearing the stuck test wrapper:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T102536Z_xy_lateral_m120_delivery_675235_round01_forward_localstate_first_nav_pose_error/post_relocalize_compare/summary.md`.
  True `map->base_link` correction was `0.5577 m / -4.172 deg`
  (`forward_m=0.1823`, `left_m=0.5270`). `map->odom` correction was
  `1.3296 m / -4.172 deg`. This is worse than `wheel_imu_xy_lateral_m061` on
  the same forward side (`0.2007 m / -1.974 deg`) and fails both the translation
  and yaw truth gates. It also indicates `-0.120` over-corrects the forward
  lateral component.
- Runtime was restored to stable default `wheel_imu` through the full
  navigation chain, override removed, and explicit restore relocalization
  succeeded:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T104239Z_wheel_imu_restore_after_xy_lateral_m120_round01/relocalize_compare/summary.md`.
  Restore correction was `0.0 m / 0.0 deg`. Final live state after restore:
  override absent, `AMCL_READY`, `safe_for_goal_start=true`, no duplicate nodes,
  `/wheel_odom_ekf_input odom_position_x_to_y_shear=0.0`,
  `pose_covariance_floor_yaw=0.08`, and `twist_covariance_floor_vyaw=0.08`.
- Decision: reject `wheel_imu_xy_lateral_m120`. Do not run reverse. The useful
  region, if any, is between `-0.061` and `-0.120`; stronger pure lateral shear
  is not the answer. Current runtime remains stable `wheel_imu`.

2026-06-23 mid pure body-lateral shear candidate result:

- Candidate `wheel_imu_xy_lateral_m085` was added default-off. It keeps the
  stable yaw/yaw-rate covariance floors (`0.08`) and yaw scales (`1.0/1.0`),
  but changes the anchored wheel odom preprocessor to
  `odom_position_x_to_y_shear=-0.085`. This tests the midpoint between the
  earlier useful `-0.061` lateral correction and the over-correcting `-0.120`
  candidate.
- Local and Jetson contract tests passed:
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_local_state_uses_robot_localization_ekf_with_system_time_driver -q`.
  Jetson `robot_local_state` was rebuilt and the installed config verified
  before movement. Candidate full-chain startup reached `AMCL_READY` and
  `safe_for_goal_start=true`; live checks showed
  `/wheel_odom_ekf_input odom_position_x_to_y_shear=-0.085`,
  `pose_covariance_floor_yaw=0.08`, `twist_covariance_floor_vyaw=0.08`, and
  yaw scales `1.0/1.0`.
- Before running the formal leg, the currently displayed pose near the
  `delivery_675235` side was checked with an explicit relocalization:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T111538Z_wheel_imu_xy_lateral_m085_current_pose_relocalize/relocalize_compare/summary.md`.
  That visual check looked good and the true `map->base_link` correction was
  only `0.0943 m / 0.703 deg` (`forward_m=-0.0938`, `left_m=-0.0092`), but the
  robot was still about `0.42 m` from the saved `delivery_675235` pose, so this
  was not treated as a formal pass.
- A single formal reverse leg was then run from the current `delivery_675235`
  area to `delivery_512355`:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T111848Z_xy_lateral_m085_delivery_512355_round01_reverse_localstate_first_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_odom_xy_trace/20260623T111845Z_xy_lateral_m085_delivery_512355_round01_reverse_localstate_first/summary.md`.
  Nav2 succeeded (`result_code=4`) and the online final audit before
  relocalization was `0.158657 m / 0.040870 rad`. The command chain was normal
  with `193` nonzero safety-chain samples, max `1.2 m/s` and `0.85 rad/s`;
  motion modes seen were `0` and `2`.
- Required post-leg relocalization ran automatically:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T111848Z_xy_lateral_m085_delivery_512355_round01_reverse_localstate_first_nav_pose_error/post_relocalize_compare/summary.md`.
  True `map->base_link` correction was `0.3001 m / -6.564 deg`
  (`forward_m=0.0792`, `left_m=-0.2894`). `map->odom` correction was
  `0.9672 m / -6.564 deg`. After the relocalization, the trace reported the
  corrected API pose around `(-7.042628, 7.964781, 1.435806)`, which is still
  about `0.4147 m / 7.734 deg` from the saved `delivery_512355` target. This
  fails the truth gate even though the online Nav2 audit had accepted the goal.
- Runtime was restored to stable default `wheel_imu` through the full
  navigation chain, override removed, and explicit restore relocalization
  succeeded:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T113434Z_wheel_imu_restore_after_xy_lateral_m085_round01/relocalize_compare/summary.md`.
  Restore correction was `0.0 m / -0.703 deg`. Final live state after restore:
  override absent, `AMCL_READY`, `safe_for_goal_start=true`,
  `/wheel_odom_ekf_input odom_position_x_to_y_shear=0.0`,
  `pose_covariance_floor_yaw=0.08`, and yaw scales `1.0/1.0`.
- Decision: reject `wheel_imu_xy_lateral_m085` as a promotable profile and do
  not run the forward leg. It improves the reverse-side translation compared
  with `m061`, but yaw becomes worse than the 5 degree target and the corrected
  true target error remains about `41 cm`. Current runtime remains stable
  default `wheel_imu`.

2026-06-23 smaller pure body-lateral shear candidate result:

- Candidate `wheel_imu_xy_lateral_m040` was added default-off. It keeps the
  stable yaw/yaw-rate covariance floors (`0.08`) and yaw scales (`1.0/1.0`),
  but reduces the anchored wheel odom preprocessor correction to
  `odom_position_x_to_y_shear=-0.040`. The intent was to test a smaller
  lateral correction than `m061` after `m085` and `m120` showed yaw or
  translation regressions.
- Local and Jetson contract tests passed:
  `python -m pytest src/robot_system_tests/test/test_workspace_contracts.py::test_local_state_uses_robot_localization_ekf_with_system_time_driver -q`.
  Jetson `robot_local_state` was rebuilt and the installed/runtime configs were
  verified before movement. The candidate runtime reached `AMCL_READY`,
  `safe_for_goal_start=true`, and `/local_state/odometry` about `50 Hz`; live
  candidate checks showed `/wheel_odom_ekf_input odom_position_x_to_y_shear=-0.040`
  and `odom_position_y_to_x_shear=0.0`.
- The forward leg from the `delivery_512355` area to `delivery_675235` was a
  valid moving sample:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T121105Z_xy_lateral_m040_delivery_675235_round01_forward_localstate_first_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_odom_xy_trace/20260623T121102Z_xy_lateral_m040_delivery_675235_round01_forward_localstate_first/summary.md`.
  Nav2 succeeded (`result_code=4`), online final audit was
  `0.135060 m / 0.055776 rad`, and the command chain was normal with `186`
  safety-chain nonzero samples, max `1.2 m/s` and `0.85 rad/s`.
  Required post-leg relocalization reported true `map->base_link` correction
  `0.1807 m / -2.065 deg` (`forward_m=-0.0134`, `left_m=-0.1802`) and
  `map->odom` correction `0.2148 m / -2.065 deg`. This is a reasonable
  forward result, roughly comparable to `m061` forward.
- The first reverse attempt to `delivery_512355` is not usable as odom
  evidence:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T121421Z_xy_lateral_m040_delivery_512355_round02_reverse_localstate_first_nav_pose_error/summary.md`.
  Nav2 returned result code `6`, the robot did not move, all command-chain
  nonzero counts were `0`, and the log showed a transient BT/Nav2 failure:
  `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose`.
- The reverse retry was a valid moving sample:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T121803Z_xy_lateral_m040_delivery_512355_round03_reverse_retry_localstate_first_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_odom_xy_trace/20260623T121800Z_xy_lateral_m040_delivery_512355_round03_reverse_retry_localstate_first/summary.md`.
  Nav2 succeeded (`result_code=4`), online final audit was
  `0.170614 m / 0.070109 rad`, and the command chain was normal with `186`
  safety-chain nonzero samples, max `1.2 m/s` and `0.85 rad/s`. Required
  post-leg relocalization reported true `map->base_link` correction
  `0.7324 m / 2.923 deg` (`forward_m=-0.0808`, `left_m=0.7280`) and
  `map->odom` correction `0.7337 m / 2.923 deg`. This fails the reverse truth
  gate even though the online Nav2 audit accepted the goal.
- Runtime was restored to stable default `wheel_imu`. During restore, the
  runtime script was minimally extended to pass already-supported local-state
  readiness and initial-localization wait overrides into the container; this
  does not change defaults, but allows full-chain startup to avoid the 12 s
  local_state readiness false negative seen during this test. The synced Jetson
  script passed `bash -n`. The final runtime was brought back through the full
  `njrh_systemd_runtime.sh run` entrypoint after local_state stabilized.
  Final live state after restore: API `state=running`, `healthy=true`,
  `amcl_mode=gated`, `AMCL_READY`, `safe_for_goal_start=true`,
  `/wheel_odom_ekf_input odom_position_x_to_y_shear=0.0`,
  `/wheel_odom_ekf_input odom_position_y_to_x_shear=0.0`, and
  `/local_state/odometry` at about `50 Hz`.
- Explicit restore relocalization succeeded:
  `/workspaces/njrh-v3/workspace1/reports/imu_ekf_delivery_ab/20260623T125002Z_wheel_imu_restore_after_xy_lateral_m040_round01/relocalize_compare/summary.md`.
  Restore correction was `0.0 m / 0.0 deg` across `map->base_link`,
  `map->odom`, `odom->base_link`, `/wheel/odom`, and `/local_state/odometry`.
- Decision: reject `wheel_imu_xy_lateral_m040` as a promotable profile. It can
  look visually good at one endpoint and it passed the forward side, but the
  valid reverse retry needed about `73 cm` true correction, almost entirely
  lateral. The current runtime remains stable default `wheel_imu`.

2026-06-23 default wheel_imu root-cause trace:

- The runtime was left on stable default `LOCAL_STATE_EKF_PROFILE=wheel_imu`;
  live checks before testing showed API `healthy=true`, `AMCL_READY`, and
  `safe_for_goal_start=true`. No EKF/shear candidate was active:
  `/wheel_odom_ekf_input odom_position_x_to_y_shear=0.0`,
  `odom_position_y_to_x_shear=0.0`, `pose_covariance_floor_yaw=0.08`, and
  `twist_covariance_floor_vyaw=0.08`.
- A first attempt from the current `delivery_675235` area to
  `delivery_512355` did not move and is not odom evidence:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T133025Z_wheel_imu_rootcause_delivery_512355_01_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_terminal_yaw_trace/20260623T133022Z_wheel_imu_rootcause_delivery_512355_01/summary.md`.
  Nav2 returned result code `6`, every command-chain stage had `0` nonzero
  samples, and resident logs showed
  `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose`.
  The post-goal relocalization correction was effectively zero, confirming the
  robot had not moved and localization was stable.
- The retry to `delivery_512355` was a valid moving sample:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T133437Z_wheel_imu_rootcause_delivery_512355_retry02_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_terminal_yaw_trace/20260623T133434Z_wheel_imu_rootcause_delivery_512355_retry02/summary.md`.
  Online Nav2/API final audit passed at `0.1185 m / 0.00483 rad`, but required
  post-goal relocalization reported true `map->base_link` correction
  `0.8460 m / -3.089 deg`, mostly lateral (`left_m=-0.8146 m`). During the
  moving window, `/wheel/odom` and `/local_state/odometry` were effectively the
  same trajectory: wheel distance `10.7241 m`, local distance `10.7231 m`,
  wheel yaw `-173.395 deg`, local yaw `-173.384 deg`. Corrected IMU yaw-rate
  matched wheel yaw-rate closely: sign match `0.989`, ratio mean `1.0067`, and
  average absolute difference `0.0119 rad/s`.
- The reverse leg to `delivery_675235` was also a valid moving sample:
  `/workspaces/njrh-v3/workspace1/reports/navigation_pose_error_test/20260623T134908Z_wheel_imu_rootcause_delivery_675235_reverse01_nav_pose_error/summary.md`
  and
  `/workspaces/njrh-v3/workspace1/reports/navigation_terminal_yaw_trace/20260623T134905Z_wheel_imu_rootcause_delivery_675235_reverse01/summary.md`.
  Online Nav2/API final audit passed at `0.1780 m / 0.00276 rad`, but required
  post-goal relocalization reported true `map->base_link` correction
  `0.6481 m / -3.182 deg`, again mostly lateral (`left_m=-0.6383 m`). During
  the moving window, `/wheel/odom` and `/local_state/odometry` again overlapped:
  wheel distance `10.5335 m`, local distance `10.5367 m`, wheel yaw
  `-178.927 deg`, local yaw `-178.943 deg`. Corrected IMU yaw-rate again
  matched wheel yaw-rate: sign match `0.990`, ratio mean `1.0575`, and average
  absolute difference `0.0148 rad/s`.
- The current `ranger_msgs/msg/MotionState` installed on the robot only exposes
  `motion_mode`; it does not expose `linear_velocity`, `angular_velocity`, or
  `steering_angle`. Those feedback fields exist inside the AgileX SDK state
  used by `ranger_messenger.cpp`, but are not published in `/motion_state`.
- Code audit of the copied official SDK source confirms `/wheel/odom` is
  produced inside
  `external_sources/jetson_ranger_ros2_20260620/ranger_base/src/ranger_messenger.cpp`.
  `PublishStateToROS()` calls
  `UpdateOdometry(state.motion_state.linear_velocity,
  state.motion_state.angular_velocity, state.motion_state.steering_angle, dt)`,
  but in dual-Ackermann mode `UpdateOdometry()` integrates pose from
  `linear_velocity` and `ConvertInnerAngleToCentral(steering_angle)`. It ignores
  the supplied `angular_velocity` for pose integration and publishes
  `twist.angular.z` as
  `2 * linear * sin(ConvertInnerAngleToCentral(steering_angle)) / wheelbase`.
  Therefore `/wheel/odom` is an SDK kinematic-model output, not a direct
  measured body twist/pose truth.
- Conclusion: continuing EKF covariance or shear sweeps is no longer justified
  by the data. The current `wheel_imu` EKF inherits the SDK wheel pose model;
  removing wheel pose through `twist_imu`, `twist_wheel_yaw_imu`, and
  `wheel_xy_diff_yaw_imu` was already tested and rejected as runtime unsafe or
  still truth-gate failing. The next engineering step is below the EKF: expose
  or use the SDK feedback `angular_velocity`/body twist in the official odom
  model, or otherwise calibrate the SDK's dual-Ackermann odom integration.
  Any such change must be tested with the same two-leg rule and full
  `njrh_systemd_runtime.sh stop && run` restart, not by restarting individual
  services.
