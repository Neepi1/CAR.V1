# Phase D3 Docking Framework State Machine

Phase D3 keeps the existing `robot_docking_manager`, GS2 scan driver, BMS/contact
checks, and `robot_safety` command chain. It does not replace the stack with
`opennav_docking`.

The default API return-to-dock path is staged as:

`DOCK_REQUESTED -> RESOLVE_DOCK_PROFILE -> BEFORE_PREDOCK_RELOCALIZE -> BEFORE_PREDOCK_SETTLE -> NAV_TO_STAGING_NATIVE_NAV2 -> STAGING_NAV2_EARLY_HANDOFF|STAGING_NAV2_GOAL_SUCCEEDED|STAGING_NAV2_GOAL_ABORTED_HANDOFF_CHECK -> PREDOCK_POSE_VERIFY -> PREDOCK_YAW_ALIGN_RECOVERY -> PREDOCK_LATERAL_ALIGN -> AFTER_PREDOCK_RELOCALIZE -> AFTER_PREDOCK_SETTLE -> GS2_DOCK_DETECT -> FINE_DOCKING_BRIDGE_SETTLE -> PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE -> PREDOCK_YAW_ALIGN_AFTER_BRIDGE_SETTLE -> PREDOCK_LATERAL_ALIGN_AFTER_BRIDGE_SETTLE_VERIFY -> FINE_DOCKING_ENTRY_CHECK -> FINE_ALIGN`.

`PREDOCK_POSE_VERIFY` runs after the Nav2 predock action succeeds, after a
Nav2 aborted result, or after the API cancels Nav2 early at
`STAGING_NAV2_EARLY_HANDOFF`. Predock staging intentionally uses a different
contract from ordinary delivery goals: Nav2 is the coarse approach owner, while
docking owns final yaw and lateral capture. While Nav2 is running, the API
periodically checks the map-frame approach error; once the pose is inside the
docking-owned recovery window, it cancels the Nav2 goal, publishes zero velocity,
sets `predock_nav_early_handoff=true`, and avoids letting RotationShim/MPPI keep
switching between terminal spin and turning near the pre-dock point. If Nav2
aborts while the pose is outside the XY recovery window, the API retries the
predock Nav2 goal within the configured `docking_max_retries` budget. Once XY is inside
`docking_predock_pose_max_distance_m=0.30`, yaw is judged separately and recovered
through `PREDOCK_YAW_ALIGN_RECOVERY` unless it exceeds
`predock_yaw_align_hard_fail_rad`. The older
`docking_predock_pose_max_yaw_rad=0.35` value remains a diagnostic/relocalization
sanity bound, not the gate that blocks yaw recovery when XY is already inside
the docking handoff window. The fine entry lateral gate is `0.08 m`, and the GS2 docking manager keeps the final
`0.030 m` lateral and `4.0 deg` yaw tolerances for contact alignment. It still
requires XY plus base/contact yaw before GS2 fine docking can start. After
`FINE_DOCKING_BRIDGE_SETTLE`, the API rechecks the staging pose using the latest
TF; if bridge smoothing exposes yaw error while XY remains valid, docking-owned
predock yaw alignment runs before `/docking/start`. The post-bridge lateral step
is read-only: it verifies that the centerline error is still inside
`fine_docking_entry_max_lateral_m` and does not publish a second side-slip
command. This prevents a small map-frame estimate change after bridge smoothing
from dragging an already physically centered robot away from the dock before GS2
can observe the charger. The initial staging correction is a closed loop, not a
one-shot check or a normal circular XY goal. In the dock
approach frame, forward/x is only a broad safety window
(`predock_forward_capture_min_m` to `predock_forward_capture_max_m`) because the
fine docking behavior drives forward into the dock; lateral/y is the charging
dock centerline error and is the tight capture target. Up to
`predock_staging_capture_max_cycles` times the API re-reads map pose, reruns
predock yaw alignment when yaw is outside tolerance, reruns side-slip lateral
capture when the centerline error is outside target, and only exits successfully
after strict forward-window/yaw/lateral verification passes.
`PREDOCK_LATERAL_ALIGN` projects the pose error into the approach frame, requests
`/ranger_mini3/forced_mode=side_slip`, and publishes a bounded `linear.y`
command to `/cmd_vel_docking` until the signed centerline lateral error is within
`predock_lateral_align_target_m=0.03`; a single capture attempt is bounded by
`predock_lateral_align_max_correction_m=0.25`. During side-slip, yaw uses a
short-horizon hysteresis gate
`predock_yaw_align_tolerance_rad + predock_lateral_align_yaw_slack_rad` so tiny
map-frame yaw jitter does not abort capture. If the measured map-frame lateral
error diverges by `predock_lateral_align_divergence_epsilon_m` for
`predock_lateral_align_divergence_count` samples, the API publishes a zero burst,
reverses the side-slip command direction once, and fails with
`PREDOCK_LATERAL_ALIGN_DIVERGING` if the reversed command also diverges. If yaw drifts outside that gate or
the strict final verify yaw, the next closed-loop cycle reruns predock yaw
alignment and then retries lateral capture. After bridge smoothing, lateral is
verified against the fine-entry window only; if it exceeds that window the dock
job fails instead of performing another map-frame side-slip. This is a capture
correction for GS2 acquisition only; the GS2 docking manager still owns the final `3 cm`
lateral and `4 deg` contact tolerance. The bridge-smoothing wait is
bounded to a short field timeout so docking does not sit for a minute at the
handoff. If verification fails, `/docking/start` is not called and the failure code is
`PREDOCK_NATIVE_GOAL_VERIFY_FAILED`, `PREDOCK_POSE_DRIFTED_AFTER_BRIDGE_SETTLE`,
`PREDOCK_YAW_NOT_ALIGNED_AFTER_BRIDGE_SETTLE`, or `PREDOCK_LATERAL_NOT_ALIGNED`.

`PREDOCK_YAW_ALIGN_RECOVERY` is retained only as explicit fallback/recovery when
`predock_yaw_align_enabled=true` and `predock_yaw_align_fallback_enabled=true`.
When enabled, it publishes pure yaw only to `/cmd_vel_docking`. That command
still flows through:

`/cmd_vel_docking -> robot_safety -> /cmd_vel -> ranger_base_node`. `/cmd_vel_safe` is a robot_safety diagnostic mirror.

`PREDOCK_LATERAL_ALIGN` uses the same `/cmd_vel_docking` path and never lets the
App publish chassis velocity directly. It additionally requests
`/ranger_mini3/forced_mode=side_slip` before lateral commands and releases
`/ranger_mini3/forced_mode=auto` after the zero-velocity burst.

It is separate from normal delivery `final_yaw_align`, which uses
`/cmd_vel_collision_checked` and is intentionally blocked by dock/contact gates.
Phase N2 makes that separation explicit in state: the predock Nav2 target is
`goal_completion_policy=dock_staging`, ordinary `final_yaw_align` is forbidden
for staging goals, and `ordinary_final_yaw_align_active` /
`predock_yaw_align_active` must not be true at the same time.

The API checks `ranger_mini3_mode_controller/status` during explicit predock yaw
recovery. When `predock_yaw_align_require_actual_spin=true`, the actual AgileX
motion mode must enter `SPINNING=2` before the mode-switch timeout.

Fine docking entry is refused before `/docking/start` when:

- The docking job is not `goal_completion_policy=dock_staging`.
- `dock_staging_handoff_ready` is false.
- `predock_pose_verified` is false.
- Post-predock relocalization/settle has not completed.
- `robot_localization_bridge` still reports active `map->odom` smoothing.
- Global correction pause has not been applied.
- GS2 scan is not fresh.
- The staging pose is too far from the expected pre-dock pose.
- Predock yaw alignment did not complete.
- Predock lateral capture did not complete.
- Base/contact yaw is above the fine-entry yaw limit.
- Lateral error is above the fine-entry lateral limit.

Immediately before GS2 fine docking, `robot_api_server` waits for
`robot_localization_bridge.safe_for_goal_start=true` for the configured stable
sample count. If smoothing does not settle within the bounded wait, the job
fails with `DOCK_FAILED_FINE_LOCALIZATION_TRANSITION_TIMEOUT` and
`/docking/start` is not called. During GS2 fine docking, `robot_api_server`
calls `/robot_localization_bridge/set_correction_paused` so AMCL/Isaac
candidates are recorded but not allowed to update `map->odom`. The bridge status
exposes `global_correction_paused`, `correction_paused`, and
`correction_pause_reason`. The API docking state exposes the same pause state
plus the fine bridge settle fields and a display pose based on the frozen
`map->odom` and live `odom->base_link`. No extra TF publisher is added.

The correction pause is owned by the GS2 fine-docking lifecycle only. If a fine
docking job is canceled, fails, stops, or is preempted after `/docking/start`
has succeeded, the API releases the `docking_fine` pause on job finish. Before
auto-undock for a normal navigation goal, and again before post-undock
relocalization, the API checks for a stale `correction_pause_reason=docking_fine`
and releases it instead of letting the next relocalization be rejected by
`GLOBAL_CORRECTION_PAUSED`.

Key diagnostics:

- `scripts/jetson/runtime_overlay/scripts/verify_docking_framework_state_machine.sh`
- `scripts/jetson/runtime_overlay/scripts/observe_docking_predock_yaw_align.sh --duration-sec 180 --label dock_test`
- `scripts/jetson/runtime_overlay/scripts/run_docking_framework_ab.sh --profile d3 --duration-sec 180`
- `scripts/jetson/runtime_overlay/scripts/observe_predock_yaw_alignment_trace.sh --duration-sec 180`
- `scripts/jetson/runtime_overlay/scripts/run_predock_yaw_alignment_probe.sh --dry-run`
- `scripts/jetson/runtime_overlay/scripts/verify_fine_docking_entry_gate.sh`
- `scripts/jetson/runtime_overlay/scripts/run_v1_navigation_docking_validation.sh --observe-only --duration-sec 120`

The Phase V1 scripts keep the default path observation-only. The predock yaw
probe publishes motion only when `--apply-small-yaw-test` is explicitly passed,
and then it publishes only to `/cmd_vel_docking`. For the N3-D native predock
contract, prefer
`scripts/jetson/runtime_overlay/scripts/observe_docking_predock_native_nav2.sh`
or the compatible `observe_docking_predock_yaw_align.sh`.

Rollback:

Keep `predock_yaw_align_enabled=false`,
`predock_yaw_align_fallback_enabled=false`,
`predock_lateral_align_enabled=false`,
`fine_docking_retry_on_yaw_reject=false`, and
`docking_pause_global_correction_during_fine=false` in
`robot_api_server.yaml`, then restart the full `njrh-runtime.service` owner. The
GS2 docking manager and speed chain are unchanged.

Hardware validation still required:

- Confirm `/cmd_vel_docking` yaw alignment rotates in the expected direction.
- Confirm `/ranger_mini3_mode_controller/status.actual_motion_mode.code` reaches
  `2` during predock yaw alignment.
- Confirm bridge `global_correction_paused=true` only while GS2 fine docking is
  active, and returns to `false` before post-dock relocalization.
- Confirm auto-undock-before-navigation does not inherit a stale
  `docking_fine` correction pause from a previous docking attempt.
