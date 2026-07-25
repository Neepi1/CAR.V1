# Phase D3 Docking Framework State Machine

Phase D3 keeps the existing `robot_docking_manager`, GS2 scan driver, BMS/contact
checks, and `robot_safety` command chain. It does not replace the stack with
`opennav_docking`.

The default API return-to-dock path is staged as:

`DOCK_REQUESTED -> RESOLVE_DOCK_PROFILE -> BEFORE_PREDOCK_RELOCALIZE -> BEFORE_PREDOCK_SETTLE -> NAV_TO_STAGING_NATIVE_NAV2 -> PREDOCK_CONTACT_STOP|STAGING_NAV2_EARLY_HANDOFF|STAGING_NAV2_GOAL_SUCCEEDED|STAGING_NAV2_GOAL_ABORTED_HANDOFF_CHECK -> PREDOCK_POSE_VERIFY -> PREDOCK_ALIGNMENT_DEFERRED_FOR_BRIDGE_SETTLE -> AFTER_PREDOCK_RELOCALIZE -> AFTER_PREDOCK_SETTLE -> GS2_DOCK_DETECT -> FINE_DOCKING_BRIDGE_SETTLE -> freeze map->odom correction -> PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE -> PREDOCK_YAW_ALIGN_AFTER_BRIDGE_SETTLE -> PREDOCK_LATERAL_ALIGN_AFTER_BRIDGE_SETTLE -> FINE_DOCKING_ENTRY_CHECK -> FINE_ALIGN`.

The coarse pre-dock action explicitly selects `navigate_to_predock.xml`. That
tree computes one SmacPlanner2D path per action attempt and then continuously
executes `FollowPath`; it does not run the ordinary 1 Hz `RateController`.
SmacPlanner2D is not kinematically constrained, so replacing its path every
second can change the first-segment heading and repeatedly cross
RotationShim's entry threshold on Ranger's Ackermann controller. Ordinary
delivery navigation keeps 1 Hz replanning; only the pre-dock action uses the
stable-path contract. If that action aborts, the docking job's bounded retry
creates a fresh path.

While the pre-dock Nav2 action is running, a fresh BMS charging contact that has
remained stable for the configured two seconds has priority over all navigation
and staging corrections. The API enters `PREDOCK_CONTACT_STOP`, cancels the
active Nav2 goal, publishes a zero burst through the existing safety chain, and
waits for `/wheel/odom` linear and angular speed to settle. If charging remains
confirmed, the job finishes as `charging`; yaw and lateral alignment are never
run after electrical contact. If contact drops, the job remains stopped and
requires explicit undock/retry instead of resuming motion across the contacts.

`PREDOCK_POSE_VERIFY` runs after the Nav2 predock action succeeds, after a
Nav2 aborted result, or after the API cancels Nav2 early at
`STAGING_NAV2_EARLY_HANDOFF`. Predock staging intentionally uses a different
contract from ordinary delivery goals: Nav2 is the coarse approach owner, while
docking owns final yaw and lateral capture. The first pose verification is
read-only and proves only that the vehicle is inside the bounded docking recovery
window. It does not publish yaw or side-slip commands while an accepted
`map->odom` correction may still be smoothing. While Nav2 is running, the API
periodically checks the map-frame approach error; once the pose is inside the
docking-owned recovery window, it cancels the Nav2 goal, publishes zero velocity,
sets `predock_nav_early_handoff=true`, and avoids letting RotationShim/MPPI keep
switching between terminal spin and turning near the pre-dock point. If Nav2
aborts while the pose is outside the XY recovery window, the API retries the
predock Nav2 goal within the configured `docking_max_retries` budget. Once XY is inside
`docking_predock_pose_max_distance_m=0.30`, yaw is judged separately after bridge
settle and recovered through `PREDOCK_YAW_ALIGN_AFTER_BRIDGE_SETTLE` unless it exceeds
`predock_yaw_align_hard_fail_rad`. The older
`docking_predock_pose_max_yaw_rad=0.35` value remains a diagnostic/relocalization
sanity bound, not the gate that blocks yaw recovery when XY is already inside
the docking handoff window. The fine entry lateral gate is `0.08 m`, and the GS2 docking manager keeps the final
`0.030 m` lateral and `4.0 deg` yaw tolerances for contact alignment. It still
requires XY plus base/contact yaw before GS2 fine docking can start. After
`FINE_DOCKING_BRIDGE_SETTLE`, the API rechecks the staging pose using the latest
TF; if bridge smoothing exposes yaw error while XY remains valid, docking-owned
predock yaw alignment runs before `/docking/start`. Global corrections are frozen
immediately after the settle barrier, then yaw and lateral capture run once
against that stable transform. There is no pre-settle physical correction and
therefore no second map-frame side-slip. This prevents bridge yaw smoothing from
satisfying a spin command while Ranger is only changing wheel posture. The
staging correction is a closed loop, not a
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
alignment and then retries lateral capture. After bridge smoothing and correction
freeze, lateral is corrected once against the stable approach frame. This is a
capture correction for sensor acquisition only; the fine-docking manager still
owns the final `3 cm` lateral and `5 deg` contact tolerance. The bridge-smoothing wait is
bounded to a short field timeout so docking does not sit for a minute at the
handoff. If verification fails, `/docking/start` is not called and the failure code is
`PREDOCK_NATIVE_GOAL_VERIFY_FAILED`, `PREDOCK_POSE_DRIFTED_AFTER_BRIDGE_SETTLE`,
`PREDOCK_YAW_NOT_ALIGNED_AFTER_BRIDGE_SETTLE`, or `PREDOCK_LATERAL_NOT_ALIGNED`.

`PREDOCK_YAW_ALIGN_RECOVERY` is retained only as explicit fallback/recovery when
`predock_yaw_align_enabled=true` and `predock_yaw_align_fallback_enabled=true`.
It is called only after bridge settle and correction freeze. When enabled, it
publishes pure yaw only to `/cmd_vel_docking`. That command
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

The API checks Ranger feedback during explicit predock yaw recovery. When
`predock_yaw_align_require_actual_spin=true`, the actual AgileX motion mode must
enter fresh `SPINNING=2` before the mode-switch timeout. Missing confirmation is
a hard `PREDOCK_YAW_ALIGN_MODE_SWITCH_TIMEOUT`, not a warning. Map-frame yaw
entering tolerance before that confirmation is rejected as
`PREDOCK_YAW_ALIGN_NO_CONFIRMED_PHYSICAL_SPIN`.

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
`/docking/start` is not called. Before stable staging yaw/lateral alignment,
`robot_api_server` calls `/robot_localization_bridge/set_correction_paused` so AMCL/Isaac
candidates are recorded but not allowed to update `map->odom`. The bridge status
exposes `global_correction_paused`, `correction_paused`, and
`correction_pause_reason`. The API docking state exposes the same pause state
plus the fine bridge settle fields and a display pose based on the frozen
`map->odom` and live `odom->base_link`. No extra TF publisher is added.

For the Orbbec backend, camera control ends at the transition into
`ContactVerify`. The calibrated near protrusion is observable before contact but
disappears in the camera blind zone during spring compression; the remaining
wide plane is the background wall. Perception therefore accepts only the
calibrated fixed-face and near-protrusion span profiles. The near protrusion
uses its first-touch feature-center calibration of `y=-0.0418 m`, while the
fixed-face profile remains in the normal contact-frame center contract.
`ContactVerify`
does not consume target geometry. It commands only bounded straight motion
with `linear.y=0` and `angular.z=0`, stops immediately on BMS contact, and is
bounded by fresh odometry, insertion distance, and timeout. A no-contact limit
enters the existing straight backoff/reacquire sequence.

The correction pause is owned by stable staging alignment plus the fine-docking
lifecycle. It starts only after bridge smoothing reports settled, remains active
through physical yaw/lateral capture and sensor-guided docking, and never covers
the Nav2 coarse approach. If a fine
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
