# Phase D3 Docking Framework State Machine

Phase D3 keeps the existing `robot_docking_manager`, GS2 scan driver, BMS/contact
checks, and `robot_safety` command chain. It does not replace the stack with
`opennav_docking`.

The API return-to-dock path is now staged as:

`DOCK_REQUESTED -> RESOLVE_DOCK_PROFILE -> BEFORE_PREDOCK_RELOCALIZE -> BEFORE_PREDOCK_SETTLE -> NAV_TO_STAGING -> STAGING_NAV_SUCCEEDED -> PREDOCK_POSE_VERIFY -> PREDOCK_YAW_ALIGN -> PREDOCK_YAW_ALIGN_SETTLE -> AFTER_PREDOCK_RELOCALIZE -> AFTER_PREDOCK_SETTLE -> GS2_DOCK_DETECT -> FINE_DOCKING_ENTRY_CHECK -> FINE_ALIGN`.

The new `PREDOCK_YAW_ALIGN` step publishes pure yaw only to `/cmd_vel_docking`.
That command still flows through:

`/cmd_vel_docking -> robot_safety -> /cmd_vel_safe -> ranger_mini3_mode_controller -> /cmd_vel -> ranger_base_node`.

It is separate from normal delivery `final_yaw_align`, which uses
`/cmd_vel_collision_checked` and is intentionally blocked by dock/contact gates.

The API checks `ranger_mini3_mode_controller/status` during predock yaw alignment.
When `predock_yaw_align_require_actual_spin=true`, the actual AgileX motion mode
must enter `SPINNING=2` before the mode-switch timeout.

Fine docking entry is refused before `/docking/start` when:

- GS2 scan is not fresh.
- The staging pose is too far from the expected pre-dock pose.
- Predock yaw alignment did not complete.
- Base/contact yaw is above the fine-entry yaw limit.
- Lateral error is above the fine-entry lateral limit.

During GS2 fine docking, `robot_api_server` calls
`/robot_localization_bridge/set_correction_paused` so AMCL/Isaac candidates are
recorded but not allowed to update `map->odom`. The bridge status exposes
`global_correction_paused`, `correction_paused`, and `correction_pause_reason`.
The API docking state exposes the same pause state plus a display pose based on
the frozen `map->odom` and live `odom->base_link`. No extra TF publisher is added.

Key diagnostics:

- `scripts/jetson/runtime_overlay/scripts/verify_docking_framework_state_machine.sh`
- `scripts/jetson/runtime_overlay/scripts/observe_docking_predock_yaw_align.sh --duration-sec 180 --label dock_test`
- `scripts/jetson/runtime_overlay/scripts/run_docking_framework_ab.sh --profile d3 --duration-sec 180`

Rollback:

Set `predock_yaw_align_enabled=false` and
`docking_pause_global_correction_during_fine=false` in
`robot_api_server.yaml`, then restart `robot_api_server`. The GS2 docking
manager and speed chain are unchanged.

Hardware validation still required:

- Confirm `/cmd_vel_docking` yaw alignment rotates in the expected direction.
- Confirm `/ranger_mini3_mode_controller/status.actual_motion_mode.code` reaches
  `2` during predock yaw alignment.
- Confirm bridge `global_correction_paused=true` only while GS2 fine docking is
  active, and returns to `false` before post-dock relocalization.
