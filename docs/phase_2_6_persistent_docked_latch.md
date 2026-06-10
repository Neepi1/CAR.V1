# Phase 2.6 Persistent Docked Latch

This phase covers the field case where the robot is physically on the charger
but live contact evidence is missing. A full battery or missing chassis contact
signal can report `current=0`, `present=false`, and
`power_supply_status=UNKNOWN`, so BMS contact may legitimately be false.

The fix is explicit and non-position-based:

- `docking_contact_latch.json` persists `latched_docked`.
- `POST /api/v1/docking/confirm_docked` sets the latch for maintenance
  recovery and sends no velocity.
- `POST /api/v1/docking/clear_docked_latch` clears the latch and sends no
  velocity.
- Successful docking or live charging contact sets the latch.
- Odometry-confirmed undock or explicit clear clears the latch.
- BMS contact false alone never clears the latch.

`pre_navigation_dock_check` now exposes `docked_state_class`,
`docked_evidence`, and `docked_warnings`. `DOCKED_CONFIRMED` comes from live
BMS contact, `/docking/status`, or runtime docking state. `DOCKED_LATCHED`
comes from the persistent latch. Ordinary navigation must auto-undock before
Nav2 in both cases.

`robot_safety` reads the same latch at low frequency and blocks normal
`/cmd_vel_collision_checked` while preserving `/cmd_vel_docking` for controlled
undock. Position proximity is not used as confirmed dock evidence.

Maintenance commands:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_docked_latch.sh --print \
  --building-id B10 --floor-id F1 --pose-id POSE_ID

bash scripts/jetson/runtime_overlay/scripts/set_docked_latch.sh --confirm \
  --building-id B10 --floor-id F1 --pose-id POSE_ID

bash scripts/jetson/runtime_overlay/scripts/set_docked_latch.sh --clear \
  --building-id B10 --floor-id F1 --pose-id POSE_ID
```

Set `ROBOT_API_TOKEN` if `api_token` is configured.
