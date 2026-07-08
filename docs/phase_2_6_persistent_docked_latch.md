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
- Successful docking or live charging contact sets the latch. New live charging
  evidence is recorded as `source=charging_session`; legacy `source=bms` remains
  weak TTL evidence for compatibility.
- Odometry-confirmed undock or explicit clear clears the latch.
- BMS contact false alone never clears a `source=charging_session` or
  `source=docking_job` latch while live docking context or full-charge-idle
  evidence still suggests physical dock occupancy. Strong
  `source=charging_session` evidence is not cleared by restart-time
  idle/no-contact context; it can be auto-cleared only after confirmed live
  undock plus stable BMS `no_contact`, or by explicit maintenance/session
  clear. Stale `source=bms` evidence follows the D2 TTL rules and the
  no-live-dock-context clear path.

`pre_navigation_dock_check` now exposes `docked_state_class`,
`docked_evidence`, and `docked_warnings`. `DOCKED_CONFIRMED` comes from live
BMS contact, `/docking/status`, or runtime docking state. `DOCKED_LATCHED`
comes from the persistent latch. Ordinary navigation must auto-undock before
Nav2 in both cases.

`robot_safety` reads the same latch at low frequency and blocks normal
`/cmd_vel_collision_checked` while preserving `/cmd_vel_docking` for controlled
undock. Position proximity is not used as confirmed dock evidence.

Phase D2.3 adds `dock_occupancy_state` on top of the raw latch. Full-charge
charger idle is represented as `DOCKED_CHARGE_IDLE` when a strong charging
session or docking-job latch exists and live BMS has become inconclusive
(`status=0`, `present=false`, `current=0`, or full/idle). `UNCERTAIN_ON_DOCK`
is used when recent strong dock/session evidence exists but BMS and
`/docking/status` are inconclusive. Both states require auto-undock or block
direct Nav2 goal submission. SOC=100 alone does not create a charging-session
latch and does not prove physical dock occupancy.

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
