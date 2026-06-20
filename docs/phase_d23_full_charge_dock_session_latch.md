# Phase D2.3 Full-Charge Dock Session Latch

This phase fixes the false negative where the robot is still physically on the
charging dock after a full charge, but Ranger BMS no longer reports charging
contact, charging current, or `present=true`.

The runtime now separates three concepts:

- BMS charging/contact telemetry: live electrical evidence only.
- Physical dock occupancy: whether the robot should be treated as still on the
  dock.
- Dock/session latch: persistent safety memory written by controlled charging
  or docking events.

New charging evidence is stored as `source=charging_session` in
`docking_contact_latch.json`. This source is strong evidence, like
`source=docking_job`. BMS `no_contact`, `current=0`, `present=false`, or
`power_supply_status=UNKNOWN` cannot clear these strong sources by themselves.
They clear only through controlled undock success with retreat-distance
confirmation or explicit maintenance/manual clear. Old `source=bms` latch files
remain weak evidence with the existing D2 TTL behavior.

`pre_navigation_dock_check` now exposes:

- `dock_occupancy_state`
- `dock_occupancy_evidence`
- `dock_occupancy_reason`
- `charging_session_latched`
- `charging_session_age_sec`
- `charging_session_last_confirmed_at`
- `dock_contact_latch_source_strength`
- `bms_live_contact`, `bms_live_contact_reason`
- `bms_percentage`, `bms_current`, `bms_present`
- `full_charge_idle_on_dock`

Navigation admission treats `CONFIRMED_DOCKED`, `DOCKED_CHARGING`,
`DOCKED_CHARGE_IDLE`, and `UNCERTAIN_ON_DOCK` as requiring controlled
auto-undock or blocking direct Nav2 submission. `SOC=100` alone never creates a
charging-session latch and does not prove the robot is on the dock.

Read-only verification:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_full_charge_dock_session_gate.sh \
  --dry-run \
  --mock-charging-observed \
  --mock-full-charge-idle \
  --mock-bms-no-contact \
  --expect-auto-undock \
  --expect-docked-charge-idle \
  --expect-no-latch-clear
```

Live status inspection:

```bash
curl -s http://127.0.0.1:8080/api/v1/navigation/state | jq
curl -s http://127.0.0.1:8080/api/v1/docking/state | jq
```
