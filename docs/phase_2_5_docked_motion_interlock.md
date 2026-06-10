# Phase 2.5 Docked Motion Interlock

This phase closes the unsafe case where a normal navigation goal can rotate the
robot while it is still on the charger.

The dock state is not inferred from current map position. It is explicit state:
BMS charging contact, `/docking/status`, backend docking job state, and the
persistent `docking_contact_latch.json` written by `robot_api_server` and
`robot_docking_manager`.

Normal navigation still enters `POST /api/v1/navigation/goal`. Before any
`NavigateToPose` goal is sent, the API evaluates `pre_navigation_dock_check`.
If docked, charging, contact, or the dock latch is active, the API must call
`/docking/undock`, wait for odometry-confirmed `undocked`, trigger post-undock
relocalization, wait for fresh `map -> base_link`, and only then send the Nav2
goal. If undock or relocalization fails, no Nav2 goal is sent.

`final_yaw_align` rechecks the same dock/contact gate before and during its
loop. If the gate becomes active, it publishes zero, exits with
`final_yaw_align_blocked=true`, and reports
`final_yaw_align_blocked_reason=DOCKED_OR_CHARGING_CONTACT`. It is never an
undock substitute.

`robot_safety` is the second defense line. With
`block_normal_motion_when_docked=true`, normal `/cmd_vel_collision_checked`
commands are zeroed while dock/contact evidence is active, and `/safety/status`
reports `DOCKED_CONTACT_BLOCK`. The docking input `/cmd_vel_docking` is still
allowed when `allow_docking_cmd_when_docked=true`, so the controlled undock
path remains usable.

The verification script is read-only by default:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_docked_navigation_undock_gate.sh \
  --building-id B10 \
  --floor-id F1 \
  --pose-id POSE_ID
```

Only add `--execute-goal` when movement is intended and the field is clear.
The optional `--test-normal-cmd-block` and `--test-docking-cmd-allowed`
checks publish one low test command and are intentionally not enabled by
default. Use them only during controlled bench validation with dock/contact
evidence already active.
