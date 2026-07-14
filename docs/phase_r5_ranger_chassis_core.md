# Phase R5: Ranger chassis-core ownership

Date: 2026-07-13

## Decision

The project-maintained `ranger_base` is the only Ranger motion-mode and CAN
owner. The repository-owned `ranger_mini3_mode_controller` shadow process is
retired and removed from common, Nav2, and rapid-avoidance startup paths.

The final command path is:

```text
Nav2 / docking / teleop
  -> robot_safety
  -> /cmd_vel
  -> ranger_base
  -> pinned UGV SDK
  -> can0
  -> Ranger Mini 3
```

`robot_safety` remains the only final command arbiter. `ranger_base` owns only
hardware adaptation, mode transitions, chassis limits, feedback, and wheel
odometry.

## Root Cause Addressed

The official UGV SDK 0.8.0 submitted an asynchronous SocketCAN write using the
caller's stack `can_frame`, did not serialize writes, and documented TX as
unbuffered/latest-only. The Ranger ROS callback sent mode `0x141` immediately
followed by motion `0x111`. Under scheduling delay, the first frame could lose
its storage or be overtaken, leaving the chassis in DUAL_ACKERMAN while Nav2
continued requesting SPINNING.

The pinned SDK now copies frames into a bounded queue, permits only one active
write, coalesces stale pending motion/mode frames, and writes a complete CAN
frame. `ranger_base` additionally holds zero until the firmware confirms the
requested mode and `mode_changing=0`.

## Compatibility

- New status owner: `/ranger_base/status`.
- API and safety defaults consume `/ranger_base/status`.
- `ranger_base` temporarily mirrors the same payload to
  `/ranger_mini3_mode_controller/status` for older field diagnostics.
- No node publishes `/ranger_mini3/mode_controller_shadow_cmd_vel`.

## Rollback

1. Set `RANGER_MODE_SWITCH_HANDSHAKE_ENABLED=false`.
2. Rebuild `ranger_base` and restart `njrh-runtime.service` as a whole.
3. Do not restart only the chassis process.

The compatibility status alias allows rollback without changing API or field
capture scripts in the same operation.

## Validation Completed

- ARM64 isolated and production builds passed for `ranger_base`,
  `robot_safety`, and `robot_api_server`.
- `njrh-runtime.service` was restarted as one owner; resident navigation
  reached `nav2_layer_ready` in 36 seconds.
- `/cmd_vel` has one publisher (`robot_safety`) and one chassis subscriber
  (`ranger_base_node`).
- `/ranger_base/status` reported `stable`, DUAL_ACKERMAN desired/actual,
  `mode_changing=false`, and `mode_aligned=true` while stationary.
- `/wheel/odom` measured approximately 50.4-50.9 Hz.
- `can0` remained `ERROR-ACTIVE` with zero tx/rx errors and zero bus-off count.
- The retired controller package is absent from `ros2 pkg list`, no retired
  process is running, and the shadow command topic has no publisher.
- Repository Ranger/safety contract tests passed. The full contract file still
  has a pre-existing Nav2 plugin-list assertion failure outside this phase.

## Hardware Validation Still Required

- 20 supervised spin-mode entry/exit cycles.
- CAN capture of `0x141`, `0x291`, and `0x111` proving ordered handoff.
- Two-point Nav2 navigation without mode oscillation.
- Predock yaw, lateral capture, fine docking, and controlled undock.
