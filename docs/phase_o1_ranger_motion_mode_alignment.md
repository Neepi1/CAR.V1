# Phase O1: Ranger motion mode desired/actual alignment

Date: 2026-06-11

## Scope

Use the official AgileX Ranger `motion_mode` enum inside the repository-owned
`ranger_mini3_mode_controller` while preserving the existing speed chain:

`controller_server / final_yaw_align / docking / teleop -> robot_safety -> ranger_mini3_mode_controller -> ranger_base_node`

This phase does not change Nav2 plugins, EKF production defaults, pointcloud
transport, FAST-LIO2, DDS/RMW settings, JT128 timestamps, or Ranger CAN protocol.

## Official enum

The internal enum mirrors the upstream Ranger messages:

| Code | Name |
| --- | --- |
| 0 | `MOTION_MODE_DUAL_ACKERMAN` |
| 1 | `MOTION_MODE_PARALLEL` |
| 2 | `MOTION_MODE_SPINNING` |
| 3 | `MOTION_MODE_SIDE_SLIP` |
| 255 | `MOTION_MODE_UNKNOWN` |

## Audit table

| File | Function/class/script | Computes desired? | Publishes desired? | Current desired type/semantics | Subscribes actual? | Reads actual mode? | Cmd->mode decision? | Final yaw/spin? | Wheel/EKF impact | Risk | O1 change |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `src/ranger_mini3_mode_controller/src/mode_controller_node.cpp` | `RangerMini3ModeController` | Yes | Yes | Before O1: `std_msgs/String` legacy strings `dual_ackermann`, `crab`, `spin`, `park` | Before O1: no | Before O1: no | Yes, maps `/cmd_vel_safe` to `/cmd_vel` | Yes, pure yaw becomes spin | Indirect, because Ranger driver odom model follows chassis mode | Desired and actual could silently diverge | Add official enum mapping, actual subscriptions, status alignment fields |
| `src/ranger_mini3_mode_controller/config/ranger_mini3_mode_controller.yaml` | ROS params | No | Configures topic | `desired_mode_topic: /ranger_mini3/desired_motion_mode` | No | No | Configures node | Indirect | No | Missing actual feedback topics | Add `/motion_state`, `/system_state`, age/warn thresholds |
| `scripts/jetson/runtime_overlay/config/ranger_mini3_mode_controller.yaml` | Runtime params | No | Configures topic | Same as package config | No | No | Configures node | Indirect | No | Jetson runtime would miss O1 params | Add same actual feedback params |
| `src/robot_api_server/src/robot_api_server_node.cpp` | `run_final_yaw_align`, `publish_final_yaw_align_command` | No | No | N/A | No | No | Publishes pure yaw to `/cmd_vel_api` | Yes | Indirect through robot_safety | No actual mode check before final yaw | Leave command path unchanged; O1 status shows desired SPINNING and actual alignment after safety/mode controller |
| `src/robot_nav_config/config/nav2.yaml` and overlay copy | `RotationShimController`, velocity smoother, collision monitor | No | No | N/A | No | No | Produces pure yaw during startup alignment | Yes | Indirect through speed chain | RotationShim can request yaw before actual mode is visible | Leave plugins unchanged; mode controller maps pure yaw to desired SPINNING |
| `src/robot_docking_manager/src/docking_manager_node.cpp` | `enter_docking_motion_mode`, `publish_forced_mode` | Optional official side-slip request | No desired topic directly | Production config publishes `side_slip` to `/ranger_mini3/forced_mode` while fine docking is active | No | No | Requests docking-only lateral motion for GS2 fine alignment | No | Indirect | Normal navigation must still reject lateral commands | Accept both legacy words and official code/name strings; docking side-slip maps to official `SIDE_SLIP=3` |
| `src/robot_local_state/src/local_state_node.cpp` | `local_state_node` | No | No | N/A | No | No | No | No | Republishes odom in passthrough/fastlio modes | Does not know actual motion mode | No O1 behavior change |
| `src/robot_local_state/config/local_state_wheel_odom_ekf.yaml` and overlay copy | `wheel_odom_ekf_input` params | No | No | N/A | No | No | No | No | Normalizes `/wheel/odom` to `/wheel/odom_ekf` | Cannot guard spin-mode odom anomalies | Future guard must use actual mode |
| `src/robot_local_state/config/local_state_ekf.yaml` and overlay copy | `robot_localization` EKF params | No | No | N/A | No | No | No | No | Fuses wheel x/y/yaw pose plus wheel/IMU yaw-rate | Bad wheel pose can move `/local_state/odometry` | No O1 EKF default change |

## Runtime observations before O1

- `/ranger_mini3/desired_motion_mode` was repository-owned and was not subscribed by
  App/API/RViz/scripts in the repository.
- Jetson had official `/motion_state [ranger_msgs/msg/MotionState]` and
  `/system_state [ranger_msgs/msg/SystemState]` from `ranger_base_node`.
- Both official topics contained `motion_mode`; current stationary value was `0`.
- No repository-owned node read actual `motion_mode`.
- `final_yaw_align` and RotationShim pure yaw commands were not checking actual
  chassis mode.
- `wheel_odom_ekf_input` and `robot_localization` EKF did not know desired or
  actual motion mode.
- The EKF directly fused wheel odom x/y/yaw pose; future odom guards must be based
  on actual chassis mode, not desired intent.

## Implementation

- Added `ranger_motion_mode.hpp` with the official enum and JSON/string helpers.
- Added `ranger_msgs` dependency to `ranger_mini3_mode_controller`.
- Kept `/ranger_mini3/desired_motion_mode` as `std_msgs/String` but changed its
  value to official enum JSON with the legacy mode included.
- Subscribed to `/motion_state` and `/system_state` to capture actual
  `motion_mode`.
- Extended `/ranger_mini3_mode_controller/status` with:
  - `desired_motion_mode`
  - `actual_motion_mode`
  - `mode_aligned`
  - `mode_alignment_state`
- Preserved legacy `/ranger_mini3/forced_mode` inputs and added acceptance for
  official code/name strings such as `2` and `motion_mode_spinning`.

## Important behavior note

O1 does not block the first spin command while waiting for actual SPINNING feedback.
The upstream Ranger driver derives actual mode from incoming `/cmd_vel`, so blocking
the command before actual mode changes would deadlock mode switching. O1 makes the
desired/actual mismatch observable and warns when a live motion command remains out
of alignment; subsequent wheel-odom/EKF guards should consume actual `motion_mode`.

## Required hardware validation

After syncing and restarting the mode controller on Jetson:

1. Confirm `/ranger_mini3/desired_motion_mode` is still `std_msgs/String`.
2. Confirm `/motion_state` and `/system_state` are subscribed by
   `ranger_mini3_mode_controller`.
3. Confirm idle status reports desired `MOTION_MODE_DUAL_ACKERMAN` and actual mode
   from the official feedback topic.
4. During a supervised pure-yaw command through the normal safety chain, confirm
   desired `MOTION_MODE_SPINNING` and that actual mode reaches `MOTION_MODE_SPINNING`.
