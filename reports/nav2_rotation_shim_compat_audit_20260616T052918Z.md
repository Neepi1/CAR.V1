# Nav2 RotationShim Compatibility Audit

Timestamp: 2026-06-16T05:29:18Z

## Scope

Phase N3 read-only audit before migrating ordinary `pose_required` navigation yaw completion from the API `final_yaw_align` speed path to Nav2 native `RotationShimController` goal completion.

## Jetson / Runtime Findings

- `ros2 pkg prefix nav2_rotation_shim_controller`: `/opt/ros/humble`
- Plugin XML contains `nav2_rotation_shim_controller::RotationShimController`:
  `/opt/ros/humble/share/nav2_rotation_shim_controller/nav2_rotation_shim_controller.xml`
- The installed Humble package exposes a `FollowPath.rotate_to_goal_heading` parameter at runtime. `ros2 param get /controller_server FollowPath.rotate_to_goal_heading` returned `False`.
- Current runtime `FollowPath.plugin`: `nav2_rotation_shim_controller::RotationShimController`
- Current runtime `FollowPath.primary_controller`: `nav2_mppi_controller::MPPIController`
- Current runtime `controller_plugins`: `['FollowPath', 'FollowPathFallback']`
- Current runtime `goal_checker_plugins`: `['goal_checker']`
- Current runtime `goal_checker.stateful`: `False`
- Current runtime `goal_checker.xy_goal_tolerance`: `0.2`
- Current runtime `goal_checker.yaw_goal_tolerance`: `0.15`

Conclusion: RotationShimController is supported and `rotate_to_goal_heading` is supported. No backport or Nav2 upgrade blocker was found.

## Workspace Static Findings

- `src/robot_nav_config/config/nav2.yaml` and `scripts/jetson/runtime_overlay/config/nav2.yaml` already wrap the ordinary `FollowPath` controller with `nav2_rotation_shim_controller::RotationShimController`.
- Existing MPPI tuning parameters are stored under `FollowPath` and are preserved as the RotationShim primary controller configuration.
- Existing RPP fallback is `FollowPathFallback` using `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`. The active BT uses `controller_id="FollowPath"`, so the fallback entry is retained and not migrated or deleted.
- Existing blocker values:
  - `FollowPath.rotate_to_goal_heading: false`
  - `goal_checker.stateful: false`
- Existing API ordinary final-yaw path:
  - `navigation_final_yaw_align_enable: true`
  - `run_final_yaw_align(...)`
  - `publish_final_yaw_align_command(...)`
  - `publish_final_yaw_align_zero_burst(...)`
  - runtime phase `position_reached_yaw_aligning`
  - command topic `/cmd_vel_collision_checked`

## Migration Table

| Component | Before | Phase N3 target |
| --- | --- | --- |
| `FollowPath` | RotationShim wrapper, terminal heading disabled | RotationShim wrapper, terminal heading enabled |
| MPPI | Primary controller parameters under `FollowPath` | Preserve unchanged |
| RPP fallback | `FollowPathFallback` in `controller_plugins` | Preserve unchanged |
| Goal checker | `SimpleGoalChecker`, `stateful=false` | `SimpleGoalChecker`, `stateful=true` |
| Ordinary API final yaw | Default enabled, publishes `/cmd_vel_collision_checked` | Default disabled behind explicit fallback |
| Dock staging yaw | Docking-owned `PREDOCK_YAW_ALIGN` on `/cmd_vel_docking` | Preserve unchanged |

## Required Changes

- Enable `FollowPath.rotate_to_goal_heading=true`.
- Enable `goal_checker.stateful=true`.
- Keep `xy_goal_tolerance=0.20` and `yaw_goal_tolerance=0.15`.
- Keep planner, costmap, progress checker, TF tolerance, AMCL/Isaac/bridge strategy, pointcloud transport, FAST-LIO2, Ranger odom, EKF, and robot safety chain unchanged.
- Add `api_final_yaw_align_fallback_enabled=false` and default ordinary API final yaw disabled.
- Change ordinary `pose_required` API completion so Nav2 action success is required before read-only final pose verification.

## Blocker Status

`requires_nav2_rotation_shim_backport_or_upgrade`: false
