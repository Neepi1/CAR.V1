# Phase 2.3 Nav2 Rotation Progress

## Field Symptom

During short point navigation the robot accepted the Nav2 goal, turned slightly
in place, then stayed nearly stationary until Nav2 aborted at roughly the
12 second progress-check window. The capture showed that the velocity chain was
alive, but the controller output contained angular velocity only:

- `/cmd_vel_nav_raw` was published at about 12 Hz.
- `/cmd_vel_nav`, `/cmd_vel_collision_checked`, and `/cmd_vel` were published.
- `max linear.x` on `/cmd_vel_nav_raw` and `/cmd_vel` was 0.0.
- `max angular.z` was about 0.36 rad/s.
- Wheel odom and local-state odom changed by only about 0.00025 m.
- Final mission-layer yaw alignment was not requested or attempted.

This is not a final-yaw three-step validation failure. It is a controller-stage
startup alignment failure mode: the controller emitted rotation-only commands,
the SimpleProgressChecker only counted XY movement, and the goal aborted for
lack of positional progress.

## Audit Table

| File | Parameter or function | Current value | Affects startup rotation | Affects progress failure | Overlaps mission final yaw | Recommendation | Risk |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `progress_checker.plugin` | `nav2_controller::PoseProgressChecker` | Yes | Yes | No | Count yaw progress during legal in-place startup alignment | Requires Humble plugin availability |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `required_movement_radius` | `0.10` | No | Yes | No | Keep unchanged | Too small would hide stalls |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `required_movement_angle` | `0.10` | Yes | Yes | No | Initial 5.7 degree yaw-progress threshold | Too small may count vibration |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `movement_time_allowance` | `12.0` | No | Yes | No | Keep unchanged | Increasing would mask the bug |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `FollowPath.plugin` | `RotationShimController` | Yes | Indirect | Yes | Keep plugin type unchanged | Removing it changes controller architecture |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `FollowPath.primary_controller` | `MPPIController` | Yes | Indirect | No | Keep MPPI as primary controller | Changing controller would broaden the fix |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `angular_dist_threshold` | `1.20` | Yes | Indirect | No | Keep production default, evaluate A/B profiles only | Large changes can alter path entry behavior |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `rotate_to_goal_heading` | `false` | No | Indirect | Yes | Let mission-layer `final_yaw_align` own terminal heading | Requires final-yaw alignment to remain enabled |
| `src/robot_api_server/src/robot_api_server_node.cpp` | `navigation_final_yaw_*` | mission-layer final yaw | No | No | Owns final yaw | Keep unchanged | Bypassing safety chain would be unsafe |
| `src/robot_bringup/launch/standard_navigation.launch.py` | command remaps | `cmd_vel_nav_raw -> cmd_vel_nav -> cmd_vel_collision_checked -> robot_safety -> cmd_vel_safe -> cmd_vel` | No | Diagnosis path | No | Keep unchanged | Broken chain would hide root cause |

## Configuration Change

The progress checker now uses `nav2_controller::PoseProgressChecker`:

```yaml
progress_checker:
  plugin: "nav2_controller::PoseProgressChecker"
  required_movement_radius: 0.10
  required_movement_angle: 0.10
  movement_time_allowance: 12.0
```

The radius and timeout are unchanged. The new angle threshold lets legal
RotationShim yaw movement count as progress, so a robot that is genuinely
turning in place to align with the path is not falsely treated as completely
stalled. If angular commands are present but yaw barely changes, the diagnostic
script classifies that separately as `CASE_G_ROTATION_STALL`.

`FollowPath.rotate_to_goal_heading` is now `false`. Startup path alignment can
still be handled by RotationShim. Terminal heading is owned by the API
mission-layer `final_yaw_align` after the position has been verified.

## Diagnostics

Use the short-goal diagnostic when a goal fails or appears to only rotate:

```bash
bash scripts/jetson/runtime_overlay/scripts/diagnose_nav2_zero_linear_progress_failure.sh --duration-sec 20
```

It writes:

```text
reports/nav2_zero_linear_progress_<timestamp>.md
```

The report distinguishes:

- `CASE_A_CONTROLLER_ZERO_LINEAR`: controller emits angular velocity but no linear velocity.
- `CASE_B_COLLISION_ZERO_LINEAR`: collision monitor zeros linear velocity.
- `CASE_C_SAFETY_ZERO_LINEAR`: robot_safety zeros linear velocity.
- `CASE_D_MODE_CONTROLLER_OR_CHASSIS_NOT_EXECUTING`: final command exists but odom does not move.
- `CASE_E_ODOM_NOT_REFLECTING_MOTION_REQUIRES_PHYSICAL_CONFIRMATION`: physical motion must be checked against odom.
- `CASE_F_ROTATION_PROGRESS_ONLY`: yaw changes enough that PoseProgressChecker should avoid false XY-only aborts.
- `CASE_G_ROTATION_STALL`: angular command exists but yaw barely changes.

Use the A/B helper only after the baseline profile is verified:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_nav2_rotation_shim_ab.sh --profile pose_progress_only --print
bash scripts/jetson/runtime_overlay/scripts/run_nav2_rotation_shim_ab.sh --profile relaxed_shim_1p8 --apply --restart
bash scripts/jetson/runtime_overlay/scripts/run_nav2_rotation_shim_ab.sh --restore --restart
```

The production default keeps `angular_dist_threshold=1.20`. Relaxed thresholds
are A/B diagnostics, not a permanent default.

## Non-Changes

This phase does not change pointcloud processing, DDS/RMW, PointCloud2 QoS,
JT128 timestamps, local costmap frame ownership, EKF, FAST-LIO2, App API shape,
or the final speed chain. It also does not increase `movement_time_allowance`
or set `required_movement_radius` to zero.
