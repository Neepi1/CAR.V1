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
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `required_movement_radius` | `0.03` | No | Yes | No | Keep legal terminal creep below the 6 cm point-goal gate | Larger values delay near-goal fallback |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `required_movement_angle` | `0.05` | Yes | Yes | No | Match the 0.05 rad terminal yaw gate | Larger values delay near-goal fallback |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `movement_time_allowance` | `12.0` | No | Yes | No | Avoid aborting measurable terminal creep or RotationShim yaw progress | Too large delays final yaw fallback |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `FollowPath.plugin` | `RotationShimController` | Yes | Indirect | Yes | Keep plugin type unchanged | Removing it changes controller architecture |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `FollowPath.primary_controller` | `MPPIController` | Yes | Indirect | No | Keep MPPI as primary controller | Changing controller would broaden the fix |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `angular_dist_threshold` / `angular_disengage_threshold` | `0.45` / `0.075` | Yes | Indirect | No | Avoid entering RotationShim on small path changes, but exit only after yaw is close enough for smooth straight-line handoff | Too-low thresholds can make path following over-rotate on gentle curves |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | `rotate_to_goal_heading` | `true` | Yes | Indirect | Yes | Let Nav2 own terminal heading first | API final-yaw alignment remains a bounded fallback |
| `src/robot_api_server/src/robot_api_server_node.cpp` | `navigation_final_yaw_*` | mission-layer final yaw | No | No | Owns final yaw | Keep unchanged | Bypassing safety chain would be unsafe |
| `src/robot_bringup/launch/standard_navigation.launch.py` | command remaps | `cmd_vel_nav_raw -> cmd_vel_nav -> cmd_vel_collision_checked -> robot_safety -> cmd_vel_safe -> cmd_vel` | No | Diagnosis path | No | Keep unchanged | Broken chain would hide root cause |

## Configuration Change

The progress checker now uses `nav2_controller::PoseProgressChecker`:

```yaml
progress_checker:
  plugin: "nav2_controller::PoseProgressChecker"
  required_movement_radius: 0.03
  required_movement_angle: 0.05
  movement_time_allowance: 12.0
```

The radius, angle, and timeout are intentionally close to the tight terminal
goal tolerances. Legal RotationShim yaw movement still counts as progress, but
a near-goal stall no longer waits through a long progress window before
business-layer final yaw recovery can run. If angular commands are present but
yaw barely changes, the diagnostic script classifies that separately as
`CASE_G_ROTATION_STALL`.

`FollowPath.rotate_to_goal_heading` is now `true`. Startup path alignment and
terminal heading are attempted inside Nav2 first. API mission-layer
`final_yaw_align` is only a bounded near-goal fallback after Nav2 fails.

RotationShim path-entry engagement is now explicit:

```yaml
FollowPath:
  angular_dist_threshold: 0.45
  angular_disengage_threshold: 0.075
```

The entry threshold keeps RotationShim from interrupting ordinary mid-route
Ackermann turns for small heading changes. The lower disengage threshold keeps
the robot in pure-yaw alignment until residual yaw is small, while
`robot_safety` checks both `/wheel/odom` yaw-rate settle and the
`/local_state/odometry` heading state consumed by Nav2 before releasing the
first following linear command after a pure spin.

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

Legacy relaxed-threshold A/B profiles remain diagnostics only. The production
default is the lower Ackermann startup gate above, not the older
`angular_dist_threshold=1.20`.

## Non-Changes

This phase does not change pointcloud processing, DDS/RMW, PointCloud2 QoS,
JT128 timestamps, local costmap frame ownership, EKF, FAST-LIO2, App API shape,
or the final speed chain. It also does not increase `movement_time_allowance`
or set `required_movement_radius` to zero.
