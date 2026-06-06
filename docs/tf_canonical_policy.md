# Canonical TF Policy

## Ownership

- `robot_localization_bridge`: only owner of `map -> odom`
- `robot_local_state`: only owner of `odom -> base_link`; production runtime is a `robot_localization` EKF named `robot_local_state`
- `robot_description` with `robot_state_publisher`: only owner of static sensor extrinsics
- `robot_hesai_jt128`, `robot_fastlio_mapping`, `robot_pgo_mapping`, `robot_global_localization`: must not inject third-party internal frames into the navigation main tree

## Forbidden Conditions

- More than one publisher for `map -> odom`
- More than one publisher for `odom -> base_link`
- Duplicate `base_link -> lidar_link` or `base_link -> imu_link`
- Direct exposure of third-party frames such as `camera_init`, `aft_mapped`, or vendor odom frames into the canonical tree

## Wrapper Suppression Rules

- Prefer upstream parameters that disable TF publication
- If an upstream component cannot disable TF publication, isolate its frames outside the navigation tree
- Canonical frame names are injected from wrapper configs, not hard-coded inside third-party launch files
- All hardware extrinsics live in YAML or URDF/xacro, never duplicated in both places

## Current Car-Repo Audit Inputs

The local car repository at `D:\codespace\car` currently contains these non-canonical patterns that must stay isolated from the navigation main tree:

- FAST-LIO2 config uses `send_odom_base_tf: true` and `sensor_frame_id: hesai_lidar_fastlio`
- PGO config uses `map_frame: slam_map` and `local_frame: camera_init`
- the historical stack documents `map -> camera_init -> body -> base_link`, which conflicts with the canonical `map -> odom -> base_link`
- `jt128_nav_tools` contains a legacy `map_to_odom_tf_bridge`, but this repository keeps the canonical owner as `robot_localization_bridge`

## Runtime Command Chain

```text
controller_server
  -> /cmd_vel_nav_raw
  -> velocity_smoother
  -> /cmd_vel_nav
  -> collision_monitor
  -> /cmd_vel_collision_checked
  -> robot_safety
  -> /cmd_vel_safe
  -> ranger_mini3_mode_controller
  -> /cmd_vel
  -> ranger_base_node
```

Near-field docking uses a separate pre-safety input so Nav2 collision-monitor zero commands do not overwrite docking commands:

```text
robot_docking_manager
  -> /cmd_vel_docking
  -> robot_safety
  -> /cmd_vel_safe
  -> ranger_mini3_mode_controller
  -> /cmd_vel
  -> ranger_base_node
```
