# robot_local_state

Local odometry wrapper and the only canonical owner of `odom -> base_link`.

Production runtime uses `robot_localization` EKF and publishes
`/local_state/odometry`. The EKF consumes wheel odometry plus JT128 IMU yaw-rate.
The Jetson runtime forces the JT128 driver to publish host/system timestamps, so
`/lidar_imu` can share the same time base as `/wheel/odom`.

## Parameters

- EKF output topic: `/local_state/odometry` via `/odometry/filtered` remap
- `odom0`: `/wheel/odom`, using planar `vx` and `vyaw`
- `imu0`: `/lidar_imu`, using gyro `vyaw`
- `two_d_mode`: `true`
- `world_frame`: `odom`
- `publish_tf`: `true`, so this package remains the only `odom -> base_link` owner
- IMU linear acceleration is intentionally not fused in the first production profile
- `odom_yaw_offset_rad`: optional diagnostic planar correction applied by the
  wheel-only C++ passthrough node before publishing canonical
  `/local_state/odometry`. Field runtime keeps it `0.0` because Ranger
  `/wheel/odom` is treated as the chassis odometry truth.
- `rotate_odom_position_with_yaw_offset`: when `true`, the passthrough node
  rotates both odom-plane position and base yaw. Keep it `false` when correcting
  only the native chassis child-frame convention while preserving the same odom
  origin.
- `input_base_frame`: expected native chassis frame in `/wheel/odom`, defaults
  to `ranger_base_link`. The node always republishes canonical `base_link`.

## TF Contract

- This package is the sole publisher of `odom -> base_link`
- No map frame publication is allowed here
- Jetson default runtime starts `robot_localization/ekf_node` through `scripts/jetson/runtime_overlay/scripts/run_local_state.sh`
- To run the wheel-only passthrough wrapper for diagnostics, set `LOCAL_STATE_MODE=passthrough`
- The upstream Ranger driver is started with `base_frame=ranger_base_link`;
  this package is the boundary that converts native chassis odom into the
  project canonical `base_link`.
- The Jetson passthrough profile uses `odom_yaw_offset_rad=0.0` and
  `rotate_odom_position_with_yaw_offset=false`: it republishes Ranger
  `/wheel/odom` as canonical `/local_state/odometry` without changing the SDK
  heading.
