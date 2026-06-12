# robot_local_state

Local odometry wrapper and the only canonical owner of `odom -> base_link`.

Production runtime defaults to the `robot_localization` EKF profile with
`LOCAL_STATE_MODE=ekf`. In that mode a small odom preprocessor republishes Ranger
`/wheel/odom` as `/wheel/odom_ekf` with sane pose and twist covariance floors,
then the IMU gyro-bias filter republishes `/lidar_imu` as
`/lidar_imu_bias_corrected`, and the EKF fuses wheel x/y/yaw pose, wheel
forward/yaw velocity, and corrected JT128 IMU yaw-rate.

For temporary chassis-odom isolation, keep `LOCAL_STATE_MODE=ekf` and set
`LOCAL_STATE_EKF_PROFILE=wheel_only`. That profile uses
`local_state_ekf_wheel_only.yaml`, starts the wheel odom preprocessor, skips the
IMU gyro-bias filter, and runs `robot_localization` from `/wheel/odom_ekf` only.
It preserves `/local_state/odometry` and the single canonical
`odom -> base_link` TF owner. Return to the default fusion profile with
`LOCAL_STATE_EKF_PROFILE=wheel_imu`. The Jetson runtime override lives in
`scripts/jetson/runtime_overlay/config/local_state_ekf_profile.env`.

The raw `/lidar_imu` stream remains high-rate for JT128 and FAST-LIO2 mapping.
Only the EKF input branch is bounded: `imu_gyro_bias_filter` still reads every
raw IMU sample for bias estimation, but publishes `/lidar_imu_bias_corrected` at
100 Hz by default and `/local_state/imu_bias` at 10 Hz. The wheel odom
preprocessor publishes `/wheel/odom_ekf` from its timer at 50 Hz with
`publish_on_callback=false`, so it does not double-publish from both callback and
timer. The EKF output remains 50 Hz and remains the only owner of
`odom -> base_link`.

FAST-LIO2 local-state remains available for diagnostics with
`LOCAL_STATE_MODE=fastlio`. In that mode FAST-LIO2 consumes canonical
`/lidar_points` and `/lidar_imu`, publishes raw `/Odometry` with its public TF
remapped away, then the C++ `robot_fastlio_mapping/fastlio_odom_bridge_node`
converts that stream to planar `/fastlio/base_odometry`. This package republishes
that odom as `/local_state/odometry` and remains the only canonical owner of
`odom -> base_link`.

## Parameters

- FAST-LIO diagnostic input: `/fastlio/base_odometry`
- FAST-LIO local-state config: `local_state_fastlio.yaml`
- EKF output topic: `/local_state/odometry` via `/odometry/filtered` remap
- `odom0`: `/wheel/odom_ekf`, using planar `x`, `y`, `yaw`, `vx`, and `vyaw`
- Wheel-only EKF config: `local_state_ekf_wheel_only.yaml`, selected by
  `LOCAL_STATE_EKF_PROFILE=wheel_only`, removes `imu0` entirely for temporary
  chassis odom isolation
- `local_state_wheel_odom_ekf.yaml`: applies nonzero pose covariance floors,
  including a bounded yaw covariance floor, so zero-covariance Ranger odometry
  anchors heading without completely dominating IMU yaw-rate smoothing
- `anchor_pose_to_first_sample`: enabled for the `/wheel/odom_ekf`
  preprocessor, subtracting the first wheel odom x/y/yaw and rotating later
  positions into that local frame before EKF fusion
- `imu0`: `/lidar_imu_bias_corrected`, using gyro `vyaw`
- `local_state_wheel_odom_ekf.yaml`: publishes `/wheel/odom_ekf` from
  `/wheel/odom` with covariance floors so zero-covariance Ranger messages do
  not dominate IMU fusion
- `local_state_imu_bias_filter.yaml`: estimates gyro bias while
  `/wheel/odom_ekf` and `/cmd_vel_safe` indicate the robot is stationary,
  publishes corrected IMU on `/lidar_imu_bias_corrected`, and publishes the
  current bias on `/local_state/imu_bias`
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
- `input_base_frame`: expected chassis frame in `/wheel/odom`, defaults to
  canonical `base_link`. The node always republishes canonical `base_link`.

## TF Contract

- This package is the sole publisher of `odom -> base_link`
- No map frame publication is allowed here
- Jetson default navigation runtime starts the wheel odom preprocessor, IMU
  bias filter, and `robot_localization` EKF through
  `scripts/jetson/runtime_overlay/scripts/run_local_state.sh`
- In FAST-LIO mode, `run_local_state.sh` checks endpoint registration once at
  startup. It does not run a periodic ROS graph endpoint self-monitor, so a
  transient graph probe miss during Nav2 startup cannot stop the canonical
  odom owner.
- `run_local_state.sh` stops the FAST-LIO odom bridge, `local_state_node`,
  and EKF fallback children with bounded INT/TERM/KILL waits so a ROS shutdown
  hang cannot leave a process alive after its ROS endpoints have disappeared.
- To run the wheel+IMU EKF default explicitly, set `LOCAL_STATE_MODE=ekf`
- To run EKF from chassis odometry only, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_only`
- To run FAST-LIO local-state for diagnostics, set `LOCAL_STATE_MODE=fastlio`
- To run the wheel-only passthrough wrapper for diagnostics, set `LOCAL_STATE_MODE=passthrough`
- The upstream Ranger driver is started with `base_frame=base_link` and does
  not publish odom TF; this package remains the only owner of `odom -> base_link`.
- The Jetson passthrough profile uses `odom_yaw_offset_rad=0.0` and
  `rotate_odom_position_with_yaw_offset=false`: it republishes Ranger
  `/wheel/odom` as canonical `/local_state/odometry` without changing the SDK
  heading.
