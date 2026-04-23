# robot_local_state

Local odometry wrapper and the only canonical owner of `odom -> base_link`.

## Parameters

- `publish_tf`: defaults to `true`
- `output_topic`: `/local_state/odometry`
- `input_odom_topic`: defaults to `/wheel/odom`
- `input_imu_topic`: defaults to `/sensors/lidar/imu`
- `odom_frame`: `odom`
- `base_frame`: `base_link`
- `mock_mode`: when `false`, republish wheel odom as the canonical local odometry owner

## TF Contract

- This package is the sole publisher of `odom -> base_link`
- No map frame publication is allowed here
- The current live runtime implementation uses chassis wheel odom as the canonical source until the full EKF fusion step is wired into Jetson bringup
