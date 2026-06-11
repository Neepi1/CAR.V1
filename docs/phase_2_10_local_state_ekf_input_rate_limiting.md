# Phase 2.10 Local State EKF Input Rate Limiting

This phase protects the `robot_local_state` EKF receive path without changing
mapping quality or navigation ownership.

## Scope

- `/lidar_imu` remains the raw high-rate JT128 IMU stream.
- FAST-LIO2 mapping still consumes `/lidar_points` and `/lidar_imu`.
- `imu_gyro_bias_filter` reads every raw IMU sample for bias estimation.
- `/lidar_imu_bias_corrected` is published at 100 Hz by default for EKF input.
- `/local_state/imu_bias` is published at 10 Hz as a diagnostic/status topic.
- `/wheel/odom_ekf` is timer-published at 50 Hz with callback publishing
  disabled.
- `robot_localization` EKF output remains `frequency: 50.0` and owns
  `odom -> base_link`.

## Non-Goals

This phase does not change PointCloud2 QoS, DDS/RMW defaults, JT128 timestamps,
FAST-LIO2 inputs, Nav2 controller/planner parameters, CAN behavior, App API
logic, watchdog policy, or socket buffer settings.

## Field Verification

After syncing and rebuilding `robot_local_state` on the Jetson, restart the
local-state helper and localization bridge helper, then run:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_local_state_input_rates.sh
```

Expected steady-state rates:

- `/lidar_imu`: raw high-rate stream, roughly 300-500 Hz.
- `/lidar_imu_bias_corrected`: bounded at about 100 Hz.
- `/local_state/imu_bias`: bounded at about 10 Hz.
- `/wheel/odom_ekf`: about 50 Hz.
- `/local_state/odometry`: about 50 Hz.

The report also checks `/robot_local_state` graph visibility, `/tf` ownership,
EKF subscribers on `/wheel/odom_ekf` and `/lidar_imu_bias_corrected`, and UDP
`RcvbufErrors` growth during the sampling window.
