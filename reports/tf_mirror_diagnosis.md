# TF mirror diagnosis

Date: 2026-05-18

## Runtime facts

- `/tf_static` has one publisher: `/robot_description_static_tf`.
- `/tf` publishes `odom -> base_link` from `/robot_local_state`; `/wheel/odom` and `/local_state/odometry` have matching orientation.
- A controlled forward test using positive `linear.x` moved the chassis forward and produced positive forward projection in `/wheel/odom` and `/local_state/odometry`; this only proves Ranger odom is internally self-consistent. It does not prove the SDK's `base_link` convention matches the physical vehicle front.
- Live static transform is:
  - `base_link -> lidar_mount_link`: `xyz=(0.25, 0.0, 0.85)`, `rpy=(0, -20deg, 0)`
  - `lidar_mount_link -> lidar_link`: identity
  - `base_link -> lidar_level_link`: `xyz=(0.25, 0.0, 0.85)`, `rpy=(0, 0, 0)`
- `/lidar_points` is canonicalized by `pointcloud_axis_remap` using:
  - `x_out = y_raw`
  - `y_out = -x_raw`
  - `z_out = z_raw`
  - output frame: `lidar_link`

## Root cause found

The live Jetson runtime and the source-side robot description were split:

- Runtime overlay `scripts/jetson/runtime_overlay/config/sensors.yaml` used `lidar_axis_yaw: 0.0`.
- Source config `src/robot_description/config/sensors.yaml` still used `lidar_axis_rpy: [0.0, 0.0, -1.5707963267948966]`.
- Xacro defaults also still carried `lidar_axis_yaw=-90deg`, `lidar_level_yaw=-90deg`, and stale IMU defaults.

This meant the runtime TF tree was not mirrored, but any RViz/RobotModel or manual `description.launch.py` session that loaded the source description could display a different sensor-axis tree. That makes the frame view look mirrored or rotated even though the live `/tf_static` tree is canonical.

## Fix applied

- Set source `lidar_axis_rpy` to zero because `/lidar_points` is already normalized before publishing.
- Set Xacro defaults for `lidar_axis_yaw` and `lidar_level_yaw` to zero.
- Set Xacro IMU defaults to the current installation pose.
- Updated stale scan-band documentation.

## Follow-up correction

The operator-side field check showed the static lidar tree is not the active mirror source: `lidar_link` is already mounted under `base_link` and follows the same axis convention. The next diagnosis point is the Ranger odom source because `/wheel/odom` is published directly by `ranger_base_node`.

`robot_local_state` now has an explicit passthrough correction:

- `odom_yaw_offset_rad`
- `rotate_odom_position_with_yaw_offset`

The following field check was later superseded by the 2026-05-22 live check,
after the lidar point cloud was made canonical in the remap layer. It is kept
here as historical context only:

- Forward chassis motion produces the correct `/wheel/odom` and `/local_state/odometry` position displacement.
- In `Fixed Frame=odom`, the `odom` red X axis matches the physical forward direction.
- The temporary `odom_yaw_offset_rad=pi` made `base_link` exactly opposite to `odom`, proving that the 180 degree correction does not belong in `robot_local_state`.
- Field checks showed the dense physical-front return cluster requires a 180 degree correction in the JT128 install relation. Keep that correction in `base_link -> lidar_mount_link` and do not apply it to Ranger odom.

The current passthrough profile no longer follows that historical conclusion:
Ranger native `ranger_base_link` is converted to the project canonical
`base_link` with a child-frame yaw correction in `robot_local_state`.

Use `scripts/jetson/runtime_overlay/scripts/diagnose_wheel_odom_direction.sh` while driving forward 0.10-0.30 m to verify whether `/wheel/odom` reports positive or negative forward displacement.

## Remaining check

If RViz still looks mirrored, verify the exact RViz display after restarting `robot_local_state`:

- `TF` display: confirm whether the user is looking at `odom` axes expressed in fixed `base_link`; parent-frame axes will appear as the inverse transform and can be visually misleading.
- `LaserScan` / map display: `/scan` is derived from `/lidar_points -> /points_nav -> /scan_raw -> /scan`; scan slicing can produce directional artifacts independent of the base TF tree.

## 2026-05-22 regression root cause

The long-running front/back mismatch reappeared because incompatible correction
models were mixed:

1. a previous runtime rotated `/lidar_points` numerically in `pointcloud_axis_remap`;
2. the intended operator model uses `lidar_axis_*` for raw lidar-axis alignment
   to the chassis axes; and
3. a stale report still treated `base_link -> lidar_link` as the place that
   must carry an additional `pi` yaw correction.

The corrected convention is now explicit: public `/lidar_points` and
`/lidar_imu` are canonical sensor-frame outputs. The remap nodes rotate JT128
raw axes numerically (`x=raw_y`, `y=-raw_x`, `z=raw_z`) before publishing them
as `lidar_link` / `imu_link`. The static TF chain keeps only the physical
installation pose. The install yaw remains `0.0`; adding `pi` at this layer
double-corrects front/back.

The stale `pi` correction was briefly restored because the report previously
claimed:

- `scripts/jetson/runtime_overlay/config/sensors.yaml`: `lidar_yaw=0.0`,
  `imu_yaw=0.0`
- `src/robot_description/config/sensors.yaml`: `lidar_rpy` and `imu_rpy`
  ended in `0.0`
- `src/robot_description/urdf/robot.urdf.xacro`: default `lidar_yaw`,
  `lidar_level_yaw`, and `imu_yaw` were `0.0`
- `src/robot_system_tests/test/test_workspace_contracts.py` asserted the
  regressed `0.0` yaw values, so the regression was locked into the contract
  tests.

Live Jetson verification before the fix showed:

- only one `/tf_static` publisher: `/robot_description_static_tf`
- no active `odom -> base_link` publisher when the check was performed
- `base_link -> lidar_link`: `xyz=(0.25, 0.0, 0.85)`,
  `rpy=(0deg, -20deg, 0deg)`
- `base_link -> imu_link`: `xyz=(0.25, 0.0, 0.85)`,
  `rpy=(0deg, -20deg, 0deg)`

This static-only check did not cover the active live odom heading. After the
point cloud was made canonical, the remaining front/back issue was isolated to
the native Ranger child-frame heading being passed through unchanged.

The final fix keeps:

- `lidar_yaw = pi`
- `imu_yaw = pi`
- `lidar_level_yaw = pi`
- `odom_yaw_offset_rad = 0.0`
- `lidar_axis_roll = 0.0`
- `lidar_axis_pitch = 0.0`
- `lidar_axis_yaw = 0.0`
- `pointcloud_axis_remap.rotation_matrix = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]`
- `imu_axis_remap.rotation_matrix = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]`

Post-fix live Jetson verification for the lidar chain:

- `base_link -> lidar_mount_link`: `rpy=(0deg, -20deg, 180deg)`
- `lidar_mount_link -> lidar_link`: `rpy=(0deg, 0deg, 0deg)`
- `base_link -> imu_link`: `rpy=(0deg, -20deg, 180deg)`
- `base_link -> lidar_level_link`: `rpy=(0deg, 0deg, 180deg)`
- `/tf_static` publisher count remains `1`
- `src/robot_system_tests/test/test_workspace_contracts.py` passes in the
  Jetson container.

## Field correction: base_link and lidar consistency

The latest live scan-sector check confirmed that the public `/lidar_points`
stream must be canonicalized numerically before it is labeled `lidar_link`, and
the 2D slicing frame must remain aligned with `base_link +X`. The static sensor
transform must therefore stay:

- `base_link -> lidar_mount_link`: `xyz=(0.25, 0.0, 0.85)`, `rpy=(0, -20deg, 180deg)`
- `base_link -> lidar_mount_link`: carries the physical 180 degree yaw required by the current JT128 installation
- `lidar_mount_link -> lidar_link`: identity
- `pointcloud_axis_remap`: raw-to-canonical rotation

The remaining whole-vehicle heading mismatch investigation showed that the
temporary `odom_yaw_offset_rad=pi` was a visual compensation, not a valid
chassis-odom correction. Ranger SDK `/wheel/odom` is treated as the chassis
truth, so the sole dynamic TF owner now passes yaw through unchanged:

- Runtime uses `odom_yaw_offset_rad = 0.0`.
- Runtime uses `rotate_odom_position_with_yaw_offset = false`; passthrough mode
  only renames the SDK child frame to canonical `base_link`.
- Runtime uses `lidar_yaw = pi` and `imu_yaw = pi` for the JT128 install relation.
- Runtime uses `lidar_axis_yaw = 0.0`; JT128 raw-axis alignment is done inside
  the remap nodes before publishing canonical topics.
- Runtime labels upstream Ranger odom as `ranger_base_link` and only
  `robot_local_state` republishes canonical `base_link`.
- The static lidar installation remains unchanged, so point cloud orientation is not double-corrected.
