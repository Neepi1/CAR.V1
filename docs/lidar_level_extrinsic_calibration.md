# Lidar Level Extrinsic Calibration

This procedure calibrates the planar `base_link -> lidar_level_link` transform
used by Isaac relocalization, AMCL scan admission, Nav2 scan costmaps, and
post-relocalization pose checks.

Scope:

- Calibrate only `lidar_x`, `lidar_y`, and optionally `lidar_yaw` in
  `sensors.yaml`.
- Do not change FAST-LIO2, JT128 pointcloud/IMU transport, Ranger odom, Nav2
  planner/controller plugins, or the safety command chain.
- Restart the full runtime after applying a candidate transform:
  `sudo systemctl restart njrh-runtime.service`.

## Why Four Headings

At the same physical floor point, an incorrect planar sensor offset makes the
relocalized `map -> base_link` pose trace a small circle as the robot faces
different directions. For each static sample:

```text
observed_base_xy = physical_center_xy + R(base_yaw) * (true_lidar_xy - configured_lidar_xy)
```

The fitter solves this linear system for the XY error. The suggested update is:

```text
new_lidar_x = current_lidar_x + fitted_error_x
new_lidar_y = current_lidar_y + fitted_error_y
```

Absolute yaw is different: same-spot 0/90/180/270 samples prove yaw step
consistency, but they do not identify absolute sensor yaw unless one heading is
known in the map. Provide `--expected-first-heading-deg` only when the robot is
physically aligned to a known map direction at heading `0`.

## Capture

Run this inside the Jetson container with the robot on open floor and stopped:

```bash
cd /workspaces/njrh-v3/workspace1
bash scripts/jetson/runtime_overlay/scripts/run_lidar_level_extrinsic_calibration.sh \
  --repeat 2 \
  --angles-deg 0,90,180,270
```

The script captures eight static relocalization samples and spins between
headings through:

```text
/cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base
```

If an operator wants to rotate the robot manually, use:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_lidar_level_extrinsic_calibration.sh \
  --manual-step \
  --repeat 2 \
  --angles-deg 0,90,180,270
```

For yaw calibration, align heading `0` to a known map yaw and pass it:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_lidar_level_extrinsic_calibration.sh \
  --repeat 2 \
  --angles-deg 0,90,180,270 \
  --expected-first-heading-deg 0
```

## Output

The report is written under:

```text
reports/lidar_extrinsic_calibration/<timestamp>_<label>/summary.md
reports/lidar_extrinsic_calibration/<timestamp>_<label>/calibration_fit.json
```

Acceptance targets for a good candidate:

- XY fit residual RMS <= `0.03 m`.
- XY fit residual max <= `0.05 m`.
- Post-apply repeated four-heading relocalization has no stable one-sided
  lateral correction larger than `0.05 m`.
- If yaw was calibrated with a known heading, yaw residual RMS <= `1.0 deg`.

Apply the candidate only after reviewing the report. The current field
candidate is `x=0.3450, y=0.0000, z=0.85, yaw=3.1764992386296798`; it should
be treated as provisional until a post-apply four-heading validation run shows
that the fitted follow-up correction is near zero. Update both runtime overlay
and source config so future deploys do not revert the calibration:

- `scripts/jetson/runtime_overlay/config/sensors.yaml`
- `src/robot_description/config/sensors.yaml`
