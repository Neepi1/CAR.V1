# Localization Runtime Validation

Date: 2026-04-22  
Workspace: `C:\Users\86236\Desktop\workspace1`  
Jetson host: `192.168.31.23`  
Jetson workspace: `/home/nvidia/workspaces/njrh-v3/workspace1`  
Container: `NJRH-car`

## Scope

This validation covers the repository-owned JT128 canonical ingress, the leveled 2D slice chain, and Web-triggered standard navigation on `test-16`.

Changes validated in this run:

- `run_driver.sh` now keeps repository-owned canonical ingress, preferring the compiled `robot_hesai_jt128` pointcloud and imu remap nodes and falling back to the Python helpers only when the compiled binaries are unavailable
- `robot_local_perception` now subscribes to `/lidar_points` with sensor-data QoS
- `run_projected_map.sh` and `run_occupancy_grid_localization.sh` now kill stale `scan_flip_republisher.py` before relaunch

## Live Verification

### 1. Canonical lidar ingress

Command result on Jetson:

- `ros2 topic info -v /lidar_points`

Observed:

- publisher count: `1`
- publisher node: `pointcloud_axis_remap`
- publisher QoS: `RELIABLE`, `VOLATILE`
- subscriber count: `2`
- subscriber: `nav_cloud_preprocessor`, QoS `BEST_EFFORT`
- subscriber: `robot_local_perception`, QoS `BEST_EFFORT`

This confirms the previous `/lidar_points` QoS incompatibility with `robot_local_perception` is no longer present in the current runtime.

Command result on Jetson:

- `ros2 topic echo /lidar_points --once`

Observed:

- `/lidar_points.header.frame_id = lidar_link`

### 2. Leveled slice input

Command result on Jetson:

- `ros2 topic echo /points_nav --once`

Observed:

- `/points_nav.header.frame_id = lidar_level_link`

This confirms the preprocessor is publishing the navigation slice cloud in the leveled frame, not just rewriting the scan frame after slicing.

### 3. 2D scan output

Command result on Jetson:

- `ros2 topic info -v /scan`

Observed:

- publisher count: `1`
- publisher node: `scan_flip_republisher`
- subscriber count: `1`
- subscriber node: `laser_scan_to_flatscan`

This confirms the duplicate `/scan` publisher problem is cleared in the current runtime.

Command result on Jetson:

- `ros2 topic echo /scan --once`

Observed:

- `/scan.header.frame_id = lidar_level_link`
- `range_min = 0.25`
- `range_max = 40.0`

### 4. Flatscan output

Command result on Jetson:

- `ros2 topic echo /flatscan --once`

Observed:

- `/flatscan.header.frame_id = lidar_level_link`

### 5. pointcloud_to_laserscan slicing frame

Repository contract in:

- `scripts/jetson/runtime_overlay/launch/jt128_localization_sensing.launch.py`
- `scripts/jetson/runtime_overlay/config/jt128_scan_slam2d.yaml`

Observed:

- `pointcloud_to_laserscan` consumes `cloud_in = /points_nav`
- `/points_nav` is already in `lidar_level_link`
- `pointcloud_to_laserscan.target_frame = lidar_level_link`

Conclusion:

- the actual 2D slice is performed in `lidar_level_link`
- this is not a "slice first, relabel later" path

## Standard Navigation Validation

Target map:

- `test-16`

Dashboard trace on Jetson:

- `2026-04-22 14:29:29` `HTTP /api/navigation/start map_name=test-16`
- `2026-04-22 14:30:05` localization stack start requested
- `2026-04-22 14:30:39` localizer log: `Triggering localization now.`
- `2026-04-22 14:30:44` Nav2 stack start requested
- `2026-04-22 14:30:54` `start_navigation success map=test-16 profile=standard`

Dashboard status after startup:

- `mode_running = true`
- `localization_running = true`
- `navigation_running = true`
- `phase = ready`
- `summary = 标准导航 is ready`
- `map_pose_ready = true`
- `last_relocalize.status = succeeded`
- `last_relocalize.message = grid search localization succeeded on attempt 1/3`

## Notes

- `localization_result` is still effectively a one-shot event topic in this runtime. It is not expected to remain continuously visible after the relocalization event has been consumed.
- `navigation.status.localization_pose` may be `null` even when `map_pose_ready=true` and standard navigation is already ready. The authoritative success signal in the current runtime is:
  - `last_relocalize.status = succeeded`
  - `map_pose_ready = true`
  - `phase = ready`
- `robot_local_perception` still logs a transient TF warning during early startup if `base_link <- lidar_link` is queried before the static chain is fully available. In the current run that warning was transient and did not block navigation startup.
