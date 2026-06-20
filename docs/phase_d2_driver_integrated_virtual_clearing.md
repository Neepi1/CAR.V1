# Phase D2 Driver-Integrated Virtual Clearing

## Scope

This phase fixes stale local VoxelLayer cells in the JT128 accel local worker.
It keeps the existing driver-integrated topology:

```text
JT128 SDK callback
  -> in-process PointCloud2
  -> PointCloudAccelCore
  -> /lidar_points
  -> /perception/obstacle_points
  -> /perception/clearing_points
  -> /scan
```

No architecture contract changes are made:

- `/lidar_points` remains the full-density/full-fields trunk.
- `/perception/obstacle_points` remains the Nav2 local costmap obstacle source.
- `/perception/clearing_points` remains a clearing-only local costmap input.
- `map->odom` stays owned by `robot_localization_bridge`.
- `odom->base_link` stays owned by `robot_local_state`.
- FAST-LIO2 remains mapping-only in standard navigation.
- DDS/QoS, TF ownership, Nav2 planner/controller parameters, and MPPI are unchanged.
- Only the derived local-costmap clouds (`/perception/obstacle_points` and
  `/perception/clearing_points`) use the TF-aligned local odom stamp described
  below. The `/lidar_points` trunk, `/scan`, raw JT128 timestamps, and mapping
  chain are unchanged.

## Root Cause

Field logs showed Nav2 active and the driver-integrated pointcloud ingress
healthy, but MPPI aborted immediately with result code `6` (`STATUS_ABORTED`).
Read-only probes showed the global costmap and delivery targets were free, while
the local costmap around `base_link` was mostly lethal. At the same time,
`/perception/obstacle_points` had no near returns and
`/perception/clearing_points` contained only real JT128 returns starting more
than two meters away.

That means the local worker could mark obstacles, but it did not consistently
publish near and multi-height clearing rays to remove stale VoxelLayer cells
around the robot.

A later field check found the virtual clearing cloud was being published and
received, but local VoxelLayer cells near `base_link` still stayed high/lethal
after the current `/perception/obstacle_points` cloud had no near returns. The
local VoxelLayer now uses `combination_method: 0` so clearing-only rays can
lower stale costs in the master grid instead of preserving old lethal cells
through MAX combination.

## Implementation

`PointCloudAccelCore` now builds bounded virtual clearing rays in the local
worker when `clearing_worker_virtual_rays_enabled=true`.

Runtime defaults:

```yaml
local_worker_stamp_source: local_odom
local_worker_stamp_odom_topic: /local_state/odometry
local_worker_stamp_max_odom_age_sec: 0.25
clearing_worker_virtual_rays_enabled: true
clearing_worker_virtual_ray_angle_resolution_deg: 1.0
clearing_worker_virtual_ray_min_angle_deg: -110.0
clearing_worker_virtual_ray_max_angle_deg: 110.0
clearing_worker_virtual_rays_allow_self_mask_endpoints: true
clearing_worker_virtual_ray_range: 8.00
clearing_worker_virtual_ray_range_steps: [0.10, 0.15, 0.20, 0.35, 0.50, 0.75, 1.00, 1.50, 2.50, 4.00, 6.00, 8.00]
clearing_worker_virtual_ray_endpoint_z_values: [-0.10, 0.05, 0.20, 0.40, 0.60, 0.80, 1.00, 1.20, 1.40]
clearing_worker_max_points: 30000
```

Nav2 local VoxelLayer:

```yaml
combination_method: 0
```

For each angular bin, the worker records the farthest valid clearing return. It
then emits virtual endpoints up to that return range, or to the configured
virtual range when no return exists in that bin.

Obstacle marking still excludes the configured self mask. Virtual clearing
uses the same angular observation window as obstacle marking, but its endpoints
are allowed inside that self-mask region by default so near-body stale
VoxelLayer cells are actively cleared instead of accumulating around the robot
footprint.

The generated cloud keeps:

- frame: `base_link`
- stamp: latest fresh `/local_state/odometry.header.stamp` in the Jetson
  runtime profile; if that stamp is missing or older than 250 ms, the worker
  falls back to the source pointcloud stamp and increments
  `local_worker_stamp_fallback_count` in `/lidar/pointcloud_accel_status`
- topic: `/perception/clearing_points`

## Validation

After deploying to the Jetson, verify:

1. `/perception/clearing_points` is published by `hesai_accel_driver_node`.
2. Clearing point count is roughly the virtual ray envelope rather than only the
   small set of real JT128 returns.
3. The local costmap cells around `base_link` clear after restart while current
   `/perception/obstacle_points` has no near returns.
4. Two consecutive App/API navigation targets do not immediately fail from
   local-costmap stale obstacle cells.

Remaining real-hardware checks:

- Validate near-person and near-wall behavior at low speed.
- Confirm `collision_monitor` still stops on close body-risk returns.
- Run at least one 20-minute standard navigation pass with driver-integrated
  ingress and FAST-LIO2 stopped.
