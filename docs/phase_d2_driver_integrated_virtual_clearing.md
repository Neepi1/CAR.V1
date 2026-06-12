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
- DDS/QoS, PointCloud2 timestamps, TF ownership, Nav2 planner/controller parameters, and MPPI are unchanged.

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

## Implementation

`PointCloudAccelCore` now builds bounded virtual clearing rays in the local
worker when `clearing_worker_virtual_rays_enabled=true`.

Runtime defaults:

```yaml
clearing_worker_virtual_rays_enabled: true
clearing_worker_virtual_ray_angle_resolution_deg: 1.0
clearing_worker_virtual_ray_range: 8.00
clearing_worker_virtual_ray_range_steps: [0.50, 1.00, 2.00, 3.50, 5.50, 8.00]
clearing_worker_virtual_ray_endpoint_z_values: [-0.10, 0.05, 0.20, 0.40, 0.60, 0.85, 1.10, 1.30]
clearing_worker_max_points: 15000
```

For each angular bin, the worker records the farthest valid clearing return. It
then emits virtual endpoints up to that return range, or to the configured
virtual range when no return exists in that bin. Endpoints inside the configured
self mask are skipped.

The generated cloud keeps:

- frame: `base_link`
- stamp: source pointcloud stamp
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
