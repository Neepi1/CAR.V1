# Phase 2.4b Local Costmap Stamp Contract

Phase 2.4a proved that the local costmap can reject live obstacle clouds when
the cloud is already transformed into `base_link` but still carries an older
source acquisition stamp. Nav2 then asks its `odom` costmap TF buffer for
`odom <- base_link` at that old stamp and can report:

```text
Message Filter dropping message: frame 'base_link'
the timestamp on the message is earlier than all the data in the transform cache
```

## Contract

- `/lidar_points` keeps the JT128 source stamp and remains the canonical full
  density trunk.
- `/scan` and `/flatscan` keep their existing localization timing policy.
- `/perception/obstacle_points` and `/perception/clearing_points` are derived
  local-costmap products in `base_link`; in the Jetson runtime profile they
  are stamped from the latest fresh `/local_state/odometry.header.stamp`.
- Source stamp age remains available through status diagnostics as
  `obstacle_output_source_age_ms` and `clearing_output_source_age_ms`.
- If `/local_state/odometry` is missing or older than 250 ms, the worker falls
  back to the source pointcloud stamp and increments
  `local_worker_stamp_fallback_count` in `/lidar/pointcloud_accel_status`.
- No QoS, DDS/RMW, FAST-LIO2, EKF, Nav2 controller/planner, local costmap frame,
  or TF owner changes are part of this phase.

## Runtime Validation

After deploying and restarting the driver-integrated pipeline, verify:

```bash
ros2 topic info -v /perception/obstacle_points
ros2 topic delay /perception/obstacle_points
ros2 topic echo --once /lidar/pointcloud_accel_status
docker exec NJRH-car grep -E "Message Filter dropping message: frame 'base_link'" \
  /root/.ros/log/controller_server_*.log | tail -50
```

Expected result: `/perception/obstacle_points` remains owned by the accel core
process, `obstacle_output_header_stamp_source=local_odom` during normal
runtime, fallback count stays flat after startup, and continuous local-costmap
earlier-than-cache drops stop outside short startup warm-up windows.
