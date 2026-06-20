# robot_local_perception

This package is retired from the production navigation runtime.

Production local dynamic-obstacle handling now follows the standard Nav2
LaserScan flow:

- `/scan` is produced by the JT128 accel scan worker.
- Nav2 `local_costmap` uses `nav2_costmap_2d::ObstacleLayer`.
- The single observation source is `/scan` with `marking=true`,
  `clearing=true`, and `inf_is_valid=true`.
- `collision_monitor` also consumes `/scan`.

The previous custom PointCloud2 obstacle path is intentionally disabled:

- no production local PointCloud2 obstacle publisher
- no production local PointCloud2 clearing publisher
- no production `robot_local_perception` process
- no production local-costmap subscription to derived PointCloud2 clouds

`scripts/jetson/runtime_overlay/scripts/run_local_perception.sh` exits
intentionally, and the default config sets `enabled=false` with empty input and
output topics. Keeping this source directory is only for historical review and
explicit rollback auditing; it must not be started in normal navigation.
