# robot_fastlio_mapping

Wrapper-only package for FAST-LIO2 frontend integration.

## Parameters

- `navigation_mode`: when true, wrapper must keep non-canonical TF isolated
- `publish_tf`: defaults to `false`
- `artifact_dir`: standardized frontend artifact root
- `frontend_pose_topic`: repository-owned frontend pose contract, defaults to `/mapping/frontend_pose`
- `local_config`: reuses `D:/codespace/car/ros2_ws/src/fast_lio/config/jt128.yaml`
- `upstream_points_topic`, `upstream_imu_topic`, `upstream_sensor_frame`: default to canonical `/lidar_points`, `/lidar_imu`, and `lidar_link`
- upstream car config historically used `hesai_lidar_fastlio` and `send_odom_base_tf: true`; wrapper policy no longer permits that path in the repository-owned runtime

## Output Contract

- Emits `mapping_result/frontend_result/frontend_result.json`
- Exposes the repository-owned live draft pose contract on `/mapping/frontend_pose`
- Must remain upstream-config driven; wrapper does not modify FAST-LIO2 core source
- Frontend metadata now records the canonical Fast-LIO input contract and does not expose a legacy remap fallback in the repository-owned runtime
