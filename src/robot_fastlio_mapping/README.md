# robot_fastlio_mapping

Wrapper-only package for FAST-LIO2 frontend integration.

## Parameters

- `navigation_mode`: when true, wrapper must keep non-canonical TF isolated
- `publish_tf`: defaults to `false`
- `artifact_dir`: standardized frontend artifact root
- `frontend_pose_topic`: repository-owned frontend pose contract, defaults to `/mapping/frontend_pose`
- `local_config`: reuses `D:/codespace/car/ros2_ws/src/fast_lio/config/jt128.yaml`
- `upstream_points_topic`, `upstream_imu_topic`, `upstream_sensor_frame`: default to canonical `/lidar_points`, `/lidar_imu`, and `lidar_link` for wrapper metadata; the Jetson runtime uses `/lidar_points` directly for the estimator hot path
- upstream car config historically used `hesai_lidar_fastlio` and `send_odom_base_tf: true`; wrapper policy no longer permits that path in the repository-owned runtime

## Output Contract

- Emits `mapping_result/frontend_result/frontend_result.json`
- Exposes the repository-owned live draft pose contract on `/mapping/frontend_pose`
- Provides `/cloud_registered_body` as the default deskewed current-frame point cloud for live `slam_toolbox` 2D mapping.
- Provides the C++ `fastlio_odom_bridge_node` for converting FAST-LIO2 `/Odometry` to a base-link odom stream. The production use is live mapping: `/Odometry -> /mapping/fastlio_odometry` plus private `/tf_slam2d` (`mapping_odom -> base_link`) for `slam_toolbox`. Explicit diagnostic `LOCAL_STATE_MODE=fastlio` can still use it as `/Odometry -> /fastlio/base_odometry` with `publish_tf=false`; `robot_local_state` remains the only owner of canonical `/local_state/odometry` and `odom -> base_link`. The legacy Python script remains installed for compatibility only and should not be used on the production path.
- Isaac occupancy-grid relocalization consumes the derived `/lidar_points_nav` localization branch for the stationary global match; FAST-LIO2 is not the default continuous local odom source underneath navigation.
- Must remain upstream-config driven. The Jetson deployment carries `scripts/jetson/runtime_overlay/patches/fast_lio_reliable_lidar_qos.patch` for the reused upstream FAST-LIO2 source so lidar input QoS is configurable; mapping sets `/lidar_points` to best-effort/depth `1` and removes the default `/lidar_points_fastlio` identity hop to avoid large-message backpressure. This is a transport hardening patch, not a change to the FAST-LIO2 estimation math. The patched build is installed as a writable overlay under `${NJRH_FASTLIO_PATCHED_OVERLAY}` instead of overwriting the root-owned upstream install.
- Frontend metadata now records the canonical Fast-LIO input contract and does not expose a legacy remap fallback in the repository-owned runtime
