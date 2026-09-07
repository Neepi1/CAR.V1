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
- Provides `/mapping/fastlio/cloud_registered_body` as the deskewed/body-frame source for the mapping-only leveled slice. `nav_cloud_preprocessor -> pointcloud_to_laserscan` publishes that corrected slice directly onto canonical `/scan` for `slam_toolbox`.
- Gives the resident `/lidar_points` publisher a stable Fast DDS `UDPv4 + SHM` profile and applies a separate mapping-session `UDPv4 + SHM` profile to FAST-LIO2, `nav_cloud_preprocessor`, and `pointcloud_to_laserscan`. Full-size same-container clouds negotiate SHM while UDP remains the fallback. Other resident processes keep the normal UDP profile. This restores high-rate corrected scans without reducing point density or changing QoS/timestamps; `NJRH_SLAM2D_FASTDDS_TRANSPORT=inherit` reverts the mapping-side participants on the next session.
- The live mapping session creates no `/mapping/scan`, LaserScan relay, or timestamp rewrite.
- Provides the C++ `fastlio_odom_bridge_node` for converting mapping-owned FAST-LIO2 `/mapping/fastlio/odometry` to `/mapping/fastlio_odometry` plus private `/tf_slam2d` (`mapping_odom -> base_link`) for `slam_toolbox`. Explicit diagnostic `LOCAL_STATE_MODE=fastlio` can still use the bridge as `/Odometry -> /fastlio/base_odometry` with `publish_tf=false`; `robot_local_state` remains the only owner of canonical `/local_state/odometry` and `odom -> base_link`. The legacy Python script remains installed for compatibility only and should not be used on the production path.
- The former `mapping_scan_tf_gate_node` remains buildable for historical diagnostics but is retired from production. There is no `/mapping/scan_raw` or `/mapping/scan`; corrected slices go straight to `/scan`. See [docs/slam_toolbox_direct_scan.md](docs/slam_toolbox_direct_scan.md).
- `run_projected_map.sh` proves the resident owner, starts and pair-validates FAST-LIO2, unregisters only the resident LaserScan publisher, proves zero `/scan` publishers, then admits and verifies exactly one mapping publisher. It applies the stamped TF gate to that corrected scan and restores the navigation publisher on every normal or trapped exit. API stop gives the ordered cleanup a 30-second graceful budget and does not report success until it independently proves the restored navigation owner.
- Isaac occupancy-grid relocalization consumes the derived `/lidar_points_nav` localization branch for the stationary global match; FAST-LIO2 is not the default continuous local odom source underneath navigation.
- Must remain upstream-config driven. The Jetson deployment carries `scripts/jetson/runtime_overlay/patches/fast_lio_reliable_lidar_qos.patch` for the reused upstream FAST-LIO2 source so lidar input QoS is configurable; mapping sets `/lidar_points` to best-effort/depth `1` and removes the default `/lidar_points_fastlio` identity hop to avoid large-message backpressure. This is a transport hardening patch, not a change to the FAST-LIO2 estimation math. The patched build is installed as a writable overlay under `${NJRH_FASTLIO_PATCHED_OVERLAY}` instead of overwriting the root-owned upstream install.
- Frontend metadata now records the canonical Fast-LIO input contract and does not expose a legacy remap fallback in the repository-owned runtime

## Live acceptance

Acceptance requires exactly one `/scan` publisher named
`pointcloud_to_laserscan` while mapping (`pointcloud_accel_axis_node` after mapping stops), fresh monotonically increasing LaserScan stamps,
the commissioned 1440-bin geometry, exact-stamp private-TF transformability,
and a fresh `/map` from `slam_toolbox`.
Probes must subscribe only to LaserScan/TF/status topics and must exit after the
measurement; no high-rate PointCloud2 subscription is required. Moving hardware
validation of the next mapping session remains required. The SM1 no-loop field
run produced worse accumulated yaw distortion, so loop closing is restored and
the remaining graph-admission settings are left unchanged pending separate
evidence.

The 2026-08-20 stationary transport acceptance observed `17.95 Hz` at the
corrected-cloud preprocessor input and `17.10 Hz` over 60 seconds on mapping
`/scan`, with zero duplicate or regressed timestamps. A separate 30-second
`ros2 topic hz` window reported `17.56 Hz` and a maximum inter-arrival gap of
`0.252 s`. Exactly one publisher, `pointcloud_to_laserscan`, owned `/scan` and
all bounded probes exited.
