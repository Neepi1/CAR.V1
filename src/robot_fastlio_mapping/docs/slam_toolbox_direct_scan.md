# slam_toolbox corrected-scan ownership

## Production graph

Navigation and mapping share canonical `/scan`, but never share publisher ownership.

```text
navigation: JT128 -> pointcloud_accel_axis_node -> /lidar_points + /scan

mapping: /lidar_points + /lidar_imu
  -> mapping-owned FAST-LIO2
  -> /mapping/fastlio/cloud_registered_body
  -> nav_cloud_preprocessor (/mapping/points_nav)
  -> pointcloud_to_laserscan
  -> /scan
  -> slam_toolbox
```

The internal `/mapping/points_nav` hop is mapping-scoped. There is no
`/mapping/scan`, `/scan_raw`, LaserScan relay, or timestamp rewrite.

Full-size local PointCloud2 delivery is the one transport exception. The
resident `/lidar_points` owner loads a stable Fast DDS `UDPv4 + SHM` profile
before its participant is created. A mapping session independently applies a
`UDPv4 + SHM` profile to mapping-owned FAST-LIO2,
`nav_cloud_preprocessor`, and `pointcloud_to_laserscan`. Each SHM participant
uses a 128 MiB segment. Same-container full-size cloud endpoints therefore
negotiate SHM, while UDP remains available for the vendor-raw ingress,
remote/legacy subscribers, and small-message consumers. The converter still
publishes canonical `/scan` directly. `slam_toolbox`, the mapping odom bridge,
API, TF/local state, Nav2, and probes keep the normal UDP participant profile.
`NJRH_SLAM2D_FASTDDS_TRANSPORT=inherit` is the mapping-consumer rollback and
takes effect on the next mapping start.

## Atomic handoff

`pointcloud_accel_axis_node` exposes private service
`~/set_scan_output_enabled` (`std_srvs/srv/SetBool`). Disabling it destroys only
the LaserScan publisher; the pointcloud trunk remains alive. The mapping runner:

1. proves exactly one resident `/scan` owner and fresh data;
2. starts and pair-validates mapping FAST-LIO2 cloud/odom and private TF;
3. disables resident scan output and proves publisher count is zero;
4. starts the corrected-cloud slice and proves exactly one publisher named
   `pointcloud_to_laserscan`;
5. proves fresh data and three original-stamp transforms to mapping odom;
6. runs the mapping session.

Cleanup stops mapping slice processes first and then restores/proves the
navigation owner. The API sends `SIGINT` and gives this ordered transaction up
to 30 seconds before escalation; it then independently requires exactly one
`pointcloud_accel_axis_node` publisher on `/scan` before the stop response can
succeed. If the mapping launcher exited before restoring the publisher, the API
makes one idempotent SetBool recovery request and verifies the graph result.
Navigation startup repeats the same restore, covering an interrupted mapping
launcher or API process.

## Frozen geometry

`jt128_scan_slam2d.yaml` keeps target frame `lidar_level_link`, height
`[-0.75, 0.35] m`, 1440 angular bins, range `[0.25, 40.0] m`, and the original
source stamp.

This correction does not modify JT128 QoS, FAST-LIO2 estimator parameters, TF
ownership, slam_toolbox loop/scan-matching parameters, or the velocity chain.
It also does not add a second scan path or subscribe a diagnostic probe to
PointCloud2. `verify_mapping_scan_throughput.py` observes only `/scan` plus the
low-rate preprocessor status topic and exits after its bounded measurement.

## 2026-08-20 transport acceptance

- corrected-cloud preprocessor input: `17.95 Hz`;
- mapping `/scan`: `17.10 Hz` over 60 seconds, 1026 scans;
- duplicate/regressed source stamps: `0 / 0`;
- second 30-second interval window: `17.56 Hz`, maximum gap `0.252 s`;
- `/scan` publisher: exactly one, `pointcloud_to_laserscan`;
- no residual throughput or `ros2 topic hz` probe process.
