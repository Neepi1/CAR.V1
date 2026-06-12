# Phase 1.15 Flatscan Lifecycle Hardening

## Scope

This phase only hardens the `/flatscan` helper lifecycle and navigation
localization startup admission. It does not change `/lidar_points`, PointCloud2
QoS, DDS/RMW, JT128 timestamp policy, FAST-LIO2, EKF, Nav2 controller/planner,
App API, or the local costmap frame.

The local costmap `MessageFilter` timestamp drops seen on
`/perception/obstacle_points` are a separate follow-up. They are not treated as
the cause of the localization startup failure fixed here.

## Ownership Audit

| File | Section/function | Starts `/flatscan` publisher | Stops `/flatscan` publisher | Supervises `/flatscan` publisher | Profile affected | Failure mode | Required change |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `scripts/jetson/runtime_overlay/launch/jt128_localization_sensing.launch.py` | `laser_scan_to_flatscan` node | Yes, inside the legacy scan launch | By launch shutdown | Via launch process only | `legacy` | launch or child exits and `/flatscan` disappears | supervise launch or at least detect missing publisher |
| `scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_pipeline.sh` | `legacy)` | Starts `jt128_localization_sensing.launch.py` | SIGINT/wait on launch pid | Now checks launch pid and `/flatscan` publisher | `legacy` | parent only waited on driver and could miss scan-chain loss | monitor legacy launch or external `/flatscan` publisher |
| `scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_pipeline.sh` | `ipc_worker)` | Starts standalone `laser_scan_to_flatscan` | SIGINT/wait, then SIGTERM if needed | Restarts by PID and by missing `/flatscan` publisher while `/scan` still exists, with bounded max restarts | `ipc_worker` | helper exits, or process remains alive after its ROS node/publisher disappears | supervise standalone helper and publisher liveness |
| `scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_pipeline.sh` | `nitros)` | Starts standalone compatibility helper | SIGINT/wait, then SIGTERM if needed | Same as `ipc_worker` | `nitros` guarded profile | helper exits, or process remains alive after its ROS node/publisher disappears while `/scan` remains alive | supervise standalone helper and publisher liveness |
| `scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh` | topic owner/rate checks | No | No | Verifies owner and hz | all profiles | `/scan` exists but `/flatscan` missing still passed weakly | report `CASE_FLATSCAN_HELPER_DEAD`, `FLATSCAN_OWNER_OK`, `FLATSCAN_HZ_OK`, `FLATSCAN_NAV_STARTUP_GATE_OK` |
| `scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh` | `ensure_localization_stack_ready_for_navigation` | No | No | Admission gate | navigation startup | `/flatscan` timeout looked like generic localization failure | write `FLATSCAN_MISSING` with scan/helper/profile diagnostics |
| `scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_ab.sh` | A/B report | No | No | Reports observed owner/hz/helper state | all profiles | A/B lacked helper pid/restart count and CASE | report scan/flatscan owners, hz, helper pid, restart count, and CASE |

## Runtime Contract

`/flatscan` is a navigation localization startup hard requirement. The Isaac
occupancy grid localizer consumes FlatScan input, so `/scan` alone is not enough.

Profile ownership is:

- `legacy`: `/lidar_points_nav -> /points_nav -> /scan_raw -> /scan -> /flatscan`
  is launched by `jt128_localization_sensing.launch.py`; that launch owns
  `laser_scan_to_flatscan`.
- `ipc_worker`: `pointcloud_accel_axis_node` owns `/scan`; standalone
  `laser_scan_to_flatscan` remains the compatibility helper for `/flatscan`.
- future direct accel `/flatscan`: verification accepts
  `pointcloud_accel_axis_node` as owner, but this phase does not implement that
  publisher.

The pointcloud accel pipeline records helper state in:

```text
scripts/jetson/runtime_overlay/web_dashboard/runtime_logs/flatscan_helper_status.env
```

Important controls:

```bash
NJRH_FLATSCAN_HELPER_REQUIRED=true
NJRH_FLATSCAN_HELPER_RESTART=true
NJRH_FLATSCAN_HELPER_MAX_RESTARTS=5
NJRH_FLATSCAN_HELPER_RESTART_BACKOFF_SEC=1.0
NJRH_FLATSCAN_WAIT_SEC=30
NJRH_FLATSCAN_MIN_HZ=5.0
NJRH_FLATSCAN_PARAMS="${NJRH_OVERLAY_ROOT}/config/jt128_flatscan.yaml"
```

The standalone helper supervisor starts the package's real
`lib/jt128_nav_tools/laser_scan_to_flatscan` binary directly instead of using
the `ros2 run` wrapper, so the tracked PID is the long-lived ROS node process.
For standalone accel profiles, startup waits for `/scan` first, then starts the
helper and waits for `/flatscan`; this avoids treating a slow
`driver_integrated` startup as a dead helper. The supervisor checks both process
liveness and ROS graph liveness. If `/scan` still has a publisher but
`/flatscan` loses its publisher, the helper is treated as
`CASE_FLATSCAN_HELPER_DEAD` even when the `laser_scan_to_flatscan` process is
still alive. The restart path sends SIGINT and waits, then sends SIGTERM if
needed. It does not use SIGKILL.

## Startup Failure Reasons

Navigation localization admission now reports the concrete failing gate:

- `GLOBAL_LOCALIZATION_TRIGGER_SERVICE_MISSING`
- `GRID_SEARCH_LOCALIZATION_SERVICE_MISSING`
- `MAP_TOPIC_MISSING`
- `FLATSCAN_MISSING`
- `LOCALIZATION_RESULT_PUBLISHER_MISSING`

For `FLATSCAN_MISSING`, the log also prints:

- `/scan` publisher count
- whether `laser_scan_to_flatscan` is running
- current pointcloud accel profile
- suggested verify/restart command

## Verification

```bash
bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile ipc_worker --restart
sleep 20
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
ros2 topic info -v /scan
ros2 topic info -v /flatscan
ros2 topic hz /scan
ros2 topic hz /flatscan
pgrep -af "pointcloud_accel_axis|laser_scan_to_flatscan"
```

Expected `ipc_worker` result:

- `/scan` publisher is `pointcloud_accel_axis_node`.
- `/flatscan` publisher is `laser_scan_to_flatscan` compatibility helper.
- `/flatscan` is at least `5Hz`, typically about `8Hz..10Hz`.
- `verify_pointcloud_accel_profile.sh` prints `FLATSCAN_OWNER_OK=true`,
  `FLATSCAN_HZ_OK=true`, and `FLATSCAN_NAV_STARTUP_GATE_OK=true`.

Legacy rollback remains:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile legacy --restart
sleep 20
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
```

Expected `legacy` result:

- `/points_nav` owner is `nav_cloud_preprocessor`.
- `/scan` owner is `scan_republisher`.
- `/flatscan` owner is `laser_scan_to_flatscan`.
