# Phase 1.13 PointCloud Acceleration Profile

## Scope

Phase 1.13 keeps the JT128 mapping trunk unchanged and adds reversible navigation-branch acceleration profiles:

- `legacy`: current validated path and rollback default.
- `ipc_worker`: `pointcloud_accel_axis_node` publishes the full `/lidar_points` trunk first, then same-process workers derive `/perception/obstacle_points`, `/perception/clearing_points`, and `/scan` from the latest normalized cloud.
- `nitros`: guarded experimental profile for future Isaac ROS NITROS compact navigation branches. It never replaces `/lidar_points`.

Default remains:

```bash
export NJRH_POINTCLOUD_ACCEL_PROFILE="${NJRH_POINTCLOUD_ACCEL_PROFILE:-legacy}"
```

## Audit Summary

| File | Current role | Default nav path | Full PointCloud2 pub/sub | Affects `/lidar_points` trunk | Change |
| --- | --- | --- | --- | --- | --- |
| `src/robot_hesai_jt128/src/pointcloud_axis_remap_node.cpp` | Legacy canonical remap plus synchronous derived branches | Yes in `legacy` | Publishes `/lidar_points`, `/lidar_points_nav`, optional local branch | Yes, single trunk publisher in legacy | Kept as rollback; status fields extended only |
| `src/robot_hesai_jt128/src/pointcloud_accel_axis_node.cpp` | New fast trunk plus async worker implementation | Yes in `ipc_worker`/`nitros` | Publishes full `/lidar_points`; compact debug branches are XYZ/XYZI only | Yes, single trunk publisher in accel profiles | Added |
| `scripts/jetson/runtime_overlay/config/pointcloud_accel_axis.yaml` | Accel profile params | Yes in accel profiles | Trunk full; compact local/nav debug branches | No trunk downsample params | Added |
| `scripts/jetson/runtime_overlay/scripts/run_driver.sh` | JT128 ingress owner | Yes | Starts either legacy remap or accel remap | Owns single trunk publisher selection | Profile-aware |
| `scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_pipeline.sh` | Profile runtime wrapper | Yes in accel profiles | Starts driver and `/scan -> /flatscan` converter | Reuses trunk unless explicit restart | Added |
| `scripts/jetson/runtime_overlay/scripts/run_local_perception.sh` | Legacy local obstacle worker | Yes only in `legacy` | Subscribes profile-selected input | No | Kept as rollback |
| `scripts/jetson/runtime_overlay/launch/jt128_localization_sensing.launch.py` | Legacy `/lidar_points_nav -> /points_nav -> /scan -> /flatscan` chain | Yes only in `legacy` | Carries derived PointCloud2 DDS hops | No | Kept as rollback |
| `scripts/jetson/runtime_overlay/launch/occupancy_localization.launch.py` | Isaac localizer without legacy sensing chain | Yes in accel profiles | Consumes `/flatscan` only | No | Reused by accel profiles |
| `src/robot_isaac_nitros_pointcloud` | NITROS skeleton | No default | Disabled unless build option is enabled | No | Added safe skeleton |

Confirmed constraints:

- `/lidar_points` is still full-density/full-fields and remains the FAST-LIO2 mapping input together with `/lidar_imu`.
- `/lidar_points` publisher is selected by profile but must remain exactly one live publisher.
- `legacy` continues to use `/_internal/lidar_points_local -> robot_local_perception` and `/lidar_points_nav -> /points_nav -> /scan -> /flatscan`.
- `ipc_worker` removes `/_internal/lidar_points_local` and `/points_nav` as production DDS hops; they remain compact debug/compat topics only.
- No Nav2 controller/planner, EKF, FAST-LIO2 mapping logic, App API, timestamp policy, DDS middleware, or PointCloud2 reliable-QoS default is changed.

## Runtime Commands

Check NITROS environment:

```bash
bash scripts/jetson/runtime_overlay/scripts/check_isaac_ros_nitros_env.sh
```

Legacy baseline:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile legacy --restart
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
```

`ipc_worker`:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile ipc_worker --restart
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
ros2 topic hz /perception/obstacle_points
ros2 topic hz /scan
ros2 topic hz /flatscan
timeout 12 ros2 topic echo /lidar/axis_remap_status --field data
```

`nitros`, only when the environment check passes:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile nitros --restart
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
```

A/B:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_ab.sh --profile legacy --duration-sec 120 --apply --restart
bash scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_ab.sh --profile ipc_worker --duration-sec 120 --apply --restart
bash scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_ab.sh --profile nitros --duration-sec 120 --apply --restart
```

Rollback:

```bash
bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile legacy --restart
bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
```

## Acceptance

Pass criteria for `ipc_worker`/`nitros` field validation:

- `/lidar_points` publisher count is `1`.
- `/lidar_points` status publish rate is at least 18 Hz while the vendor raw stream is healthy.
- `/lidar_points` remains full-density/full-fields; compact settings do not affect trunk output.
- FAST-LIO2 mapping still subscribes to `/lidar_points` and `/lidar_imu`.
- `/perception/obstacle_points` is at least 10 Hz, or 9-10 Hz with stable processing and no stale backlog warning.
- `/scan` and `/flatscan` are at least 8 Hz.
- Nav2 topics remain unchanged: `/perception/obstacle_points`, `/perception/clearing_points`, `/scan`, `/flatscan`.
- No FAST-LIO2 mapping process is resident during normal navigation.
