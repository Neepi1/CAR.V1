# Phase 2.4a Local Costmap Timestamp Audit

This phase is a read-only root-cause audit for local costmap
`MessageFilter` drops such as:

```text
Message Filter dropping message: frame 'base_link' ...
the timestamp on the message is earlier than all the data in the transform cache
```

It does not change runtime behavior. It does not restamp obstacle clouds to
`now`, change `/lidar_points`, alter PointCloud2 QoS, change DDS/RMW, change
FAST-LIO2, EKF, Nav2 controller/planner plugins, App API, `tf_filter_tolerance`,
or local costmap frame settings.
In short: this phase does not restamp clouds.

## Static Audit

| File | Module | Profile | Obstacle Publisher | Clearing Publisher | Output Stamp Source | Output Frame | Source Buffer Stamp | Publish Time Measured | Source Age Measured | Header Age Measured | Added Diagnostics |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `src/robot_hesai_jt128/src/pointcloud_accel_axis_node.cpp` | `pointcloud_accel_axis_node` local worker | `ipc_worker` | yes | yes | latest normalized source stamp | `base_link` | normalized `/lidar_points` stamp | yes | yes | yes | raw/header age, latest buffer stamp/update age, obstacle/clearing/scan source/header age, frame IDs, suspect counters |
| `src/robot_hesai_jt128/src/pointcloud_accel_axis_node.cpp` | scan worker | `ipc_worker` | no | no | latest normalized source stamp | `lidar_level_link` | normalized `/lidar_points` stamp | existing processing timing | yes | yes | scan source/header age and frame ID |
| `src/robot_local_perception/src/local_perception_node.cpp` | `robot_local_perception` | `legacy` rollback | yes | yes | input cloud stamp by default | `base_link` | input cloud stamp | existing processing timing | yes | yes | input receive/header age and obstacle/clearing output source/header age |
| `scripts/jetson/runtime_overlay/config/nav2.yaml` | local costmap VoxelLayer | both | consumes only | consumes only | n/a | sensor frame `base_link` | n/a | n/a | n/a | n/a | verified by read-only script |

## Runtime Interpretation

`/perception/obstacle_points` running at 10-12 Hz only proves that the publisher
is producing messages and DDS delivery is possible. It does not prove the local
costmap accepts those messages. The local costmap also needs a compatible frame
contract and a TF cache that can transform the message timestamp.

The new status fields separate the likely causes:

- `raw_header_age_ms`: source cloud is already old before acceleration.
- `latest_internal_buffer_stamp_age_ms`: latest in-process buffer carries an old source stamp.
- `latest_internal_buffer_update_age_ms`: in-process buffer was not refreshed recently.
- `obstacle_output_header_age_ms`: local costmap receives a cloud whose header stamp is old.
- `obstacle_output_source_age_ms`: age of the original source stamp used by the obstacle output.
- `tf_drop_suspect_obstacle_header_age_over_100ms_count` and `_over_200ms_count`: output frames that are old enough to be plausible MessageFilter drop candidates.

## Cases

| Case | Meaning | Recommendation |
| --- | --- | --- |
| `CASE_A_RAW_STAMP_ALREADY_OLD` | Raw source stamp is already old. | Audit driver timestamp and clock policy first. |
| `CASE_B_INTERNAL_BUFFER_STALE` | Raw stamp is normal but the accel buffer is stale. | Audit worker scheduling and latest buffer update latency. |
| `CASE_C_OUTPUT_REUSES_OLD_SOURCE_STAMP` | Output faithfully reuses an old source stamp. | Consider a later publish-time plus max-source-age gate. |
| `CASE_D_TF_CACHE_TIME_AHEAD` | Output stamp is fresh but MessageFilter still drops. | Audit `odom -> base_link` TF stamps/cache/clock. |
| `CASE_E_STARTUP_TF_CACHE_WARMUP` | Drops only happen during costmap startup. | Add a later TF cache warm-up gate. |
| `CASE_F_FRAME_MISMATCH` | Frame contract is wrong. | Fix `frame_id` or `sensor_frame`. |
| `CASE_G_UNKNOWN_NEEDS_BAG` | Status fields are missing or contradictory. | Record a short targeted bag. |

Run:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_local_costmap_observation_timestamp_root_cause.sh
```

The script is read-only. It writes
`reports/local_costmap_timestamp_audit_<timestamp>.md` and suggests a next
phase, but it does not apply any fix.
