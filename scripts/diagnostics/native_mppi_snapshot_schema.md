# NAVLITE MPPI internal snapshot, schema 1

Diagnostic-only candidate. Default disabled; no deployment authorization is implied.

The recorder writes `/tmp/njrh_navlite_capture.request` atomically:

```json
{"schema":1,"session":"unique_ascii_id","output_dir":"/tmp/njrh_reports/existing_capture","max_bytes":268435456,"deadline_monotonic_ns":1234567890123456}
```

Deadline is absolute Linux CLOCK_MONOTONIC nanoseconds, same host/boot as the controller,
at most one hour ahead. Removing the request disables diagnostics. A session ID is
single-use within this plugin instance: expiration/error/budget cannot silently restart it.
The writer checks the request every 500 ms. Control code checks only an atomic enabled
generation and never reads request files. No ROS entities are added.
Optional `min_free_bytes` defaults to 64 MiB; it is checked by the writer at most 2 Hz.
Low disk stops only diagnostics. The CLI should use recording-duration + 10 seconds for
the deadline, then remove its own request on normal exit; expiry is marked incomplete.

Each plugin instance creates an exclusive `mppi_<pid>_<monotonic_ns>/` under output_dir:

- `schema.json`: schema, endianness, capacities, process ID, session.
- `frames.jsonl`: one metadata/index JSON per successfully written frame.
- `payload.bin`: raw binary blocks described by `{offset,length,dtype,shape}` in each frame.
- `status.json`: written on close; reason, counts, dropped frames, stale generations,
  write errors, complete. Missing status means the capture was not confirmed closed.
  `dropped` is this session's difference from `dropped_baseline`, not the lifetime
  total `dropped_lifetime`. Pending/in-flight frames at close force incomplete.

Payload offsets are absolute bytes. `f8` is native-endian IEEE754 binary64, `i8` signed
64-bit, `u1` unsigned byte; read schema.byte_order explicitly. Blocks appear in this order:

| frame key | shape | meaning |
|---|---|---|
| costmap | [height,width] u1 | exact master grid copied under the controller's existing map lock; row-major |
| path | [N,7] f8 | actual transformed/pruned reference, x,y,z,qx,qy,qz,qw |
| path_stamps | [N] i8 | individual PoseStamped source stamps, seconds×1e9+nanoseconds |
| sequence | [T,3] f8 | final post-filter **pre-shift** control sequence vx,vy,wz |

Paths have header `path_frame`, `path_stamp_ns`; `path_frames_all_equal_header=true`
means every individual frame_id equals path_frame. Otherwise `path_frames` contains N IDs.
`path_original_count`, `sequence_original_count` expose capacity truncation.

Frame metadata:

- `compute_seq`, `start_monotonic_ns`, `end_monotonic_ns`, `start_wall_ns`, `session`.
- `map_copy_monotonic_ns`, `model_input_monotonic_ns` distinguish lock/capture timing;
  these are diagnostic copy times, never substituted for sensor/header timestamps.
- `pose`=[x,y,z,qx,qy,qz,qw], `pose_stamp_ns`, `pose_frame`.
- `odom` and `command`=[vx,vy,vz,wx,wy,wz]. Odom is the Twist passed to this controller,
  not a fresh wheel message. `odom_source_stamp=null`, `odom_covariance=null` explicitly.
- `resolution`, `origin`=[x,y], `map_dimensions`=[width,height], `map_frame`, `base_frame`,
  `footprint`=[[x,y,z],...]. Grid has no acquisition source stamp: compute copy is not sensor time.
- `model_enabled`, `model_dt`, `constraints`=[vx_min,vx_max,vy,wz], `min_turning_radius`.
- `dynamics`=[linear_delay,linear_tau,acceleration,deceleration,steering_delay,
  steering_tau,wheelbase,track]; `smoother_limits`=[linear_accel,linear_decel,
  angular_accel,angular_decel], positive magnitudes.
- `smoother_valid`, `smoother_age_sec` (-1 unavailable), `smoother_initial`=[vx,wz],
  `issued_age_sec` (-1 unavailable), `issued`=[[relative_time_sec,vx,wz],...]. These are
  the inputs actually used by RangerMotionModel, not newly queried states.
- `command_offset`: native selector offset (0 or 1); `command_returned`, `sequence_valid`.
- `stage`, `exception_type`, `exception`, `fail_flag`, `optimize_passes`,
  `passes_entered_failed`, `incomplete`, `dropped_before`.

Failure frames have `command_returned=false`; no final sequence is claimed if filtering
or command selection did not complete. Sequence is not the visualization's shifted output.
No trajectory is recomputed in the control callback. Offline replay must use the same
response_model/motion_model implementation and these captured inputs; it remains a
prediction, not measured future motion or a proof the real scan points were identical.

Capacities: 32 queue slots; 65536 grid cells; 2048 path poses; 256 steps; 128 history;
32 footprint points. Exceeding any capacity sets incomplete and original counts remain
visible. Full queue increments dropped, without waiting or modifying control. No exact
internal reconstruction may be claimed from incomplete/truncated/missing frames.

`status.complete` concerns capture transport only, never navigation or safety acceptance.
The recorder's shutdown/expiry/budget affect diagnostics only, not navigation lifetimes.

## Offline replay

Build optional `navlite_mppi_replay` target, then:

```sh
navlite_mppi_replay --input /tmp/njrh_reports/CAPTURE/mppi_PID_INSTANCE --output /tmp/njrh_reports/NEW_REPLAY
```

It initializes no ROS context or node. It reuses the actual `ResponseModel`,
`RangerMotionModel::smooth`, and installed Humble `integrateStateVelocities`.
The single-row prediction loop is checked against actual RangerMotionModel::predict.
Outputs `trajectory.csv` with columns compute_seq,index,time_sec,x,y,yaw,predicted_vx,
predicted_wz,command_offset and `replay_status.json` with processed/skipped/reasons.
Rows describe the final pre-shift sequence's model prediction; command_offset is checked
against the actual returned command. They are not a proof the subsequent optimizer,
smoother or robot executed that entire prediction. No replayed velocity is published.
