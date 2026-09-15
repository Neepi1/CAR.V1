# IMU ROS communication and scheduling

`robot_bringup/imu_pipeline_node` hosts two **separate modules** in one ROS
context: the `robot_hesai_jt128` canonical IMU remap and the `robot_local_state`
gyro-bias filter. Each keeps a single-threaded executor. A filter TF lookup
cannot stall the remap executor and therefore cannot delay raw IMU delivery to
mapping simply by occupying that executor.

Only the remap publisher and filter IMU subscription enable rclcpp intra-process
communication. The remap publishes an owned message; all its values are still
computed by the existing implementation. Vendor input, TF/static TF, odometry,
commands, corrected IMU and bias endpoints retain DDS. External `/lidar_imu`
readers still receive normal DDS messages; this is not a global SHM, DDS-version,
pointcloud, or zero-copy-all-topics change. External readers reduce the potential
saving because the public raw IMU still needs serialization for those readers.

The existing raw rate, public names, frames, source stamps, QoS depths/reliability,
bias updates on every accepted raw sample, 100 Hz latest-sample output, 10 Hz bias
output, freshness policy and TF error behavior are unchanged. No safety gates,
navigation parameters, pointcloud processing, EKF or arm logic are added/changed.

## Ownership

- `run_driver.sh` owns the combined IMU process and stops it with sensor ingress.
- In combined mode, `run_local_state.sh` checks the existing filter output
  readiness but does not start or stop the filter. Stopping/restarting just EKF
  therefore does not reset bias learning or interrupt canonical `/lidar_imu`.
  Reloading filter parameters now requires restarting its **driver-owned** host
  (on this robot, only the user-authorized whole-runtime restart is permitted).
- `NJRH_IMU_PIPELINE_MODE=intra_process` is the requested default. The shared
  helper chooses it only for EKF mode, enabled bias filtering, and identical
  nonempty steady CPU masks for remap and filter. This preserves the current
  five-core allocation; profiles with different masks keep standalone nodes.
  Startup CPU boosting and subsequent restoration use the existing affinity
  mechanism, with no new placement policy.
- Explicit `NJRH_IMU_PIPELINE_MODE=standalone`, non-EKF modes and disabled bias
  filtering retain standalone/remap-only behavior. Existing standalone executable
  names and direct launches remain supported.
- Ingress reuse must match the requested topology. Full-runtime cleanup includes
  the new host, while local-state-only cleanup does not. A future driver topology
  switch removes the exact old standalone filter before starting the host.
- Both modules now share a process failure domain. This is an explicit tradeoff;
  no new automatic restart/watchdog policy is introduced here.

## Verification and deployment

`src/robot_system_tests/test/test_imu_pipeline_runtime.py` tests selection,
CPU-profile compatibility, ownership and process patterns without ROS/hardware.
`src/robot_bringup/test/isolated_imu_pipeline.py` runs real Humble nodes in a
device-free network-none container, compares old standalone and combined output,
and tests canonical-reader joins/leaves, TF changes, bias learning, rejected
stamps, raw callback counts (not just deduplicated output), remap-only mode and
SIGINT/SIGTERM teardown. Its relaxed timestamps/freshness apply to fixtures only.

The staged build uses actual package CMake targets but compiles only IMU targets
and relevant tests. Unchanged installed dependencies/pointcloud artifacts are
reused. The IMU host links only the two small IMU libraries, not pointcloud code.
Reports and A/B runners are under `/tmp/njrh_reports/imu_ipc_20260913`.

Before calling the optimization live, verify hashes and perform a separately
authorized full-runtime restart. Then verify unique publishers, current raw and
corrected rates, existing TF/odom evidence and matched stationary CPU windows.
Real mapping transitions, moving spin/arc accuracy and whole-navigation timing
are hardware acceptance work, not proved by isolated message equivalence.
