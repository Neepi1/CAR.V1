# FAST-LIO2 four-thread matching on Jetson

The reused ARM build previously defined `MP_PROC_NUM=1` without `MP_EN`.
`-fopenmp` alone only supplied OpenMP support/timers; it did not activate the
guarded matching loop in `h_share_model`.

The user-authorized `fast_lio_openmp_threads.patch` changes only the reused
upstream CMake thread selection. `FASTLIO_MATCH_THREADS` defaults to 4 and
defines `MP_EN` when greater than 1. Setting 1 builds the existing serial path.
OpenMP must be available at build time. The existing per-point nearest-neighbor,
plane-fit and residual calculations are unchanged; residual collection, EKF,
IMU handling and map updates are not newly parallelized.

Copy the reused `ros2_ws/src/fast_lio` build inputs into an isolated writable
source directory, preserving its prior QoS and other patches. The container's
upstream source mount is read-only; do not remount or modify it. Apply the patch
with `git apply --check` then `git apply` in the copy. Configure a fresh build
directory with `-DFASTLIO_MATCH_THREADS=4 -DCMAKE_C_COMPILER=gcc
-DCMAKE_CXX_COMPILER=g++`, then build target `fastlio_mapping`. This avoids old
host-path CMake cache entries. Do not build the entire workspace or install
over a running executable in place.

Verification: inspect generated flags for exactly `-DMP_EN -DMP_PROC_NUM=4`,
inspect the binary for `GOMP_parallel`, and run the no-ROS synthetic matching
equivalence test against the actual reused loop and ikd-Tree. It checks one
versus four threads, repeatability and inherited CPU1-4 eligibility. It does
not subscribe to sensors or publish motion/TF. This does not replace a moving
mapping accuracy and scheduling/throughput field test.

Strip build-tree RPATH from a candidate copy using CMake's `file(RPATH_REMOVE)`
and verify that existing runtime libraries resolve (`ldd -r`). This avoids a
runtime dependency on temporary generated interface libraries. Only the
executable changes; the ROS message schema/libraries are unchanged.
After verification, back up the installed executable under `/tmp/njrh_reports`
and atomically replace its directory entry with the verified build. Active
processes retain their old executable; verify `/proc/PID/exe` separately from
the installed file. Do not restart an active mapping session automatically.
The next user-started mapping session loads the new binary on CPU1-4.

CPU2 is shared with IMU/EKF and CPU3 with JT128. Four threads do not imply four
times the speed or lower total CPU work. Other process affinities, RPS, point
density, QoS, DDS, timestamps, TF ownership and navigation remain unchanged.

Targeted tests on the Jetson:

```bash
python3 -m pytest src/robot_system_tests/test/test_fastlio_openmp_build.py -q
taskset -c 1-4 python3 src/robot_fastlio_mapping/test/verify_openmp_matching.py \
  --source /workspaces/isaac_ros-dev/ros2_ws/src/fast_lio \
  --output-dir /tmp/njrh_reports/fastlio_matching_check
```

Reports must use a fresh directory for each verification run. The standalone
matching test compiles against the installed Humble PCL 1.12 and Eigen headers;
it creates no ROS node and exits immediately after its bounded checks.
