# IMU arithmetic reuse

This optimization changes no ROS interface, parameter, frequency, timestamp,
TF authority, readiness rule, bias-learning sample, or missing-TF behavior.
It does not change the JT128 driver, pointcloud processing, DDS, EKF or arm.

## Scope

- The canonical IMU remap constructs its configured covariance overrides once.
  An enabled override skips the covariance rotation that was immediately
  overwritten; disabled overrides retain the original computation.
- The bias filter still queries its TF buffer on every input requiring a frame
  transform. `ImuRotationCache` only reuses arithmetic: normalization/matrix
  construction for the same quaternion and the rotated covariance for the same
  quaternion plus exact covariance input. Vectors are always recalculated.
- Quaternion changes invalidate both covariance entries immediately; either
  covariance changes independently invalidate its own entry. Keys compare
  representation bits, including signed zero, without an epsilon threshold.
  Nonfinite keys are recomputed, not retained as valid cache entries.

The helper does not decide whether TF exists or is fresh. It cannot bypass a
failed lookup, does not permanently assume a transform is static, and requires
no extra `/tf` subscription, timer or cache-expiry policy. The caller retains
the existing frame checks, lookup timeout, drop/passthrough option, bias update,
output generation and source-stamp handling. It remains owned by the existing
single-threaded callback path.

## Validation

`test_imu_rotation_cache` compares the production helper against frozen old
arithmetic across repeated/changing covariance and rotations, non-unit/zero/
nonfinite quaternions, NaN/infinite/unknown covariance, and small quaternion
changes. Finite results use a 16-epsilon scaled tolerance because changing
inlining on ARM can change final floating-point rounding even without caching;
finite/nonfinite class and zero sign are checked separately.

`isolated_imu_equivalence.py` runs old and candidate real ROS executables with
private topics, checks complete IMU and bias messages against each other and
an independent numeric oracle, and covers overrides, orientation states, TF
updates, missing/empty source frames after cache warmup, transform bypass,
bias learning/subtraction, duplicate/out-of-order/stale stamps, and recovery.
It requires an explicitly acknowledged network-none, device-free container.
Its widened freshness window and preserved test timestamps never affect field
configuration. A same-binary oracle smoke run is not an old/new differential.

`benchmark_imu_rotation` measures only arithmetic CPU time. Input/output compiler
barriers prevent constant folding and dropping unobserved result fields. It
does not initialize ROS or run a robot node and has no noisy timing threshold
in CTest. A function speedup is not an equivalent whole-process CPU saving.

Before activation, preserve the exact deployed sources/binaries and verify
source hashes, selected builds and isolated tests. Replace only the two IMU
executables, not the shared pointcloud library or sensor driver. The running
processes remain unchanged until a separately authorized whole-runtime restart.
Then compare matched stationary CPU windows and existing IMU/odom/TF rate/gap
evidence. Moving spin/arc and navigation accuracy still require explicit user
authorization and are not proved by arithmetic tests.
