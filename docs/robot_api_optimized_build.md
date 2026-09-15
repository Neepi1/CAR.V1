# API optimized build — 2026-09-11

## Scope

An unspecified single-config `robot_api_server` build now defaults to
`RelWithDebInfo`. On the deployed GCC toolchain this selects `-O2 -g -DNDEBUG`.
Explicit build types and multi-config generators retain their existing choices.
No fast-math, LTO, new dependencies, caching, sampling reduction or changed
business source are introduced.

HTTP schemas, callback rates, map/localization semantics, navigation arrival
checks, elevator/docking behavior and velocity arbitration are unchanged. The
arm black box, CPU affinity, sensor transport and runtime startup are untouched.
This does not repair the separately recorded local-odometry delivery stall.

## Baseline and deployment boundary

The running API's original build flags were `-Wall -Wextra -Wpedantic`, without
an optimization flag or a selected build type. Its executable SHA-256 was:

`69028718894a941e651e25ca0c903770c79ad5b6cc91b6938433e2cecf730312`

Re-linking the preserved relocalization build with the two previously deployed
floor-interlock object files reproduced that exact executable hash. The
optimized candidate uses that deployed source snapshot plus those exact source
overlays, not the current dirty workspace's additional undeployed features.
The source manifest and build log are retained in
`/tmp/njrh_reports/api_optimize_20260911` on Jetson.

Local and Jetson workspace CMake files already had different test declarations.
Apply only this build-policy change on each side; do not replace unrelated test
declarations to make their complete hashes match. Matching scoped changes and
any intentionally different complete hashes must be recorded at deployment.

## Verification

- `test/test_default_build_type.py` exercises the package's actual CMake prelude
  using isolated temporary projects: unspecified, empty, explicit Debug,
  explicit Release, and multi-config selections. It failed before the fix and
  passed all five cases after it.
- A three-pass CPU-time microbenchmark compares identical deployed helper
  sources compiled with `-O0` and `-O2`. AMCL status-file parsing used about 61%
  less CPU time; JSON field lookup used about 13% less. All output checksums
  matched. These are helper results, not whole-API or whole-robot savings.
- Whole-API fixtures run in private network and IPC namespaces with a private
  ROS domain, temporary state, and the arm/elevator adapter disabled. They
  cannot publish into the vehicle's ROS network. Results must be labeled
  synthetic; a production comparison is a separate acceptance step.

Optimized compilation may expose pre-existing undefined behavior. A successful
build and benchmark alone do not prove functional equivalence. The deployment
record must include targeted contract tests and the candidate executable hash.

## Activation and remaining hardware acceptance

Compilation and installation do not imply that a running process has loaded
the candidate. This task has no authorization to restart or move the robot.
After explicit authorization, only the complete `njrh-runtime.service` restart
is allowed. Verify `/proc/<api-pid>/exe` against the installed candidate, then
compare API CPU usage under matched App/input load and check normal HTTP status.
Navigation, map switching, elevator and docking physical acceptance remain
user-operated tests; this task must not initiate them.

Current candidate build, test, synchronization and installation outcomes are
recorded in the report directory; live performance acceptance remains pending
until an authorized restart and measurement.
