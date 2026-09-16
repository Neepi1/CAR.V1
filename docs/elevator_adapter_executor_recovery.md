# Elevator adapter executor recovery (2026-09-16)

Scope: only `ElevatorRosRuntimePort` ROS execution, internal failure propagation,
and teardown. No elevator FSM, navigation policy, arm protocol, HTTP protocol,
robot parameters, supervisor, or installed ROS/DDS dependency changes.

## Evidence and pre-change audit

The 2026-09-16 15:17:52 UTC API core recorded
`std::runtime_error: Taking data from action client but no ready event`.
The adapter's two-thread `MultiThreadedExecutor::spin()` unwound across a
joinable internal worker, calling `std::terminate` before the adapter's catch.
An idle elevator Action client still receives shared navigation status traffic;
an elevator test does not have to be running to expose this defect.

Audited current adapter: no additional callback groups, no Reentrant groups,
no callback-side future waits, no business-side `spin_until_future_complete`,
and only one adapter caller of the spin API. Subscriptions and floor feedback
update protected state and notify. Business waits remain outside ROS callbacks.
Evidence/floor condition-variable waits release their callback-required locks;
service waits hold no lock needed to deliver the corresponding ROS response.
Goal, binding, submission and lease locks are retained.

Installed rclcpp is 16.0.15 with the previously identified waitable-queue
backport. The installed Executor header exposes `spin_once(nanoseconds)`;
the actual `librclcpp.so` disassembly at `0xc0650` confirms spinning is cleared
on both return (`0xc0694`) and unwind (`0xc06d8`). Tests below exercise the
installed binary, not an assumed upstream-only build.

## Implementation

- One dedicated worker calls `SingleThreadedExecutor::spin_once(50ms)`.
  Fifty milliseconds is a maximum idle wait, not a callback period or sleep.
- Independent stop flag plus the node's own Context are checked before every
  turn and before recovery. `cancel()` wakes the executor; a cancel-before-spin
  race is bounded by the independent flag and the next idle wait.
- Only a `std::runtime_error` with the exact complete known text is retried.
  No node/client/executor recreation, goal resend, cancel, future reset or
  business stage transition occurs in this recovery path.
- Known errors have total/consecutive counters. Logs are first/every 64th;
  after eight consecutive throws, a stop-interruptible 10ms backoff prevents
  an exception storm from busy-spinning. Ordinary turns and isolated errors
  have no added sleep. A normal/idle turn is never labelled Action progress.
- Unknown exceptions retain their original `exception_ptr`, log adapter failure,
  terminate this worker loop, and notify dependent waits. The existing
  `ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY` result carries local inability to observe
  the remote outcome; it does not fabricate an Action terminal state.
- Future waits wake after executor turns/fault/stop. Endpoint waits are sliced
  within their original steady-clock budget. Existing business deadlines and
  normal timeout/cancellation policies are not extended or replaced.
- The owning execution module already requests its ordinary cancellation and
  joins its business worker before releasing its shared port. Adapter destruction
  declares stop and notifies first, joins the remaining keepalive and ROS worker
  without callback locks, then removes the node. No detach or shared-context
  shutdown is used. Worker-side stop requests defer joining to the owner.
- The worker is declared last so it is destroyed first, including constructor
  unwinding. Its repeated stop notifications cannot outlive the condition
  variables, clients or executor. `test_elevator_executor_lifetime.py` locks
  this production-member and shutdown ordering in addition to runtime tests.
- Controller-session scope cleanup uses the same internal failure conversion;
  transport failure during unwinding cannot throw a second transport exception.
  Generic business cleanup exceptions have not been globally swallowed.

## Test layers and interpretation

1. `test_elevator_ros_executor`: lifecycle, early/idle/concurrent/repeated stop,
   exact matching versus near-match text, unknown standard/nonstandard errors,
   waiter/logger exceptions, storm throttling, worker-side stop and unchanged
   incomplete futures. Spin substitutes exist only in the test translation unit.
2. `test_elevator_ros_events`: real rclcpp clients and mock NavigateToPose,
   FloorSwitch and Trigger servers. Test-only client subclasses inject at
   `take_data`, before consuming an event. Tests assert unchanged client/UUID,
   continued goal response/feedback/status/result/service processing, exactly
   one goal, zero recovery cancels (one explicitly requested cancel in cancel
   tests), one completion notification, unknown-error wakeup and pending-result
   shutdown. These are ROS transport integration tests, not physical elevator
   runs or a full API/arm/hardware end-to-end test.
3. Original installed-library synthetic-ready queue reproduction is retained
   under the diagnostic report. Control exits 0; repeated-ready reproductions
   exit 42; the wrapper validates 42 and returns 0. This is not a real server
   interaction test and does not mean the system library was repaired.

Existing execution-module/mock-port regressions separately check business order,
floor transition, cancellation and blocked-effect shutdown. The isolated runner
uses private network, PID, IPC, mount and `/dev/shm` namespaces, a separate DDS
domain and no production task requests. It does not install packages or use
alternate ROS libraries.

Build the small transport test project from
`src/robot_api_server/test/features/elevator/execution/executor_isolation` with
the normal Humble/workspace environment. Run `run_isolated.sh BUILD_DIR` as a
container user allowed to create namespaces. `NJRH_EXECUTOR_TEST_REPEAT=10`
repeats both transport/lifecycle suites. The tests are also normal package
GTest targets.

## Build and activation boundary

Report: `/tmp/njrh_reports/elevator_executor_20260916T1630_IOUxjE`.
The coherent deployed source/build graph is reused incrementally, with its old
source, executable and manifests preserved in the report first. Only the
adapter production translation unit changes; no unrelated objects are imported.
Local and deployed sources already differ in an unrelated cleanup-readiness
policy block and CMake tests. Only this executor patch is synchronized; those
pre-existing differences are not silently deployed or overwritten.

No physical movement, arm calls or service restart is part of these tests.
Activation and real robot validation require a separately authorized whole
`njrh-runtime.service` restart. Do not call simulated results hardware acceptance.
