# Elevator execution

The ROS port uses one bounded SingleThreadedExecutor worker, exact known-action
exception recovery, and stop/fault-aware internal waits. Original clients and
goals survive executor-exception recovery; that recovery does not retry goals. See
[audit, isolation tests and activation boundary](../../../../../../docs/elevator_adapter_executor_recovery.md).

Separately, the [navigation/floor failure recovery candidate](../../../../../../docs/elevator_navigation_floor_retry.md)
retries confirmed transient failures in the same effect. Unknown goals are not
duplicated; a committed floor is reverified without another switch. No arm code
or ordinary navigation parameters change. The jointly verified candidate was
activated on 2026-09-22; physical elevator recovery is not hardware-accepted.
The reviewed candidate accepts the original goal's success even when its result
arrives during timeout cleanup; it never treats terminal proof alone as retry
permission. Floor retry also needs the matching FloorManager/bridge cleanup
implementation. A cold startup owner's exact exit receipt ends an impossible
wait and preserves the original floor error; it does not restart that owner.

Owns the elevator-test transaction adapter, arm black-box HTTP client, bounded
task polling, execution runtime policy, recovery-action barrier, and ROS-facing
execution port.

The 2026-09-21 arm retry candidate keeps explicitly failed ready/button/release
tasks in the same effect until success or cancellation. Button success is retained
while release alone retries; transient GET failures poll the original task ID.
Each new physical attempt follows a one-second, cancellation-aware backoff.
Unknown POST acceptance or task timeout never authorizes another button POST.
Cancellation retains the existing single bounded release cleanup, not an endless
cleanup loop. No arm health/status gate or black-box change is added. This is
tested in isolation and activated on 2026-09-22, not hardware-accepted; see
[retry contract and verification](../../../../../../docs/elevator_arm_automatic_retry.md).

Retry waits also observe the adapter's existing local fault evidence. A fault
stops further attempts and returns its original code, with one bounded release
cleanup; it is not reported as user cancellation or a remote task terminal state.
No new worker, ROS communication or arm-status admission condition is introduced.

Cabin-panel approach selects `ElevatorCabinPanelFollowPath` and the sibling
`navigate_elevator_cabin_panel.xml` beside the configured direct-cabin tree.
Only that role receives the 0.20 m/s lateral cap; entry, return-center and
egress keep their original controller. Session BEGIN/END and BT selection
share the role policy. No arm behavior or arrival tolerance is changed.

During feature validation the gateway does not add arm readiness/health motion
gates. The module does not implement or modify the arm black box and does not
bypass the vehicle safety velocity chain.

Nonpersistent failure cleanup is independent of map localization readiness:
`check_elevator_cleanup_runtime_readiness` bypasses only the strict map-ready
probe for the existing source/target outside cleanup classification. This
classification is not itself physical proof that the robot is outside the
cabin. The policy does not supply hold,
stop, or resource-release proof. Owned action termination, mode/pause absence,
dual-odometry stop, unknown-side-effect checks and conditional hold release
remain in the ROS callers. Persistent recovery continues using the exact,
fresh localizer/bridge identity probe. The callable policy has red/green C++
tests; no physical-motion validation is implied. The 2026-09-21 deployment
uses a single adapter translation-unit rebuild and a relink of the verified
deployed API objects. See the deployment evidence under
`/tmp/njrh_reports/elevator_cleanup_deploy_20260921_t1OKhw`; replacing the
installed binary does not activate it in an already running process.

During this deployment the existing process-ownership guard instead classified
the still-running old executable as absent after replacement and automatically
restarted the full runtime. The new API then loaded the candidate. This was an
unintended deployment side effect, not an issued restart command; see the report
for the 15:18:08 UTC ownership fault and subsequent activation evidence.
