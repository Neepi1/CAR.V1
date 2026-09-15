# Elevator execution

Owns the elevator-test transaction adapter, arm black-box HTTP client, bounded
task polling, execution runtime policy, recovery-action barrier, and ROS-facing
execution port.

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
probe after an outside disposition is established. It does not supply hold,
stop, or resource-release proof. Owned action termination, mode/pause absence,
dual-odometry stop, unknown-side-effect checks and conditional hold release
remain in the ROS callers. Persistent recovery continues using the exact,
fresh localizer/bridge identity probe. The callable policy has red/green C++
tests; no production deployment or physical-motion validation is implied.
