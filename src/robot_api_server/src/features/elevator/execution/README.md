# Elevator execution

Owns the elevator-test transaction adapter, arm black-box HTTP client, bounded
task polling, execution runtime policy, recovery-action barrier, and ROS-facing
execution port.

During feature validation the gateway does not add arm readiness/health motion
gates. The module does not implement or modify the arm black box and does not
bypass the vehicle safety velocity chain.
