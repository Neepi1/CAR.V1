# robot_mode_manager

`robot_mode_manager` owns the mission operating-mode control plane. It does not
publish velocity commands, publish TF, change Nav2 parameters, or select Ranger
chassis motion modes.

## Interface

- Command service: `/robot_mode/set_mode` (`robot_interfaces/srv/SetMode`)
- Authoritative state: `/robot_mode/state`
  (`robot_interfaces/msg/OperatingModeState`)
- State QoS: reliable, transient-local, keep-last 1, plus periodic heartbeat

`NORMAL` is always unowned. Every non-`NORMAL` mode requires a non-empty
`owner`, `mission_id`, and `lease_id`, with a bounded steady-clock TTL. The
exact lease may renew, change mode, or release. Other leases are rejected.
Only the configured recovery owner may preempt an active lease by requesting
`RECOVERY`. Released, expired, or preempted lease IDs cannot be reused.

Supported operating modes:

- `NORMAL`
- `RAMP`
- `ELEVATOR_WAIT`
- `ELEVATOR_RIDE`
- `DOORWAY`
- `RECOVERY`

These names describe mission context. They are deliberately separate from
Ranger Mini 3 chassis modes such as `SPINNING`, `DUAL_ACKERMAN`, and
`PARALLEL`.

## Fail-safe boundary

Lease expiry returns this control-plane state to `NORMAL`; it is not a motion
permission. Elevator ride and doorway motion still require a separate,
owner-scoped hold/execution lease enforced by `robot_safety`. Until that
interlock is integrated and tested, this package must not be used to authorize
real elevator movement.

The node uses `std::chrono::steady_clock` for TTL decisions so ROS time pauses
or jumps cannot extend a lease. Restart always begins in unowned `NORMAL`.

## Validation

The pure C++ tests cover startup, acquisition, invalid requests, conflicts,
renewal and mode change, exact release, one-shot expiry, stale-lease rejection,
and configured recovery preemption. A service-level smoke test should run in
an isolated `ROS_DOMAIN_ID`; it must acquire a short lease and observe the
latched state return to `NORMAL` after expiry.

No physical hardware validation is required for the arbiter itself. Before
deployment, verify one service server and one state publisher, consumer
heartbeat timeout behavior, and the separate `robot_safety` execution
interlock. Do not add this node to the production runtime until those consumers
exist.
