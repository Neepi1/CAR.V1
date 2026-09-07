# Elevator non-persistent recovery policy

Date: 2026-08-21

## Decision

The deployed Ranger Mini 3 elevator commissioning flow does not use a
persistent operator-cleared recovery lock.

An elevator failure still has two responsibilities:

1. Stop/cancel the transaction-owned actions and prevent further effects from
   that failed transaction.
2. Reconcile and release its mode lease, localization/floor handoff resources,
   and owner-scoped safety hold. Recovery also proves that no legacy execution
   session remains, but new elevator-test transactions do not create one.

Those responsibilities run automatically. Failure of a cleanup observation no
longer changes the transaction to `LOCKED` after three attempts. The worker
backs off briefly and retries. Restart recovery and final hold release follow
the same automatic retry rule.

A complete runtime restart may intentionally come up on a confirmed map that
is unrelated to the failed transaction's frozen source and target assets. In
non-persistent restart recovery only, that fresh runtime may supersede the
stale transaction after the localizer, localization bridge, TF, Nav2 action
endpoints, absence of transaction-owned mode/correction resources, and dual
odometry stop are proven. Cleanup then conditionally releases only the stale
transaction's owner hold and terminates it as failed; it does not switch back
to the old map. Ordinary in-process failure cleanup still requires the exact
frozen outside-floor identity.

## API admission

In production, historical `LOCKED`, unknown terminal hold evidence, and an
unreadable recovery marker are not converted into the global
`ELEVATOR_EXECUTION_RECOVERY_REQUIRED` admission result. A historical journal
that can be parsed is converted to
`FAILURE_CLEANUP / AUTOMATIC_RESTART_RECOVERY` on the next complete runtime
start.

An ordinary active elevator task is still mutually exclusive with another
elevator task. Automatic cleanup is part of the failed transaction and is not
reported as successful task completion, but it is excluded from the global
state-changing API interlock. A new elevator transaction can still be rejected
until the old transaction's owned resources have actually reconciled.

## Runtime fence

The motion-admission fence is transient. It closes while `prepare` checks the
live runtime. When `prepare` rejects before a new transaction effect, the fence
reopens as the scope exits; it is not retained as an operator-cleared latch.
Failure cleanup and restart reconciliation use the same scoped rule, so an
offline Nav2 endpoint cannot leave ordinary API admission fenced indefinitely.

## Unchanged safety ownership

This decision does not bypass or remove:

- Emergency stop.
- Command watchdog and stale-command rejection.
- An actually active transaction's owner hold and exact operating-mode
  ownership. The legacy execution-lease arbiter remains available to other
  clients and for stale-resource recovery proof.
- `robot_safety` velocity limits, mode permission, localization gates, reverse
  and lateral permission.
- `Nav2 -> velocity_smoother -> collision_monitor -> robot_safety -> /cmd_vel`
  for ordinary navigation, including the already documented scoped cabin
  exception.
- Single-owner TF and floor-switch atomicity rules.

The App no longer needs to expose an elevator unlock workflow. It should show
the failed task and automatic cleanup state, then refresh until a terminal
`FAILED` or `CANCELLED` state is reported.

## Verification

The regression contract covers:

- Retrying failed cleanup past the historical three-attempt limit without
  producing `LOCKED`.
- Ignoring retained terminal recovery fields in the production API interlock.
- Keeping the legacy strict policy available only for compatibility tests.
- Loading a historical retained journal into automatic restart cleanup.
- Retiring a stale cross-building transaction when a different complete
  runtime is confirmed ready, while keeping ordinary live cleanup exact.
