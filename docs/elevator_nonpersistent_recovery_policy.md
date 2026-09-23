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

Non-persistent cleanup must not depend on a navigable map. For the existing
source/target outside cleanup classification, the cleanup-only policy skips
the strict runtime-map readiness probe, including during restart cleanup.
The classification represents the existing cleanup context, not independent
physical proof that the robot is outside the cabin. Owned action termination,
mode/correction resource absence, fresh dual-odometry stop, unknown-side-effect
checks and conditional owner-hold release remain independently required.
Cleanup does not switch back to an old map and does not mark a failed task as
successful. Persistent recovery retains the original exact map identity and
fresh localization readiness checks.

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
- Non-persistent source/target cleanup does not call the strict readiness
  probe and does not invent stopping, resource, or hold evidence.
- Persistent cleanup preserves the strict probe's original failure result;
  an unresolved retain-lock classification is not accepted by the policy.

## 2026-09-21 deployment boundary

Only the elevator ROS adapter translation unit is rebuilt for this change;
other API objects are reused after reproducing the deployed binary hash.
Tests and candidate/deployment manifests are recorded in
`/tmp/njrh_reports/elevator_cleanup_deploy_20260921_t1OKhw`.
Deployment replaces the on-disk candidate only. A separately authorized full
runtime restart is required for the running API to load it. Isolated regression
tests are not physical elevator acceptance, and no motion is performed here.

Deployment outcome: at 15:18:08 UTC, replacing the executable caused the existing
ownership checker to report `supervisor=1, node=0` while the old process was
still alive. The supervisor exited and systemd automatically restarted the
whole runtime at 15:18:42 UTC. No restart command was issued by this deployment.
The new API process loaded the candidate; HTTP status subsequently reported
healthy navigation. The ownership checker itself was not changed. See the
deployment report for this unintended side effect and the remaining hardware
acceptance boundary.
