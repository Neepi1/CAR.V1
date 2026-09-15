# Floor-transition failure termination

The bridge distinguishes a finished transaction from valid localization.

- `ABORT`: `transition_active=false`, `runtime_context_valid=false`,
  `failed_locked=false`, `recovery_required=false`, detail
  `ABORTED_CONTEXT_INVALID`. The failed target remains diagnostic evidence,
  not a claim that it became active.
- Invalid context does not accept ordinary correction/force-accept as a shortcut
  to readiness. Navigation goal safety remains false.
- A new exact transaction may BEGIN under its own FloorManager correction pause.
  BEGIN still leaves context invalid. COMMIT requires a newer explicit localization
  sequence and a valid, settled, published target transform.
- ABORT and pre-mutation cleanup permanently fence their transaction IDs against
  later BEGIN commands, including higher sequence numbers. Old ABORT/COMMIT
  commands cannot modify a replacement transaction. COMMIT replay requires the
  same complete identity, including transaction ID, not merely the same map.
- Pre-mutation cleanup can preserve proven source context; source seeding cannot
  turn an aborted, invalid context into a valid one.

## Ownership and outstanding operations

The bridge owns only its floor-context and transform acceptance state. It does not
observe whether map-server or localizer asset writes have finished. FloorManager
must settle or isolate outstanding old writes before a new switch is admitted;
otherwise an old response could race the new map. It must release only its own
pause/hold resources and distinguish cleanup completion from localization validity.
ABORT is not evidence that those external writes or leases have been cleaned up.

## Regression checks

`test_floor_transition_context` exercises the production context class. The
isolated ROS smoke `test/run_isolated_floor_transition_smoke.sh` additionally
checks the actual bridge service and typed health behavior with TF publication
disabled. Run it only in a separate ROS domain, preferably a `--network none`
container. Do not point the smoke at the live bridge.

Validation for this change: 15 production-context tests passed. The original
permanent-lock behavior, same-map old-COMMIT acceptance, and higher-sequence
post-cleanup BEGIN were each reproduced by a failing test before repair. The
Release candidate passed the real bridge ROS smoke in a network-disabled,
device-free container with domain 183 and a read-only production workspace.
Its original-object baseline relink was byte-identical to the installed bridge;
only the node and floor-context objects were replaced. Logs are under
`reports/bridge_abort_end_20260909/`. No deployment or robot restart was performed
by these checks.

Hardware acceptance remains required: fail one real switch, confirm its own
operation ends and resources are released, explicitly retry, verify no late old
asset write changes the replacement map, then confirm localization and TF before
allowing navigation. No robot motion is authorized by these isolated tests.
