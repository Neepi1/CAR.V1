# Navigation mission

After a failed terminal correction, a later successful same-goal Nav2 retry
triggers current strict XY/lateral/yaw, bridge, fresh-pose and existing stop
evidence verification. The historical result remains failed for diagnostics;
the correction is not re-entered, and retry policy/budgets are unchanged.
See [repair boundaries and tests](../../../../../../docs/navigation_terminal_revalidation.md).

The persistent-recovery candidate keeps the original job while Nav2 reports
goal-correlated `waiting`/`recovering` phases. The ordinary result budget excludes
only those fresh intervals; the final-yaw-drift reposition deadline and final
pose criteria are unchanged. This is staged code, not physical acceptance.

Owns the App-visible navigation goal model, goal HTTP payloads, completion
policy selection, position-only yaw selection, final-pose evaluation, bounded
retry classification, acceptance-slack policy, and the complete ordinary-goal
execution state machine.

`NavigationGoalExecutor` now owns the ordered goal-start readiness/undock
sequence, Nav2 terminal wait, commercial final verification, bounded retry,
terminal correction selection, and final job outcome. ROS-side observations,
actual Nav2 submission, and safe velocity publication remain explicit ports;
the executor cannot bypass the existing command chain.

`NavigationGoalExecutionModule` is the concrete execution edge behind that
port. It owns initial and retry action submission evidence, near-goal stalled
handoff, bridge-settle waits, final-pose bookkeeping, deterministic terminal
correction, final-yaw correction-pause lifetime, and mutual exclusion with
predock yaw/lateral command ownership. Cross-domain floor, undock,
localization, safety, dock-contact, and runtime-state operations are injected;
the composition root retains only wiring and neighboring-interface adapters.
