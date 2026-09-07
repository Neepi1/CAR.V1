# Navigation mission

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
