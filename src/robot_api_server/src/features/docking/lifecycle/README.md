# Docking lifecycle

Owns the App-visible docking job model, exact JSON serialization, parsing of
leading docking/undocking status codes into lifecycle outcomes, and the
complete return-to-dock execution state machine.

`DockingJobStore` is the one canonical owner of the mutable `DockingJob`, its
mutex, and the monotonically increasing job sequence. HTTP admission, the
executor, and `/docking/status` all reference this store rather than retaining
copied or parallel state. It also owns terminal job commits, exact state JSON,
dock-latch/runtime completion, and correction-pause release after leaving the
job lock. This is an ownership extraction only: phase names, thresholds,
failure codes, and required-versus-warning decisions are unchanged.

`DockingCorrectionPauseModule` owns the complete `docking_fine` global
correction-pause lifecycle. It applies/releases the bridge request, projects the
result and frozen display pose into the canonical job, refuses to clear a pause
while a running fine-docking phase still owns it, and cleans only stale job or
bridge evidence. The composition root injects bridge/pose/logging ports; the
phase set, reason strings, timeout, and success/failure behavior are unchanged.

`DockingHttpModule` owns the complete App-facing docking HTTP surface:
`state`, `start`, `undock`, `confirm_docked`, `clear_docked_latch`, and
`cancel/stop`. It preserves the existing map-context admission, elevator
motion-admission fence, persistent dock latch, controlled-undock evidence, and
response schema. During the lifecycle migration it shares the single existing
`DockingJobStore` with the status callback and executor; it does not introduce
a second task state.

`DockingStatusModule` owns the complete `/docking/status` transaction. It
classifies terminal fine-docking and undock states, commits the shared job and
runtime-mode state, queues correction-pause release outside the ROS callback,
and owns the post-fine/post-undock relocalization worker. Localization trigger,
settle proof, persistent dock-latch updates, and deferred execution remain
explicit composition ports. This extraction preserves the existing required
versus warning behavior and does not add a motion or localization gate.

`DockContactInterlockModule` is the single owner of persistent dock-contact
memory and the complete evidence-to-occupancy decision. It reads and atomically
updates `docking_contact_latch.json`, assigns source strength, accepts stable
BMS charging-session evidence, preserves strong evidence during full-charge
idle, applies the existing confirmed-undock contradiction clear, and renders
the exact App diagnostics. The composition root supplies immutable runtime,
BMS, and navigation-job observations only. Controlled undock execution remains
a separate lifecycle transaction; this ownership move changes no interlock,
threshold, latch-clear rule, or motion path.

`PreNavigationUndockModule` owns the complete synchronous admission transaction
between dock classification and Nav2 goal submission. It preserves the one
start mutex and canonical `DockingJobStore`, clears stale teleop motion, releases
only stale fine-docking correction pause, reuses an already-running undock,
records the exact Trigger-service evidence, and waits for proven physical
undock plus configured post-undock localization readiness. A failed or
unproven readiness result still prevents the pending Nav2 goal from being sent.
The composition root only maps the immutable dock-check fields and injects
neighboring effects; no timeout, retry, phase name, or gate was added.

Fixed-distance automatic undock accepts a missing `dock_id`; a known identity
is diagnostic metadata, not an execution prerequisite. The manager uses its
existing distance/speed and odometry feedback. Status cleanup and post-undock
localization work with an empty identity; only the separate no-motion,
dock-zone-based interlock reconciliation still requires a resolved dock ID.
See [scope and validation](../../../../../../docs/pre_navigation_dock_interlock_recovery.md#fixed-distance-undock-identity-correction-2026-09-16).

`DockingRuntimeModule` owns the process/ROS/worker edge used by all three
lifecycle collaborators. It supervises the optional docking-manager child,
owns the `/docking/start|stop|undock` Trigger clients, consumes
`/docking/status`, selects exactly one configured target-observation or GS2
freshness subscription, publishes predock commands only on
`/cmd_vel_docking`, publishes the existing Ranger forced-mode request, and
joins the one docking execution worker. Target observations remain usable only
when their configured source matches and both `sensor_healthy` and `valid` are
true; GS2 messages remain freshness evidence only. Service timeouts retain the
existing delayed-side-effect evidence and charging-state retry behavior. This
ownership move adds no stage, threshold, retry, BMS interlock, or motion gate.

`DockingJobExecutor` orders runtime readiness, pre-dock Nav2, BMS contact stop,
optional localization barriers, deferred yaw/lateral staging, bridge freeze,
fine-entry verification, and the final `/docking/start` handoff. ROS actions,
safe-command publication, TF/bridge observations, and the actual fine-docking
controller are explicit composition ports. The executor cannot publish to the
chassis or bypass `robot_safety`.

`DockingJobExecutionModule` is the complete concrete implementation of that
executor port. It binds the one `DockingJobStore`, serializes pre-dock Nav2
submission on the shared action mutex, retains unresolved side-effect evidence
after a timed-out submission, and forwards the existing localization, terminal
control, BMS, teleop-zero and runtime-state effects. The composition root no
longer inherits `DockingJobExecutionPort`; it only supplies explicit neighboring
ports and passes this module to `DockingJobExecutor`. This ownership move changes
no phase, timeout, retry, tolerance, command topic, or safety gate.
