# Navigation

- `configuration/` owns the complete navigation parameter declaration and
  construction graph. It returns the existing navigation, terminal-runtime,
  goal-execution, and goal-executor configs without owning runtime behavior.
- `runtime/` owns resident Nav2 handoff, bridge settling, and cancellation
  runtime state.
- `mission/` owns App goal requests, goal-job state, and commercial completion
  decisions, including the ordinary navigation execution state machine.
- `terminal_control/` owns bounded post-Nav2 correction through the existing
  safe command adapter.

`NavigationModule` owns goal admission, resident runtime start/stop, action and
cancel runtimes, cancel/stop job orchestration, and the complete
`/api/v1/navigation/*` HTTP route surface. `navigation_state` owns the stable
commercial state projection, including bridge-over-AMCL precedence,
transition-versus-recovery classification, dock occupancy, safety, settle,
goal, and cancel evidence. The composition root only adapts shared docking,
settle, safety, costmap, and safe-command evidence.

`NavigationFeatureModule` is the feature-level composition boundary. It owns
`NavigationModule`, `NavigationTerminalRuntimeModule`,
`NavigationGoalExecutionModule`, and `NavigationGoalExecutor` as one lifetime,
and owns their complete Ports graph. Cross-domain dependencies are supplied
once through `NavigationFeatureModuleDependencies`; the process entry point no
longer carries navigation's delayed callback cycle, bridge-status projection,
or pre-navigation undock request conversion.

Within `mission/`, `NavigationGoalExecutor` owns the ordered commercial state
machine and `NavigationGoalExecutionModule` owns its concrete ROS/action and
terminal-recovery transaction. This keeps Nav2 result evidence, bounded retry,
bridge settling, final pose/yaw verification, and predock command ownership in
one module instead of distributing them across the composition root.

State polling remains cache-only: it refreshes the already-owned runtime
process/context cache but performs no lifecycle probe, relocalization, motion,
or other new gate. The final velocity chain and TF ownership remain unchanged.
