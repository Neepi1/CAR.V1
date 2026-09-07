# Runtime mode

`RuntimeModeCoordinator` owns the in-process business state shared by mapping,
navigation, and docking HTTP workflows. It provides one coherent snapshot for
`/api/v1/status` and `/api/v1/navigation/state`, plus the single-owner mode
transition token used by asynchronous mapping start/stop orchestration.

Owned invariants:

- snapshot priority is `ERROR > DOCKING > MAPPING_2D > NAVIGATION > IDLE`;
- activating mapping clears active navigation and docking state;
- activating navigation or docking clears active mapping state;
- docking admission updates active state, dock identity, status, health, and
  mapping exclusion under one lock;
- docking completion updates the terminal state atomically and stops
  navigation only for `docked` or `charging` completion;
- a mode transition has at most one owner, and only that owner may release it;
- runtime profiles remain `normal`, `recovery`, or `docking_fine` using the
  existing state-name contract.

The coordinator does not start or stop processes, call ROS services, publish
velocity, inspect the ROS graph, or add admission gates. Those side effects
remain in the composition root and feature-specific modules. Unit tests live
under `test/application/runtime_mode/` and lock the existing state semantics.
