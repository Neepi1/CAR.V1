# Docking

- `DockingFeatureModule` is the one process-level owner of this complete
  feature family. Its core phase creates contact evidence, the canonical job
  store, correction-pause ownership, docking-manager runtime, and automatic
  pre-navigation undock before ordinary navigation is composed. Its explicit
  completion phase then attaches navigation, elevator, and localization-settle
  owners and creates predock control, job execution, pose resolution, HTTP,
  and status reconciliation before the API gateway starts. This resolves the
  construction dependency cycle without exposing individual docking Ports or
  submodule pointers in `robot_api_server_node.cpp`.
- `configuration/` owns the complete docking ROS-parameter graph as well as
  commissioning-time manual predock pose resolution and geometry sanity
  validation. `DockingConfigurationModule` declares the 100 existing
  docking/predock parameters, applies their unchanged dependent clamps, and
  emits ready-to-use configs for all docking submodules. Explicit pose IDs keep
  highest priority, followed by the established dock-specific ID/name
  conventions and the unique `dock_predock`-type fallback. Neighbor-domain
  values and map reads are injected; the aggregate duplicates neither
  parameter policy nor pose policy.
- `lifecycle/` owns the canonical docking job store and job/status data
  contracts, all App-facing docking HTTP transactions, `/docking/status`
  terminal/relocalization handling, coarse return-to-dock orchestration, and
  `DockingRuntimeModule`. The runtime module contains the manager process,
  Trigger clients, status/observation subscriptions, safe predock publishers,
  observation cache, and single execution worker; the feature aggregate
  connects neighboring-domain ports. `DockContactInterlockModule` owns the
  persistent dock-contact latch, BMS/runtime evidence classification,
  full-charge-idle policy, automatic contradiction clear, and all
  pre-navigation dock-check JSON. `PreNavigationUndockModule` consumes that
  immutable decision and owns the entire controlled-undock-before-Nav2
  transaction, including job serialization, service evidence, timeout, and
  optional post-undock localization readiness.
  `DockingCorrectionPauseModule` separately owns the `docking_fine` bridge
  correction-pause lifecycle, canonical job/display-pose projection, active
  owner protection, and stale-pause cleanup. `DockingJobExecutionModule` owns
  the complete concrete executor adapter, including the shared Nav2 action-send
  evidence and every cross-domain effect required by `DockingJobExecutor`; the
  feature aggregate implements that docking interface.
- `predock_alignment/` owns the pure pose policy plus the complete stateful
  bridge-freeze, yaw/lateral capture, fine-entry, and docking-manager handoff
  transaction. The lifecycle executor calls only its semantic interface; ROS
  resources remain injected aggregate ports.

Fine docking remains owned by `robot_docking_manager`; this API never publishes
directly to the chassis. Predock commands still enter `/cmd_vel_docking` and
the established `robot_safety` arbitration chain.
