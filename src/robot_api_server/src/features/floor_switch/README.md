# Floor switch

Floor failures are task history, not permanent API locks. Each typed observation
replaces that source's previous evidence. Current activity still rejects every
conflicting operation; current invalid localization still rejects navigation
goals and live-pose capture. Exact source-independent mapping, localization,
navigation-service startup and floor-selection operations can enter their own
recovery paths without claiming that localization is ready. Entry, worker and
commit checks use the same `decision_for_operation` policy. The current-state
change is local/tested, not deployed. See
[current-state admission](../../../../../docs/floor_failure_current_state_admission.md).
The earlier, narrower offline-only deployment remains documented separately in
[failure/asset admission](../../../../../docs/floor_failure_asset_admission.md).

Source readiness is not a prerequisite for manual target loading. The runtime
context is used only for the exact already-active optimization. Unknown, failed,
starting or unlocalized sources proceed to the existing target transaction;
invalid source health alone does not block that request. This does not change
navigation lifecycle or the target completion contract. The real-API isolated
regression is `test/features/floor_switch/floor_switch_unready_http_smoke.py`.

Owns the complete API-side floor-switch vertical slice:

- `floor_switch_feature_module` is the aggregate owner of configuration
  projections, the complete `FloorSwitchModule`, and all neighboring runtime
  observation/admission wiring. Late providers preserve the existing startup
  order and are fully bound before HTTP starts.
- `floor_switch_module` owns the four HTTP routes, the strict live Action
  client, the legacy offline-only service client, transaction worker and
  cancellation lifetime, exact-map source verification, floor/localization
  health subscriptions, and the negative runtime interlock.
- `floor_switch_configuration_module` uniquely declares all 6 API-side floor
  status/switch parameters, preserves the 15-to-300-second live Action bound,
  and combines maps, localization, and shared timeout inputs into the complete
  module config. It performs no switch or runtime side effect.
- `floor_switch_http_transaction` owns the thread-safe App-visible live
  transaction state.
- `floor_runtime_interlock` reduces the latest floor-manager and localization
  health observations into an operation-specific current-state decision.
- `runtime_map_context_io` owns the persisted context format shared with
  navigation/localization startup.
- `floor_switch_handoff_tracker` retains the elevator post-switch handoff
  observation policy.

`robot_api_server_node.cpp` constructs the aggregate and no longer owns the
floor-switch port graph, HTTP handlers, ROS clients/subscriptions, transaction
state, or worker threads.

The authoritative atomic floor-switch Action remains in `robot_floor_manager`.
This extraction adds no gate and changes no timeout, endpoint, asset proof,
TF, DDS, localization, Nav2, or velocity-chain behavior. This directory does
not publish TF or use relocalization to mask odometry.
