# Floor switch

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
- `floor_runtime_interlock` reduces retained floor-manager and localization
  health observations into one fail-closed decision.
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
