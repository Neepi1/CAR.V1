# robot_global_localization

Wrapper for pointcloud-to-flatscan and Isaac Occupancy Grid Localizer responsibilities.

The runtime wrapper node is the compiled C++ executable `global_localization_node`.

Manual trigger confirmation uses the explicit Isaac sequence and settled TF,
not the all-source rejection counter. The legacy trigger service is preserved;
API clients use the tracked request/outcome interface. See
[protocol, limits and isolated tests](../../docs/relocalization_trigger_confirmation.md).

Isaac and its wrapper use CPU3 without moving the common localization/AMCL
pool off CPU6. See [CPU placement, field override and acceptance](docs/isaac_cpu_affinity.md).

The production cold-start coordinator now waits for the current launch's
post-GXF Isaac initialization event before its first trigger, while map loading
and sensor initialization overlap. The C++ trigger/reload algorithm is unchanged.
See [startup scope and acceptance](../../docs/isaac_parallel_startup.md).
The former Python script is kept only as historical reference during migration and is not installed or launched by the runtime path.

## Parameters

- `publish_tf`: defaults to `false`
- `pose_topic`: `/global_localization/pose`
- `health_topic`: `/global_localization/health`
- `default_floor_id`: mock floor asset binding
- `grid_search_trigger_service`: Isaac relocalization trigger service, defaults to `/trigger_grid_search_localization`
- `service_call_timeout_sec`, `result_wait_timeout_sec`, `bridge_accept_timeout_sec`, `map_to_odom_wait_timeout_sec`: staged `/global_localization/trigger` timeouts
- `localizer_input_freshness_enabled`, `localizer_input_topic`, `localizer_input_wait_timeout_sec`, `localizer_input_max_age_sec`, `localizer_input_min_fov_deg`: bounded pre-trigger admission for Isaac's localizer input. Each existing input wait defaults to at most 1 second and returns early on valid input. Both receipt age and original header age must be within 0.5 seconds; angular coverage must be at least 115 degrees. Startup's existing pre-dispatch retry budget is unchanged; no Isaac request is sent while input validation fails.
- `post_reload_minimum_settle_sec`, `post_reload_readiness_timeout_sec`, and
  `post_reload_required_service_ready_samples`: generation-fenced readiness
  barrier after a non-idempotent component replacement. Defaults are `1.0 s`,
  `8.0 s`, and three consecutive ready samples.
- After bridge force-accept is armed, the wrapper requires the existing two distinct, advancing, fresh `/flatscan` samples before recording its trigger baseline and calling Isaac. Failure returns `LOCALIZER_INPUT_NOT_FRESH dispatch_state=not_dispatched`, never a fall-through dispatch. A confirmed arm without any attempted Isaac call is a known not-dispatched outcome, not an unresolved computation; this does not claim that the bridge arm was cancelled. Unknown arm replies and attempted dispatches retain their existing UNKNOWN handling. See [fresh-input dispatch](docs/fresh_input_dispatch.md).
- `result_allowed_pretrigger_age_sec`: rejects stale `/localization_result` messages that arrive after a trigger but were stamped before the trigger window.
- `bridge_status_topic`: defaults to `/localization/bridge_status`
- `bridge_force_accept_service`: defaults to `/robot_localization_bridge/force_accept_next_localization`
- `require_grid_search_trigger`: fail `/global_localization/trigger` when Isaac trigger service is unavailable
- `floor_asset_root`: only canonical, regular, non-symlink floor assets below
  this absolute root may be applied; default
  `/workspaces/njrh-v3/workspace1/maps_release`
- `asset_state_topic`: typed transient-local `LocalizerAssetState`, default
  `/global_localization/asset_state`
- `runtime_map_context_file`: durable floor transaction context used only for
  restart bootstrap, default `/tmp/njrh_runtime_map_context.json`
- `runtime_context_bootstrap_retry_sec`: low-rate retry interval while the
  durable context, current release, or live component cannot yet prove one
  exact active identity; default `5.0`
- `localizer_component_container`, `localizer_component_full_name`,
  `localizer_component_package`, `localizer_component_plugin`,
  `localizer_component_node_name`, and
  `localizer_component_node_namespace`: the exact Isaac composition identity
  used for controlled replacement
- `localizer_component_remap_rules`: recreated component remaps. Node-name or
  namespace remaps are rejected because they would invalidate component
  identity verification.
- `localizer_component_timeout_sec`: bounded timeout for every composition and
  parameter service operation. A late or timed-out operation is never reported
  as success.
  The shipped YAML sets this to `15.0 s`; successful confirmation returns early.
  See [configuration scope and activation](../../docs/isaac_component_timeout.md).
- The production Isaac component uses `component_container_isolated` rather
  than `component_container_mt`. Humble stops and joins the component's
  dedicated executor before destruction. The launch owner respawns only the
  empty component manager after an abnormal process exit; the asset transaction
  remains the only owner allowed to load a localizer component.
- `floor_asset_max_yaml_bytes`, `floor_asset_max_png_bytes`: bounded asset file
  admission limits
- `local_flatscan_config`: reuses the validated `jt128_flatscan.yaml`
- `local_localizer_config`: reuses the validated Isaac occupancy grid localizer parameters from the car repo
- The deployed JT128 profile keeps `max_beam_error: 0.50`, matching the
  upstream Isaac default. This is a per-beam scan-match error cap, not a Nav2
  clearance setting. See `docs/isaac_relocalization_tuning.md`.
- Jetson verification found `isaac_ros_occupancy_grid_localizer` installed under `/home/nvidia/workspaces/isaac_ros-dev/install`
- Jetson verification found `PointCloudToFlatScanNode` inside `isaac_ros_pointcloud_utils`

## TF Contract

- Publishes global pose outputs, not `odom -> base_link`
- Leaves canonical `map -> odom` synthesis to `robot_localization_bridge`

## Runtime Contract

`/global_localization/apply_floor_assets` must be available before
`robot_floor_manager` applies a floor. The service is an internal floor
transaction step, not an operator or App shortcut. Its caller must already own
the floor transition and persistent motion hold. This wrapper does not acquire
the safety hold, stop Nav2, select a map, or resume navigation.

Isaac's `map_yaml_path` is consumed at component construction, so this wrapper
does not pretend that a parameter write hot-reloads the map. It performs this
bounded sequence:

1. validate the transaction identity, canonical digest syntax, paths, file
   sizes, PNG signature, and localizer YAML;
2. verify the component manager exposes list/load/unload plus the live
   localizer's list/get-parameter services;
3. require exactly one `/occupancy_grid_localizer`, then capture every declared
   parameter before any unload;
4. preserve captured tuning and override only `map_yaml_path`, `image`,
   `resolution`, `origin`, and `occupied_thresh` from the target localizer
   YAML;
5. unload and recreate the component through
   `/occupancy_grid_localizer_container/_container/*`, then verify the returned
   unique ID against a fresh component listing.

Unload is not decided from one graph sample. Within the same bounded operation
window the adapter correlates the unload response with repeated exact
`list_nodes` evidence and requires list/load/unload services to be live. A
successful response followed by short graph ambiguity may converge to success.
If the response is lost, confirmed absence is returned as a failed operation so
the reloader may restore the captured source component; it never authorizes the
requested target. Persistent absence ambiguity remains
`LOCALIZER_UNLOAD_NOT_CONFIRMED` and keeps the outer transaction held.

Target load failure returns failure even if the captured component is restored.
Rollback is attempted only after target absence is proven; ambiguous graph
state fails closed rather than risking a duplicate component. Every confirmed
component load, including rollback, increments `localizer_generation`. On a
wrapper-process restart, the generation is seeded only from a confirmed durable
context after the exact live-state bootstrap below succeeds. Repeating the exact
active `building/floor/map/epoch/digest` with identical canonical paths is
idempotent and does not increment the generation. Reusing that exact identity
with different paths is rejected.

At process startup, the wrapper does not infer the active map from a default
floor, a filename, or component presence alone. It publishes
`BOOTSTRAP_CHECKING`, then accepts restart identity only when all of these agree:

- the parameterized durable context is a regular non-symlink file with
  `state=ready`, `confirmed=true`, safe exact
  `building/floor/map/epoch/digest`, and positive persisted localizer generation
  and explicit-relocalization sequence;
- `<floor_asset_root>/<building>/<floor>/current/manifest.json` is an active v2
  manifest using the canonical SHA-256 bundle contract with that exact
  identity;
- the component manager reports exactly one configured Isaac component; and
- that component's typed `map_yaml_path`, `image`, `resolution`, `origin`, and
  `occupied_thresh` exactly match the parsed fixed-role current localizer asset.

Success publishes `BOOTSTRAP_READY` with
`active_identity_valid=true`, `localizer_ready=true`, and a nonzero generation.
Missing legacy fields, identity drift, an ambiguous/missing component, or any
live parameter mismatch publishes a `BOOTSTRAP_*` failure with both active flags
false and retries at the configured low rate. Bootstrap never unloads or loads a
component.

The synchronous `ApplyFloorAssets` response echoes the transaction and exact
requested identity. `/global_localization/asset_state` is reliable and
transient-local. The normal path correlates both. If the RPC outcome is delayed
or lost, `robot_floor_manager` may reconcile only a fresh terminal state that
echoes the same transaction and requested identity, proves the same active
identity, reports `reloaded=true`, advances the generation, and reports the
localizer ready. A floor-only status or an applying/stale/foreign state is never
accepted.

A component-manager `load_node` response proves construction, not that Isaac's
GXF graph and trigger service can already process a request. The wrapper keeps
one trigger client for its complete process lifetime; the client rediscovers
the same-named service after component replacement without mutating the
executor's callback group. After every real reload the wrapper arms a gate with
the new localizer generation and current `/flatscan` sequence. The following explicit
trigger waits for the minimum settle interval, a post-reload fresh input, and
three consecutive ready observations of
`/trigger_grid_search_localization`. If those conditions do not converge in the
bounded window it returns `LOCALIZER_POST_RELOAD_NOT_READY` before bridge
force-accept is armed. A later retry may complete the same pending gate; it
does not reload the component again.

The full ownership, failure, and field-validation contract is recorded in
`docs/isaac_floor_asset_reload.md`.

`/global_localization/trigger` proxies the request into Isaac's
`/trigger_grid_search_localization`; navigation resume must wait for the
wrapper service and the Isaac trigger service before calling floor switch.

Phase L1.1 makes the trigger service a staged success gate and the only owner of one explicit-localization transaction. It arms `robot_localization_bridge` exactly once, records an immutable arm time, calls Isaac's `std_srvs/Empty` grid-search trigger once, waits for a current-arm `/localization_result`, then requires bridge acceptance, `has_map_to_odom=true`, and a live `map -> odom` owned by `robot_localization_bridge`. A pre-arm result is drained without completing the result wait, changing the arm time, extending the deadline, or issuing another Isaac trigger. The wrapper also requires `correction_active=false`, equal current/target sequences, and either the completed Isaac target or a completed AMCL refine tied to the same explicit sequence. It does not require `safe_for_goal_start`: navigation admission belongs to the consuming operation, not localization completion. A successful response therefore means the triggered correction has reached the canonical TF tree, not merely that a candidate incremented an acceptance counter. Failure messages include `failure_code=` and `dispatch_state=not_dispatched|dispatched` so callers can retry only before dispatch. See [responsibility boundary and regression coverage](../../docs/relocalization_completion_responsibility.md).

Phase A2 keeps Isaac as triggered global relocalization only. Runtime continuous localization candidates come from AMCL on `/scan`; no runtime path forwards `/flatscan` into Isaac's trigger input for background updates.

Hardware validation still needs a cold navigation start while recording `/global_localization/health`, `/localization/bridge_status`, `/localization_result`, and `/tf` to confirm the wrapper reaches bridge acceptance without false timeouts. Startup readiness should use bridge acceptance plus `map -> odom`; `/localization/health` is not used as a navigation startup probe. Before calling Isaac's trigger, the wrapper requires a fresh localizer input sample. If the bridge ignores a result stamped before force-accept was armed, the wrapper keeps the same immutable arm and waits for the current request's result. An explicitly armed result is evaluated at its original timestamp and is not rejected solely because delivery exceeded `triggered_max_result_age_ms`; missing historical odom TF, stale latest odom, wrong ownership, or an expired transaction remain terminal. Callers do not re-trigger after any dispatched failure.

Real floor-switch validation still requires one supervised, stationary
transaction with the floor-manager hold active: confirm the old component ID,
new ID, exact target YAML/PNG, generation increment, fresh localization result,
and bridge-approved `map -> odom`. Then inject a bad target in a non-production
test asset and prove rollback restores the old parameters while the floor
transaction remains failed and motion stays held. Do not test this by directly
calling the service during navigation.
