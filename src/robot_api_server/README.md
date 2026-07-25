# robot_api_server

`robot_api_server` is the production-facing HTTP gateway for Android and other non-ROS clients.

It does not own mapping, localization, navigation, or chassis control logic. It only exposes a narrow HTTP API and forwards requests into existing ROS 2 topics and services.

## Code layout

- `src/robot_api_server_node.cpp` owns ROS wiring, HTTP routing, runtime process control, and API state.
- `include/robot_api_server/bms_contact.hpp` plus `src/bms_contact.cpp` own the BMS charging-contact policy used by teleop guard, status reporting, and undock admission.
- `include/robot_api_server/api_time_utils.hpp` plus `src/api_time_utils.cpp` own UTC timestamp formatting and generated map/current-pose IDs.
- `include/robot_api_server/docking_job_model.hpp` plus `src/docking_job_model.cpp` own the docking job data contract and JSON state payload.
- `include/robot_api_server/docking_status_utils.hpp` plus `src/docking_status_utils.cpp` own docking/undocking status string classification used by docking state transitions.
- `include/robot_api_server/elevator_configuration_module.hpp` plus
  `src/elevator_configuration_module.cpp` own elevator commissioning drafts,
  validation, immutable building releases, optimistic revisions, rollback,
  and the private elevator-waypoint asset. The module has no ROS, Nav2,
  FloorSwitch, safety, TF, or Twist port.
- `include/robot_api_server/file_utils.hpp` plus `src/file_utils.cpp` own common text/binary file reads and writes, PGM output, and map YAML image-file rewrites.
- `include/robot_api_server/floor_asset_resolver.hpp` plus `src/floor_asset_resolver.cpp` own floor asset completeness checks, active `current/` selection, `poses.yaml` fallback, and stored pose lookup for navigation/docking.
- `include/robot_api_server/floor_runtime_interlock.hpp` plus
  `src/floor_runtime_interlock.cpp` own the negative-only typed floor
  transaction admission rule. They do not prove positive readiness.
- `include/robot_api_server/http_common.hpp` plus `src/http_common.cpp` own HTTP request/response structs, WebSocket accept-key helpers, and the lightweight JSON helpers used by the gateway.
- `include/robot_api_server/keepout_layer.hpp` plus `src/keepout_layer.cpp` own strict keepout geometry validation, map-origin rotation, PGM Y conversion, conservative line/polygon rasterization, multi-projection file rollback, revision/digest generation, and the runtime-effect proof seam.
- `include/robot_api_server/localization_result_model.hpp` plus `src/localization_result_model.cpp` own localization result snapshots and relocalization diagnostic text.
- `include/robot_api_server/storage_models.hpp` plus `src/storage_models.cpp` own map/pose data models and safe ID/name validation used by map assets, poses, navigation goals, and docking poses.
- `include/robot_api_server/map_asset_io.hpp` plus `src/map_asset_io.cpp` own grayscale PNG encoding, PGM dimension reads, and Nav map YAML metadata extraction.
- `include/robot_api_server/map_asset_writer.hpp` plus `src/map_asset_writer.cpp` own OccupancyGrid-to-image conversion, map YAML text generation, neutral costmap filter assets, and asset reports for saved 2D maps.
- `include/robot_api_server/map_catalog.hpp` plus `src/map_catalog.cpp` own released map directory paths, manifest traversal, and map lookup by ID, floor/name, or active state.
- `include/robot_api_server/map_manifest_io.hpp` plus `src/map_manifest_io.cpp`
  own strict `MapManifest v2` parsing and durable atomic serialization.
- `include/robot_api_server/map_asset_identity_binding.hpp` plus
  `src/map_asset_identity_binding.cpp` authenticate the canonical 11-file
  source bundle, bind it to `robot_map_asset_identity`, and stamp the manifest.
- `include/robot_api_server/navigation_cancel_job_model.hpp` plus `src/navigation_cancel_job_model.cpp` own the navigation-cancel job data contract and JSON state payload.
- `include/robot_api_server/poses_io.hpp` plus `src/poses_io.cpp` own `poses.yaml` parsing, writing, lookup, and JSON array formatting for stored poses.
- `include/robot_api_server/runtime_map_context_io.hpp` plus `src/runtime_map_context_io.cpp` own runtime map context JSON file read/write formatting.
- `include/robot_api_server/runtime_map_lookup.hpp` plus `src/runtime_map_lookup.cpp` own saved 2D PNG lookup, runtime flat-map companion file paths, and safe runtime map-name validation.
- `include/robot_api_server/robot_pose_model.hpp` plus `src/robot_pose_model.cpp` own robot pose snapshots and `/api/v1/robot/pose` JSON payloads.
- `include/robot_api_server/runtime_process_utils.hpp` plus `src/runtime_process_utils.cpp` own Linux child-process setup, `/proc` cmdline reads, pid/pgid liveness checks, and process-group signaling helpers.
- `include/robot_api_server/semantic_layer_io.hpp` plus `src/semantic_layer_io.cpp` own keepout semantic JSON paths, raw JSON passthrough, and keepout filter response fragments.
- `include/robot_api_server/subscription_api.hpp` plus `src/subscription_api.cpp` own App subscription request parsing, client ID validation, TTL clamping, and resource-list JSON formatting.
- `include/robot_api_server/subscription_manager.hpp` plus `src/subscription_manager.cpp` own App page-scoped subscription leases, TTL expiry, and subscription state JSON.
- `include/robot_api_server/tf_pose_utils.hpp` plus `src/tf_pose_utils.cpp` own frame ID normalization, yaw extraction, angle wrapping, and ROS timestamp helpers shared by pose and TF handling.

## Boundaries

- Safety stop and resume publish `std_msgs/Bool` to `/safety/estop`.
- Robot battery state is read from Ranger's `/battery_state` (`sensor_msgs/BatteryState`) and exposed in `/api/v1/status` as `bms.soc`, power-supply fields, `bms.present`, `bms.charging_contact`, and `bms.charging_contact_reason`.
- Floor switching calls `/floor_manager/switch_floor` when `resume_navigation=false`.
- Floor switching with `resume_navigation=true` is rejected before map
  selection or runtime mutation with `LIVE_FLOOR_SWITCH_DISABLED`. Live
  cross-floor recovery must eventually use the strict
  `/floor_manager/floor_switch` transaction; the legacy HTTP/service path is
  not an atomic switch.
- Localization trigger calls `/global_localization/trigger`.
- Map listing reads released floor assets and runtime flat map files.
- `POST /api/v1/mapping/2d/start` starts the repository-owned `slam_toolbox` 2D mapping runtime chain.
- `POST /api/v1/mapping/2d/stop` terminates the App-started 2D mapping chain without stopping common services.
- `POST /api/v1/mapping/2d/save` saves the current `slam_toolbox` occupancy grid as flat runtime assets plus a structured floor bundle, then terminates the mapping chain without selecting that map for navigation.
- `POST /api/v1/mapping/stop` and `POST /api/v1/mapping/save` are REST aliases for the same 2D mapping stop/save operations.
- `POST /api/v1/maps/delete` deletes only inactive, non-runtime-bound saved maps that are not referenced by the current elevator configuration. Active/runtime-bound targets fail with `ACTIVE_MAP_DELETE_DISABLED`; elevator-bound targets fail with `ELEVATOR_CONFIG_MAP_IN_USE`. Before deletion it re-resolves the exact building/floor/map root, rejects every symlinked ancestor, atomically renames the map to a same-parent tombstone, fsyncs the namespace, removes the tombstone, and fsyncs again. The endpoint never auto-activates a remaining map.
- `GET /api/v1/robot/pose` returns only a fresh `map -> base_link` TF pose. It never falls back to `/odom` or wheel odom. The server keeps its `/tf` subscription resident at process startup; pose requests and App page leases must not create and destroy reliable `/tf` subscribers because that can churn Fast DDS endpoints and stall the `robot_localization_bridge` TF publisher.
- `GET /api/v1/maps/semantic_layer` reads the backend-owned editable map overlays for the selected `map_id`.
- `GET /api/v1/maps/poses` reads semantic delivery points from the selected `maps/<map_id>/poses.yaml`.

## Elevator configuration management

The commissioning API is deliberately separate from ordinary semantic points:

```text
GET  /api/v1/elevator-config?building_id=<building>[&release_id=<release>]
PUT  /api/v1/elevator-config/draft
POST /api/v1/elevator-config/publish
POST /api/v1/elevator-config/rollback
```

A building draft contains one or more elevators. Every served floor binds an
exact `floor_id` and `map_id`, five map-frame poses (`hall_call`, `hall_wait`,
`doorway`, `cabin`, `exit`), and the door threshold (`left`, `right`,
`cabin_reference`, `clearance_m`, `jamb_clearance_m`). Draft save is allowed
while incomplete and returns structured issues. Publish always revalidates:

- path-safe and unique IDs;
- at least two floors and all five roles;
- finite pose/threshold coordinates and map bounds with rotated map origins;
- exact building/floor/map ownership and all required nav/localizer assets;
- the bound map content digest;
- `robot_elevator_manager` topology validation and generated YAML round-trip.

Draft and release mutations use optimistic `expected_draft_revision` and
`expected_release_id`; stale clients receive `409`. Published releases are
immutable under:

```text
maps_release/<building_id>/.elevator_config/
  draft.json
  drafts/<draft_revision>/
    configuration.yaml
    validation.json
  current -> releases/<release_id>
  current.json -> current/current.json
  releases/<release_id>/
    configuration.yaml
    elevators.yaml
    elevator_internal_poses.yaml
    validation.json
    manifest.json
    current.json
```

A valid draft is persisted with each resolved `map_asset_epoch` and
`map_asset_digest` stamped into its floor binding. If either identity component
changes after review, publish fails with `MAP_ASSET_EPOCH_CHANGED` or
`MAP_ASSET_DIGEST_CHANGED` until the App saves and reviews a new draft.
An invalid saved draft can never become publishable merely because maps are
later created or repaired: publish returns `DRAFT_REVIEW_REQUIRED`, and the
client must save/review a new revision so every digest is server-stamped.
Those digest fields are server-managed; an explicit rebind operation removes
the stale fields before saving so the server can stamp current values and
produce a new revision.
The commissioning document is bounded to 2 MiB, 16 elevators, 64 floors per
elevator, and 128 total floor bindings. Asset resolution is cached per
`(floor_id,map_id)` during one validation. Map manifests and Nav map YAML are
bounded before parsing; each Nav YAML `image` must be exactly one relative
root-level filename matching the digested `nav_map.pgm` (no parent
components). Root `image`, `resolution`, and `origin` must each occur exactly
once; map dimensions are read only from that authenticated PGM, never from a
nested or duplicate `image` key.
Despite the historical `configuration.yaml` filename, the commissioning
document and every persisted configuration/manifest/current selector use the
strict JSON subset documented by the App API. They are limited to 64 nesting
levels. YAML-only syntax, malformed UTF-8/Unicode escapes, trailing content,
and duplicate decoded object keys are rejected before `yaml-cpp` is called.
This includes escaped-equivalent keys such as `elevators` and
`\u0065levators`, and applies at every object level (including `floors`,
`map_asset_epoch`, and `map_asset_digest`). Stored duplicates are an integrity
failure, not a last-key-wins update. Integral scalars stay textual through
preflight so a valid positive `uint64` epoch is never rounded through
floating-point.
The reviewed configuration, including unknown extension fields and their JSON
scalar types, is retained in the release; generated runtime topology remains a
separate strictly validated projection.
The default GET reports `configuration_source: "draft"|"current"|null`;
when a draft exists it is returned for editing even if
`current_release_id` points at a different published release.

`map_asset_epoch` is a persistent positive `uint64` version allocated by
`robot_map_asset_identity` under
`maps_release/.map_asset_registry`. The registry is process-safe and globally
monotonic; identical current content is idempotent, while changed content or a
return to an older digest consumes a new epoch. Empty, corrupt, or regressed
state fails closed, and lookup is read-only. Every bind and lookup audits the
exact source-map manifest layout against the maximum committed epoch; an older
non-empty registry is rejected, while unrelated elevator/current manifests are
ignored. A missing read-only registry or lock is not created as a side effect.

Every committed source bundle is written as `njrh.map_manifest.v2` with the
same epoch, digest algorithm, digest contract, and digest. Manifest writes use
a same-directory temporary file, file fsync, atomic rename, and parent
directory fsync on Linux. The manifest itself and `poses.yaml` are excluded
from the content digest.

`map_asset_digest` is a canonical `sha256:<64 lowercase hex>` wire identity.
The v1 digest sorts stable logical asset names and length-prefixes every name
and byte payload before hashing. The framing itself does not inject filesystem
roots; payload bytes are nevertheless hashed exactly, including paths already
recorded inside `asset_report.json`. The release manifest records
`asset_epoch`, `asset_digest_algorithm: "sha256"`, and
`asset_digest_contract: "njrh-map-asset-bundle-v1"`. The release's own historical
`configuration_digest_algorithm: "fnv1a64"` remains a separate legacy content
identity and must not be substituted for the map asset digest.

Once the current elevator release binds a floor/map, the HTTP keepout mutation
endpoint rejects in-place edits with `ELEVATOR_CONFIG_MAP_IN_USE`. This prevents
the immutable release from silently becoming stale. Commissioning must create
or select the intended map asset version, update/review the elevator draft, and
publish a new release instead of modifying a bound bundle in place.

`maps_release/<building_id>/elevators.yaml` and
`elevator_internal_poses.yaml` are stable symlink views through the same
authoritative `current` directory selector. On Linux, publishing writes and
fsyncs the immutable release first, then performs one atomic selector rename;
consumers that need topology and poses as a coherent pair must resolve/fix the
selected release once and read both files from that release directory.
Rollback copies a historical configuration into a new monotonic release; it
never rewinds or overwrites history. A selector fsync failure returns an
explicit reconciliation error instead of claiming success.

The five internal poses never enter a floor's ordinary `poses.yaml`.
`GET /maps/poses` and `/maps/semantic_layer` also hide any legacy
`type=elevator_internal` or reserved `eip_` records. Ordinary pose mutation
cannot create, replace, or delete them, and ordinary precheck/navigation plus
docking target/predock resolution rejects them with
`ELEVATOR_INTERNAL_POSE_REQUIRES_MISSION`.

Every successful publish/rollback says:

```json
{"asset_published":true,"runtime_applied":false}
```

It does not switch a map, reload localization, start or stop Nav2, send a
goal, publish Twist, or release a safety hold. The future elevator mission
adapter must consume a selected release through
`robot_elevator_manager::load_elevator_release()`, its own execution lease, and
a floor-switch transaction. Loader success is configuration preflight only.
The authoritative epoch/digest identity is frozen in the returned source and
target plans, but the live floor/localizer/costmap evidence chain remains
disabled.

## P6 floor-runtime negative interlock

The API permanently subscribes to:

- `/floor_manager/transition_status`
  (`robot_interfaces/msg/FloorSwitchStatus`);
- `/localization/floor_health`
  (`robot_interfaces/msg/LocalizationHealth`).

This is deliberately a negative-only compatibility fence. Before a live P6
transaction is deployed, missing typed messages and
`LEGACY_CONTEXT_UNSCOPED` do not grant or revoke ordinary runtime readiness.
Explicit `transition_active`, invalid runtime context, mutation-stage
`FloorSwitchStatus`, or `FAILED_LOCKED` blocks new navigation, mapping start or
save, map/pose/keepout mutation, floor selection, manual localization, docking,
undocking, and safety resume. Safety stop plus navigation/mapping/docking
cancel/stop remain available so an operator can converge to a safe state.

The current preflight-only floor Action reports `PREFLIGHT` then `BLOCKED`
without changing assets. Those two states do not latch the API interlock.
`FAILED_LOCKED` is sticky across an inconsistent healthy bridge sample. HTTP
rejections use top-level code `FLOOR_TRANSITION_BLOCKED`, with a specific
`reason_code` and transaction ID. `/api/v1/status` exposes
`floor_runtime_interlock`.

Entry checks are repeated immediately before the main background commits
(Nav2 goal send, mapping process launch, localization trigger, docking/fine
docking, undock service, and asset writes). This narrows races but is not a
distributed execution lease. A completed transaction occurring between target
resolution and a second check can only be eliminated by wiring the future
Mode/Floor execution lease across nodes; therefore this interlock does not
authorize real elevator or floor-switch motion.

Jetson-isolated verification is part of the package tests:

```bash
colcon test --packages-select robot_api_server
```

`floor_runtime_http_smoke` uses its own ROS domain and temporary API process. It
verifies thirteen blocked positive-action endpoints, permits safety stop, confirms a
non-mutating preflight failure does not latch, and confirms a failed lock
cannot be hidden by a healthy sample.
- `POST /api/v1/maps/poses` upserts one semantic delivery point into backend map assets.
- `PUT /api/v1/maps/poses/{pose_id}` updates one semantic delivery point by stable ID.
- `DELETE /api/v1/maps/poses/{pose_id}` deletes one semantic delivery point by stable ID.
- `PUT /api/v1/maps/poses/batch` replaces the full point set for backend admin or batch import.
- `POST /api/v1/maps/poses/save` upserts semantic delivery points into the selected `maps/<map_id>/poses.yaml` and synchronizes `current/poses.yaml` when that map is active.
- `POST /api/v1/maps/poses/save_current` writes a semantic point using the same fresh `map -> base_link` pose as `/api/v1/robot/pose`, so the App does not convert pixels to metric coordinates for live marking.
- `GET /api/v1/maps/filters/keepout` reads the keepout mask asset metadata plus App-authored keepout semantic JSON.
- `POST /api/v1/maps/filters/keepout/save` replaces the complete App-authored keepout layer. It generates the semantic JSON and matching Nav2 mask YAML/PGM as one rollback-capable transaction, synchronizes active-map projections, and hot-loads the mask only when the exact runtime map is selected.
- `GET /api/v1/navigation/pre_goal_check` returns the read-only dock/contact gate that will be used before a normal point navigation goal. It resolves `pose_id` from `poses.yaml` when provided, checks direct `x/y/yaw` map-frame goals when provided, reports `/navigate_to_pose` action-server admission readiness, and never calls `/docking/undock` or sends a Nav2 action.
- `POST /api/v1/navigation/goal` resolves a saved pose or direct map-frame pose, creates a `navigation_goal` job, and returns `202` quickly before waiting for controlled undock, post-undock relocalization, bridge readiness, or the Nav2 action goal handle. The background job then performs any required `/docking/undock`, waits for post-undock localization/settle readiness, waits for `robot_localization_bridge.safe_for_goal_start`, sends the `NavigateToPose` goal to `/navigate_to_pose`, and records `pre_navigation_undocking`, `waiting_for_goal_start_readiness`, `sending_nav2_goal`, and failure diagnostics for App polling. Normal goal admission does not synchronously poll Nav2 lifecycle `GetState` services or wait for the action goal response; those probes are startup/diagnostic checks because they can time out while the controller-hosted costmaps and planner are actually active under Jetson/FastDDS load. Success responses include the dock snapshot plus `pre_navigation_undock` / `pre_navigation_undock_detail`; later stage failures are reported in `/api/v1/navigation/state`.
- Dock/contact detection for navigation admission is explicit, not position-based. In addition to stable BMS contact and `/docking/status`, the server reads `docking_contact_latch.json`, written by charging-session evidence, docking success, manual maintenance confirmation, and undock success. `pre_navigation_dock_check.dock_contact_snapshot`, `dock_contact_latch_source_strength`, `charging_session_latched`, `dock_occupancy_state`, `dock_occupancy_evidence`, `strong_live_docked`, `latch_valid_for_auto_undock`, `docked_state_class`, `docked_evidence`, and `docked_warnings` expose this state. A stale legacy `source=bms` latch is weak safety memory and can be cleared by stable BMS no-contact plus no live docked/charging/undocking context. New charging evidence is stored as strong `source=charging_session`; restart-time idle/no-contact is not enough to clear it. A `source=charging_session` latch is auto-cleared only after confirmed live undock plus stable BMS no-contact, or by explicit maintenance/session clear, so full-charge BMS idle (`current=0`, `present=false`, status unknown) does not let normal Nav2 skip controlled undock when the robot is still physically on the dock.
- `POST /api/v1/navigation/cancel` accepts a background cancel job, publishes zero velocity immediately, uses a short action-server probe for responsive cancel behavior, then cancels the active Nav2 goal while keeping the resident Nav2/localization runtime alive by default. Use `POST /api/v1/navigation/stop` or `/api/v1/navigation/stop_runtime` only for diagnostics or recovery when the runtime must be torn down.
- `GET /api/v1/navigation/state` returns the latest background navigation cancel job and `navigation_goal` state for App polling and diagnostics. Normal point goals default to `goal_completion_policy=pose_required`; the API sends x/y/yaw to Nav2 and treats the Nav2 action result as input to commercial final verification, not business completion by itself. The API waits for bridge `map->odom` smoothing before the final pose read, requires 0.06 m XY and 0.05 rad yaw for normal completion, retries the same Nav2 goal for bounded light/recovery overruns, and enters `degraded` instead of reporting false `task_complete=true` when final verification remains outside the commercial gate. The ordinary terminal correction has a separate lateral hysteresis contract: body-frame lateral error above 0.04 m enters correction, an engaged correction cannot complete until lateral error is at most 0.03 m, and success is rechecked only after `/wheel/odom` is physically stable and Ranger feedback has returned to aligned DUAL_ACKERMAN mode. One bounded correction is allowed if the settled pose leaves the strict gate. If only yaw is outside the yaw-alignable XY window, one bounded ordinary API `final_yaw_align` fallback may run through `/cmd_vel_api`; that fallback also waits for bridge smoothing and pauses `robot_localization_bridge` global-correction intake until the final spin exits. `position_only` is an explicit engineering opt-out.
- `POST /api/v1/docking/start` resolves a saved dock contact pose, prefers a manual pre-dock pose (`predock_pose_id`, `approach_pose_id`, or the `dock_id_predock` naming convention), falls back to a geometric pre-dock offset only when no manual point exists, checks bridge `safe_for_goal_start`, and sends Nav2 toward that pose as a coarse approach. Docking staging uses a different completion contract from ordinary delivery goals: Nav2 is the primary pre-dock owner by default, and API completion is judged in the dock approach frame instead of a normal circular XY goal. With `docking_predock_early_handoff_enabled=false`, the API lets Nav2 finish the pre-dock goal before it enters closed-loop staging capture; the older early-cancel behavior is available only when that engineering switch is explicitly enabled, in which case `predock_nav_early_handoff` and `predock_nav_handoff_detail` expose the cancellation detail. The forward/x error along the dock centerline only has a broad safety window (`predock_forward_capture_min_m` to `predock_forward_capture_max_m`); it is not required to be exactly on the pre-dock point because final docking drives forward into the dock. The lateral/y error to the charging-dock centerline is the tight capture target. A Nav2 `ABORTED` result is retried up to `docking_max_retries` while the pose is outside the docking recovery window. Once the pose is near the handoff, the API enters a closed-loop staging capture for up to `predock_staging_capture_max_cycles`: each cycle re-reads map pose, runs predock yaw alignment if yaw is outside tolerance, then runs bounded side-slip lateral capture if centerline lateral error is outside target, and repeats until strict forward-window/yaw/lateral verification passes. Yaw recovery is allowed as long as it remains below `predock_yaw_align_hard_fail_rad`; it does not short-circuit only because yaw is above `docking_predock_pose_max_yaw_rad`. Before calling `/docking/start` for GS2 fine docking, the API waits for bridge `map -> odom` smoothing to finish in `FINE_DOCKING_BRIDGE_SETTLE`, may rerun yaw alignment if smoothing exposes yaw error, then performs a read-only post-bridge lateral check against `fine_docking_entry_max_lateral_m` without issuing a second side-slip command. Predock lateral capture uses `predock_lateral_align_yaw_slack_rad` as a short-horizon yaw hysteresis gate during side-slip; if the measured lateral error diverges, it stops, reverses side-slip direction once, and then fails with `PREDOCK_LATERAL_ALIGN_DIVERGING` if the reversed command also diverges. It then rechecks the strict final yaw/lateral target before bridge settle; after bridge settle, lateral is only accepted or rejected for fine-entry safety. It requests `/ranger_mini3/forced_mode=side_slip` and releases it back to `auto`; final low-speed contact alignment still belongs to `robot_docking_manager`. When fine docking reports success, failure, or stop, the API can trigger a configured localization pass before it returns to the final docking state so Nav2 does not resume from a pose accumulated only through non-Nav2 fine-docking motion.
- Closed-loop predock staging keeps yaw and lateral predicates separate. `predock_yaw_angles_met` is used to decide whether yaw still needs a spin; strict pose completion still requires the forward window and lateral target. This prevents a lateral miss outside the circular `docking_predock_pose_max_distance_m` from being misreported as `PREDOCK_YAW_NOT_ALIGNED` before side-slip capture can run.
- `POST /api/v1/docking/undock` accepts the App's undock intent only when the robot is docked, live charging contact is detected/inferred from BMS status, or the explicit dock-contact latch indicates a manually confirmed docked state, then calls `/docking/undock`; after odometry-confirmed departure it triggers `/global_localization/trigger`, arms `/robot_localization_bridge/force_accept_next_localization`, waits for `map -> base_link` to reflect the localization result, and exposes the result in docking state. The App must not publish reverse velocity directly.
- `POST /api/v1/docking/confirm_docked` and `POST /api/v1/docking/clear_docked_latch` are maintenance recovery endpoints. They only set or clear the persistent docked latch and never send velocity.
- `POST /api/v1/docking/cancel` / `POST /api/v1/docking/stop` cancels the pre-dock Nav2 goal, publishes zero velocity, and calls `/docking/stop` using the configured docking stop service wait.
- `POST /api/v1/navigation/goal` uses the resident runtime context as the immediate admission contract and defers `robot_localization_bridge.safe_for_goal_start` plus AMCL correction readiness to the accepted background job. Confirmed same-map runtime context plus a fresh `map -> base_link` pose skips `/global_localization/trigger`; if bridge/AMCL readiness is still transitioning, the job waits briefly and then fails with `failed_goal_start_readiness` instead of blocking the mobile HTTP request. Explicit localization recovery remains a separate endpoint for cold or unconfirmed runtime context, stale/missing map-frame pose, controlled undock, docking transitions, or explicit `force_relocalize`. If the Nav2 action server is unavailable after admission, the accepted background `navigation_goal` is marked failed instead of blocking the mobile HTTP request.
- `GET /api/v1/docking/state` returns the latest docking job, `/docking/status`, `charging_contact`, `inferred_docked`, `can_auto_undock`, and the same `pre_navigation_dock_check` used by normal point navigation.
- `GET /api/v1/status` includes HTTP active/max connection counters; the server rejects excess clients with `503` instead of spawning unbounded detached threads.
- `POST /api/v1/subscriptions/acquire|heartbeat|release` controls page-scoped ROS subscriptions for App resources.
- WebSocket teleop keeps a permanent `/battery_state` guard and publishes zero velocity when charging/full or charge current is detected.
- Nav2 action calls have their own exception boundary, and the ROS callback executor is single-threaded so transient action-client faults are caught by the process-level spin loop instead of terminating the HTTP server.
- Other mapping and navigation start endpoints intentionally return `501` until formal ROS-native services/actions are exposed by `robot_mode_manager` or `robot_mission_manager`.
- The test web dashboard is not used as a backend dependency.

## Start

```bash
ros2 launch robot_api_server robot_api_server.launch.py
```

For phone clients, set a token before exposing the robot hotspot:

```bash
ros2 launch robot_api_server robot_api_server.launch.py \
  config_file:=/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/config/robot_api_server.yaml
```

## Minimal API

- `GET /api/v1/status`
- `GET /api/v1/robot/pose`
- `GET /api/v1/maps`
- `GET /api/v1/maps/semantic_layer`
- `GET /api/v1/maps/poses`
- `GET /api/v1/maps/filters/keepout`
- `GET /api/v1/mapping/2d/map`
- `GET /api/v1/openapi`
- `POST /api/v1/maps/poses`
- `PUT /api/v1/maps/poses/{pose_id}`
- `DELETE /api/v1/maps/poses/{pose_id}`
- `PUT /api/v1/maps/poses/batch`
- `POST /api/v1/subscriptions/acquire`
- `POST /api/v1/subscriptions/release`
- `POST /api/v1/subscriptions/heartbeat`
- `POST /api/v1/mapping/2d/start`
- `POST /api/v1/mapping/2d/stop`
- `POST /api/v1/mapping/2d/save`
- `POST /api/v1/mapping/stop`
- `POST /api/v1/mapping/save`
- `POST /api/v1/maps/delete`
- `POST /api/v1/maps/poses/save`
- `POST /api/v1/maps/poses/save_current`
- `POST /api/v1/maps/filters/keepout/save`
- `POST /api/v1/safety/stop`
- `POST /api/v1/safety/resume`
- `POST /api/v1/floors/switch`
- `POST /api/v1/localization/trigger`
- `GET /api/v1/navigation/state`
- `GET /api/v1/navigation/pre_goal_check`
- `POST /api/v1/navigation/goal`
- `POST /api/v1/navigation/cancel`
- `POST /api/v1/navigation/stop`
- `POST /api/v1/navigation/stop_runtime`
- `GET /api/v1/docking/state`
- `POST /api/v1/docking/start`
- `POST /api/v1/docking/undock`
- `POST /api/v1/docking/cancel`
- `POST /api/v1/docking/stop`
- `WS /ws/v1/teleop`

`GET /api/v1/status` includes:

```json
{
  "mode": "IDLE",
  "state": "idle",
  "mapping_active": false,
  "navigation_active": false,
  "healthy": true,
  "message": "",
  "mapping": {
    "active": false,
    "state": "stopped",
    "map_topic": "/map",
    "map_endpoint": "/api/v1/mapping/2d/map"
  },
  "navigation": {
    "active": false,
    "state": "stopped",
    "action": "/navigate_to_pose"
  },
  "docking": {
    "active": false,
    "state": "stopped",
    "dock_id": "",
    "status_topic": "/docking/status",
    "last_status": ""
  },
  "bms": {
    "soc": 48.0,
    "soc_valid": true,
    "source_topic": "/battery_state",
    "age_sec": 0.02,
    "voltage": 491.0,
    "current": -0.7,
    "temperature": 18.9
  }
}
```

The App should treat `mode`, `state`, `mapping_active`, `navigation_active`, `healthy`, and `message` as the lightweight business-state contract. These fields are maintained by API transitions rather than by asking the App to inspect ROS nodes or topics directly. Status and navigation-state handlers must not synchronously probe Nav2 lifecycle services on every mobile poll; blocking lifecycle checks belong to resume/navigation admission and explicit readiness diagnostics. The socket server uses a fixed worker pool (`max_http_connections`, default `16`) instead of detached per-request threads. If the server returns `503 {"error":"server busy"}`, the App should back off and retry instead of opening additional parallel status requests.

AMCL continuous-localization readiness is exposed from the runtime status file and bridge status, not inferred from old logs. `GET /api/v1/status` and `GET /api/v1/navigation/state` include `amcl_state`, `amcl_ready`, `amcl_degraded`, `amcl_degraded_reason`, `amcl_process_alive`, `amcl_scan_admission_alive`, `/amcl_pose` publisher count, scan-admission status publisher count, `amcl_correction_ready`, `amcl_correction_pending`, `localization_degraded`, and `using_triggered_baseline_only`. In `shadow` mode an AMCL startup/readiness failure can continue as a visible degraded Isaac-triggered baseline; in `gated` mode the runtime reports a readiness failure instead of silently treating AMCL as active. A stationary, seeded AMCL with no fresh correction yet is `amcl_correction_pending=true`; it is not a localization recovery requirement by itself, and clean no-motion static standby does not block goal start. Non-standby pending/not-ready correction is treated as a transition; accepted background navigation jobs wait briefly for correction readiness before sending Nav2, then fail as `failed_goal_start_readiness` if it does not recover.

Explicit business relocalization has a second gate after bridge acceptance. `robot_api_server` reads `/localization/bridge_status.last_explicit_relocalization_sequence` and waits for the expected sequence, `map -> odom` owner/freshness, `odom -> base_link` freshness, the static `base_link -> lidar_level_link` transform, at least two `/local_costmap/costmap` updates, and no new local-costmap MessageFilter drops before it sends the next Nav2 goal or starts GS2 fine docking. This is a settle barrier, not a TF tolerance increase. For post-undock goal release, the TF/bridge checks remain hard gates, while local-costmap update/drop and AMCL scan-admission transient checks are recorded as warnings so a successful undock and accepted relocalization do not discard the original navigation goal. Failures are reported as `POST_RELOCALIZATION_*` or `CANCELLED_BY_APP`, and `/api/v1/status` plus `/api/v1/navigation/state` expose `post_relocalization_settle`.

`soc` is normalized to `0..100`. Ranger currently publishes `/battery_state.percentage` as a percent value; if a future driver follows the ROS convention `0.0..1.0`, the API converts it to percent.

## Subscription Manager

High-rate or page-specific ROS inputs are not kept subscribed permanently. The App should acquire resources when entering a page, heartbeat every 3-5 seconds, and release them on exit:

```json
{"client_id":"app_device_001","resources":["status","live_map","tf"],"ttl_ms":10000}
```

For compatibility with older App/Web builds, the server also accepts `clientId`, `lease_id`, `leaseId`, `subscription_id`, and `subscriptionId`. If no client identifier is provided, the request uses the compatibility lease `http:compat-default` instead of returning HTTP 400. Heartbeats may omit `resources`; the server refreshes the client's existing resources, or returns a successful no-op if none are currently leased.

Resources:

- `status`: compatibility resource for status pages. Safety state itself (`/safety/status` and `/safety/motion_allowed`) is subscribed for the whole API process with transient-local QoS, so releasing a page lease does not clear the safety cache used by `/api/v1/status` or motion admission.
- `live_map`: live `slam_toolbox` `/map` PNG rendering permission; the API also keeps an internal `/map` cache while 2D mapping is active for status readiness and save
- `scan`: `/scan` cache hook for laser-layer views
- `tf`: `/tf` cache for `map -> odom -> base_link` pose
- `teleop`: WebSocket mapping teleop lifecycle

When a resource refcount changes `0 -> 1`, page-specific ROS subscriptions are created. When it changes `1 -> 0`, page-only subscriptions are reset and high-frequency cache is cleared. Core health subscriptions and the `/map` subscription retained during active 2D mapping are not page-owned, so startup readiness and save do not depend on App polling order. Leases expire automatically after `ttl_ms`; default is `10000`.

`GET /api/v1/mapping/2d/map` only serves live `/map` when `live_map` is currently acquired. Saved map preview through `?source=saved` or `?name=<map>` does not require `live_map`.

## Floor Selection And Disabled Live Switch

`POST /api/v1/floors/switch` with `resume_navigation=false` is an offline,
selection-only asset request to `/floor_manager/switch_floor`. It is rejected
with `FLOOR_SELECTION_RUNTIME_BUSY` unless navigation, mapping, docking, API
goal jobs, Nav2 action goals, and the navigation process are all stopped. On a
successful selection it clears the stale runtime-map context before projecting
the selected manifest into `current/`. It does not reload localization or start
Nav2.

`resume_navigation=true` is now rejected with
`LIVE_FLOOR_SWITCH_DISABLED` before map lookup, active-manifest mutation, ROS
service calls, or process launch. The previous direct path to:

```text
scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh <building_id> <floor_id>
```

was removed because it bypassed the transaction Action and could update the
selected map before live switch failure was known. The script remains a
repository-owned single-floor startup implementation used by controlled
startup/recovery paths; it is not a valid cross-floor commit protocol.
Production floor switching remains disabled until the Action owns exact asset
identity, bridge fencing, localizer reload proof, target localization, fresh
costmaps, and final readiness under a motion hold.

## WebSocket Teleop

`/ws/v1/teleop` is for App-driven low-speed mapping teleop only. It is accepted only while the 2D mapping chain is active by default. It publishes `geometry_msgs/Twist` to `/cmd_vel_api`, so the command still flows through `robot_safety`, final `/cmd_vel`, and the `ranger_base` chassis core.

Client messages:

```json
{"type":"cmd_vel","linear_x":0.20,"angular_z":0.10}
{"type":"cmd_vel","linear_x":-0.12,"angular_z":0.00}
{"type":"stop"}
```

Limits are enforced by server parameters:

- `teleop_max_linear_x_mps`: default `1.00`
- `teleop_max_angular_z_radps`: default `0.55`
- `teleop_allow_reverse`: default `true` for mapping teleop only
- `teleop_require_mapping_active`: default `true`
- `teleop_watchdog_timeout_sec`: default `0.5`

During a WebSocket teleop session, the server publishes `/ranger_mini3/teleop_allow_reverse=true` so `robot_safety` accepts low-speed reverse commands. When the WebSocket disconnects, times out, or receives `stop`, the server publishes one zero `Twist` and disables teleop reverse again. When no teleop session is active, it does not publish reverse-disable messages, so docking undock permissions remain independent.

## 2D Map PNG

`POST /api/v1/mapping/2d/start` starts the same repository-owned `slam_toolbox` chain used by the debug Web 2D mapping path. If navigation is active, the server cancels the active Nav2 task and publishes zero velocity for the mode switch, but it does not call the destructive navigation runtime stop path.

`POST /api/v1/mapping/2d/stop` stops the 2D mapping-side chain and returns `{"ok":true,"mapping_active":false}`. The stop scope includes `run_projected_map.sh`, `slam_toolbox`, scan preprocessing / republishing nodes, the C++ mapping-only odom bridge, and the mapping-owned FAST-LIO2 process marked with `NJRH_SLAM2D_PRIVATE_FASTLIO=1`. It does not kill chassis, canonical TF, local state, safety, or the API server.

`POST /api/v1/mapping/2d/save` accepts `building_id`, `floor_id`, and a business `map_name`. The server generates a stable `map_id`, preserves the original name as `display_name`, writes the map bundle under `maps_release/<building_id>/<floor_id>/maps/<map_id>/`, and then runs the same mapping-chain stop logic as `/mapping/2d/stop`. It intentionally does not activate the saved map. Runtime consumers keep reading fixed role files from `current/nav/nav_map.yaml` and `current/localizer/localizer_map.png`; those files change only after a later explicit `/api/v1/floors/switch` map selection.

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_name": "一楼大厅配送图"
}
```

Within the map bundle, nav files use a safe filename stem derived from
`map_name`, for example `nav/<safe_map_name>.yaml/.pgm`; Isaac localization
uses `localizer/<safe_map_name>.png`. `manifest.json` also records the strict
v2 epoch/digest identity in addition to map metadata and asset paths.

`GET /api/v1/maps` returns `floor_maps[]` for real map records and keeps `floors[]` only as a compatibility floor/current-map view. `floors[]` should not be treated as the map list. Each map entry includes `map_info` parsed from the Nav2 map YAML and image header so App-side taps can be converted to real map coordinates without assuming PNG scale:

```json
{
  "map_id": "map_20260520T120000Z_012345abcd",
  "display_name": "Lobby delivery map",
  "asset_epoch": 17,
  "asset_digest": "sha256:<64 lowercase hex>",
  "map_info": {
    "width": 242,
    "height": 103,
    "resolution": 0.05,
    "origin": [-3.2, -1.8, 0.0]
  }
}
```

`POST /api/v1/floors/switch` accepts `map_id` or `map_name`; `map_id` is
preferred. In selection-only mode it first waits for the legacy floor service
to validate the request, and only after a successful response activates the
selected manifest into `current/`. A failed service request therefore does not
change the active manifest.

Activation treats `maps_release/<building_id>/<floor_id>/current/` as a backend-owned runtime mirror. If an older dashboard or root-run process left `current/` as a non-empty real directory, the API first attempts normal removal, then safely quarantines the stale directory inside the same floor folder before creating a fresh runtime mirror. App clients should not write into `current/` directly.

`POST /api/v1/maps/delete` deletes only by `map_id`:

```json
{"map_id":"map_20260520T120000Z_012345abcd"}
```

The endpoint refuses building/floor-only deletion so a phone client cannot
accidentally remove an entire building or floor asset tree. It also serializes
against elevator draft/publish/rollback. A map referenced by the authoritative
current elevator release returns HTTP `409 ELEVATOR_CONFIG_MAP_IN_USE`; edit,
validate, and publish a release that no longer references that map before
deleting it.

## Semantic Poses And Navigation Goals

Semantic App points and keepout edits are stored as backend-owned overlay assets, not baked into `nav_map.pgm` or `localizer_map.png`. Android should not restore these overlays from local files.

Read the complete editable overlay layer:

```http
GET /api/v1/maps/semantic_layer?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

The response includes `poses[]`, `filters.keepout`, `keepout_mask.yaml/.pgm` paths, and any App-authored `filters/keepout_semantic_layer.json` content. If a client needs only keepout:

```http
GET /api/v1/maps/filters/keepout?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

Replace the complete keepout layer:

```http
POST /api/v1/maps/filters/keepout/save
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "keepout_lines": [
    {
      "id": "door_barrier_1",
      "name": "door barrier",
      "width_m": 0.6,
      "points": [{"x": 1.2, "y": 3.4}, {"x": 2.8, "y": 3.4}]
    }
  ],
  "keepout_polygons": [],
  "expected_revision": "keepout-v1-fnv64-0123456789abcdef",
  "reload_filter": true
}
```

Coordinates are finite map-frame metres. `width_m` is the full physical line width. The renderer applies the Nav map's complete `[origin_x, origin_y, origin_yaw]`, converts ROS bottom-left grid rows to PGM top-left rows, and conservatively marks cells touched by lines or polygons. Sending both arrays empty clears the complete layer and restores an all-neutral mask.

The backend canonicalizes and writes `keepout_semantic_layer.json`, `keepout_mask.yaml`, `keepout_mask.pgm`, and `keepout_commit.json` for the immutable map bundle, fixed active projections, and the actual Nav2 runtime staging directory. Each feature must hit the map and at least one free nav cell. It rejects malformed/self-intersecting geometry, duplicate IDs, excessive raster work, a layer that blocks every map cell, an older non-neutral mask with no editable semantic source, and semantic/PGM disagreement. File replacement uses checked write/fsync/rename, parent-directory fsync, whole-transaction readback, and rollback. The commit marker is written last so startup can fail closed after an interrupted multi-file update.

`GET /api/v1/maps/filters/keepout` returns `revision` / `keepout_revision`. Clients should send that value as `expected_revision` on full-layer replacement. A stale value returns HTTP `412 REVISION_CONFLICT` without writing. The revision is computed from canonical geometry, not JSON whitespace, `reload_filter`, or the expected revision itself.

When the exact map is loaded by an idle navigation runtime, `reload_filter` must remain `true`. Success then requires all of these proofs: the mask and filter-info lifecycle servers and global costmap are active, `KeepoutFilter` is enabled, `LoadMap` succeeds, a fresh `/keepout_filter_mask` message matches the candidate geometry and occupancy digest, the global costmap clear service acknowledges, and a full global costmap with a header stamp strictly after that clear is published. Per-feature samples added by the update must be exactly lethal (`100`, not inflation value `99`); sampled cells removed by the update must no longer be lethal or unknown. The global-costmap subscription exists only for this bounded proof and is destroyed before the request returns. The response uses:

- `outcome: "APPLIED"`, `runtime_selected: true`, and `runtime_effective: true` for a proven live update.
- `outcome: "SAVED_DEFERRED_INACTIVE"`, `runtime_selected: false`, and `effective_on_next_activation: true` when the target map is not currently loaded.
- `outcome: "NO_CHANGE"` for an identical inactive replacement.

A changed, unbound keepout layer recomputes the complete map bundle digest and
atomically stamps a new monotonic `asset_epoch`; `NO_CHANGE` does not consume
an epoch. If that identity stamp cannot be completed after the keepout
transaction, integrity is latched degraded and later admission fails closed.
The latch is persisted before the first payload write. Restart recovery keeps
it repairable only for the same map when the descriptor-pinned digest of all
nine non-keepout roles still matches; any extra map fault, missing map, or
non-keepout drift converts the global latch to an unrepairable state.

Map activation separately persists a transaction journal before changing
active manifests or the `current/` and floor compatibility views. Projection
files and their child/root/parent directories are fsynced before that journal
is removed, and startup replays a surviving journal. Live floor switching
nonetheless remains disabled: compatibility projection still reopens some
source paths and is not yet end-to-end descriptor-pinned from the verified
snapshot through publication.

The endpoint shares an admission barrier with navigation, mapping, docking, floor switching, map deletion, and map saving. It also checks the Nav2 action-status topic and requires stable stopped wheel odometry before an active-map update. Rollback ambiguity latches `keepout_integrity_degraded`, reports persistence/effectiveness as unknown, and blocks motion admission until a successful replacement re-proves integrity. The endpoint never cancels Nav2, sends velocity, or restarts a node. Neutral keepout mask servers/plugins remain resident by default so the first line can be applied without a Nav2 restart.

Read points for an existing map:

```http
GET /api/v1/maps/poses?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

`map_id` is preferred and reads that exact `maps/<map_id>/poses.yaml`. If `map_id` is omitted, the server reads the active map for the requested `building_id` and `floor_id`; `map_name` / `display_name` is accepted only as a fallback selector.

```json
{
  "ok": true,
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "display_name": "test-17",
  "poses": [
    {"pose_id": "delivery_123456", "type": "delivery_point", "name": "Point A", "x": 5.75, "y": -0.82, "yaw": 1.57}
  ]
}
```

Create or upsert a point:

```http
POST /api/v1/maps/poses
```

```json
{"building_id":"B1","floor_id":"F1","map_id":"map_20260520T120000Z_012345abcd","pose_id":"delivery_123456","type":"delivery_point","name":"Point A","x":5.75,"y":-0.82,"yaw":1.57}
```

Update a point by stable ID:

```http
PUT /api/v1/maps/poses/delivery_123456
```

```json
{"building_id":"B1","floor_id":"F1","map_id":"map_20260520T120000Z_012345abcd","name":"Point A","x":5.75,"y":-0.82,"yaw":1.57}
```

Delete a point by stable ID:

```http
DELETE /api/v1/maps/poses/delivery_123456?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

Replace the full point set for admin tools or batch import:

```http
PUT /api/v1/maps/poses/batch
```

```json
{"building_id":"B1","floor_id":"F1","map_id":"map_20260520T120000Z_012345abcd","poses":[{"pose_id":"delivery_123456","type":"delivery_point","name":"Point A","x":5.75,"y":-0.82,"yaw":1.57}]}
```

Batch replacement writes exactly the submitted list; `"poses":[]` clears all semantic points for that selected map. Active map changes are synchronized to `current/poses.yaml` so task navigation reads the same backend state the App just edited.

Legacy save-or-update endpoint kept for existing clients:

```http
POST /api/v1/maps/poses/save
```

```json
{"building_id":"B1","floor_id":"F1","map_id":"map_20260520T120000Z_012345abcd","pose_id":"delivery_123456","type":"delivery_point","name":"Point A","x":5.75,"y":-0.82,"yaw":1.57}
```

If `map_id` is omitted, the active floor map receives the pose. Active map edits are synchronized back into `current/`.

Preview the current robot pose in map coordinates:

```http
GET /api/v1/robot/pose
```

```json
{
  "ok": true,
  "frame_id": "map",
  "child_frame_id": "base_link",
  "x": 1.23,
  "y": -0.45,
  "yaw": 0.78,
  "stamp": 1770000000.123,
  "age_sec": 0.03,
  "map_id": "map_20260520T120000Z_012345abcd",
  "floor_id": "F1",
  "building_id": "B1"
}
```

If the server cannot read a fresh `map -> base_link` pose within `robot_pose_freshness_sec` (default `0.5` seconds), it returns `503` and never returns an old cached pose:

```json
{"ok":false,"error":"no fresh map-frame robot pose","frame_id":"map","child_frame_id":"base_link","age_sec":null}
```

The map identity attached to this pose is not discovered by scanning every active floor map. Navigation writes a runtime map context when a floor switch starts and marks it confirmed only after localization has produced `map->odom` and Nav2 has a ready global costmap. While that context is missing or still pending, `/api/v1/robot/pose` returns `503` during navigation/docking instead of attaching a stale map such as a previous floor's `building_id` / `floor_id`.

Save the robot's current map-frame pose as a semantic point:

```http
POST /api/v1/maps/poses/save_current
```

```json
{"building_id":"B1","floor_id":"F1","map_id":"map_20260520T120000Z_012345abcd","type":"delivery_point","name":"Current Point"}
```

The server requires the same fresh `map -> base_link` pose as `GET /api/v1/robot/pose`, writes the selected `maps/<map_id>/poses.yaml`, and synchronizes `current/poses.yaml` when that map is active. If no `pose_id` is supplied, the server generates one. Request-body `yaw` / `theta` is intentionally ignored; saved `x/y/yaw` must equal the live robot pose. For `type: "dock"`, the saved pose is the final charging-contact `base_link` pose, not the pre-dock approach point. This endpoint is for live field marking; static map editing should keep using `POST /api/v1/maps/poses`.

Send the robot to a saved point through Nav2:

```http
POST /api/v1/navigation/goal
```

```json
{"building_id":"B1","floor_id":"F1","pose_id":"delivery_123456"}
```

The API server reads `maps_release/<building_id>/<floor_id>/poses.yaml` and queues a background `NavigateToPose` send to `/navigate_to_pose`. Direct map-frame goals are also accepted with `x`, `y`, and `yaw`, but phone clients should normally use `pose_id` so the car remains the source of truth for floor assets. Goal and cancel calls are serialized around the Nav2 action client; transient rclcpp action-client exceptions are captured by the background job or logged without taking down port 8080. All HTTP handlers are wrapped by a request-level exception guard, so individual ROS service/action/file failures return JSON errors instead of aborting the API process. Successful acceptance returns `navigation_goal_id`; Nav2 action-send failure and completion are reported by the background `navigation_goal` object in `/api/v1/status` and `/api/v1/navigation/state`.

Delivery completion is policy-driven and has one safety-owned command chain. Normal delivery defaults to `goal_completion_policy=pose_required`, so target x/y/yaw is sent in the Nav2 action and Nav2 native `RotationShimController + SimpleGoalChecker(stateful=false)` handles the primary XY+yaw approach. The default Nav2 pose gate is 0.06 m XY and 0.05 rad yaw. After any Nav2 result, the API waits for bridge smoothing, re-reads a fresh `map -> base_link`, and only sets `task_complete=true` when the commercial gate is satisfied. Active bridge smoothing/correction still blocks final completion, but clean gated-AMCL stationary standby (`amcl_static_standby=true` and `amcl_not_moving_no_update_ok=true`) is tolerated even if the AMCL status file still reports `amcl_correction_pending=true`. A 0.06-0.12 m XY overrun or 0.05-0.15 rad yaw overrun triggers bounded same-goal retry or final yaw alignment; API final yaw targets 0.045 rad internally before the unchanged 0.05 rad commercial gate. A 0.12-0.35 m XY overrun or 0.15-0.35 rad yaw overrun enters recovery retry. During those final-verify retries the API publishes a short-lived `/ranger_mini3/allow_reverse` permit so `robot_safety` can allow bounded overshoot correction; outside that permit, the safety layer clamps ordinary reverse. If retries cannot satisfy the gate, the job is marked `degraded` with `task_complete=false` instead of being reported as arrived. Post-retry XY within 0.08 m may be accepted through `post_nav2_final_verify_acceptance_slack_m=0.02`. `position_only` remains an explicit engineering opt-out when final heading is irrelevant.

API-owned ordinary final yaw and predock yaw alignment also check actual chassis stop before declaring success. Non-zero yaw commands reset the stop-stability counter for the current motion. When the remaining yaw is inside the configured stop-lead window (`abs(command_wz) * yaw_align_stop_lead_time_sec`, capped by `yaw_align_stop_lead_max_rad`), the server sends zero early, waits for `/wheel/odom.twist.twist.angular.z` to remain below `yaw_align_actual_wz_threshold_radps` for `yaw_align_actual_wz_stable_samples`, bounded by `yaw_align_actual_stop_timeout_ms`, then re-reads the pose and rechecks yaw. This prevents a commanded-zero spin from being treated as finished while Ranger Mini 3 still has residual angular motion.

The production API does not proactively cancel an executing near-goal Nav2 action: `navigation_near_goal_stalled_handoff_enabled=false`. `GoalScopedRotationShimController` owns the bounded non-Ackermann terminal residual while the same `FollowPath` action remains active, so controller success and progress-checker state stay coherent. The API deterministic terminal servo remains the fallback only after a true Nav2 abort. That fallback decomposes the target error into signed yaw, body-frame forward error, and body-frame lateral error, then corrects yaw first with pure `angular.z`, lateral second with pure `linear.y` in `side_slip`, and forward/reverse third with pure `linear.x`. Neither owner loosens the 0.06 m / 0.05 rad acceptance gate, and Ranger Mini3 mixed x/y commands remain prohibited.

The `navigation_goal` JSON exposes final verification and recovery diagnostics for the App and field logs: `final_pose_verified`, `task_complete`, `final_pose_verify_reason`, `final_verify_retry_count`, `final_verify_retry_reason`, `final_verify_retry_goal_sent`, `final_verify_xy_error_m`, `final_verify_yaw_error_rad`, `final_verify_failure_is_terminal`, `final_yaw_align_attempted`, `final_yaw_align_blocked_reason`, `final_yaw_align_duration_sec`, `final_yaw_align_timeout_sec`, `final_yaw_align_target_yaw_rad`, `final_yaw_align_initial_yaw_error_rad`, `final_yaw_align_final_yaw_error_rad`, `final_yaw_align_max_xy_drift_m`, `final_yaw_align_observed_xy_drift_m`, `final_yaw_align_cmd_topic`, and `final_yaw_align_bypass_collision_monitor`. App clients should show ordinary point success only when `task_complete=true`.

When the backend state is `docked`, `/docking/status` starts with `docked` or `charging`, stable BMS charging contact is fresh, or valid non-stale latch evidence is present, `/api/v1/navigation/goal` accepts the navigation job and the background job automatically performs controlled undocking before sending the Nav2 goal. The same snapshot is available through read-only `GET /api/v1/navigation/pre_goal_check` and through `/api/v1/status` / `/api/v1/docking/state` as `pre_navigation_dock_check`. The read-only endpoint keeps the existing `would_auto_undock` field and also exposes `auto_undock_required`. The snapshot exposes `api_bms_charging_contact`, `api_bms_charging_contact_stable`, `api_bms_charging_contact_reason`, `dock_contact_snapshot`, `dock_contact_latch_age_sec`, `dock_contact_latch_stale`, `dock_contact_latch_contradicted_by_live_state`, `dock_contact_latch_auto_cleared`, `strong_live_docked`, `latch_valid_for_auto_undock`, `docked_state_class`, `docked_evidence`, `docked_warnings`, `bms.power_supply_status`, `bms.current`, `docking.last_status`, `final_is_docked_or_charging`, `final_auto_undock_required`, and `auto_undock_reason`, so a full battery with `current=0` is still diagnosable through `POWER_SUPPLY_STATUS_FULL` or the BMS contact reason. If BMS reports no contact because the charger signal is missing or the robot was manually pushed onto the dock, maintenance can call `POST /api/v1/docking/confirm_docked`; this writes only the latch and sends no velocity. `POST /api/v1/docking/clear_docked_latch` clears only that latch and also sends no velocity. `scripts/jetson/runtime_overlay/scripts/verify_dock_contact_latch_gate.sh` checks the latch file, API gate, `/docking/status`, and `/battery_state` without moving the robot; `--clear-stale-bms-latch` is the explicit operator path to clear old BMS latch evidence. Because `robot_docking_manager` is normally resident, `/docking/undock` should already be available; the API start command remains only as a fallback when the resident service is absent. After `/docking/status` reports odometry-confirmed `undocked`, the background job arms `/robot_localization_bridge/force_accept_next_localization`, triggers `/global_localization/trigger`, waits until the fresh localization result is reflected in `map -> base_link`, and then releases the held Nav2 send. If undocking, post-undock relocalization, or bridge acceptance fails or times out, the accepted navigation job moves to failed state and no Nav2 goal is sent. Goal responses include `pre_navigation_undock`, `pre_navigation_undock_detail`, and `pre_navigation_dock_check` so the App can show whether departure from the charger is queued.

`/api/v1/status` and `/api/v1/navigation/state` also expose `safety.status`, `safety.motion_allowed`, `navigation.blocked_by_docked_contact`, and `navigation.normal_motion_blocked_reason`. If the API gate is bypassed and `robot_safety` blocks a normal command while dock/contact evidence is active, the App can display `DOCKED_CONTACT_BLOCK` without inferring dock state from position.

Field diagnostic:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_pre_navigation_undock_gate.sh \
  --building-id building_1 --floor-id floor_1 --pose-id delivery_1

bash scripts/jetson/runtime_overlay/scripts/verify_docked_navigation_undock_gate.sh \
  --building-id building_1 --floor-id floor_1 --pose-id delivery_1
```

The scripts are read-only unless `--execute-goal` is supplied. The docked interlock verifier also has optional `--test-normal-cmd-block` and `--test-docking-cmd-allowed` checks that publish one low test command; use them only during controlled bench validation.

Cancel requests are wired to Nav2, not only acknowledged over HTTP:

```http
POST /api/v1/navigation/cancel
```

By default this is a task cancel, not a process stop. The HTTP handler publishes a zero burst to `/cmd_vel_api`, creates a background cancel job, and returns `202 Accepted` quickly. The background job uses `navigation_cancel_action_wait_sec` as a short `/navigate_to_pose` action-server probe, cancels the cached API-started `NavigateToPose` goal handle when the action server is available, sends cancel-all to `/navigate_to_pose`, publishes another zero burst, and leaves the resident Nav2/localization runtime alive.

All API-owned Twist command publishers use `KEEP_LAST(1)` QoS. `/cmd_vel_api` is a latest-command control stream, not a queued instruction log; a zero command must replace older nonzero yaw/linear commands immediately before `robot_safety` arbitrates the final `/cmd_vel`.

The endpoint remains idempotent for task exit: if the Nav2 action server is already gone, it still publishes zero velocity and records the cancel job failure reason without tearing down localization.

For diagnostics or recovery, call `POST /api/v1/navigation/stop` or `POST /api/v1/navigation/stop_runtime`. These endpoints force the same background cancel job with `stop_stack=true`: they cancel any Nav2 goal, publish zero velocity, terminate the resident Nav2/localization runtime, clear `/tmp/njrh_runtime_map_context.json`, and verify that no Nav2/localizer/`robot_localization_bridge` processes remain. The stop script tears down Nav2/localization process patterns before bounded AMCL cleanup, so AMCL lifecycle waits cannot consume the API stop window before Nav2 has been cleaned up. Startup failure before confirmed `ready` also rolls back both Nav2 and the occupancy-localization helper layer so the next resume starts from a clean process set. The App's return-to-charger action should not call these endpoints first; it should call `/api/v1/docking/start` directly so the backend can switch modes without stopping Nav2 and localization.

Poll the latest cancel job and goal job with:

```http
GET /api/v1/navigation/state
```

`GET /api/v1/mapping/2d/map` returns `Content-Type: image/png` for the live `slam_toolbox` `/map` cached after that App-started mapping session begins. It does not return a map_server static map or historical saved map by default.

## Docking

Docking is a two-stage backend workflow. The App stores a dock pose in the same `poses.yaml` mechanism as delivery points, normally with `type: "dock"`. That pose represents the final robot `base_link` pose when the front charging contacts are aligned with the physical dock. The API computes the pre-dock pose by backing up from that final pose along its yaw by `docking_pre_dock_distance_m` (default `0.60 m`).

Start docking:

```http
POST /api/v1/docking/start
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "dock_id": "dock_main",
  "resume_navigation": true
}
```

The docking endpoint never activates a map. `map_id` must already be the active
manifest and must exactly match the confirmed, ready runtime context
(`building_id/floor_id/map_id`); otherwise admission fails with
`FLOOR_SWITCH_REQUIRED`. The backend rechecks that identity immediately before
accepting the job, then starts/reuses only that same-map navigation runtime,
cancels any cached API navigation goal without stopping the Nav2/localization
stack, checks bridge `safe_for_goal_start`, and sends Nav2 toward the pre-dock
pose as coarse approach. The remaining manual pre-dock, staging, bridge-settle,
and fine-docking gates are unchanged.
`docking_manual_predock_distance_check_enable` is `false` by default, so the
manual pre-dock distance check remains optional while yaw sanity still applies.

Cancel docking:

```http
POST /api/v1/docking/cancel
```

This cancels the cached pre-dock Nav2 goal, calls `/docking/stop`, and publishes a zero velocity burst through the existing safe command path. The App should poll:

```http
GET /api/v1/docking/state
GET /api/v1/status
```

Undock from a charger:

```http
POST /api/v1/docking/undock
```

```json
{
  "dock_id": "dock_main",
  "reason": "app_manual_undock"
}
```

The undock endpoint is accepted only when the backend state is already docked, the API sees live charging contact, or the persistent dock-contact latch is explicitly set. It calls the resident `robot_docking_manager` through `/docking/undock`, starting it only as a fallback if the service is unexpectedly absent, and tracks the job as `undocking` until `/docking/status` reports `undocked` or `undock_failed...`. `robot_docking_manager` reads the same latch before accepting `/docking/undock`, so `pre_navigation_dock_check` must not claim `can_auto_undock=true` unless the controlled undock service will accept that dock evidence. Once odometry confirms departure, the API enters `relocalize_after_undock`, arms `/robot_localization_bridge/force_accept_next_localization`, calls `/global_localization/trigger`, verifies that the resulting localization is reflected in `map -> base_link`, and records `post_undock_relocalization_*` fields in the docking job. Manual undock remains `undocked` even if the relocalization warning needs operator attention. Auto-undock before `/api/v1/navigation/goal` also remains `undocked` after physical departure; if post-undock relocalization or hard TF/bridge readiness fails, the pending Nav2 goal is not sent and `post_undock_navigation_readiness_failed=true` explains the blocker. Local costmap and AMCL scan-admission transients after an accepted post-undock relocalization are warnings, not blockers. Reverse motion remains inside `robot_docking_manager -> /cmd_vel_docking -> robot_safety -> /cmd_vel -> ranger_base`; `/cmd_vel_safe` is diagnostic only. The retained undock speed is `0.06 m/s`; first-motion delay is handled by `undock.motion_start_timeout_s`, while `undock.no_progress_timeout_s` is reserved for a stall after movement has started.

`accepted=true` is an API admission result, not a synonym for the underlying ROS Trigger result. The undock response and `GET /api/v1/docking/state` expose `api_accepted`, `already_running`, `docking_service_called`, `docking_service_success`, `docking_service_message`, `docking_status_at_request`, `docking_status_after_request`, `undock_started_observed`, `undock_cmd_count_observed`, `undock_failure_reason`, and `docking_service_warning` so field diagnostics can distinguish API admission, `/docking/undock` service success, `/docking/status` observation, and downstream motion execution.

Relevant states are `accepted`, `relocalize_before_predock`, `nav_to_predock`, `STAGING_NAV2_EARLY_HANDOFF`, `STAGING_NAV2_GOAL_ABORTED_HANDOFF_CHECK`, `relocalize_after_predock`, `fine_bridge_settle`, `fine_docking`, `relocalize_after_fine_docking`, `docked`, `undocking`, `relocalize_after_undock`, `undocked`, `failed`, and `canceled`.

Saved-map preview is explicit:

```text
GET /api/v1/mapping/2d/map?source=saved
```

Optional exact lookup:

```text
GET /api/v1/mapping/2d/map?name=test-16
```

This returns `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/maps/test-16.png` when it exists.

Saved mode reads an existing PNG asset; it does not convert PGM at request time.

If `api_token` is non-empty, requests must include:

```text
X-Robot-Token: <token>
```
