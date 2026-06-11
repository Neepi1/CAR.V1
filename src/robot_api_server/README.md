# robot_api_server

`robot_api_server` is the production-facing HTTP gateway for Android and other non-ROS clients.

It does not own mapping, localization, navigation, or chassis control logic. It only exposes a narrow HTTP API and forwards requests into existing ROS 2 topics and services.

## Code layout

- `src/robot_api_server_node.cpp` owns ROS wiring, HTTP routing, runtime process control, and API state.
- `include/robot_api_server/bms_contact.hpp` plus `src/bms_contact.cpp` own the BMS charging-contact policy used by teleop guard, status reporting, and undock admission.
- `include/robot_api_server/api_time_utils.hpp` plus `src/api_time_utils.cpp` own UTC timestamp formatting and generated map/current-pose IDs.
- `include/robot_api_server/docking_job_model.hpp` plus `src/docking_job_model.cpp` own the docking job data contract and JSON state payload.
- `include/robot_api_server/docking_status_utils.hpp` plus `src/docking_status_utils.cpp` own docking/undocking status string classification used by docking state transitions.
- `include/robot_api_server/file_utils.hpp` plus `src/file_utils.cpp` own common text/binary file reads and writes, PGM output, and map YAML image-file rewrites.
- `include/robot_api_server/floor_asset_resolver.hpp` plus `src/floor_asset_resolver.cpp` own floor asset completeness checks, active `current/` selection, `poses.yaml` fallback, and stored pose lookup for navigation/docking.
- `include/robot_api_server/http_common.hpp` plus `src/http_common.cpp` own HTTP request/response structs, WebSocket accept-key helpers, and the lightweight JSON helpers used by the gateway.
- `include/robot_api_server/localization_result_model.hpp` plus `src/localization_result_model.cpp` own localization result snapshots and relocalization diagnostic text.
- `include/robot_api_server/storage_models.hpp` plus `src/storage_models.cpp` own map/pose data models and safe ID/name validation used by map assets, poses, navigation goals, and docking poses.
- `include/robot_api_server/map_asset_io.hpp` plus `src/map_asset_io.cpp` own grayscale PNG encoding, PGM dimension reads, and Nav map YAML metadata extraction.
- `include/robot_api_server/map_asset_writer.hpp` plus `src/map_asset_writer.cpp` own OccupancyGrid-to-image conversion, map YAML text generation, neutral costmap filter assets, and asset reports for saved 2D maps.
- `include/robot_api_server/map_catalog.hpp` plus `src/map_catalog.cpp` own released map directory paths, manifest traversal, and map lookup by ID, floor/name, or active state.
- `include/robot_api_server/map_manifest_io.hpp` plus `src/map_manifest_io.cpp` own `MapManifest` path derivation and `manifest.json` read/write formatting.
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
- Floor switching with `resume_navigation=true` starts the repository-owned floor navigation runtime, which loads the requested floor assets, starts localization, and starts standard Nav2.
- Localization trigger calls `/global_localization/trigger`.
- Map listing reads released floor assets and runtime flat map files.
- `POST /api/v1/mapping/2d/start` starts the repository-owned `slam_toolbox` 2D mapping runtime chain.
- `POST /api/v1/mapping/2d/stop` terminates the App-started 2D mapping chain without stopping common services.
- `POST /api/v1/mapping/2d/save` saves the current `slam_toolbox` occupancy grid as flat runtime assets plus a structured floor bundle, then terminates the mapping chain without selecting that map for navigation.
- `POST /api/v1/mapping/stop` and `POST /api/v1/mapping/save` are REST aliases for the same 2D mapping stop/save operations.
- `POST /api/v1/maps/delete` deletes saved map assets using the same `map_name` and `building_id` / `floor_id` naming rules as save.
- `GET /api/v1/robot/pose` returns only a fresh `map -> base_link` TF pose. It never falls back to `/odom` or wheel odom. The server keeps its `/tf` subscription resident at process startup; pose requests and App page leases must not create and destroy reliable `/tf` subscribers because that can churn Fast DDS endpoints and stall the `robot_localization_bridge` TF publisher.
- `GET /api/v1/maps/semantic_layer` reads the backend-owned editable map overlays for the selected `map_id`.
- `GET /api/v1/maps/poses` reads semantic delivery points from the selected `maps/<map_id>/poses.yaml`.
- `POST /api/v1/maps/poses` upserts one semantic delivery point into backend map assets.
- `PUT /api/v1/maps/poses/{pose_id}` updates one semantic delivery point by stable ID.
- `DELETE /api/v1/maps/poses/{pose_id}` deletes one semantic delivery point by stable ID.
- `PUT /api/v1/maps/poses/batch` replaces the full point set for backend admin or batch import.
- `POST /api/v1/maps/poses/save` upserts semantic delivery points into the selected `maps/<map_id>/poses.yaml` and synchronizes `current/poses.yaml` when that map is active.
- `POST /api/v1/maps/poses/save_current` writes a semantic point using the same fresh `map -> base_link` pose as `/api/v1/robot/pose`, so the App does not convert pixels to metric coordinates for live marking.
- `GET /api/v1/maps/filters/keepout` reads the keepout mask asset metadata plus App-authored keepout semantic JSON.
- `POST /api/v1/maps/filters/keepout/save` stores App-authored keepout semantic JSON beside the map bundle and synchronizes `current/filters/` for the active map.
- `GET /api/v1/navigation/pre_goal_check` returns the read-only dock/contact gate that will be used before a normal point navigation goal. It resolves `pose_id` from `poses.yaml` when provided, checks direct `x/y/yaw` map-frame goals when provided, reports the critical Nav2 lifecycle summary, and never calls `/docking/undock` or sends a Nav2 action.
- `POST /api/v1/navigation/goal` resolves a saved pose or direct map-frame pose, waits for a fresh `map -> odom -> base_link` TF chain, sends a Nav2 `NavigateToPose` goal, and tracks the goal in a background job exposed as `navigation_goal`. If the robot is docked or live charging contact is detected, the API first performs `/docking/undock`, triggers post-undock relocalization after odometry confirms departure, waits for that result to be accepted by `robot_localization_bridge`, waits for the live TF stream to settle, and only then sends the Nav2 goal. Success and admission-failure responses include `pre_navigation_undock`, `pre_navigation_undock_detail`, and `pre_navigation_dock_check`.
- Dock/contact detection for navigation admission is explicit, not position-based. In addition to BMS and `/docking/status`, the server reads `docking_contact_latch.json`, written by BMS contact, docking success, manual maintenance confirmation, and undock success. `pre_navigation_dock_check.dock_contact_snapshot`, `docked_state_class`, `docked_evidence`, and `docked_warnings` expose this state. If it indicates confirmed or latched docked, normal navigation must undock first; final yaw alignment is blocked instead of rotating on the charger.
- `POST /api/v1/navigation/cancel` accepts a background cancel job, publishes zero velocity immediately, uses a short action-server probe for responsive cancel behavior, then cancels the active Nav2 goal while keeping the resident Nav2/localization runtime alive by default. Use `POST /api/v1/navigation/stop` or `/api/v1/navigation/stop_runtime` only for diagnostics or recovery when the runtime must be torn down.
- `GET /api/v1/navigation/state` returns the latest background navigation cancel job and `navigation_goal` state for App polling and diagnostics. After Nav2 returns, normal point goals are verified against a fresh `map -> base_link` pose. If position is reached and yaw error exceeds `navigation_final_yaw_align_trigger_rad`, the API performs a bounded mission-layer in-place yaw alignment, then verifies the final pose again. If the position remains reached but the heading correction is blocked, `navigation_goal.state` stays `succeeded` with `phase: "position_reached_yaw_warning"` and `final_yaw_align_blocked: true`.
- `POST /api/v1/docking/start` resolves a saved dock contact pose, prefers a manual pre-dock pose (`predock_pose_id`, `approach_pose_id`, or the `dock_id_predock` naming convention), falls back to a geometric pre-dock offset only when no manual point exists, waits for a fresh TF chain after each relocalization gate, sends Nav2 to that pose, relocalizes again after the pre-dock goal succeeds, verifies that the fresh `map -> base_link` pose is still near the pre-dock approach pose, then calls `/docking/start` for GS2 fine docking. When fine docking reports success, failure, or stop, the API triggers another localization pass before it returns to the final docking state so Nav2 does not resume from a pose accumulated only through non-Nav2 fine-docking motion.
- `POST /api/v1/docking/undock` accepts the App's undock intent only when the robot is docked, live charging contact is detected/inferred from BMS status, or the explicit dock-contact latch indicates a manually confirmed docked state, then calls `/docking/undock`; after odometry-confirmed departure it triggers `/global_localization/trigger`, arms `/robot_localization_bridge/force_accept_next_localization`, waits for `map -> base_link` to reflect the localization result, and exposes the result in docking state. The App must not publish reverse velocity directly.
- `POST /api/v1/docking/confirm_docked` and `POST /api/v1/docking/clear_docked_latch` are maintenance recovery endpoints. They only set or clear the persistent docked latch and never send velocity.
- `POST /api/v1/docking/cancel` / `POST /api/v1/docking/stop` cancels the pre-dock Nav2 goal, publishes zero velocity, and calls `/docking/stop` using the configured docking stop service wait.
- `POST /api/v1/navigation/goal` checks the critical Nav2 lifecycle nodes and uses a when-needed relocalization gate. Confirmed same-map runtime context plus a fresh `map -> base_link` pose skips `/global_localization/trigger`; cold or unconfirmed runtime context, stale/missing map-frame pose, controlled undock, docking transitions, or explicit `force_relocalize` still arms the bridge force-accept service, triggers `/global_localization/trigger`, waits for a fresh `/localization_result` to be accepted by `robot_localization_bridge`, and only then sends the Nav2 goal. If the lifecycle is inactive or required localization cannot refresh, the goal is rejected instead of navigating from a stale odometry-only map pose.
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

## Floor Switch And Navigation Resume

`POST /api/v1/floors/switch` with `resume_navigation=false` is a selection-only asset switch request to `/floor_manager/switch_floor`. It validates and records the requested floor assets for the next navigation start, but it does not require `/map_server`, Isaac localization, or Nav2 to already be running.

With `resume_navigation=true`, the API server starts:

```text
scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh <building_id> <floor_id>
```

That runtime validates `maps_release/<building_id>/<floor_id>/current` when present, starts the occupancy localization stack with `current/nav/nav_map.yaml` and `current/localizer/localizer_params.yaml`, starts the `robot_global_localization` service wrapper, waits for `/global_localization/apply_floor_assets`, `/global_localization/trigger`, `/map`, `/flatscan`, and then Isaac `/trigger_grid_search_localization`, calls `/floor_manager/switch_floor`, rechecks and explicitly triggers `/global_localization/trigger`, waits for `map -> odom`, and runs standard Nav2 with the active floor filter masks. Before launching Isaac localization, the runtime uses canonical `/lidar_points`; if it misses fresh point clouds across the startup window, it repairs the JT128 driver/remap chain with the navigation profile and waits again instead of exiting immediately. If the same floor/map already has `map -> odom`, a resized global costmap, and active Nav2 lifecycle nodes, the runtime reuses the existing stack. Repeated same-map resume requests are idempotent once the runtime context is confirmed `ready`: the API returns `navigation_runtime_reused` and leaves the current process group alive. When a map is selected for navigation resume, the API writes `maps_release/last_navigation_map.json`; boot autostart uses that file and the matching `current/manifest.json` to resume the last selected map. On a cold start, Nav2 can be prestarted after `/map` is available so lifecycle startup overlaps with Isaac relocalization, but readiness is still confirmed only after `map -> odom` and the global costmap are available. Service and node readiness checks use direct `ros2 service type` / `ros2 node info` probes before falling back to list commands, so ROS daemon discovery lag under startup load does not falsely abort navigation. The requested map confirmation also falls back to live `/map` metadata and waits several seconds for transient-local map discovery under Jetson startup load. It observes `/localization_result` when available, but does not fail solely because that one-shot result was published before the script subscribed. This is the App path for "switch map and resume navigation"; the App still does not start Nav2 lifecycle directly. When the runtime context reaches confirmed `ready`, `/api/v1/status` and `/api/v1/navigation/state` report navigation as `running`; after an API-server restart this is recovered from the confirmed context plus a ready `/navigate_to_pose` action server. If the navigation resume child process exits before ready, they mark navigation as failed instead of leaving the App stuck in `starting`.

## WebSocket Teleop

`/ws/v1/teleop` is for App-driven low-speed mapping teleop only. It is accepted only while the 2D mapping chain is active by default. It publishes `geometry_msgs/Twist` to `/cmd_vel_collision_checked`, so the command still flows through `robot_safety`, `/cmd_vel_safe`, and the Ranger Mini 3 mode controller before reaching the chassis.

Client messages:

```json
{"type":"cmd_vel","linear_x":0.20,"angular_z":0.10}
{"type":"cmd_vel","linear_x":-0.12,"angular_z":0.00}
{"type":"stop"}
```

Limits are enforced by server parameters:

- `teleop_max_linear_x_mps`: default `0.30`
- `teleop_max_angular_z_radps`: default `0.55`
- `teleop_allow_reverse`: default `true` for mapping teleop only
- `teleop_require_mapping_active`: default `true`
- `teleop_watchdog_timeout_sec`: default `0.5`

During a WebSocket teleop session, the server publishes `/ranger_mini3/teleop_allow_reverse=true` so the mode controller accepts low-speed reverse commands. When the WebSocket disconnects, times out, or receives `stop`, the server publishes one zero `Twist` and disables teleop reverse again. When no teleop session is active, the server does not publish reverse-disable messages, so docking undock permissions remain independent. `robot_safety` still provides the final watchdog stop.

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

Within the map bundle, nav files use a safe filename stem derived from `map_name`, for example `nav/<safe_map_name>.yaml/.pgm`; Isaac localization uses `localizer/<safe_map_name>.png`. `manifest.json` records `map_id`, `display_name`, `building_id`, `floor_id`, asset paths, `created_at`, and `active`.

`GET /api/v1/maps` returns `floor_maps[]` for real map records and keeps `floors[]` only as a compatibility floor/current-map view. `floors[]` should not be treated as the map list. Each map entry includes `map_info` parsed from the Nav2 map YAML and image header so App-side taps can be converted to real map coordinates without assuming PNG scale:

```json
{
  "map_id": "map_20260520T120000Z_012345abcd",
  "display_name": "Lobby delivery map",
  "map_info": {
    "width": 242,
    "height": 103,
    "resolution": 0.05,
    "origin": [-3.2, -1.8, 0.0]
  }
}
```

`POST /api/v1/floors/switch` accepts `map_id` or `map_name`; `map_id` is preferred. It activates the selected manifest into `current/` before running floor switch or navigation resume.

Activation treats `maps_release/<building_id>/<floor_id>/current/` as a backend-owned runtime mirror. If an older dashboard or root-run process left `current/` as a non-empty real directory, the API first attempts normal removal, then safely quarantines the stale directory inside the same floor folder before creating a fresh runtime mirror. App clients should not write into `current/` directly.

`POST /api/v1/maps/delete` deletes only by `map_id`:

```json
{"map_id":"map_20260520T120000Z_012345abcd"}
```

The endpoint refuses building/floor-only deletion so a phone client cannot accidentally remove an entire building or floor asset tree.

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

Save keepout edit semantics:

```http
POST /api/v1/maps/filters/keepout/save
```

```json
{"building_id":"B1","floor_id":"F1","map_id":"map_20260520T120000Z_012345abcd","keepout":{"lines":[]}}
```

The backend stores the submitted JSON at `maps/<map_id>/filters/keepout_semantic_layer.json` and mirrors it to `current/filters/keepout_semantic_layer.json` when the map is active. Runtime Nav2 still consumes `filters/keepout_mask.yaml` and `filters/keepout_mask.pgm`.

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

The API server reads `maps_release/<building_id>/<floor_id>/poses.yaml` and sends a `NavigateToPose` action to `/navigate_to_pose`. Direct map-frame goals are also accepted with `x`, `y`, and `yaw`, but phone clients should normally use `pose_id` so the car remains the source of truth for floor assets. Goal and cancel calls are serialized around the Nav2 action client; transient rclcpp action-client exceptions are returned as HTTP errors or logged without taking down port 8080. All HTTP handlers are wrapped by a request-level exception guard, so individual ROS service/action/file failures return JSON errors instead of aborting the API process. Successful acceptance returns `navigation_goal_id`; completion is reported by the background `navigation_goal` object in `/api/v1/status` and `/api/v1/navigation/state`.

Delivery success is position-first. Nav2 action success is not accepted by itself: the API always rechecks the fresh `map -> base_link` pose against `navigation_goal_position_success_tolerance_m` (`0.20 m` by default). If position is reached and yaw error is above `navigation_final_yaw_align_trigger_rad` (`0.08 rad`), the API runs a 15 Hz mission-layer final yaw alignment with `kp=1.2`, `min=0.06 rad/s`, `max=0.25 rad/s`, `timeout=8.0 s`, and `max_xy_drift=0.08 m`, then rechecks the fresh map pose. Yaw error at or below `navigation_final_yaw_tolerance_rad` (`0.05 rad`) is fully verified; yaw between tolerance and trigger is treated as a no-spin deadband. If only the last heading correction is blocked but the final position remains inside tolerance, the job reports success with `phase: "position_reached_yaw_warning"` and `final_yaw_align_blocked: true`. This is not implemented by tightening Nav2's yaw goal checker. The first field version publishes final yaw commands to `/cmd_vel_collision_checked`, so `robot_safety` remains the final arbiter; this intentionally exposes `final_yaw_align_bypass_collision_monitor=true` until a dedicated `/cmd_vel_nav` mux is introduced. A missed position remains a failed navigation goal, even if Nav2 falsely reports success.

The `navigation_goal` JSON exposes final-yaw diagnostics for the App and field logs: `final_pose_verified`, `final_pose_verify_reason`, `final_yaw_align_attempted`, `final_yaw_align_blocked_reason`, `final_yaw_align_duration_sec`, `final_yaw_align_timeout_sec`, `final_yaw_align_target_yaw_rad`, `final_yaw_align_initial_yaw_error_rad`, `final_yaw_align_final_yaw_error_rad`, `final_yaw_align_max_xy_drift_m`, `final_yaw_align_observed_xy_drift_m`, `final_yaw_align_cmd_topic`, and `final_yaw_align_bypass_collision_monitor`.

When the backend state is `docked`, `/docking/status` starts with `docked` or `charging`, the persistent dock-contact latch is active, or live BMS charging contact is fresh, `/api/v1/navigation/goal` automatically performs controlled undocking before sending the Nav2 goal. The same snapshot is available through read-only `GET /api/v1/navigation/pre_goal_check` and through `/api/v1/status` / `/api/v1/docking/state` as `pre_navigation_dock_check`. The read-only endpoint keeps the existing `would_auto_undock` field and also exposes `auto_undock_required`. The snapshot exposes `api_bms_charging_contact`, `api_bms_charging_contact_reason`, `dock_contact_snapshot`, `docked_state_class`, `docked_evidence`, `docked_warnings`, `bms.power_supply_status`, `bms.current`, `docking.last_status`, `final_is_docked_or_charging`, `final_auto_undock_required`, and `auto_undock_reason`, so a full battery with `current=0` is still diagnosable through `POWER_SUPPLY_STATUS_FULL` or the BMS contact reason. If BMS reports no contact because the charger signal is missing or the robot was manually pushed onto the dock, maintenance can call `POST /api/v1/docking/confirm_docked`; this writes only the latch and sends no velocity. `POST /api/v1/docking/clear_docked_latch` clears only that latch and also sends no velocity. If `robot_docking_manager` has just been started and has not received `/battery_state` yet, the API retries `/docking/undock` for `docking_undock_charging_retry_sec` while the API-side charging contact remains fresh. After `/docking/status` reports odometry-confirmed `undocked`, the API arms `/robot_localization_bridge/force_accept_next_localization`, triggers `/global_localization/trigger`, and waits until the fresh localization result is reflected in `map -> base_link`; if undocking, post-undock relocalization, or bridge acceptance fails or times out, the navigation goal is not sent. Goal responses include `pre_navigation_undock`, `pre_navigation_undock_detail`, and `pre_navigation_dock_check` so the App can show whether departure from the charger happened inside the request.

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

By default this is a task cancel, not a process stop. The HTTP handler publishes a zero burst to `/cmd_vel_collision_checked`, creates a background cancel job, and returns `202 Accepted` quickly. The background job uses `navigation_cancel_action_wait_sec` as a short `/navigate_to_pose` action-server probe, cancels the cached API-started `NavigateToPose` goal handle when the action server is available, sends cancel-all to `/navigate_to_pose`, publishes another zero burst, and leaves the resident Nav2/localization runtime alive.

The endpoint remains idempotent for task exit: if the Nav2 action server is already gone, it still publishes zero velocity and records the cancel job failure reason without tearing down localization.

For diagnostics or recovery, call `POST /api/v1/navigation/stop` or `POST /api/v1/navigation/stop_runtime`. These endpoints force the same background cancel job with `stop_stack=true`: they cancel any Nav2 goal, publish zero velocity, terminate the resident Nav2/localization runtime, clear `/tmp/njrh_runtime_map_context.json`, and verify that no Nav2/localizer/`robot_localization_bridge` processes remain. Startup failure before confirmed `ready` also rolls back both Nav2 and the occupancy-localization helper layer so the next resume starts from a clean process set. The App's return-to-charger action should not call these endpoints first; it should call `/api/v1/docking/start` directly so the backend can switch modes without stopping Nav2 and localization.

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

The backend activates `map_id` when provided, waits for or starts the normal navigation runtime, cancels any cached API navigation goal without stopping the Nav2/localization stack, triggers `/global_localization/trigger`, waits for a fresh `/localization_result` or a very recent existing result, then sends Nav2 to the pre-dock pose. Manual pre-dock poses are preferred: pass `predock_pose_id`/`approach_pose_id`, or save a pose named `dock_id_predock`, `dock_id_pre_dock`, `dock_id_approach`, `predock_dock_id`, `pre_dock_dock_id`, or `approach_dock_id`. A single pose with type `dock_predock`, `predock`, or `dock_approach` is also accepted. Manual pre-dock poses pass by yaw sanity only by default: `docking_manual_predock_distance_check_enable` is `false`, so the configured distance range is diagnostic/optional and short points such as `0.356m` are not rejected only because they are close to the dock contact. `docking_manual_predock_max_yaw_error_rad` still rejects a point whose heading is clearly not aligned to the charger. If no manual pre-dock pose is present, the backend falls back to the configured geometric offset from the saved dock contact pose. The docking state exposes `predock_pose_id` and `approach_source` so the App can show whether the target came from a manual point or from geometry. After Nav2 reaches the pre-dock pose, the API relocalizes and checks the resulting `map -> base_link` pose against the approach pose before starting GS2 fine docking. If the post-predock pose is outside `docking_predock_pose_max_distance_m` or `docking_predock_pose_max_yaw_rad`, the job fails instead of blindly entering fine docking from a bad pose. GS2 fine docking still owns the final low-speed contact motion and charging detection. When it terminates with `docked`, `dock_feature_not_found`, `contact_verify_timeout`, or another terminal status, the API state becomes `relocalize_after_fine_docking` while `/global_localization/trigger` refreshes `map -> odom`, then the job finishes as `docked`, `failed`, `stopped`, or `canceled`.

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

The undock endpoint is accepted only when the backend state is already docked, the API sees live charging contact, or the persistent dock-contact latch is explicitly set. It starts `robot_docking_manager` if needed, calls `/docking/undock`, and tracks the job as `undocking` until `/docking/status` reports `undocked` or `undock_failed...`. `robot_docking_manager` reads the same latch before accepting `/docking/undock`, so `pre_navigation_dock_check` must not claim `can_auto_undock=true` unless the controlled undock service will accept that dock evidence. Once odometry confirms departure, the API enters `relocalize_after_undock`, arms `/robot_localization_bridge/force_accept_next_localization`, calls `/global_localization/trigger`, verifies that the resulting localization is reflected in `map -> base_link`, and records `post_undock_relocalization_*` fields in the docking job. Manual undock remains `undocked` even if the relocalization warning needs operator attention; auto-undock before `/api/v1/navigation/goal` treats relocalization failure as a blocker and does not send the Nav2 goal. Reverse motion remains inside the normal safety chain: `robot_docking_manager -> /cmd_vel_docking -> robot_safety -> /cmd_vel_safe -> ranger_mini3_mode_controller -> /cmd_vel`. The retained undock speed is `0.06 m/s`; first-motion delay is handled by `undock.motion_start_timeout_s`, while `undock.no_progress_timeout_s` is reserved for a stall after movement has started.

`accepted=true` is an API admission result, not a synonym for the underlying ROS Trigger result. The undock response and `GET /api/v1/docking/state` expose `api_accepted`, `already_running`, `docking_service_called`, `docking_service_success`, `docking_service_message`, `docking_status_at_request`, `docking_status_after_request`, `undock_started_observed`, `undock_cmd_count_observed`, `undock_failure_reason`, and `docking_service_warning` so field diagnostics can distinguish API admission, `/docking/undock` service success, `/docking/status` observation, and downstream motion execution.

Relevant states are `accepted`, `relocalize_before_predock`, `nav_to_predock`, `relocalize_after_predock`, `fine_docking`, `relocalize_after_fine_docking`, `docked`, `undocking`, `relocalize_after_undock`, `undocked`, `failed`, and `canceled`.

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
