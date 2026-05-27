# robot_api_server

`robot_api_server` is the production-facing HTTP gateway for Android and other non-ROS clients.

It does not own mapping, localization, navigation, or chassis control logic. It only exposes a narrow HTTP API and forwards requests into existing ROS 2 topics and services.

## Boundaries

- Safety stop and resume publish `std_msgs/Bool` to `/safety/estop`.
- Robot battery state is read from Ranger's `/battery_state` (`sensor_msgs/BatteryState`) and exposed as `bms.soc` in `/api/v1/status`.
- Floor switching calls `/floor_manager/switch_floor` when `resume_navigation=false`.
- Floor switching with `resume_navigation=true` starts the repository-owned floor navigation runtime, which loads the requested floor assets, starts localization, and starts standard Nav2.
- Localization trigger calls `/global_localization/trigger`.
- Map listing reads released floor assets and runtime flat map files.
- `POST /api/v1/mapping/2d/start` starts the repository-owned `slam_toolbox` 2D mapping runtime chain.
- `POST /api/v1/mapping/2d/stop` terminates the App-started 2D mapping chain without stopping common services.
- `POST /api/v1/mapping/2d/save` saves the current `slam_toolbox` occupancy grid as flat runtime assets plus a structured floor bundle, then terminates the mapping chain.
- `POST /api/v1/mapping/stop` and `POST /api/v1/mapping/save` are REST aliases for the same 2D mapping stop/save operations.
- `POST /api/v1/maps/delete` deletes saved map assets using the same `map_name` and `building_id` / `floor_id` naming rules as save.
- `GET /api/v1/robot/pose` returns only a fresh `map -> base_link` TF pose. It never falls back to `/odom` or wheel odom.
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
- `POST /api/v1/navigation/goal` resolves a saved pose or direct map-frame pose and sends a Nav2 `NavigateToPose` goal.
- `POST /api/v1/navigation/cancel` accepts a background cancel job, publishes zero velocity immediately, then cancels Nav2 and stops the Nav2 plus 2D localization runtime while keeping common services alive.
- `GET /api/v1/navigation/state` returns the latest background navigation cancel job state for App polling and diagnostics.
- `POST /api/v1/docking/start` resolves a saved dock pose, computes a pre-dock approach pose, sends Nav2 to that pose, then calls `/docking/start` for GS2 fine docking.
- `POST /api/v1/docking/cancel` / `POST /api/v1/docking/stop` cancels the pre-dock Nav2 goal, publishes zero velocity, and calls `/docking/stop`.
- `GET /api/v1/docking/state` returns the latest docking job and `/docking/status` state.
- `GET /api/v1/status` includes HTTP active/max connection counters; the server rejects excess clients with `503` instead of spawning unbounded detached threads.
- `POST /api/v1/subscriptions/acquire|heartbeat|release` controls page-scoped ROS subscriptions for App resources.
- WebSocket teleop keeps a permanent `/battery_state` guard and publishes zero velocity when charging/full or charge current is detected.
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
- `POST /api/v1/navigation/goal`
- `POST /api/v1/navigation/cancel`
- `GET /api/v1/docking/state`
- `POST /api/v1/docking/start`
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

The App should treat `mode`, `state`, `mapping_active`, `navigation_active`, `healthy`, and `message` as the lightweight business-state contract. These fields are maintained by API transitions rather than by asking the App to inspect ROS nodes or topics directly.

`soc` is normalized to `0..100`. Ranger currently publishes `/battery_state.percentage` as a percent value; if a future driver follows the ROS convention `0.0..1.0`, the API converts it to percent.

## Subscription Manager

High-rate or page-specific ROS inputs are not kept subscribed permanently. The App must acquire resources when entering a page, heartbeat every 3-5 seconds, and release them on exit:

```json
{"client_id":"app_device_001","resources":["status","live_map","tf"],"ttl_ms":10000}
```

For compatibility with older App/Web builds, the server also accepts `clientId`, `lease_id`, `leaseId`, `subscription_id`, and `subscriptionId`. If no client identifier is provided, the request uses the compatibility lease `http:compat-default` instead of returning HTTP 400. Heartbeats may omit `resources`; the server refreshes the client's existing resources, or returns a successful no-op if none are currently leased.

Resources:

- `status`: `/safety/status`, `/safety/motion_allowed`, `/floor_manager/status`, `/battery_state`
- `live_map`: live `slam_toolbox` `/map` cache used by `GET /api/v1/mapping/2d/map` and save
- `scan`: `/scan` cache hook for laser-layer views
- `tf`: `/tf` cache for `map -> odom -> base_link` pose
- `teleop`: WebSocket mapping teleop lifecycle

When a resource refcount changes `0 -> 1`, the ROS subscription is created. When it changes `1 -> 0`, the ROS subscription is reset and high-frequency cache is cleared. Leases expire automatically after `ttl_ms`; default is `10000`.

`GET /api/v1/mapping/2d/map` only serves live `/map` when `live_map` is currently acquired. Saved map preview through `?source=saved` or `?name=<map>` does not require `live_map`.

## Floor Switch And Navigation Resume

`POST /api/v1/floors/switch` with `resume_navigation=false` is a selection-only asset switch request to `/floor_manager/switch_floor`. It validates and records the requested floor assets for the next navigation start, but it does not require `/map_server`, Isaac localization, or Nav2 to already be running.

With `resume_navigation=true`, the API server starts:

```text
scripts/jetson/runtime_overlay/scripts/run_floor_navigation.sh <building_id> <floor_id>
```

That runtime validates `maps_release/<building_id>/<floor_id>/current` when present, starts the occupancy localization stack with `current/nav/nav_map.yaml` and `current/localizer/localizer_params.yaml`, starts the `robot_global_localization` service wrapper, waits for `/global_localization/apply_floor_assets`, `/global_localization/trigger`, Isaac `/trigger_grid_search_localization`, `/map`, and `/flatscan`, calls `/floor_manager/switch_floor` to load assets and trigger Isaac localization, then treats `map -> odom` as the authoritative localization-ready signal before starting standard Nav2 with the active floor filter masks. It observes `/localization_result` when available, but does not fail solely because that one-shot result was published before the script subscribed. This is the App path for "switch map and resume navigation"; the App still does not start Nav2 lifecycle directly.

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

During a WebSocket teleop session, the server publishes `/ranger_mini3/allow_reverse=true` so the mode controller accepts low-speed reverse commands. When the WebSocket disconnects, times out, or receives `stop`, the server publishes one zero `Twist` and disables reverse again. `robot_safety` still provides the final watchdog stop.

## 2D Map PNG

`POST /api/v1/mapping/2d/start` starts the same repository-owned `slam_toolbox` chain used by the debug Web 2D mapping path.

`POST /api/v1/mapping/2d/stop` stops the 2D mapping-side chain and returns `{"ok":true,"mapping_active":false}`. The stop scope includes `run_projected_map.sh`, `slam_toolbox`, scan preprocessing / republishing nodes, and the FAST-LIO2 deskew source used by mapping (`fastlio_mapping` / `laser_mapping`). It keeps chassis, canonical TF, local state, safety, and the API server running.

`POST /api/v1/mapping/2d/save` accepts `building_id`, `floor_id`, and a business `map_name`. The server generates a stable `map_id`, preserves the original name as `display_name`, writes the map bundle under `maps_release/<building_id>/<floor_id>/maps/<map_id>/`, activates it through `maps_release/<building_id>/<floor_id>/current/`, and then runs the same mapping-chain stop logic as `/mapping/2d/stop`. Runtime consumers keep reading fixed role files from `current/nav/nav_map.yaml` and `current/localizer/localizer_map.png`.

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

The API server reads `maps_release/<building_id>/<floor_id>/poses.yaml` and sends a `NavigateToPose` action to `/navigate_to_pose`. Direct map-frame goals are also accepted with `x`, `y`, and `yaw`, but phone clients should normally use `pose_id` so the car remains the source of truth for floor assets. Goal and cancel calls are serialized around the Nav2 action client; transient rclcpp action-client exceptions are returned as HTTP errors or logged without taking down port 8080. All HTTP handlers are wrapped by a request-level exception guard, so individual ROS service/action/file failures return JSON errors instead of aborting the API process.

Cancel requests are wired to Nav2, not only acknowledged over HTTP:

```http
POST /api/v1/navigation/cancel
```

By default this is a navigation-mode stop, not just a goal cancel. The HTTP handler publishes a zero burst to `/cmd_vel_collision_checked`, creates a background cancel job, and returns `202 Accepted` quickly. The background job cancels the cached API-started `NavigateToPose` goal handle when `/navigate_to_pose` is available, sends cancel-all to `/navigate_to_pose`, publishes another zero burst, and runs `scripts/jetson/runtime_overlay/scripts/stop_floor_navigation.sh` to stop Nav2, map server, Isaac occupancy localization, scan conversion, and `robot_localization_bridge`. Common services such as the JT128 driver, chassis driver, `robot_safety`, local odom, robot description, local perception, floor manager, and the API server remain alive.

The endpoint remains idempotent for mode exit: if the Nav2 action server is already gone, it still publishes zero velocity and stops any remaining navigation/localization processes.

For diagnostics only, pass `"stop_stack": false` to cancel the active Nav2 goal without tearing down the navigation/localization runtime.

Poll the latest cancel job with:

```http
GET /api/v1/navigation/state
```

`GET /api/v1/mapping/2d/map` returns `Content-Type: image/png` for the live `slam_toolbox` `/map` cached after that App-started mapping session begins. It does not return a map_server static map or historical saved map by default.

## Docking

Docking is a two-stage backend workflow. The App stores a dock pose in the same `poses.yaml` mechanism as delivery points, normally with `type: "dock"`. That pose represents the final robot `base_link` pose when the front charging contacts are aligned with the physical dock. The API computes the pre-dock pose by backing up from that final pose along its yaw by `docking_pre_dock_distance_m` (default `0.80 m`).

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

The backend activates `map_id` when provided, waits for or starts the normal navigation runtime, sends Nav2 to the computed pre-dock pose, then starts `robot_docking_manager` if `/docking/start` is not already available and calls that service. GS2 fine docking still owns the final low-speed contact motion and charging detection.

Cancel docking:

```http
POST /api/v1/docking/cancel
```

This cancels the cached pre-dock Nav2 goal, calls `/docking/stop`, and publishes a zero velocity burst through the existing safe command path. The App should poll:

```http
GET /api/v1/docking/state
GET /api/v1/status
```

Relevant states are `accepted`, `nav_to_predock`, `fine_docking`, `docked`, `failed`, and `canceled`.

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
