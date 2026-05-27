# Android App API Gateway

The Android app should connect to the robot hotspot and talk to `robot_api_server` over HTTP plus WebSocket. The app should not join ROS 2 DDS directly.

## Network

- Robot hotspot address: use the Jetson hotspot IP or static LAN IP.
- Default API port: `8080`.
- Use `X-Robot-Token` when `api_token` is configured.

## Production Boundary

`robot_api_server` is a gateway. It forwards requests to ROS 2 topics and services that already own robot behavior:

- Safety: `/safety/estop`
- Floor switch: `/floor_manager/switch_floor`
- Localization trigger: `/global_localization/trigger`
- Status: `/safety/status`, `/safety/motion_allowed`, `/floor_manager/status`, Ranger `/battery_state`

It must not bypass `robot_safety`, publish chassis commands directly, or call the test dashboard as a backend.

## Current Endpoints

```text
GET  /api/v1/status
GET  /api/v1/robot/pose
GET  /api/v1/maps
GET  /api/v1/maps/semantic_layer
GET  /api/v1/maps/poses
GET  /api/v1/maps/filters/keepout
GET  /api/v1/mapping/2d/map
GET  /api/v1/openapi
POST /api/v1/mapping/2d/start
POST /api/v1/mapping/2d/stop
POST /api/v1/mapping/2d/save
POST /api/v1/subscriptions/acquire
POST /api/v1/subscriptions/release
POST /api/v1/subscriptions/heartbeat
POST /api/v1/mapping/stop
POST /api/v1/mapping/save
POST /api/v1/maps/delete
POST /api/v1/maps/poses
PUT  /api/v1/maps/poses/{pose_id}
DELETE /api/v1/maps/poses/{pose_id}
PUT  /api/v1/maps/poses/batch
POST /api/v1/maps/poses/save
POST /api/v1/maps/poses/save_current
POST /api/v1/maps/filters/keepout/save
POST /api/v1/safety/stop
POST /api/v1/safety/resume
POST /api/v1/floors/switch
POST /api/v1/localization/trigger
GET  /api/v1/navigation/state
POST /api/v1/navigation/goal
POST /api/v1/navigation/cancel
GET  /api/v1/docking/state
POST /api/v1/docking/start
POST /api/v1/docking/cancel
POST /api/v1/docking/stop
WS   /ws/v1/teleop
```

Example floor switch body:

```json
{
  "building_id": "building_1",
  "floor_id": "floor_1",
  "resume_navigation": false
}
```

Reserved navigation start and 3D mapping endpoints return `501` until `robot_mode_manager` or `robot_mission_manager` exposes formal ROS-native services/actions. 2D mapping is wired to the repository-owned `slam_toolbox` runtime chain, not to the test Web dashboard. Navigation goals are wired to Nav2 `NavigateToPose`; the App should send saved `pose_id` targets instead of publishing velocity commands.

When `POST /api/v1/floors/switch` uses `"resume_navigation": true`, the car starts the floor navigation runtime for that `building_id` and `floor_id`: load the released `nav/nav_map.yaml`, start the occupancy localization stack, start the `robot_global_localization` wrapper, wait for `/global_localization/apply_floor_assets`, `/global_localization/trigger`, Isaac `/trigger_grid_search_localization`, `/map`, and `/flatscan`, call `/floor_manager/switch_floor` to apply localizer assets and trigger Isaac localization, then starts standard Nav2 with the active floor filter masks. The backend treats `map -> odom` as the authoritative localization-ready signal. `/localization_result` is observed when available, but the backend does not fail solely because that one-shot result was already published before the runtime subscribed. The App still only requests the mode transition; it must not launch Nav2 or send task-navigation velocity commands.

When the request includes `map_id` or `map_name`, the backend activates that saved map into `maps_release/<building_id>/<floor_id>/current/` before floor switch or navigation resume. `current/` is a backend-owned runtime mirror; the App should edit saved map records through map APIs and must not write files under `current/` directly.

## Status / Battery

The App reads lightweight robot business state and live chassis power from:

```text
GET http://<robot-ip>:8080/api/v1/status
```

Relevant response fields:

```json
{
  "ok": true,
  "mode": "MAPPING_2D",
  "state": "running",
  "mapping_active": true,
  "navigation_active": false,
  "healthy": true,
  "message": "",
  "mapping": {
    "active": true,
    "state": "running",
    "map_topic": "/map",
    "map_endpoint": "/api/v1/mapping/2d/map"
  },
  "navigation": {
    "active": false,
    "state": "stopped",
    "action": "/navigate_to_pose"
  },
  "docking_active": false,
  "docking": {
    "active": false,
    "state": "stopped",
    "dock_id": "",
    "status_topic": "/docking/status",
    "last_status": ""
  },
  "safety_status": "OK",
  "motion_allowed": true,
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

The App should use these lightweight business fields instead of inferring state from ROS nodes or topics:

- Mapping: `mode == "MAPPING_2D" && mapping_active == true`.
- Navigation: `mode == "NAVIGATION" && navigation_active == true`.
- Docking: `mode == "DOCKING" || docking.active == true || docking.state in ["accepted","nav_to_predock","fine_docking","docked","failed","canceled"]`.
- Idle: `mode == "IDLE"`.
- Error: `mode == "ERROR" || healthy == false`.

The backend intentionally keeps this status lightweight. It is driven by official API transitions such as `/mapping/2d/start`, `/mapping/2d/stop`, `/mapping/2d/save`, `/floors/switch?resume_navigation=true`, `/navigation/goal`, and `/navigation/cancel`. Deep ROS checks remain backend diagnostics and should not be duplicated in the App.

`bms.soc` is the value the App should display as real battery percentage. The data path is Ranger CAN BMS frame decoded by `ranger_base_node` into `/battery_state`, then subscribed by `robot_api_server`; the App must not read CAN or ROS 2 DDS directly. Treat `soc_valid=false` as stale or unavailable power data.

## Page-Scoped Subscriptions

Before opening a page that needs live robot data, acquire the corresponding resources:

```text
POST http://<robot-ip>:8080/api/v1/subscriptions/acquire
Content-Type: application/json
```

```json
{
  "client_id": "app_device_001",
  "resources": ["status", "live_map", "tf"],
  "ttl_ms": 10000
}
```

Refresh the lease every 3-5 seconds:

```text
POST http://<robot-ip>:8080/api/v1/subscriptions/heartbeat
```

Release resources when leaving the page:

```text
POST http://<robot-ip>:8080/api/v1/subscriptions/release
```

```json
{
  "client_id": "app_device_001",
  "resources": ["live_map", "tf", "teleop"]
}
```

Resource mapping:

- Home / robot detail: acquire `status`.
- Mapping page: acquire `live_map`, `tf`, and `teleop`, then open `WS /ws/v1/teleop`.
- Map editing page: usually does not acquire ROS resources; read saved map assets and semantic layers only.

If the App crashes or network drops, the server expires the lease after `ttl_ms`. WebSocket disconnect immediately releases the internal `teleop` lease and publishes one zero velocity command.

## 2D Mapping Start / Stop

Start live `slam_toolbox` mapping before requesting live map images:

```text
POST http://<robot-ip>:8080/api/v1/mapping/2d/start
```

Successful response:

```json
{
  "ok": true,
  "state": "starting",
  "map_topic": "/map",
  "map_endpoint": "/api/v1/mapping/2d/map"
}
```

Stop live `slam_toolbox` mapping when the App user exits mapping mode:

```text
POST http://<robot-ip>:8080/api/v1/mapping/2d/stop
```

Equivalent alias:

```text
POST http://<robot-ip>:8080/api/v1/mapping/stop
```

Successful response:

```json
{
  "ok": true,
  "mapping_active": false
}
```

The stop endpoint terminates the complete 2D mapping-side chain: `run_projected_map.sh`, the `slam_toolbox` launch/processes, scan preprocessing and republishing nodes, and the FAST-LIO2 deskew source used for live mapping (`fastlio_mapping` / `laser_mapping`). It does not stop canonical chassis, TF, local state, safety, or API services.

Save the current live or just-stopped `slam_toolbox` map into flat runtime preview files plus a business map bundle. A successful save also runs the same mapping-chain stop logic as `/mapping/2d/stop`, so the App does not need to call stop again after saving:

```text
POST http://<robot-ip>:8080/api/v1/mapping/2d/save
Content-Type: application/json
```

Equivalent alias:

```text
POST http://<robot-ip>:8080/api/v1/mapping/save
Content-Type: application/json
```

Request body:

```json
{
  "building_id": "building_1",
  "floor_id": "floor_1",
  "map_name": "test-17"
}
```

Successful response includes:

```json
{
  "ok": true,
  "map_id": "map_20260520T120000Z_012345abcd",
  "display_name": "test-17",
  "map_name": "test-17",
  "mapping_active": false,
  "runtime_map": {
    "yaml": ".../maps/test-17.yaml",
    "pgm": ".../maps/test-17.pgm",
    "png": ".../maps/test-17.png",
    "localizer_yaml": ".../maps/test-17.localizer.yaml",
    "localizer_png": ".../maps/test-17.localizer.png"
  },
  "floor_assets": {
    "root": ".../maps_release/building_1/floor_1/maps/map_20260520T120000Z_012345abcd",
    "current_root": ".../maps_release/building_1/floor_1/current",
    "manifest_json": ".../maps_release/building_1/floor_1/maps/map_20260520T120000Z_012345abcd/manifest.json",
    "nav_map_yaml": ".../maps_release/building_1/floor_1/maps/map_20260520T120000Z_012345abcd/nav/test-17.yaml",
    "nav_map_pgm": ".../maps_release/building_1/floor_1/maps/map_20260520T120000Z_012345abcd/nav/test-17.pgm",
    "localizer_map_png": ".../maps_release/building_1/floor_1/maps/map_20260520T120000Z_012345abcd/localizer/test-17.png"
  }
}
```

The Nav2 `PGM` and Isaac localizer `PNG` are generated from the same cached occupancy grid. The endpoint also creates neutral filter masks, `asset_report.json`, `poses.yaml`, and `manifest.json`. Runtime consumers do not use the business filename directly; activation copies fixed role files into `maps_release/<building_id>/<floor_id>/current/nav/nav_map.yaml` and `current/localizer/localizer_map.png`.

List maps:

```text
GET http://<robot-ip>:8080/api/v1/maps
```

Use `floor_maps[]` as the real map list. Each item includes `map_id`, `display_name`, `map_name`, `building_id`, `floor_id`, `active`, `nav_map_yaml`, and `localizer_map_png`. The legacy `floors[]` array only tells the App which floor exists and which map is currently active.

## Map Delete

Delete one saved business map by stable `map_id`:

```text
POST http://<robot-ip>:8080/api/v1/maps/delete
Content-Type: application/json
```

```json
{"map_id": "map_20260520T120000Z_012345abcd"}
```

Successful response:

```json
{
  "ok": true,
  "deleted": true,
  "map_id": "map_20260520T120000Z_012345abcd",
  "display_name": "test-17",
  "building_id": "building_1",
  "floor_id": "floor_1",
  "active_deleted": false
}
```

The endpoint refuses building-only or floor-only deletion. It does not stop active mapping/navigation stacks, so the App should stop mapping or leave navigation mode before deleting the map currently in use.

## Semantic Points And Navigation

App-created delivery points are map overlays stored in the floor bundle `poses.yaml`. They must not be drawn into `nav/nav_map.pgm` or `localizer/localizer_map.png`.

The App should treat the car backend as the source of truth for semantic map overlays. On map-edit page entry, read the complete semantic layer first:

```text
GET http://<robot-ip>:8080/api/v1/maps/semantic_layer?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

`map_id` is preferred and can be used without relying on Android local cache. Successful response includes `poses[]`, `filters.keepout`, the Nav2 keepout mask asset paths, and any App-authored keepout JSON saved by `POST /api/v1/maps/filters/keepout/save`. If this endpoint is unavailable on an older car build, the App may fall back to `GET /api/v1/maps/poses` and `GET /api/v1/maps/filters/keepout`.

Read only the keepout layer:

```text
GET http://<robot-ip>:8080/api/v1/maps/filters/keepout?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

Save or update App-authored keepout semantics:

```text
POST http://<robot-ip>:8080/api/v1/maps/filters/keepout/save
Content-Type: application/json
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "keepout": {
    "lines": []
  }
}
```

The server stores this JSON beside the map bundle as `filters/keepout_semantic_layer.json` and synchronizes it into `current/filters/` when that map is active. The Nav2 mask files remain `filters/keepout_mask.yaml` and `filters/keepout_mask.pgm`; the semantic JSON is the editable App overlay, not a replacement for the runtime costmap filter asset.

Read existing points from the car:

```text
GET http://<robot-ip>:8080/api/v1/maps/poses?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

`map_id` is the stable selector the App should use. If `map_id` is omitted, the server reads the active map for that floor; `map_name` / `display_name` is only a fallback. Successful response:

```json
{
  "ok": true,
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "display_name": "test-17",
  "map_name": "test-17",
  "active": true,
  "poses": [
    {
      "pose_id": "delivery_123456",
      "id": "delivery_123456",
      "type": "delivery_point",
      "name": "Point A",
      "x": 5.75,
      "y": -0.82,
      "yaw": 1.57
    }
  ]
}
```

Create or upsert one point:

```text
POST http://<robot-ip>:8080/api/v1/maps/poses
Content-Type: application/json
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "pose_id": "delivery_123456",
  "type": "delivery_point",
  "name": "Point A",
  "x": 5.75,
  "y": -0.82,
  "yaw": 1.57
}
```

Update one known point by URL path:

```text
PUT http://<robot-ip>:8080/api/v1/maps/poses/delivery_123456
Content-Type: application/json
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "name": "Point A",
  "x": 5.75,
  "y": -0.82,
  "yaw": 1.57
}
```

Delete one point:

```text
DELETE http://<robot-ip>:8080/api/v1/maps/poses/delivery_123456?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

Replace the full point set for a map, used by admin tools or batch import:

```text
PUT http://<robot-ip>:8080/api/v1/maps/poses/batch
Content-Type: application/json
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "poses": [
    {"pose_id": "delivery_123456", "type": "delivery_point", "name": "Point A", "x": 5.75, "y": -0.82, "yaw": 1.57}
  ]
}
```

`PUT /api/v1/maps/poses/batch` is a replacement operation, not an append. Sending `"poses":[]` intentionally clears all App semantic points for that map. The backend writes the selected `maps/<map_id>/poses.yaml` and synchronizes `current/poses.yaml` when that map is active.

Legacy save-or-update endpoint kept for existing App builds:

```text
POST http://<robot-ip>:8080/api/v1/maps/poses/save
Content-Type: application/json
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "pose_id": "delivery_123456",
  "type": "delivery_point",
  "name": "Point A",
  "x": 5.75,
  "y": -0.82,
  "yaw": 1.57
}
```

If `map_id` is omitted, the server writes to the active map for that floor. If the map is active, the server also synchronizes `current/poses.yaml`.

Preview the robot's current map-frame pose before live marking:

```text
GET http://<robot-ip>:8080/api/v1/robot/pose
```

The backend reads TF and requires a fresh `map -> base_link` pose. It does not use `/odom`, wheel odom, or PNG pixel projection as a fallback. A successful response is flat map-frame robot pose data:

```json
{
  "ok": true,
  "frame_id": "map",
  "child_frame_id": "base_link",
  "x": 1.23,
  "y": 4.56,
  "yaw": 1.5708,
  "stamp": 1770000000.123,
  "age_sec": 0.05,
  "map_id": "map_20260520T120000Z_012345abcd",
  "floor_id": "F1",
  "building_id": "B1"
}
```

If this returns `503`, the App should tell the operator to start mapping, navigation, or localization before saving a live point. The error body is:

```json
{"ok":false,"error":"no fresh map-frame robot pose","frame_id":"map","child_frame_id":"base_link","age_sec":null}
```

`map_id`, `floor_id`, and `building_id` are returned only from the robot's confirmed runtime map context. During navigation startup or floor switching, the backend may already have a fresh TF pose but still return `503` if the requested map has not been confirmed by localization and Nav2 readiness. The App must treat that as a real backend state mismatch, not silently reuse an older map context.

Save the robot's current position as a point:

```text
POST http://<robot-ip>:8080/api/v1/maps/poses/save_current
Content-Type: application/json
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "type": "delivery_point",
  "name": "Current Point"
}
```

The App should use this endpoint for "mark point at current robot position". Do not convert PNG pixels to map coordinates for this workflow. The server fills `x`, `y`, and `yaw` from the same live `map -> base_link` pose returned by `/api/v1/robot/pose`; if `/robot/pose` would return `503`, `save_current` also returns `503` and does not save. Request-body `yaw` or `theta` is ignored so saved orientation always equals the live robot heading. `type: "dock"` means the final charging-contact `base_link` pose, not the pre-dock point. The server writes `maps/<map_id>/poses.yaml` and synchronizes `current/poses.yaml` if that map is active. `pose_id` is optional; when omitted, the server generates a stable ID and returns it.

Send a saved point as a real navigation goal:

```text
POST http://<robot-ip>:8080/api/v1/navigation/goal
Content-Type: application/json
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "pose_id": "delivery_123456"
}
```

Successful response returns `202 Accepted`. The server resolves the pose from `maps_release/<building_id>/<floor_id>/poses.yaml` and sends a Nav2 `NavigateToPose` goal on `/navigate_to_pose`. The App must not send `/cmd_vel` for task navigation.

Cancel active navigation goals:

```text
POST http://<robot-ip>:8080/api/v1/navigation/cancel
```

```json
{
  "reason": "app_manual_cancel",
  "building_id": "B1",
  "floor_id": "F1",
  "pose_id": "delivery_123456"
}
```

By default this endpoint exits navigation mode, not just the current goal. The car-side server publishes a zero burst into `/cmd_vel_collision_checked`, accepts a background cancel job, and returns `202 Accepted` quickly. The background job cancels the cached API-started `NavigateToPose` goal handle when `/navigate_to_pose` is available, sends cancel-all to `/navigate_to_pose`, publishes another zero burst, and stops the Nav2 plus 2D localization runtime. It keeps common services alive: driver, chassis, `robot_safety`, local odom, robot description, local perception, floor manager, and the API server.

The endpoint is idempotent for mode exit: if Nav2 is already partly down, it still publishes zero velocity and stops any remaining navigation/localization processes.

If an engineering tool only wants to cancel the current goal but keep Nav2 and localization running, it may send `"stop_stack": false`. The App normal cancel button should not set this field.

The App may poll the latest cancel job:

```text
GET http://<robot-ip>:8080/api/v1/navigation/state
```

## Docking

The App stores a charger as a semantic pose on the selected map. Use `type: "dock"` and make the pose represent the final robot `base_link` pose where the front charging contacts are aligned with the dock. The backend computes the pre-dock pose automatically by backing away along the dock yaw using `docking_pre_dock_distance_m` (default `0.80 m`).

Save or update a dock pose:

```text
POST http://<robot-ip>:8080/api/v1/maps/poses
Content-Type: application/json
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "pose_id": "dock_main",
  "type": "dock",
  "name": "Main charger",
  "x": 1.25,
  "y": -0.35,
  "yaw": 3.14159
}
```

Start docking:

```text
POST http://<robot-ip>:8080/api/v1/docking/start
Content-Type: application/json
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

The backend activates the requested map, starts or waits for the standard navigation runtime, sends Nav2 to the computed pre-dock pose, starts `robot_docking_manager` if needed, then calls `/docking/start`. GS2 fine docking owns the final low-speed alignment and charging-contact confirmation.

Cancel docking:

```text
POST http://<robot-ip>:8080/api/v1/docking/cancel
Content-Type: application/json
```

```json
{"reason":"app_cancel"}
```

The cancel endpoint cancels the pre-dock Nav2 goal, calls `/docking/stop`, and publishes zero velocity through the same safe command path. The App should poll:

```text
GET http://<robot-ip>:8080/api/v1/docking/state
GET http://<robot-ip>:8080/api/v1/status
```

The App must not use mapping teleop or direct velocity commands for docking. Docking motion is owned by Nav2 plus `robot_docking_manager`.

## 2D Map Image

The default map image endpoint returns the current live `slam_toolbox` `/map` from the App-started 2D mapping session:

```text
GET http://<robot-ip>:8080/api/v1/mapping/2d/map
Content-Type: image/png
```

This endpoint is live-only by default. If no App-started 2D mapping session is active, or if live `/map` data has not arrived yet, it returns JSON `409`/`404` instead of falling back to map_server or an existing saved map.

The App must acquire `live_map` before polling this endpoint:

```json
{"client_id":"app_device_001","resources":["live_map"],"ttl_ms":10000}
```

Saved map preview is explicit:

```text
GET http://<robot-ip>:8080/api/v1/mapping/2d/map?source=saved
GET http://<robot-ip>:8080/api/v1/mapping/2d/map?name=test-16
```

Saved mode only serves existing PNG map assets, for example `<name>.png`; it does not convert `PGM` to `PNG` during the request.

## Mapping Teleop WebSocket

Use this only for App-driven low-speed movement while building a map:

```text
ws://<robot-ip>:8080/ws/v1/teleop
```

If `api_token` is set, include the same header as HTTP:

```text
X-Robot-Token: <token>
```

The App should send commands at 10-20 Hz while the user is holding a movement control, and send `stop` when the control is released:

```json
{"type":"cmd_vel","linear_x":0.20,"angular_z":0.10}
{"type":"cmd_vel","linear_x":-0.12,"angular_z":0.00}
{"type":"cmd_vel","vx":0.20,"wz":0.10}
{"type":"stop"}
```

Server-side safety contract:

- Published topic: `/cmd_vel_collision_checked`
- Final chain: `/cmd_vel_collision_checked -> robot_safety -> /cmd_vel_safe -> ranger_mini3_mode_controller -> /cmd_vel -> ranger_base_node`
- Default limits: `linear_x <= 0.30 m/s`, `abs(angular_z) <= 0.55 rad/s`
- Mapping teleop allows low-speed reverse. The API server enables `/ranger_mini3/allow_reverse` only during an active WebSocket mapping teleop session.
- Navigation still cannot reverse by default because `ranger_mini3_mode_controller.allow_reverse` remains `false`; reverse permission expires if the App stops sending commands.
- Teleop is accepted only while 2D mapping is active by default. If mapping is not active, the WebSocket upgrade returns `409`.
- Teleop stops automatically if the robot reports charging/full or charge current on `/battery_state`.
- WebSocket disconnect or receive timeout publishes a zero command; `robot_safety` watchdog remains the final stop layer.

The server also pushes `mapping_state` frames after connection and command acknowledgements:

```json
{
  "type": "mapping_state",
  "state": "running",
  "teleop_allowed": true,
  "allow_reverse": true,
  "area_m2": 42.3,
  "pose": {"x": 1.2, "y": -0.4, "yaw": 0.8}
}
```

The App must not publish to ROS 2, `/cmd_vel`, or `/cmd_vel_safe` directly.

## Jetson Start

```bash
docker exec -d NJRH-car bash -lc 'cd /workspaces/njrh-v3/workspace1 && ROBOT_API_TOKEN=change-me bash scripts/jetson/runtime_overlay/scripts/run_robot_api_server.sh'
```

The Web dashboard is not required for this API server. It should stay off in production unless you explicitly need the debug observation page.

Then test from the phone or development machine:

```bash
curl -H 'X-Robot-Token: change-me' http://192.168.31.23:8080/api/v1/status
```
