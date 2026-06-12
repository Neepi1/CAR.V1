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
GET  /api/v1/navigation/pre_goal_check
POST /api/v1/navigation/goal
POST /api/v1/navigation/cancel
POST /api/v1/navigation/stop
POST /api/v1/navigation/stop_runtime
GET  /api/v1/docking/state
POST /api/v1/docking/start
POST /api/v1/docking/undock
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

When `POST /api/v1/floors/switch` uses `"resume_navigation": true`, the car starts the floor navigation runtime for that `building_id` and `floor_id`: resolve the released `nav/nav_map.yaml`, start the occupancy localization stack, start the `robot_global_localization` wrapper, call `/floor_manager/switch_floor` best-effort with `resume_navigation=false`, wait for the localizer services/map/flatscan inputs, and send one bounded `/global_localization/trigger` request. The startup waits for the trigger wrapper to report a bridge-accepted `map -> odom` from `robot_localization_bridge` before starting standard Nav2 with the active floor filter masks. Local-costmap and safety-topic shell probes remain diagnostics and API goal-admission inputs after the runtime is up; they are not restored as high-frequency startup loops. The backend records that manual navigation selection in `maps_release/last_navigation_map.json`; the next boot uses only that recorded map plus the matching `current/manifest.json` to auto-resume navigation. If no valid last map exists, common services stay alive and Nav2 is not started. If the navigation resume child process exits during startup, `/api/v1/status` and `/api/v1/navigation/state` report navigation `failed` instead of staying in `starting`. The App still only requests the mode transition; it must not launch Nav2 or send task-navigation velocity commands.

Repeated `resume_navigation=true` for the same ready `map_id/building_id/floor_id` returns `state: "navigation_runtime_reused"` and keeps the existing runtime process alive. The API must not stop a healthy Nav2/localization stack just because the App page refreshed or sent the same floor request again.

When the request includes `map_id` or `map_name`, the backend activates that saved map into `maps_release/<building_id>/<floor_id>/current/` before floor switch or navigation resume. `current/` is a backend-owned runtime mirror; the App should edit saved map records through map APIs and must not write files under `current/` directly. Saving a new 2D map does not activate it; the App must explicitly select the saved `map_id` before navigation starts.

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
    "map_endpoint": "/api/v1/mapping/2d/map",
    "live_map_available": true,
    "live_map_age_sec": 0.2,
    "live_map_width": 1024,
    "live_map_height": 1024
  },
  "navigation": {
    "active": false,
    "state": "stopped",
    "action": "/navigate_to_pose",
    "blocked_by_docked_contact": false,
    "normal_motion_blocked_reason": ""
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
  "safety": {
    "status": "OK",
    "motion_allowed": true,
    "motion_allowed_valid": true
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

The App should use these lightweight business fields instead of inferring state from ROS nodes or topics:

- Mapping: `mode == "MAPPING_2D" && mapping_active == true`.
- Navigation: `mode == "NAVIGATION" && navigation_active == true`.
- Docking: `mode == "DOCKING" || docking.active == true || docking.state in ["accepted","nav_to_predock","fine_docking","docked","undocking","undocked","failed","canceled"]`.
- Idle: `mode == "IDLE"`.
- Error: `mode == "ERROR" || healthy == false`.

The backend intentionally keeps this status lightweight. It is driven by official API transitions such as `/mapping/2d/start`, `/mapping/2d/stop`, `/mapping/2d/save`, `/floors/switch?resume_navigation=true`, `/navigation/goal`, and `/navigation/cancel`. Deep ROS checks remain backend diagnostics and should not be duplicated in the App. The API has a fixed worker pool instead of one thread per request, so the App should poll with a modest interval, for example `500ms..1000ms` while a task is active and slower while idle. Treat HTTP `503 {"error":"server busy"}` as a transient backend overload and retry with backoff instead of launching more parallel requests.

`bms.soc` is the value the App should display as real battery percentage. The data path is Ranger CAN BMS frame decoded by `ranger_base_node` into `/battery_state`, then subscribed by `robot_api_server`; the App must not read CAN or ROS 2 DDS directly. Treat `soc_valid=false` as stale or unavailable power data. Use `bms.charging_contact`, `bms.charging_contact_reason`, and `bms.contact_snapshot` for dock-contact diagnostics; full batteries may report `current=0`, so App logic must not use current alone to decide whether undocking is allowed.

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

- Home / robot detail: `status` may still be acquired for compatibility, but safety state is now a backend-resident cache. Releasing the page lease must not be treated as clearing `/safety/status` or `/safety/motion_allowed`.
- Mapping page: acquire `live_map`, `tf`, and `teleop`, then open `WS /ws/v1/teleop`. The API keeps an internal `/map` cache during active 2D mapping for status and save, but the page should still acquire `live_map` before polling the PNG endpoint.
- Map editing page: usually does not acquire ROS resources; read saved map assets and semantic layers only.

If the App crashes or network drops, the server expires the lease after `ttl_ms`. WebSocket disconnect immediately releases the internal `teleop` lease and publishes one zero velocity command.

## 2D Mapping Start / Stop

Start live `slam_toolbox` mapping before requesting live map images:

```text
POST http://<robot-ip>:8080/api/v1/mapping/2d/start
```

If a navigation task is active, the backend cancels the task and publishes zero velocity before entering mapping mode. The App should not call `/api/v1/navigation/stop_runtime` before mapping; that endpoint is reserved for engineering recovery.

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

The stop endpoint terminates the complete 2D mapping-side chain: `run_projected_map.sh`, the `slam_toolbox` launch/processes, scan preprocessing and republishing nodes, the C++ mapping odom bridge, and the mapping-owned FAST-LIO2 process marked with `NJRH_SLAM2D_PRIVATE_FASTLIO=1`. It does not stop canonical chassis, TF, local state, safety, or API services.

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

The Nav2 `PGM` and Isaac localizer `PNG` are generated from the same cached occupancy grid. The endpoint also creates neutral filter masks, `asset_report.json`, `poses.yaml`, and `manifest.json`. Runtime consumers do not use the business filename directly; activation copies fixed role files into `maps_release/<building_id>/<floor_id>/current/nav/nav_map.yaml` and `current/localizer/localizer_map.png`. The save response includes `requires_manual_navigation_selection:true`; call `/api/v1/floors/switch` with the returned `map_id` and `resume_navigation:true` to start navigation on that map.

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

Check whether a normal point navigation request would need auto-undock:

```text
GET http://<robot-ip>:8080/api/v1/navigation/pre_goal_check?building_id=B1&floor_id=F1&pose_id=delivery_123456
```

This endpoint is read-only. It resolves the saved pose from `poses.yaml`, reports the critical Nav2 lifecycle summary, and returns `would_auto_undock`, `auto_undock_required`, and `pre_navigation_dock_check` with backend docking state, `/docking/status`, BMS age/contact reason, `power_supply_status`, current, voltage, `dock_contact_snapshot`, `docked_state_class`, `docked_evidence`, `docked_warnings`, `final_is_docked_or_charging`, `final_auto_undock_required`, and `auto_undock_reason`. It never calls `/docking/undock` and never sends a Nav2 goal. Use it for diagnostics before the user taps "go to point"; production execution still uses `POST /api/v1/navigation/goal`.

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

Successful response returns `202 Accepted` with `navigation_goal_id`. The server resolves the pose from `maps_release/<building_id>/<floor_id>/poses.yaml`, sends a Nav2 `NavigateToPose` goal on `/navigate_to_pose`, and tracks the result in the background. If the car is already docked, `/docking/status` starts with `docked` or `charging`, or the backend sees fresh BMS charging contact, this endpoint automatically performs controlled undocking first, waits for odometry-confirmed `undocked`, arms the bridge one-shot correction service, triggers post-undock relocalization, waits until the result is reflected in `map -> base_link`, and only then sends the Nav2 goal. A full battery may report `current=0`; use `pre_navigation_dock_check.api_bms_charging_contact`, `api_bms_charging_contact_reason`, and `bms.power_supply_status` rather than current alone. If undocking, post-undock relocalization, bridge acceptance, or the wait times out, the response is an error and no navigation goal is sent. Success and admission-failure responses include `pre_navigation_undock`, `pre_navigation_undock_detail`, and `pre_navigation_dock_check`. The App must not send `/cmd_vel` for task navigation, and it should not call `/api/v1/docking/undock` separately before every normal point navigation.

The App must not infer docked state solely from current map position. Use `bms.charging_contact`, `docking.inferred_docked`, and `pre_navigation_dock_check.dock_contact_snapshot`. While docked or charging, normal navigation auto-undocks first; if that fails, no Nav2 goal is sent. Normal command velocity is also blocked by `robot_safety`, and only the controlled docking/undock path may move the chassis. Undock first-motion delay is handled in `robot_docking_manager`: the retained calibrated reverse speed is `undock.speed_mps=0.06`, `motion_start_timeout_s` waits for the first odometry displacement, and `no_progress_timeout_s` applies only after movement has started. The App should surface the backend failure string, such as `undock_failed_motion_start_timeout` or `undock_failed_no_progress`, rather than publishing reverse velocity or enabling ordinary navigation reverse.

Maintenance recovery for a robot physically on the charger with missing BMS/contact evidence is handled through protected backend endpoints, not ordinary user UI:

```text
POST /api/v1/docking/confirm_docked
POST /api/v1/docking/clear_docked_latch
```

Both endpoints send no velocity. They only set or clear the persistent docked latch used by pre-goal admission and `robot_safety`.

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

By default this endpoint cancels the current navigation task and keeps the resident navigation runtime alive. The car-side server publishes a zero burst into `/cmd_vel_collision_checked`, accepts a background cancel job, and returns `202 Accepted` quickly. The background job uses a short `/navigate_to_pose` action-server probe so a partly degraded Nav2 stack does not block cancellation for the generic service timeout; when `/navigate_to_pose` is available it cancels the cached API-started `NavigateToPose` goal handle, sends cancel-all to `/navigate_to_pose`, publishes another zero burst, and leaves Nav2 plus localization resident.

The endpoint is idempotent for task exit: if Nav2 is already partly down, it still publishes zero velocity and records the cancel job failure reason without tearing down localization.

If an engineering tool needs to tear down Nav2/localization for recovery, call `POST /api/v1/navigation/stop` or `POST /api/v1/navigation/stop_runtime`. These force `stop_stack=true`, cancel any Nav2 goal, publish zero velocity, terminate Nav2/localizer/`robot_localization_bridge`, and clear the runtime map context. The App normal cancel button should not call these endpoints. For the user-facing "return to charger" button, do not call `/api/v1/navigation/cancel` first; call `/api/v1/docking/start` directly so the backend can cancel the current API navigation goal without stopping the Nav2/localization stack.

Before accepting a normal `POST /api/v1/navigation/goal`, the backend checks that the critical Nav2 lifecycle nodes are active and evaluates whether relocalization is needed. Same-map goals with confirmed runtime map context and a fresh `map -> base_link` pose skip `/global_localization/trigger`; cold or unconfirmed runtime context, stale/missing map-frame pose, controlled undock, docking transitions, or an explicit `"force_relocalize": true` request still refresh Isaac localization and require the result to be accepted by `robot_localization_bridge` before any Nav2 goal is sent. Successful responses include `pre_navigation_relocalization_requested`, `pre_navigation_relocalization_succeeded`, and `pre_navigation_relocalization_detail` for App diagnostics.

The backend treats delivery-point success as reaching the saved position first. A Nav2 action result of `succeeded` is not accepted by itself: after Nav2 returns, the API re-reads a fresh `map -> base_link` pose and requires the final distance to be within `navigation_goal_position_success_tolerance_m` (`0.20 m` by default). If position is reached and yaw error is above `navigation_final_yaw_align_trigger_rad` (`0.08 rad`), the API runs a bounded mission-layer final yaw alignment and then verifies the fresh final pose again. `navigation_final_yaw_tolerance_rad` is `0.05 rad`; yaw between tolerance and trigger is a no-spin deadband. This is not a request for the App to send velocity, and it is not a tiny Nav2 yaw-goal-tolerance workaround. The first field version publishes the API-owned final yaw command through `/cmd_vel_collision_checked -> robot_safety -> /cmd_vel_safe`, so `robot_safety` remains the arbiter and the job exposes `final_yaw_align_bypass_collision_monitor: true` until a `/cmd_vel_nav` mux is introduced. If only the final heading remains and alignment is blocked, the navigation goal job still reports `state: "succeeded"` with `phase: "position_reached_yaw_warning"` and `final_yaw_align_blocked: true`. This is a user-visible heading warning, not a failed delivery. If the position itself is not reached, the job remains failed even if Nav2 reported success.

`navigation_goal` includes final-yaw diagnostics for status pages and support logs: `final_pose_verified`, `final_pose_verify_reason`, `final_yaw_align_attempted`, `final_yaw_align_blocked_reason`, `final_yaw_align_duration_sec`, `final_yaw_align_timeout_sec`, `final_yaw_align_target_yaw_rad`, `final_yaw_align_initial_yaw_error_rad`, `final_yaw_align_final_yaw_error_rad`, `final_yaw_align_max_xy_drift_m`, `final_yaw_align_observed_xy_drift_m`, `final_yaw_align_cmd_topic`, and `final_yaw_align_bypass_collision_monitor`.

The App may poll the latest cancel job and goal job:

```text
GET http://<robot-ip>:8080/api/v1/navigation/state
```

## Docking

The App stores a charger as a semantic pose on the selected map. Use `type: "dock"` and make the pose represent the final robot `base_link` pose where the front charging contacts are aligned with the dock. The backend computes the pre-dock pose automatically by backing away along the dock yaw using `docking_pre_dock_distance_m` (default `0.60 m`).

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

The backend activates the requested map, starts or waits for the standard navigation runtime, cancels any cached API navigation goal without stopping the Nav2/localization stack, triggers `/global_localization/trigger`, waits for a fresh `/localization_result` or a very recent existing result, sends Nav2 to the pre-dock pose, triggers localization again after Nav2 reports the pre-dock goal succeeded, and verifies the refreshed `map -> base_link` pose is still close to the approach pose before starting `robot_docking_manager`. The App may keep the existing body unchanged. If the map contains a manual pre-dock pose named `dock_id_predock`, `dock_id_pre_dock`, `dock_id_approach`, `predock_dock_id`, `pre_dock_dock_id`, or `approach_dock_id`, the backend uses it after checking yaw sanity only by default. Engineering tools may also pass `"predock_pose_id": "dock_main_predock"` or `"approach_pose_id": "dock_main_predock"`. A manual pre-dock point should normally be saved with the robot centered in front of the charger, about `0.6m..0.9m` away, but `docking_manual_predock_distance_check_enable` defaults to `false`, so short points such as `0.356m` are not rejected only because of distance. If no manual point exists, the backend falls back to the geometric offset from the saved dock contact pose. `/api/v1/docking/state` exposes `predock_pose_id` and `approach_source`. If localization does not refresh, is stale, is rejected by `robot_localization_bridge`, or shows the car is outside the configured approach-pose tolerance, the request fails before the robot continues into GS2 fine docking. GS2 fine docking owns the final low-speed alignment and charging-contact confirmation. After GS2 fine docking returns a terminal status such as `docked`, `dock_feature_not_found`, or `contact_verify_timeout`, the backend briefly reports `relocalize_after_fine_docking`, triggers `/global_localization/trigger`, then reports the final docking state so later Nav2 actions do not inherit uncorrected fine-docking odometry drift.

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

Undock from a charger:

```text
POST http://<robot-ip>:8080/api/v1/docking/undock
Content-Type: application/json
```

```json
{"dock_id":"dock_main","reason":"app_manual_undock"}
```

The undock endpoint is accepted only when the car is already docked, the backend sees live charging contact, or the explicit dock-contact latch indicates a manually confirmed docked state. Contact can come from `power_supply_status=CHARGING/FULL`, charge current, `present=true` with valid voltage, or `present=true` with full-SOC/valid-voltage inference until the dedicated contact signal is wired. Full SOC plus pack voltage alone is not treated as dock contact because a full battery away from the charger can report the same values. It calls the car-side `/docking/undock` service; the App must not send reverse velocity directly. The car backs out through `/cmd_vel_docking`, `robot_safety`, and the Ranger mode controller, with reverse permitted by `/ranger_mini3/docking_allow_reverse`. `robot_docking_manager` allows a command-settle window and a first-motion window before declaring `undock_failed_motion_start_timeout`; once odometry has moved, a later stall reports `undock_failed_no_progress`. After `/local_state/odometry` confirms departure, the backend state briefly becomes `relocalize_after_undock` while it triggers `/global_localization/trigger`; the API requires the new localization result to be reflected in `map -> base_link` so a rejected `map -> odom` jump is not reported as success. The docking job exposes `post_undock_relocalization_requested`, `post_undock_relocalization_succeeded`, `post_undock_relocalization_required`, and `post_undock_relocalization_detail`. Poll `/api/v1/docking/state` until `state` becomes `undocked` or `failed`; use `charging_contact`, `charging_contact_reason`, and `inferred_docked` to explain why undock is available while `docking.state` is not `docked`.

The App must not use mapping teleop or direct velocity commands for docking. Docking motion is owned by Nav2 plus `robot_docking_manager`.

## 2D Map Image

The default map image endpoint returns the current live `slam_toolbox` `/map` from the App-started 2D mapping session:

```text
GET http://<robot-ip>:8080/api/v1/mapping/2d/map
Content-Type: image/png
```

This endpoint is live-only by default. If no App-started 2D mapping session is active, or if live `/map` data has not arrived yet, it returns JSON `409`/`404` instead of falling back to map_server or an existing saved map. `GET /api/v1/status` and save use the API's internal `/map` cache while mapping is active, so they do not depend on the App acquiring this page resource first.

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
- Mapping teleop allows low-speed reverse. The API server enables `/ranger_mini3/teleop_allow_reverse` only during an active WebSocket mapping teleop session.
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
