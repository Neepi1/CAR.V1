# Android App API Gateway

The Android app should connect to the robot hotspot and talk to `robot_api_server` over HTTP plus WebSocket. The app should not join ROS 2 DDS directly.

## Network

- Robot hotspot address: use the Jetson hotspot IP or static LAN IP.
- Default API port: `8080`.
- Use `X-Robot-Token` when `api_token` is configured.

The server declares `host`, `port`, `api_token`, and
`max_http_connections` through one HTTP-gateway configuration boundary. This
internal ownership does not change the App endpoint, header, authentication,
or overload behavior.

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
GET  /api/v1/elevator-config
PUT  /api/v1/elevator-config/draft
POST /api/v1/elevator-config/publish
POST /api/v1/elevator-config/rollback
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
POST /api/v1/floor-switch/start
GET  /api/v1/floor-switch/state
POST /api/v1/floor-switch/cancel
POST /api/v1/floors/switch
POST /api/v1/localization/trigger
POST /api/v1/navigation/start
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
  "map_id": "map_20260728T104137Z_aa94536eb2",
  "resume_navigation": false
}
```

The App supplies the map ID but never invents an asset epoch or digest. The
gateway resolves the immutable manifest, sends the complete identity to
`robot_floor_manager`, verifies the echoed proof, and then creates the fixed
`current/` projection. A successful offline selection does not start the occupancy localization stack or Nav2. The App must next call
`POST /api/v1/navigation/start` and poll navigation state before showing the
map as current or sending a goal.

The 3D mapping start endpoint remains reserved and returns `501`. 2D mapping is wired to the repository-owned `slam_toolbox` runtime chain, not to the test Web dashboard. Navigation startup and goals are wired to the repository-owned runtime and Nav2 `NavigateToPose`; the App should send saved `pose_id` targets instead of publishing velocity commands.

`POST /api/v1/floors/switch` remains the legacy offline selector. A request
with `"resume_navigation": true` is rejected before map lookup or mutation;
the App must not use that flag as a live-switch shortcut.

Manual live switching uses a separate strict transaction:

```text
POST /api/v1/floor-switch/start
GET  /api/v1/floor-switch/state?transaction_id=<id>
POST /api/v1/floor-switch/cancel
```

The start body contains only the exact `building_id/floor_id/map_id`. The
server resolves and freezes the current immutable asset epoch/digest and calls
only `/floor_manager/floor_switch`. Navigation/localization processes remain
resident; an active Nav2 goal is not allowed, and the car must be stationary
so the Action can prove Nav2-idle plus stable wheel/local odometry before any
map mutation. The App polls the returned transaction ID to an explicit
terminal state. It never calls offline `/floors/switch`, `/navigation/start`,
or retries a timed-out start as part of this workflow.

Success requires all of the following from the same transaction: `COMPLETE`,
`success=true`, zero `failure_code`, `runtime_context_valid=true`,
`recovery_required=false`, and exact equality between the requested target,
the frozen target epoch/digest, and the active target epoch/digest. A cancel
request is not completion; polling continues until `CANCELLED`, `FAILED`,
`COMPLETE`, or fail-closed `UNKNOWN` is returned.

Choosing a map in the App editor is App-local UI state. Read that map by
`map_id` through the map, preview, semantic-layer, pose, keepout, and elevator
configuration APIs. Do not call `/api/v1/floors/switch` merely because the
operator opened or selected a map in the editor.

`"resume_navigation": false` normally remains an offline runtime-asset
selection operation. A different map is accepted only after navigation,
mapping, docking, API goal jobs, Nav2 goals, and the navigation runtime process
have all stopped. The backend validates the selected bundle, clears the stale
runtime-map context, and then projects that bundle into
`maps_release/<building_id>/<floor_id>/current/` for a later controlled
startup. It does not reload localization, start Nav2, or authorize motion.

Ordinary single-floor navigation therefore uses two explicit requests, never
`resume_navigation=true`:

```text
POST /api/v1/floors/switch   {"building_id":"B1","floor_id":"F1","map_id":"<exact-map-id>","resume_navigation":false}
POST /api/v1/navigation/start {"building_id":"B1","floor_id":"F1","map_id":"<exact-map-id>"}
```

`POST /api/v1/navigation/start` requires the requested bundle to be the
floor's single active map, verifies its server-managed epoch/digest and the
backend-owned `current/` projection, rejects mapping/docking/goal/transition
conflicts, and starts or idempotently reuses only that exact runtime. HTTP
`202` means startup was accepted, not ready. Poll
`GET /api/v1/navigation/state` and enable navigation goals or display “当前”
only when all of these are true:

- `navigation_active == true`
- `healthy == true`
- `state` or `navigation_status` is `ready`/`running`
- `safe_for_goal_start == true`
- `runtime_map_context.confirmed == true`
- `runtime_map_context.state == "ready"`
- runtime `building_id/floor_id/map_id` exactly match the request

There is one compatibility no-op: while navigation is running, an exact
`building_id + floor_id + map_id` match with the confirmed, `ready` runtime map
returns `200` without calling the floor service, clearing runtime context,
writing `current/`, reloading localization, or changing any process:

```json
{
  "ok": true,
  "state": "runtime_map_already_selected",
  "already_active": true,
  "runtime_unchanged": true,
  "selection_performed": false
}
```

Any different or unconfirmed target still returns `409
FLOOR_SELECTION_RUNTIME_BUSY` while a motion runtime is active. This no-op is
not a floor switch and is not permission to switch maps in an elevator. The
App must not use this endpoint while the robot is riding or straddling an
elevator doorway.

`current/` is a backend-owned selection mirror; the App must edit saved map
records through map APIs and must not write files under `current/` directly.
Saving a new 2D map does not select or run it. The strict live transaction
startup waits for the trigger wrapper to report a bridge-accepted `map -> odom`,
then requires fresh costmaps and final runtime-context commit before reporting
`COMPLETE`.

Manual relocalization uses:

```text
POST http://<robot-ip>:8080/api/v1/localization/trigger
```

With gated AMCL active, this endpoint first waits for the explicit Isaac correction to finish, then waits for the bridge-owned post-Isaac AMCL refinement. The bridge requests bounded no-motion updates while the robot is stationary, rejects queued pre-seed AMCL poses, requires two consistent new-generation candidates, and finishes publishing the selected correction before HTTP success. This normally adds about one second after Isaac; the AMCL-refine failure bound is four seconds. The endpoint still does not make its result depend on the separate Nav2/local-costmap settle barrier, so an accepted localization correction is not reported as failed only because a diagnostic settle window was not clean. Diagnostic tools may pass `"wait_for_settle": true` when they intentionally want the response to include and enforce that additional barrier.

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
- Mapping transition: `mode_transition.active == true && mode_transition.owner == "mapping_start"`; show the current `mapping.start_job.phase` and keep navigation/mapping buttons disabled until the job finishes.
- Navigation: `mode == "NAVIGATION" && navigation_active == true`.
- Docking: `mode == "DOCKING" || docking.active == true || docking.state in ["accepted","nav_to_predock","fine_docking","docked","undocking","undocked","failed","canceled"]`.
- Idle: `mode == "IDLE"`.
- Error: `mode == "ERROR" || healthy == false`.

The backend intentionally keeps this status lightweight. It is driven by official API transitions such as `/mapping/2d/start`, `/mapping/2d/stop`, `/mapping/2d/save`, offline `/floors/switch?resume_navigation=false`, `/navigation/start`, `/navigation/goal`, and `/navigation/cancel`. Deep ROS checks remain backend diagnostics and should not be duplicated in the App. The API has a fixed worker pool instead of one thread per request, so the App should poll with a modest interval, for example `500ms..1000ms` while a task is active and slower while idle. Treat HTTP `503 {"error":"server busy"}` as a transient backend overload and retry with backoff instead of launching more parallel requests.

`bms.soc` is the value the App should display as real battery percentage. The data path is Ranger CAN BMS frame decoded by `ranger_base_node` into `/battery_state`, then subscribed by `robot_api_server`; the App must not read CAN or ROS 2 DDS directly. Treat `soc_valid=false` as stale or unavailable power data. Use `bms.charging_contact`, `bms.charging_contact_reason`, and `bms.contact_snapshot` for dock-contact diagnostics; full batteries may report `current=0`, so App logic must not use current alone to decide whether undocking is allowed.

## Safety Stop And Resume

`POST /api/v1/safety/stop` publishes `true` to `/safety/estop` and returns
`202 {"ok":true,"estop":true}`. It remains available while other
state-changing requests are blocked so an operator can converge to a stopped
state. `POST /api/v1/safety/resume` publishes `false` only after the existing
floor-runtime and elevator-transaction admission checks succeed, then returns
`202 {"ok":true,"estop":false}`.

The API's `/safety/status` and `/safety/motion_allowed` subscriptions are
process-resident reliable transient-local inputs. They are not page resources.
An unavailable motion sample is reported with `motion_allowed_valid=false` and
does not create a new gateway-side interlock; final arbitration remains in
`robot_safety`. A false sample with `status=COMMAND_STALE` remains a
first-command warmup case, while another explicit false state is treated as a
hard block by the existing API motion checks.

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
- Live field-marking mode: keep the editor selection local, poll
  `GET /api/v1/robot/pose`, and enable capture only when the returned
  `building_id/floor_id/map_id` exactly matches the editor map. Navigation and
  localization may remain resident; wait for the current goal to end and the
  robot to settle before recording a point.

If the App crashes or network drops, the server expires the lease after `ttl_ms`. WebSocket disconnect immediately releases the internal `teleop` lease and publishes one zero velocity command.

The subscription application module owns the scan topic/freshness and lease
TTL configuration as one boundary. Existing TTL normalization remains: the
default is at least 1000 ms and the maximum is never below the normalized
default. This extraction does not add a lease, resource, or App-side gate.

## 2D Mapping Start / Stop

Start live `slam_toolbox` mapping before requesting live map images:

```text
POST http://<robot-ip>:8080/api/v1/mapping/2d/start
```

The endpoint owns the complete navigation-to-mapping transition and returns `202 Accepted` without waiting for mode services to stop. In the background it cancels any active navigation task, publishes zero velocity, stops the navigation/localization mode services, clears the transient navigation map context, and then starts the mapping process. The App may call ordinary `/api/v1/navigation/cancel` first for its own UI flow, but it is not required and it must not call `/api/v1/navigation/stop_runtime` to assemble this transition itself.

Poll `GET /api/v1/status` and inspect `mode_transition` plus `mapping.start_job`. Expected phases are `accepted`, `cancel_navigation`, `stop_navigation_runtime`, `clear_navigation_context`, `start_mapping_process`, and `finished`. Mapping is usable only after `mapping.active=true`, `mapping.state="running"`, and `mapping.live_map_available=true`. Navigation goals, docking requests, and navigation cancel/stop requests return `409` while mapping is active or starting.

Successful response:

```json
{
  "ok": true,
  "accepted": true,
  "state": "starting",
  "map_topic": "/map",
  "map_endpoint": "/api/v1/mapping/2d/map",
  "mapping_start": {
    "id": 4,
    "state": "running",
    "phase": "accepted",
    "navigation_was_active": true,
    "navigation_cancel_ok": false,
    "navigation_stop_ok": false
  }
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
  "asset_epoch": 17,
  "asset_digest": "sha256:0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
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

The Nav2 `PGM` and Isaac localizer `PNG` are generated from the same cached occupancy grid. The endpoint also creates neutral filter masks, `asset_report.json`, `poses.yaml`, and an atomic `njrh.map_manifest.v2` manifest. The server calculates the canonical `sha256:<64 lowercase hex>` digest and binds it to a persistent, globally monotonic positive `asset_epoch`; the App must treat both fields as opaque, server-managed identity evidence. It must never calculate, increment, reuse, or substitute either value. Runtime consumers do not use the business filename directly; offline selection copies fixed role files into `maps_release/<building_id>/<floor_id>/current/nav/nav_map.yaml` and `current/localizer/localizer_map.png`. The save response includes `requires_manual_navigation_selection:true`; after all motion runtimes are stopped, call `/api/v1/floors/switch` with the returned `map_id` and `resume_navigation:false` to select it. This does not start navigation; if the operator requested navigation, follow it with `/api/v1/navigation/start` for the same exact identity and wait for the readiness proof above.

List maps:

```text
GET http://<robot-ip>:8080/api/v1/maps
```

Use `floor_maps[]` as the real map list. Each item includes `map_id`, `display_name`, `map_name`, `building_id`, `floor_id`, `asset_epoch`, `asset_digest`, `active`, `nav_map_yaml`, and `localizer_map_png`. The legacy `floors[]` array only tells the App which floor exists and which map is currently active; its active identity fields are `active_asset_epoch` and `active_asset_digest`. Cache the epoch and digest only together with the exact `building_id/floor_id/map_id`; a digest match or epoch match by itself is not sufficient identity evidence.

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

The endpoint refuses building-only or floor-only deletion. It does not stop
active mapping/navigation stacks, so the App should stop mapping or leave
navigation mode before deleting the map currently in use. It also refuses a
map referenced by the current elevator configuration with HTTP
`409 ELEVATOR_CONFIG_MAP_IN_USE`; publish a reviewed elevator release that no
longer references the map before retrying deletion.

## Elevator Commissioning Configuration

The App should implement elevator setup as a commissioning workflow, not by
editing floor `poses.yaml`. For every elevator and served floor, collect:

- exact `building_id`, `floor_id`, and `map_id`;
- `hall_call`, `landing`, and `cabin` in map-frame metres plus yaw.

`landing` is the one shared physical point outside the elevator. It replaces
the old hall-wait, doorway, and exit points. The operator does not draw a
threshold, mark jambs, or enter clearance values. Entry and exit direction are
runtime context, not extra commissioning points.

For live commissioning of each `hall_call`, `landing`, and `cabin` role:

1. Select the floor/map in the editor without calling `/floors/switch`.
2. Move the robot to the role position, end the active goal, and let the robot
   settle.
3. Call `GET /api/v1/robot/pose`.
4. Require the returned `building_id/floor_id/map_id` to exactly match the
   floor binding being edited.
5. Copy the returned `x/y/yaw` into the role in the in-memory elevator
   configuration, then `PUT` the complete draft with the latest
   `expected_draft_revision`.

Repeat this only while the robot is physically localized on the corresponding
runtime map. Do not paste the current robot pose into a different or offline
map. Elevator roles must not be written through
`/api/v1/maps/poses/save_current`; that endpoint rejects the reserved elevator
namespace. Publishing the elevator draft generates the private
`elevator_internal_poses.yaml`.

Save the whole building draft:

```text
PUT http://<robot-ip>:8080/api/v1/elevator-config/draft
Content-Type: application/json
```

```json
{
  "actor_id": "commissioning_app",
  "configuration": {
    "schema_version": 2,
    "building_id": "B3",
    "elevators": [{
      "elevator_id": "elevator_1",
      "display_name": "1号电梯",
      "floors": [{
        "floor_id": "F3",
        "map_id": "map_f3",
        "poses": {
          "hall_call": {"x": -1.0, "y": 0.0, "yaw": 0.0},
          "landing": {"x": -0.4, "y": 0.0, "yaw": 0.0},
          "cabin": {"x": 0.8, "y": 0.0, "yaw": 3.14159}
        }
      }]
    }]
  }
}
```

The example is intentionally incomplete because a publishable elevator needs
at least two floors. Draft save still returns `200` and structured `issues`;
use `valid_for_publish` to drive the wizard's missing-field UI. On the first
save omit `expected_draft_revision`; on every later save send the latest
returned revision. A stale revision returns `409`. On a first valid binding,
also omit `map_asset_epoch` and `map_asset_digest`: the server resolves the
exact map and stamps both fields into the saved draft. The App must never
invent, hash, increment, or copy those fields from another map.

Read the active draft/current release:

```text
GET http://<robot-ip>:8080/api/v1/elevator-config?building_id=B3
```

Publish only after `valid_for_publish=true`:

```json
POST /api/v1/elevator-config/publish
{
  "building_id": "B3",
  "expected_draft_revision": "draft-v2-...",
  "expected_release_id": "elevator-config-000001-...",
  "actor_id": "commissioning_app"
}
```

Omit `expected_release_id` only for the first publication. Publishing always
revalidates exact map ownership, required assets, map bounds, the indivisible
map asset epoch/digest pair, all three roles, and the generated runtime
topology.
The successful response includes a new immutable `release_id`,
`asset_published:true`, and `runtime_applied:false`.

Schema-v1 drafts are shown as an in-memory migration preview. The server chooses
`landing` from the first present legacy field in this order:
`hall_wait`, `doorway`, `exit`; a malformed higher-priority field is not hidden
by a lower-priority fallback. GET never rewrites the stored draft. The next
explicit PUT persists a `draft-v2-*` revision. Immutable schema-v1 releases
remain readable for audit but are `legacy_read_only` and cannot be rolled back
into the executable configuration.

Saving a valid draft stamps the resolved positive `map_asset_epoch` and
canonical `map_asset_digest` into every floor binding. The pair belongs to the
exact `building_id/floor_id/map_id` and must be preserved unchanged during
ordinary edits. If a map changes between review and publish, publish returns
HTTP `422`; `issues[].code` identifies `MAP_ASSET_EPOCH_CHANGED`,
`MAP_ASSET_DIGEST_CHANGED`, or both. Reload the map list, explicitly rebind the
floor, save a new draft, and review it again. A draft saved while invalid never
becomes publishable just because the map is later created or repaired: publish
returns `422 DRAFT_REVIEW_REQUIRED`; PUT the complete configuration again with
the old `expected_draft_revision`, review the new revision, and then publish.
Both identity fields are server-managed. Only an explicit user-requested
“rebind map” operation may remove both stale fields before PUT; the server then
stamps the current pair and returns a new revision for review. Do not remove or
rewrite either field during an unrelated edit. `GET /api/v1/elevator-config`
includes
`configuration_source: "draft"|"current"|null`. When a draft exists,
`configuration` is the editable draft and can differ from
`current_release_id`; do not mistake it for the selected runtime release.
The backend accepts at most 2 MiB, 16 elevators, 64 floors per elevator, and
128 total floor bindings per building document. Nav map YAML must name its
digested PGM with one plain relative filename; path components such as `..`
are rejected. Root `image`, `resolution`, and `origin` must each occur exactly
once, and dimensions are read only from the authenticated PGM; nested or
duplicate `image` keys cannot redirect validation. Unknown commissioning
extension fields are preserved in the reviewed release.

Rollback also requires the current release as a compare-and-swap guard:

```json
POST /api/v1/elevator-config/rollback
{
  "building_id": "B3",
  "release_id": "elevator-config-000001-...",
  "expected_release_id": "elevator-config-000002-...",
  "actor_id": "commissioning_app"
}
```

Rollback creates a third release from the selected historical content; it does
not move the pointer backward or overwrite history. Rollback re-resolves every
historical map binding and requires the exact stored epoch/digest pair. Drift
returns HTTP `422 ROLLBACK_TARGET_INVALID` with
`MAP_ASSET_EPOCH_CHANGED` and/or `MAP_ASSET_DIGEST_CHANGED` in `issues[]`; the
App must not retry with locally edited identity fields.

Internal elevator points live only in the release-private
`elevator_internal_poses.yaml`. Ordinary semantic-point APIs hide legacy
`type:elevator_internal`, reject writes/deletes in the reserved namespace, and
ordinary navigation returns `ELEVATOR_INTERNAL_POSE_REQUIRES_MISSION`.
Configuration publication does not switch floors, reload localization,
start/stop Nav2, send a goal, or publish velocity. The App must not offer a
"run elevator" button until the separate mission/elevator execution adapter
and live atomic FloorSwitch are deployed and commissioned.

On Linux the backend selects one immutable release with the atomic
`.elevator_config/current` directory symlink. `elevators.yaml`,
`elevator_internal_poses.yaml`, and `current.json` are compatibility views
through that selector. A consumer needing both topology and internal poses
must first fix one `release_id`, then read both files from that exact release;
two independent opens across a publication can otherwise observe different
generations.

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

Replace the complete App-authored keepout layer:

```text
POST http://<robot-ip>:8080/api/v1/maps/filters/keepout/save
Content-Type: application/json
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
      "points": [
        {"x": 1.2, "y": 3.4},
        {"x": 2.8, "y": 3.4}
      ]
    }
  ],
  "keepout_polygons": [],
  "expected_revision": "keepout-v1-fnv64-0123456789abcdef",
  "reload_filter": true
}
```

This is full-layer replacement, not an append operation. Send `keepout_lines: []` and `keepout_polygons: []` to clear the layer. Line points and polygon vertices are map-frame metres; `width_m` is the full line width. The App must convert map-image touches with the map's resolution and complete origin `[x, y, yaw]`, including the image/map Y flip. Read `keepout_revision` from `GET /api/v1/maps/semantic_layer` (or `revision` from the keepout-only GET) and return it as `expected_revision`; HTTP `412 REVISION_CONFLICT` means another client changed the full layer and the App must reload before retrying.

The backend validates and rasterizes every feature, then updates the canonical semantic/YAML/PGM assets, a last-written commit marker, the fixed active projections, and Nav2's current runtime staging copy. It rejects duplicate IDs, non-finite or degenerate geometry, self-intersecting polygons, out-of-map or non-free-cell features, excessive raster work, a layer that blocks the whole map, unmanaged legacy non-neutral masks, and semantic/PGM disagreement. A selected invalid managed mask fails startup closed rather than being replaced by neutral.

For the map currently loaded by Nav2, a successful response is not just an HTTP write acknowledgement:

```json
{
  "ok": true,
  "persisted": true,
  "outcome": "APPLIED",
  "runtime_selected": true,
  "runtime_effective": true,
  "effective": true,
  "effective_on_next_activation": false,
  "revision": "keepout-v1-fnv64-...",
  "mask": {
    "width": 1029,
    "height": 681,
    "active_cells": 274,
    "occupancy_digest": "occupancy-fnv64-..."
  },
  "runtime": {
    "mask_server_active": true,
    "filter_plugin_enabled": true,
    "load_map_succeeded": true,
    "mask_topic_matches": true,
    "global_costmap_cleared": true,
    "global_costmap_updated": true,
    "global_costmap_content_matches": true,
    "costmap_samples_checked": 8,
    "costmap_samples_blocked": 8,
    "costmap_samples_cleared": 0
  }
}
```

For a map that is not currently loaded, the server persists the assets and returns `outcome: "SAVED_DEFERRED_INACTIVE"`, `runtime_selected: false`, and `effective_on_next_activation: true`. The App may display “saved; effective after this map is activated”, but must not display “navigation effective”. The App must reject any 2xx response that proves neither of these states.

The server returns `409` without writing while mapping, docking, floor/map mutation, an API goal, or a Nav2 action goal is active, and while wheel odometry is not stably stopped. A map bound by the current elevator release is also immutable: keepout save returns `409 ELEVATOR_CONFIG_MAP_IN_USE`; publish a reviewed elevator release against the new map assets instead of mutating the bound map in place. It also returns non-2xx if the active runtime mask cannot be loaded and proven; the backend best-effort rolls back both asset files and the previous runtime mask. If rollback cannot be proven, `persisted`/`effective` are `null`, `integrity_degraded=true`, and new motion admission stays blocked until repair. No App workflow may cancel navigation or publish velocity to make a keepout edit succeed. Keepout save uses a dedicated 45-second client timeout because proof includes bounded ROS lifecycle, mask, and costmap waits.

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

`PUT /api/v1/maps/poses/batch` replaces all ordinary App semantic points, not
the elevator module's private points. Sending `"poses":[]` clears ordinary
points for that map; any legacy `type:elevator_internal` records are preserved
and remain hidden. The backend writes the selected `maps/<map_id>/poses.yaml`
and synchronizes `current/poses.yaml` when that map is active.

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

If this returns `503`, the App should wait for the target map's navigation and
localization runtime to reach confirmed ready state before saving a live point.
Starting mapping does not satisfy the live-point identity contract. The error
body is:

```json
{"ok":false,"error":"no fresh map-frame robot pose","frame_id":"map","child_frame_id":"base_link","age_sec":null}
```

When the pose is withheld because a floor transaction has not committed, the
response additionally contains a machine-readable code and the actual runtime
state, for example:

```json
{"ok":false,"code":"FLOOR_CONTEXT_NOT_READY","error":"no fresh map-frame robot pose","runtime_state":"floor_switch_failed_locked","detail":"runtime map context is not ready: ...","frame_id":"map","child_frame_id":"base_link","age_sec":null}
```

The App should display the floor transaction state/detail instead of describing
this case as a TF sampling timeout.

`map_id`, `floor_id`, and `building_id` are returned only from the robot's confirmed runtime map context. During navigation startup or floor switching, the backend may already have a fresh TF pose but still return `503` if the requested map has not been confirmed by localization and Nav2 readiness. The App must treat that as a real backend state mismatch, not silently reuse an older map context.

Navigation and localization may remain running while the operator marks or
updates points on that same confirmed runtime map. The App must compare all
three identity fields with its editor selection before enabling capture. A
different/offline map can still be edited with explicit static coordinates,
but it cannot receive the current robot pose. Selecting an editor map never
requires `/api/v1/floors/switch`.

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

The App should use this endpoint for "mark point at current robot position". Do not convert PNG pixels to map coordinates for this workflow. The server fills `x`, `y`, and `yaw` from the same live `map -> base_link` pose returned by `/api/v1/robot/pose`; if `/api/v1/robot/pose` would return `503`, `/api/v1/maps/poses/save_current` also returns `503` and does not save. Request-body `yaw` or `theta` is ignored so saved orientation always equals the live robot heading. `type: "dock"` means the final charging-contact `base_link` pose, not the pre-dock point. The server writes `maps/<map_id>/poses.yaml` and synchronizes `current/poses.yaml` if that map is active. `pose_id` is optional; when omitted, the server generates a stable ID and returns it.

This endpoint is for ordinary semantic points only. For the five elevator
roles, first `GET /api/v1/elevator-config` and retain the complete editable
configuration plus its latest draft revision. Then use
`GET /api/v1/robot/pose`, update only the selected role's `x/y/yaw`, and send
the complete configuration with the latest `expected_draft_revision` through
`PUT /api/v1/elevator-config/draft`, as described above.

Check whether a normal point navigation request would need auto-undock:

```text
GET http://<robot-ip>:8080/api/v1/navigation/pre_goal_check?building_id=B1&floor_id=F1&pose_id=delivery_123456
```

This endpoint is read-only. It resolves the saved pose from `poses.yaml`, reports `/navigate_to_pose` action-server admission readiness, and returns `would_auto_undock`, `auto_undock_required`, and `pre_navigation_dock_check` with backend docking state, `/docking/status`, BMS age/contact reason, `power_supply_status`, current, voltage, `dock_contact_snapshot`, `dock_contact_latch_source_strength`, `charging_session_latched`, `full_charge_idle_on_dock`, `dock_occupancy_state`, `dock_occupancy_evidence`, `docked_state_class`, `docked_evidence`, `docked_warnings`, `final_is_docked_or_charging`, `final_auto_undock_required`, and `auto_undock_reason`. It never calls `/docking/undock` and never sends a Nav2 goal. Use it for diagnostics before the user taps "go to point"; production execution still uses `POST /api/v1/navigation/goal`.

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

Successful response returns `202 Accepted` with `navigation_goal_id`. The server resolves the pose from `maps_release/<building_id>/<floor_id>/poses.yaml`, creates a background `navigation_goal`, and returns before waiting for controlled undock, post-undock relocalization, bridge readiness, or the Nav2 action goal handle. If the car is already docked, `/docking/status` starts with `docked` or `charging`, the backend sees fresh BMS charging contact, or `dock_occupancy_state` is `DOCKED_CHARGE_IDLE` / `UNCERTAIN_ON_DOCK`, the accepted background job automatically performs controlled undocking first, waits for odometry-confirmed `undocked`, arms the bridge one-shot correction service, triggers post-undock relocalization, waits until the result is reflected in `map -> base_link`, and only then sends the Nav2 goal. A full battery may report `current=0`; use `pre_navigation_dock_check.dock_occupancy_state`, `charging_session_latched`, `api_bms_charging_contact_reason`, and `bms.power_supply_status` rather than current alone. If undocking, post-undock relocalization, bridge acceptance, or the readiness wait times out, `/api/v1/navigation/state` reports the failed phase and no Nav2 goal is sent. The immediate `202` response includes `pre_navigation_undock`, `pre_navigation_undock_detail`, and `pre_navigation_dock_check` so the App can show queued departure from the charger instead of timing out the HTTP request. The App must not send `/cmd_vel` for task navigation, and it should not call `/api/v1/docking/undock` separately before every normal point navigation.

During normal point navigation, `robot_api_server` also publishes a distance-based Nav2 `/speed_limit` derived from the current map-frame distance to the target. This keeps the 1.2 m/s cruise speed at long range but steps down near the goal to match the measured Ranger Mini 3 stop distance, then restores the cruise limit when the Nav2 task exits. It must not publish `0.0` as a clear signal because controller-server treats that as a stop limit. The App should not publish `/speed_limit` or velocity commands.

The App must not infer docked state solely from current map position. Use `bms.charging_contact`, `docking.inferred_docked`, and `pre_navigation_dock_check.dock_contact_snapshot`. While docked or charging, normal navigation auto-undocks first; if that fails, no Nav2 goal is sent. Normal command velocity is also blocked by `robot_safety`, and only the controlled docking/undock path may move the chassis. Undock first-motion delay is handled in `robot_docking_manager`: the configured reverse speed is `undock.speed_mps=0.50`, independently capped by `undock.max_speed_mps=0.50`; `motion_start_timeout_s` waits for the first odometry displacement, and `no_progress_timeout_s` applies only after movement has started. The App should surface the backend failure string, such as `undock_failed_motion_start_timeout` or `undock_failed_no_progress`, rather than publishing reverse velocity or enabling ordinary navigation reverse.

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

By default this endpoint cancels the current navigation task and keeps the resident navigation runtime alive. The car-side server publishes a zero burst into `/cmd_vel_api`, accepts a background cancel job, and returns `202 Accepted` quickly. The background job uses a short `/navigate_to_pose` action-server probe so a partly degraded Nav2 stack does not block cancellation for the generic service timeout; when `/navigate_to_pose` is available it cancels the cached API-started `NavigateToPose` goal handle, sends cancel-all to `/navigate_to_pose`, publishes another zero burst, and leaves Nav2 plus localization resident. For this ordinary path, `navigation_cancel.navigation_stack_stopped` is `false`; it becomes `true` only when `stop_stack=true` was requested and the navigation stop command completed successfully.

The endpoint is idempotent for task exit: if Nav2 is already partly down, it still publishes zero velocity and records the cancel job failure reason without tearing down localization.

If an engineering tool needs to tear down Nav2/localization for recovery, call `POST /api/v1/navigation/stop` or `POST /api/v1/navigation/stop_runtime`. These force `stop_stack=true`, cancel any Nav2 goal, publish zero velocity, terminate Nav2/localizer/`robot_localization_bridge`, and clear the runtime map context. The car-side stop script kills Nav2/localization process patterns before bounded AMCL cleanup, so AMCL lifecycle waits cannot block the whole stop path before Nav2 is torn down. The App normal cancel button should not call these endpoints. For the user-facing "return to charger" button, do not call `/api/v1/navigation/cancel` first; call `/api/v1/docking/start` directly so the backend can cancel the current API navigation goal without stopping the Nav2/localization stack.

Before accepting a normal `POST /api/v1/navigation/goal`, the backend checks the resident runtime context and rejects only immediate request errors such as wrong map/floor, missing pose, duplicate running goal, or explicit `"force_relocalize": true`. Bridge readiness and `/navigate_to_pose` action-server readiness are checked in the accepted background job; transient `safe_for_goal_start=false` or non-standby AMCL pending states become `waiting_for_goal_start_readiness` and then either proceed or fail the job with a phase-specific reason. Normal goals do not call `/global_localization/trigger`, force-accept, post-relocalization settle, or synchronous Nav2 lifecycle `GetState` probes in the HTTP handler; lifecycle services are startup/diagnostic checks because they can time out while Nav2 is actually active under Jetson/FastDDS load. Successful responses include `pre_navigation_relocalization_requested=false`, `pre_navigation_relocalization_succeeded=false`, and a detail saying goal-start readiness is deferred to the navigation job.

The backend treats a Nav2 action result of `succeeded` as input to API-side verification, not as business completion by itself. Normal delivery goals default to `goal_completion_policy: "pose_required"` and include the target yaw in the `NavigateToPose` action. Nav2 native `RotationShimController + SimpleGoalChecker` is the primary XY/yaw completion path with a 0.06 m XY and 0.05 rad yaw gate. After Nav2 returns or aborts, the API waits for bridge `map->odom` smoothing, re-reads a fresh `map -> base_link` pose, and only sets `task_complete=true` when the commercial gate is satisfied. A 0.06-0.12 m XY overrun or 0.05-0.15 rad yaw overrun triggers bounded same-goal retry or final yaw alignment; a 0.12-0.35 m XY overrun or 0.15-0.35 rad yaw overrun enters recovery retry. If Nav2 is still executing inside 0.30 m but has not improved by 0.02 m over 1.5 s after the 3 s minimum wait, the API cancels only that Nav2 goal and enters the same final verification path instead of waiting through repeated BT recovery. Terminal pose correction is deterministic: the backend decomposes target error into signed yaw, body-frame lateral error, and body-frame forward error; it corrects yaw first with pure `angular.z`, then lateral with pure `linear.y`, then forward/reverse with pure `linear.x`. Body-frame lateral error has independent hysteresis from the circular XY gate: above 0.04 m starts terminal correction, and once started the correction remains latched until lateral error is at most 0.03 m. The backend then publishes zero, releases side-slip mode, waits for fresh `/wheel/odom` speed to remain below the configured linear/angular thresholds for 0.30 s and for Ranger feedback to confirm aligned DUAL_ACKERMAN mode, and re-reads the pose before reporting success. A settled miss receives at most one bounded recorrection; settle values and direction-reversal state are retained in `final_pose_verify_reason`. If final verification remains outside the gate after retries, the goal is reported as `state=degraded` with `task_complete=false`, not as success. If the robot is already inside the yaw-alignable XY window but yaw remains outside tolerance, the API may run one bounded ordinary `final_yaw_align` through `/cmd_vel_api -> robot_safety -> /cmd_vel`; during that spin it pauses `robot_localization_bridge` global-correction intake so AMCL/Isaac candidates cannot move `map->odom`. It never publishes API motion commands to collision_monitor's `/cmd_vel_collision_checked`, never uses `/cmd_vel_docking` for normal delivery, and the App must not publish chassis velocity. Engineering tools can request `goal_completion_policy: "position_only"` only when the final heading is irrelevant. `dock_staging` is reserved for `/api/v1/docking/start` and must not be used for normal delivery goals. The App must show success only when `task_complete=true`.

`navigation_goal` includes completion and final-yaw diagnostics for status pages and support logs: `goal_completion_policy`, `position_reached`, `yaw_align_required`, `yaw_align_active`, `yaw_align_succeeded`, `yaw_align_failed`, `final_pose_verified`, `task_complete`, `final_pose_verify_reason`, `post_nav2_final_verify_enabled`, `post_nav2_final_verify_bridge_wait_elapsed_ms`, `post_nav2_final_verify_bridge_wait_timeout`, `final_verify_retry_count`, `final_verify_retry_reason`, `final_verify_retry_goal_sent`, `final_verify_xy_error_m`, `final_verify_yaw_error_rad`, `final_verify_failure_is_terminal`, `final_yaw_align_retry_count`, `reposition_after_yaw_drift_retry_count`, `final_yaw_align_attempted`, `final_yaw_align_blocked_reason`, `final_yaw_align_duration_sec`, `final_yaw_align_timeout_sec`, `final_yaw_align_target_yaw_rad`, `final_yaw_align_initial_yaw_error_rad`, `final_yaw_align_final_yaw_error_rad`, `final_yaw_align_max_xy_drift_m`, `final_yaw_align_observed_xy_drift_m`, `final_yaw_align_cmd_topic`, and `final_yaw_align_bypass_collision_monitor`.

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

The backend never activates a map from the docking endpoint. The requested
`building_id/floor_id/map_id` must already equal the active manifest and the
confirmed, ready runtime-map context; otherwise the request returns `409
FLOOR_SWITCH_REQUIRED` before resolving a dock pose or starting a job. For that
same already-loaded map, the backend starts or reuses the standard navigation
runtime, cancels any cached API navigation goal without stopping the
Nav2/localization stack, checks bridge `safe_for_goal_start`, and sends Nav2 to
the pre-dock pose with the expected predock base yaw. If the map contains a
manual pre-dock pose named `dock_id_predock`, `dock_id_pre_dock`,
`dock_id_approach`, `predock_dock_id`, `pre_dock_dock_id`, or
`approach_dock_id`, the backend uses it after checking yaw sanity only by
default; this preserves checking yaw sanity only by default. Engineering tools
may also pass `"predock_pose_id":
"dock_main_predock"` or `"approach_pose_id": "dock_main_predock"`. A manual
pre-dock point should normally be saved with the robot centered in front of the
charger, about `0.6m..0.9m` away, but
`docking_manual_predock_distance_check_enable` defaults to `false`, so short
points such as `0.356m` are not rejected only because of distance. If no manual
point exists, the backend falls back to the geometric offset from the saved
dock contact pose. `/api/v1/docking/state` exposes `predock_pose_id`,
`approach_source`, `expected_base_yaw_at_predock`,
`expected_contact_yaw_at_predock`, `current_base_yaw_map`, `base_yaw_error`,
`reverse_yaw_offset_applied`, and `predock_yaw_verified_by_nav2`. After Nav2
reports the pre-dock goal succeeded, or after Nav2 aborts while the robot is
still inside the docking handoff window, the backend performs
`PREDOCK_POSE_VERIFY`; if XY is acceptable but yaw is not, the docking-owned
`PREDOCK_YAW_ALIGN_RECOVERY` can run before `/docking/start`. If XY or yaw is
still not verified it fails with
`DOCK_FAILED_PREDOCK_NAV_OUTSIDE_HANDOFF_WINDOW`,
`PREDOCK_NATIVE_GOAL_VERIFY_FAILED`, or
`PREDOCK_YAW_NOT_ALIGNED_AFTER_NAV2` and does not call `/docking/start`. GS2
fine docking is not called until the fine-entry path confirms
`predock_pose_verified`, distance, lateral error, yaw error, GS2 freshness,
staging handoff readiness, bridge `map->odom` smoothing completion in
`FINE_DOCKING_BRIDGE_SETTLE`, and global-correction pause. If bridge smoothing
does not settle within the bounded wait, the job fails with
`DOCK_FAILED_FINE_LOCALIZATION_TRANSITION_TIMEOUT` before `/docking/start`.
GS2 fine docking owns only the final low-speed alignment and charging-contact
confirmation.

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

The undock endpoint is accepted only when the car is already docked, the backend sees live charging contact, or the explicit dock-contact latch indicates a manually confirmed docked state. Contact can come from `power_supply_status=CHARGING/FULL`, charge current, `present=true` with valid voltage, or `present=true` with full-SOC/valid-voltage inference until the dedicated contact signal is wired. Full SOC plus pack voltage alone is not treated as dock contact because a full battery away from the charger can report the same values. It calls the car-side `/docking/undock` service; the App must not send reverse velocity directly. The car backs out through `/cmd_vel_docking`, `robot_safety`, and the Ranger mode controller, with reverse permitted by `/ranger_mini3/docking_allow_reverse`. `robot_docking_manager` allows a command-settle window and a first-motion window before declaring `undock_failed_motion_start_timeout`; once odometry has moved, a later stall reports `undock_failed_no_progress`. After `/local_state/odometry` confirms departure, the backend state briefly becomes `relocalize_after_undock` while it triggers `/global_localization/trigger`; the API requires the new localization result to be reflected in `map -> base_link` so a rejected `map -> odom` jump is not reported as success. The docking job exposes `post_undock_relocalization_requested`, `post_undock_relocalization_succeeded`, `post_undock_relocalization_required`, `post_undock_relocalization_detail`, and post-undock navigation readiness fields. If auto-undock physically succeeds but localization/TF readiness fails before releasing the pending navigation goal, `/api/v1/docking/state` remains `state: "undocked"` with `post_undock_navigation_readiness_failed: true`; the navigation request fails with a readiness message and no Nav2 goal is sent. Poll `/api/v1/docking/state` until `state` becomes `undocked` or `failed`; use `charging_contact`, `charging_contact_reason`, and `inferred_docked` to explain why undock is available while `docking.state` is not `docked`.

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
GET http://<robot-ip>:8080/api/v1/mapping/2d/map?source=saved&map_id=map_...&building_id=B11&floor_id=F1
```

The App must use this exact form for a selected catalog map. The backend
resolves the immutable manifest and returns only
`maps_release/<building>/<floor>/maps/<map_id>/localizer/<safe_map_name>.png`.
It returns JSON `404` if the map ID or that exact PNG is missing and JSON `400`
if the ID belongs to another building or floor. It never substitutes the
newest runtime PNG or a `current/` projection for an exact request.

`?name=test-16` and selector-free `?source=saved` are legacy commissioning
preview forms and must not be used to render an App-selected catalog map.
Saved mode only serves an existing PNG asset; it does not convert `PGM` to `PNG` during the request.

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

- Published topic: `/cmd_vel_api`
- Final chain: `/cmd_vel_api -> robot_safety -> /cmd_vel -> ranger_base_node`; `/cmd_vel_safe` is a robot_safety mirror for diagnostics, and no separate mode-controller process exists.
- Default limits: `abs(linear_x) <= 1.00 m/s`, `abs(angular_z) <= 0.55 rad/s`
- Mapping teleop allows low-speed reverse. The API server enables `/ranger_mini3/teleop_allow_reverse` only during an active WebSocket mapping teleop session.
- Navigation still cannot reverse by default because `robot_safety.allow_reverse` remains `false`; reverse permission expires if the App stops sending commands.
- Teleop is accepted only while 2D mapping is active by default. If mapping is not active, the WebSocket upgrade returns `409`.
- Teleop stops automatically if the robot reports charging/full or charge current on `/battery_state`.
- WebSocket disconnect or receive timeout publishes a zero command; `robot_safety` watchdog remains the final stop layer.

The server-side Teleop feature owns all `teleop_*` parameters as one
configuration boundary. Subscription lease TTL remains application-owned and
is projected into Teleop after normalization; this internal ownership change
does not alter the App protocol, defaults, admission rules, or velocity chain.

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

### Elevator test arm automation (2026-08-25)

The existing elevator-test start form remains authoritative for destination
selection: the operator selects the building/elevator plus exact source and
target floor/map/release identities. The App must not call port 8083 directly.
It renders the server transaction and only asks for a confirmation when
`awaiting_confirmation=true`.

With production arm automation enabled, a successful cabin floor press does
not produce a `TARGET_BUTTON_PRESSED` prompt. Door-open, target-floor-arrived,
and target-door-open prompts remain. Because the deployed hall-call endpoint
currently reports `capability_unavailable`, the App will still receive
`CALL_BUTTON_PRESSED` after the server completes the arm `release` task. During
feature validation the server does not add health/status/pose safety gates.

### Elevator restart-lock recovery

The App treats the elevator execution state as the only recovery authority.
`GET /api/v1/elevator-test/state` supplies `physical_zone`,
`interrupted_state`, `interrupted_expected_confirmation`, and
`allowed_recovery_actions`. A restart `LOCKED` card must therefore keep the
real interrupted timeline step instead of resetting to step 1.

When the server advertises `CONFIRM_SOURCE_OUTSIDE_AND_RELEASE`, the App shows
one field-recovery dialog and requires all of the following before submitting:

- the complete robot is outside the source-floor elevator doorway;
- the door zone and surrounding area are clear;
- the robot and wheels are fully stationary.

The request carries `action`, `physical_zone`, `stationary_confirmed`, and
`door_zone_clear_confirmed` in addition to the current transaction/state/
sequence/operator and source-floor confirmation. Submission does not mean
success: the App polls until the vehicle returns an explicit terminal result.
A `409` refreshes state and never replays the stale sequence. If the server
later advertises `RETRY_SAFETY_VERIFICATION`, the App retries only the vehicle
safety proof and does not fabricate a second on-site observation.
