# Floor Asset And Switching

This repository supports multiple immutable map bundles per floor. A
backend-owned `current/` mirror selects one bundle for a later controlled
runtime startup. The legacy floor service is source-preflight-only; the strict live
FloorSwitch Action is currently non-mutating preflight and does not yet switch
a running robot between floors.

## Asset Layout

```text
maps_release/<building_id>/<floor_id>/
  maps/<map_id>/
    manifest.json
    nav/<safe_map_name>.yaml
    nav/<safe_map_name>.pgm
    localizer/<safe_map_name>.png
    localizer/<safe_map_name>.yaml
    filters/keepout_mask.yaml
    filters/keepout_mask.pgm
    filters/speed_mask.yaml
    filters/speed_mask.pgm
    filters/binary_mask.yaml
    filters/binary_mask.pgm
    reports/asset_report.json
    poses.yaml
  current/
    manifest.json
    nav/nav_map.yaml
    nav/nav_map.pgm
    localizer/localizer_map.png
    localizer/localizer_params.yaml
    filters/...
    reports/asset_report.json
    poses.yaml
```

The Nav2 map and localizer PNG in one immutable bundle must come from the same
occupancy result. `manifest.json` binds the exact
`building_id/floor_id/map_id` to a positive `asset_epoch` and canonical asset
digest. `current/` is a projection, not an editable or authoritative map
record; App clients must address `maps/<map_id>/` through the HTTP APIs.

The `robot_occupancy_builder release_rebuild` path writes this structure
directly. For a legacy map already saved by the Web dashboard, use the
promotion helper:

```bash
bash scripts/jetson/runtime_overlay/scripts/promote_map_to_floor.sh test-16 building_1 floor_1
```

## Controlled Runtime Selection

Production and manual navigation startup consume the map that an earlier
offline selection already projected into `<floor>/current/`. Startup does not
select an immutable map from `NJRH_MAP_ID`; the selected projection's
`manifest.json` supplies the exact map ID, epoch, and digest, while
`asset_report.json` remains display/report metadata.

To select a different immutable map, first stop navigation, mapping, docking,
API goal jobs, and Nav2 goals. Then call
`POST /api/v1/floors/switch` with the exact
`building_id/floor_id/map_id` and `resume_navigation:false`. Only after that
request returns success may the controlled navigation runtime be started for
the selected building and floor:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh \
  building_1 floor_1
```

Runtime consumers resolve fixed roles from:

- `NAV2_MAP_YAML=<floor>/current/nav/nav_map.yaml`
- `NAV2_LOCALIZER_MAP_YAML=<floor>/current/localizer/localizer_params.yaml`
- `NAV2_LOCALIZER_MAP_PNG=<floor>/current/localizer/localizer_map.png`
- filter mask paths and `poses.yaml`

App editor-map selection is unrelated to this operation. The editor stores its
selected `building_id/floor_id/map_id` locally and reads saved assets and
overlays by `map_id`; opening a map editor must not activate runtime assets.

For a stationary robot with the navigation/localization runtime already
resident, manual cross-floor switching does not use the offline selector.
The App submits `POST /api/v1/floor-switch/start`, polls the exact transaction
through `GET /api/v1/floor-switch/state`, and may request cancellation through
`POST /api/v1/floor-switch/cancel`. This path wraps only the strict
`/floor_manager/floor_switch` Action. It keeps the runtime resident, requires
Nav2 to have no active goal, and does not call `/navigation/start` afterward.

The API implementation for both this live path and the legacy offline selector
is isolated in `features/floor_switch/floor_switch_module`. That module owns the
HTTP dispatch, ROS Action/service clients, exact transaction worker,
cancellation terminal proof, retained transition/localization-health
subscriptions, negative runtime interlock, and exact source verification. The
API composition root provides only cross-domain runtime observations, map
activation, and elevator motion admission. The extraction does not change the
120-second live transaction budget, the offline-only legacy semantics, or any
floor-manager/Nav2/localization behavior.

Nav2's action-status topic is event-driven and has no initial empty sample when
the current action-server process has never accepted a goal. For that exact
cold-start case, `robot_floor_manager` requires both the action server and its
status publisher to remain discovered for two seconds before proving idle.
Any active status wins immediately, and graph loss returns the proof to
unknown. This is an idle-evidence repair only: it does not skip the motion
hold, dual-odometry stop, localization, asset, bridge, costmap, or commit
barriers.

## Costmap Filters

The standard Nav2 path now consumes the floor filter assets instead of only validating them:

- `filters/keepout_mask.yaml` is loaded by `keepout_filter_mask_server` and applied by the global costmap `KeepoutFilter`.
- `filters/speed_mask.yaml` is staged for compatibility, but production startup does not launch `speed_filter_mask_server` and the global costmap does not consume `SpeedFilter` by default. Set `NJRH_ENABLE_SPEED_FILTER=true` only for a deliberate speed-zone rollback/A-B. Field startup traces showed delayed `/speed_filter_mask` lifecycle and delivery can block Nav2 readiness; the `SpeedFilter` block remains in config for sites that require speed zones.
- `filters/binary_mask.yaml` is still generated and validated as part of the floor bundle, but it is reserved for later semantic mode switching and is not an active Nav2 plugin yet.

When no structured floor bundle is selected, `run_nav2_navigation.sh` generates neutral keepout/speed masks that match the current Nav2 map dimensions, resolution, and origin. The neutral mask asset uses white/free PGM pixels (`254`) with trinary map YAML so `map_server` loads the filter `OccupancyGrid` as value `0`. That loaded value means no keepout; for Nav2 `SpeedFilter`, `0` also means no speed restriction if `speed_filter` is explicitly added back to the global costmap `filters` list. A black trinary PGM pixel (`0`) would load as occupied and must not be used for an empty neutral mask. Non-zero speed-mask values are percentage limits because the speed filter info server uses `type=1`, `base=0.0`, and `multiplier=1.0`.

During `/floor_manager/switch_floor`, `resume_navigation=false` is
selection-only: `robot_floor_manager` requires the exact
`building_id/floor_id/map_id/expected_asset_epoch/expected_asset_digest`,
validates the immutable `maps/<map_id>/` source bundle, and echoes the verified
identity and dynamic source paths. It does not require `current/`,
`/map_server`, Isaac localization, or Nav2. Service success is only a
source-preflight; the API remains responsible for the serialized `current/`
activation. It does not publish `switching:`/`active:` or retain selected-map
runtime state. The API holds the cross-process map-asset commit lock through
the service proof, re-verifies the same epoch/digest, and rejects source drift
before activation. `resume_navigation=true` is rejected with
`LEGACY_RESUME_NAVIGATION_DISABLED` before asset validation or ROS side
effects. The legacy service does not reload a running localization/Nav2 stack.

## Test Web Controls

The dashboard exposes test-only floor buttons for field validation:

- `测试：列出楼层资产` calls `GET /api/floors/list` and reports valid or missing floor bundles.
- `测试：归档地图到楼层` calls `promote_map_to_floor.sh <map_name> <building_id> <floor_id>`.
- `测试：选择楼层资产` calls `select_floor_assets.sh` and stores the selected floor environment in the Web dashboard process for later Web-launched localization/navigation stacks.
- `测试：切换楼层` calls the selection-only
  `/floor_manager/switch_floor` path with `resume_navigation=false`.

These controls are not the production mission UI. They are only a test harness
for validating the floor asset contract, asset promotion, and offline
floor-asset selection service/contract on Jetson.

## Floor Switch Service

`robot_floor_manager` provides:

```bash
ros2 service call /floor_manager/switch_floor robot_interfaces/srv/SwitchFloor \
  "{building_id: 'building_1', floor_id: 'floor_2', map_id: 'map_...', \
expected_asset_epoch: 42, expected_asset_digest: 'sha256:<64-lowercase-hex>', \
resume_navigation: false}"
```

With `resume_navigation=false`, the service verifies the exact immutable
source bundle and returns the same identity. It does not create `current/` or
claim that the source is selected/active. Only the API transaction may commit
that state.
With `resume_navigation=true`, it returns
`LEGACY_RESUME_NAVIGATION_DISABLED`.

The public HTTP `/api/v1/floors/switch` adds two admission rules around this
service:

- selecting a different map is rejected with
  `FLOOR_SELECTION_RUNTIME_BUSY` until navigation, mapping, docking, active
  jobs, and the navigation process have all stopped;
- requesting the exact confirmed `ready` navigation runtime map is a
  compatibility no-op that returns `runtime_map_already_selected` without
  calling this service or mutating `current/` or runtime context.

That no-op is not a floor switch. The strict `/floor_manager/floor_switch`
Action is the only live mutation path. When explicitly enabled by the runtime
profile, it owns the stopped-motion proof, exact asset snapshot, map/filter
reload, localizer generation change, bridge BEGIN/COMMIT fence, explicit
target localization, costmap refresh, runtime-context commit, and scoped hold
release. The package default remains disabled so installing a package alone
cannot silently enable live switching.

### Post-reload ordering and timeout hierarchy

The live transaction must not call Isaac immediately after the component
manager acknowledges `load_node`. That acknowledgement precedes completion of
the localizer's GXF graph. The global-localization wrapper therefore keeps its
original trigger client alive so DDS can rediscover the same-named service
after replacement, and waits for all of the following before arming the bridge:

- the new localizer generation is the one loaded for the exact target asset;
- at least `1.0 s` has elapsed after replacement;
- a fresh `/flatscan` sequence newer than the reload baseline is present; and
- `/trigger_grid_search_localization` is ready for three consecutive samples.

The readiness barrier is bounded at `8 s` and reports
`LOCALIZER_POST_RELOAD_NOT_READY` without mutating `map->odom` when it fails.
The floor manager gives the component asset operation a separate `30 s` budget;
a delayed/lost Apply response is reconciled only from fresh terminal typed
state with the same transaction, exact requested and active identity,
`reloaded=true`, a newer generation, and `localizer_ready=true`. It then uses a
separate `75 s` outer timeout for the composite explicit-localization
transaction; its ordinary service timeout remains `10 s`. If that trigger
response is delayed or lost, the floor manager evaluates the generation- and
identity-fenced target evidence. It accepts only a newer exact-target explicit
localization sequence (`OK_RECONCILED`); no evidence still fails closed.

A wrapper response is retried only when it proves
`dispatch_state=not_dispatched`: localizer busy/post-reload/input readiness,
Isaac service unavailable, or bridge-arm service unavailable/timeout. A queued
pre-arm result is drained by the same wrapper request without changing its arm
time or issuing another Isaac request. After dispatch, result/bridge/
`map->odom` failures are reconciled against exact newer target evidence but do
not authorize a retry. The floor manager does not reload target assets, release
the safety hold, change map identity, or restart the 75 second deadline. Wrong
TF ownership, missing TF history, and bridge rejection remain terminal and fail
closed.
Both the direct App floor-switch worker and the elevator runtime adapter inherit
the API's `120 s` live-Action timeout. The elevator adapter must not silently
fall back to its shorter library default.

The bridge's ordinary 20 m forced-correction limit is a same-map recovery
guard, not a cross-floor coordinate-origin constraint. During the exact
transaction above, it is bypassed only after the target Localizer state proves
the same transaction, requested identity, active identity, successful reload,
positive generation, and readiness. The same 29.671 m result remains rejected
for startup/manual/same-map relocalization or for any transaction/identity
mismatch. No threshold in the production YAML is raised.

## Hardware Validation Still Required

- Run the App transaction against two real floor bundles while the resident
  navigation runtime is active but has no goal; verify no Nav2 process restart.
- Verify Isaac target-map reload and target-floor relocalization on the real
  elevator ride, including a rejected/wrong-floor localization sample.
- Verify bridge-owned `map->odom`, both fresh costmaps, exact runtime context,
  and motion-hold release before the next navigation goal is admitted.
- Request cancellation in each pre/post-mutation phase and verify the returned
  terminal/recovery state without automatic offline fallback.
