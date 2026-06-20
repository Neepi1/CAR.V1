# Floor Asset And Switching

This repository now has a minimal multi-floor contract. It supports one map bundle per floor and a runtime service that switches the active floor assets.

## Asset Layout

```text
maps_release/<building_id>/<floor_id>/
  nav/nav_map.yaml
  nav/nav_map.pgm
  localizer/localizer_map.png
  localizer/localizer_params.yaml
  filters/keepout_mask.yaml
  filters/keepout_mask.pgm
  filters/speed_mask.yaml
  filters/speed_mask.pgm
  filters/binary_mask.yaml
  filters/binary_mask.pgm
  reports/asset_report.json
  poses.yaml
```

`nav_map.yaml` and `localizer_map.png` must come from the same occupancy result. The `robot_occupancy_builder release_rebuild` path writes this structure directly. For a map already saved by the Web dashboard, use the promotion helper:

```bash
bash scripts/jetson/runtime_overlay/scripts/promote_map_to_floor.sh test-16 building_1 floor_1
```

## Runtime Selection

To start localization/navigation against a structured floor bundle:

```bash
export NJRH_BUILDING_ID=building_1
export NJRH_FLOOR_ID=floor_1
bash scripts/jetson/runtime_overlay/scripts/run_occupancy_grid_localization.sh
bash scripts/jetson/runtime_overlay/scripts/run_nav2_navigation.sh
```

The helper resolves:

- `NAV2_MAP_YAML=<floor>/nav/nav_map.yaml`
- `NAV2_LOCALIZER_MAP_YAML=<floor>/localizer/localizer_params.yaml`
- `NAV2_LOCALIZER_MAP_PNG=<floor>/localizer/localizer_map.png`
- filter mask paths and `poses.yaml`

## Costmap Filters

The standard Nav2 path now consumes the floor filter assets instead of only validating them:

- `filters/keepout_mask.yaml` is loaded by `keepout_filter_mask_server` and applied by the global costmap `KeepoutFilter`.
- `filters/speed_mask.yaml` is staged for compatibility, but production startup does not launch `speed_filter_mask_server` and the global costmap does not consume `SpeedFilter` by default. Set `NJRH_ENABLE_SPEED_FILTER=true` only for a deliberate speed-zone rollback/A-B. Field startup traces showed delayed `/speed_filter_mask` lifecycle and delivery can block Nav2 readiness; the `SpeedFilter` block remains in config for sites that require speed zones.
- `filters/binary_mask.yaml` is still generated and validated as part of the floor bundle, but it is reserved for later semantic mode switching and is not an active Nav2 plugin yet.

When no structured floor bundle is selected, `run_nav2_navigation.sh` generates neutral keepout/speed masks that match the current Nav2 map dimensions, resolution, and origin. The neutral mask asset uses white/free PGM pixels (`254`) with trinary map YAML so `map_server` loads the filter `OccupancyGrid` as value `0`. That loaded value means no keepout; for Nav2 `SpeedFilter`, `0` also means no speed restriction if `speed_filter` is explicitly added back to the global costmap `filters` list. A black trinary PGM pixel (`0`) would load as occupied and must not be used for an empty neutral mask. Non-zero speed-mask values are percentage limits because the speed filter info server uses `type=1`, `base=0.0`, and `multiplier=1.0`.

During `/floor_manager/switch_floor`, `resume_navigation=false` is selection-only: `robot_floor_manager` validates the requested floor assets, records the active floor for the next navigation start, and does not require `/map_server` or Isaac localization services to be running. With `resume_navigation=true`, the caller must have already started the localization runtime; then `robot_floor_manager` reloads `/map_server`, applies localizer assets, triggers localization, and leaves filter masks to the standard Nav2 startup path when filter mask lifecycle nodes are not active yet. This prevents a test/app map selection from failing just because Nav2 is not currently running.

## Test Web Controls

The dashboard exposes test-only floor buttons for field validation:

- `测试：列出楼层资产` calls `GET /api/floors/list` and reports valid or missing floor bundles.
- `测试：归档地图到楼层` calls `promote_map_to_floor.sh <map_name> <building_id> <floor_id>`.
- `测试：选择楼层资产` calls `select_floor_assets.sh` and stores the selected floor environment in the Web dashboard process for later Web-launched localization/navigation stacks.
- `测试：切换楼层` calls `/floor_manager/switch_floor` through `robot_floor_manager`.

These controls are not the production mission UI. They are only a test harness for validating the floor asset contract, asset promotion, and atomic floor-switch service on Jetson.

## Floor Switch Service

`robot_floor_manager` provides:

```bash
ros2 service call /floor_manager/switch_floor robot_interfaces/srv/SwitchFloor \
  "{building_id: 'building_1', floor_id: 'floor_2', resume_navigation: true}"
```

With `resume_navigation=false`, the service validates and selects assets only. With `resume_navigation=true`, the service calls `/map_server/load_map`, applies localizer assets through `/global_localization/apply_floor_assets`, triggers relocalization through `/global_localization/trigger`, and clears costmaps where appropriate. Navigation resume must start the `robot_global_localization` wrapper first and wait for `/global_localization/apply_floor_assets`, `/global_localization/trigger`, and Isaac `/trigger_grid_search_localization` before calling this service.

## Hardware Validation Still Required

- Verify Isaac localizer reload on real floor assets without stale NITROS graph state.
- Verify `map->odom` stability after switching floors before sending motion commands.
- Verify Web UI floor selection is wired to the same asset contract; the current implementation provides the backend and runtime helper first.
