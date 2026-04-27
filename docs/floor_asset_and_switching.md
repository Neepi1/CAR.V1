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

The service validates assets, calls `/map_server/load_map`, applies localizer assets through `/global_localization/apply_floor_assets`, triggers relocalization, and clears costmaps.

## Hardware Validation Still Required

- Verify Isaac localizer reload on real floor assets without stale NITROS graph state.
- Verify `map->odom` stability after switching floors before sending motion commands.
- Verify Web UI floor selection is wired to the same asset contract; the current implementation provides the backend and runtime helper first.
