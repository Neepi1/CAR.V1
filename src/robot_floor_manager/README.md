# robot_floor_manager

`robot_floor_manager` owns only floor asset switching. It does not publish TF and does not alter FAST-LIO2, PGO, local perception, or Nav2 controller behavior.

## Services

- `/floor_manager/switch_floor` (`robot_interfaces/srv/SwitchFloor`)

The switch sequence is:

1. Validate `maps_root/<building_id>/<floor_id>` assets.
2. Load `nav/nav_map.yaml` into `/map_server`.
3. Apply `localizer/localizer_map.png` and `localizer/localizer_params.yaml` through `/global_localization/apply_floor_assets`.
4. Trigger global localization through `/global_localization/trigger`.
5. Clear global and local costmaps.

## Required Assets

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

## Field Validation Still Required

- Verify `/map_server/load_map` latency while Nav2 is active.
- Verify Isaac localizer reloads the floor PNG and params without stale NITROS graph state.
- Verify `map->odom` restabilizes after floor switch before motion is resumed.
