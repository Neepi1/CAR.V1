# robot_map_toolkit

Offline import/export skeleton for turning `mapping_result` into per-floor navigation and localization assets.

## Parameters

- `mapping_result_dir`: source mapping artifact root
- `maps_root`: destination asset root
- `default_building_id`, `default_floor_id`: scaffold defaults

## Scope

- Import and export contract only in this iteration
- No online localization or runtime TF responsibilities
- Formal release 2D asset generation is expected to call `robot_occupancy_builder release_rebuild` rather than projecting the final saved point cloud directly

## CLI

Create or validate a structured floor bundle:

```bash
ros2 run robot_map_toolkit map_toolkit_cli.py \
  --maps-root maps_release \
  --building-id building_1 \
  --floor-id floor_1
```

Promote a flat Web-saved map into a floor bundle:

```bash
python3 src/robot_map_toolkit/scripts/map_toolkit_cli.py \
  --maps-root maps_release \
  --building-id building_1 \
  --floor-id floor_1 \
  --flat-maps-dir /workspaces/isaac_ros-dev/maps \
  --flat-map-name test-16
```

The promoted bundle is the input expected by `robot_floor_manager`.
