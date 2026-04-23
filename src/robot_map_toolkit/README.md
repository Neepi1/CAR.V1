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
