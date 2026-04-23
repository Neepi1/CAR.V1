# Occupancy Builder Workflow

## Scope

This document describes the repository-owned `robot_occupancy_builder` workflow added for the v3 mapping update.

The package is intentionally isolated from the current Nav2 local costmap chain:

- live draft output is `/mapping/draft_map`
- local costmap obstacle source remains `/perception/obstacle_points`
- the package publishes no TF

## Live Draft

Launch:

```bash
ros2 launch robot_occupancy_builder live_draft.launch.py
```

Default contract:

- input cloud: `/sensors/lidar/points_raw`
- input pose: `/mapping/frontend_pose`
- output map: `/mapping/draft_map`
- status topic: `/mapping/draft_map/status`

Notes:

- the node prefers TF lookup for `cloud_frame -> base_link` using the canonical static tree
- if TF lookup is unavailable, it falls back to configured sensor extrinsics
- the draft map is for operator feedback during mapping, not for direct local costmap wiring
- Jetson runtime tuning owner is `scripts/jetson/runtime_overlay/config/occupancy_builder_live.yaml`
- the current draft profile is intentionally biased toward thinner walls: less occupied dilation/closing, a higher occupied threshold, and stricter obstacle-vs-ground separation

## Release Rebuild

Launch:

```bash
ros2 launch robot_occupancy_builder release_rebuild.launch.py
```

Or invoke the helper directly:

```bash
ros2 run robot_occupancy_builder rebuild_from_bag.py \
  --raw-bag-path /path/to/raw_bag \
  --optimized-trajectory-csv /path/to/optimized_trajectory.csv \
  --output-root /path/to/maps/building_1/floor_1
```

Release outputs:

- `nav/nav_map.yaml`
- `nav/nav_map.pgm`
- `localizer/localizer_map.png`
- `localizer/localizer_params.yaml`
- `reports/asset_report.json`
- `intermediate/occupancy_layers.npz`

## Current Limits

- Jetson field runtime now uses `slam_toolbox` as the default live 2D `/map` producer for operator-facing mapping; `robot_occupancy_builder` remains in-repo for formal rebuild / compatibility work instead of the default live draft source.
- A repository-owned release rebuild compatibility script exists at `scripts/jetson/runtime_overlay/scripts/release_rebuild_compat.py`; it writes formal assets under `maps_release/<name>/...` and compatibility `maps/<name>.yaml/.pgm` files for the current dashboard/Nav2 picker.
- The preferred formal save path requires `raw bag + optimized trajectory csv`; current Jetson runtime wiring still needs a field-ready producer for those inputs before the legacy `export_pgo_map_2d.py` fallback can be fully removed.
- `robot_fastlio_mapping` now exposes a repository-owned `/mapping/frontend_pose` contract in mock mode, but the real upstream FAST-LIO integration still needs runtime validation on Jetson.
- `robot_pgo_mapping` now exports a mock `optimized_trajectory.csv` with a repository-owned `yaw` column, but the real backend trajectory export format still needs validation against the actual PGO runtime.
