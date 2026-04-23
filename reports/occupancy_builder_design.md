# Occupancy Builder Design

## Scope

This document defines the repository-owned `robot_occupancy_builder` package requested for the v3 stack update.

Design constraints:

- Do not modify FAST-LIO2 core source code.
- Reuse local `car` assets first.
- Keep the current navigation main chain stable until the design is confirmed.
- `robot_local_perception` remains the only local costmap obstacle source.
- The live draft map is for operator feedback and offline asset production input only, not for direct local costmap feeding.

## Requested Modes

### `live_draft`

Contract:

- Subscribe to JT128 point cloud.
- Subscribe to `/mapping/frontend_pose`.
- Publish `/mapping/draft_map`.

Purpose:

- Provide a real-time 2D occupancy draft during mapping so the operator can see map closure quality early.
- Keep the live draft aligned with the current frontend pose stream instead of waiting for backend optimization.

### `release_rebuild`

Contract:

- Input raw bag.
- Input optimized trajectory from the PGO backend.
- Rebuild the official 2D occupancy products from the raw sensor history plus optimized poses.
- Output `nav_map` and `localizer_map` from the same occupancy intermediate result.

Purpose:

- Produce the official release assets after mapping is finished.
- Avoid the invalid shortcut of projecting a final saved point cloud directly into the release 2D map.

## Reuse Plan From Local Car Assets

### Reuse directly

- `D:\codespace\car\scripts\projected_occupancy_mapper.py`
  - reuse as the reference for raw `PointCloud2` parsing
  - reuse as the reference for free-space ray tracing
  - reuse as the reference for log-odds style occupancy accumulation
- `D:\codespace\car\ros2_ws\src\jt128_nav_tools\src\terrain_map_builder_node.cpp`
  - reuse as the reference for local ground estimation by cell neighborhood
  - reuse as the reference for obstacle extraction from JT128 in vehicle frame
- `D:\codespace\car\nav2_test\params\jt128_nav_cloud_preprocessor.yaml`
  - reuse range, height, azimuth, self-mask, and front-mask parameter conventions
- `D:\codespace\car\scripts\export_pgo_map_2d.py`
  - reuse only for post-processing/export heuristics

### Do not reuse directly

- Do not reuse `projected_occupancy_mapper.py` accumulated-cloud projection as the release path.
- Do not reuse `export_pgo_map_2d.py` direct saved-PLY rasterization as the official release path.
- Do not reuse the historical `/projected_map` topic as the new canonical draft map contract.

## Package Boundary

`robot_occupancy_builder` only owns:

- live 2D draft occupancy generation
- offline release 2D rebuild from raw bag + optimized trajectory
- shared occupancy intermediate generation
- `nav_map` and `localizer_map` asset emission from the same intermediate result

`robot_occupancy_builder` does not own:

- TF publication
- local obstacle filtering for local costmap
- global localization
- Nav2 runtime
- FAST-LIO2 frontend internals
- PGO backend internals

## Inputs And Outputs

### Inputs

Live mode:

- point cloud topic from `robot_hesai_jt128`
- `/mapping/frontend_pose`
- JT128 filtering and classification parameters

Release mode:

- raw bag path
- optimized trajectory file or canonical trajectory export from `robot_pgo_mapping`
- occupancy builder release parameters
- output asset root

### Outputs

Live mode:

- `/mapping/draft_map` (`nav_msgs/OccupancyGrid`)
- optional debug topics:
  - `/mapping/draft_map/ground_points`
  - `/mapping/draft_map/ramp_points`
  - `/mapping/draft_map/obstacle_points`
  - `/mapping/draft_map/status`

Release mode:

- `nav/nav_map.yaml`
- `nav/nav_map.pgm`
- `localizer/localizer_map.png`
- `localizer/localizer_params.yaml`
- `reports/asset_report.json`
- optional intermediate debug artifacts:
  - `reports/rebuild_stats.json`
  - `intermediate/occupancy_layers.npz`
  - `intermediate/trajectory_aligned.csv`

## Message And Topic Contract

### Cloud input

Preferred repository-owned input:

- default: `robot_hesai_jt128` output topic already standardized in this repository

Compatibility:

- allow remap from historical `car` topic `/lidar_points`

### Pose input

- fixed live input: `/mapping/frontend_pose`

Rationale:

- the builder should consume the repository-owned frontend pose contract, not infer pose from third-party TF.
- this avoids polluting the canonical navigation TF tree with mapping-internal frames.

### Draft map output

- fixed live output: `/mapping/draft_map`

Constraints:

- do not connect `/mapping/draft_map` to `robot_local_perception`
- do not use `/mapping/draft_map` as the local costmap obstacle layer

## Core Algorithm

### 1. Pre-filtering

Apply JT128 prefiltering using the validated `car` conventions:

- range gate
- broad height gate
- azimuth gate
- self-mask
- front-mask

These filters run in the vehicle frame and remove obvious self points and rear-sector noise before occupancy update.

### 2. Per-scan semantic classification

Each scan is classified into three classes:

- `ground`
- `ramp`
- `obstacle`

Baseline method:

1. transform points into the builder working frame using the incoming pose
2. bin points into a 2D terrain grid
3. estimate neighborhood ground height with a low quantile, following the `terrain_map_builder` approach
4. compute `relative_z = point_z - local_ground_z`
5. classify:
   - `ground`: near-zero relative height and low local slope
   - `ramp`: continuous traversable elevation change below configured slope / step thresholds
   - `obstacle`: relative height above configured obstacle threshold or non-traversable slope discontinuity

The exact thresholds must stay in YAML so they can be tuned per site.

### 3. Free-space ray tracing

For each retained beam:

- trace from the sensor origin or pose origin through the 2D grid
- decrement traversed cells as free-space evidence
- stop at the endpoint cell

Rules:

- `ground` and `ramp` hits mainly reinforce free traversability along the ray
- `obstacle` hits reinforce the endpoint occupied evidence
- cells behind an obstacle endpoint are not marked free by that beam

### 4. Occupied endpoint accumulation

Maintain separate accumulation channels:

- `ground_hits`
- `ramp_hits`
- `obstacle_hits`
- `free_space_hits`

Primary occupancy state comes from log-odds or probability accumulation:

- free evidence from rays
- occupied evidence from obstacle endpoints

Secondary semantic state is derived from the per-class accumulators for later asset export.

### 5. Temporal accumulation model

Live mode:

- bounded log-odds
- optional short decay for unstable draft artifacts
- publish at fixed rate

Release mode:

- no short-horizon decay
- full pass over the raw bag
- deterministic rebuild using optimized trajectory

### 6. Post-processing

Required post-processing:

- hole fill for small free-space gaps
- obstacle speckle removal
- enclosure / unknown region inference
- small ramp-region cleanup
- binary occupancy thresholding for Nav2 asset export

Important:

- `nav_map.pgm` and `localizer_map.png` must be produced from the same occupancy intermediate, not from two unrelated conversions.

## Live Draft Pipeline

```text
robot_hesai_jt128
  -> raw cloud
  -> robot_occupancy_builder(live_draft)
     -> semantic layers + log-odds grid
     -> /mapping/draft_map
```

Characteristics:

- low latency
- visually useful during mapping
- acceptable to be slightly noisy
- not the authoritative release asset

## Release Rebuild Pipeline

```text
raw bag + optimized trajectory
  -> robot_occupancy_builder(release_rebuild)
     -> replay every raw JT128 scan at optimized pose
     -> rebuild shared occupancy intermediate
     -> emit nav_map + localizer_map + reports
```

Key rule:

- the release rebuild consumes the original scan history plus the optimized trajectory.
- it does not consume only the final saved PCD or PLY as its source of truth.

## Integration With Existing Packages

### `robot_fastlio_mapping`

- expose or confirm canonical `/mapping/frontend_pose`
- do not add new TF publication

### `robot_pgo_mapping`

- export optimized trajectory in a repository-owned format that `release_rebuild` can consume
- keep `loop_report.json`

### `robot_map_toolkit`

- own final asset directory layout and asset validation
- call into `robot_occupancy_builder release_rebuild` rather than directly projecting the final point cloud

### `robot_nav_config`

- keep local costmap obstacle source as `/perception/obstacle_points`
- do not bind draft occupancy into the local costmap

## Proposed Package Layout

```text
src/robot_occupancy_builder/
  README.md
  package.xml
  CMakeLists.txt
  launch/
    live_draft.launch.py
    release_rebuild.launch.py
  config/
    live_draft.yaml
    release_rebuild.yaml
  scripts/
    occupancy_builder_live_node.py
    occupancy_builder_release_node.py
    rebuild_from_bag.py
    occupancy_postprocess.py
  test/
    test_live_draft_contract.py
    test_release_rebuild_contract.py
    test_same_intermediate_exports.py
```

## Required Tests

- draft map topic contract test
- free-space ray tracing unit test
- semantic class split test for `ground / ramp / obstacle`
- log-odds accumulation test
- release rebuild reproducibility test
- `nav_map` and `localizer_map` same-intermediate consistency test
- no-local-costmap-coupling test

## Phase Plan Before Mainline Integration

### Phase A: design and reports

- complete car reuse scan
- complete TF boundary audit
- complete occupancy builder design

### Phase B: package scaffold

- add `robot_occupancy_builder`
- add params, README, tests, and no-op launch contracts

### Phase C: live draft implementation

- implement JT128 + `/mapping/frontend_pose` -> `/mapping/draft_map`
- add debug layers

### Phase D: release rebuild implementation

- implement raw bag + optimized trajectory rebuild
- connect output into `robot_map_toolkit`

### Phase E: operator wiring

- expose live draft preview and release asset generation in the operator workflow

## Current Recommendation

- Confirm this design before any large changes to the current navigation main chain.
- After confirmation, implement `robot_occupancy_builder` as a new repository-owned package and keep the existing Nav2 runtime chain unchanged until the package contract is verified.
