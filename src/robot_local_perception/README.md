# robot_local_perception

Local obstacle cloud filter for the canonical Nav2 local costmap input.

## Reuse Baseline

- validated reference params: `D:/codespace/car/nav2_test/params/jt128_nav_cloud_preprocessor.yaml`
- validated reference launch: `D:/codespace/car/nav2_test/jt128_nav_sensing.launch.py`
- current repository implementation uses a repo-owned C++ node for Jetson runtime stability; the Python fallback path has been removed

## Canonical Contract

- input cloud: `/_internal/lidar_points_local` in production component mode
- output cloud: `/perception/obstacle_points`
- clearing cloud: `/perception/clearing_points`
- output frame: `base_link`
- mode topic: `/robot_mode`
- supported modes: `NORMAL`, `RAMP`, `ELEVATOR_WAIT`, `DOORWAY`

The node transforms the incoming cloud into `base_link` first, then applies:

- range filtering
- height filtering
- forward-sector azimuth filtering
- self-mask removal
- front-mask removal
- optional voxel speckle suppression
- latest-frame processing with a fixed publish rate
- point sampling and capped output size to keep Jetson runtime stable
- synthetic clearing-ray endpoint generation for local costmap free-space clearing, published at a decimated cadence

This keeps the Nav2 local costmap source canonical without maintaining a second raw pointcloud transform path inside local perception. The production runtime loads this package in the same `component_container_mt` as `pointcloud_axis_remap_node` and consumes the hidden `/_internal/lidar_points_local` branch with best-effort input QoS and a latest-only queue (`input_qos_depth=1`). That branch is produced by the single raw-ingress remap component, not by a second subscriber on `/lidar_points`, so the FAST-LIO2 full-density trunk keeps low fan-out and the local obstacle path avoids a public full-size DDS stream. Local perception uses an identity input rotation because JT128 raw-to-canonical remap has already been applied by `pointcloud_axis_remap_node`, then transforms into `base_link` before local obstacle filtering. Runtime keeps `input_transform_use_latest=true` because `lidar_link -> base_link` is static sensor extrinsic; obstacle cloud header stamps are still preserved, but the hot path does not wait for a historical TF lookup for every input frame. Runtime uses callback-driven processing (`process_on_callback=true`) so obstacle marking follows the JT128 frame cadence instead of merging fresh frames behind a timer. Output obstacle and clearing clouds remain best-effort/depth 1 for Nav2.

The hot path fuses the configured input rotation and the TF transform into one 3x4 matrix per frame, then applies that matrix directly while iterating over the PointCloud2 byte buffer. Obstacle output is written directly as a PCL-compatible PointCloud2 layout instead of building an intermediate PCL cloud and calling `pcl::toROSMsg`. NORMAL-mode voxel speckle suppression uses packed voxel keys in an `unordered_map` rather than a tree `map` of tuples, keeping the filter cost bounded enough for the JT128 frame cadence. Obstacle clouds are published before clearing work is handed to a latest-only worker, so periodic clearing serialization cannot delay the safety-critical marking update in the subscriber callback.

For source-timing diagnostics, the runtime defaults to `restamp_to_now=false`, `restamp_to_latest_tf=false`, and `require_output_stamp_tf=false`. The output clouds are already transformed into `base_link`, but they preserve the original pointcloud acquisition stamp instead of hiding JT128/DDS/perception latency behind current-stamped obstacle clouds. Nav2 startup and readiness checks must therefore allow their TF buffers to accumulate enough `odom <- base_link` history before judging these clouds transformable; a cold TF listener can otherwise report that a truthful cloud stamp is earlier than all data in its empty cache.

Startup keeps TF validation inside the C++ node rather than shell probes. The node enforces `require_startup_tf_ready=true` with `startup_tf_warmup_sec=1.0`, dropping early input frames until its own TF listener can resolve both the sensor static transform and the odom transform used by Nav2. This preserves original cloud stamps while preventing cold-cache point clouds from being published into the local costmap.

The runtime Nav2 voxel layer consumes `/perception/obstacle_points` for marking and `/perception/clearing_points` for raytrace clearing. Both clouds are produced by this package after transforming the hidden `/_internal/lidar_points_local` branch into `base_link`; raw JT128 clouds and the FAST-LIO2 `/lidar_points` trunk are not wired directly into the local costmap. This split is intentional: a sparse obstacle-only cloud cannot clear cells after a moving obstacle leaves, because it lacks free-space rays. The node publishes the obstacle cloud before building synthetic clearing endpoints, so clearing work cannot delay safety-critical obstacle marking.

`/perception/clearing_points` is not a second obstacle source. In field runtime it publishes synthetic max-range clearing endpoints for each horizontal azimuth bin and each local costmap voxel height layer, with the ray origin taken from the live `lidar_link` pose in `base_link`. Real returns update the farthest observed range per bin; bins without a return still publish a max-range endpoint so Nav2 can clear stale voxels behind moved obstacles.

## Mode Profiles

- `NORMAL`: reuse the validated broad JT128 nav cloud settings from `car`
- `RAMP`: relax height limits and widen azimuth slightly for slope entries
- `ELEVATOR_WAIT`: tighten range and enable voxel speckle removal for near-field waiting zones
- `DOORWAY`: tighten frontal sector and enable voxel speckle removal for narrow transitions

## Notes

- This package only filters local obstacle clouds. It does not publish TF and does not own any Nav2 lifecycle or command arbitration logic.
- Jetson runtime normally starts local perception through `scripts/jetson/runtime_overlay/launch/pointcloud_perception_pipeline.launch.py` so it shares a component container with the pointcloud remap component and can use intra-process transport. `scripts/jetson/runtime_overlay/scripts/run_local_perception.sh` remains as a standalone fallback when `NJRH_JT128_USE_POINTCLOUD_PIPELINE_CONTAINER=false`.
- For field runtime, local perception now runs at the JT128 input cadence target (`processing_rate_hz=20.0`, `point_sample_stride=1`, `max_filtered_points=12000`) so obstacle updates are not hidden behind source-side throttling. NORMAL marking is capped at `5.50 m`; clearing rays use `clearing.publish_every_n=3`, `clearing.point_sample_stride=1`, `clearing.virtual_rays.angular_resolution_deg=0.75`, and range steps `[0.50, 1.00, 2.00, 3.50, 5.50, 8.00]` because reliable clearing depends on angular and vertical voxel coverage rather than dense obstacle marking.
- Hardware validation still needs a navigation run while recording `/tf`, `/local_state/odometry`, `/perception/obstacle_points`, `/perception/clearing_points`, `/local_costmap/costmap`, and the navigation runtime log to confirm the TF-backed stamp preserves obstacle throughput and stops continuous local-costmap MessageFilter drops after the startup TF gate.
