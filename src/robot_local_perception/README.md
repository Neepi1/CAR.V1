# robot_local_perception

Local obstacle cloud filter for the canonical Nav2 local costmap input.

## Reuse Baseline

- validated reference params: `D:/codespace/car/nav2_test/params/jt128_nav_cloud_preprocessor.yaml`
- validated reference launch: `D:/codespace/car/nav2_test/jt128_nav_sensing.launch.py`
- current repository implementation uses a repo-owned C++ node for Jetson runtime stability; the Python fallback path has been removed

## Canonical Contract

- input cloud: `/lidar_points` in production standalone mode
- output cloud: `/perception/obstacle_points`
- clearing cloud: `/perception/clearing_points`
- status: `/perception/local_perception_status`
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

This keeps the Nav2 local costmap source canonical without maintaining a second raw pointcloud transform path inside local perception. The production runtime starts this package as a standalone process that subscribes to `/lidar_points` with best-effort input QoS and a latest-only queue (`input_qos_depth=1`). It is intentionally separated from `pointcloud_axis_remap_node`, so heavy local obstacle filtering cannot share the raw-ingress callback pressure domain or slow the canonical `/lidar_points` publisher. Local perception uses an identity input rotation because JT128 raw-to-canonical remap has already been applied by `pointcloud_axis_remap_node`, then transforms into `base_link` before local obstacle filtering. Runtime keeps `input_transform_use_latest=true` because `lidar_link -> base_link` is static sensor extrinsic; obstacle cloud header stamps are still preserved, but the hot path does not wait for a historical TF lookup for every input frame. Runtime uses timer-driven latest-frame processing (`process_on_callback=false`, `processing_rate_hz=15.0`) so obstacle filtering cannot force every JT128 frame through the local perception workload. Output obstacle and clearing clouds remain best-effort/depth 1 for Nav2.

The node also publishes `/perception/local_perception_status` as a lightweight C++ runtime status string. It reports input callback rate, accepted cloud rate, input interarrival timing, input cloud size, subscription QoS, processing-timer rate, processed cloud rate, obstacle and clearing publish rates, skip counters, and last-frame timing so field validation can distinguish subscriber delivery, processing, and publish gating without adding Python probes or changing QoS.

The hot path fuses the configured input rotation and the TF transform into one 3x4 matrix per frame, then applies that matrix directly while iterating over the PointCloud2 byte buffer. Obstacle output is written directly as a PCL-compatible PointCloud2 layout instead of building an intermediate PCL cloud and calling `pcl::toROSMsg`. NORMAL-mode voxel speckle suppression uses packed voxel keys in an `unordered_map` rather than a tree `map` of tuples, keeping the filter cost bounded enough for the JT128 frame cadence. Obstacle clouds are published before clearing work is handed to a latest-only worker, so periodic clearing serialization cannot delay the safety-critical marking update in the subscriber callback.

For source-timing diagnostics, the runtime defaults to `restamp_to_now=false`, `restamp_to_latest_tf=false`, and `require_output_stamp_tf=false`. The output clouds are already transformed into `base_link`, but they preserve the original pointcloud acquisition stamp instead of hiding JT128/DDS/perception latency behind current-stamped obstacle clouds. Nav2 startup and readiness checks must therefore allow their TF buffers to accumulate enough `odom <- base_link` history before judging these clouds transformable; a cold TF listener can otherwise report that a truthful cloud stamp is earlier than all data in its empty cache.

Startup keeps TF validation inside the C++ node rather than shell probes. The node enforces `require_startup_tf_ready=true` with `startup_tf_warmup_sec=1.0`, dropping early input frames until its own TF listener can resolve both the sensor static transform and the odom transform used by Nav2. This preserves original cloud stamps while preventing cold-cache point clouds from being published into the local costmap.

The runtime Nav2 voxel layer consumes `/perception/obstacle_points` for marking and `/perception/clearing_points` for raytrace clearing. Both clouds are produced by this package after transforming `/lidar_points` into `base_link`; raw vendor JT128 clouds are not wired directly into the local costmap. This split is intentional: a sparse obstacle-only cloud cannot clear cells after a moving obstacle leaves, because it lacks free-space rays. The node publishes the obstacle cloud before building synthetic clearing endpoints, so clearing work cannot delay safety-critical obstacle marking.

`/perception/clearing_points` is not a second obstacle source. In field runtime it publishes synthetic max-range clearing endpoints for each horizontal azimuth bin and each local costmap voxel height layer, with the ray origin taken from the live `lidar_link` pose in `base_link`. Real returns update the farthest observed range per bin; bins without a return still publish a max-range endpoint so Nav2 can clear stale voxels behind moved obstacles.

## Mode Profiles

- `NORMAL`: reuse the validated broad JT128 nav cloud settings from `car`
- `RAMP`: relax height limits and widen azimuth slightly for slope entries
- `ELEVATOR_WAIT`: tighten range and enable voxel speckle removal for near-field waiting zones
- `DOORWAY`: tighten frontal sector and enable voxel speckle removal for narrow transitions

## Notes

- This package only filters local obstacle clouds. It does not publish TF and does not own any Nav2 lifecycle or command arbitration logic.
- Jetson runtime normally starts local perception through `scripts/jetson/runtime_overlay/scripts/run_local_perception.sh` as a standalone process. `scripts/jetson/runtime_overlay/launch/pointcloud_perception_pipeline.launch.py` remains available only as an explicit legacy/debug mode.
- For field runtime, local perception uses timer-driven latest-frame processing (`processing_rate_hz=15.0`, `process_on_callback=false`, `point_sample_stride=1`, `max_filtered_points=12000`) so `/lidar_points` remains the priority trunk. NORMAL marking is capped at `5.50 m`; clearing rays use `clearing.publish_every_n=4`, `clearing.point_sample_stride=2`, `clearing.max_points=15000`, `clearing.virtual_rays.angular_resolution_deg=1.0`, and range steps `[0.50, 1.00, 2.00, 3.50, 5.50, 8.00]` to keep clearing useful without dominating the lidar pipeline.
- Hardware validation still needs a navigation run while recording `/tf`, `/local_state/odometry`, `/perception/obstacle_points`, `/perception/clearing_points`, `/local_costmap/costmap`, and the navigation runtime log to confirm the TF-backed stamp preserves obstacle throughput and stops continuous local-costmap MessageFilter drops after the startup TF gate.
