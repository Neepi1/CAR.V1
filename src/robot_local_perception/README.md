# robot_local_perception

Local obstacle cloud filter for the canonical Nav2 local costmap input.

## Reuse Baseline

- validated reference params: `D:/codespace/car/nav2_test/params/jt128_nav_cloud_preprocessor.yaml`
- validated reference launch: `D:/codespace/car/nav2_test/jt128_nav_sensing.launch.py`
- current repository implementation uses a repo-owned C++ node for Jetson runtime stability; the Python fallback path has been removed

## Canonical Contract

- input cloud: `/lidar_points`
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
- synthetic clearing-ray endpoint generation for local costmap free-space clearing

This keeps the Nav2 local costmap source canonical and prevents raw JT128 clouds from being wired into local costmap directly.

For Nav2 stability, the runtime defaults to `restamp_to_latest_tf=true`, `require_output_stamp_tf=true`, and `output_stamp_tf_target_frame=odom`. The output clouds are still in `base_link`, but their header stamp is aligned to the latest available `odom -> base_link` TF so local costmap message filters do not drop them as outside the TF cache. During startup, clouds are skipped until that TF is available instead of being published with wall-clock time.

The runtime Nav2 voxel layer consumes `/perception/obstacle_points` for marking and `/perception/clearing_points` for raytrace clearing. Both clouds are produced by this package after transforming raw JT128 points into `base_link`; raw JT128 clouds are not wired directly into the local costmap. This split is intentional: a sparse obstacle-only cloud cannot clear cells after a moving obstacle leaves, because it lacks free-space rays.

`/perception/clearing_points` is not a second obstacle source. In field runtime it publishes synthetic max-range clearing endpoints for each horizontal azimuth bin and each local costmap voxel height layer, with the ray origin taken from the live `lidar_link` pose in `base_link`. Real returns update the farthest observed range per bin; bins without a return still publish a max-range endpoint so Nav2 can clear stale voxels behind moved obstacles.

## Mode Profiles

- `NORMAL`: reuse the validated broad JT128 nav cloud settings from `car`
- `RAMP`: relax height limits and widen azimuth slightly for slope entries
- `ELEVATOR_WAIT`: tighten range and enable voxel speckle removal for near-field waiting zones
- `DOORWAY`: tighten frontal sector and enable voxel speckle removal for narrow transitions

## Notes

- This package only filters local obstacle clouds. It does not publish TF and does not own any Nav2 lifecycle or command arbitration logic.
- Jetson runtime executes `scripts/jetson/runtime_overlay/scripts/run_local_perception.sh`, which requires the compiled `local_perception_node` binary and fails fast if it has not been built yet.
- For field runtime, the default safety-oriented throttling is `processing_rate_hz=8.0`, `point_sample_stride=4`, `max_filtered_points=12000`. Clearing rays use `clearing.point_sample_stride=1` and `clearing.virtual_rays.angular_resolution_deg=0.5` because reliable clearing depends on angular and vertical voxel coverage rather than dense obstacle marking.
