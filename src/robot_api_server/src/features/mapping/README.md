# Mapping

This directory is one complete API-facing mapping module, split internally by
lifecycle responsibility:

- `mapping_feature_module` is the aggregate owner of `MappingModule` and its
  floor/maps/elevator/navigation/teleop handoff ports. It preserves the same
  cancellation, zero-command and runtime-context effects while removing that
  wiring from the API root.
- `mapping_module` is the deep public boundary. It owns mapping HTTP dispatch,
  the async start worker, process/start-job collaboration, live `/map`
  subscription and cache lifetime, exact navigation `/scan` owner restoration,
  save/stop behavior, status JSON, and the compact snapshot consumed by
  teleop/status callers.
- `mapping_configuration_module` uniquely declares all 10 mapping API
  parameters, preserves the existing timeout/freshness bounds, and projects
  the maps-owned release/runtime roots into one complete `MappingModuleConfig`.
  It performs no process, scan-owner, asset, or ROS side effect.

- `map_asset_writer` converts a completed 2D occupancy grid into released map
  assets: Nav YAML/PGM, localizer PNG, neutral filters, and the asset report.
- `runtime/mapping_process_runtime` owns the mapping launcher PID/process group,
  mapping-marked residual cleanup, graceful escalation, and restoration of the
  temporary LiDAR RPS/XPS state.
- `runtime/mapping_start_job` owns the thread-safe public state of the one active
  asynchronous mapping-start transaction.

The aggregate supplies only explicit cross-domain ports. The module
retains the established commercial sequence: cancel navigation, stop navigation
mode services, clear runtime map context, then launch mapping. Saving a map
still writes one inactive immutable bundle and never activates it for
navigation. Stopping still restores and proves the exact canonical `/scan`
owner before reporting success.

The public seam is intentionally small: `handle_http`, `status_json`,
`snapshot`, `set_live_map_page_active`, and `shutdown`. Tests in
`test/features/mapping/test_mapping_module.cpp` exercise that boundary, while
the runtime tests continue to cover private process classification and the
start-job transaction.

This module does not change FAST-LIO2/JT128 parameters, pointcloud or LaserScan
geometry/QoS/timestamps, DDS, canonical TF ownership, or the final velocity
chain.
