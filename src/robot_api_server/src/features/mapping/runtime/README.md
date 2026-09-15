# Mapping Runtime

This internal directory contains the process-lifetime collaborators used only
by the public `features/mapping/mapping_module` boundary:

- `mapping_process_runtime` owns the marked mapping launcher/process group,
  residual-owner classification, bounded stop escalation, and temporary LiDAR
  RPS/XPS restoration.
- `mapping_start_job` owns the thread-safe state of the single asynchronous
  App mapping-start transaction.
- `mapping_save_job` owns one asynchronous save and its persistent receipts in
  `runtime_maps_dir/save_jobs`. Its status separates committed assets from
  confirmed shutdown; request retries reuse the same ID rather than writing
  another map. An interrupted receipt becomes `recovery_required`, with no
  automatic stop or replay against a newly started mapping session.

Neither component exposes HTTP routes or changes mapping, scan, pointcloud,
DDS, TF, or navigation parameters. Cross-domain orchestration remains behind
`MappingModule` ports.

Start/stop use an operation mutex; snapshots use a short state mutex. A stopping
process is not concurrently reaped or rediscovered by readers. The original
stop budget remains. `test_mapping_stop_concurrency` uses fake OS boundaries
and never enumerates/signals production PIDs. See the workspace document
`docs/mapping_startup_latency.md` for deployment and hardware acceptance.
