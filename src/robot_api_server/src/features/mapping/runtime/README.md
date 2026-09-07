# Mapping Runtime

This internal directory contains the process-lifetime collaborators used only
by the public `features/mapping/mapping_module` boundary:

- `mapping_process_runtime` owns the marked mapping launcher/process group,
  residual-owner classification, bounded stop escalation, and temporary LiDAR
  RPS/XPS restoration.
- `mapping_start_job` owns the thread-safe state of the single asynchronous
  App mapping-start transaction.

Neither component exposes HTTP routes or changes mapping, scan, pointcloud,
DDS, TF, or navigation parameters. Cross-domain orchestration remains behind
`MappingModule` ports.
