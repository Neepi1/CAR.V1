# robot_pgo_mapping

Loop-closure or PGO backend wrapper with formalized `mapping_result` output.

## Parameters

- `frontend_artifact_dir`: standardized frontend input root
- `output_dir`: defaults to `mapping_result`
- `loop_report_name`: defaults to `loop_report.json`
- `local_config`: reuses `D:/codespace/car/ros2_ws/src/fastlio_pgo/config/pgo.yaml`
- upstream car config uses `slam_map` and `camera_init`; those frames stay internal and must not be merged into the canonical navigation TF tree

## Output Contract

- `optimized_map.pcd`
- `optimized_trajectory.csv`
- `metadata.yaml`
- `loop_report.json`
- `optimized_trajectory.csv` should be exported in a repository-owned format that can be consumed by `robot_occupancy_builder release_rebuild`
