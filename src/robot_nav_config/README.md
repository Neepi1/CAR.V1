# robot_nav_config

Fixed Nav2 and canonical TF defaults for the first production-oriented scaffold.

## Parameters

- Planner: `nav2_smac_planner/SmacPlanner2D`
- Optional planner profile reserved for `SmacHybrid`
- Controller: `nav2_mppi_controller::MPPIController`
- Fallback controller: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
- Smoother: `nav2_smoother::SimpleSmoother`
- Progress checker: `nav2_controller::PoseProgressChecker`
- Goal checker: `nav2_controller::SimpleGoalChecker` with `stateful=false`
  so XY remains checked while terminal yaw is being satisfied.
- Local obstacle topic: `/scan`
- `suppress_third_party_tf`: true

## Notes

- This package owns the policy artifacts, not the live TF publication.
- Wrapper packages must consume this policy and keep non-canonical TF disabled by default.
- `config/local_reuse_sources.yaml` records the current validated `D:/codespace/car` and Jetson workspace inputs used by the wrapper configs.
- The same reuse manifest now also records the Dockerfile and dashboard entrypoints used by the temporary `NJRH-car` runtime on Jetson.
