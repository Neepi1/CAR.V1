# robot_nav_config

Fixed Nav2 and canonical TF defaults for the first production-oriented scaffold.

## Parameters

- Planner: `nav2_smac_planner/SmacPlanner2D`
- Optional planner profile reserved for `SmacHybrid`
- Controller: `nav2_mppi_controller::MPPIController`
- Fallback controller: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
- Ranger-matched MPPI: the controller keeps the 1.2 m/s field speed target,
  but measured chassis response is handled by `velocity_smoother`
  in open-loop ramp mode (`smoothing_frequency=30.0`,
  `max_accel=[0.55, 0.0, 0.45]`,
  `max_decel=[-0.70, 0.0, -0.45]`, `max_velocity=[1.20, 0.0, 0.70]`)
  and 1.5 m near-goal critic windows. The primary controller runs at 15 Hz with
  `model_dt=0.0666666667`, `vx_std=0.30`, and `wz_std=0.32` so ordinary
  building navigation uses less jagged sample commands without changing the
  Nav2 plugin types. Closed-loop velocity-smoother feedback is
  avoided on Ranger Mini 3 because odom=0 plus the chassis motion-mode deadband
  can pin angular output at the single-cycle acceleration increment before the
  chassis starts moving. Normal API point navigation publishes a
  distance-based `/speed_limit` so the robot steps down from 1.20 m/s outside
  2.4 m to 0.08 m/s inside 0.35 m, then restores the cruise limit when the
  Nav2 task exits. It must not publish `0.0` as a clear signal, because
  controller-server treats that as a stop limit.
- Field behavior trees: `navigate_to_pose.xml` runs
  `ComputePathToPose -> FollowPath`, and `navigate_through_poses.xml` runs
  `ComputePathThroughPoses -> FollowPath`. The Savitzky-Golay `SmoothPath` BT
  pass and BT `Wait` recovery were rolled back after field delivery tests showed
  a mid-route pure-spin/replan chain. `controller_server.failure_tolerance=3.0`
  handles very short MPPI no-trajectory intervals; longer blocked corridors
  should be handled by explicit API/mission wait or retry policy.
- BT server waits: `default_server_timeout=5000` and
  `wait_for_service_timeout=8000` keep Nav2 from aborting a valid goal when
  Jetson/Fast DDS/costmap load delays internal action or service acknowledgement.
- Smoother server: `nav2_smoother::SimpleSmoother` remains configured for
  lifecycle compatibility and tooling; the active field BT does not call
  `SmoothPath`.
- Progress checker: `nav2_controller::PoseProgressChecker`
- Goal checker: `nav2_controller::SimpleGoalChecker` with `stateful=false`
  so terminal XY and yaw are rechecked together after dynamic-obstacle
  interruption or near-goal avoidance drift. The ordinary pose-required
  tolerances are `xy_goal_tolerance=0.06` and `yaw_goal_tolerance=0.05`.
- RotationShim handoff: `angular_dist_threshold=0.45` and
  `angular_disengage_threshold=0.075`. This keeps ordinary path-entry and
  mid-route Ackermann turns smooth instead of stopping for small heading
  changes, but once a pure-yaw shim starts it turns closer to the target before
  handing back to MPPI. Ranger spin-tail is handled downstream by
  `robot_safety`, which waits for actual `/wheel/odom` yaw rate to settle
  before releasing the next linear command after a pure spin.
- Local obstacle topic: `/scan`
- `suppress_third_party_tf`: true

## Notes

- This package owns the policy artifacts, not the live TF publication.
- Wrapper packages must consume this policy and keep non-canonical TF disabled by default.
- `config/local_reuse_sources.yaml` records the current validated `D:/codespace/car` and Jetson workspace inputs used by the wrapper configs.
- The same reuse manifest now also records the Dockerfile and dashboard entrypoints used by the temporary `NJRH-car` runtime on Jetson.
