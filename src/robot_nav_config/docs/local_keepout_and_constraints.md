# Local keepout and MPPI constraints (2026-09-16)

Geometry update, 2026-09-19: the current local candidate uses `0.75 m` rather
than the historical `0.60 m` radii described below, with the confirmed larger
body. Filter order and cost scaling remain unchanged. See
[body geometry alignment](body_geometry_alignment.md); not deployed.

## Scope

Both source and Jetson overlay `nav2.yaml` enable the installed Humble
`ConstraintCritic` in ordinary `FollowPath`, with native defaults: enabled,
power 1, weight 4.0. It scores predicted velocity/turning-radius violations;
the measured Ranger model still derives from `AckermannMotionModel`.
Existing output constraints, all other critics, speed limits and the 0.81 m
minimum radius remain unchanged. This also affects predock transit and any
distant hall approach using this same FollowPath, not contact docking.

The local rolling costmap keeps `odom`, `/scan`, the original obstacle and
inflation plugins, footprint/padding, resolution and update rate. Its new
ordered filters are:

1. Native `KeepoutFilter`, consuming the existing
   `/costmap_filter_info/keepout` and its existing mask publisher.
2. Native `InflationLayer`, named `keepout_inflation_layer`, using the same
   0.60 m / 6.0 inflation parameters as `local_inflation_layer`.

Humble applies filters after ordinary plugins. Inflation only in the plugin
list therefore cannot inflate newly marked keepout cells. The post-filter
inflation enables native MPPI ObstaclesCritic's center-cost shortcut to request
full-footprint checks near a forbidden region. It is not a second physical
footprint, an extra 0.60 m hard-stop distance, or doubled inflation distance.
The neutral-mask equivalence test checks that existing wall inflation is unchanged.

The same local costmap is consumed by ordinary local path repair, fallback
and collision-checked terminal control. Global filtering is unchanged.
Elevator post-call unchecked test legs retain their commissioned policy;
no controller/BT/permit/safety change, arm API change or new startup gate is made.
The normal `map -> odom` transform maps mask coordinates into the rolling map;
native behavior for missing filter data/TF is not redesigned by this change.

## Regression and deployment

- Python contracts check both configuration copies, critic settings, shared
  topic, filter order, matching inflation, and unchanged geometry/motion policy.
- `test_keepout_and_constraints` uses installed pluginlib critics/filters, a
  synthetic obstacle input, transient-local masks and an in-memory TF buffer.
  It checks stationary/near-zero finite scores, forbidden velocity/curvature,
  body-edge collision with a non-lethal center, rotated footprint, mask
  add/move/delete, transformed masks, neutral equivalence and update timings.
- The same body-edge case is run without post-filter inflation as a negative
  control. Existing real Ranger MPPI tests cover hot paths, speed limits and
  fully blocked no-control failure, with before/after timing observations.
- Run integration tests only in a separate network/device-isolated container.
  No production sensor stream, robot goal, localization or motion is required.
  Evidence: `/tmp/njrh_reports/constraint_keepout_20260916/`.

Deployment updates the two configs and installed package config only after
tests. It does not activate filters inside already configured processes.
Activation requires the user's App navigation-service stop/start or separately
authorized complete `njrh-runtime.service` restart; no individual node restart.
Verify loaded parameter values and a single active chain after activation.

Hardware acceptance still requires user-supervised ordinary navigation around
an App-authored forbidden line/polygon, add/delete and map switching, boundary
clearance and controller timing, then unchanged elevator/docking regressions.
Passing synthetic tests is not proof of real-world obstacle avoidance or smoother
motion. This phase does not claim to fix every repeated micro-motion.
