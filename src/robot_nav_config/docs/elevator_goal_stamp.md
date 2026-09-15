# Elevator direct-path goal time

The fixed map-frame endpoint of `ElevatorScopedPlanner`'s
`unchecked_direct_path` branch has a zero `PoseStamped` timestamp. In TF this
means latest available transform, not a goal at the Unix epoch. The path
header and start-pose timestamp retain the planning time; endpoint coordinates,
orientation, bounded-distance validation and path geometry are unchanged.

Humble ControllerServer stores `path.poses.back()` as its completion target and
transforms it into the local costmap's `odom` frame before SimpleGoalChecker.
The elevator controller already transforms its canonical map goal using stamp
zero. Keeping a planning-time stamp on the server's endpoint instead makes the
controller and outer completion checker use different targets after map->odom
corrections. This change aligns the two without adding an API acceptance step.

Evidence: `elevator_panel_20260910T135907Z_panel_01_45ar94de`, panel FollowPath
success at 2026-09-10 13:59:38.793053 UTC. Recorded TF replay gives 0.058056 m
to the old transformed endpoint, but 0.081887 m to the canonical map goal.
The actual Path timestamp/private GoalChecker input was not captured; the
planning +/-0.5-second map->odom window was constant. The regression uses that
bounded reconstruction with the real planner, nav_2d_utils transform and Humble
SimpleGoalChecker, rather than treating the replay as an internal trace.

## Scope

Only the existing direct-path branch changes. Checked elevator search and
ordinary/docking planners are untouched. Tolerances remain 0.06 m / 0.05 rad.
No changes to AMCL, TF publishers, lidar timestamps, arm services, speed limits,
elevator collision policy, control ownership or readiness conditions.

## Verification

`test_elevator_goal_stamp` verifies preserved geometry/headers, rejection of
the recorded premature-success pose, and unchanged XY/yaw acceptance limits.
It calls the real planner and goal checker with locally inserted TF; run it
in an isolated network namespace, without production topics or motion goals.

The planner is compiled through the complete `ranger_mini3_lattice_planner`
CMake target, not linked against stale helper objects. Build/test evidence and
the candidate library are under `/tmp/njrh_reports/elevator_goal_stamp_fix_20260910`.
Source/test synchronization does not activate a new running planner. A
separately authorized full-chain restart and supervised cabin-panel test are
still required. At completion, compare the original map goal with aligned live
TF and require the existing 0.06 m / 0.05 rad limits; absolute physical accuracy
also depends on localization and requires an independent measurement.
