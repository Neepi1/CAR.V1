# Runtime Force-Accept And Bridge Smoothing Audit

Phase: R0-R2
Timestamp: 20260615T204650Z
Scope: read-only audit before reducing normal-path relocalization and adding bridge map->odom smoothing.

## Current Evidence

- `robot_api_server_node.cpp` still owns high-risk relocalization timing in business paths.
- `trigger_localization_and_wait_for_result()` arms `/robot_localization_bridge/force_accept_next_localization`, calls `/global_localization/trigger`, waits for `/localization_result`, then waits for bridge acceptance.
- Normal navigation can call that path before `NavigateToPose` when pre-goal relocalization is requested.
- Docking normal path defaults to before-predock, after-predock, and after-fine-docking relocalization.
- `robot_localization_bridge::apply_candidate()` accepts corrections by replacing the active `map_to_odom_` output transform immediately.
- The existing 50 Hz publisher decouples `sendTransform()` from correction callbacks, but it publishes the latest accepted value directly; it does not smooth current toward target.

## Latest Repo-Local Nav2 TF Abort Report

Source: `reports/nav2_tf_abort_diagnosis/20260613T192650Z_nav_fail_20260613T192649Z_180s/summary.md`

- `/tf` map->odom sample count: 8995 over 180 s.
- `/tf` map->odom receive gap average: about 19.9 ms.
- map->odom publish cadence was generally alive.
- Controller abort lines show future extrapolation inside `controller_server`:
  - requested 1781377334.715207, latest map->odom in controller buffer 1781377334.567207, lag about 148 ms.
  - requested 1781378834.775120, latest map->odom in controller buffer 1781378834.067130, lag about 708 ms.
  - requested 1781378855.775131, latest map->odom in controller buffer 1781378855.067091, lag about 708 ms.
- This report does not prove bridge publisher stopped; it shows controller-side TF consumption became stale during action execution.

## Current Normal Navigation Path

`POST /api/v1/navigation/goal`
-> dock/contact pre-check and optional controlled undock
-> Nav2 lifecycle snapshot
-> `navigation_goal_relocalization_decision()`
-> if requested: `trigger_localization_and_wait_for_result()`
-> `wait_for_post_relocalization_settle_barrier()`
-> `wait_for_fresh_tf_chain()`
-> send Nav2 `NavigateToPose`
-> action result
-> API final pose verify / optional yaw align / reposition retry
-> App/API state write.

Normal-path high-risk calls:

- `trigger_localization_and_wait_for_result()` from `handle_navigation_goal()`
- `/robot_localization_bridge/force_accept_next_localization`
- `/global_localization/trigger`
- `/localization_result` wait
- bridge acceptance wait
- post-relocalization settle wait

## Current Docking Path

`POST /api/v1/docking/start`
-> resolve dock profile / predock
-> ensure navigation runtime
-> cancel active nav/final-yaw owner
-> before-predock relocalization
-> before-predock settle
-> fresh TF chain check
-> Nav2 predock goal
-> predock pose verify
-> predock yaw align when needed
-> after-predock relocalization
-> after-predock settle
-> optional post-relocalization predock validation
-> fresh TF chain check
-> GS2/fine docking entry check
-> pause global correction
-> `robot_docking_manager` fine docking
-> optional after-fine-docking relocalization.

Default config currently enables:

- `docking_relocalize_before_predock: true`
- `docking_relocalize_after_predock: true`
- `docking_relocalize_after_predock_required: true`
- `docking_relocalize_after_fine_docking: true`

## Current Bridge Correction Apply

`robot_localization_bridge` correction flow:

- `/localization_result` or `/amcl_pose`
-> `build_candidate()`
-> age/frame/covariance/TF gates
-> force/jump/AMCL small-medium-hard gates
-> `apply_candidate()`
-> direct active output replacement:
  - `map_to_odom_ = candidate.transform`
  - `update_map_odom_state_from_candidate(candidate)`
-> 50 Hz timer publishes the latest state.

There is no current/target split and no slew-rate limit in the current bridge.

## Force-Accept Entrances

Normal-path risk:

- `handle_navigation_goal()` can call `trigger_localization_and_wait_for_result()`.
- `run_docking_job()` calls it before predock when enabled.
- `run_docking_job()` calls it after predock when enabled.

Recovery/maintenance entrances to preserve:

- manual `handle_trigger_localization()`
- cold-start navigation runtime startup
- floor switch / selected-map startup sequence
- post-undock recovery path
- explicit localization degraded recovery

## Required Reduction

- Remove implicit force-accept and `/global_localization/trigger` from normal navigation goal handling.
- Disable default before-predock, after-predock, and after-fine-docking relocalization in normal docking path.
- Keep AMCL gated correction enabled as the continuous correction layer.
- Keep explicit recovery relocalization available, but keep it out of normal `FollowPath` startup.
- Add bridge current/target smoothing so accepted corrections no longer produce immediate output jumps.

## Files To Modify In R1-R2

- `src/robot_api_server/src/robot_api_server_node.cpp`
- `src/robot_api_server/config/robot_api_server.yaml`
- `scripts/jetson/runtime_overlay/config/robot_api_server.yaml`
- `src/robot_localization_bridge/src/localization_bridge_node.cpp`
- `src/robot_localization_bridge/config/localization_bridge.yaml`
- `scripts/jetson/runtime_overlay/config/localization_bridge.yaml`
- runtime verification scripts
- workspace contract tests
- README/docs describing normal vs recovery path separation.
