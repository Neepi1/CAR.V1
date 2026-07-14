# Phase V1 Runtime Config Audit

Timestamp: `20260616T020613Z`

Scope: read-only repository audit for Phase V1 validation. This report checks the currently configured and compiled contracts without changing runtime behavior.

| Field | Current Value | Expected Value | PASS/FAIL | Evidence |
| --- | --- | --- | --- | --- |
| `navigation_default_goal_completion_policy` | `pose_required` in package and Jetson overlay configs | `pose_required` | PASS | `src/robot_api_server/config/robot_api_server.yaml:177`, `scripts/jetson/runtime_overlay/config/robot_api_server.yaml:176` |
| `position_only` explicit opt-out | Supported policy, with final yaw alignment skipped only for `position_only` | Retained only as explicit engineering opt-out | PASS | `src/robot_api_server/src/robot_api_server_node.cpp:3368`, `src/robot_api_server/src/robot_api_server_node.cpp:8426` |
| Ordinary navigation default is `pose_required` | Default declared as `pose_required`; invalid `dock_staging` default is forced back to `pose_required` | `pose_required` default for normal navigation | PASS | `src/robot_api_server/src/robot_api_server_node.cpp:488`, `src/robot_api_server/src/robot_api_server_node.cpp:496` |
| `final_yaw_align` trigger scope | Normal goal job sets `yaw_align_required = goal_completion_policy == "pose_required"` | Only `pose_required` normal navigation requires final yaw alignment | PASS | `src/robot_api_server/src/robot_api_server_node.cpp:7160`, `src/robot_api_server/src/robot_api_server_node.cpp:8426` |
| `dock_staging` API scope | `dock_staging` is rejected in normal navigation and assigned by docking flow | Only `/api/v1/docking/start` may use `dock_staging` | PASS | `src/robot_api_server/src/robot_api_server_node.cpp:7010`, `src/robot_api_server/src/robot_api_server_node.cpp:10420` |
| Ordinary `final_yaw_align` and `PREDOCK_YAW_ALIGN` owner mutex | Both owners tracked; conflicts produce `PREDOCK_YAW_ALIGN_OWNER_CONFLICT` or block final yaw | Owner conflict protection exists | PASS | `src/robot_api_server/src/robot_api_server_node.cpp:3047`, `src/robot_api_server/src/robot_api_server_node.cpp:3081`, `src/robot_api_server/src/robot_api_server_node.cpp:8024` |
| `/api/v1/localization/trigger` default settle behavior | `wait_for_settle` defaults false; response emits `post_relocalization_settle_requested` from request flag | Default `post_relocalization_settle_requested=false` | PASS | `src/robot_api_server/src/robot_api_server_node.cpp:9086`, `src/robot_api_server/src/robot_api_server_node.cpp:9114` |
| `robot_localization_bridge` `map->odom` owner | Bridge status reports `map_to_odom_publisher_owner=robot_localization_bridge`; single broadcaster call remains in bridge | Bridge is sole `map->odom` owner | PASS | `src/robot_localization_bridge/src/localization_bridge_node.cpp:1554`, `src/robot_localization_bridge/src/localization_bridge_node.cpp:2135` |
| AMCL TF broadcast | `tf_broadcast: false` | AMCL must not publish TF | PASS | `scripts/jetson/runtime_overlay/config/amcl_shadow.yaml:12` |
| `PREDOCK_YAW_ALIGN` command topic | `predock_yaw_align_cmd_topic: "/cmd_vel_docking"` | Predock yaw uses `/cmd_vel_docking -> robot_safety -> /cmd_vel_safe -> /cmd_vel` | PASS | `src/robot_api_server/config/robot_api_server.yaml:85`, `scripts/jetson/runtime_overlay/config/robot_api_server.yaml:85` |
| Fine docking entry requires predock yaw aligned | Config requires it and source rejects when `predock_yaw_aligned=false` | Fine docking only after `predock_yaw_aligned=true` | PASS | `src/robot_api_server/config/robot_api_server.yaml:101`, `src/robot_api_server/src/robot_api_server_node.cpp:3308` |

## Notes

- This audit is static/read-only. It does not prove field readiness; use `run_v1_navigation_docking_validation.sh --observe-only` on the robot to collect runtime evidence.
- No Nav2 controller/planner/MPPI/progress checker, TF tolerance, AMCL/Isaac/bridge correction strategy, pointcloud QoS/DDS, FAST-LIO2, Ranger driver/odom, or EKF fusion policy was changed for this audit.
