# robot_bringup

Bringup composition for the repository-owned localization and navigation baseline.

## Launches

- `mock_navigation.launch.py`: first-round lightweight stack for scaffold and mock validation
- `localization_bringup.launch.py`: canonical platform stack plus optional `nav2_map_server`
- `localization_bringup.launch.py`: also starts `robot_floor_manager` so floor switches can reuse the active map server and localization services
- `navigation_bringup.launch.py`: `localization_bringup` + repo-owned standard navigation chain
- `standard_navigation.launch.py`: repo-owned standard Nav2 stack only, for runtime paths that already started localization separately
- `local_costmap_debug.launch.py`: repo-owned local-costmap-only debug stack; it starts only `controller_server` and its local costmap lifecycle owner, with `cmd_vel` remapped away from the real control chain

## Parameters

- `use_sim_time`: default `false`
- `autostart`: default `true`
- `map_yaml`: Nav2 map asset used by the repo-owned map server entrypoint
- `params_file`: defaults to `robot_nav_config/config/nav2.yaml`
- `use_respawn`: passed to the repo-owned Nav2 node set
- `use_composition`: accepted for API compatibility; the field runtime still launches the stack non-composed
- `canonical_tf_policy`: points to `robot_nav_config/config/tf_policy.yaml`

## Notes

- `localization_bringup.launch.py` keeps the repository-owned canonical stack in front: description, chassis, JT128, local perception, local state, global localization, localization bridge, robot safety, and optional map server.
- `navigation_bringup.launch.py` reuses that same stack and then loads the repo-owned Nav2 chain from `standard_navigation.launch.py` with this repository's `robot_nav_config/config/nav2.yaml`.
- `standard_navigation.launch.py` explicitly launches `controller_server`, `behavior_server`, `velocity_smoother`, `collision_monitor`, `lifecycle_manager_costmap_filters`, and `lifecycle_manager_navigation` so every Nav2 velocity path is routed into the repository-owned safety chain while costmap filter servers have an isolated lifecycle owner. The field runner passes `navigation_lifecycle_autostart:=false` and activates the core navigation nodes with Nav2's `nav2_util/lifecycle_bringup` helper so Humble's fixed 2 second lifecycle-manager `get_state` wait does not abort startup while `planner_server` is loading the global costmap.
- `scripts/jetson/runtime_overlay/launch/occupancy_localization.launch.py` exposes `map_lifecycle_manager_enabled`. The production resident runtime passes `false` and activates `/map_server` with `nav2_util/lifecycle_bringup map_server`, so a map-server lifecycle-manager response timeout cannot leave a successfully loaded selected-floor map inactive.
- `standard_navigation.launch.py` and `local_costmap_debug.launch.py` read `NJRH_CPUSET_*` environment variables and add `taskset` prefixes when the Jetson runtime CPU affinity policy is enabled. The stack remains non-composed so critical nodes can be assigned different cores.
- The Jetson runtime wrapper around `standard_navigation.launch.py` now requires an active `/map_server`, waits for `/map`, and waits for `/global_costmap/costmap` to resize to the static map before considering the standard navigation stack ready.
- `local_costmap_debug.launch.py` is only for obstacle-layer verification. It does not start planner, BT navigator, velocity smoother, collision monitor, map server, or robot safety, so it must not be treated as the production navigation path.
- Jetson field runtime still uses the temporary `NJRH-car` container and shell helpers for operator workflows, but these launch files now define the repository-owned bringup contract that those helpers should converge toward.
