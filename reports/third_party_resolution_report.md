# Third Party Resolution Report

- Resolution order: local workspace -> `D:\codespace\car` -> fallback `.repos` metadata
- Fetch policy: do not download automatically; document fallback only

## Component Resolution

### hesai_driver
- local candidate: `D:\codespace\car\ros2_ws\src\hesai_lidar_ros2`
- local candidate: `D:\codespace\car\ros2_ws\src\hesai_lidar_ros2\config\config.yaml`
- decision: direct local reuse

### fastlio2
- local candidate: `D:\codespace\car\ros2_ws\src\fast_lio`
- local candidate: `D:\codespace\car\ros2_ws\src\fast_lio\config\jt128.yaml`
- decision: direct local reuse

### pgo_backend
- local candidate: `D:\codespace\car\ros2_ws\src\fastlio_pgo`
- local candidate: `D:\codespace\car\ros2_ws\src\fastlio_pgo\config\pgo.yaml`
- decision: direct local reuse

### isaac_localizer
- local candidate: `D:\codespace\car\nav2_test\params\jt128_occupancy_grid_localizer.yaml`
- local candidate: `D:\codespace\car\nav2_test\params\jt128_flatscan.yaml`
- local candidate: `D:\codespace\car\nav2_test\jt128_occupancy_localization.launch.py`
- remote candidate: `/home/nvidia/workspaces/isaac_ros-dev/install/isaac_ros_occupancy_grid_localizer`
- remote candidate: `/home/nvidia/workspaces/isaac_ros-dev/install/isaac_ros_pointcloud_utils`
- decision: reuse validated params and launch first; Jetson already has the required install-time packages

### nav2
- local candidate: `D:\codespace\car\nav2_test\params\nav2_jt128_rapid_avoidance.yaml`
- local candidate: `D:\codespace\car\nav2_test\behavior_tree\jt128_rapid_avoidance_replanning_and_recovery.xml`
- decision: reuse validated profiles as reference, but keep this repository's fixed planner/controller choices

### robot_localization
- local candidate: none found in `D:\codespace\car`
- remote candidate: `/opt/ros/humble`
- decision: reuse the system package on Jetson; keep source fallback metadata only if a source overlay becomes necessary

### ranger_driver
- local candidate: `D:\codespace\car\ros2_ws\src\ranger_ros2`
- local candidate: `D:\codespace\car\ros2_ws\src\ugv_sdk`
- decision: direct local reuse

### nav_tools
- local candidate: `D:\codespace\car\ros2_ws\src\jt128_nav_tools`
- local candidate: `D:\codespace\car\nav2_test\params\jt128_map_to_odom_tf_bridge.yaml`
- decision: reuse helper logic and calibration references, but keep canonical TF ownership in this repository

### docker_runtime
- local candidate: `D:\codespace\car\Dockerfile.car`
- remote candidate: `isaac_ros_dev-aarch64:latest`
- decision: keep runtime local-first by building `njrh-car:latest` from the validated Jetson workspace Dockerfile and existing Isaac ROS dev base image; if the current Jetson Docker setup blocks rebuild, fall back to the already-validated base image for runtime startup

### operator_frontend
- local candidate: `D:\codespace\car\web_dashboard`
- local candidate: `D:\codespace\car\scripts\run_web_dashboard.sh`
- decision: reuse the validated web assets, but start them through this repository's runtime overlay so the current project owns runtime orchestration, default Nav2 configuration, local obstacle filtering, and final command arbitration entrypoints

## Fallback Policy
- Missing components remain declared in `.repos/third_party.repos` only as explicit fallback metadata.
- No automatic network fetch was performed in this pass.
