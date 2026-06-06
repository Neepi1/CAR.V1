#!/usr/bin/env python3
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def cpu_affinity_prefix(service_name):
    enabled = os.environ.get("NJRH_CPU_AFFINITY_ENABLED", "true").lower()
    if enabled not in ("1", "true", "yes", "on"):
        return None
    key = service_name.upper().replace("-", "_").replace(".", "_").replace("/", "_")
    cpuset = os.environ.get(f"NJRH_CPUSET_{key}", "")
    if not cpuset:
        return None
    return f"taskset -c {cpuset}"


def generate_launch_description():
    upstream_root = Path(os.environ.get("NJRH_UPSTREAM_ROOT", "/workspaces/isaac_ros-dev"))
    overlay_root = Path(__file__).resolve().parents[1]
    params_dir = upstream_root / "nav2_test" / "params"

    map_yaml_default = upstream_root / "maps" / "jt128_map.yaml"
    localizer_params_default = overlay_root / "config" / "jt128_occupancy_grid_localizer.yaml"

    map_yaml = LaunchConfiguration("map_yaml")
    localizer_map_yaml = LaunchConfiguration("localizer_map_yaml")
    localizer_params = LaunchConfiguration("localizer_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_map_server = LaunchConfiguration("start_map_server")
    map_frame = LaunchConfiguration("map_frame")

    map_server = Node(
        condition=IfCondition(start_map_server),
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        prefix=cpu_affinity_prefix("nav2_map_server"),
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
        }],
    )

    lifecycle_manager = Node(
        condition=IfCondition(start_map_server),
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        prefix=cpu_affinity_prefix("nav2_lifecycle_manager"),
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )

    occupancy_grid_localizer = ComposableNode(
        package="isaac_ros_occupancy_grid_localizer",
        plugin="nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode",
        name="occupancy_grid_localizer",
        parameters=[
            localizer_map_yaml,
            str(localizer_params_default),
            localizer_params,
            {
                "map_yaml_path": localizer_map_yaml,
                "loc_result_frame": map_frame,
            },
        ],
        remappings=[
            ("flatscan", "/flatscan"),
            ("localization_result", "/localization_result"),
        ],
    )

    localizer_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="occupancy_grid_localizer_container",
        namespace="",
        composable_node_descriptions=[occupancy_grid_localizer],
        output="screen",
        prefix=cpu_affinity_prefix("occupancy_grid_localizer"),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", default_value=str(map_yaml_default)),
        DeclareLaunchArgument("localizer_map_yaml", default_value=str(map_yaml_default)),
        DeclareLaunchArgument("localizer_params", default_value=str(localizer_params_default)),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_map_server", default_value="true"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        map_server,
        lifecycle_manager,
        localizer_container,
    ])
