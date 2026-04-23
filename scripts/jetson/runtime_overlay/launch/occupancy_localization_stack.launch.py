#!/usr/bin/env python3
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    overlay_root = Path(__file__).resolve().parent.parent
    sensing_launch = overlay_root / "launch" / "jt128_localization_sensing.launch.py"
    localization_launch = Path(__file__).resolve().parent / "occupancy_localization.launch.py"
    upstream_root = Path(os.environ.get("NJRH_UPSTREAM_ROOT", "/workspaces/isaac_ros-dev"))

    map_yaml_default = upstream_root / "maps" / "jt128_map.yaml"
    localizer_map_yaml_default = map_yaml_default
    localizer_params_default = overlay_root / "config" / "jt128_occupancy_grid_localizer.yaml"

    map_yaml = LaunchConfiguration("map_yaml")
    localizer_map_yaml = LaunchConfiguration("localizer_map_yaml")
    localizer_params = LaunchConfiguration("localizer_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_lidar_tf = LaunchConfiguration("publish_lidar_tf")

    sensing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sensing_launch)),
        launch_arguments={
            "points_topic": "/lidar_points",
            "scan_topic": "/scan",
            "flatscan_topic": "/flatscan",
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(localization_launch)),
        launch_arguments={
            "map_yaml": map_yaml,
            "localizer_map_yaml": localizer_map_yaml,
            "localizer_params": localizer_params,
            "use_sim_time": use_sim_time,
            "start_map_server": "true",
            "map_frame": "map",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", default_value=str(map_yaml_default)),
        DeclareLaunchArgument("localizer_map_yaml", default_value=str(localizer_map_yaml_default)),
        DeclareLaunchArgument("localizer_params", default_value=str(localizer_params_default)),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("publish_lidar_tf", default_value="false"),
        sensing,
        localization,
    ])
