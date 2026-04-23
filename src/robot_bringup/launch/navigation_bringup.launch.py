from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package_name: str, launch_name: str, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_name])
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map_yaml")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    use_composition = LaunchConfiguration("use_composition")

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("robot_nav_config"), "config", "nav2.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("map_yaml", default_value=""),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("use_respawn", default_value="false"),
            DeclareLaunchArgument("use_composition", default_value="false"),
            include(
                "robot_bringup",
                "localization_bringup.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "map_yaml": map_yaml,
                    "load_map_server": "true",
                    "params_file": params_file,
                },
            ),
            include(
                "robot_bringup",
                "standard_navigation.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_respawn": use_respawn,
                    "use_composition": use_composition,
                },
            ),
        ]
    )
