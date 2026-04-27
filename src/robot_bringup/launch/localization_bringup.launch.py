from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    load_map_server = LaunchConfiguration("load_map_server")

    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("robot_nav_config"), "config", "nav2.yaml"]
    )

    map_server = Node(
        condition=IfCondition(load_map_server),
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml,
            }
        ],
    )

    lifecycle_manager_map_server = Node(
        condition=IfCondition(load_map_server),
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server"],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("load_map_server", default_value="false"),
            DeclareLaunchArgument("map_yaml", default_value=""),
            DeclareLaunchArgument("params_file", default_value=nav2_params_file),
            include("robot_description", "description.launch.py"),
            include("robot_chassis_bridge", "chassis_bridge.launch.py"),
            include("robot_hesai_jt128", "jt128.launch.py"),
            include("robot_local_perception", "local_perception.launch.py"),
            include("robot_local_state", "local_state.launch.py"),
            include("robot_global_localization", "global_localization.launch.py"),
            include("robot_localization_bridge", "localization_bridge.launch.py"),
            include("robot_floor_manager", "floor_manager.launch.py"),
            include("robot_safety", "robot_safety.launch.py"),
            map_server,
            lifecycle_manager_map_server,
        ]
    )
