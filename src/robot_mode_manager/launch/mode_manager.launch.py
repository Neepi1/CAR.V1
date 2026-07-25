from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    default_params = PathJoinSubstitution(
        [FindPackageShare("robot_mode_manager"), "config", "mode_manager.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="robot_mode_manager",
                executable="mode_manager_node",
                name="robot_mode_manager",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
