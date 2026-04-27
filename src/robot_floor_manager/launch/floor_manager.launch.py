from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("robot_floor_manager"), "config", "floor_manager.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            Node(
                package="robot_floor_manager",
                executable="floor_manager_node",
                name="robot_floor_manager",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
