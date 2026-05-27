from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_api_server"), "config", "robot_api_server.yaml"]
                ),
                description="robot_api_server parameter file",
            ),
            Node(
                package="robot_api_server",
                executable="robot_api_server_node",
                name="robot_api_server",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
