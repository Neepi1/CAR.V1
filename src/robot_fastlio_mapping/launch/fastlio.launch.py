from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_fastlio_mapping",
                executable="fastlio_wrapper_node.py",
                name="robot_fastlio_mapping",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("robot_fastlio_mapping"), "config", "fastlio.yaml"])
                ],
            )
        ]
    )
