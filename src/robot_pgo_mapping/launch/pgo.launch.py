from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_pgo_mapping",
                executable="pgo_wrapper_node.py",
                name="robot_pgo_mapping",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("robot_pgo_mapping"), "config", "pgo.yaml"])
                ],
            )
        ]
    )
