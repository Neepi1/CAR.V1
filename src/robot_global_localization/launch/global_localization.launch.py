from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_global_localization",
                executable="global_localization_node.py",
                name="robot_global_localization",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("robot_global_localization"), "config", "global_localization.yaml"]
                    )
                ],
            )
        ]
    )
