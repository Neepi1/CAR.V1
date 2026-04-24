from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_safety",
                executable="robot_safety_node",
                name="robot_safety",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("robot_safety"), "config", "robot_safety.yaml"])
                ],
            )
        ]
    )
