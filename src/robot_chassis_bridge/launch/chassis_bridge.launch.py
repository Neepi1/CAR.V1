from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_chassis_bridge",
                executable="chassis_bridge_node.py",
                name="robot_chassis_bridge",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("robot_chassis_bridge"), "config", "chassis_bridge.yaml"])
                ],
            )
        ]
    )
