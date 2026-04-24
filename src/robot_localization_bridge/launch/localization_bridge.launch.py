from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_localization_bridge",
                executable="localization_bridge_node",
                name="robot_localization_bridge",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("robot_localization_bridge"), "config", "localization_bridge.yaml"]
                    )
                ],
            )
        ]
    )
