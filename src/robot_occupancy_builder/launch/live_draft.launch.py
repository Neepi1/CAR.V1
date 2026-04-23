from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_occupancy_builder",
                executable="occupancy_builder_live_node.py",
                name="robot_occupancy_builder_live",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("robot_occupancy_builder"), "config", "live_draft.yaml"]
                    )
                ],
            )
        ]
    )
