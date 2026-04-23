from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_occupancy_builder",
                executable="occupancy_builder_release_node.py",
                name="robot_occupancy_builder_release",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("robot_occupancy_builder"), "config", "release_rebuild.yaml"]
                    )
                ],
            )
        ]
    )
