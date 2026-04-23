from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_local_perception",
                executable="local_perception_node",
                name="robot_local_perception",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("robot_local_perception"), "config", "local_perception.yaml"]
                    )
                ],
            )
        ]
    )
