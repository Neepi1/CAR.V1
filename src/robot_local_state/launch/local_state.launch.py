from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="robot_local_state",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("robot_local_state"), "config", "local_state_ekf.yaml"])
                ],
                remappings=[
                    ("/odometry/filtered", "/local_state/odometry"),
                ],
            )
        ]
    )
