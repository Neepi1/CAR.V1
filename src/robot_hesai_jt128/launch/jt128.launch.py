from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_hesai_jt128",
                executable="jt128_wrapper_node.py",
                name="robot_hesai_jt128",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("robot_hesai_jt128"), "config", "jt128.yaml"])
                ],
            )
        ]
    )
