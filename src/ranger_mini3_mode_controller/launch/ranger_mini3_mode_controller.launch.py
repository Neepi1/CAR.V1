from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("ranger_mini3_mode_controller"), "config", "ranger_mini3_mode_controller.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="ranger_mini3_mode_controller",
                executable="mode_controller_node",
                name="ranger_mini3_mode_controller",
                output="screen",
                parameters=[config],
            )
        ]
    )
