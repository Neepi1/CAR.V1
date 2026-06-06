from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_local_state",
                executable="local_state_node",
                name="wheel_odom_ekf_input",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("robot_local_state"), "config", "local_state_wheel_odom_ekf.yaml"]
                    )
                ],
            ),
            Node(
                package="robot_local_state",
                executable="imu_gyro_bias_filter_node",
                name="imu_gyro_bias_filter",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("robot_local_state"), "config", "local_state_imu_bias_filter.yaml"]
                    )
                ],
            ),
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
