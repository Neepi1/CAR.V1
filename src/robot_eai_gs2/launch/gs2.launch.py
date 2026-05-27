from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")
    scan_topic = LaunchConfiguration("scan_topic")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")

    default_config = PathJoinSubstitution([
        FindPackageShare("robot_eai_gs2"),
        "config",
        "gs2.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("serial_port", default_value="/dev/gs2"),
        DeclareLaunchArgument("serial_baudrate", default_value="921600"),
        DeclareLaunchArgument("frame_id", default_value="gs2_link"),
        DeclareLaunchArgument("scan_topic", default_value="/dock/gs2_scan"),
        DeclareLaunchArgument("point_cloud_topic", default_value="/dock/gs2_points"),
        Node(
            package="robot_eai_gs2",
            executable="gs2_driver_node",
            name="gs2_driver_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "serial_port": serial_port,
                    "serial_baudrate": serial_baudrate,
                    "frame_id": frame_id,
                    "scan_topic": scan_topic,
                    "point_cloud_topic": point_cloud_topic,
                },
            ],
        ),
    ])
