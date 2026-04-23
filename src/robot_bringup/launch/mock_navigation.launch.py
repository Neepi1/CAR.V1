from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package_name: str, launch_name: str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_name])
        )
    )


def generate_launch_description():
    return LaunchDescription(
        [
            include("robot_description", "description.launch.py"),
            include("robot_chassis_bridge", "chassis_bridge.launch.py"),
            include("robot_hesai_jt128", "jt128.launch.py"),
            include("robot_local_perception", "local_perception.launch.py"),
            include("robot_local_state", "local_state.launch.py"),
            include("robot_global_localization", "global_localization.launch.py"),
            include("robot_localization_bridge", "localization_bridge.launch.py"),
            include("robot_safety", "robot_safety.launch.py"),
        ]
    )
