import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def cpu_affinity_prefix(service_name: str) -> str | None:
    enabled = os.environ.get("NJRH_CPU_AFFINITY_ENABLED", "true").lower()
    if enabled not in ("1", "true", "yes", "on"):
        return None
    key = service_name.upper().replace("-", "_").replace(".", "_").replace("/", "_")
    cpuset = os.environ.get(f"NJRH_CPUSET_{key}", "")
    if cpuset:
        return f"taskset -c {cpuset}"
    return None


def generate_launch_description():
    overlay_root = Path(os.environ.get("NJRH_OVERLAY_ROOT", "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay"))
    pointcloud_params_default = overlay_root / "config" / "jt128_canonical_pointcloud_remap.yaml"
    local_perception_params_default = overlay_root / "config" / "local_perception.yaml"

    pointcloud_params = LaunchConfiguration("pointcloud_params")
    local_perception_params = LaunchConfiguration("local_perception_params")

    return LaunchDescription(
        [
            DeclareLaunchArgument("pointcloud_params", default_value=str(pointcloud_params_default)),
            DeclareLaunchArgument("local_perception_params", default_value=str(local_perception_params_default)),
            ComposableNodeContainer(
                name="pointcloud_perception_pipeline",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                prefix=cpu_affinity_prefix("pointcloud_perception_pipeline"),
                composable_node_descriptions=[
                    ComposableNode(
                        package="robot_hesai_jt128",
                        plugin="PointCloudAxisRemapNode",
                        name="pointcloud_axis_remap",
                        parameters=[pointcloud_params],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                    ComposableNode(
                        package="robot_local_perception",
                        plugin="LocalPerceptionNode",
                        name="robot_local_perception",
                        parameters=[local_perception_params],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
            ),
        ]
    )
