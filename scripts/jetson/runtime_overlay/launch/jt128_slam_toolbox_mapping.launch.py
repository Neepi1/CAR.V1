#!/usr/bin/env python3

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def cpu_affinity_prefix(service_name: str) -> str | None:
    enabled = os.environ.get("NJRH_CPU_AFFINITY_ENABLED", "true").lower()
    if enabled not in ("1", "true", "yes", "on"):
        return None
    key = service_name.upper().replace("-", "_").replace(".", "_").replace("/", "_")
    cpuset = os.environ.get(f"NJRH_CPUSET_{key}", "")
    if not cpuset:
        return None
    return f"taskset -c {cpuset}"


def generate_launch_description() -> LaunchDescription:
    overlay_root = Path(__file__).resolve().parents[1]
    upstream_root = Path(os.environ.get("NJRH_UPSTREAM_ROOT", "/workspaces/isaac_ros-dev"))
    upstream_params_dir = upstream_root / "nav2_test" / "params"

    preprocessor_params_default = overlay_root / "config" / "jt128_nav_cloud_preprocessor.yaml"
    scan_params_default = overlay_root / "config" / "jt128_scan_slam2d.yaml"
    slam_params_default = overlay_root / "config" / "jt128_slam_toolbox_mapping.yaml"
    upstream_slam_params = upstream_params_dir / "jt128_slam_toolbox_mapping.yaml"

    preprocessor_params = LaunchConfiguration("preprocessor_params")
    scan_params = LaunchConfiguration("scan_params")
    slam_params = LaunchConfiguration("slam_params")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    points_topic = LaunchConfiguration("points_topic")
    nav_points_topic = LaunchConfiguration("nav_points_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    tf_topic = LaunchConfiguration("tf_topic")
    mapping_pipeline_env = {"NJRH_SLAM2D_MAPPING_PIPELINE": "1"}
    active_fastdds_profile = os.environ.get(
        "NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE", ""
    )
    base_fastdds_profile = os.environ.get(
        "NJRH_SLAM2D_FASTDDS_BASE_PROFILE_FILE",
        os.environ.get(
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            os.environ.get("FASTDDS_DEFAULT_PROFILES_FILE", ""),
        ),
    )

    # Only the three participants carrying the two full-size local cloud hops
    # use the mapping UDP+SHM profile.  ExecuteProcess `env` lets us remove the
    # global UDPv4 built-in override so the explicit custom transports apply.
    cloud_transport_env = os.environ.copy()
    cloud_transport_env.update(mapping_pipeline_env)
    cloud_transport_env["NJRH_SLAM2D_FASTDDS_ROLE"] = "large_cloud"
    if active_fastdds_profile:
        cloud_transport_env["FASTRTPS_DEFAULT_PROFILES_FILE"] = active_fastdds_profile
        cloud_transport_env["FASTDDS_DEFAULT_PROFILES_FILE"] = active_fastdds_profile
        cloud_transport_env.pop("FASTDDS_BUILTIN_TRANSPORTS", None)

    # slam_toolbox only consumes LaserScan.  Keep it on the robot-wide UDP
    # profile rather than allocating another 128 MiB SHM participant.
    control_transport_env = {
        **mapping_pipeline_env,
        "NJRH_SLAM2D_FASTDDS_ROLE": "control_udp",
    }
    if base_fastdds_profile:
        control_transport_env["FASTRTPS_DEFAULT_PROFILES_FILE"] = base_fastdds_profile
        control_transport_env["FASTDDS_DEFAULT_PROFILES_FILE"] = base_fastdds_profile
        control_transport_env["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"

    # Mapping deliberately slices FAST-LIO2's deskewed/body-frame cloud.  It
    # publishes directly to the canonical /scan name after the resident scan
    # publisher has surrendered ownership; the source timestamp is preserved.
    nav_cloud_preprocessor = Node(
        package="jt128_nav_tools",
        executable="nav_cloud_preprocessor",
        name="nav_cloud_preprocessor",
        output="screen",
        env=cloud_transport_env,
        prefix=cpu_affinity_prefix("nav_cloud_preprocessor"),
        parameters=[
            str(preprocessor_params_default),
            preprocessor_params,
            {
                "input_topic": points_topic,
                "output_topic": nav_points_topic,
                "output_frame_id": "lidar_level_link",
            },
        ],
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        env=cloud_transport_env,
        prefix=cpu_affinity_prefix("pointcloud_to_laserscan"),
        parameters=[str(scan_params_default), scan_params],
        remappings=[
            ("cloud_in", nav_points_topic),
            ("scan", scan_topic),
        ],
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        additional_env=control_transport_env,
        prefix=cpu_affinity_prefix("slam_toolbox_mapping"),
        remappings=[
            ("/tf", tf_topic),
        ],
        parameters=[
            str(upstream_slam_params),
            slam_params,
            {
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "scan_topic": scan_topic,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("preprocessor_params", default_value=str(preprocessor_params_default)),
            DeclareLaunchArgument("scan_params", default_value=str(scan_params_default)),
            DeclareLaunchArgument("slam_params", default_value=str(slam_params_default)),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("points_topic", default_value="/mapping/fastlio/cloud_registered_body"),
            DeclareLaunchArgument("nav_points_topic", default_value="/mapping/points_nav"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("tf_topic", default_value="/tf"),
            nav_cloud_preprocessor,
            pointcloud_to_scan,
            slam_toolbox_node,
        ]
    )
