#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    overlay_root = Path(__file__).resolve().parents[1]
    preprocessor_params_default = overlay_root / "config" / "jt128_nav_cloud_preprocessor.yaml"
    scan_params_default = overlay_root / "config" / "jt128_scan_slam2d.yaml"
    flatscan_params_default = overlay_root / "config" / "jt128_flatscan.yaml"

    preprocessor_params = LaunchConfiguration("preprocessor_params")
    scan_params = LaunchConfiguration("scan_params")
    flatscan_params = LaunchConfiguration("flatscan_params")
    points_topic = LaunchConfiguration("points_topic")
    nav_points_topic = LaunchConfiguration("nav_points_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    flatscan_topic = LaunchConfiguration("flatscan_topic")

    nav_cloud_preprocessor = Node(
        package="jt128_nav_tools",
        executable="nav_cloud_preprocessor",
        name="nav_cloud_preprocessor",
        output="screen",
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
        parameters=[str(scan_params_default), scan_params],
        remappings=[
            ("cloud_in", nav_points_topic),
            ("scan", "/scan_raw"),
        ],
    )

    restamp_scan = Node(
        package="robot_hesai_jt128",
        executable="scan_republisher_node",
        name="scan_republisher",
        output="screen",
        parameters=[
            {
                "input_topic": "/scan_raw",
                "output_topic": scan_topic,
                "restamp_to_now": True,
            }
        ],
    )

    laser_scan_to_flatscan = Node(
        package="jt128_nav_tools",
        executable="laser_scan_to_flatscan",
        name="laser_scan_to_flatscan",
        output="screen",
        parameters=[str(flatscan_params_default), flatscan_params],
        remappings=[
            ("scan", scan_topic),
            ("flatscan", flatscan_topic),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("preprocessor_params", default_value=str(preprocessor_params_default)),
            DeclareLaunchArgument("scan_params", default_value=str(scan_params_default)),
            DeclareLaunchArgument("flatscan_params", default_value=str(flatscan_params_default)),
            DeclareLaunchArgument("points_topic", default_value="/lidar_points"),
            DeclareLaunchArgument("nav_points_topic", default_value="/points_nav"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("flatscan_topic", default_value="/flatscan"),
            nav_cloud_preprocessor,
            pointcloud_to_scan,
            restamp_scan,
            laser_scan_to_flatscan,
        ]
    )
