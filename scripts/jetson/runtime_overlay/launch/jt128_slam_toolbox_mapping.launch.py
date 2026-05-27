#!/usr/bin/env python3

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    overlay_root = Path(__file__).resolve().parents[1]
    upstream_root = Path(os.environ.get("NJRH_UPSTREAM_ROOT", "/workspaces/isaac_ros-dev"))
    upstream_params_dir = upstream_root / "nav2_test" / "params"

    preprocessor_params_default = overlay_root / "config" / "jt128_nav_cloud_preprocessor.yaml"
    slam_params_default = overlay_root / "config" / "jt128_slam_toolbox_mapping.yaml"
    scan_params_default = overlay_root / "config" / "jt128_scan_slam2d.yaml"
    upstream_slam_params = upstream_params_dir / "jt128_slam_toolbox_mapping.yaml"

    preprocessor_params = LaunchConfiguration("preprocessor_params")
    slam_params = LaunchConfiguration("slam_params")
    scan_params = LaunchConfiguration("scan_params")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    points_topic = LaunchConfiguration("points_topic")
    nav_points_topic = LaunchConfiguration("nav_points_topic")
    scan_topic = LaunchConfiguration("scan_topic")

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

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
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
            DeclareLaunchArgument("slam_params", default_value=str(slam_params_default)),
            DeclareLaunchArgument("scan_params", default_value=str(scan_params_default)),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("points_topic", default_value="/cloud_registered_body"),
            DeclareLaunchArgument("nav_points_topic", default_value="/points_nav"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            nav_cloud_preprocessor,
            pointcloud_to_scan,
            restamp_scan,
            slam_toolbox_node,
        ]
    )
