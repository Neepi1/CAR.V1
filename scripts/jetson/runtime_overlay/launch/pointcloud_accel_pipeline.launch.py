#!/usr/bin/env python3

from __future__ import annotations

import os
import shlex
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    session = os.environ.get("NJRH_STARTUP_CPU_SESSION", "")
    if (session and os.environ.get("NJRH_NAVIGATION_CPU_PROFILE") == "navigation_5cpu"
            and key in ("POINTCLOUD_ACCEL_CONTAINER", "LASER_SCAN_TO_FLATSCAN")):
        overlay = Path(os.environ.get("NJRH_OVERLAY_ROOT", Path(__file__).resolve().parents[1]))
        return shlex.join([
            "python3", (overlay / "scripts/startup_cpu_affinity.py").as_posix(),
            "exec", "--session", session, "--steady-cpus", cpuset,
            "--role", key.lower(), "--",
        ])
    return f"taskset -c {cpuset}"


def generate_launch_description() -> LaunchDescription:
    overlay_root = Path(__file__).resolve().parents[1]
    accel_params_default = overlay_root / "config" / "pointcloud_accel_axis.yaml"
    flatscan_params_default = overlay_root / "config" / "jt128_flatscan.yaml"

    accel_profile = LaunchConfiguration("accel_profile")
    accel_params = LaunchConfiguration("accel_params")
    start_flatscan = LaunchConfiguration("start_flatscan")
    flatscan_params = LaunchConfiguration("flatscan_params")

    accel_axis = Node(
        package="robot_hesai_jt128",
        executable="pointcloud_accel_axis_node",
        name="pointcloud_axis_remap",
        output="screen",
        prefix=cpu_affinity_prefix("pointcloud_accel_container"),
        parameters=[
            accel_params,
            {"accel_profile": accel_profile},
        ],
    )

    flatscan = Node(
        package="jt128_nav_tools",
        executable="laser_scan_to_flatscan",
        name="laser_scan_to_flatscan",
        output="screen",
        prefix=cpu_affinity_prefix("laser_scan_to_flatscan"),
        condition=IfCondition(start_flatscan),
        parameters=[flatscan_params],
        remappings=[
            ("scan", "/scan"),
            ("flatscan", "/flatscan"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("accel_profile", default_value="ipc_worker"),
            DeclareLaunchArgument("accel_params", default_value=str(accel_params_default)),
            DeclareLaunchArgument("start_flatscan", default_value="true"),
            DeclareLaunchArgument("flatscan_params", default_value=str(flatscan_params_default)),
            accel_axis,
            flatscan,
        ]
    )
