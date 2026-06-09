#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "robot_isaac_nitros_pointcloud skeleton loaded: NITROS is reserved "
                    "for compact navigation branches only; /lidar_points remains the "
                    "full-density mapping trunk. Future NITROS accelerated branch "
                    "nodes must run in one same-process component container."
                )
            )
        ]
    )
