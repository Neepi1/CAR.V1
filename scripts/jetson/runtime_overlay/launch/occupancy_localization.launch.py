#!/usr/bin/env python3
import os
import shlex
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessIO, OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from isaac_startup_state import IsaacStartupState


def cpu_affinity_prefix(service_name):
    enabled = os.environ.get("NJRH_CPU_AFFINITY_ENABLED", "true").lower()
    if enabled not in ("1", "true", "yes", "on"):
        return None
    key = service_name.upper().replace("-", "_").replace(".", "_").replace("/", "_")
    cpuset = os.environ.get(f"NJRH_CPUSET_{key}", "")
    if not cpuset:
        return None
    session = os.environ.get("NJRH_STARTUP_CPU_SESSION", "")
    if (session and os.environ.get("NJRH_NAVIGATION_CPU_PROFILE") == "navigation_5cpu"
            and key in ("NAV2_MAP_SERVER", "NAV2_LIFECYCLE_MANAGER", "OCCUPANCY_GRID_LOCALIZER")):
        overlay = Path(os.environ.get("NJRH_OVERLAY_ROOT", Path(__file__).resolve().parents[1]))
        return shlex.join([
            "python3", (overlay / "scripts/startup_cpu_affinity.py").as_posix(),
            "exec", "--session", session, "--steady-cpus", cpuset,
            "--role", key.lower(), "--",
        ])
    return f"taskset -c {cpuset}"


def generate_launch_description():
    upstream_root = Path(os.environ.get("NJRH_UPSTREAM_ROOT", "/workspaces/isaac_ros-dev"))
    overlay_root = Path(__file__).resolve().parents[1]
    params_dir = upstream_root / "nav2_test" / "params"

    map_yaml_default = upstream_root / "maps" / "jt128_map.yaml"
    localizer_params_default = overlay_root / "config" / "jt128_occupancy_grid_localizer.yaml"

    map_yaml = LaunchConfiguration("map_yaml")
    localizer_map_yaml = LaunchConfiguration("localizer_map_yaml")
    localizer_params = LaunchConfiguration("localizer_params")
    flatscan_topic = LaunchConfiguration("flatscan_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_map_server = LaunchConfiguration("start_map_server")
    map_lifecycle_manager_enabled = LaunchConfiguration("map_lifecycle_manager_enabled")
    map_frame = LaunchConfiguration("map_frame")
    log_level = LaunchConfiguration("log_level")

    map_server = Node(
        condition=IfCondition(start_map_server),
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        prefix=cpu_affinity_prefix("nav2_map_server"),
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
        }],
    )

    lifecycle_manager = Node(
        condition=IfCondition(PythonExpression([
            "'", start_map_server,
            "'.lower() in ['true', '1', 'yes', 'on'] and '",
            map_lifecycle_manager_enabled,
            "'.lower() in ['true', '1', 'yes', 'on']",
        ])),
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        prefix=cpu_affinity_prefix("nav2_lifecycle_manager"),
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )

    occupancy_grid_localizer = ComposableNode(
        package="isaac_ros_occupancy_grid_localizer",
        plugin="nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode",
        name="occupancy_grid_localizer",
        parameters=[
            localizer_map_yaml,
            str(localizer_params_default),
            localizer_params,
            {
                "map_yaml_path": localizer_map_yaml,
                "loc_result_frame": map_frame,
            },
        ],
        remappings=[
            ("flatscan", flatscan_topic),
            ("localization_result", "/localization_result"),
        ],
    )

    localizer_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_isolated",
        name="occupancy_grid_localizer_container",
        namespace="",
        composable_node_descriptions=[occupancy_grid_localizer],
        output="screen",
        prefix=cpu_affinity_prefix("occupancy_grid_localizer"),
        # The pinned Nitros completion marker is INFO on this logger only.
        arguments=["--ros-args", "--log-level", log_level,
                   "--log-level", "occupancy_grid_localizer:=info"],
        respawn=True,
        respawn_delay=0.5,
    )

    def register_startup_evidence(context):
        evidence = IsaacStartupState(
            os.environ.get("NJRH_ISAAC_STARTUP_STATE_FILE", ""),
            map_yaml.perform(context), localizer_map_yaml.perform(context))

        def started(event, _context):
            evidence.started(event.pid)

        def exited(event, _context):
            evidence.exited(event.pid)

        def stdout(event):
            evidence.output(event.pid, "stdout", event.text)

        def stderr(event):
            evidence.output(event.pid, "stderr", event.text)

        return [
            RegisterEventHandler(OnProcessStart(target_action=localizer_container, on_start=started)),
            RegisterEventHandler(OnProcessIO(target_action=localizer_container,
                                           on_stdout=stdout, on_stderr=stderr)),
            RegisterEventHandler(OnProcessExit(target_action=localizer_container, on_exit=exited)),
        ]

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", default_value=str(map_yaml_default)),
        DeclareLaunchArgument("localizer_map_yaml", default_value=str(map_yaml_default)),
        DeclareLaunchArgument("localizer_params", default_value=str(localizer_params_default)),
        DeclareLaunchArgument("flatscan_topic", default_value="/flatscan"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_map_server", default_value="true"),
        DeclareLaunchArgument("map_lifecycle_manager_enabled", default_value="true"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("log_level", default_value="info"),
        OpaqueFunction(function=register_startup_evidence),
        map_server,
        lifecycle_manager,
        localizer_container,
    ])
