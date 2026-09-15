import os
import shlex
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


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
            and key in ("CONTROLLER_SERVER", "NAV2_LIFECYCLE_MANAGER")):
        overlay = os.environ.get("NJRH_OVERLAY_ROOT") or next((
            parent / "scripts/jetson/runtime_overlay"
            for parent in Path(__file__).resolve().parents
            if (parent / "scripts/jetson/runtime_overlay/scripts").is_dir()
        ), None)
        if overlay:
            return shlex.join([
                "python3", (Path(overlay) / "scripts/startup_cpu_affinity.py").as_posix(),
                "exec", "--session", session, "--steady-cpus", cpuset,
                "--role", key.lower(), "--",
            ])
    return f"taskset -c {cpuset}"


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("robot_nav_config"), "config", "nav2.yaml"]
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("use_respawn", default_value="False"),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_local_costmap_debug")],
                arguments=["--ros-args", "--log-level", log_level],
                prefix=cpu_affinity_prefix("controller_server"),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_local_costmap_debug",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                prefix=cpu_affinity_prefix("nav2_lifecycle_manager"),
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": ["controller_server"]},
                ],
            ),
        ]
    )
