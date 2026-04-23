from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    use_composition = LaunchConfiguration("use_composition")
    log_level = LaunchConfiguration("log_level")

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("robot_nav_config"), "config", "nav2.yaml"]
    )

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor",
    ]

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

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

    node_kwargs = {
        "output": "screen",
        "respawn": use_respawn,
        "respawn_delay": 2.0,
        "parameters": [configured_params],
        "arguments": ["--ros-args", "--log-level", log_level],
    }

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("use_respawn", default_value="False"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("log_level", default_value="info"),
            # This runtime path stays non-composed intentionally so the field overlay
            # can keep deterministic process ownership for crash / restart handling.
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                remappings=remappings + [("cmd_vel", "cmd_vel_nav_raw")],
                **node_kwargs,
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                remappings=remappings,
                **node_kwargs,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                remappings=remappings,
                **node_kwargs,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                **node_kwargs,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                remappings=remappings,
                **node_kwargs,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                remappings=remappings,
                **node_kwargs,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                remappings=remappings + [("cmd_vel", "cmd_vel_nav_raw"), ("cmd_vel_smoothed", "cmd_vel_nav")],
                **node_kwargs,
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                remappings=remappings,
                **node_kwargs,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
    )
