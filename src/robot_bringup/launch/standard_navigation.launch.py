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
    keepout_mask_yaml = LaunchConfiguration("keepout_mask_yaml")
    speed_mask_yaml = LaunchConfiguration("speed_mask_yaml")
    use_respawn = LaunchConfiguration("use_respawn")
    use_composition = LaunchConfiguration("use_composition")
    log_level = LaunchConfiguration("log_level")

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("robot_nav_config"), "config", "nav2.yaml"]
    )
    default_keepout_mask_yaml = PathJoinSubstitution(
        [FindPackageShare("robot_nav_config"), "config", "neutral_keepout_mask.yaml"]
    )
    default_speed_mask_yaml = PathJoinSubstitution(
        [FindPackageShare("robot_nav_config"), "config", "neutral_speed_mask.yaml"]
    )

    lifecycle_nodes = [
        "keepout_filter_mask_server",
        "keepout_costmap_filter_info_server",
        "speed_filter_mask_server",
        "speed_costmap_filter_info_server",
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
            DeclareLaunchArgument("keepout_mask_yaml", default_value=default_keepout_mask_yaml),
            DeclareLaunchArgument("speed_mask_yaml", default_value=default_speed_mask_yaml),
            DeclareLaunchArgument("use_respawn", default_value="False"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="keepout_filter_mask_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"yaml_filename": keepout_mask_yaml},
                    {"topic_name": "/keepout_filter_mask"},
                    {"frame_id": "map"},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="keepout_costmap_filter_info_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"type": 0},
                    {"filter_info_topic": "/costmap_filter_info/keepout"},
                    {"mask_topic": "/keepout_filter_mask"},
                    {"base": 0.0},
                    {"multiplier": 1.0},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="speed_filter_mask_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"yaml_filename": speed_mask_yaml},
                    {"topic_name": "/speed_filter_mask"},
                    {"frame_id": "map"},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="speed_costmap_filter_info_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"type": 1},
                    {"filter_info_topic": "/costmap_filter_info/speed"},
                    {"mask_topic": "/speed_filter_mask"},
                    {"base": 0.0},
                    {"multiplier": 1.0},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
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
