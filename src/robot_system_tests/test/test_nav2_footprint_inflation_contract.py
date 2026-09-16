import ast
import math
import re
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
CONFIGS = (
    ROOT / "src" / "robot_nav_config" / "config" / "nav2.yaml",
    ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml",
)


def _section(text: str, start: str, end: str) -> str:
    start_index = text.index(start)
    return text[start_index : text.index(end, start_index)]


def _number(text: str, key: str) -> float:
    match = re.search(
        rf"^\s*{re.escape(key)}:\s*([-+]?\d+(?:\.\d+)?)\s*$",
        text,
        re.MULTILINE,
    )
    assert match, key
    return float(match.group(1))


def _footprint(text: str) -> list[list[float]]:
    match = re.search(r'^\s*footprint:\s*"([^\"]+)"\s*$', text, re.MULTILINE)
    assert match, "footprint"
    return ast.literal_eval(match.group(1))


def _points(text: str) -> list[list[float]]:
    match = re.search(r"^\s*points:\s*(\[[^\n]+\])\s*$", text, re.MULTILINE)
    assert match, "points"
    flat_points = ast.literal_eval(match.group(1))
    assert len(flat_points) % 2 == 0
    return [flat_points[index : index + 2] for index in range(0, len(flat_points), 2)]


@pytest.mark.parametrize("config_path", CONFIGS)
def test_local_keepout_runs_before_matching_post_filter_inflation(config_path: Path):
    nav2 = config_path.read_text(encoding="utf-8")
    local = _section(nav2, "local_costmap:\n  local_costmap:\n", "\ncollision_monitor:\n")
    global_map = _section(nav2, "global_costmap:\n", "\nlocal_costmap:\n")
    assert 'filters: ["keepout_filter", "keepout_inflation_layer"]' in local
    keepout = _section(local, "      keepout_filter:\n", "      keepout_inflation_layer:\n")
    assert 'plugin: "nav2_costmap_2d::KeepoutFilter"' in keepout
    assert "enabled: true" in keepout
    topic = "filter_info_topic: /costmap_filter_info/keepout"
    assert topic in keepout and topic in global_map
    post = local.split("      keepout_inflation_layer:\n", 1)[1]
    original = _section(local, "      local_inflation_layer:\n", "      keepout_filter:\n")
    assert 'plugin: "nav2_costmap_2d::InflationLayer"' in post
    assert "enabled: true" in post
    for key in ("inflation_radius", "cost_scaling_factor"):
        assert _number(post, key) == pytest.approx(_number(original, key))
    assert 'plugins: ["obstacle_layer", "local_inflation_layer"]' in local
    assert "global_frame: odom" in local


@pytest.mark.parametrize("config_path", CONFIGS)
def test_local_inflation_covers_padded_rectangular_footprint_and_margin(config_path: Path):
    nav2 = config_path.read_text(encoding="utf-8")
    controller = _section(nav2, "    FollowPath:\n", "    FollowPathFallback:\n")
    obstacle_critic = _section(
        controller,
        "      ObstaclesCritic:\n",
        "      PathAlignCritic:\n",
    )
    local_costmap = _section(
        nav2,
        "local_costmap:\n  local_costmap:\n",
        "\ncollision_monitor:\n",
    )
    local_inflation = local_costmap[local_costmap.index("      local_inflation_layer:\n") :]

    padding = _number(local_costmap, "footprint_padding")
    resolution = _number(local_costmap, "resolution")
    collision_margin = _number(obstacle_critic, "collision_margin_distance")
    padded_circumscribed_radius = max(
        math.hypot(abs(float(x)) + padding, abs(float(y)) + padding)
        for x, y in _footprint(local_costmap)
    )
    required_radius = (
        math.ceil((padded_circumscribed_radius + collision_margin) / resolution)
        * resolution
    )
    local_radius = _number(local_inflation, "inflation_radius")
    critic_radius = _number(obstacle_critic, "inflation_radius")

    assert local_radius >= required_radius - 1e-9, (
        f"{config_path}: local inflation {local_radius:.2f} m does not cover "
        f"padded circumscribed radius {padded_circumscribed_radius:.3f} m + "
        f"collision margin {collision_margin:.2f} m on a {resolution:.2f} m grid; "
        f"need at least {required_radius:.2f} m"
    )
    assert critic_radius == pytest.approx(local_radius), (
        f"{config_path}: MPPI ObstaclesCritic inflation radius must match "
        "local_inflation_layer"
    )


@pytest.mark.parametrize("config_path", CONFIGS)
def test_collision_monitor_uses_requested_stop_zone_and_covers_padded_body(config_path: Path):
    nav2 = config_path.read_text(encoding="utf-8")
    local_costmap = _section(
        nav2,
        "local_costmap:\n  local_costmap:\n",
        "\ncollision_monitor:\n",
    )
    collision_monitor = nav2[nav2.index("\ncollision_monitor:\n") :]
    stop_zone = _section(collision_monitor, "    StopZone:\n", "    SlowZone:\n")
    slow_zone = _section(collision_monitor, "    SlowZone:\n", "    FootprintApproach:\n")
    approach = _section(
        collision_monitor,
        "    FootprintApproach:\n",
        "    observation_sources:",
    )

    padding = _number(local_costmap, "footprint_padding")
    footprint = _footprint(local_costmap)
    required_x = max(abs(float(point[0])) for point in footprint) + padding
    required_y = max(abs(float(point[1])) for point in footprint) + padding

    stop_points = _points(stop_zone)
    assert stop_points == [[0.47, 0.36], [0.47, -0.36], [-0.47, -0.36], [-0.47, 0.36]]
    stop_min_x = min(point[0] for point in stop_points)
    stop_max_x = max(point[0] for point in stop_points)
    stop_min_y = min(point[1] for point in stop_points)
    stop_max_y = max(point[1] for point in stop_points)
    assert stop_min_x <= -required_x + 1e-9
    assert stop_max_x >= required_x - 1e-9
    assert stop_min_y <= -required_y + 1e-9
    assert stop_max_y >= required_y - 1e-9

    # A StopZone coincident with the scan mask cannot see lateral returns.
    mask = (ROOT / "scripts/jetson/runtime_overlay/config/pointcloud_accel_axis.yaml").read_text(
        encoding="utf-8"
    )
    assert _number(mask, "scan_worker_self_mask_max_x") == pytest.approx(0.39)
    assert _number(mask, "scan_worker_self_mask_max_y") == pytest.approx(0.28)
    assert stop_max_x - _number(mask, "scan_worker_self_mask_max_x") >= 0.08 - 1e-9
    assert stop_max_y - _number(mask, "scan_worker_self_mask_max_y") >= 0.08 - 1e-9

    slow_points = _points(slow_zone)
    assert min(point[0] for point in slow_points) <= stop_min_x
    assert max(point[0] for point in slow_points) >= stop_max_x
    assert min(point[1] for point in slow_points) <= stop_min_y
    assert max(point[1] for point in slow_points) >= stop_max_y

    assert _number(stop_zone, "max_points") == 1
    assert 'polygons: ["StopZone", "SlowZone", "FootprintApproach"]' in collision_monitor
    assert 'action_type: "approach"' in approach
    assert 'footprint_topic: "/local_costmap/published_footprint"' in approach
    assert _number(approach, "time_before_collision") == pytest.approx(2.0)
    assert _number(approach, "simulation_time_step") == pytest.approx(0.1)


@pytest.mark.parametrize("config_path", CONFIGS)
def test_ordinary_mppi_is_forward_only_while_terminal_reverse_remains_available(config_path: Path):
    nav2 = config_path.read_text(encoding="utf-8")
    controller = _section(nav2, "    FollowPath:\n", "    FollowPathFallback:\n")
    assert _number(controller, "vx_min") == pytest.approx(0.0)
    assert _number(controller, "terminal_handoff_reverse_max_speed_mps") == pytest.approx(0.08)
    assert _number(controller, "terminal_handoff_goal_xy_tolerance_m") == pytest.approx(0.06)
    assert _number(controller, "terminal_handoff_minimum_abs_lateral_m") == pytest.approx(0.06)
    smoother = _section(nav2, "velocity_smoother:\n", "\nlocal_costmap:\n")
    match = re.search(r"^\s*min_velocity:\s*(\[[^\n]+\])", smoother, re.MULTILINE)
    assert match
    assert ast.literal_eval(match.group(1))[0] <= -0.08


@pytest.mark.parametrize("config_path", CONFIGS)
def test_transient_controller_failure_waits_within_the_progress_budget(config_path: Path):
    nav2 = config_path.read_text(encoding="utf-8")
    controller_server = _section(nav2, "controller_server:\n", "\nbehavior_server:\n")
    failure_tolerance = _number(controller_server, "failure_tolerance")
    movement_time_allowance = _number(controller_server, "movement_time_allowance")

    assert failure_tolerance == pytest.approx(10.0)
    assert failure_tolerance < movement_time_allowance


@pytest.mark.parametrize("config_path", CONFIGS)
def test_ordinary_local_repair_matches_ranger_and_local_costmap_contract(config_path: Path):
    nav2 = config_path.read_text(encoding="utf-8")
    controller = _section(nav2, "    FollowPath:\n", "    FollowPathFallback:\n")
    obstacle_critic = _section(
        controller,
        "      ObstaclesCritic:\n",
        "      PathAlignCritic:\n",
    )
    local_costmap = _section(
        nav2,
        "local_costmap:\n  local_costmap:\n",
        "\ncollision_monitor:\n",
    )

    assert "ordinary_local_repair_enabled: true" in controller
    assert _number(controller, "ordinary_local_repair_minimum_turning_radius_m") == pytest.approx(
        _number(controller, "min_turning_r")
    )
    assert _number(controller, "ordinary_local_repair_collision_margin_m") == pytest.approx(
        _number(obstacle_critic, "collision_margin_distance")
    )
    assert _number(controller, "ordinary_local_repair_max_planning_time_sec") <= _number(
        controller, "ordinary_local_repair_retry_period_sec"
    )

    width = _number(local_costmap, "width")
    lookahead = _number(controller, "ordinary_local_repair_lookahead_distance_m")
    footprint = _footprint(local_costmap)
    padding = _number(local_costmap, "footprint_padding")
    margin = _number(controller, "ordinary_local_repair_collision_margin_m")
    circumscribed = max(
        math.hypot(abs(float(x)) + padding + margin, abs(float(y)) + padding + margin)
        for x, y in footprint
    )
    assert lookahead + circumscribed < width / 2.0

    progress_topic = "/ranger_mini3/nav_elevator_scoped_progress_state"
    assert f"ordinary_local_repair_progress_topic: {progress_topic}" in controller
    assert (
        "ordinary_local_repair_path_topic: /ranger_mini3/ordinary_local_repair_path"
        in controller
    )
    assert f"elevator_progress_state_topic: {progress_topic}" in nav2


@pytest.mark.parametrize("config_path", CONFIGS)
def test_preferred_clearance_is_separate_from_collision_and_shared_by_repair(config_path: Path):
    nav2 = config_path.read_text(encoding="utf-8")
    controller = _section(nav2, "    FollowPath:\n", "    FollowPathFallback:\n")
    preference = _section(controller, "      RangerClearanceCritic:\n", "      PathAlignCritic:\n")
    local_costmap = _section(nav2, "local_costmap:\n  local_costmap:\n", "\ncollision_monitor:\n")
    stop_zone = _section(nav2, "    StopZone:\n", "    SlowZone:\n")
    assert '- "RangerClearanceCritic"' in controller
    assert 'enabled: true' in preference
    assert _number(preference, "preferred_margin") == pytest.approx(0.05)
    assert _number(preference, "near_goal_distance") == pytest.approx(
        _number(controller, "ordinary_local_repair_minimum_goal_distance_m")
    )
    for axis, key in enumerate(("stop_half_x", "stop_half_y")):
        stop = max(abs(point[axis]) for point in _points(stop_zone))
        assert _number(preference, key) == pytest.approx(stop)
        padded = max(abs(point[axis]) for point in _footprint(local_costmap)) + _number(
            local_costmap, "footprint_padding"
        )
        assert padded == pytest.approx((0.39, 0.28)[axis])
        assert padded + _number(controller, "ordinary_local_repair_collision_margin_m") == pytest.approx(stop)
        assert stop + _number(preference, "preferred_margin") == pytest.approx((0.52, 0.41)[axis])
