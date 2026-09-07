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
def test_collision_monitor_matches_the_mppi_collision_envelope(config_path: Path):
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
    collision_monitor = nav2[nav2.index("\ncollision_monitor:\n") :]
    stop_zone = _section(collision_monitor, "    StopZone:\n", "    SlowZone:\n")
    slow_zone = _section(collision_monitor, "    SlowZone:\n", "    FootprintApproach:\n")
    approach = _section(
        collision_monitor,
        "    FootprintApproach:\n",
        "    observation_sources:",
    )

    padding = _number(local_costmap, "footprint_padding")
    resolution = _number(local_costmap, "resolution")
    collision_margin = _number(obstacle_critic, "collision_margin_distance")
    footprint = _footprint(local_costmap)
    required_x = math.ceil(
        (max(abs(float(point[0])) for point in footprint) + padding + collision_margin)
        / resolution
    ) * resolution
    required_y = math.ceil(
        (max(abs(float(point[1])) for point in footprint) + padding + collision_margin)
        / resolution
    ) * resolution

    stop_points = _points(stop_zone)
    stop_min_x = min(point[0] for point in stop_points)
    stop_max_x = max(point[0] for point in stop_points)
    stop_min_y = min(point[1] for point in stop_points)
    stop_max_y = max(point[1] for point in stop_points)
    assert stop_min_x <= -required_x + 1e-9
    assert stop_max_x >= required_x - 1e-9
    assert stop_min_y <= -required_y + 1e-9
    assert stop_max_y >= required_y - 1e-9

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
def test_transient_controller_failure_waits_within_the_progress_budget(config_path: Path):
    nav2 = config_path.read_text(encoding="utf-8")
    controller_server = _section(nav2, "controller_server:\n", "\nbehavior_server:\n")
    failure_tolerance = _number(controller_server, "failure_tolerance")
    movement_time_allowance = _number(controller_server, "movement_time_allowance")

    assert failure_tolerance == pytest.approx(10.0)
    assert failure_tolerance < movement_time_allowance
