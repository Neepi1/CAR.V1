import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def value(text, key):
    match = re.search(rf"^\s*{re.escape(key)}:\s*([^\n#]+)", text, re.MULTILINE)
    assert match, key
    raw = match.group(1).strip()
    if raw in ("true", "false"):
        return raw == "true"
    return ast.literal_eval(raw)


def test_prediction_smoother_coefficients_match_actual_smoother():
    paths = [ROOT / "src/robot_nav_config/config/nav2.yaml",
             ROOT / "scripts/jetson/runtime_overlay/config/nav2.yaml"]
    assert paths[0].read_bytes() == paths[1].read_bytes()
    config = paths[0].read_text(encoding="utf-8")
    follow = config.split("    FollowPath:\n", 1)[1].split("    FollowPathFallback:", 1)[0]
    smoother = config.split("\nvelocity_smoother:\n", 1)[1].split("\ncollision_monitor:", 1)[0]
    assert value(follow, "primary_controller") == "robot_nav_config::RangerMPPIController"
    dynamics = follow.split("      RangerDynamics:\n", 1)[1]
    assert value(dynamics, "enabled") is True
    assert value(dynamics, "smoother_max_accel") == value(smoother, "max_accel")
    assert value(dynamics, "smoother_max_decel") == value(smoother, "max_decel")
    assert value(smoother, "feedback") == "OPEN_LOOP"
    assert value(smoother, "scale_velocities") is False
    assert value(smoother, "deadband_velocity") == [0.0, 0.0, 0.0]
    assert value(follow, "motion_model") == "Ackermann"
    assert value(follow, "vx_min") == 0.0
    assert value(follow, "vy_max") == 0.0
    assert value(follow, "min_turning_r") == 0.81
    # Plant identification is not substituted for the output smoother limits.
    assert value(dynamics, "acceleration_mps2") != value(smoother, "max_accel")[0]
    assert config.count("      RangerDynamics:") == 1


def test_chassis_model_has_no_new_command_or_motion_gate():
    source = (ROOT / "src/robot_nav_config/src/chassis_dynamics/mppi_controller.cpp").read_text()
    assert "create_publisher" not in source
    assert "create_client" not in source
    assert '"/cmd_vel"' in source
    assert '"/cmd_vel_nav"' in source
    assert "optimizer.evalControl" in source
    assert "class RangerOptimizer : public mppi::Optimizer" in source
    assert "motion_model_ = model_" in source
