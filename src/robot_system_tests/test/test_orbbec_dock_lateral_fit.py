import importlib.util
import math
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[3]
    / "scripts"
    / "jetson"
    / "runtime_overlay"
    / "scripts"
    / "orbbec_dock_lateral_fit.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("orbbec_dock_lateral_fit", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def observation(module, lateral=-0.05, gap=0.80, yaw=0.01, confidence=0.90):
    return module.ObservationSnapshot(
        sensor_healthy=True,
        valid=True,
        forward_gap_m=gap,
        lateral_error_m=lateral,
        yaw_error_rad=yaw,
        lateral_span_m=0.18,
        confidence=confidence,
        source="orbbec_336l_depth",
    )


def test_positive_body_y_targets_lower_observed_lateral():
    module = load_module()
    config = module.LateralServoConfig(response_sign=-1)
    command = module.compute_lateral_command(
        observation(module, lateral=-0.05),
        target_lateral_m=-0.10,
        direction=1,
        config=config,
    )
    assert command.state == "drive"
    assert command.speed_mps > 0.0


def test_negative_body_y_targets_higher_observed_lateral():
    module = load_module()
    config = module.LateralServoConfig(response_sign=-1)
    command = module.compute_lateral_command(
        observation(module, lateral=-0.05),
        target_lateral_m=0.02,
        direction=-1,
        config=config,
    )
    assert command.state == "drive"
    assert command.speed_mps < 0.0


def test_lateral_servo_stops_at_target_and_rejects_wrong_direction():
    module = load_module()
    config = module.LateralServoConfig(response_sign=-1)
    at_target = module.compute_lateral_command(
        observation(module, lateral=-0.050), -0.054, direction=1, config=config
    )
    wrong = module.compute_lateral_command(
        observation(module, lateral=-0.050), -0.100, direction=-1, config=config
    )
    assert at_target.state == "at_target"
    assert at_target.speed_mps == 0.0
    assert wrong.state == "direction_mismatch"


def test_lateral_servo_fails_closed_on_gap_yaw_and_invalid_observation():
    module = load_module()
    config = module.LateralServoConfig(response_sign=-1)
    gap = module.compute_lateral_command(
        observation(module, gap=0.60), -0.10, direction=1, config=config
    )
    yaw = module.compute_lateral_command(
        observation(module, yaw=0.20), -0.10, direction=1, config=config
    )
    invalid_observation = observation(module)
    invalid_observation = module.ObservationSnapshot(
        **{**invalid_observation.__dict__, "valid": False}
    )
    invalid = module.compute_lateral_command(
        invalid_observation, -0.10, direction=1, config=config
    )
    assert gap.state == "forward_gap_gate"
    assert yaw.state == "yaw_gate"
    assert invalid.state == "invalid_observation"


def test_targets_are_symmetric_and_return_to_baseline():
    module = load_module()
    targets = module.build_lateral_targets(-0.05, 0.08, cycles=1, response_sign=-1)
    assert targets == [
        (-0.13, 1, "left_1"),
        (0.03, -1, "right_1"),
        (-0.05, 1, "return_baseline"),
    ]


def test_body_projection_handles_rotated_odom_frame():
    module = load_module()
    forward, lateral = module.project_body_displacement(
        1.0,
        2.0,
        math.pi / 2.0,
        0.90,
        2.20,
    )
    assert math.isclose(forward, 0.20, abs_tol=1.0e-9)
    assert math.isclose(lateral, 0.10, abs_tol=1.0e-9)


def test_lateral_fit_converts_negative_camera_response_to_positive_scale():
    module = load_module()
    fit = module.fit_lateral_motion(
        wheel_lateral_m=[0.0, 0.02, 0.04, 0.06],
        observation_lateral_delta_m=[0.0, -0.021, -0.042, -0.063],
        response_sign=-1,
    )
    assert math.isclose(fit["scale"], 1.05, rel_tol=1.0e-9)
    assert math.isclose(fit["offset_m"], 0.0, abs_tol=1.0e-9)
    assert math.isclose(fit["r_squared"], 1.0, abs_tol=1.0e-9)


def test_probe_response_ratio_preserves_sign():
    module = load_module()
    ratio = module.probe_response_ratio(0.020, -0.019)
    assert math.isclose(ratio, -0.95, abs_tol=1.0e-9)


def test_crossed_target_accepts_only_bounded_stopped_overshoot():
    module = load_module()
    assert module.crossed_target_accepted(False, True, True, -0.011, 0.020)
    assert not module.crossed_target_accepted(False, True, True, -0.021, 0.020)
    assert not module.crossed_target_accepted(False, True, False, -0.005, 0.020)


def test_ranger_mini3_parallel_mode_is_a_lateral_mode():
    module = load_module()
    assert module.is_lateral_motion_mode(1)
    assert module.is_lateral_motion_mode(3)
    assert not module.is_lateral_motion_mode(0)


def test_active_fit_sample_requires_confirmed_lateral_motion():
    module = load_module()
    assert module.is_active_lateral_fit_sample(1, True, 0.025, 0.005)
    assert module.is_active_lateral_fit_sample(3, True, -0.025, 0.005)
    assert not module.is_active_lateral_fit_sample(1, False, 0.025, 0.005)
    assert not module.is_active_lateral_fit_sample(0, True, 0.025, 0.005)
    assert not module.is_active_lateral_fit_sample(1, True, 0.001, 0.005)
