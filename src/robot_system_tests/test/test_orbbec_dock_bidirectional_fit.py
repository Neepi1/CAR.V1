import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = (
    ROOT
    / "scripts"
    / "jetson"
    / "runtime_overlay"
    / "scripts"
    / "orbbec_dock_bidirectional_fit.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("orbbec_dock_bidirectional_fit", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def observation(module, gap, lateral=-0.03, yaw=0.01, confidence=0.90):
    return module.ObservationSnapshot(
        sensor_healthy=True,
        valid=True,
        forward_gap_m=gap,
        lateral_error_m=lateral,
        yaw_error_rad=yaw,
        confidence=confidence,
        source="orbbec_336l_depth",
    )


def test_bidirectional_servo_commands_only_the_requested_leg_direction():
    module = load_module()
    config = module.SweepServoConfig()

    forward = module.compute_leg_command(
        observation(module, 0.80), target_gap_m=0.45, direction=1, config=config
    )
    reverse = module.compute_leg_command(
        observation(module, 0.45), target_gap_m=0.80, direction=-1, config=config
    )

    assert forward.state == "drive"
    assert 0.0 < forward.speed_mps <= config.max_speed_mps
    assert reverse.state == "drive"
    assert -config.max_reverse_speed_mps <= reverse.speed_mps < 0.0

    wrong_way = module.compute_leg_command(
        observation(module, 0.70), target_gap_m=0.45, direction=-1, config=config
    )
    assert wrong_way.state == "direction_mismatch"
    assert wrong_way.speed_mps == 0.0


def test_bidirectional_servo_stops_at_target_and_on_geometry_gates():
    module = load_module()
    config = module.SweepServoConfig()

    at_target = module.compute_leg_command(
        observation(module, 0.455), target_gap_m=0.45, direction=1, config=config
    )
    assert at_target.state == "at_target"
    assert at_target.speed_mps == 0.0

    lateral = module.compute_leg_command(
        observation(module, 0.70, lateral=0.15),
        target_gap_m=0.45,
        direction=1,
        config=config,
    )
    yaw = module.compute_leg_command(
        observation(module, 0.70, yaw=0.20),
        target_gap_m=0.45,
        direction=1,
        config=config,
    )
    assert lateral.state == "lateral_gate"
    assert yaw.state == "yaw_gate"


def test_range_motion_fit_reports_scale_offset_and_r_squared():
    module = load_module()

    fit = module.fit_range_motion(
        wheel_forward_m=[0.0, 0.10, 0.20, 0.30],
        observed_range_change_m=[0.002, 0.103, 0.204, 0.305],
    )

    assert abs(fit["scale"] - 1.01) < 1.0e-9
    assert abs(fit["offset_m"] - 0.002) < 1.0e-9
    assert fit["r_squared"] > 0.999999
    assert fit["sample_count"] == 4


def test_range_fit_rows_exclude_invalid_zero_geometry_and_wrong_source():
    module = load_module()
    rows = [
        {
            "observation_sensor_healthy": True,
            "observation_valid": True,
            "observation_source": "orbbec_336l_depth",
            "wheel_x_m": 0.0,
            "wheel_y_m": 0.0,
            "forward_gap_m": 1.20,
            "lateral_error_m": 0.02,
            "yaw_error_rad": 0.01,
            "confidence": 0.85,
        },
        {
            "observation_sensor_healthy": True,
            "observation_valid": True,
            "observation_source": "orbbec_336l_depth",
            "wheel_x_m": -0.05,
            "wheel_y_m": 0.0,
            "forward_gap_m": 1.25,
            "lateral_error_m": 0.02,
            "yaw_error_rad": 0.01,
            "confidence": 0.84,
        },
        {
            "observation_sensor_healthy": False,
            "observation_valid": False,
            "observation_source": "orbbec_336l_depth",
            "wheel_x_m": -0.10,
            "wheel_y_m": 0.0,
            "forward_gap_m": 0.0,
            "lateral_error_m": 0.0,
            "yaw_error_rad": 0.0,
            "confidence": 0.0,
        },
        {
            "observation_sensor_healthy": True,
            "observation_valid": True,
            "observation_source": "unexpected_source",
            "wheel_x_m": -0.12,
            "wheel_y_m": 0.0,
            "forward_gap_m": 1.32,
            "lateral_error_m": 0.02,
            "yaw_error_rad": 0.01,
            "confidence": 0.83,
        },
        {
            "observation_sensor_healthy": True,
            "observation_valid": True,
            "observation_source": "orbbec_336l_depth",
            "wheel_x_m": -0.15,
            "wheel_y_m": 0.0,
            "forward_gap_m": 1.35,
            "lateral_error_m": 0.02,
            "yaw_error_rad": 0.01,
            "confidence": 0.82,
        },
    ]

    selected = module.select_valid_fit_rows(rows, "orbbec_336l_depth")
    fit_x = [float(row["wheel_x_m"]) for row in selected]
    fit_y = [1.20 - float(row["forward_gap_m"]) for row in selected]
    fit = module.fit_range_motion(fit_x, fit_y)

    assert len(selected) == 3
    assert all(float(row["forward_gap_m"]) > 0.0 for row in selected)
    assert abs(fit["scale"] - 1.0) < 1.0e-9
    assert fit["r_squared"] > 0.999999


def test_reverse_leg_uses_the_docking_reverse_permit_contract():
    module = load_module()
    args = module.parse_args(["--enable-reverse"])
    source = MODULE_PATH.read_text(encoding="utf-8")

    assert args.reverse_enable_topic == "/ranger_mini3/docking_allow_reverse"
    assert args.reverse_permit_settle_sec > 0.0
    assert "self.reverse_enable_pub" in source
    assert "node.publish_reverse_enable(direction < 0)" in source
    assert "node.publish_reverse_enable(False)" in source


def test_return_to_far_mode_runs_only_the_reverse_leg():
    module = load_module()

    assert module.build_sweep_targets(cycles=2, return_to_far_only=False) == [
        (1, "forward"),
        (-1, "reverse"),
        (1, "forward"),
        (-1, "reverse"),
    ]
    assert module.build_sweep_targets(cycles=2, return_to_far_only=True) == [
        (-1, "reverse")
    ]


def test_reverse_minimum_gap_guard_stops_and_confirms_before_aborting():
    module = load_module()

    first = module.evaluate_minimum_safe_gap(
        forward_gap_m=0.31,
        minimum_gap_m=0.40,
        direction=-1,
        low_streak=0,
        reverse_confirm_samples=3,
    )
    second = module.evaluate_minimum_safe_gap(
        forward_gap_m=0.32,
        minimum_gap_m=0.40,
        direction=-1,
        low_streak=first.low_streak,
        reverse_confirm_samples=3,
    )
    recovered = module.evaluate_minimum_safe_gap(
        forward_gap_m=0.78,
        minimum_gap_m=0.40,
        direction=-1,
        low_streak=second.low_streak,
        reverse_confirm_samples=3,
    )
    confirmed = module.evaluate_minimum_safe_gap(
        forward_gap_m=0.30,
        minimum_gap_m=0.40,
        direction=-1,
        low_streak=second.low_streak,
        reverse_confirm_samples=3,
    )

    assert first.state == "hold"
    assert first.low_streak == 1
    assert second.state == "hold"
    assert second.low_streak == 2
    assert recovered.state == "clear"
    assert recovered.low_streak == 0
    assert confirmed.state == "abort"
    assert confirmed.low_streak == 3


def test_forward_minimum_gap_guard_aborts_on_first_low_sample():
    module = load_module()

    decision = module.evaluate_minimum_safe_gap(
        forward_gap_m=0.39,
        minimum_gap_m=0.40,
        direction=1,
        low_streak=0,
        reverse_confirm_samples=3,
    )

    assert decision.state == "abort"
    assert decision.low_streak == 1


def test_minimum_gap_guard_ignores_invalid_and_does_not_recount_same_frame():
    module = load_module()

    invalid = module.evaluate_minimum_safe_gap(
        forward_gap_m=0.0,
        minimum_gap_m=0.40,
        direction=-1,
        low_streak=2,
        reverse_confirm_samples=3,
        observation_usable=False,
        is_new_observation=True,
    )
    repeated = module.evaluate_minimum_safe_gap(
        forward_gap_m=0.31,
        minimum_gap_m=0.40,
        direction=-1,
        low_streak=1,
        reverse_confirm_samples=3,
        observation_usable=True,
        is_new_observation=False,
    )

    assert invalid.state == "clear"
    assert invalid.low_streak == 0
    assert repeated.state == "hold"
    assert repeated.low_streak == 1
