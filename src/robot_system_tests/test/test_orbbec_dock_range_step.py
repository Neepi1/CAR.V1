import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = (
    ROOT
    / "scripts"
    / "jetson"
    / "runtime_overlay"
    / "scripts"
    / "orbbec_dock_range_step.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("orbbec_dock_range_step", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def observation(module, gap, lateral=-0.05, yaw=0.01, confidence=0.85):
    return module.ObservationSnapshot(
        sensor_healthy=True,
        valid=True,
        forward_gap_m=gap,
        lateral_error_m=lateral,
        yaw_error_rad=yaw,
        confidence=confidence,
        source="orbbec_336l_depth",
    )


def test_range_servo_is_forward_only_and_speed_limited():
    module = load_module()
    config = module.RangeServoConfig(target_gap_m=0.713)

    far = module.compute_forward_command(observation(module, 0.935), config)
    near = module.compute_forward_command(observation(module, 0.730), config)
    overshot = module.compute_forward_command(observation(module, 0.690), config)

    assert far.state == "drive"
    assert far.speed_mps == config.max_speed_mps
    assert near.state == "drive"
    assert config.min_speed_mps <= near.speed_mps < config.max_speed_mps
    assert overshot.state == "overshoot"
    assert overshot.speed_mps == 0.0


def test_range_servo_requires_stable_geometry_and_expected_source():
    module = load_module()
    config = module.RangeServoConfig(target_gap_m=0.713)

    assert module.compute_forward_command(
        observation(module, 0.80, lateral=0.15), config
    ).state == "lateral_gate"
    assert module.compute_forward_command(
        observation(module, 0.80, yaw=0.20), config
    ).state == "yaw_gate"
    assert module.compute_forward_command(
        observation(module, 0.80, confidence=0.1), config
    ).state == "confidence_gate"

    wrong_source = observation(module, 0.80)
    wrong_source = module.ObservationSnapshot(**{**wrong_source.__dict__, "source": "gs2"})
    assert module.compute_forward_command(wrong_source, config).state == "source_mismatch"


def test_range_servo_stops_inside_tolerance_and_on_invalid_observation():
    module = load_module()
    config = module.RangeServoConfig(target_gap_m=0.713)

    at_target = module.compute_forward_command(observation(module, 0.718), config)
    invalid = module.ObservationSnapshot(
        sensor_healthy=True,
        valid=False,
        forward_gap_m=0.80,
        lateral_error_m=0.0,
        yaw_error_rad=0.0,
        confidence=0.0,
        source="orbbec_336l_depth",
    )

    assert at_target.state == "at_target"
    assert at_target.speed_mps == 0.0
    assert module.compute_forward_command(invalid, config).state == "invalid_observation"


def test_range_servo_uses_latched_docking_status_and_safety_command_path():
    source = MODULE_PATH.read_text(encoding="utf-8")

    assert 'parser.add_argument("--cmd-topic", default="/cmd_vel_docking")' in source
    assert 'if args.cmd_topic != "/cmd_vel_docking"' in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "args.docking_status_topic" in source
