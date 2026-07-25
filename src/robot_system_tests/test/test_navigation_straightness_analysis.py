import csv
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
ANALYZER_PATH = (
    ROOT
    / "scripts"
    / "jetson"
    / "runtime_overlay"
    / "scripts"
    / "analyze_navigation_straightness.py"
)


def load_analyzer():
    spec = importlib.util.spec_from_file_location("analyze_navigation_straightness", ANALYZER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_capture(report_dir, lateral_values, cmd_wz_values):
    report_dir.mkdir(parents=True, exist_ok=True)
    fields = [
        "local_x",
        "local_y",
        "cmd_vel_nav_raw_x",
        "cmd_vel_nav_raw_z",
        "motion_linear_velocity",
        "motion_angular_velocity",
        "motion_steering_angle",
    ]
    with (report_dir / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for index, (lateral, cmd_wz) in enumerate(zip(lateral_values, cmd_wz_values)):
            writer.writerow(
                {
                    "local_x": index * 0.25,
                    "local_y": lateral,
                    "cmd_vel_nav_raw_x": 0.5,
                    "cmd_vel_nav_raw_z": cmd_wz,
                    "motion_linear_velocity": 0.5,
                    "motion_angular_velocity": cmd_wz,
                    "motion_steering_angle": cmd_wz,
                }
            )
    (report_dir / "path_metrics.json").write_text(
        json.dumps(
            {
                "/plan_smoothed": {
                    "count": 1,
                    "max_cross_track_m": 0.01,
                    "max_length_ratio": 1.001,
                }
            }
        ),
        encoding="utf-8",
    )


def test_straight_capture_passes(tmp_path):
    analyzer = load_analyzer()
    lateral = [0.002 * ((index % 3) - 1) for index in range(48)]
    cmd_wz = [0.005] * 48
    write_capture(tmp_path, lateral, cmd_wz)

    result = analyzer.analyze(tmp_path)

    assert result["verdict"] == "PASS"
    assert result["trajectory"]["cross_track_peak_to_peak_m"] < 0.01


def test_wavy_capture_is_red(tmp_path):
    analyzer = load_analyzer()
    lateral = [0.09 if (index // 4) % 2 == 0 else -0.09 for index in range(48)]
    cmd_wz = [0.08 if (index // 4) % 2 == 0 else -0.08 for index in range(48)]
    write_capture(tmp_path, lateral, cmd_wz)

    result = analyzer.analyze(tmp_path)

    assert result["verdict"] == "RED"
    assert result["trajectory"]["cmd_wz_reversals"] > 3
    assert result["trajectory"]["cross_track_peak_to_peak_m"] > 0.12
