import csv
import importlib.util
import json
import math
import random
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = (
    ROOT
    / "scripts"
    / "jetson"
    / "runtime_overlay"
    / "scripts"
    / "orbbec_dock_surface_calibration.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("orbbec_dock_surface_calibration", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def make_plane(gap_m, center_y_m, span_m, correction_yaw_rad, count, seed):
    rng = random.Random(seed)
    slope = -math.tan(correction_yaw_rad)
    points = []
    for index in range(count):
        fraction = index / max(1, count - 1)
        y = center_y_m - 0.5 * span_m + fraction * span_m
        y += rng.gauss(0.0, 0.0007)
        x = 0.398 + gap_m + slope * y + rng.gauss(0.0, 0.002)
        points.append((x, y))
    return points


def test_detects_telescoping_head_and_fixed_face_as_two_surfaces():
    module = load_module()
    points = make_plane(0.800, -0.050, 0.075, 0.01, 180, 1)
    points += make_plane(0.935, -0.050, 0.185, 0.01, 620, 2)

    estimate = module.detect_dock_surfaces(points, module.SurfaceDetectionConfig())

    assert estimate.valid, estimate.reason
    assert abs(estimate.head.forward_gap_m - 0.800) < 0.008
    assert abs(estimate.face.forward_gap_m - 0.935) < 0.008
    assert abs(estimate.protrusion_m - 0.135) < 0.010
    assert abs(estimate.head.center_y_m + 0.050) < 0.008
    assert abs(estimate.face.center_y_m + 0.050) < 0.008


def test_preserves_lateral_and_yaw_signs_for_both_surfaces():
    module = load_module()
    points = make_plane(0.600, 0.035, 0.080, 0.055, 180, 3)
    points += make_plane(0.730, 0.038, 0.190, 0.052, 620, 4)

    estimate = module.detect_dock_surfaces(points, module.SurfaceDetectionConfig())

    assert estimate.valid, estimate.reason
    assert estimate.head.center_y_m > 0.0
    assert estimate.face.center_y_m > 0.0
    assert estimate.head.yaw_error_rad > 0.0
    assert estimate.face.yaw_error_rad > 0.0
    assert abs(estimate.head.yaw_error_rad - 0.055) < 0.012
    assert abs(estimate.face.yaw_error_rad - 0.052) < 0.012


def test_accepts_angled_telescoping_head_when_fixed_face_is_aligned():
    module = load_module()
    points = make_plane(0.326, -0.047, 0.020, -0.19, 180, 10)
    points += make_plane(0.394, -0.049, 0.190, 0.002, 900, 11)

    estimate = module.detect_dock_surfaces(points, module.SurfaceDetectionConfig())

    assert estimate.valid, estimate.reason
    assert abs(estimate.head.yaw_error_rad + 0.19) < 0.015
    assert abs(estimate.face.yaw_error_rad - 0.002) < 0.012


def test_rejects_single_plane_and_unrelated_depth_clusters():
    module = load_module()
    config = module.SurfaceDetectionConfig()

    single_face = make_plane(0.935, 0.0, 0.185, 0.0, 620, 5)
    single_estimate = module.detect_dock_surfaces(single_face, config)
    assert not single_estimate.valid
    assert single_estimate.reason == "two_surface_pair_not_found"

    too_far_head = make_plane(0.55, 0.0, 0.075, 0.0, 180, 6)
    too_far_face = make_plane(0.95, 0.0, 0.185, 0.0, 620, 7)
    separated_estimate = module.detect_dock_surfaces(too_far_head + too_far_face, config)
    assert not separated_estimate.valid
    assert separated_estimate.reason == "two_surface_pair_not_found"


def test_rejects_pair_with_inconsistent_centers_or_yaw():
    module = load_module()
    config = module.SurfaceDetectionConfig()
    head = make_plane(0.80, -0.16, 0.075, -0.12, 180, 8)
    face = make_plane(0.935, 0.05, 0.185, 0.08, 620, 9)

    estimate = module.detect_dock_surfaces(head + face, config)

    assert not estimate.valid
    assert estimate.reason == "two_surface_pair_not_found"


def test_report_keeps_candidate_planes_when_pair_is_rejected(tmp_path):
    module = load_module()
    points = make_plane(0.245, -0.042, 0.110, -0.003, 180, 12)
    points += make_plane(0.448, -0.046, 0.770, 0.015, 620, 13)
    estimate = module.detect_dock_surfaces(points, module.SurfaceDetectionConfig())
    assert not estimate.valid
    assert len(estimate.planes) == 2

    args = SimpleNamespace(
        charge_contact_x_m=0.398,
        expected_head_gap_m=0.035,
        expected_head_gap_tolerance_m=0.15,
        min_valid_samples=15,
        min_valid_ratio=0.70,
        min_protrusion_m=0.06,
        max_protrusion_m=0.22,
        max_protrusion_mad_m=0.01,
        max_pair_center_delta_m=0.08,
        max_pair_yaw_delta_rad=0.25,
    )
    summary, accepted = module._write_report(tmp_path / "report", args, [(1.0, estimate)], points)

    assert not accepted
    assert summary.is_file()
    with (summary.parent / "candidate_planes.csv").open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 2
    assert float(rows[0]["forward_gap_m"]) < float(rows[1]["forward_gap_m"])
    metrics = json.loads((summary.parent / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["nearest_candidate"]["samples"] == 1
    assert metrics["second_candidate"]["samples"] == 1
