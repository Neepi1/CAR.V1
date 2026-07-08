#!/usr/bin/env python3
"""Fit base_link -> lidar_level_link XY/yaw corrections from static relocalization samples."""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import statistics
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


def norm_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def angle_diff(a: float, b: float) -> float:
    return norm_angle(a - b)


def parse_float_list(text: str) -> List[float]:
    return [float(part.strip()) for part in text.split(",") if part.strip()]


def parse_config(path: str) -> Dict[str, float]:
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    def scalar(name: str) -> Optional[float]:
        match = re.search(rf"(?m)^\s*{re.escape(name)}\s*:\s*([-+0-9.eE]+)\s*$", text)
        return float(match.group(1)) if match else None

    flat_x = scalar("lidar_x")
    flat_y = scalar("lidar_y")
    flat_z = scalar("lidar_z")
    flat_yaw = scalar("lidar_yaw")
    if flat_x is not None and flat_y is not None and flat_z is not None and flat_yaw is not None:
        return {"x": flat_x, "y": flat_y, "z": flat_z, "yaw": flat_yaw}

    xyz_match = re.search(r"(?m)^\s*lidar_xyz\s*:\s*\[([^\]]+)\]\s*$", text)
    rpy_match = re.search(r"(?m)^\s*lidar_rpy\s*:\s*\[([^\]]+)\]\s*$", text)
    if xyz_match and rpy_match:
        xyz = parse_float_list(xyz_match.group(1))
        rpy = parse_float_list(rpy_match.group(1))
        if len(xyz) >= 3 and len(rpy) >= 3:
            return {"x": xyz[0], "y": xyz[1], "z": xyz[2], "yaw": rpy[2]}

    raise ValueError(f"could not read lidar x/y/z/yaw from {path}")


def heading_hint_from_name(path: str) -> Optional[float]:
    name = os.path.basename(os.path.normpath(path))
    match = re.search(r"heading_([A-Za-z0-9+.-]+)", name)
    if not match:
        return None
    token = match.group(1)
    sign = 1.0
    if token.startswith("m"):
        sign = -1.0
        token = token[1:]
    elif token.startswith("p") and len(token) > 1 and token[1].isdigit():
        token = token[1:]
    token = token.replace("p", ".")
    try:
        return sign * float(token)
    except ValueError:
        return None


def sample_dirs_from_args(root: Optional[str], samples: Sequence[str]) -> List[str]:
    out: List[str] = []
    if root:
        root = os.path.abspath(root)
        if os.path.isfile(os.path.join(root, "after_snapshot.json")):
            out.append(root)
        out.extend(sorted(glob.glob(os.path.join(root, "sample_*"))))
    out.extend(os.path.abspath(sample) for sample in samples)

    deduped: List[str] = []
    seen = set()
    for item in out:
        item = os.path.abspath(item)
        if item in seen:
            continue
        seen.add(item)
        if os.path.isfile(os.path.join(item, "after_snapshot.json")):
            deduped.append(item)
    return deduped


def load_sample(path: str) -> Dict[str, Any]:
    with open(os.path.join(path, "after_snapshot.json"), "r", encoding="utf-8") as f:
        after = json.load(f)
    pose = ((after.get("tf") or {}).get("map_base_link") or {})
    if not all(key in pose for key in ("x", "y", "yaw_rad")):
        raise ValueError(f"{path}: after_snapshot.json has no tf.map_base_link pose")

    metrics: Dict[str, Any] = {}
    metrics_path = os.path.join(path, "correction_metrics.json")
    if os.path.isfile(metrics_path):
        with open(metrics_path, "r", encoding="utf-8") as f:
            metrics = json.load(f)

    return {
        "dir": path,
        "name": os.path.basename(os.path.normpath(path)),
        "heading_hint_deg": heading_hint_from_name(path),
        "x": float(pose["x"]),
        "y": float(pose["y"]),
        "yaw_rad": float(pose["yaw_rad"]),
        "yaw_deg": math.degrees(float(pose["yaw_rad"])),
        "map_base_correction_m": ((metrics.get("map_base_link_delta") or {}).get("translation_m")),
        "map_base_correction_dyaw_deg": ((metrics.get("map_base_link_delta") or {}).get("dyaw_deg")),
        "trigger_accepted": ((metrics.get("trigger") or {}).get("accepted")),
        "bridge_source": ((metrics.get("bridge") or {}).get("last_correction_source")),
        "bridge_accept_reason": ((metrics.get("bridge") or {}).get("last_accept_reason")),
    }


def solve_linear_system(matrix: List[List[float]], vector: List[float]) -> List[float]:
    n = len(vector)
    aug = [row[:] + [rhs] for row, rhs in zip(matrix, vector)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(aug[row][col]))
        if abs(aug[pivot][col]) < 1e-12:
            raise ValueError("singular fit matrix; use at least three distinct headings")
        if pivot != col:
            aug[col], aug[pivot] = aug[pivot], aug[col]
        pivot_value = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= pivot_value
        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            if factor == 0.0:
                continue
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]
    return [aug[i][n] for i in range(n)]


def least_squares(rows: Iterable[Tuple[Sequence[float], float]]) -> List[float]:
    ata = [[0.0 for _ in range(4)] for _ in range(4)]
    atb = [0.0 for _ in range(4)]
    for coeffs, value in rows:
        for i in range(4):
            atb[i] += coeffs[i] * value
            for j in range(4):
                ata[i][j] += coeffs[i] * coeffs[j]
    return solve_linear_system(ata, atb)


def fit_xy(samples: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    rows: List[Tuple[Sequence[float], float]] = []
    for sample in samples:
        yaw = sample["yaw_rad"]
        c = math.cos(yaw)
        s = math.sin(yaw)
        rows.append(([1.0, 0.0, c, -s], sample["x"]))
        rows.append(([0.0, 1.0, s, c], sample["y"]))

    cx, cy, dx, dy = least_squares(rows)
    residuals = []
    for sample in samples:
        yaw = sample["yaw_rad"]
        c = math.cos(yaw)
        s = math.sin(yaw)
        pred_x = cx + c * dx - s * dy
        pred_y = cy + s * dx + c * dy
        rx = sample["x"] - pred_x
        ry = sample["y"] - pred_y
        residuals.append(
            {
                "name": sample["name"],
                "x_m": rx,
                "y_m": ry,
                "norm_m": math.hypot(rx, ry),
            }
        )

    norms = [item["norm_m"] for item in residuals]
    rms = math.sqrt(sum(value * value for value in norms) / len(norms))
    return {
        "fit_center_map_m": {"x": cx, "y": cy},
        "configured_to_true_lidar_xy_error_base_m": {"x": dx, "y": dy},
        "residuals": residuals,
        "residual_rms_m": rms,
        "residual_max_m": max(norms),
    }


def expected_yaws(
    samples: Sequence[Dict[str, Any]],
    expected_yaws_deg: Optional[str],
    expected_first_heading_deg: Optional[float],
) -> Optional[List[float]]:
    if expected_yaws_deg:
        values = parse_float_list(expected_yaws_deg)
        if len(values) != len(samples):
            raise ValueError("--expected-yaws-deg count must match sample count")
        return [math.radians(value) for value in values]

    if expected_first_heading_deg is None:
        return None

    hints = [sample.get("heading_hint_deg") for sample in samples]
    if any(value is None for value in hints):
        raise ValueError("--expected-first-heading-deg requires sample directory names with heading_<deg>")
    return [math.radians(expected_first_heading_deg + float(hint)) for hint in hints]


def yaw_report(
    samples: Sequence[Dict[str, Any]],
    expected: Optional[Sequence[float]],
    current_yaw: float,
) -> Dict[str, Any]:
    step_rows = []
    if samples:
        first_yaw = samples[0]["yaw_rad"]
        first_hint = samples[0].get("heading_hint_deg")
        for sample in samples:
            observed_delta = math.degrees(angle_diff(sample["yaw_rad"], first_yaw))
            hint = sample.get("heading_hint_deg")
            hinted_delta = None if hint is None or first_hint is None else float(hint) - float(first_hint)
            step_rows.append(
                {
                    "name": sample["name"],
                    "observed_delta_from_first_deg": observed_delta,
                    "heading_hint_delta_from_first_deg": hinted_delta,
                    "delta_error_deg": None
                    if hinted_delta is None
                    else math.degrees(angle_diff(math.radians(observed_delta), math.radians(hinted_delta))),
                }
            )

    if expected is None:
        return {
            "absolute_yaw_calibrated": False,
            "reason": "no known map heading was provided; static four-heading samples identify XY offset but not absolute yaw",
            "step_consistency": step_rows,
        }

    errors = [angle_diff(sample["yaw_rad"], exp) for sample, exp in zip(samples, expected)]
    sin_mean = statistics.fmean(math.sin(value) for value in errors)
    cos_mean = statistics.fmean(math.cos(value) for value in errors)
    bias = math.atan2(sin_mean, cos_mean)
    residuals = [angle_diff(value, bias) for value in errors]
    abs_residuals = [abs(math.degrees(value)) for value in residuals]
    return {
        "absolute_yaw_calibrated": True,
        "observed_minus_expected_bias_rad": bias,
        "observed_minus_expected_bias_deg": math.degrees(bias),
        "recommended_lidar_yaw_rad": norm_angle(current_yaw + bias),
        "recommended_lidar_yaw_deg": math.degrees(norm_angle(current_yaw + bias)),
        "yaw_residual_rms_deg": math.sqrt(sum(value * value for value in abs_residuals) / len(abs_residuals)),
        "yaw_residual_max_deg": max(abs_residuals),
        "step_consistency": step_rows,
    }


def write_summary(path: str, report: Dict[str, Any]) -> None:
    current = report["current_lidar_extrinsic"]
    rec = report["recommended_lidar_extrinsic"]
    xy = report["xy_fit"]
    yaw = report["yaw_fit"]
    with open(path, "w", encoding="utf-8") as f:
        f.write("# Lidar Level Extrinsic Calibration Fit\n\n")
        f.write("This report fits `base_link -> lidar_level_link` planar extrinsics from static Isaac relocalization samples.\n\n")
        f.write("## Recommendation\n\n")
        f.write(f"- current_lidar_x_m: `{current['x']:.6f}`\n")
        f.write(f"- current_lidar_y_m: `{current['y']:.6f}`\n")
        f.write(f"- current_lidar_yaw_rad: `{current['yaw']:.9f}`\n")
        f.write(f"- suggested_lidar_x_m: `{rec['x']:.6f}`\n")
        f.write(f"- suggested_lidar_y_m: `{rec['y']:.6f}`\n")
        f.write(f"- suggested_lidar_yaw_rad: `{rec['yaw']:.9f}`\n")
        f.write(f"- xy_error_base_m: `x={xy['configured_to_true_lidar_xy_error_base_m']['x']:.6f}, y={xy['configured_to_true_lidar_xy_error_base_m']['y']:.6f}`\n")
        f.write(f"- xy_fit_residual_rms_m: `{xy['residual_rms_m']:.6f}`\n")
        f.write(f"- xy_fit_residual_max_m: `{xy['residual_max_m']:.6f}`\n")
        if yaw.get("absolute_yaw_calibrated"):
            f.write(f"- yaw_bias_deg: `{yaw['observed_minus_expected_bias_deg']:.3f}`\n")
            f.write(f"- yaw_residual_rms_deg: `{yaw['yaw_residual_rms_deg']:.3f}`\n")
            f.write(f"- yaw_residual_max_deg: `{yaw['yaw_residual_max_deg']:.3f}`\n")
        else:
            f.write(f"- yaw_calibration: `{yaw['reason']}`\n")
        f.write("\n## Samples\n\n")
        f.write("| sample | heading_hint_deg | map_x | map_y | yaw_deg | relocalize_translation_m | relocalize_dyaw_deg |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|\n")
        for sample in report["samples"]:
            correction = sample.get("map_base_correction_m")
            correction_yaw = sample.get("map_base_correction_dyaw_deg")
            heading_text = "" if sample.get("heading_hint_deg") is None else f"{sample['heading_hint_deg']:.3f}"
            correction_text = "" if correction is None else f"{float(correction):.6f}"
            correction_yaw_text = "" if correction_yaw is None else f"{float(correction_yaw):.3f}"
            f.write(
                f"| `{sample['name']}` | "
                f"{heading_text} | "
                f"{sample['x']:.6f} | {sample['y']:.6f} | {sample['yaw_deg']:.3f} | "
                f"{correction_text} | "
                f"{correction_yaw_text} |\n"
            )
        f.write("\n## XY Residuals\n\n")
        f.write("| sample | residual_x_m | residual_y_m | residual_norm_m |\n")
        f.write("|---|---:|---:|---:|\n")
        for residual in xy["residuals"]:
            f.write(
                f"| `{residual['name']}` | {residual['x_m']:.6f} | "
                f"{residual['y_m']:.6f} | {residual['norm_m']:.6f} |\n"
            )
        f.write("\n## Yaw Step Check\n\n")
        f.write("| sample | observed_delta_from_first_deg | heading_hint_delta_from_first_deg | delta_error_deg |\n")
        f.write("|---|---:|---:|---:|\n")
        for row in yaw.get("step_consistency", []):
            hinted = row.get("heading_hint_delta_from_first_deg")
            error = row.get("delta_error_deg")
            hinted_text = "" if hinted is None else f"{hinted:.3f}"
            error_text = "" if error is None else f"{error:.3f}"
            f.write(
                f"| `{row['name']}` | {row['observed_delta_from_first_deg']:.3f} | "
                f"{hinted_text} | "
                f"{error_text} |\n"
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", help="Calibration capture root containing sample_* directories.")
    parser.add_argument("--sample", action="append", default=[], help="One sample directory. May be repeated.")
    parser.add_argument("--current-config", required=True, help="sensors.yaml containing current lidar extrinsic.")
    parser.add_argument("--expected-yaws-deg", help="Comma-separated absolute expected map yaws, one per sample.")
    parser.add_argument(
        "--expected-first-heading-deg",
        type=float,
        help="Absolute map yaw for heading_0; remaining samples use heading_<deg> directory hints.",
    )
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--summary-md", required=True)
    args = parser.parse_args()

    dirs = sample_dirs_from_args(args.root, args.sample)
    if len(dirs) < 3:
        raise SystemExit("need at least three static heading samples")

    samples = [load_sample(path) for path in dirs]
    current = parse_config(args.current_config)
    xy = fit_xy(samples)
    expected = expected_yaws(samples, args.expected_yaws_deg, args.expected_first_heading_deg)
    yaw = yaw_report(samples, expected, current["yaw"])

    delta = xy["configured_to_true_lidar_xy_error_base_m"]
    recommended = {
        "x": current["x"] + delta["x"],
        "y": current["y"] + delta["y"],
        "z": current["z"],
        "yaw": yaw["recommended_lidar_yaw_rad"] if yaw.get("absolute_yaw_calibrated") else current["yaw"],
    }

    report = {
        "current_lidar_extrinsic": current,
        "recommended_lidar_extrinsic": recommended,
        "samples": samples,
        "xy_fit": xy,
        "yaw_fit": yaw,
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output_json)), exist_ok=True)
    with open(args.output_json, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, sort_keys=True)
        f.write("\n")
    write_summary(args.summary_md, report)
    print(f"[lidar-extrinsic-fit] summary: {args.summary_md}")
    print(f"[lidar-extrinsic-fit] json: {args.output_json}")
    print(
        "[lidar-extrinsic-fit] suggested "
        f"lidar_x={recommended['x']:.6f} lidar_y={recommended['y']:.6f} "
        f"lidar_yaw={recommended['yaw']:.9f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
