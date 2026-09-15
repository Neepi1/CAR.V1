#!/usr/bin/env python3
"""Offline identification from existing Ranger CAN captures; never imports ROS."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


def load(path, kind):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    segments = {}
    for row in rows:
        if row["segment_kind"] in kind:
            segments.setdefault(row["segment"], []).append(row)
    return segments


def series(rows, steering=False):
    t = np.array([float(r["segment_elapsed_sec"]) for r in rows])
    if steering:
        # The recorded profile contains a constant, independently recorded target.
        vx, wz = float(rows[-1]["req_vx"]), float(rows[-1]["req_wz"])
        central = np.arcsin(min(abs(wz) * 0.494 / (2 * abs(vx)), 1.0))
        target = np.sign(wz * vx) * np.arctan2(
            0.494 * np.sin(central), 0.494 * np.cos(central) - 0.364 * np.sin(central))
        u = np.array([target if abs(float(r["cmd_wz"])) > 0.01 else 0.0 for r in rows])
        y = np.array([float(r["can221_steering"]) for r in rows])
    else:
        u = np.array([float(r["cmd_vx"]) for r in rows])
        y = np.array([float(r["can221_linear"]) for r in rows])
    return t, u, y


def simulate(data, parameters, steering=False):
    t, u, y = data
    delay, tau = parameters[:2]
    accel, decel = (100.0, 100.0) if steering else parameters[2:]
    result = np.empty_like(y)
    result[0] = y[0]
    for i in range(1, len(t)):
        j = np.searchsorted(t, t[i - 1] - delay, side="right") - 1
        target = u[j] if j >= 0 else y[0]
        dt = t[i] - t[i - 1]
        delta = (target - result[i - 1]) * (-np.expm1(-dt / tau))
        result[i] = result[i - 1] + np.clip(delta, -decel * dt, accel * dt)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    names = [
        "20260630T011801Z_standard_low_mid_v2_standard",
        "20260630T011908Z_standard_low_mid_v2_standard",
        "20260630T013101Z_linear_1p2_nav_speed_v1_linear",
        "20260630T013457Z_linear_1p2_nav_speed_v1_linear",
    ]
    output = {"source_sha256": {}, "models": {}}
    for steering in (False, True):
        captures = []
        for index, name in enumerate(names):
            path = args.report_root / name / "samples.csv"
            output["source_sha256"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
            kind = {"steering_step"} if steering else {"linear_accel", "linear_decel"}
            for segment, rows in load(path, kind).items():
                captures.append((index % 2 == 0, name, segment, series(rows, steering)))
        initial = [0.1, 0.05] if steering else [0.08, 0.1, 0.65, 1.2]
        bounds = ([0, 0.005], [0.4, 0.4]) if steering else (
            [0, 0.005, 0.1, 0.1], [0.4, 0.6, 3, 4])

        def residual(parameters):
            return np.concatenate([
                (simulate(data, parameters, steering) - data[2]) / np.sqrt(len(data[0]))
                for training, _, _, data in captures if training])

        # Delay indexing is piecewise constant: explicitly scan delays, then fit
        # continuous coefficients. Do not let a zero finite-difference gradient
        # silently leave delay at the initial guess.
        best = None
        for delay in np.arange(0, 0.241, 0.01):
            result = least_squares(lambda p: residual([delay, *p]), initial[1:],
                                   bounds=(bounds[0][1:], bounds[1][1:]), max_nfev=50)
            score = np.mean(result.fun ** 2)
            if best is None or score < best[0]:
                best = score, [float(delay), *map(float, result.x)]
        parameters = best[1]
        records = []
        for training, name, segment, data in captures:
            predicted = simulate(data, parameters, steering)
            t, u, y = data
            records.append({
                "report": name, "segment": segment, "training": training,
                "rmse": float(np.sqrt(np.mean((predicted - y) ** 2))),
                "instantaneous_rmse": float(np.sqrt(np.mean((u - y) ** 2))),
                "measured_integral": float(np.trapz(y, t)),
                "predicted_integral": float(np.trapz(predicted, t)),
            })
        output["models"]["steering" if steering else "linear"] = {
            "parameters_delay_tau_accel_decel": parameters, "segments": records}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
