#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

DURATION_SEC=60
LABEL="amcl_correction_window"
OUTPUT_ROOT="${NJRH_PROJECT_ROOT}/reports/amcl_navigation_correction_count"

usage() {
  cat <<'EOF'
Usage: record_amcl_correction_window.sh [--duration-sec N] [--label LABEL] [--output-root DIR]

Records AMCL gated correction counts and correction magnitudes from
/localization/bridge_status and robot_localization_bridge rosout logs. This
script is read-only: it does not send goals, publish velocity, or change params.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[amcl-correction-window] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[amcl-correction-window] --duration-sec must be an integer" >&2
    exit 2
    ;;
esac

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
report_dir="${OUTPUT_ROOT}/${timestamp}_${safe_label}_${DURATION_SEC}s"
mkdir -p "${report_dir}"

echo "[amcl-correction-window] report_dir=${report_dir}" >&2

python3 - "${DURATION_SEC}" "${report_dir}" <<'PY'
import csv
import json
import math
import re
import statistics
import sys
import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import String

duration_sec = float(sys.argv[1])
report_dir = Path(sys.argv[2])
report_dir.mkdir(parents=True, exist_ok=True)

accept_re = re.compile(
    r"accepted map->odom correction reason=(?P<reason>\S+) source=(?P<source>\S+) "
    r"translation=(?P<translation>[-+0-9.eE]+) yaw=(?P<yaw>[-+0-9.eE]+)"
)


def finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def stats(values):
    vals = sorted(v for v in (finite_float(v) for v in values) if v is not None)
    if not vals:
        return {"count": 0, "min": None, "mean": None, "p50": None, "p95": None, "max": None}

    def pct(percentile):
        if len(vals) == 1:
            return vals[0]
        k = (len(vals) - 1) * percentile / 100.0
        f = math.floor(k)
        c = math.ceil(k)
        if f == c:
            return vals[int(k)]
        return vals[f] * (c - k) + vals[c] * (k - f)

    return {
        "count": len(vals),
        "min": vals[0],
        "mean": statistics.mean(vals),
        "p50": pct(50),
        "p95": pct(95),
        "max": vals[-1],
    }


class Probe(Node):
    def __init__(self):
        super().__init__("record_amcl_correction_window")
        self.start_wall = time.time()
        self.first_bridge = None
        self.last_bridge = None
        self.last_amcl_accepted = None
        self.bridge_events = []
        self.rosout_events = []

        self.create_subscription(String, "/localization/bridge_status", self.on_bridge, 50)
        self.create_subscription(Log, "/rosout", self.on_rosout, 100)

        self.samples_file = (report_dir / "bridge_status_samples.csv").open("w", newline="", encoding="utf-8")
        self.samples_writer = csv.writer(self.samples_file)
        self.samples_writer.writerow([
            "rel_time_sec",
            "amcl_gate_mode",
            "amcl_accepted_count",
            "amcl_candidate_count",
            "amcl_rejected_count",
            "accepted_result_count",
            "amcl_last_state",
            "last_translation_m",
            "last_yaw_rad",
            "last_accepted_source",
            "active_correction_source",
            "amcl_robot_moving",
            "localization_degraded",
            "raw_json",
        ])

        self.events_file = (report_dir / "bridge_count_events.csv").open("w", newline="", encoding="utf-8")
        self.events_writer = csv.writer(self.events_file)
        self.events_writer.writerow([
            "rel_time_sec",
            "delta_amcl_accepted",
            "amcl_accepted_count",
            "amcl_candidate_count",
            "amcl_rejected_count",
            "amcl_last_state",
            "last_translation_m",
            "last_yaw_rad",
            "last_accepted_source",
            "active_correction_source",
            "amcl_robot_moving",
        ])

        self.rosout_file = (report_dir / "rosout_amcl_accept_events.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.rosout_writer = csv.writer(self.rosout_file)
        self.rosout_writer.writerow([
            "rel_time_sec",
            "node",
            "reason",
            "source",
            "translation_m",
            "yaw_rad",
            "message",
        ])

    def rel_time(self):
        return time.time() - self.start_wall

    def on_bridge(self, msg):
        rel = self.rel_time()
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if self.first_bridge is None:
            self.first_bridge = data
        self.last_bridge = data

        accepted = int(data.get("amcl_accepted_count") or 0)
        candidate = int(data.get("amcl_candidate_count") or 0)
        rejected = int(data.get("amcl_rejected_count") or 0)
        last_translation = data.get("last_accepted_correction_translation_m")
        last_yaw = data.get("last_accepted_correction_yaw_rad")

        self.samples_writer.writerow([
            f"{rel:.6f}",
            data.get("amcl_gate_mode"),
            accepted,
            candidate,
            rejected,
            data.get("accepted_result_count"),
            data.get("amcl_last_state"),
            last_translation,
            last_yaw,
            data.get("last_accepted_source"),
            data.get("active_correction_source"),
            data.get("amcl_robot_moving"),
            data.get("localization_degraded"),
            json.dumps(data, ensure_ascii=False, sort_keys=True),
        ])
        self.samples_file.flush()

        if self.last_amcl_accepted is None:
            self.last_amcl_accepted = accepted
            return
        delta = accepted - self.last_amcl_accepted
        if delta > 0:
            event = {
                "rel_time_sec": rel,
                "delta_amcl_accepted": delta,
                "amcl_accepted_count": accepted,
                "amcl_candidate_count": candidate,
                "amcl_rejected_count": rejected,
                "amcl_last_state": data.get("amcl_last_state"),
                "last_translation_m": last_translation,
                "last_yaw_rad": last_yaw,
                "last_accepted_source": data.get("last_accepted_source"),
                "active_correction_source": data.get("active_correction_source"),
                "amcl_robot_moving": data.get("amcl_robot_moving"),
            }
            self.bridge_events.append(event)
            self.events_writer.writerow([
                f"{rel:.6f}",
                delta,
                accepted,
                candidate,
                rejected,
                event["amcl_last_state"],
                last_translation,
                last_yaw,
                event["last_accepted_source"],
                event["active_correction_source"],
                event["amcl_robot_moving"],
            ])
            self.events_file.flush()
        self.last_amcl_accepted = accepted

    def on_rosout(self, msg):
        match = accept_re.search(msg.msg or "")
        if not match:
            return
        reason = match.group("reason")
        if not reason.startswith("AMCL_"):
            return
        event = {
            "rel_time_sec": self.rel_time(),
            "node": msg.name,
            "reason": reason,
            "source": match.group("source"),
            "translation_m": float(match.group("translation")),
            "yaw_rad": float(match.group("yaw")),
            "message": msg.msg,
        }
        self.rosout_events.append(event)
        self.rosout_writer.writerow([
            f"{event['rel_time_sec']:.6f}",
            event["node"],
            event["reason"],
            event["source"],
            event["translation_m"],
            event["yaw_rad"],
            event["message"],
        ])
        self.rosout_file.flush()

    def close(self):
        self.samples_file.close()
        self.events_file.close()
        self.rosout_file.close()


rclpy.init()
node = Probe()
deadline = time.time() + duration_sec
try:
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    first = node.first_bridge or {}
    last = node.last_bridge or first
    first_acc = int(first.get("amcl_accepted_count") or 0)
    last_acc = int(last.get("amcl_accepted_count") or first_acc)
    first_cand = int(first.get("amcl_candidate_count") or 0)
    last_cand = int(last.get("amcl_candidate_count") or first_cand)
    first_rej = int(first.get("amcl_rejected_count") or 0)
    last_rej = int(last.get("amcl_rejected_count") or first_rej)

    reason_counts = {}
    for event in node.rosout_events:
        reason_counts[event["reason"]] = reason_counts.get(event["reason"], 0) + 1

    summary = {
        "duration_sec": duration_sec,
        "amcl_gate_mode_start": first.get("amcl_gate_mode"),
        "amcl_gate_mode_end": last.get("amcl_gate_mode"),
        "baseline_amcl_accepted_count": first_acc,
        "end_amcl_accepted_count": last_acc,
        "delta_amcl_accepted_count": last_acc - first_acc,
        "baseline_amcl_candidate_count": first_cand,
        "end_amcl_candidate_count": last_cand,
        "delta_amcl_candidate_count": last_cand - first_cand,
        "baseline_amcl_rejected_count": first_rej,
        "end_amcl_rejected_count": last_rej,
        "delta_amcl_rejected_count": last_rej - first_rej,
        "bridge_count_event_rows": len(node.bridge_events),
        "rosout_amcl_accept_event_rows": len(node.rosout_events),
        "rosout_reason_counts": reason_counts,
        "rosout_translation_m_stats": stats([e["translation_m"] for e in node.rosout_events]),
        "rosout_abs_yaw_rad_stats": stats([abs(e["yaw_rad"]) for e in node.rosout_events]),
        "bridge_event_last_translation_m_stats": stats(
            [e["last_translation_m"] for e in node.bridge_events]
        ),
        "bridge_event_abs_yaw_rad_stats": stats(
            [abs(finite_float(e["last_yaw_rad"]) or 0.0) for e in node.bridge_events]
        ),
        "last_amcl_state": last.get("amcl_last_state"),
        "last_accepted_source": last.get("last_accepted_source"),
        "last_accepted_correction_translation_m": last.get("last_accepted_correction_translation_m"),
        "last_accepted_correction_yaw_rad": last.get("last_accepted_correction_yaw_rad"),
        "localization_degraded_end": last.get("localization_degraded"),
        "amcl_robot_moving_end": last.get("amcl_robot_moving"),
    }
    (report_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# AMCL Correction Window",
        "",
        f"- duration_sec: `{duration_sec:.0f}`",
        f"- amcl_gate_mode_start: `{summary['amcl_gate_mode_start']}`",
        f"- amcl_gate_mode_end: `{summary['amcl_gate_mode_end']}`",
        f"- baseline_amcl_accepted_count: `{first_acc}`",
        f"- end_amcl_accepted_count: `{last_acc}`",
        f"- delta_amcl_accepted_count: `{last_acc - first_acc}`",
        f"- delta_amcl_candidate_count: `{last_cand - first_cand}`",
        f"- delta_amcl_rejected_count: `{last_rej - first_rej}`",
        f"- rosout_amcl_accept_event_rows: `{len(node.rosout_events)}`",
        f"- rosout_reason_counts: `{reason_counts}`",
        f"- rosout_translation_m_stats: `{summary['rosout_translation_m_stats']}`",
        f"- rosout_abs_yaw_rad_stats: `{summary['rosout_abs_yaw_rad_stats']}`",
        f"- bridge_event_last_translation_m_stats: `{summary['bridge_event_last_translation_m_stats']}`",
        f"- bridge_event_abs_yaw_rad_stats: `{summary['bridge_event_abs_yaw_rad_stats']}`",
        f"- last_amcl_state: `{summary['last_amcl_state']}`",
        f"- last_accepted_source: `{summary['last_accepted_source']}`",
        f"- last_accepted_correction_translation_m: `{summary['last_accepted_correction_translation_m']}`",
        f"- last_accepted_correction_yaw_rad: `{summary['last_accepted_correction_yaw_rad']}`",
        f"- localization_degraded_end: `{summary['localization_degraded_end']}`",
        "",
        "Files:",
        "",
        "- `bridge_status_samples.csv`",
        "- `bridge_count_events.csv`",
        "- `rosout_amcl_accept_events.csv`",
        "- `summary.json`",
    ]
    (report_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    node.close()
    node.destroy_node()
    rclpy.shutdown()
    print(report_dir)
PY
