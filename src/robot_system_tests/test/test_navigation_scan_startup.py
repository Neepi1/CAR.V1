"""Cold-start /scan admission against the real shell branch and ownership helper.

Only the ROS readiness-probe interface is replaced. No ROS process, service,
robot, production temporary file or network access is used by these tests.
"""

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = Path(os.environ.get(
    "NJRH_NAV_SCAN_TEST_SCRIPT_DIR", ROOT / "scripts/jetson/runtime_overlay/scripts"))


def cold_start_scan_branch():
    source = (SCRIPTS / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    marker = 'log_startup_stage "navigation_scan_owner_ready"'
    ready = source.rindex(marker)
    start = source.rfind(
        'if [[ "${navigation_start_source}" == "systemd_autostart" ]]; then', 0, ready)
    assert start >= 0
    end = source.index("\nfi", ready) + len("\nfi")
    logger = re.search(r"(?ms)^log_startup_stage\(\) \{\n.*?^\}\n", source)
    assert logger is not None
    return logger.group(0), source[start:end]


def run_scan_startup(tmp_path, scenario, start_source="systemd_autostart"):
    logger, branch = cold_start_scan_branch()
    helper = SCRIPTS / "scan_ownership_helpers.sh"
    script = f"""set -euo pipefail
export TMPDIR="$PWD"
navigation_start_source={start_source}
startup_epoch_sec=$(date +%s)
# Exercise the real stage logger without unrelated map-context I/O.
runtime_ready=1
cleanup_started=0
export NJRH_SCAN_OWNERSHIP_TIMEOUT_SEC=12
export NJRH_RESIDENT_SCAN_OWNER_NODE=pointcloud_accel_axis_node
source "{helper.as_posix()}"
SCENARIO={scenario}
runtime_readiness_probe() {{
  printf '%s\\n' "$*" >> probe_calls
  case "$1" in
    exact-publisher-owner)
      [[ "$2" == /scan && "$3" == pointcloud_accel_axis_node && "$4" == 1 ]] || return 91
      # Each temporary reader starts discovery from scratch. One second can
      # miss a healthy owner; a single complete 12-second observation finds it.
      [[ "$5" != 1 ]] || return 1
      [[ "$5" == 12 ]] || return 92
      case "$SCENARIO" in
        cold_late_owner) return 0 ;;
        missing) echo 'actual_count=0 actual_owners=[]' >&2; return 1 ;;
        duplicate) echo 'actual_count=2 actual_owners=[pointcloud_accel_axis_node,pointcloud_accel_axis_node]' >&2; return 1 ;;
        foreign_owner) echo 'actual_count=1 actual_owners=[pointcloud_to_laserscan]' >&2; return 1 ;;
        *) return 96 ;;
      esac
      ;;
    publisher-count)
      [[ "$2" == /scan && "$3" == 0 && "$4" == 1 ]] || return 93
      # The old path treats a new reader's undiscovered graph as count zero.
      return 0
      ;;
    service)
      # Reject at the probe boundary, before any ros2 service call is possible.
      echo 'unexpected scan mutation service path' >&2
      return 94
      ;;
    *) return 95 ;;
  esac
}}
{logger}
{branch}
echo navigation-scan-admitted
"""
    path = tmp_path / "scan_startup.sh"
    path.write_text(script, encoding="utf-8")
    bash = "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")
    run = subprocess.run([bash, str(path)], cwd=tmp_path,
                         capture_output=True, text=True, timeout=5)
    calls = ((tmp_path / "probe_calls").read_text().splitlines()
             if (tmp_path / "probe_calls").exists() else [])
    return run, calls


def test_cold_discovery_delay_does_not_reenable_an_existing_scan_publisher(tmp_path):
    run, calls = run_scan_startup(tmp_path, "cold_late_owner")
    assert run.returncode == 0, run.stdout + run.stderr + repr(calls)
    assert "navigation-scan-admitted" in run.stdout
    assert "STARTUP_STAGE stage=navigation_scan_owner_ready" in run.stderr
    assert calls == ["exact-publisher-owner /scan pointcloud_accel_axis_node 1 12"]


@pytest.mark.parametrize("scenario,observed", [
    ("missing", "actual_count=0 actual_owners=[]"),
    ("duplicate", "actual_count=2 actual_owners=[pointcloud_accel_axis_node,pointcloud_accel_axis_node]"),
    ("foreign_owner", "actual_count=1 actual_owners=[pointcloud_to_laserscan]"),
])
def test_unproven_cold_scan_ownership_cannot_mutate_publisher_or_report_ready(
        tmp_path, scenario, observed):
    run, calls = run_scan_startup(tmp_path, scenario)
    assert run.returncode == 1, run.stdout + run.stderr + repr(calls)
    assert observed in run.stderr
    assert "navigation-scan-admitted" not in run.stdout
    assert "STARTUP_STAGE stage=navigation_scan_owner_ready" not in run.stderr
    assert calls == ["exact-publisher-owner /scan pointcloud_accel_axis_node 1 12"]
