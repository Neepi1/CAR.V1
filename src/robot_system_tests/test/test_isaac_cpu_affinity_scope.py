"""Exercise the real affinity resolver without pinning or starting any process."""

import os
from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[3]
OVERLAY = ROOT / "scripts/jetson/runtime_overlay"
SERVICES = (
    "localization",
    "occupancy_grid_localizer",
    "robot_global_localization",
    "amcl",
    "amcl_scan_admission",
    "imu_axis_remap",
    "laser_scan_to_flatscan",
    "pointcloud_to_laserscan",
    "scan_republisher",
    "robot_localization_bridge",
    "controller_server",
)


def resolved_cpus(tmp_path, inherited=None, override=""):
    """Source production files with an isolated runtime override fixture."""
    override_file = tmp_path / "cpu_affinity_runtime_override.env"
    override_file.write_text(override, encoding="utf-8")
    env = {
        key: value for key, value in os.environ.items()
        if not key.startswith("NJRH_") and key not in ("BASH_ENV", "ENV")
    }
    env.update(inherited or {})
    env.update({
        "NJRH_OVERLAY_ROOT": OVERLAY.as_posix(),
        "NJRH_CPU_AFFINITY_CONFIG": (OVERLAY / "config/cpu_affinity.env").as_posix(),
        "NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE": override_file.as_posix(),
        "NJRH_CPU_AFFINITY_ENABLED": "false",
        "TEST_AFFINITY_SOURCE": (OVERLAY / "scripts/cpu_affinity.sh").as_posix(),
    })
    bash = "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")
    assert bash and Path(bash).is_file(), "Bash is required for the real affinity resolver test"
    script = '''set -euo pipefail
source "$TEST_AFFINITY_SOURCE"
for service in "$@"; do
  printf '%s=%s\\n' "$service" "$(njrh_cpuset_for "$service")"
done
'''
    result = subprocess.run(
        [bash, "--noprofile", "--norc", "-c", script, "affinity-test", *SERVICES],
        env=env, capture_output=True, text=True, timeout=8,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return dict(line.split("=", 1) for line in result.stdout.splitlines())


def test_only_isaac_and_reload_wrapper_default_to_cpu3(tmp_path):
    values = resolved_cpus(tmp_path)
    assert values["localization"] == "6"
    assert values["occupancy_grid_localizer"] == "3"
    assert values["robot_global_localization"] == "3"


def test_other_localization_scan_and_control_defaults_are_unchanged(tmp_path):
    values = resolved_cpus(tmp_path)
    expected = {
        "localization": "6",
        "amcl": "6",
        "amcl_scan_admission": "6",
        "imu_axis_remap": "6",
        "laser_scan_to_flatscan": "6",
        "pointcloud_to_laserscan": "6",
        "scan_republisher": "6",
        "robot_localization_bridge": "7",
        "controller_server": "3",
    }
    assert {name: values[name] for name in expected} == expected


def test_runtime_override_replaces_inherited_cpu6_and_preserves_control_wide(tmp_path):
    inherited = {
        "NJRH_CPUSET_OCCUPANCY_GRID_LOCALIZER": "6",
        "NJRH_CPUSET_ROBOT_GLOBAL_LOCALIZATION": "6",
        "NJRH_NAV2_CONTROLLER_CPU_PROFILE": "current",
        "NJRH_CPUSET_CONTROLLER_SERVER": "3",
    }
    # Preserve the existing site controller profile alongside the two new keys.
    override = '''export NJRH_NAV2_CONTROLLER_CPU_PROFILE="control_wide"
export NJRH_CPUSET_NAV2_CONTROLLER_WIDE="3,5"
export NJRH_CPUSET_OCCUPANCY_GRID_LOCALIZER="3"
export NJRH_CPUSET_ROBOT_GLOBAL_LOCALIZATION="3"
'''
    values = resolved_cpus(tmp_path, inherited=inherited, override=override)
    assert values["occupancy_grid_localizer"] == "3"
    assert values["robot_global_localization"] == "3"
    assert values["controller_server"] == "3,5"
    for service in (
        "localization", "amcl", "amcl_scan_admission", "imu_axis_remap",
        "laser_scan_to_flatscan", "pointcloud_to_laserscan", "scan_republisher",
    ):
        assert values[service] == "6", service
    assert values["robot_localization_bridge"] == "7"
