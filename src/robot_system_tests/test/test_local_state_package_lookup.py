"""Exercise the local-state package lookup without loading ROS or starting EKF."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_state.sh"


@pytest.fixture(scope="module")
def bash():
    candidates = [Path("C:/Program Files/Git/bin/bash.exe")] if sys.platform == "win32" else []
    if discovered := shutil.which("bash"):
        candidates.append(Path(discovered))
    executable = next((candidate for candidate in candidates if candidate.exists()), None)
    if executable is None:
        pytest.skip("Bash is required for the production shell block")
    return str(executable)


FAKE_AMENT = '''
import os
from pathlib import Path

class PackageNotFoundError(LookupError):
    pass

def get_package_prefix(name):
    with open(os.environ["NJRH_TEST_INDEX_CALLS"], "a", encoding="utf-8") as log:
        log.write(name + "\\n")
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix and (Path(prefix) / "share/ament_index/resource_index/packages" / name).is_file():
            return Path(prefix).as_posix()
    raise PackageNotFoundError(name)
'''


def package_prefix(tmp_path, name, *, package_present=True, executable_present=True):
    prefix = tmp_path / name
    prefix.mkdir()
    if package_present:
        marker = prefix / "share/ament_index/resource_index/packages/robot_localization"
        marker.parent.mkdir(parents=True)
        marker.write_text("", encoding="utf-8")
    if executable_present:
        executable = prefix / "lib/robot_localization/ekf_node"
        executable.parent.mkdir(parents=True)
        executable.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
        executable.chmod(0o755)
    return prefix


def lookup(bash, tmp_path, prefixes):
    source = RUNNER.read_text(encoding="utf-8")
    diagnostic = source.index('echo "[runtime-overlay] ROS package missing: robot_localization"')
    start = source.rfind("\nif ", 0, diagnostic) + 1
    end = source.index('\n[[ -x "${NODE_BIN}" ]]', diagnostic)
    block = source[start:end]
    assert block.startswith("if ")
    assert 'EKF_NODE_BIN="${ROBOT_LOCALIZATION_PREFIX}/lib/robot_localization/ekf_node"' in block

    modules = tmp_path / "python_modules"
    ament = modules / "ament_index_python"
    ament.mkdir(parents=True)
    (ament / "__init__.py").write_text("", encoding="utf-8")
    (ament / "packages.py").write_text(FAKE_AMENT, encoding="utf-8")
    index_calls = tmp_path / "index_calls.txt"
    cli_calls = tmp_path / "cli_calls.txt"
    environment = os.environ.copy()
    environment.update(
        PYTHONPATH=str(modules),
        AMENT_PREFIX_PATH=os.pathsep.join(str(prefix) for prefix in prefixes),
        NJRH_TEST_INDEX_CALLS=str(index_calls),
        NJRH_TEST_PYTHON=Path(sys.executable).as_posix(),
    )
    # ros2 is a fake command boundary: it delegates only to our fake index,
    # never to a ROS executable. The replacement runs the real Python snippet
    # from the production block against the same fake ament API.
    prelude = r'''
set -euo pipefail
python3() { "$NJRH_TEST_PYTHON" "$@"; }
ros2() {
  printf '%s\n' "$*" >> "$cli_calls"
  [[ "$*" == 'pkg prefix robot_localization' ]] || return 90
  "$NJRH_TEST_PYTHON" -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("robot_localization"))'
}
'''
    harness = (
        f"cli_calls={shlex.quote(cli_calls.as_posix())}\n"
        + prelude
        + block
        + '\nprintf "SELECTED_PREFIX=%s\\nEKF_BINARY=%s\\n" "$ROBOT_LOCALIZATION_PREFIX" "$EKF_NODE_BIN"\n'
    )
    result = subprocess.run(
        [bash, "--noprofile", "--norc", "-s"],
        input=harness,
        text=True,
        capture_output=True,
        env=environment,
        check=False,
        timeout=10,
    )
    calls = index_calls.read_text(encoding="utf-8").splitlines() if index_calls.exists() else []
    cli = cli_calls.read_text(encoding="utf-8").splitlines() if cli_calls.exists() else []
    return result, calls, cli


def test_prefix_is_resolved_once_without_loading_ros2_cli(bash, tmp_path):
    prefix = package_prefix(tmp_path, "selected overlay with spaces")
    result, calls, cli = lookup(bash, tmp_path, [prefix])
    assert result.returncode == 0, result.stderr
    assert calls == ["robot_localization"]
    assert cli == []
    assert f"EKF_BINARY={prefix.as_posix()}/lib/robot_localization/ekf_node" in result.stdout


@pytest.mark.parametrize("reverse_order", [False, True])
def test_ament_overlay_search_order_is_preserved(bash, tmp_path, reverse_order):
    overlay = package_prefix(tmp_path, "workspace overlay")
    underlay = package_prefix(tmp_path, "ros underlay")
    prefixes = [underlay, overlay] if reverse_order else [overlay, underlay]
    result, _calls, _cli = lookup(bash, tmp_path, prefixes)
    assert result.returncode == 0, result.stderr
    assert f"SELECTED_PREFIX={prefixes[0].as_posix()}" in result.stdout


def test_missing_package_keeps_existing_actionable_error(bash, tmp_path):
    prefix = package_prefix(tmp_path, "no package", package_present=False)
    result, _calls, _cli = lookup(bash, tmp_path, [prefix])
    assert result.returncode == 1
    assert "ROS package missing: robot_localization" in result.stderr
    assert "install ros-humble-robot-localization in the NJRH-car image/container." in result.stderr
    assert "EKF_BINARY=" not in result.stdout


def test_selected_overlay_without_ekf_does_not_fall_back_to_underlay(bash, tmp_path):
    overlay = package_prefix(tmp_path, "incomplete overlay", executable_present=False)
    underlay = package_prefix(tmp_path, "complete underlay")
    result, _calls, _cli = lookup(bash, tmp_path, [overlay, underlay])
    assert result.returncode == 1
    assert "EKF binary missing or not executable:" in result.stderr
    assert f"{overlay.as_posix()}/lib/robot_localization/ekf_node" in result.stderr
    assert "EKF_BINARY=" not in result.stdout
