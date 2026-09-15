"""Launch-prefix contract; stub ROS imports, never start any ROS process."""
import ast
import importlib.util
import os
from pathlib import Path
import shlex
import sys
import types

import pytest


ROOT = Path(__file__).resolve().parents[3]
OVERLAY = ROOT / "scripts/jetson/runtime_overlay"
LAUNCH_CASES = [
    (OVERLAY / "launch/occupancy_localization.launch.py", "occupancy_grid_localizer"),
    (OVERLAY / "launch/jt128_localization_sensing.launch.py", "nav_cloud_preprocessor"),
    (OVERLAY / "launch/pointcloud_accel_pipeline.launch.py", "pointcloud_accel_container"),
    (OVERLAY / "launch/pointcloud_perception_pipeline.launch.py", "pointcloud_perception_pipeline"),
    (ROOT / "src/robot_bringup/launch/standard_navigation.launch.py", "controller_server"),
    (ROOT / "src/robot_bringup/launch/local_costmap_debug.launch.py", "controller_server"),
]


def load_launch(path, monkeypatch):
    # ROS is the external launch adapter. Import the complete production module;
    # exercise its public prefix interface without creating ROS participants.
    for node in ast.parse(path.read_text(encoding="utf-8")).body:
        if isinstance(node, ast.ImportFrom) and node.module.startswith(
                ("launch", "nav2_common")):
            module = types.ModuleType(node.module)
            for name in node.names:
                setattr(module, name.name, type(name.name, (), {}))
            monkeypatch.setitem(sys.modules, node.module, module)
    monkeypatch.syspath_prepend(str(OVERLAY / "scripts"))
    spec = importlib.util.spec_from_file_location("startup_prefix_fixture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def startup_env(monkeypatch):
    for key in tuple(os.environ):
        if key.startswith("NJRH_"):
            monkeypatch.delenv(key)
    monkeypatch.setenv("NJRH_CPU_AFFINITY_ENABLED", "true")
    monkeypatch.setenv("NJRH_NAVIGATION_CPU_PROFILE", "navigation_5cpu")
    monkeypatch.setenv("NJRH_STARTUP_CPU_SESSION", "/tmp/njrh_reports/session with spaces")
    monkeypatch.setenv("NJRH_OVERLAY_ROOT", "/fixture/overlay with spaces")


@pytest.mark.parametrize("path,role", LAUNCH_CASES)
def test_startup_launch_defers_mask_selection_until_process_execution(
        path, role, startup_env, monkeypatch):
    monkeypatch.setenv("NJRH_CPUSET_" + role.upper(), "0-1,4")
    module = load_launch(path, monkeypatch)
    assert shlex.split(module.cpu_affinity_prefix(role)) == [
        "python3", "/fixture/overlay with spaces/scripts/startup_cpu_affinity.py",
        "exec", "--session", "/tmp/njrh_reports/session with spaces",
        "--steady-cpus", "0-1,4", "--role", role, "--",
    ]
    assert os.environ["NJRH_CPUSET_" + role.upper()] == "0-1,4"


@pytest.mark.parametrize("path,role", LAUNCH_CASES)
@pytest.mark.parametrize("unrelated", ["fastlio_mapping", "arm_control", "custom_unrelated"])
def test_navigation_launch_never_boosts_mapping_arm_or_unknown_roles(
        path, role, unrelated, startup_env, monkeypatch):
    monkeypatch.setenv("NJRH_CPUSET_" + unrelated.upper(), "5-7")
    module = load_launch(path, monkeypatch)
    assert module.cpu_affinity_prefix(unrelated) == "taskset -c 5-7"


@pytest.mark.parametrize("path,role", LAUNCH_CASES)
@pytest.mark.parametrize("change,expected", [
    ({"NJRH_STARTUP_CPU_SESSION": ""}, "taskset -c 0-1,4"),
    ({"NJRH_NAVIGATION_CPU_PROFILE": "site_default"}, "taskset -c 0-1,4"),
    ({"NJRH_CPU_AFFINITY_ENABLED": "false"}, None),
])
def test_regular_and_disabled_launch_affinity_are_unchanged(
        path, role, change, expected, startup_env, monkeypatch):
    monkeypatch.setenv("NJRH_CPUSET_" + role.upper(), "0-1,4")
    for name, value in change.items():
        monkeypatch.setenv(name, value)
    assert load_launch(path, monkeypatch).cpu_affinity_prefix(role) == expected


@pytest.mark.parametrize("path,role", LAUNCH_CASES)
def test_launch_without_cpuset_adds_no_prefix(path, role, startup_env, monkeypatch):
    assert load_launch(path, monkeypatch).cpu_affinity_prefix(role) is None


@pytest.mark.parametrize("path,role", LAUNCH_CASES)
def test_workspace_fallback_does_not_depend_on_launch_working_directory(
        path, role, startup_env, monkeypatch, tmp_path):
    monkeypatch.delenv("NJRH_OVERLAY_ROOT")
    monkeypatch.setenv("NJRH_CPUSET_" + role.upper(), "0-1,4")
    monkeypatch.chdir(tmp_path)
    command = shlex.split(load_launch(path, monkeypatch).cpu_affinity_prefix(role))
    assert command[1] == (OVERLAY / "scripts/startup_cpu_affinity.py").as_posix()


@pytest.mark.parametrize("path,role", LAUNCH_CASES)
def test_all_navigation_roles_present_in_launch_use_session_wrapper(
        path, role, startup_env, monkeypatch):
    module = load_launch(path, monkeypatch)
    roles = {
        node.args[0].value
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8")))
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        and node.func.id in ("cpu_affinity_prefix", "with_cpu_affinity")
        and node.args and isinstance(node.args[0], ast.Constant)
    }
    assert roles
    for launch_role in roles:
        monkeypatch.setenv("NJRH_CPUSET_" + launch_role.upper(), "1,4")
        command = shlex.split(module.cpu_affinity_prefix(launch_role))
        assert command[-4:] == ["1,4", "--role", launch_role, "--"]
