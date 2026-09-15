"""Exercise extracted compatibility logic; never run a dashboard/CPU tool."""

import ast
import re
from types import SimpleNamespace
import textwrap

import pytest

from test_imu_pipeline_runtime import SCRIPTS, bash, run_shell, shell_function


def injected_driver_methods():
    source = (SCRIPTS / "patch_dashboard_runtime.py").read_text(encoding="utf-8")
    module = ast.parse(source)
    value = next(stmt.value.value for stmt in module.body
                 if isinstance(stmt, ast.Assign) and isinstance(stmt.value, ast.Constant)
                 and any(isinstance(target, ast.Name) and target.id == "NEW_DRIVER_READY_BLOCK"
                         for target in stmt.targets))
    return ast.parse(textwrap.dedent(value))


@pytest.mark.parametrize("imu_command,expected", [
    ("/install/robot_bringup/lib/robot_bringup/imu_pipeline_node --remap-params /r --filter-params /f", True),
    ("/install/robot_hesai_jt128/lib/robot_hesai_jt128/imu_axis_remap_node --ros-args", True),
    ("/install/robot_bringup/lib/robot_bringup/imu_pipeline_node_other --ros-args", False),
    ("", False),
])
def test_dashboard_injected_presence_check_accepts_exact_host_or_standalone(imu_command, expected):
    method = next(node for node in injected_driver_methods().body
                  if isinstance(node, ast.FunctionDef) and node.name == "_driver_stack_running")
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), "injected_dashboard_method", "exec"), namespace)
    commands = ["/bin/hesai_ros_driver_node --ros-args", "/bin/pointcloud_axis_remap_node --ros-args", imu_command]
    owner = SimpleNamespace(_process_exists=lambda pattern: any(re.search(pattern, command) for command in commands))
    assert namespace["_driver_stack_running"](owner) is expected


def test_dashboard_existing_driver_rebuild_cleanup_covers_host_not_other_executables():
    ready = next(node for node in injected_driver_methods().body
                 if isinstance(node, ast.FunctionDef) and node.name == "_ensure_driver_ready")
    lists = [ast.literal_eval(node.args[0]) for node in ast.walk(ready)
             if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
             and node.func.attr == "_kill_patterns"]
    assert len(lists) == 2
    for patterns in lists:
        assert "imu_axis_remap" in patterns and "hesai_ros_driver_node" in patterns
        assert any(re.search(pattern, "/install/robot_bringup/lib/robot_bringup/imu_pipeline_node --filter-params /f")
                   for pattern in patterns)
        assert not any(re.search(pattern, "/install/robot_bringup/lib/robot_bringup/imu_pipeline_node_other --ros-args")
                       for pattern in patterns)


@pytest.mark.parametrize("remap_cpus,filter_cpus,host_present,applied", [
    ("2", "2", True, True),
    ("2", "2", False, False),
    ("6", "2", True, False),
    ("", "", True, False),
])
def test_manual_host_mapping_reuses_one_existing_compatible_role(bash, remap_cpus, filter_cpus, host_present, applied):
    result = run_shell(bash, r'''
pgrep() {
  [[ "$1" == -f && "$2" == "$NJRH_IMU_PIPELINE_PROCESS_PATTERN" && "$TEST_HOST_PRESENT" == true ]] || return 1
  printf '412\n413\n'
}
njrh_apply_affinity_to_pids() { printf 'APPLY'; printf ' <%s>' "$@"; printf '\n'; }
taskset() { echo UNEXPECTED_TASKSET; return 97; }
njrh_apply_imu_pipeline_host_affinity
printf 'MASKS <%s> <%s>\n' "$TEST_REMAP_CPUS" "$TEST_FILTER_CPUS"
''', TEST_REMAP_CPUS=remap_cpus, TEST_FILTER_CPUS=filter_cpus,
                       TEST_HOST_PRESENT="true" if host_present else "false")
    assert result.returncode == 0, result.stderr
    assert "UNEXPECTED_TASKSET" not in result.stdout
    assert f"MASKS <{remap_cpus}> <{filter_cpus}>" in result.stdout
    if applied:
        assert result.stdout.count("APPLY") == 1
        assert "APPLY <imu_axis_remap> <412> <413>" in result.stdout
        assert "robot_local_state" not in result.stdout
    else:
        assert "APPLY" not in result.stdout
        if host_present:
            assert "IMU host affinity unchanged" in result.stderr


def test_manual_tools_wire_shared_host_mapping_without_changing_existing_roles():
    direct = (SCRIPTS / "apply_cpu_affinity.sh").read_text(encoding="utf-8")
    ab = (SCRIPTS / "run_cpu_core_allocation_ab.sh").read_text(encoding="utf-8")
    for source in (direct, ab):
        assert 'source "${SCRIPT_DIR}/imu_pipeline_helpers.sh"' in source
        assert source.count("\nnjrh_apply_imu_pipeline_host_affinity\n") + source.count("\n  njrh_apply_imu_pipeline_host_affinity\n") == 1
    assert 'apply_pattern imu_axis_remap "imu_axis_remap"' in direct
    apply_live = shell_function(ab, "apply_live")
    assert "njrh_apply_imu_pipeline_host_affinity" in apply_live
    assert "njrh_apply_affinity_to_pids imu_axis_remap ${pids}" in apply_live
    assert "njrh_apply_affinity_to_pids robot_local_state ${pids}" in apply_live
