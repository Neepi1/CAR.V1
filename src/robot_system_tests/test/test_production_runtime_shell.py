import os
from pathlib import Path
import shutil
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
CAN_WAIT_SCRIPT = REPO_ROOT / "scripts" / "jetson" / "bringup_ranger_can_wait.sh"
CONTAINER_SCRIPT = REPO_ROOT / "scripts" / "jetson" / "njrh_container.sh"
SYSTEMD_RUNTIME_SCRIPT = REPO_ROOT / "scripts" / "jetson" / "njrh_systemd_runtime.sh"
AUTOSTART_SCRIPT = REPO_ROOT / "scripts" / "jetson" / "install_njrh_autostart.sh"


def _bash_executable() -> str:
    if os.name == "nt":
        git_bash = Path("C:/Program Files/Git/bin/bash.exe")
        if git_bash.is_file():
            return str(git_bash)
    discovered = shutil.which("bash")
    if discovered:
        return discovered
    pytest.skip("bash is required for production runtime shell dry-run tests")


def _bash_path(path: Path) -> str:
    if os.name != "nt":
        return str(path)
    completed = subprocess.run(
        [
            _bash_executable(),
            "-lc",
            'cygpath -u "$1"',
            "njrh-path",
            str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return completed.stdout.strip()


def _function_block(source: str, name: str, next_name: str) -> str:
    start = source.index(f"{name}()")
    end = source.index(f"\n{next_name}()", start)
    return source[start:end]


def test_can_wait_accepts_readable_non_executable_bringup_script() -> None:
    source = CAN_WAIT_SCRIPT.read_text(encoding="utf-8")

    assert '[[ ! -f "${CAN_BRINGUP_SCRIPT}" || ! -r "${CAN_BRINGUP_SCRIPT}" ]]' in source
    assert '[[ ! -x "${CAN_BRINGUP_SCRIPT}" ]]' not in source
    assert 'bash "${CAN_BRINGUP_SCRIPT}"' in source


@pytest.mark.parametrize(
    "action",
    ("start-common", "start-runtime", "start-debug-runtime"),
)
def test_runtime_start_actions_refuse_existing_provision_motion_lock(
    tmp_path: Path,
    action: str,
) -> None:
    motion_lock = tmp_path / "motion.lock"
    motion_lock.write_text("provisioning\n", encoding="utf-8")
    environment = os.environ.copy()
    environment["NJRH_PROVISION_MOTION_LOCK"] = _bash_path(motion_lock)

    completed = subprocess.run(
        [_bash_executable(), _bash_path(CONTAINER_SCRIPT), action],
        env=environment,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=10,
    )

    output = completed.stdout + completed.stderr
    assert completed.returncode != 0
    assert "provisioning motion lock is present" in output


def test_common_services_docker_exec_places_environment_before_container() -> None:
    source = CONTAINER_SCRIPT.read_text(encoding="utf-8")
    block = _function_block(
        source,
        "start_common_services",
        "stop_detached_runtime_processes",
    )
    invocation_start = block.index("docker exec ")
    invocation_end = block.index("\n  sleep 2", invocation_start)
    invocation = block[invocation_start:invocation_end]
    container_position = invocation.index('"$CONTAINER_NAME"')

    assert "-e " in invocation
    assert invocation.rfind("-e ") < container_position


def test_api_secret_is_loaded_only_by_runtime_service() -> None:
    source = AUTOSTART_SCRIPT.read_text(encoding="utf-8")

    assert source.count("EnvironmentFile=-${SECRETS_ENV_FILE}") == 1


def _write_fake_docker(fake_bin: Path) -> Path:
    docker = fake_bin / "docker"
    docker.write_text(
        """#!/usr/bin/env bash
set -euo pipefail

case "${1:-}" in
  ps)
    printf '%s\n' "${NJRHTEST_CONTAINER_NAME:-NJRH-car}"
    ;;
  top)
    count=0
    if [[ -f "${NJRHTEST_TOP_COUNT_FILE}" ]]; then
      count="$(cat "${NJRHTEST_TOP_COUNT_FILE}")"
    fi
    count=$((count + 1))
    printf '%s\n' "${count}" >"${NJRHTEST_TOP_COUNT_FILE}"
    if [[ "${NJRHTEST_TOP_MODE:-clears}" == "unavailable" ]] \
      || { [[ "${NJRHTEST_TOP_MODE:-clears}" == "unavailable-after" ]] && (( count > 1 )); }; then
      exit 2
    fi
    printf 'PID COMMAND\n'
    if [[ "${NJRHTEST_TOP_MODE:-clears}" == "remains" ]] \
      || { [[ "${NJRHTEST_TOP_MODE:-clears}" == "clears" ]] && (( count == 1 )); } \
      || { [[ "${NJRHTEST_TOP_MODE:-clears}" == "unavailable-after" ]] && (( count == 1 )); }; then
      printf '42 /bin/bash run_common_services.sh\n'
    fi
    ;;
  exec)
    printf 'exec\n' >>"${NJRHTEST_DOCKER_LOG}"
    exit "${NJRHTEST_EXEC_RC:-0}"
    ;;
  *)
    exit 0
    ;;
esac
""",
        encoding="utf-8",
        newline="\n",
    )
    docker.chmod(0o755)
    return docker


def _run_systemd_stop(
    tmp_path: Path,
    *,
    top_mode: str,
    exec_rc: int = 0,
) -> tuple[subprocess.CompletedProcess[str], str]:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    _write_fake_docker(fake_bin)
    top_count = tmp_path / "top-count"
    docker_log = tmp_path / "docker.log"
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{_bash_path(fake_bin)}:/usr/bin:/bin",
            "NJRH_WORKSPACE_HOST": _bash_path(REPO_ROOT),
            "NJRH_CONTAINER_NAME": "NJRH-car",
            "NJRHTEST_CONTAINER_NAME": "NJRH-car",
            "NJRHTEST_TOP_COUNT_FILE": _bash_path(top_count),
            "NJRHTEST_DOCKER_LOG": _bash_path(docker_log),
            "NJRHTEST_TOP_MODE": top_mode,
            "NJRHTEST_EXEC_RC": str(exec_rc),
        }
    )
    completed = subprocess.run(
        [_bash_executable(), _bash_path(SYSTEMD_RUNTIME_SCRIPT), "stop"],
        env=environment,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=10,
    )
    log = docker_log.read_text(encoding="utf-8") if docker_log.exists() else ""
    return completed, log


def test_systemd_stop_succeeds_without_runtime_processes(tmp_path: Path) -> None:
    completed, docker_log = _run_systemd_stop(tmp_path, top_mode="none")

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert docker_log == ""


def test_systemd_stop_succeeds_after_verified_cleanup(tmp_path: Path) -> None:
    completed, docker_log = _run_systemd_stop(tmp_path, top_mode="clears")

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert docker_log == "exec\n"


def test_systemd_stop_propagates_cleanup_failure(tmp_path: Path) -> None:
    completed, _ = _run_systemd_stop(tmp_path, top_mode="clears", exec_rc=17)

    output = completed.stdout + completed.stderr
    assert completed.returncode != 0
    assert "runtime cleanup command failed" in output


@pytest.mark.parametrize("top_mode", ("remains", "unavailable-after"))
def test_systemd_stop_requires_verified_process_exit(
    tmp_path: Path,
    top_mode: str,
) -> None:
    completed, _ = _run_systemd_stop(tmp_path, top_mode=top_mode)

    output = completed.stdout + completed.stderr
    assert completed.returncode != 0
    assert "unable to verify that runtime processes stopped" in output
