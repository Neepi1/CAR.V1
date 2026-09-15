"""Exercise the production entry point with fake Docker/filesystem boundaries."""

import os
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]
KEY = "NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION"


def bash():
    return "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")


def executable(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("#!/usr/bin/env bash\nset -eu\n" + content, encoding="utf-8")
    path.chmod(0o755)


def run_outer_entry(tmp_path, env_text="", override_text=""):
    executable(tmp_path / "scripts/jetson/njrh_container.sh", "exit 0\n")
    executable(tmp_path / "bin/docker", '''
if [[ "$1" == ps ]]; then echo NJRH-test; exit 0; fi
printf '%s\\0' "$@" > "$TEST_DOCKER_ARGS"
''')
    # The real entry point also clears old /tmp status files on startup.
    # Intercept that filesystem boundary; never touch actual runtime files.
    executable(tmp_path / "bin/rm", "exit 0\n")
    (tmp_path / "runtime.env").write_text(env_text, encoding="utf-8")
    (tmp_path / "override.env").write_text(override_text, encoding="utf-8")
    env = {k: v for k, v in os.environ.items()
           if not k.startswith("NJRH_") and k != "ROBOT_API_TOKEN"}
    env.update({
        "TEST_RUNNER": (ROOT / "scripts/jetson/njrh_systemd_runtime.sh").as_posix(),
        "TEST_DOCKER_ARGS": (tmp_path / "docker.args").as_posix(),
        "NJRH_WORKSPACE_HOST": tmp_path.as_posix(),
        "NJRH_WORKSPACE_CONTAINER": "/test/workspace",
        "NJRH_CONTAINER_NAME": "NJRH-test",
        "NJRH_PROVISION_MOTION_LOCK": (tmp_path / "absent-motion-lock").as_posix(),
        "NJRH_RUNTIME_OVERRIDE_ENV": (tmp_path / "override.env").as_posix(),
        "NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE": "skip",
    })
    command = '''export PATH="$PWD/bin:$PATH"
set -a
source runtime.env
set +a
exec bash "$TEST_RUNNER" run
'''
    result = subprocess.run([bash(), "-c", command], cwd=tmp_path, env=env,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr
    args = (tmp_path / "docker.args").read_bytes().decode().split("\0")
    assert "scripts/run_common_services.sh" in args[-2]
    values = [args[i + 1] for i, arg in enumerate(args[:-1]) if arg == "-e"]
    return dict(item.split("=", 1) for item in values if "=" in item)


@pytest.mark.parametrize("env_text,override_text,expected", [
    ("", "", "true"),
    (f"{KEY}=true\n", "", "true"),
    (f"{KEY}=false\n", "", "false"),
    (f"{KEY}=true\n", f"{KEY}=false\n", "false"),
])
def test_outer_entry_passes_prestart_policy_to_container(tmp_path, env_text, override_text, expected):
    assert run_outer_entry(tmp_path, env_text, override_text)[KEY] == expected


@pytest.mark.parametrize("existing", [None, "false", "true"])
def test_installer_env_creation_and_reinstall_enable_prestart(tmp_path, existing):
    source = (ROOT / "scripts/jetson/install_njrh_autostart.sh").read_text(encoding="utf-8")
    # Exercise the real env writer, but never install units or run systemctl.
    functions = source[:source.index('\ncase "${ACTION}" in')]
    harness = '''#!/usr/bin/env bash
set -euo pipefail
export NJRH_AUTOSTART_ENV_FILE="$PWD/runtime.env"
export NJRH_SECRETS_ENV_FILE="$PWD/secrets.env"
sudo() {
  case "$1" in
    -v|chown|chmod) return 0 ;;
    grep|sed|mkdir|tee|awk) command "$@" ;;
    *) echo "unexpected sudo command" >&2; return 99 ;;
  esac
}
'''
    harness += functions + '\nwrite_env_file_if_missing\nwrite_env_file_if_missing\n'
    (tmp_path / "installer-env.sh").write_text(harness, encoding="utf-8")
    if existing is not None:
        (tmp_path / "runtime.env").write_text(
            f"{KEY}={existing}\nUNRELATED_SETTING=preserve_me\n", encoding="utf-8")
    result = subprocess.run([bash(), "installer-env.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    text = (tmp_path / "runtime.env").read_text(encoding="utf-8")
    assert [line for line in text.splitlines() if line.startswith(KEY + "=")] == [KEY + "=true"]
    if existing is not None:
        assert "UNRELATED_SETTING=preserve_me" in text
    # Use only the policy under test: fixture workspace paths must stay isolated.
    policy = "\n".join(line for line in text.splitlines() if line.startswith(KEY + "="))
    assert run_outer_entry(tmp_path / "entry", policy)[KEY] == "true"
