"""Native /proc ownership audit and extracted common-owner shell regressions.

No ROS, SSH, service owner, or robot connection is started. Linux native tests
compile the real checker; shell-only contracts can also run under Git Bash.
Set NJRH_RUNTIME_PROCESS_CHECK_BIN / NJRH_TEST_BASH to test local candidates.
"""

import errno
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "src/robot_bringup/src/runtime_process_check.cpp"
COMMON = ROOT / "scripts/jetson/runtime_overlay/scripts/run_common_services.sh"


def shell_function(name):
    source = COMMON.read_text(encoding="utf-8")
    start = source.index(name + "() {")
    return source[start:source.index("\n}\n", start) + 3]


@pytest.fixture(scope="session")
def bash():
    candidate = os.environ.get("NJRH_TEST_BASH")
    if not candidate and sys.platform == "win32":
        candidate = "C:/Program Files/Git/bin/bash.exe"
    candidate = candidate or shutil.which("bash")
    if not candidate or not Path(candidate).is_file():
        pytest.skip("No Bash available for isolated shell sections")
    return candidate


def compiler_command():
    if not sys.platform.startswith("linux"):
        pytest.skip("Native proc checker requires Linux; no robot/remote execution allowed")
    result = shlex.split(os.environ.get("CXX", ""))
    result = result or next(([x] for n in ("c++", "g++", "clang++") if (x := shutil.which(n))), [])
    if not result:
        pytest.fail("Native checker tests require a C++17 compiler")
    return result


@pytest.fixture(scope="session")
def compiler():
    return compiler_command()


@pytest.fixture(scope="session")
def checker(tmp_path_factory):
    if not sys.platform.startswith("linux"):
        pytest.skip("Native proc checker requires Linux; no robot/remote execution allowed")
    configured = os.environ.get("NJRH_RUNTIME_PROCESS_CHECK_BIN")
    if configured:
        binary = Path(configured).resolve()
        assert binary.is_file() and os.access(binary, os.X_OK)
        return binary
    compiler = compiler_command()
    binary = tmp_path_factory.mktemp("native_process_check") / "runtime_process_check"
    built = subprocess.run(compiler + ["-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                                     str(SOURCE), "-o", str(binary)],
                           capture_output=True, text=True, timeout=120)
    assert built.returncode == 0, built.stdout + built.stderr
    return binary


@pytest.fixture(scope="session")
def faults(tmp_path_factory, compiler):
    library = tmp_path_factory.mktemp("proc_faults") / "faults.so"
    source = Path(__file__).with_name("runtime_process_check_faults.cpp")
    built = subprocess.run(compiler + ["-std=c++17", "-shared", "-fPIC", str(source),
                                     "-ldl", "-o", str(library)],
                           capture_output=True, text=True, timeout=120)
    assert built.returncode == 0, built.stdout + built.stderr
    return library


def stat_text(pid, start=100, state="S", flags=0, comm="name ) with spaces"):
    fields = [state] + ["0"] * 19
    fields[6] = str(flags)
    fields[19] = str(start)
    return f"{pid} ({comm}) " + " ".join(fields) + "\n"


class ProcFixture:
    def __init__(self, directory):
        self.root = directory / "proc"
        self.root.mkdir()
        boot = self.root / "sys/kernel/random/boot_id"
        boot.parent.mkdir(parents=True)
        boot.write_text("fixture-boot\n")
        self.supervisor_exe = directory / "bash"
        self.api_exe = directory / "robot_api_server_node"
        self.other_exe = directory / "diagnostic"
        for path in (self.supervisor_exe, self.api_exe, self.other_exe):
            path.touch()
        self.argument = "/overlay/scripts/run_robot_api_server_supervised.sh"

    def add(self, pid, exe, args=(), **stat):
        directory = self.root / str(pid)
        directory.mkdir()
        (directory / "stat").write_text(stat_text(pid, **stat))
        if exe is not None:
            (directory / "exe").symlink_to(exe)
        (directory / "cmdline").write_bytes(b"".join(os.fsencode(a) + b"\0" for a in args))
        return directory

    def owners(self):
        self.add(10, self.supervisor_exe, ["bash", self.argument])
        self.add(20, self.api_exe, [str(self.api_exe)])

    def command(self, binary):
        return [str(binary), "--proc-root", str(self.root),
                "--supervisor-exe", str(self.supervisor_exe),
                "--supervisor-arg", self.argument, "--api-exe", str(self.api_exe)]

    def run(self, binary, env=None):
        return subprocess.run(self.command(binary), capture_output=True, text=True,
                              env=env, timeout=5)


@pytest.fixture
def proc(tmp_path, checker):
    return ProcFixture(tmp_path)


def test_exact_identity_ignores_diagnostic_argv(proc, checker):
    proc.owners()
    proc.add(30, proc.other_exe, [str(proc.api_exe), proc.argument])
    proc.add(40, proc.supervisor_exe, ["bash", "-c", f"cat {proc.argument}"])
    proc.add(50, proc.supervisor_exe, ["bash", proc.argument + ".backup"])
    proc.add(60, proc.supervisor_exe, ["bash", "--script=" + proc.argument])
    result = proc.run(checker)
    assert (result.returncode, result.stdout) == (0, "unique 1 1\n")


@pytest.mark.parametrize("supervisors,nodes", [(0, 0), (0, 1), (1, 0), (2, 1), (1, 2), (2, 2)])
def test_missing_and_duplicates(proc, checker, supervisors, nodes):
    for index in range(supervisors):
        proc.add(10 + index, proc.supervisor_exe, ["bash", proc.argument])
    for index in range(nodes):
        proc.add(20 + index, proc.api_exe)
    result = proc.run(checker)
    assert (result.returncode, result.stdout) == (
        50, f"ownership_fault {supervisors} {nodes}\n")


def test_expected_symlink_is_canonicalized(proc, checker, tmp_path):
    proc.owners()
    alias = tmp_path / "api-alias"
    alias.symlink_to(proc.api_exe)
    proc.api_exe = alias
    assert proc.run(checker).stdout == "unique 1 1\n"


def test_zombies_and_kernel_threads_are_not_owners(proc, checker):
    proc.owners()
    proc.add(30, None, state="Z")
    proc.add(40, None, flags=0x00200000)
    proc.add(50, proc.api_exe, state="Z")
    assert proc.run(checker).returncode == 0


@pytest.mark.parametrize("failure,code", [
    ("missing_root", 40), ("missing_boot", 41), ("missing_binary", 40),
    ("truncated_stat", 41), ("unterminated_cmdline", 41), ("missing_exe", 42),
])
def test_unavailable_evidence_is_not_missing_process(proc, checker, failure, code):
    proc.owners()
    if failure == "missing_root":
        proc.root = proc.root / "missing"
    elif failure == "missing_boot":
        (proc.root / "sys/kernel/random/boot_id").unlink()
    elif failure == "missing_binary":
        proc.api_exe.unlink()
    elif failure == "truncated_stat":
        (proc.root / "10/stat").write_text("10 (broken) S\n")
    elif failure == "unterminated_cmdline":
        (proc.root / "10/cmdline").write_bytes(os.fsencode(proc.argument))
    elif failure == "missing_exe":
        (proc.root / "20/exe").unlink()
    result = proc.run(checker)
    assert result.returncode == code, result.stdout + result.stderr
    assert result.stdout.endswith(" - -\n")


@pytest.mark.parametrize("action,code", [("permission", 41), ("io", 41),
                                        ("exit_after_exe", 42), ("reuse", 42),
                                        ("boot_change", 42)])
def test_observer_errors_and_owner_races(proc, checker, faults, action, code):
    proc.owners()
    env = dict(os.environ, LD_PRELOAD=str(faults), NJRH_TEST_PROC_TARGET=str(proc.root / "10"),
               NJRH_TEST_PROC_FAULT=action, NJRH_TEST_REPLACEMENT_STAT=stat_text(10, start=999))
    result = proc.run(checker, env)
    assert result.returncode == code, result.stdout + result.stderr
    assert result.stdout.endswith(" - -\n")


def test_unrelated_exit_is_not_observer_failure(proc, checker, faults):
    proc.owners()
    proc.add(30, proc.other_exe)
    env = dict(os.environ, LD_PRELOAD=str(faults), NJRH_TEST_PROC_TARGET=str(proc.root / "30"),
               NJRH_TEST_PROC_FAULT="exit_before_exe")
    result = proc.run(checker, env)
    assert (result.returncode, result.stdout) == (0, "unique 1 1\n")


@pytest.mark.parametrize("action,error", [("exe_enoent", errno.ENOENT), ("exe_esrch", errno.ESRCH)])
@pytest.mark.parametrize("pid,on_read", [(10, 1), (20, 1), (30, 1),
                                       (10, 2), (20, 2), (10, 3), (20, 3)])
def test_exe_exit_race_with_retained_pid_is_unknown(proc, checker, faults, action, error, pid, on_read):
    proc.owners()
    proc.add(30, proc.other_exe)
    target = proc.root / str(pid)
    env = dict(os.environ, LD_PRELOAD=str(faults), NJRH_TEST_PROC_TARGET=str(target),
               NJRH_TEST_PROC_FAULT=action, NJRH_TEST_PROC_FAULT_ON_READ=str(on_read))
    result = proc.run(checker, env)
    assert target.is_dir() and (target / "stat").read_text() == stat_text(pid)
    assert (result.returncode, result.stdout) == (42, "observer_transient - -\n"), result.stderr
    assert f"pid={pid}" in result.stderr and f"errno={error}" in result.stderr
    # A failed exe read cannot prove a PID irrelevant, even when we know its
    # fixture executable. The next clean observation must still count owners.
    recovered = proc.run(checker)
    assert (recovered.returncode, recovered.stdout) == (0, "unique 1 1\n")


def test_unreadable_proc_root_never_becomes_zero_counts(proc, checker, faults):
    proc.owners()
    env = dict(os.environ, LD_PRELOAD=str(faults), NJRH_TEST_PROC_TARGET=str(proc.root),
               NJRH_TEST_PROC_FAULT="root_permission")
    result = proc.run(checker, env)
    assert (result.returncode, result.stdout) == (40, "observer_unavailable - -\n")


def run_section(bash, tmp_path, suffix, stub=None, env=None):
    environment = dict(os.environ, SCRIPT_DIR="/test/scripts", PROJECT_ROOT="/test/project")
    environment.update(env or {})
    if stub is not None:
        stub_file = tmp_path / "checker-stub.sh"
        stub_file.write_text("#!/usr/bin/env bash\n" + stub, encoding="utf-8", newline="\n")
        stub_file.chmod(0o755)
        environment["NJRH_RUNTIME_PROCESS_CHECK_BIN"] = str(stub_file)
    functions = "\n".join(shell_function(name) for name in (
        "query_robot_api_process_ownership", "verify_robot_api_server_common_health_or_exit",
        "wait_for_robot_api_server_common_ready"))
    return subprocess.run([bash, "--noprofile", "--norc", "-s"],
                          input="set -euo pipefail\n" + functions + "\n" + suffix,
                          capture_output=True, text=True, env=environment, timeout=8)


@pytest.mark.parametrize("record,code,expected", [
    ("unique 1 1", 0, 0), ("ownership_fault 0 1", 50, 1),
    ("ownership_fault 1 0", 50, 1), ("ownership_fault 2 1", 50, 1),
    ("ownership_fault 1 2", 50, 1), ("observer_unavailable - -", 40, 0),
    ("observer_error - -", 41, 0), ("observer_transient - -", 42, 0),
    ("ownership_fault 0 0", 41, 0), ("unique 0 0", 0, 0),
    ("ownership_fault 1 1", 50, 0), ("garbage", 0, 0),
    ("unique 1 1\nextra", 0, 0), ("ownership_fault 999999999999999999999 0", 50, 0),
    ("ownership_fault $(touch /tmp/never-execute-checker-output) 0", 50, 0),
    ("", 127, 0), ("unique 1 1", 143, 0),
])
def test_health_shell_classification(bash, tmp_path, record, code, expected):
    stub = f"printf '%s\\n' {shlex.quote(record)}\nexit {code}\n"
    result = run_section(bash, tmp_path, "verify_robot_api_server_common_health_or_exit\n", stub)
    assert result.returncode == expected, result.stdout + result.stderr
    if code not in (0, 50) or record != "unique 1 1" and expected == 0:
        assert "does not authorize complete-chain recovery" in result.stderr


def test_missing_checker_never_falls_back_or_exits_runtime(bash, tmp_path):
    result = run_section(bash, tmp_path, "verify_robot_api_server_common_health_or_exit\n",
                         env={"NJRH_RUNTIME_PROCESS_CHECK_BIN": str(tmp_path / "missing")})
    assert result.returncode == 0
    assert "missing native checker" in result.stderr
    assert "does not authorize complete-chain recovery" in result.stderr


def test_one_invocation_checks_both_identities(bash, tmp_path):
    call_file = tmp_path / "calls"
    arg_file = tmp_path / "args"
    stub = "printf 'call\\n' >> \"$CALL_FILE\"\nprintf '%s\\n' \"$@\" > \"$ARG_FILE\"\nprintf 'unique 1 1\\n'\n"
    result = run_section(bash, tmp_path, "verify_robot_api_server_common_health_or_exit\n", stub,
                         env={"CALL_FILE": str(call_file), "ARG_FILE": str(arg_file)})
    assert result.returncode == 0, result.stderr
    assert call_file.read_text().splitlines() == ["call"]
    args = arg_file.read_text().splitlines()
    assert args[0::2] == ["--supervisor-exe", "--supervisor-arg", "--api-exe"]
    assert args[3] == "/test/scripts/run_robot_api_server_supervised.sh"
    assert args[5] == "/test/project/install/robot_api_server/lib/robot_api_server/robot_api_server_node"


@pytest.mark.parametrize("stub", ["printf 'observer_unavailable - -\\n'; exit 40\n",
                                 "printf 'ownership_fault 0 1\\n'; exit 50\n"])
def test_startup_timeout_does_not_accept_unknown_or_missing(bash, tmp_path, stub):
    # Advance SECONDS in a shell sleep stub, preserving the actual readiness loop.
    result = run_section(bash, tmp_path,
                         "sleep() { SECONDS=$((SECONDS + 121)); }\n"
                         "wait_for_robot_api_server_common_ready\n", stub)
    assert result.returncode == 1
    assert "within 120s" in result.stderr
    assert "ownership ready" not in result.stderr


def test_startup_retries_then_accepts_unique_without_changing_grace(bash, tmp_path):
    marker = tmp_path / "marker"
    stub = ("if [[ -f \"$MARKER\" ]]; then printf 'unique 1 1\\n'; exit 0; fi\n"
            ": > \"$MARKER\"\nprintf 'observer_transient - -\\n'\nexit 42\n")
    result = run_section(bash, tmp_path,
                         "sleep() { printf 'poll=%s\\n' \"$1\"; SECONDS=$((SECONDS + 1)); }\n"
                         "wait_for_robot_api_server_common_ready\n", stub,
                         env={"MARKER": str(marker)})
    assert result.returncode == 0, result.stderr
    assert result.stdout == "poll=1\n"
    assert "up to 120s" in result.stderr
    assert "ownership ready (supervisor=1, node=1)" in result.stderr


def test_shell_runs_real_checker_once_with_proc_fixture(bash, tmp_path, proc, checker):
    proc.owners()
    # A direct-command override supplies the isolated fixture root and exact
    # executable paths while still exercising the production shell classifier.
    wrapper = "exec " + shlex.join(proc.command(checker)) + "\n"
    result = run_section(bash, tmp_path, "verify_robot_api_server_common_health_or_exit\n", wrapper)
    assert result.returncode == 0, result.stderr
    proc.add(21, proc.api_exe)
    result = run_section(bash, tmp_path, "verify_robot_api_server_common_health_or_exit\n", wrapper)
    assert result.returncode == 1
    assert "supervisor=1, node=2" in result.stderr


def test_no_per_pid_shell_scan_and_period_unchanged():
    source = COMMON.read_text(encoding="utf-8")
    body = shell_function("query_robot_api_process_ownership")
    assert "/proc/[0-9]*/exe" not in source
    assert "readlink" not in body and "pgrep" not in body and "eval " not in body
    assert 'sleep "${NJRH_COMMON_MAIN_HEALTH_PERIOD_SEC:-5}"' in source
    assert '"${NJRH_ROBOT_API_PROCESS_READY_TIMEOUT_SEC:-120}"' in source
    assert '"${NJRH_ROBOT_API_PROCESS_READY_POLL_SEC:-1}"' in source


def test_common_script_syntax(bash):
    result = subprocess.run([bash, "-n"], input=COMMON.read_text(encoding="utf-8"),
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
