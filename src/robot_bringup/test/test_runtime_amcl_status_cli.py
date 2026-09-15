#!/usr/bin/env python3
"""ROS-free CLI/Shell/socket integration. Only isolated sleep fixtures are started."""
import os
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest


CLI = os.environ.get("RUNTIME_AMCL_STATUS_BIN", "")
FIXTURE = os.environ.get("RUNTIME_AMCL_STATUS_FIXTURE_BIN", "")
ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts/jetson/runtime_overlay/scripts"
BASH = ("C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash"))


def function_text(path, name):
    text = path.read_text(encoding="utf-8")
    start = text.index(name + "() {")
    end = text.index("\n}\n", start) + 3
    return text[start:end]


@unittest.skipUnless(CLI and FIXTURE, "set native CLI and fixture paths")
class Integration(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="amcl-cli-")
        self.root = Path(self.temp.name)
        self.status = self.root / "status.env"
        self.server = subprocess.Popen([FIXTURE, "--serve", str(self.status)],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.addCleanup(self.stop, self.server)
        self.amcl = subprocess.Popen(["/bin/sleep", "30"])
        self.addCleanup(self.stop, self.amcl)
        self.relay = subprocess.Popen(["/bin/sleep", "30"])
        self.addCleanup(self.stop, self.relay)
        self.pidfile = self.root / "amcl.pid"
        self.pidfile.write_text(str(self.amcl.pid))
        self.relayfile = self.root / "relay.pid"
        self.relayfile.write_text(str(self.relay.pid))
        end = time.monotonic() + 2
        while not Path(str(self.status) + ".control/socket").exists():
            if self.server.poll() is not None or time.monotonic() > end:
                self.fail("fixture observer did not create socket")
            time.sleep(0.01)

    def tearDown(self):
        # Kill/wait before removing the isolated control directory.
        self.doCleanups()
        self.temp.cleanup()

    @staticmethod
    def stop(process):
        if process.poll() is None:
            process.terminate()
        try:
            process.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=2)

    def cli(self, *args, check=True):
        result = subprocess.run([CLI, "--status-file", str(self.status), *args],
                                text=True, capture_output=True, timeout=3)
        if check:
            self.assertEqual(result.returncode, 0, result.stderr)
        return result

    def submit(self, *, seed=False, lifecycle=False, result="ready", standby=False):
        args = ["submit", "--owner-pid", str(os.getpid()), "--owner-generation", "fixture-owner",
                "--map-generation", "fixture-map", "--amcl-pid-file", str(self.pidfile),
                "--amcl-exe", "/bin/sleep", "--relay-pid-file", str(self.relayfile), "--relay-exe", "/bin/sleep"]
        fields = {"AMCL_MODE": "gated", "AMCL_START_RESULT": result, "AMCL_READY": "true",
                  "AMCL_DEGRADED": "false", "LIFECYCLE_VERIFIED": str(lifecycle).lower(),
                  "AMCL_SEED_SUCCEEDED": str(seed).lower(), "SCAN_ADMISSION_ENABLED": "true",
                  "AMCL_STATIC_STANDBY_ACCEPTED": str(standby).lower()}
        for key, value in fields.items():
            args += ["--set", f"{key}={value}"]
        return self.cli(*args)

    def read(self):
        return dict(line.split("=", 1) for line in self.cli("read").stdout.splitlines())

    def test_ack_means_public_file_committed(self):
        self.submit(seed=True, lifecycle=True, standby=True)
        self.assertTrue(self.status.is_file())
        state = self.read()
        self.assertEqual(state["AMCL_READY"], "true")
        self.assertEqual(state["AMCL_CORRECTION_READY"], "false")
        self.assertEqual(state["AMCL_STATUS_WRITER"], "runtime_health_guard")

    def test_lifecycle_and_seed_not_inferred(self):
        self.submit()
        state = self.read()
        self.assertEqual(state["AMCL_LIFECYCLE_ACTIVE"], "false")
        self.assertEqual(state["AMCL_SEEDED"], "false")
        self.assertEqual(state["AMCL_READY"], "false")

    def test_dead_amcl_invalidates_after_periodic_tick(self):
        self.submit(seed=True, lifecycle=True)
        self.stop(self.amcl)
        end = time.monotonic() + 3
        while time.monotonic() < end:
            state = self.read()
            if state["AMCL_READY"] == "false":
                break
            time.sleep(0.05)
        self.assertEqual(state["AMCL_PID_ALIVE"], "false")
        self.assertEqual(state["AMCL_SEEDED"], "false")

    def test_observer_failure_is_bounded(self):
        self.stop(self.server)
        start = time.monotonic()
        result = self.cli("ping", check=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertLess(time.monotonic() - start, 1.2)

    def test_startup_shell_submits_real_lifecycle_receipt(self):
        # Extract only AMCL evidence functions; no environment/bootstrap/ROS actions.
        path = SCRIPTS / "run_amcl_shadow_localization.sh"
        script = "set -euo pipefail\n" + function_text(path, "amcl_status_cli")
        script += function_text(path, "write_amcl_runtime_status")
        script += '\nscan_admission_enabled() { return 0; }\neffective_scan_topic() { printf /scan_amcl; }\n'
        script += "write_amcl_runtime_status ready true false 'literal $(touch DO_NOT_CREATE)'\n"
        # Sleep accepts argv[0] replacement, so exact AMCL node remap can be
        # supplied as argv[0] without trying to execute a ROS node.
        special = subprocess.Popen(["__node:=amcl", "30"], executable="/bin/sleep")
        self.addCleanup(self.stop, special)
        self.pidfile.write_text(str(special.pid))
        env = os.environ.copy()
        env.update({"NJRH_PROJECT_ROOT": str(ROOT), "NJRH_AMCL_STATUS_CPP_BIN": CLI,
                    "STATUS_FILE": str(self.status), "PID_FILE": str(self.pidfile),
                    "SCAN_RELAY_PID_FILE": str(self.relayfile), "AMCL_BIN": "/bin/sleep",
                    "SCAN_RELAY_CPP_BIN": "/bin/sleep", "SCAN_RELAY_IMPL": "cpp",
                    "AMCL_NODE_NAME": "amcl", "PARAMS_FILE": "fixture.yaml", "MODE": "gated",
                    "NJRH_AMCL_STATUS_OWNER_PID": str(os.getpid()),
                    "NJRH_AMCL_STATUS_OWNER_GENERATION": "fixture-shell",
                    "AMCL_STARTUP_EPOCH_SEC": "1", "AMCL_PROGRESS_LIFECYCLE": "true",
                    "AMCL_SEED_SUCCEEDED": "true", "AMCL_SEED_RESPONSE_OK": "false",
                    "AMCL_PID_STALE_CLEARED": "false", "SCAN_ADMISSION_PID_STALE_CLEARED": "false",
                    "AMCL_NOMOTION_PROBE_USED": "false", "AMCL_NOMOTION_POSE_RECEIVED": "false",
                    "AMCL_NOMOTION_POSE_COUNT": "0", "AMCL_NOMOTION_POSE_HEADER_AGE_MS": "",
                    "AMCL_STATIC_STANDBY_ACCEPTED": "true"})
        result = subprocess.run(["bash", "-c", script], env=env, cwd=self.root,
                                capture_output=True, text=True, timeout=3)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.read()["AMCL_LIFECYCLE_ACTIVE"], "true")
        self.assertFalse((self.root / "DO_NOT_CREATE").exists())


class ShellContract(unittest.TestCase):
    @unittest.skipUnless(BASH, "bash is required")
    def test_tf_broadcast_cmdline_fallback_keeps_pid_selection(self):
        path = SCRIPTS / "run_amcl_shadow_localization.sh"
        source = path.read_text(encoding="utf-8")
        definitions = function_text(path, "amcl_cmdline_tf_broadcast_false")
        # Redirect only the proc filesystem; execute the actual fallback caller.
        definitions = definitions.replace('"/proc/', '"${TEST_PROC_ROOT}/')
        if "first_process_pid() {" in source:
            definitions += function_text(path, "first_process_pid")
        definitions += r'''
amcl_pid_from_file() { printf '%s' "${TEST_PID_FILE_VALUE}"; }
amcl_process_pids() { printf '%s' "${TEST_CANDIDATES}"; }
amcl_cmdline_tf_broadcast_false
'''
        with tempfile.TemporaryDirectory(prefix="amcl-startup-fallback-") as directory:
            root = Path(directory)
            for pid, value in ((200, "false"), (300, "true")):
                (root / str(pid)).mkdir()
                (root / str(pid) / "cmdline").write_bytes(
                    f"/opt/ros/humble/lib/nav2_amcl/amcl\0__node:=amcl\0tf_broadcast:={value}\0".encode())
            cases = [
                ("", "\n200\n300\n", 0),  # Missing PID file selects first discovered AMCL.
                ("200", "300\n", 0),     # Validated PID file still takes precedence.
                ("", "300\n200\n", 1),   # Do not pick another process just to pass.
                ("", "", 1),              # No process is not proof of tf_broadcast=false.
            ]
            for pid, candidates, expected in cases:
                with self.subTest(pid=pid, candidates=candidates):
                    env = os.environ.copy()
                    env.update(TEST_PROC_ROOT=root.as_posix(), TEST_PID_FILE_VALUE=pid,
                               TEST_CANDIDATES=candidates)
                    result = subprocess.run([BASH, "-c", "set -euo pipefail\n" + definitions],
                                            env=env, capture_output=True, text=True, timeout=3)
                    self.assertNotIn("command not found", result.stderr)
                    self.assertEqual(result.returncode, expected, result.stderr)

    @unittest.skipUnless(BASH, "bash is required")
    def test_register_submit_identity_matches_across_child_shell(self):
        nav = SCRIPTS / "run_navigation_runtime_services.sh"
        amcl = SCRIPTS / "run_amcl_shadow_localization.sh"
        definitions = function_text(nav, "register_amcl_status_for_navigation")
        definitions += function_text(amcl, "write_amcl_runtime_status")
        definitions += r'''
amcl_mode_for_navigation() { printf gated; }
scan_admission_enabled() { return 0; }
effective_scan_topic() { printf /scan_amcl; }
amcl_status_client_for_navigation() { printf '%s\0' REGISTER "$@" END >&2; }
amcl_status_cli() { printf '%s\0' SUBMIT "$@" END >&2; }
export -f write_amcl_runtime_status scan_admission_enabled effective_scan_topic amcl_status_cli
register_amcl_status_for_navigation
bash -c 'PARAMS_FILE="${NJRH_AMCL_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/amcl_shadow.yaml}"; write_amcl_runtime_status ready true false ""'
'''
        base = {
            "NJRH_BUILDING_ID": "B15", "NJRH_FLOOR_ID": "F2", "NJRH_NAV_MAP_ID": "test-002",
            "NJRH_MAP_ID": "legacy-id-must-not-win", "NAV2_MAP_YAML": "/workspaces/maps/test 002/nav/nav_map.yaml",
            "NJRH_MAP_ASSET_EPOCH": "42", "NJRH_MAP_ASSET_DIGEST": "a" * 64,
            "NJRH_OVERLAY_ROOT": "/workspaces/overlay", "NJRH_STARTUP_OWNER_PID": "1234",
            "NJRH_AMCL_STATUS_OWNER_PID": "1234", "NJRH_AMCL_STATUS_OWNER_GENERATION": "owner-generation-1",
            "startup_epoch_sec": "1", "AMCL_STARTUP_EPOCH_SEC": "1", "MODE": "gated",
            "SCAN_RELAY_IMPL": "cpp", "SCAN_RELAY_CPP_BIN": "/fixture/relay", "AMCL_BIN": "/fixture/amcl",
            "PID_FILE": "/fixture/amcl.pid", "SCAN_RELAY_PID_FILE": "/fixture/relay.pid", "AMCL_NODE_NAME": "amcl",
            "AMCL_PID_STALE_CLEARED": "false", "SCAN_ADMISSION_PID_STALE_CLEARED": "false",
            "AMCL_PROGRESS_LIFECYCLE": "true", "AMCL_SEED_SUCCEEDED": "true", "AMCL_SEED_RESPONSE_OK": "false",
            "AMCL_NOMOTION_PROBE_USED": "false", "AMCL_NOMOTION_POSE_RECEIVED": "false", "AMCL_NOMOTION_POSE_COUNT": "0",
            "AMCL_NOMOTION_POSE_HEADER_AGE_MS": "", "AMCL_STATIC_STANDBY_ACCEPTED": "true",
        }
        for params, nav_id, epoch in [("", "test-002", "42"), ("/custom/amcl settings.yaml", "test-002", "43"),
                                       ("", "", "")]:
            with self.subTest(params=params, nav_id=nav_id, epoch=epoch):
                env = os.environ.copy()
                env.update(base)
                env.update(NJRH_AMCL_PARAMS_FILE=params, NJRH_NAV_MAP_ID=nav_id, NJRH_MAP_ASSET_EPOCH=epoch)
                result = subprocess.run([BASH, "-c", "set -euo pipefail\n" + definitions],
                                        capture_output=True, env=env, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr.decode(errors="replace"))
                tokens = result.stderr.decode().split("\0")
                register = tokens[1:tokens.index("END")]
                start = tokens.index("SUBMIT") + 1
                submit = tokens[start:tokens.index("END", start)]
                for option in ["--owner-pid", "--owner-generation", "--map-generation"]:
                    self.assertEqual(register[register.index(option) + 1], submit[submit.index(option) + 1])
                expected = "|".join(["B15", "F2", nav_id or base["NJRH_MAP_ID"], epoch,
                                     "a" * 64, base["NAV2_MAP_YAML"], params or "/workspaces/overlay/config/amcl_shadow.yaml"])
                self.assertEqual(register[register.index("--map-generation") + 1], expected)

    def test_floor_resolver_exports_identity_inputs(self):
        text = function_text(SCRIPTS / "floor_asset_helpers.sh", "resolve_floor_assets")
        for name in ["NAV2_MAP_YAML", "NJRH_BUILDING_ID", "NJRH_FLOOR_ID", "NJRH_NAV_MAP_ID",
                     "NJRH_MAP_ID", "NJRH_MAP_ASSET_EPOCH", "NJRH_MAP_ASSET_DIGEST"]:
            self.assertIn("export " + name + "=", text)

    def test_no_shell_heartbeat_writer_or_ros_cli(self):
        path = SCRIPTS / "run_amcl_shadow_localization.sh"
        heartbeat = function_text(path, "heartbeat_amcl_runtime_status")
        self.assertNotIn("while", heartbeat)
        self.assertNotIn("sleep", heartbeat)
        self.assertIn("amcl_status_cli ping", heartbeat)
        writer = function_text(path, "write_amcl_runtime_status")
        self.assertNotIn("ros2 ", writer)
        self.assertNotIn("source ", writer.replace("Never source the public file", ""))
        self.assertIn("AMCL_PROGRESS_LIFECYCLE", writer)
        self.assertIn("amcl_status_cli submit", writer)

    def test_no_heartbeat_process_death_navigation_abort(self):
        path = SCRIPTS / "run_navigation_runtime_services.sh"
        text = path.read_text(encoding="utf-8")
        self.assertNotIn("amcl_status_heartbeat_pid", text)
        self.assertNotIn('source "${NJRH_AMCL_RUNTIME_STATUS_FILE}"', text)
        self.assertNotIn('rm -f "${NJRH_AMCL_RUNTIME_STATUS_FILE}"', text)
        self.assertIn('NJRH_AMCL_STATUS_OWNER_PID="${NJRH_STARTUP_OWNER_PID}"', text)


if __name__ == "__main__":
    unittest.main()
