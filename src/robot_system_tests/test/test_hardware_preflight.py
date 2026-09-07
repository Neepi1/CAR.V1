import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
PROVISION_ROOT = ROOT / "scripts" / "jetson" / "provision"
MODULE_PATH = PROVISION_ROOT / "hardware_preflight.py"


def load_module():
    spec = importlib.util.spec_from_file_location("hardware_preflight", MODULE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample_manifest() -> dict:
    return json.loads(
        (PROVISION_ROOT / "device-release.example.json").read_text(encoding="utf-8")
    )


def command_result(command, *, stdout="", stderr="", returncode=0, error=""):
    return {
        "command": list(command),
        "returncode": returncode,
        "stdout": stdout,
        "stderr": stderr,
        "error": error,
        "timed_out": error == "COMMAND_TIMEOUT",
    }


def named_check(result, name):
    return next(check for check in result["checks"] if check["name"] == name)


def test_missing_hardware_commands_are_structured_failures(monkeypatch):
    hardware = load_module()

    def command_missing(command, *, timeout):
        return {
            "command": list(command),
            "returncode": None,
            "stdout": "",
            "stderr": "",
            "error": "COMMAND_NOT_FOUND",
            "timed_out": False,
        }

    monkeypatch.setattr(hardware, "_run_command", command_missing)
    monkeypatch.setattr(
        hardware,
        "_read_network_carrier",
        lambda interface: (None, "SYSFS_UNAVAILABLE"),
    )
    monkeypatch.setattr(hardware, "_enumerate_orbbec_devices", lambda usb_id: [])
    monkeypatch.setattr(hardware, "_nvidia_device_facts", lambda: [])

    result = hardware.collect_hardware_checks(sample_manifest())

    assert result["ok"] is False
    assert result["schema"] == "njrh.hardware_preflight.v1"
    assert result["checks"]
    assert all(
        {"name", "ok", "code", "detail"}.issubset(check)
        for check in result["checks"]
    )
    assert any(
        check["detail"].get("error") == "COMMAND_NOT_FOUND"
        for check in result["checks"]
        if isinstance(check["detail"], dict)
    )


def test_jt128_requires_exact_ip_prefix_carrier_route_and_one_expected_udp_sender(
    monkeypatch,
):
    hardware = load_module()
    commands = []

    def healthy_commands(command, *, timeout):
        del timeout
        commands.append(list(command))
        key = tuple(command)
        outputs = {
            ("ip", "-j", "link", "show", "dev", "eth1"): (
                '[{"ifname":"eth1","flags":["UP","LOWER_UP"],'
                '"operstate":"UP","link_type":"ether"}]'
            ),
            ("ip", "-j", "-4", "addr", "show", "dev", "eth1"): (
                '[{"ifname":"eth1","addr_info":[{"family":"inet",'
                '"local":"192.168.1.100","prefixlen":24,"scope":"global"}]}]'
            ),
            ("ip", "-j", "route", "get", "192.168.1.201"): (
                '[{"dst":"192.168.1.201","dev":"eth1",'
                '"prefsrc":"192.168.1.100","flags":[]}]'
            ),
        }
        if key in outputs:
            return command_result(command, stdout=outputs[key])
        if command and command[0] == "tcpdump":
            return command_result(
                command,
                stdout=(
                    "20:00:00.000000 IP 192.168.1.201.2368 > "
                    "192.168.1.100.2368: UDP, length 1248\n"
                ),
            )
        return command_result(command, error="COMMAND_NOT_FOUND", returncode=None)

    monkeypatch.setattr(hardware, "_run_command", healthy_commands)
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (1, "")
    )
    monkeypatch.setattr(hardware, "_enumerate_orbbec_devices", lambda usb_id: [])
    monkeypatch.setattr(hardware, "_nvidia_device_facts", lambda: [])

    manifest = sample_manifest()
    manifest["hardware"]["orbbec"]["serial"] = "REPLACE_WITH_ORBBEC_SERIAL"
    result = hardware.collect_hardware_checks(manifest)

    assert named_check(result, "jt128_network")["ok"] is True
    assert named_check(result, "jt128_udp_2368")["ok"] is True
    tcpdump = next(command for command in commands if command[0] == "tcpdump")
    assert tcpdump[tcpdump.index("-c") + 1] == "1"
    assert int(tcpdump[tcpdump.index("-s") + 1]) <= 96
    assert "pointcloud" not in " ".join(tcpdump).lower()


def test_can_requires_physical_error_active_500k_and_one_passive_221_frame(
    monkeypatch,
):
    hardware = load_module()
    commands = []

    def can_commands(command, *, timeout):
        del timeout
        commands.append(list(command))
        if tuple(command) == (
            "ip",
            "-j",
            "-details",
            "link",
            "show",
            "dev",
            "can0",
        ):
            return command_result(
                command,
                stdout=(
                    '[{"ifname":"can0","flags":["UP","LOWER_UP","ECHO"],'
                    '"operstate":"UP","link_type":"can",'
                    '"linkinfo":{"info_kind":"can","info_data":{'
                    '"state":"ERROR-ACTIVE",'
                    '"bittiming":{"bitrate":500000}}},'
                    '"parentbus":"platform","parentdev":"c310000.mttcan"}]'
                ),
            )
        if command and command[0] == "candump":
            return command_result(
                command, stdout="(1753444800.000001) can0 221#0102030405060708\n"
            )
        return command_result(command, error="COMMAND_NOT_FOUND", returncode=None)

    monkeypatch.setattr(hardware, "_run_command", can_commands)
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (None, "")
    )
    monkeypatch.setattr(hardware, "_enumerate_orbbec_devices", lambda usb_id: [])
    monkeypatch.setattr(hardware, "_nvidia_device_facts", lambda: [])

    result = hardware.collect_hardware_checks(sample_manifest())

    assert named_check(result, "can_interface")["ok"] is True
    assert named_check(result, "can_feedback_0x221")["ok"] is True
    candump = next(command for command in commands if command[0] == "candump")
    assert candump[candump.index("-n") + 1] == "1"
    assert int(candump[candump.index("-T") + 1]) <= 2000
    assert "can0,221:C00007FF" in candump
    assert all(command[0] != "cansend" for command in commands)


def test_orbbec_requires_one_exact_serial_at_usb3_speed(monkeypatch, tmp_path):
    hardware = load_module()
    usb_root = tmp_path / "usb"
    device = usb_root / "2-2.3.1"
    device.mkdir(parents=True)
    (device / "idVendor").write_text("2bc5\n", encoding="utf-8")
    (device / "idProduct").write_text("0807\n", encoding="utf-8")
    (device / "serial").write_text("REPLACE_WITH_ORBBEC_SERIAL\n", encoding="utf-8")
    (device / "speed").write_text("5000\n", encoding="utf-8")

    monkeypatch.setattr(hardware, "USB_SYSFS_ROOT", usb_root)
    monkeypatch.setattr(
        hardware,
        "_run_command",
        lambda command, timeout: command_result(
            command, error="COMMAND_NOT_FOUND", returncode=None
        ),
    )
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (None, "")
    )
    monkeypatch.setattr(hardware, "_nvidia_device_facts", lambda: [])

    manifest = sample_manifest()
    manifest["hardware"]["orbbec"]["serial"] = "REPLACE_WITH_ORBBEC_SERIAL"
    result = hardware.collect_hardware_checks(manifest)

    check = named_check(result, "orbbec_identity_and_speed")
    assert check["ok"] is True
    assert result["facts"]["orbbec"]["devices"][0]["speed_mbps"] == 5000


def test_nvidia_requires_host_devices_registered_runtime_and_isolated_container_probe(
    monkeypatch,
):
    hardware = load_module()
    commands = []

    def nvidia_commands(command, *, timeout):
        del timeout
        commands.append(list(command))
        if command[:2] == ["docker", "info"]:
            return command_result(
                command,
                stdout=(
                    '{"runc":{"path":"runc"},'
                    '"nvidia":{"path":"nvidia-container-runtime"}}'
                ),
            )
        if command and command[0] == "nvidia-container-runtime":
            return command_result(
                command, stdout="NVIDIA Container Runtime version 1.17.8\n"
            )
        if command[:2] == ["docker", "run"]:
            return command_result(command, stdout="NVIDIA_DEVICE_OK\n")
        return command_result(command, error="COMMAND_NOT_FOUND", returncode=None)

    monkeypatch.setattr(hardware, "_run_command", nvidia_commands)
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (None, "")
    )
    monkeypatch.setattr(hardware, "_enumerate_orbbec_devices", lambda usb_id: [])
    monkeypatch.setattr(
        hardware,
        "_nvidia_device_facts",
        lambda: [
            {"path": "/dev/nvhost-as-gpu", "exists": True, "char_device": True},
            {"path": "/dev/nvhost-ctrl-gpu", "exists": True, "char_device": True},
        ],
    )

    result = hardware.collect_hardware_checks(sample_manifest())

    assert named_check(result, "nvidia_devices")["ok"] is True
    assert named_check(result, "nvidia_container_runtime")["ok"] is True
    probe = next(command for command in commands if command[:2] == ["docker", "run"])
    assert "--rm" in probe
    assert probe[probe.index("--pull") + 1] == "never"
    assert probe[probe.index("--network") + 1] == "none"
    assert probe[probe.index("--runtime") + 1] == "nvidia"
    assert sample_manifest()["artifacts"]["runtime_image"]["reference"] in probe


@pytest.mark.parametrize(
    ("prefixlen", "carrier", "route_device"),
    [
        (16, 1, "eth1"),
        (24, 0, "eth1"),
        (24, 1, "eth10"),
    ],
)
def test_jt128_rejects_wrong_prefix_no_carrier_and_interface_substrings(
    monkeypatch, prefixlen, carrier, route_device
):
    hardware = load_module()

    def commands(command, *, timeout):
        del timeout
        key = tuple(command)
        if key == ("ip", "-j", "link", "show", "dev", "eth1"):
            return command_result(
                command,
                stdout=(
                    '[{"ifname":"eth1","flags":["UP","LOWER_UP"],'
                    '"operstate":"UP","link_type":"ether"}]'
                ),
            )
        if key == ("ip", "-j", "-4", "addr", "show", "dev", "eth1"):
            return command_result(
                command,
                stdout=(
                    '[{"ifname":"eth1","addr_info":[{"family":"inet",'
                    f'"local":"192.168.1.100","prefixlen":{prefixlen},'
                    '"scope":"global"}]}]'
                ),
            )
        if key == ("ip", "-j", "route", "get", "192.168.1.201"):
            return command_result(
                command,
                stdout=(
                    f'[{{"dst":"192.168.1.201","dev":"{route_device}",'
                    '"prefsrc":"192.168.1.100","flags":[]}]'
                ),
            )
        if command and command[0] == "tcpdump":
            return command_result(
                command,
                stdout=(
                    "IP 192.168.1.201.2368 > "
                    "192.168.1.100.2368: UDP, length 1248\n"
                ),
            )
        return command_result(command, error="COMMAND_NOT_FOUND", returncode=None)

    monkeypatch.setattr(hardware, "_run_command", commands)
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (carrier, "")
    )
    monkeypatch.setattr(hardware, "_enumerate_orbbec_devices", lambda usb_id: [])
    monkeypatch.setattr(hardware, "_nvidia_device_facts", lambda: [])

    result = hardware.collect_hardware_checks(sample_manifest())

    assert named_check(result, "jt128_network")["ok"] is False


def test_jt128_rejects_udp_packet_from_wrong_sender(monkeypatch):
    hardware = load_module()

    def commands(command, *, timeout):
        del timeout
        key = tuple(command)
        outputs = {
            ("ip", "-j", "link", "show", "dev", "eth1"): (
                '[{"ifname":"eth1","flags":["UP","LOWER_UP"],'
                '"operstate":"UP","link_type":"ether"}]'
            ),
            ("ip", "-j", "-4", "addr", "show", "dev", "eth1"): (
                '[{"ifname":"eth1","addr_info":[{"family":"inet",'
                '"local":"192.168.1.100","prefixlen":24,"scope":"global"}]}]'
            ),
            ("ip", "-j", "route", "get", "192.168.1.201"): (
                '[{"dst":"192.168.1.201","dev":"eth1",'
                '"prefsrc":"192.168.1.100","flags":[]}]'
            ),
        }
        if key in outputs:
            return command_result(command, stdout=outputs[key])
        if command and command[0] == "tcpdump":
            return command_result(
                command,
                stdout=(
                    "IP 192.168.1.202.2368 > "
                    "192.168.1.100.2368: UDP, length 1248\n"
                ),
            )
        return command_result(command, error="COMMAND_NOT_FOUND", returncode=None)

    monkeypatch.setattr(hardware, "_run_command", commands)
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (1, "")
    )
    monkeypatch.setattr(hardware, "_enumerate_orbbec_devices", lambda usb_id: [])
    monkeypatch.setattr(hardware, "_nvidia_device_facts", lambda: [])

    result = hardware.collect_hardware_checks(sample_manifest())

    assert named_check(result, "jt128_network")["ok"] is True
    assert named_check(result, "jt128_udp_2368")["ok"] is False


@pytest.mark.parametrize(
    ("info_kind", "state", "bitrate"),
    [
        ("vcan", "ERROR-ACTIVE", 500000),
        ("can", "BUS-OFF", 500000),
        ("can", "ERROR-ACTIVE", 250000),
    ],
)
def test_can_rejects_vcan_bus_off_and_wrong_bitrate(
    monkeypatch, info_kind, state, bitrate
):
    hardware = load_module()

    def commands(command, *, timeout):
        del timeout
        if tuple(command) == (
            "ip",
            "-j",
            "-details",
            "link",
            "show",
            "dev",
            "can0",
        ):
            return command_result(
                command,
                stdout=json.dumps(
                    [
                        {
                            "ifname": "can0",
                            "flags": ["UP", "LOWER_UP"],
                            "operstate": "UP",
                            "link_type": "can",
                            "linkinfo": {
                                "info_kind": info_kind,
                                "info_data": {
                                    "state": state,
                                    "bittiming": {"bitrate": bitrate},
                                },
                            },
                        }
                    ]
                ),
            )
        if command and command[0] == "candump":
            return command_result(command, stdout="can0 221#0102\n")
        return command_result(command, error="COMMAND_NOT_FOUND", returncode=None)

    monkeypatch.setattr(hardware, "_run_command", commands)
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (None, "")
    )
    monkeypatch.setattr(hardware, "_enumerate_orbbec_devices", lambda usb_id: [])
    monkeypatch.setattr(hardware, "_nvidia_device_facts", lambda: [])

    result = hardware.collect_hardware_checks(sample_manifest())

    assert named_check(result, "can_interface")["ok"] is False


def test_missing_candump_fails_closed_without_exception(monkeypatch):
    hardware = load_module()

    def commands(command, *, timeout):
        del timeout
        if tuple(command) == (
            "ip",
            "-j",
            "-details",
            "link",
            "show",
            "dev",
            "can0",
        ):
            return command_result(
                command,
                stdout=(
                    '[{"ifname":"can0","flags":["UP","LOWER_UP"],'
                    '"operstate":"UP","link_type":"can",'
                    '"linkinfo":{"info_kind":"can","info_data":{'
                    '"state":"ERROR-ACTIVE",'
                    '"bittiming":{"bitrate":500000}}}}]'
                ),
            )
        return command_result(command, error="COMMAND_NOT_FOUND", returncode=None)

    monkeypatch.setattr(hardware, "_run_command", commands)
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (None, "")
    )
    monkeypatch.setattr(hardware, "_enumerate_orbbec_devices", lambda usb_id: [])
    monkeypatch.setattr(hardware, "_nvidia_device_facts", lambda: [])

    result = hardware.collect_hardware_checks(sample_manifest())

    check = named_check(result, "can_feedback_0x221")
    assert check["ok"] is False
    assert check["detail"]["error"] == "COMMAND_NOT_FOUND"


@pytest.mark.parametrize(
    ("devices", "expected_error"),
    [
        (
            [("2-2.3.1", "REPLACE_WITH_ORBBEC_SERIAL", "480")],
            "USB_SPEED_BELOW_5000_MBPS",
        ),
        (
            [
                ("2-2.3.1", "REPLACE_WITH_ORBBEC_SERIAL", "5000"),
                ("2-2.3.2", "REPLACE_WITH_ORBBEC_SERIAL", "5000"),
            ],
            "USB_DEVICE_COUNT_MISMATCH",
        ),
        (
            [("2-2.3.1", "ANOTHER_CAMERA", "5000")],
            "SERIAL_MISMATCH",
        ),
    ],
)
def test_orbbec_rejects_usb2_duplicate_and_wrong_serial(
    monkeypatch, tmp_path, devices, expected_error
):
    hardware = load_module()
    manifest = sample_manifest()
    manifest["hardware"]["orbbec"]["serial"] = "REPLACE_WITH_ORBBEC_SERIAL"
    usb_root = tmp_path / "usb"
    for name, serial, speed in devices:
        device = usb_root / name
        device.mkdir(parents=True)
        (device / "idVendor").write_text("2bc5\n", encoding="utf-8")
        (device / "idProduct").write_text("0807\n", encoding="utf-8")
        (device / "serial").write_text(f"{serial}\n", encoding="utf-8")
        (device / "speed").write_text(f"{speed}\n", encoding="utf-8")

    monkeypatch.setattr(hardware, "USB_SYSFS_ROOT", usb_root)
    monkeypatch.setattr(
        hardware,
        "_run_command",
        lambda command, timeout: command_result(
            command, error="COMMAND_NOT_FOUND", returncode=None
        ),
    )
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (None, "")
    )
    monkeypatch.setattr(hardware, "_nvidia_device_facts", lambda: [])

    result = hardware.collect_hardware_checks(manifest)

    check = named_check(result, "orbbec_identity_and_speed")
    assert check["ok"] is False
    assert check["detail"]["error"] == expected_error


def test_nvidia_advertised_runtime_still_fails_when_container_probe_fails(
    monkeypatch,
):
    hardware = load_module()

    def commands(command, *, timeout):
        del timeout
        if command[:2] == ["docker", "info"]:
            return command_result(
                command,
                stdout='{"nvidia":{"path":"nvidia-container-runtime"}}',
            )
        if command and command[0] == "nvidia-container-runtime":
            return command_result(command, stdout="version 1.17.8\n")
        if command[:2] == ["docker", "run"]:
            return command_result(
                command,
                stderr="nvidia runtime hook failed",
                returncode=125,
                error="COMMAND_FAILED",
            )
        return command_result(command, error="COMMAND_NOT_FOUND", returncode=None)

    monkeypatch.setattr(hardware, "_run_command", commands)
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (None, "")
    )
    monkeypatch.setattr(hardware, "_enumerate_orbbec_devices", lambda usb_id: [])
    monkeypatch.setattr(
        hardware,
        "_nvidia_device_facts",
        lambda: [
            {"path": "/dev/nvhost-as-gpu", "exists": True, "char_device": True},
            {"path": "/dev/nvhost-ctrl-gpu", "exists": True, "char_device": True},
        ],
    )

    result = hardware.collect_hardware_checks(sample_manifest())

    check = named_check(result, "nvidia_container_runtime")
    assert check["ok"] is False
    assert check["detail"]["error"] == "COMMAND_FAILED"


def test_real_command_adapter_turns_file_not_found_into_structured_results(
    monkeypatch,
):
    hardware = load_module()

    def missing_binary(*args, **kwargs):
        del args, kwargs
        raise FileNotFoundError("not installed")

    monkeypatch.setattr(hardware.subprocess, "run", missing_binary)
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (None, "")
    )
    monkeypatch.setattr(hardware, "_enumerate_orbbec_devices", lambda usb_id: [])
    monkeypatch.setattr(hardware, "_nvidia_device_facts", lambda: [])

    result = hardware.collect_hardware_checks(sample_manifest())

    assert result["ok"] is False
    assert named_check(result, "jt128_network")["detail"]["error"] == (
        "COMMAND_NOT_FOUND"
    )
    assert named_check(result, "can_feedback_0x221")["detail"]["error"] == (
        "COMMAND_NOT_FOUND"
    )
    assert named_check(result, "nvidia_container_runtime")["detail"]["error"] == (
        "COMMAND_NOT_FOUND"
    )


def test_timeout_with_partial_bytes_remains_json_serializable(monkeypatch):
    hardware = load_module()

    def timed_out(command, **kwargs):
        raise hardware.subprocess.TimeoutExpired(
            command,
            kwargs["timeout"],
            output=b"partial stdout",
            stderr=b"partial stderr",
        )

    monkeypatch.setattr(hardware.subprocess, "run", timed_out)
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (None, "")
    )
    monkeypatch.setattr(hardware, "_enumerate_orbbec_devices", lambda usb_id: [])
    monkeypatch.setattr(hardware, "_nvidia_device_facts", lambda: [])

    result = hardware.collect_hardware_checks(sample_manifest())

    assert result["ok"] is False
    assert "partial stdout" in json.dumps(result)
    assert named_check(result, "jt128_network")["detail"]["error"] == (
        "COMMAND_TIMEOUT"
    )


def test_complete_golden_hardware_evidence_returns_ok(monkeypatch):
    hardware = load_module()

    def commands(command, *, timeout):
        del timeout
        key = tuple(command)
        outputs = {
            ("ip", "-j", "link", "show", "dev", "eth1"): (
                '[{"ifname":"eth1","flags":["UP","LOWER_UP"],'
                '"operstate":"UP","link_type":"ether"}]'
            ),
            ("ip", "-j", "-4", "addr", "show", "dev", "eth1"): (
                '[{"ifname":"eth1","addr_info":[{"family":"inet",'
                '"local":"192.168.1.100","prefixlen":24,"scope":"global"}]}]'
            ),
            ("ip", "-j", "route", "get", "192.168.1.201"): (
                '[{"dst":"192.168.1.201","dev":"eth1",'
                '"prefsrc":"192.168.1.100","flags":[]}]'
            ),
            (
                "ip",
                "-j",
                "-details",
                "link",
                "show",
                "dev",
                "can0",
            ): (
                '[{"ifname":"can0","flags":["UP","LOWER_UP"],'
                '"operstate":"UP","link_type":"can",'
                '"linkinfo":{"info_kind":"can","info_data":{'
                '"state":"ERROR-ACTIVE",'
                '"bittiming":{"bitrate":500000}}}}]'
            ),
        }
        if key in outputs:
            return command_result(command, stdout=outputs[key])
        if command and command[0] == "tcpdump":
            return command_result(
                command,
                stdout=(
                    "IP 192.168.1.201.2368 > "
                    "192.168.1.255.2368: UDP, length 1248\n"
                ),
            )
        if command and command[0] == "candump":
            return command_result(command, stdout="can0 221#0102\n")
        if command[:2] == ["docker", "info"]:
            return command_result(
                command,
                stdout='{"nvidia":{"path":"nvidia-container-runtime"}}',
            )
        if command and command[0] == "nvidia-container-runtime":
            return command_result(command, stdout="version 1.16.2\n")
        if command[:2] == ["docker", "run"]:
            return command_result(command, stdout="NVIDIA_DEVICE_OK\n")
        return command_result(command, error="COMMAND_NOT_FOUND", returncode=None)

    monkeypatch.setattr(hardware, "_run_command", commands)
    monkeypatch.setattr(
        hardware, "_read_network_carrier", lambda interface: (1, "")
    )
    monkeypatch.setattr(
        hardware,
        "_enumerate_orbbec_devices",
        lambda usb_id: [
            {
                "path": "/sys/bus/usb/devices/2-2.3.1",
                "usb_id": usb_id,
                "serial": "REPLACE_WITH_ORBBEC_SERIAL",
                "speed_mbps": 5000,
            }
        ],
    )
    monkeypatch.setattr(
        hardware,
        "_nvidia_device_facts",
        lambda: [
            {"path": "/dev/nvhost-as-gpu", "exists": True, "char_device": True},
            {"path": "/dev/nvhost-ctrl-gpu", "exists": True, "char_device": True},
        ],
    )

    result = hardware.collect_hardware_checks(sample_manifest())

    assert result["ok"] is True
    assert result["failed_codes"] == []
    assert {check["name"] for check in result["checks"]} == {
        "jt128_network",
        "jt128_udp_2368",
        "can_interface",
        "can_feedback_0x221",
        "orbbec_identity_and_speed",
        "nvidia_devices",
        "nvidia_container_runtime",
    }


def test_orbbec_auto_enroll_accepts_one_usb3_device(monkeypatch):
    hardware = load_module()
    manifest = sample_manifest()
    manifest["hardware"]["orbbec"]["serial"] = "AUTO_ENROLL"
    monkeypatch.setattr(
        hardware,
        "_enumerate_orbbec_devices",
        lambda usb_id: [
            {
                "path": "/sys/mock/1",
                "usb_id": usb_id,
                "serial": "FACTORY-CAMERA-001",
                "speed_mbps": 5000,
            }
        ],
    )

    checks, facts = hardware._collect_orbbec(manifest)

    assert checks[0]["ok"] is True
    assert facts["devices"][0]["serial"] == "FACTORY-CAMERA-001"


def test_orbbec_auto_enroll_rejects_device_without_serial(monkeypatch):
    hardware = load_module()
    manifest = sample_manifest()
    manifest["hardware"]["orbbec"]["serial"] = "AUTO_ENROLL"
    monkeypatch.setattr(
        hardware,
        "_enumerate_orbbec_devices",
        lambda usb_id: [
            {
                "path": "/sys/mock/1",
                "usb_id": usb_id,
                "serial": "",
                "speed_mbps": 5000,
            }
        ],
    )

    checks, _ = hardware._collect_orbbec(manifest)

    assert checks[0]["ok"] is False
    assert checks[0]["detail"]["error"] == "SERIAL_UNAVAILABLE"


def test_orbbec_cannot_disable_required_hardware_baseline(monkeypatch):
    hardware = load_module()
    manifest = sample_manifest()
    manifest["hardware"]["orbbec"]["required"] = False
    monkeypatch.setattr(
        hardware,
        "_enumerate_orbbec_devices",
        lambda usb_id: [
            {
                "path": "/sys/mock/1",
                "usb_id": usb_id,
                "serial": "FACTORY-CAMERA-001",
                "speed_mbps": 480,
            }
        ],
    )

    checks, _ = hardware._collect_orbbec(manifest)

    assert checks[0]["ok"] is False
    assert checks[0]["detail"]["error"] == "INVALID_MANIFEST"
