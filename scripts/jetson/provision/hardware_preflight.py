#!/usr/bin/env python3
"""Read-only, fail-closed hardware checks for NJRH Jetson deployments.

The public entry point returns JSON-serializable data and never raises merely
because a host command is absent.  It does not configure interfaces, transmit
CAN frames, start ROS nodes, or subscribe to point clouds.
"""

from __future__ import annotations

import ipaddress
import json
import re
import stat
import subprocess
from pathlib import Path
from typing import Any, Iterable


SCHEMA = "njrh.hardware_preflight.v1"
USB_SYSFS_ROOT = Path("/sys/bus/usb/devices")
NET_SYSFS_ROOT = Path("/sys/class/net")
COMMAND_TIMEOUT_SECONDS = 2.0
PACKET_SNAPSHOT_BYTES = 96
NVIDIA_RUNTIME_TIMEOUT_SECONDS = 10.0
NVIDIA_REQUIRED_DEVICES = (
    Path("/dev/nvhost-as-gpu"),
    Path("/dev/nvhost-ctrl-gpu"),
)


def _coerce_text(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _run_command(
    command: Iterable[str], *, timeout: float
) -> dict[str, Any]:
    rendered = list(command)
    try:
        result = subprocess.run(
            rendered,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
        )
    except FileNotFoundError:
        return {
            "command": rendered,
            "returncode": None,
            "stdout": "",
            "stderr": "",
            "error": "COMMAND_NOT_FOUND",
            "timed_out": False,
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "command": rendered,
            "returncode": None,
            "stdout": _coerce_text(exc.stdout),
            "stderr": _coerce_text(exc.stderr),
            "error": "COMMAND_TIMEOUT",
            "timed_out": True,
        }
    except OSError as exc:
        return {
            "command": rendered,
            "returncode": None,
            "stdout": "",
            "stderr": str(exc),
            "error": "COMMAND_OS_ERROR",
            "timed_out": False,
        }
    return {
        "command": rendered,
        "returncode": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
        "error": "" if result.returncode == 0 else "COMMAND_FAILED",
        "timed_out": False,
    }


def _read_network_carrier(interface: str) -> tuple[int | None, str]:
    try:
        value = (NET_SYSFS_ROOT / interface / "carrier").read_text(
            encoding="utf-8"
        ).strip()
        return int(value), ""
    except (OSError, ValueError) as exc:
        return None, f"SYSFS_UNAVAILABLE:{exc}"


def _enumerate_orbbec_devices(usb_id: str) -> list[dict[str, Any]]:
    try:
        expected_vendor, expected_product = usb_id.lower().split(":", 1)
    except ValueError:
        return []
    if not USB_SYSFS_ROOT.is_dir():
        return []

    devices: list[dict[str, Any]] = []
    try:
        candidates = sorted(USB_SYSFS_ROOT.iterdir(), key=lambda item: item.name)
    except OSError:
        return []
    for candidate in candidates:
        try:
            vendor = (
                (candidate / "idVendor")
                .read_text(encoding="utf-8")
                .strip()
                .lower()
            )
            product = (
                (candidate / "idProduct").read_text(encoding="utf-8").strip().lower()
            )
        except OSError:
            continue
        if vendor != expected_vendor or product != expected_product:
            continue
        try:
            serial = (candidate / "serial").read_text(encoding="utf-8").strip()
        except OSError:
            serial = ""
        try:
            speed_text = (candidate / "speed").read_text(encoding="utf-8").strip()
            speed_mbps: int | None = int(float(speed_text))
        except (OSError, ValueError):
            speed_mbps = None
        devices.append(
            {
                "path": str(candidate),
                "usb_id": f"{vendor}:{product}",
                "serial": serial,
                "speed_mbps": speed_mbps,
            }
        )
    return devices


def _nvidia_device_facts() -> list[dict[str, Any]]:
    facts: list[dict[str, Any]] = []
    for path in NVIDIA_REQUIRED_DEVICES:
        exists = path.exists()
        char_device = False
        if exists:
            try:
                char_device = stat.S_ISCHR(path.stat().st_mode)
            except OSError:
                char_device = False
        facts.append(
            {
                "path": str(path),
                "exists": exists,
                "char_device": char_device,
            }
        )
    return facts


def _check(
    name: str, ok: bool, code: str, detail: Any
) -> dict[str, Any]:
    return {"name": name, "ok": bool(ok), "code": code, "detail": detail}


def _command_detail(result: dict[str, Any]) -> dict[str, Any]:
    return {
        "command": result["command"],
        "returncode": result["returncode"],
        "error": result["error"],
        "stderr": str(result["stderr"]).strip()[-1000:],
    }


def _run_json_command(command: list[str]) -> tuple[Any | None, dict[str, Any]]:
    result = _run_command(command, timeout=COMMAND_TIMEOUT_SECONDS)
    if result["error"]:
        return None, result
    try:
        return json.loads(result["stdout"]), result
    except (TypeError, json.JSONDecodeError) as exc:
        result = dict(result)
        result["error"] = "INVALID_JSON_OUTPUT"
        result["stderr"] = str(exc)
        return None, result


def _collect_jt128(
    manifest: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    facts: dict[str, Any] = {}
    try:
        jt128 = manifest["hardware"]["jt128"]
        interface = str(jt128["interface"])
        host_interface = ipaddress.ip_interface(str(jt128["host_cidr"]))
        device_ip = ipaddress.ip_address(str(jt128["device_ip"]))
        if (
            host_interface.version != 4
            or device_ip.version != 4
            or not interface
        ):
            raise ValueError("JT128 requires a non-empty interface and IPv4 addresses")
    except (KeyError, TypeError, ValueError) as exc:
        detail = {"error": "INVALID_MANIFEST", "message": str(exc)}
        return [
            _check("jt128_network", False, "JT128_NETWORK_INVALID", detail),
            _check("jt128_udp_2368", False, "JT128_UDP_INVALID", detail),
        ], facts

    link, link_result = _run_json_command(
        ["ip", "-j", "link", "show", "dev", interface]
    )
    addresses, address_result = _run_json_command(
        ["ip", "-j", "-4", "addr", "show", "dev", interface]
    )
    routes, route_result = _run_json_command(
        ["ip", "-j", "route", "get", str(device_ip)]
    )
    carrier, carrier_error = _read_network_carrier(interface)

    link_entry = (
        link[0]
        if isinstance(link, list) and len(link) == 1 and isinstance(link[0], dict)
        else {}
    )
    flags = set(link_entry.get("flags", []))
    link_ok = (
        link_entry.get("ifname") == interface
        and link_entry.get("link_type") == "ether"
        and link_entry.get("operstate") == "UP"
        and {"UP", "LOWER_UP"}.issubset(flags)
        and carrier == 1
    )

    address_interface_entry = (
        addresses[0]
        if isinstance(addresses, list)
        and len(addresses) == 1
        and isinstance(addresses[0], dict)
        and addresses[0].get("ifname") == interface
        else {}
    )
    address_entries = [
        entry
        for entry in address_interface_entry.get("addr_info", [])
        if isinstance(entry, dict)
    ]
    address_ok = any(
        address_interface_entry.get("ifname") == interface
        and entry.get("family") == "inet"
        and entry.get("local") == str(host_interface.ip)
        and entry.get("prefixlen") == host_interface.network.prefixlen
        and entry.get("scope") == "global"
        for entry in address_entries
    )

    route_entry = (
        routes[0]
        if isinstance(routes, list)
        and len(routes) == 1
        and isinstance(routes[0], dict)
        else {}
    )
    route_ok = (
        route_entry.get("dst") == str(device_ip)
        and route_entry.get("dev") == interface
        and route_entry.get("prefsrc") == str(host_interface.ip)
        and not route_entry.get("gateway")
    )
    network_ok = (
        not link_result["error"]
        and not address_result["error"]
        and not route_result["error"]
        and link_ok
        and address_ok
        and route_ok
    )
    network_detail = {
        "error": next(
            (
                result["error"]
                for result in (link_result, address_result, route_result)
                if result["error"]
            ),
            carrier_error,
        ),
        "expected": {
            "interface": interface,
            "host_cidr": str(host_interface),
            "device_ip": str(device_ip),
        },
        "observed": {
            "ifname": link_entry.get("ifname"),
            "link_type": link_entry.get("link_type"),
            "operstate": link_entry.get("operstate"),
            "flags": sorted(flags),
            "carrier": carrier,
            "addresses": address_entries,
            "route": route_entry,
        },
        "errors": {
            "link": _command_detail(link_result),
            "address": _command_detail(address_result),
            "route": _command_detail(route_result),
            "carrier": carrier_error,
        },
    }
    facts["network"] = network_detail["observed"]
    checks.append(
        _check(
            "jt128_network",
            network_ok,
            "JT128_NETWORK_INVALID",
            network_detail,
        )
    )

    capture_command = [
        "tcpdump",
        "-i",
        interface,
        "-nn",
        "-Q",
        "in",
        "-c",
        "1",
        "-s",
        str(PACKET_SNAPSHOT_BYTES),
        "udp",
        "dst",
        "port",
        "2368",
    ]
    capture = _run_command(capture_command, timeout=COMMAND_TIMEOUT_SECONDS)
    packet_text = f"{capture['stdout']}\n{capture['stderr']}"
    packet_match = re.search(
        r"\bIP\s+"
        r"(?P<src>\d{1,3}(?:\.\d{1,3}){3})\.(?P<src_port>\d+)\s+>\s+"
        r"(?P<dst>\d{1,3}(?:\.\d{1,3}){3})\.(?P<dst_port>\d+):",
        packet_text,
    )
    packet: dict[str, Any] = {}
    if packet_match:
        packet = {
            "source_ip": packet_match.group("src"),
            "source_port": int(packet_match.group("src_port")),
            "destination_ip": packet_match.group("dst"),
            "destination_port": int(packet_match.group("dst_port")),
        }
    udp_ok = (
        not capture["error"]
        and packet.get("source_ip") == str(device_ip)
        and packet.get("destination_port") == 2368
    )
    udp_detail = {
        "error": capture["error"],
        "expected_sender": str(device_ip),
        "expected_destination_port": 2368,
        "packet": packet,
        "capture": _command_detail(capture),
        "snaplen_bytes": PACKET_SNAPSHOT_BYTES,
        "packet_limit": 1,
    }
    facts["udp_2368"] = packet
    checks.append(
        _check("jt128_udp_2368", udp_ok, "JT128_UDP_INVALID", udp_detail)
    )
    return checks, facts


def _collect_can(
    manifest: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    facts: dict[str, Any] = {}
    try:
        can = manifest["hardware"]["can"]
        interface = str(can["interface"])
        bitrate = can["bitrate"]
        if (
            not interface
            or isinstance(bitrate, bool)
            or not isinstance(bitrate, int)
            or bitrate <= 0
        ):
            raise ValueError("CAN interface and positive integer bitrate are required")
    except (KeyError, TypeError, ValueError) as exc:
        detail = {"error": "INVALID_MANIFEST", "message": str(exc)}
        return [
            _check("can_interface", False, "CAN_INTERFACE_INVALID", detail),
            _check("can_feedback_0x221", False, "CAN_FEEDBACK_INVALID", detail),
        ], facts

    link, link_result = _run_json_command(
        ["ip", "-j", "-details", "link", "show", "dev", interface]
    )
    link_entry = (
        link[0]
        if isinstance(link, list) and len(link) == 1 and isinstance(link[0], dict)
        else {}
    )
    linkinfo = (
        link_entry.get("linkinfo", {})
        if isinstance(link_entry.get("linkinfo", {}), dict)
        else {}
    )
    info_data = (
        linkinfo.get("info_data", {})
        if isinstance(linkinfo.get("info_data", {}), dict)
        else {}
    )
    bittiming = (
        info_data.get("bittiming", {})
        if isinstance(info_data.get("bittiming", {}), dict)
        else {}
    )
    flags = set(link_entry.get("flags", []))
    interface_ok = (
        not link_result["error"]
        and link_entry.get("ifname") == interface
        and link_entry.get("link_type") == "can"
        and linkinfo.get("info_kind") == "can"
        and linkinfo.get("info_kind") != "vcan"
        and link_entry.get("operstate") == "UP"
        and {"UP", "LOWER_UP"}.issubset(flags)
        and info_data.get("state") == "ERROR-ACTIVE"
        and bittiming.get("bitrate") == bitrate
    )
    interface_detail = {
        "error": link_result["error"],
        "expected": {
            "interface": interface,
            "bitrate": bitrate,
            "state": "ERROR-ACTIVE",
            "info_kind": "can",
        },
        "observed": {
            "ifname": link_entry.get("ifname"),
            "link_type": link_entry.get("link_type"),
            "info_kind": linkinfo.get("info_kind"),
            "operstate": link_entry.get("operstate"),
            "flags": sorted(flags),
            "state": info_data.get("state"),
            "bitrate": bittiming.get("bitrate"),
            "berr_counter": info_data.get("berr_counter"),
            "parentbus": link_entry.get("parentbus"),
            "parentdev": link_entry.get("parentdev"),
        },
        "command": _command_detail(link_result),
    }
    facts["interface"] = interface_detail["observed"]
    checks.append(
        _check(
            "can_interface",
            interface_ok,
            "CAN_INTERFACE_INVALID",
            interface_detail,
        )
    )

    capture_command = [
        "candump",
        "-L",
        "-n",
        "1",
        "-T",
        "1500",
        f"{interface},221:C00007FF",
    ]
    capture = _run_command(capture_command, timeout=COMMAND_TIMEOUT_SECONDS)
    frame_text = f"{capture['stdout']}\n{capture['stderr']}"
    frame_match = re.search(
        rf"(?:^|\s){re.escape(interface)}\s+"
        r"(?P<can_id>[0-9A-Fa-f]+)(?:#|\s+\[)",
        frame_text,
    )
    observed_id = (
        int(frame_match.group("can_id"), 16) if frame_match is not None else None
    )
    feedback_ok = not capture["error"] and observed_id == 0x221
    feedback_detail = {
        "error": capture["error"],
        "expected_can_id": "0x221",
        "observed_can_id": (
            f"0x{observed_id:X}" if observed_id is not None else None
        ),
        "capture": _command_detail(capture),
        "frame_limit": 1,
        "passive_only": True,
    }
    facts["feedback_0x221"] = {
        "observed_can_id": feedback_detail["observed_can_id"]
    }
    checks.append(
        _check(
            "can_feedback_0x221",
            feedback_ok,
            "CAN_FEEDBACK_INVALID",
            feedback_detail,
        )
    )
    return checks, facts


def _collect_orbbec(
    manifest: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    try:
        orbbec = manifest["hardware"]["orbbec"]
        usb_id = str(orbbec["usb_id"]).lower()
        expected_serial = str(orbbec["serial"])
        required = orbbec["required"]
        minimum_speed = int(orbbec.get("minimum_usb_speed_mbps", 5000))
        if (
            not usb_id
            or not expected_serial
            or required is not True
            or minimum_speed < 5000
        ):
            raise ValueError(
                "Orbbec usb_id and serial are required, required must be true, "
                "and minimum USB speed must be at least 5000 Mbps"
            )
    except (KeyError, TypeError, ValueError) as exc:
        detail = {"error": "INVALID_MANIFEST", "message": str(exc)}
        return [
            _check(
                "orbbec_identity_and_speed",
                False,
                "ORBBEC_INVALID",
                detail,
            )
        ], {}

    devices = _enumerate_orbbec_devices(usb_id)
    auto_enroll = expected_serial == "AUTO_ENROLL"
    exact_devices = [
        item
        for item in devices
        if auto_enroll or item.get("serial") == expected_serial
    ]
    if len(devices) != 1:
        ok = False
        error = "USB_DEVICE_COUNT_MISMATCH"
    elif len(exact_devices) != 1:
        ok = False
        error = "SERIAL_MISMATCH"
    elif (
        not isinstance(exact_devices[0].get("serial"), str)
        or not exact_devices[0]["serial"].strip()
    ):
        ok = False
        error = "SERIAL_UNAVAILABLE"
    elif not isinstance(exact_devices[0].get("speed_mbps"), int):
        ok = False
        error = "USB_SPEED_UNAVAILABLE"
    elif exact_devices[0]["speed_mbps"] < minimum_speed:
        ok = False
        error = f"USB_SPEED_BELOW_{minimum_speed}_MBPS"
    else:
        ok = True
        error = ""
    detail = {
        "error": error,
        "required": required,
        "expected": {
            "usb_id": usb_id,
            "serial": expected_serial,
            "minimum_speed_mbps": minimum_speed,
            "device_count": 1,
        },
        "devices": devices,
    }
    return [
        _check(
            "orbbec_identity_and_speed",
            ok,
            "ORBBEC_INVALID",
            detail,
        )
    ], {"devices": devices}


def _collect_nvidia(
    manifest: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    facts: dict[str, Any] = {}

    devices = _nvidia_device_facts()
    devices_ok = bool(devices) and all(
        item.get("exists") is True and item.get("char_device") is True
        for item in devices
    )
    checks.append(
        _check(
            "nvidia_devices",
            devices_ok,
            "NVIDIA_DEVICE_INVALID",
            {
                "error": "" if devices_ok else "NVIDIA_DEVICE_MISSING_OR_NOT_CHAR",
                "devices": devices,
            },
        )
    )
    facts["devices"] = devices

    docker_info = _run_command(
        ["docker", "info", "--format", "{{json .Runtimes}}"],
        timeout=COMMAND_TIMEOUT_SECONDS,
    )
    runtimes: dict[str, Any] = {}
    if not docker_info["error"]:
        try:
            parsed_runtimes = json.loads(docker_info["stdout"])
            if isinstance(parsed_runtimes, dict):
                runtimes = parsed_runtimes
        except (TypeError, json.JSONDecodeError):
            pass
    runtime_registered = "nvidia" in runtimes

    runtime_version = _run_command(
        ["nvidia-container-runtime", "--version"],
        timeout=COMMAND_TIMEOUT_SECONDS,
    )
    runtime_binary_ok = (
        not runtime_version["error"] and bool(runtime_version["stdout"].strip())
    )

    try:
        image_reference = str(
            manifest["artifacts"]["runtime_image"]["reference"]
        )
        if not image_reference:
            raise ValueError("runtime image reference is empty")
        image_error = ""
    except (KeyError, TypeError, ValueError) as exc:
        image_reference = ""
        image_error = f"INVALID_MANIFEST:{exc}"

    if image_reference:
        runtime_probe = _run_command(
            [
                "docker",
                "run",
                "--rm",
                "--pull",
                "never",
                "--network",
                "none",
                "--runtime",
                "nvidia",
                "--entrypoint",
                "/bin/sh",
                image_reference,
                "-c",
                (
                    "if [ -c /dev/nvhost-as-gpu ] "
                    "&& [ -c /dev/nvhost-ctrl-gpu ]; then "
                    "printf NVIDIA_DEVICE_OK; else exit 42; fi"
                ),
            ],
            timeout=NVIDIA_RUNTIME_TIMEOUT_SECONDS,
        )
    else:
        runtime_probe = {
            "command": [],
            "returncode": None,
            "stdout": "",
            "stderr": "",
            "error": image_error,
            "timed_out": False,
        }
    runtime_probe_ok = (
        not runtime_probe["error"]
        and runtime_probe["stdout"].strip() == "NVIDIA_DEVICE_OK"
    )
    runtime_ok = runtime_registered and runtime_binary_ok and runtime_probe_ok
    runtime_error = next(
        (
            error
            for error in (
                docker_info["error"],
                "" if runtime_registered else "NVIDIA_RUNTIME_NOT_REGISTERED",
                runtime_version["error"],
                runtime_probe["error"],
                "" if runtime_probe_ok else "NVIDIA_CONTAINER_PROBE_FAILED",
            )
            if error
        ),
        "",
    )
    runtime_detail = {
        "error": runtime_error,
        "registered_runtimes": sorted(runtimes),
        "docker_info": _command_detail(docker_info),
        "runtime_version": {
            **_command_detail(runtime_version),
            "stdout": runtime_version["stdout"].strip()[-500:],
        },
        "container_probe": _command_detail(runtime_probe),
        "image_reference": image_reference,
        "network": "none",
        "pull_policy": "never",
    }
    checks.append(
        _check(
            "nvidia_container_runtime",
            runtime_ok,
            "NVIDIA_RUNTIME_INVALID",
            runtime_detail,
        )
    )
    facts["registered_runtimes"] = sorted(runtimes)
    facts["runtime_version"] = runtime_version["stdout"].strip()[-500:]
    return checks, facts


def collect_hardware_checks(manifest: dict[str, Any]) -> dict[str, Any]:
    """Collect non-mutating hardware evidence.

    The returned mapping contains ``checks`` and ``facts`` and is safe to
    serialize directly into the production provisioning report.
    """

    checks: list[dict[str, Any]] = []
    facts: dict[str, Any] = {}

    jt128_checks, jt128_facts = _collect_jt128(manifest)
    checks.extend(jt128_checks)
    facts["jt128"] = jt128_facts
    can_checks, can_facts = _collect_can(manifest)
    checks.extend(can_checks)
    facts["can"] = can_facts

    orbbec_checks, orbbec_facts = _collect_orbbec(manifest)
    checks.extend(orbbec_checks)
    facts["orbbec"] = orbbec_facts

    nvidia_checks, nvidia_facts = _collect_nvidia(manifest)
    checks.extend(nvidia_checks)
    facts["nvidia"] = nvidia_facts

    failed = [item for item in checks if not item["ok"]]
    return {
        "schema": SCHEMA,
        "ok": not failed,
        "checks": checks,
        "facts": facts,
        "failed_codes": sorted({item["code"] for item in failed}),
    }


__all__ = ["collect_hardware_checks"]
