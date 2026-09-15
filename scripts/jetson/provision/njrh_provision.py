#!/usr/bin/env python3
"""Fail-closed Jetson production provisioner.

The module deliberately keeps production installation separate from the
runtime/container launch scripts.  A device-release manifest is the only input
to a deployment.  Applying it always leaves the robot in READY_LOCKED; motion
is enabled only by the separate, explicit ``activate`` command.
"""

from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import hashlib
import ipaddress
import json
import os
import platform
import re
import shlex
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import types
import urllib.parse
import urllib.request
import uuid
from pathlib import Path, PurePosixPath
from typing import Any, Iterable

try:
    import fcntl
except ImportError:  # pragma: no cover - only used by Linux deployment
    fcntl = None  # type: ignore[assignment]


SCHEMA = "njrh.device_release.v1"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
DIGEST_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
SAFE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,95}$")
SAFE_IFACE_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,31}$")
TEMPLATE_MARKERS = ("REPLACE_", "UNPUBLISHED", "0000000000000000")
FIXED_PATHS = {
    "host_path": "/home/nvidia/workspaces/njrh-v3/workspace1",
    "container_path": "/workspaces/njrh-v3/workspace1",
    "upstream_host_path": "/home/nvidia/workspaces/isaac_ros-dev",
    "upstream_container_path": "/workspaces/isaac_ros-dev",
    "runtime_env_file": "/etc/njrh/runtime.env",
    "secrets_env_file": "/etc/njrh/secrets.env",
    "provision_state_root": "/var/lib/njrh/provision",
    "motion_lock": "/var/lib/njrh/provision/motion.lock",
    "report_root": "/tmp/njrh_reports",
    "hardware_acceptance_file": "/etc/njrh/hardware_acceptance.json",
    "device_identity_file": "/etc/njrh/device-identity.json",
    "trusted_public_key": "/etc/njrh/trust/release-ed25519-public.pem",
}
FIXED_HARDWARE = {
    "can_interface": "can0",
    "can_bitrate": 500000,
    "jt128_interface": "eth1",
    "jt128_host_cidr": "192.168.1.100/24",
    "jt128_device_ip": "192.168.1.201",
    "jt128_lidar_port": 2368,
    "jt128_imu_port": 10110,
}
MAX_ARCHIVE_ENTRIES = 250000
MAX_ARCHIVE_FILE_BYTES = 20 * 1024 * 1024 * 1024
MAX_ARCHIVE_EXPANDED_BYTES = 80 * 1024 * 1024 * 1024
MAX_ARTIFACT_BYTES = 30 * 1024 * 1024 * 1024
MAX_RELEASE_PARTS = 1000
MAX_RELEASE_PART_BYTES = 1900 * 1024 * 1024
EXIT_CODES = {
    "MANIFEST_INVALID": 2,
    "LOCKED": 3,
    "HOST_INCOMPATIBLE": 10,
    "SOURCE_MISMATCH": 11,
    "ARTIFACT_INVALID": 12,
    "IMAGE_INVALID": 13,
    "HARDWARE_INVALID": 14,
    "BUILD_FAILED": 15,
    "TEST_FAILED": 16,
    "INSTALL_FAILED": 17,
    "VERIFY_FAILED": 18,
}


class ProvisionError(RuntimeError):
    def __init__(self, code: str, message: str):
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")


def object_digest(value: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def sha256_file(path: Path, chunk_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write_json(path: Path, value: Any, mode: int = 0o640) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary_path = Path(temporary)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(value, stream, indent=2, ensure_ascii=False, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary_path, mode)
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


def atomic_write_text(path: Path, value: str, mode: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary_path = Path(temporary)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(value)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary_path, mode)
        os.chown(temporary_path, 0, 0)
        os.replace(temporary_path, path)
        _fsync_directory(path.parent)
    finally:
        temporary_path.unlink(missing_ok=True)


def write_motion_lock(manifest: dict[str, Any], reason: str) -> Path:
    path = Path(manifest["installation"]["motion_lock"])
    atomic_write_text(
        path,
        f"release_id={manifest['release_id']}\nreason={reason}\nlocked_at={utc_now()}\n",
        0o600,
    )
    return path


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProvisionError("MANIFEST_INVALID", f"cannot read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ProvisionError("MANIFEST_INVALID", f"{path} must contain a JSON object")
    return value


def require_dict(parent: dict[str, Any], key: str) -> dict[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        raise ProvisionError("MANIFEST_INVALID", f"{key} must be an object")
    return value


def require_string(parent: dict[str, Any], key: str) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or not value:
        raise ProvisionError("MANIFEST_INVALID", f"{key} must be a non-empty string")
    return value


def require_positive_int(parent: dict[str, Any], key: str) -> int:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ProvisionError("MANIFEST_INVALID", f"{key} must be a positive integer")
    return value


def _contains_template_marker(value: str) -> bool:
    return any(marker in value for marker in TEMPLATE_MARKERS)


def _require_safe_relative_path(parent: dict[str, Any], key: str) -> str:
    value = require_string(parent, key)
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in ("", ".", "..") for part in path.parts):
        raise ProvisionError(
            "MANIFEST_INVALID", f"{key} must be a normalized repo-relative path"
        )
    if any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise ProvisionError("MANIFEST_INVALID", f"{key} contains control characters")
    return value


def _require_fixed_path(parent: dict[str, Any], key: str) -> str:
    value = require_string(parent, key)
    expected = FIXED_PATHS[key]
    if value != expected:
        raise ProvisionError(
            "MANIFEST_INVALID", f"{key} must be the production path {expected}"
        )
    return value


def _validate_artifact_transport(
    name: str,
    descriptor: dict[str, Any],
    overall_sha256: str,
    allow_template: bool,
) -> None:
    uri = descriptor.get("uri")
    parts = descriptor.get("parts")
    archive_size = descriptor.get("archive_size_bytes")
    if parts is None:
        if not isinstance(uri, str) or not uri:
            raise ProvisionError(
                "MANIFEST_INVALID", f"artifacts.{name}.uri is required"
            )
        parsed = urllib.parse.urlparse(uri)
        if parsed.scheme not in ("file", "https"):
            raise ProvisionError(
                "MANIFEST_INVALID",
                f"artifacts.{name}.uri must use file:// or https://",
            )
        if archive_size is not None and (
            isinstance(archive_size, bool)
            or not isinstance(archive_size, int)
            or archive_size <= 0
            or archive_size > MAX_ARTIFACT_BYTES
        ):
            raise ProvisionError(
                "MANIFEST_INVALID",
                f"artifacts.{name}.archive_size_bytes is invalid",
            )
        template_value = _contains_template_marker(uri) or set(overall_sha256) == {"0"}
    else:
        if (
            not isinstance(parts, list)
            or not parts
            or len(parts) > MAX_RELEASE_PARTS
        ):
            raise ProvisionError(
                "MANIFEST_INVALID",
                f"artifacts.{name}.parts must contain 1..{MAX_RELEASE_PARTS} entries",
            )
        if uri != "":
            raise ProvisionError(
                "MANIFEST_INVALID",
                f"artifacts.{name}.uri must be empty when parts are used",
            )
        seen_uris: set[str] = set()
        total_size = 0
        template_value = set(overall_sha256) == {"0"}
        for index, part in enumerate(parts):
            if not isinstance(part, dict) or set(part) != {
                "uri",
                "sha256",
                "size_bytes",
            }:
                raise ProvisionError(
                    "MANIFEST_INVALID",
                    f"artifacts.{name}.parts[{index}] has an invalid schema",
                )
            part_uri = part["uri"]
            part_sha = part["sha256"]
            part_size = part["size_bytes"]
            if not isinstance(part_uri, str) or not part_uri:
                raise ProvisionError(
                    "MANIFEST_INVALID",
                    f"artifacts.{name}.parts[{index}].uri is required",
                )
            parsed = urllib.parse.urlparse(part_uri)
            if parsed.scheme not in ("file", "https"):
                raise ProvisionError(
                    "MANIFEST_INVALID",
                    f"artifacts.{name}.parts[{index}].uri must use file:// or https://",
                )
            if part_uri in seen_uris:
                raise ProvisionError(
                    "MANIFEST_INVALID",
                    f"artifacts.{name}.parts contains a duplicate URI",
                )
            seen_uris.add(part_uri)
            if not isinstance(part_sha, str) or not SHA256_RE.fullmatch(part_sha):
                raise ProvisionError(
                    "MANIFEST_INVALID",
                    f"artifacts.{name}.parts[{index}].sha256 is invalid",
                )
            if (
                isinstance(part_size, bool)
                or not isinstance(part_size, int)
                or part_size <= 0
                or part_size > MAX_RELEASE_PART_BYTES
            ):
                raise ProvisionError(
                    "MANIFEST_INVALID",
                    f"artifacts.{name}.parts[{index}].size_bytes is invalid",
                )
            total_size += part_size
            template_value = (
                template_value
                or _contains_template_marker(part_uri)
                or set(part_sha) == {"0"}
            )
        if (
            isinstance(archive_size, bool)
            or not isinstance(archive_size, int)
            or archive_size != total_size
            or archive_size > MAX_ARTIFACT_BYTES
        ):
            raise ProvisionError(
                "MANIFEST_INVALID",
                f"artifacts.{name}.archive_size_bytes does not match its parts",
            )
    if not allow_template and template_value:
        raise ProvisionError(
            "MANIFEST_INVALID", f"artifacts.{name} still contains a template value"
        )


def _validate_payload(
    name: str,
    descriptor: dict[str, Any],
    expected_destination: str,
    allow_template: bool,
) -> None:
    required = descriptor.get("required")
    if not isinstance(required, bool):
        raise ProvisionError(
            "MANIFEST_INVALID", f"artifacts.{name}.required must be boolean"
        )
    if descriptor.get("kind") != "payload":
        raise ProvisionError(
            "MANIFEST_INVALID", f"artifacts.{name}.kind must be payload"
        )
    uri = descriptor.get("uri")
    sha = descriptor.get("sha256")
    parts = descriptor.get("parts")
    destination = descriptor.get("destination")
    if destination != expected_destination:
        raise ProvisionError(
            "MANIFEST_INVALID",
            f"artifacts.{name}.destination must be {expected_destination}",
        )
    if not required and uri == "" and sha == "" and parts is None:
        return
    if not isinstance(sha, str) or not SHA256_RE.fullmatch(sha):
        raise ProvisionError(
            "MANIFEST_INVALID", f"artifacts.{name}.sha256 must be 64 lowercase hex"
        )
    _validate_artifact_transport(name, descriptor, sha, allow_template)


def validate_manifest(manifest: dict[str, Any], allow_template: bool = False) -> None:
    if manifest.get("schema") != SCHEMA:
        raise ProvisionError("MANIFEST_INVALID", f"schema must be {SCHEMA}")
    release_id = require_string(manifest, "release_id")
    if not SAFE_ID_RE.fullmatch(release_id):
        raise ProvisionError("MANIFEST_INVALID", "release_id is not path-safe")
    release_state = require_string(manifest, "release_state")
    if release_state not in ("template", "published"):
        raise ProvisionError(
            "MANIFEST_INVALID", "release_state must be template or published"
        )
    if release_state != "published" and not allow_template:
        raise ProvisionError("MANIFEST_INVALID", "release is not published")
    if manifest.get("desired_state") != "ready_locked":
        raise ProvisionError(
            "MANIFEST_INVALID",
            "desired_state must be ready_locked; activation is a separate operation",
        )

    source = require_dict(manifest, "source")
    repository = require_string(source, "repository")
    if repository != "https://github.com/Neepi1/CAR.V1.git":
        raise ProvisionError(
            "MANIFEST_INVALID",
            "source.repository must be https://github.com/Neepi1/CAR.V1.git",
        )
    commit = require_string(source, "commit")
    if not COMMIT_RE.fullmatch(commit):
        raise ProvisionError(
            "MANIFEST_INVALID", "source.commit must be a full lowercase Git SHA"
        )
    if source.get("require_clean_tracked_tree") is not True:
        raise ProvisionError(
            "MANIFEST_INVALID", "source.require_clean_tracked_tree must be true"
        )
    if source.get("require_clean_tree") is not True:
        raise ProvisionError(
            "MANIFEST_INVALID", "source.require_clean_tree must be true"
        )

    platform_doc = require_dict(manifest, "platform")
    if platform_doc.get("architecture") != "aarch64":
        raise ProvisionError(
            "MANIFEST_INVALID", "platform.architecture must be aarch64"
        )
    for key in (
        "l4t_release",
        "l4t_revision",
        "jetson_model",
        "jetpack_version",
        "l4t_core_version",
        "tnspec",
        "compatible_spec",
    ):
        require_string(platform_doc, key)
    for key in (
        "minimum_online_cpus",
        "minimum_memory_bytes",
        "minimum_workspace_free_bytes",
        "minimum_docker_free_bytes",
        "minimum_state_free_bytes",
        "minimum_free_inodes",
    ):
        require_positive_int(platform_doc, key)

    workspace = require_dict(manifest, "workspace")
    workspace_paths: dict[str, str] = {}
    for key in (
        "host_path",
        "container_path",
        "upstream_host_path",
        "upstream_container_path",
    ):
        workspace_paths[key] = _require_fixed_path(workspace, key)

    artifacts = require_dict(manifest, "artifacts")
    runtime_image = require_dict(artifacts, "runtime_image")
    runtime_kind = require_string(runtime_image, "kind")
    if runtime_kind not in ("oci", "docker_archive"):
        raise ProvisionError(
            "MANIFEST_INVALID",
            "artifacts.runtime_image.kind must be oci or docker_archive",
        )
    uri = runtime_image.get("uri")
    if not isinstance(uri, str):
        raise ProvisionError(
            "MANIFEST_INVALID", "artifacts.runtime_image.uri must be a string"
        )
    reference = require_string(runtime_image, "reference")
    image_id = require_string(runtime_image, "image_id")
    if ":latest" in reference or reference.endswith("/latest"):
        raise ProvisionError(
            "MANIFEST_INVALID", "runtime image reference is mutable"
        )
    if not DIGEST_RE.fullmatch(image_id):
        raise ProvisionError(
            "MANIFEST_INVALID", "runtime image image_id must use sha256"
        )
    if runtime_kind == "oci":
        if not uri:
            raise ProvisionError(
                "MANIFEST_INVALID", "artifacts.runtime_image.uri is required"
            )
        digest = require_string(runtime_image, "digest")
        if not uri.startswith("oci://") or "@" not in uri:
            raise ProvisionError(
                "MANIFEST_INVALID", "runtime image URI must be oci://...@sha256:..."
            )
        if not DIGEST_RE.fullmatch(digest):
            raise ProvisionError(
                "MANIFEST_INVALID", "runtime image digest must use sha256"
            )
        if not uri.endswith(f"@{digest}"):
            raise ProvisionError(
                "MANIFEST_INVALID", "runtime image URI digest does not match digest"
            )
        if not reference.startswith("ghcr.io/neepi1/car-v1/njrh-car:"):
            raise ProvisionError(
                "MANIFEST_INVALID", "OCI runtime image must use the NJRH GHCR repository"
            )
        template_image = (
            _contains_template_marker(uri)
            or digest == "sha256:" + ("0" * 64)
            or _contains_template_marker(reference)
        )
    else:
        archive_sha = runtime_image.get("sha256")
        if not isinstance(archive_sha, str) or not SHA256_RE.fullmatch(archive_sha):
            raise ProvisionError(
                "MANIFEST_INVALID",
                "docker archive sha256 must be 64 lowercase hex",
            )
        _validate_artifact_transport(
            "runtime_image", runtime_image, archive_sha, allow_template
        )
        template_image = set(archive_sha) == {"0"} or _contains_template_marker(
            reference
        )
    if not allow_template and template_image:
        raise ProvisionError(
            "MANIFEST_INVALID", "runtime image still contains a template value"
        )

    _validate_payload(
        "upstream_runtime",
        require_dict(artifacts, "upstream_runtime"),
        workspace_paths["upstream_host_path"],
        allow_template,
    )
    _validate_payload(
        "runtime_overlays",
        require_dict(artifacts, "runtime_overlays"),
        f"{workspace_paths['host_path']}/.runtime",
        allow_template,
    )
    _validate_payload(
        "site_assets",
        require_dict(artifacts, "site_assets"),
        f"{workspace_paths['host_path']}/maps_release",
        allow_template,
    )

    hardware = require_dict(manifest, "hardware")
    can = require_dict(hardware, "can")
    can_iface = require_string(can, "interface")
    if can_iface != FIXED_HARDWARE["can_interface"]:
        raise ProvisionError("MANIFEST_INVALID", "CAN interface must be can0")
    if require_positive_int(can, "bitrate") != FIXED_HARDWARE["can_bitrate"]:
        raise ProvisionError("MANIFEST_INVALID", "CAN bitrate must be 500000")
    if not isinstance(can.get("require_up_during_preflight"), bool):
        raise ProvisionError(
            "MANIFEST_INVALID", "CAN require_up_during_preflight must be boolean"
        )
    jt128 = require_dict(hardware, "jt128")
    jt128_iface = require_string(jt128, "interface")
    if jt128_iface != FIXED_HARDWARE["jt128_interface"]:
        raise ProvisionError("MANIFEST_INVALID", "JT128 interface must be eth1")
    try:
        host_cidr = str(ipaddress.ip_interface(require_string(jt128, "host_cidr")))
        device_ip = str(ipaddress.ip_address(require_string(jt128, "device_ip")))
    except ValueError as exc:
        raise ProvisionError("MANIFEST_INVALID", f"invalid JT128 address: {exc}") from exc
    if (
        host_cidr != FIXED_HARDWARE["jt128_host_cidr"]
        or device_ip != FIXED_HARDWARE["jt128_device_ip"]
    ):
        raise ProvisionError(
            "MANIFEST_INVALID",
            "JT128 wiring must match eth1 192.168.1.100/24 -> 192.168.1.201",
        )
    if require_positive_int(jt128, "lidar_port") != FIXED_HARDWARE["jt128_lidar_port"]:
        raise ProvisionError("MANIFEST_INVALID", "JT128 lidar_port must be 2368")
    if require_positive_int(jt128, "imu_port") != FIXED_HARDWARE["jt128_imu_port"]:
        raise ProvisionError("MANIFEST_INVALID", "JT128 imu_port must be 10110")
    if not isinstance(jt128.get("require_dedicated_interface"), bool):
        raise ProvisionError(
            "MANIFEST_INVALID", "JT128 require_dedicated_interface must be boolean"
        )
    orbbec = require_dict(hardware, "orbbec")
    if not re.fullmatch(r"[0-9a-f]{4}:[0-9a-f]{4}", require_string(orbbec, "usb_id")):
        raise ProvisionError("MANIFEST_INVALID", "Orbbec usb_id is invalid")
    serial = require_string(orbbec, "serial")
    if (
        serial != "AUTO_ENROLL"
        and not allow_template
        and _contains_template_marker(serial)
    ):
        raise ProvisionError("MANIFEST_INVALID", "Orbbec serial is still a template")
    if orbbec.get("required") is not True:
        raise ProvisionError("MANIFEST_INVALID", "Orbbec required must be true")
    if require_positive_int(orbbec, "minimum_usb_speed_mbps") < 5000:
        raise ProvisionError(
            "MANIFEST_INVALID", "Orbbec minimum_usb_speed_mbps must be at least 5000"
        )
    cpu_ids = hardware.get("required_cpu_ids")
    if (
        not isinstance(cpu_ids, list)
        or not cpu_ids
        or any(isinstance(item, bool) or not isinstance(item, int) or item < 0 for item in cpu_ids)
        or cpu_ids != sorted(set(cpu_ids))
    ):
        raise ProvisionError(
            "MANIFEST_INVALID", "required_cpu_ids must be unique sorted integers"
        )

    build = require_dict(manifest, "build")
    for key in ("package_manifest", "pytest_failure_allowlist"):
        value = require_string(build, key)
        if Path(value).is_absolute() or ".." in Path(value).parts:
            raise ProvisionError("MANIFEST_INVALID", f"build.{key} must be repo-relative")
    if build.get("build_testing") is not True:
        raise ProvisionError("MANIFEST_INVALID", "build.build_testing must be true")

    trust = require_dict(manifest, "trust")
    if trust.get("signature_algorithm") != "ed25519":
        raise ProvisionError("MANIFEST_INVALID", "trust.signature_algorithm must be ed25519")
    release_lock_name = _require_safe_relative_path(trust, "release_lock")
    signature_name = _require_safe_relative_path(trust, "signature_file")
    if release_lock_name != "release-lock.json":
        raise ProvisionError(
            "MANIFEST_INVALID", "trust.release_lock must be release-lock.json"
        )
    if signature_name != "device-release.json.sig":
        raise ProvisionError(
            "MANIFEST_INVALID",
            "trust.signature_file must be device-release.json.sig",
        )
    trusted_public_key = require_string(trust, "trusted_public_key")
    if trusted_public_key != FIXED_PATHS["trusted_public_key"]:
        raise ProvisionError(
            "MANIFEST_INVALID",
            f"trust.trusted_public_key must be {FIXED_PATHS['trusted_public_key']}",
        )
    for key in ("release_lock_sha256", "trusted_public_key_sha256"):
        value = require_string(trust, key)
        if not SHA256_RE.fullmatch(value):
            raise ProvisionError("MANIFEST_INVALID", f"trust.{key} must be 64 lowercase hex")
        if not allow_template and set(value) == {"0"}:
            raise ProvisionError("MANIFEST_INVALID", f"trust.{key} is still a template")

    installation = require_dict(manifest, "installation")
    if installation.get("container_name") != "NJRH-car":
        raise ProvisionError("MANIFEST_INVALID", "container_name must be NJRH-car")
    if installation.get("runtime_user") != "root":
        raise ProvisionError("MANIFEST_INVALID", "runtime_user must be root")
    for key in (
        "runtime_env_file",
        "secrets_env_file",
        "provision_state_root",
        "motion_lock",
        "report_root",
        "hardware_acceptance_file",
        "device_identity_file",
    ):
        _require_fixed_path(installation, key)
    if installation.get("require_api_secret") is not True:
        raise ProvisionError(
            "MANIFEST_INVALID", "installation.require_api_secret must be true"
        )

    forbidden_keys = {
        "robot_api_token",
        "password",
        "private_key",
        "github_token",
        "registry_password",
    }

    def check_secret_keys(value: Any) -> None:
        if isinstance(value, dict):
            for key, child in value.items():
                if key.lower() in forbidden_keys:
                    raise ProvisionError(
                        "MANIFEST_INVALID", f"secret material key is forbidden: {key}"
                    )
                check_secret_keys(child)
        elif isinstance(value, list):
            for child in value:
                check_secret_keys(child)

    check_secret_keys(manifest)


def _relative_link_stays_in_tree(link_path: PurePosixPath, target: str) -> bool:
    target_path = PurePosixPath(target)
    if target_path.is_absolute() or not target or "\\" in target:
        return False
    stack = list(link_path.parent.parts)
    for part in target_path.parts:
        if part in ("", "."):
            continue
        if part == "..":
            if not stack:
                return False
            stack.pop()
        else:
            stack.append(part)
    return bool(stack)


def inspect_payload_archive(
    path: Path, *, allow_safe_symlinks: bool = False
) -> list[str]:
    if not path.is_file():
        raise ProvisionError("ARTIFACT_INVALID", f"payload archive missing: {path}")
    try:
        archive = tarfile.open(path, mode="r:*")
    except (tarfile.TarError, OSError) as exc:
        raise ProvisionError(
            "ARTIFACT_INVALID", f"cannot open payload archive {path}: {exc}"
        ) from exc
    names: list[str] = []
    seen: set[str] = set()
    try:
        members = archive.getmembers()
        if not members:
            raise ProvisionError("ARTIFACT_INVALID", "payload archive is empty")
        if len(members) > MAX_ARCHIVE_ENTRIES:
            raise ProvisionError("ARTIFACT_INVALID", "payload archive has too many entries")
        expanded_bytes = 0
        for member in members:
            name = member.name.replace("\\", "/")
            posix = PurePosixPath(name)
            if (
                not name
                or name.startswith("/")
                or any(part in ("", ".", "..") for part in posix.parts)
            ):
                raise ProvisionError(
                    "ARTIFACT_INVALID", f"unsafe archive path: {member.name}"
                )
            canonical = posix.as_posix()
            if canonical in seen:
                raise ProvisionError(
                    "ARTIFACT_INVALID", f"duplicate archive path: {canonical}"
                )
            seen.add(canonical)
            if member.islnk():
                raise ProvisionError(
                    "ARTIFACT_INVALID", f"archive hard link is forbidden: {canonical}"
                )
            if member.issym() and (
                not allow_safe_symlinks
                or not _relative_link_stays_in_tree(posix, member.linkname)
            ):
                raise ProvisionError(
                    "ARTIFACT_INVALID", f"archive link is forbidden: {canonical}"
                )
            if member.isdev() or member.isfifo():
                raise ProvisionError(
                    "ARTIFACT_INVALID", f"special archive entry is forbidden: {canonical}"
                )
            if not (member.isfile() or member.isdir() or member.issym()):
                raise ProvisionError(
                    "ARTIFACT_INVALID", f"unsupported archive entry: {canonical}"
                )
            if member.mode & 0o6000:
                raise ProvisionError(
                    "ARTIFACT_INVALID", f"setuid/setgid archive mode is forbidden: {canonical}"
                )
            if member.mode & 0o002:
                raise ProvisionError(
                    "ARTIFACT_INVALID", f"world-writable archive mode is forbidden: {canonical}"
                )
            if member.isfile():
                if member.size < 0 or member.size > MAX_ARCHIVE_FILE_BYTES:
                    raise ProvisionError(
                        "ARTIFACT_INVALID", f"archive file is too large: {canonical}"
                    )
                expanded_bytes += member.size
                if expanded_bytes > MAX_ARCHIVE_EXPANDED_BYTES:
                    raise ProvisionError(
                        "ARTIFACT_INVALID", "payload archive expands beyond the limit"
                    )
            names.append(canonical)
    finally:
        archive.close()
    return names


def tree_digest(root: Path) -> str:
    """Hash every file, mode and safe symlink target in a directory tree."""

    if root.is_symlink() or not root.is_dir():
        raise ProvisionError("ARTIFACT_INVALID", f"tree is not a directory: {root}")
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*"), key=lambda item: item.relative_to(root).as_posix()):
        relative = path.relative_to(root).as_posix()
        metadata = path.lstat()
        mode = stat.S_IMODE(metadata.st_mode)
        if stat.S_ISDIR(metadata.st_mode):
            record = f"D\0{relative}\0{mode:04o}\n".encode()
        elif stat.S_ISREG(metadata.st_mode):
            record = (
                f"F\0{relative}\0{mode:04o}\0{metadata.st_size}\0{sha256_file(path)}\n"
            ).encode()
        elif stat.S_ISLNK(metadata.st_mode):
            target = os.readlink(path)
            if not _relative_link_stays_in_tree(PurePosixPath(relative), target):
                raise ProvisionError(
                    "ARTIFACT_INVALID", f"tree symlink escapes root: {relative}"
                )
            record = f"L\0{relative}\0{target}\n".encode()
        else:
            raise ProvisionError(
                "ARTIFACT_INVALID", f"special entry in payload tree: {relative}"
            )
        digest.update(record)
    return digest.hexdigest()


def _safe_extract_payload(
    archive_path: Path, destination: Path, *, allow_safe_symlinks: bool
) -> None:
    """Extract without trusting archive owners, special entries or path handling."""

    with tarfile.open(archive_path, mode="r:*") as archive:
        members = archive.getmembers()
        required_bytes = sum(member.size for member in members if member.isfile())
        if shutil.disk_usage(destination.parent).free < required_bytes + 1024 * 1024 * 1024:
            raise ProvisionError(
                "ARTIFACT_INVALID", "insufficient free space for expanded payload"
            )
        for member in members:
            relative = PurePosixPath(member.name.replace("\\", "/"))
            target = destination.joinpath(*relative.parts)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                os.chmod(target, 0o755)
            elif member.isfile():
                target.parent.mkdir(parents=True, exist_ok=True)
                source = archive.extractfile(member)
                if source is None:
                    raise ProvisionError(
                        "ARTIFACT_INVALID", f"cannot read archive member: {member.name}"
                    )
                temporary = target.with_name(f".{target.name}.{uuid.uuid4().hex}.part")
                with source, temporary.open("xb") as output:
                    shutil.copyfileobj(source, output, length=8 * 1024 * 1024)
                    output.flush()
                    os.fsync(output.fileno())
                normalized_mode = 0o755 if member.mode & 0o111 else 0o644
                os.chmod(temporary, normalized_mode)
                os.replace(temporary, target)
        if allow_safe_symlinks:
            for member in members:
                if not member.issym():
                    continue
                relative = PurePosixPath(member.name.replace("\\", "/"))
                target = destination.joinpath(*relative.parts)
                target.parent.mkdir(parents=True, exist_ok=True)
                os.symlink(member.linkname, target)


def _run(
    command: Iterable[str],
    *,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    check: bool = True,
    timeout: int | None = None,
) -> subprocess.CompletedProcess[str]:
    rendered_command = list(command)
    try:
        result = subprocess.run(
            rendered_command,
            cwd=str(cwd) if cwd else None,
            env=env,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
        )
    except (FileNotFoundError, PermissionError, subprocess.TimeoutExpired) as exc:
        rendered = " ".join(shlex.quote(item) for item in rendered_command)
        if check:
            raise ProvisionError(
                "INSTALL_FAILED", f"cannot execute command: {rendered}: {exc}"
            ) from exc
        return subprocess.CompletedProcess(rendered_command, 127, "", str(exc))
    if check and result.returncode != 0:
        rendered = " ".join(shlex.quote(item) for item in rendered_command)
        detail = (result.stderr or result.stdout).strip()[-4000:]
        raise ProvisionError(
            "INSTALL_FAILED", f"command failed ({result.returncode}): {rendered}: {detail}"
        )
    return result


def parse_cpu_list(value: str) -> set[int]:
    cpus: set[int] = set()
    for part in value.strip().split(","):
        if not part:
            continue
        if "-" in part:
            start_text, end_text = part.split("-", 1)
            start, end = int(start_text), int(end_text)
            if end < start:
                raise ValueError(value)
            cpus.update(range(start, end + 1))
        else:
            cpus.add(int(part))
    return cpus


def _read_memory_total() -> int:
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        if line.startswith("MemTotal:"):
            return int(line.split()[1]) * 1024
    raise ProvisionError("HOST_INCOMPATIBLE", "MemTotal is unavailable")


def _existing_ancestor(path: Path) -> Path:
    candidate = path
    while not candidate.exists() and candidate != candidate.parent:
        candidate = candidate.parent
    return candidate


def _read_l4t() -> tuple[str, str, str]:
    path = Path("/etc/nv_tegra_release")
    if not path.is_file():
        raise ProvisionError("HOST_INCOMPATIBLE", "/etc/nv_tegra_release is missing")
    first_line = path.read_text(encoding="utf-8", errors="replace").splitlines()[0]
    match = re.search(r"# R(\d+).*REVISION:\s*([0-9.]+)", first_line)
    if not match:
        raise ProvisionError("HOST_INCOMPATIBLE", "cannot parse L4T release")
    return match.group(1), match.group(2), first_line


def _read_text_without_nuls(path: Path) -> str:
    try:
        return path.read_bytes().replace(b"\x00", b"").decode(
            "utf-8", errors="replace"
        ).strip()
    except OSError:
        return ""


def _dpkg_version(package: str) -> str:
    result = _run(
        ["dpkg-query", "-W", "-f=${Version}", package], check=False, timeout=10
    )
    return result.stdout.strip() if result.returncode == 0 else ""


def _boot_control_value(key: str) -> str:
    path = Path("/etc/nv_boot_control.conf")
    if not path.is_file():
        return ""
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        name, _, value = line.partition(" ")
        if name == key:
            return value.strip()
    return ""


def _orbbec_devices(usb_id: str) -> list[dict[str, str]]:
    vendor, product = usb_id.split(":", 1)
    devices: list[dict[str, str]] = []
    root = Path("/sys/bus/usb/devices")
    if not root.is_dir():
        return devices
    for candidate in root.iterdir():
        try:
            actual_vendor = (candidate / "idVendor").read_text().strip().lower()
            actual_product = (candidate / "idProduct").read_text().strip().lower()
        except OSError:
            continue
        if actual_vendor != vendor or actual_product != product:
            continue
        try:
            serial = (candidate / "serial").read_text().strip()
        except OSError:
            serial = ""
        try:
            speed = (candidate / "speed").read_text().strip()
        except OSError:
            speed = ""
        devices.append({"path": str(candidate), "serial": serial, "speed_mbps": speed})
    return devices


def verify_source(workspace: Path, source: dict[str, Any]) -> dict[str, Any]:
    if not (workspace / ".git").exists():
        raise ProvisionError("SOURCE_MISMATCH", f"not a Git worktree: {workspace}")
    head = _run(["git", "rev-parse", "HEAD"], cwd=workspace).stdout.strip()
    if head != source["commit"]:
        raise ProvisionError(
            "SOURCE_MISMATCH",
            f"HEAD {head} does not match release commit {source['commit']}",
        )
    origin = _run(
        ["git", "config", "--get", "remote.origin.url"], cwd=workspace, check=False
    ).stdout.strip()
    expected_repo = source["repository"].removesuffix(".git").lower()
    normalized_origin = origin.removesuffix(".git").replace("git@github.com:", "https://github.com/")
    if normalized_origin.lower() != expected_repo:
        raise ProvisionError(
            "SOURCE_MISMATCH",
            f"origin {origin!r} does not match {source['repository']!r}",
        )
    worktree_status = _run(
        ["git", "status", "--porcelain", "--untracked-files=all"], cwd=workspace
    ).stdout.strip()
    if worktree_status:
        raise ProvisionError(
            "SOURCE_MISMATCH", "worktree changes or untracked files are present"
        )
    return {"head": head, "origin": origin, "worktree_clean": True}


def _adjacent_release_file(manifest_path: Path, relative: str, label: str) -> Path:
    candidate = manifest_path.parent / relative
    try:
        release_root = manifest_path.parent.resolve(strict=True)
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        raise ProvisionError("SOURCE_MISMATCH", f"{label} is unavailable: {exc}") from exc
    if resolved.parent != release_root:
        raise ProvisionError("SOURCE_MISMATCH", f"{label} escapes the release bundle")
    if candidate.is_symlink() or not resolved.is_file():
        raise ProvisionError("SOURCE_MISMATCH", f"{label} must be a regular bundle file")
    return resolved


def _verify_ed25519_detached(
    public_key_bytes: bytes, signature_bytes: bytes, payload: bytes
) -> None:
    """Verify frozen bytes through seekable, inherited anonymous file handles."""

    if os.name != "posix" or not Path("/proc/self/fd").is_dir():
        raise ProvisionError(
            "HOST_INCOMPATIBLE",
            "Ed25519 release verification requires Linux /proc file descriptors",
        )
    if len(signature_bytes) != 64:
        raise ProvisionError(
            "MANIFEST_INVALID", "manifest Ed25519 signature must be exactly 64 bytes"
        )
    try:
        with (
            tempfile.TemporaryFile(prefix="njrh-release-key-") as key_stream,
            tempfile.TemporaryFile(prefix="njrh-release-signature-") as signature_stream,
            tempfile.TemporaryFile(prefix="njrh-release-manifest-") as manifest_stream,
        ):
            for stream, value in (
                (key_stream, public_key_bytes),
                (signature_stream, signature_bytes),
                (manifest_stream, payload),
            ):
                stream.write(value)
                stream.flush()
                stream.seek(0)
            key_fd = key_stream.fileno()
            signature_fd = signature_stream.fileno()
            manifest_fd = manifest_stream.fileno()
            result = subprocess.run(
                [
                    "openssl",
                    "pkeyutl",
                    "-verify",
                    "-pubin",
                    "-inkey",
                    f"/proc/self/fd/{key_fd}",
                    "-sigfile",
                    f"/proc/self/fd/{signature_fd}",
                    "-rawin",
                    "-in",
                    f"/proc/self/fd/{manifest_fd}",
                ],
                pass_fds=(key_fd, signature_fd, manifest_fd),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                timeout=30,
            )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ProvisionError(
            "MANIFEST_INVALID", f"cannot execute manifest signature verification: {exc}"
        ) from exc
    if result.returncode != 0:
        raise ProvisionError("MANIFEST_INVALID", "manifest Ed25519 signature is invalid")


def verify_release_trust(
    manifest: dict[str, Any], manifest_path: Path, workspace: Path
) -> dict[str, Any]:
    """Verify the detached manifest signature and bind it to the golden lock.

    The public key is an out-of-band factory trust root.  It is intentionally
    not bootstrapped from the cloned repository.
    """

    trust = manifest["trust"]
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise ProvisionError("MANIFEST_INVALID", "manifest must be a regular file")
    public_key = Path(trust["trusted_public_key"])
    try:
        key_stat = public_key.lstat()
    except OSError as exc:
        raise ProvisionError(
            "MANIFEST_INVALID",
            f"factory trust root is missing: {public_key}; provision it out of band",
        ) from exc
    if (
        stat.S_ISLNK(key_stat.st_mode)
        or not stat.S_ISREG(key_stat.st_mode)
        or key_stat.st_uid != 0
        or stat.S_IMODE(key_stat.st_mode) not in (0o600, 0o644)
    ):
        raise ProvisionError(
            "MANIFEST_INVALID",
            "factory trust root must be a root-owned regular file with mode 0600/0644",
        )
    try:
        public_key_bytes = public_key.read_bytes()
    except OSError as exc:
        raise ProvisionError(
            "MANIFEST_INVALID", f"cannot read factory trust root: {exc}"
        ) from exc
    actual_key_sha = hashlib.sha256(public_key_bytes).hexdigest()
    if actual_key_sha != trust["trusted_public_key_sha256"]:
        raise ProvisionError("MANIFEST_INVALID", "factory trust root SHA-256 mismatch")

    release_lock_path = _adjacent_release_file(
        manifest_path, trust["release_lock"], "production release lock"
    )
    try:
        release_lock_bytes = release_lock_path.read_bytes()
        release_lock = json.loads(release_lock_bytes)
    except (OSError, json.JSONDecodeError) as exc:
        raise ProvisionError(
            "MANIFEST_INVALID", f"cannot read production release lock: {exc}"
        ) from exc
    if not isinstance(release_lock, dict):
        raise ProvisionError("MANIFEST_INVALID", "production release lock is not an object")
    actual_lock_sha = hashlib.sha256(release_lock_bytes).hexdigest()
    if actual_lock_sha != trust["release_lock_sha256"]:
        raise ProvisionError("MANIFEST_INVALID", "production release lock SHA-256 mismatch")
    if (
        release_lock.get("schema") != "njrh.production_release_lock.v1"
        or release_lock.get("release_state") != "published"
        or release_lock.get("release_id") != manifest["release_id"]
    ):
        raise ProvisionError(
            "MANIFEST_INVALID", "release lock is not published for this release"
        )
    lock_source = release_lock.get("source", {})
    if (
        lock_source.get("repository") != manifest["source"]["repository"]
        or lock_source.get("commit") != manifest["source"]["commit"]
    ):
        raise ProvisionError("MANIFEST_INVALID", "release lock source binding mismatch")
    lock_platform = release_lock.get("platform", {})
    for key in (
        "architecture",
        "l4t_release",
        "l4t_revision",
        "jetson_model",
        "jetpack_version",
        "l4t_core_version",
        "tnspec",
        "compatible_spec",
    ):
        if lock_platform.get(key) != manifest["platform"][key]:
            raise ProvisionError(
                "MANIFEST_INVALID", f"release lock platform binding mismatch: {key}"
            )
    locked_image = release_lock.get("golden_images", {}).get("runtime_image", {})
    image = manifest["artifacts"]["runtime_image"]
    if locked_image.get("image_id") != image["image_id"]:
        raise ProvisionError("MANIFEST_INVALID", "release lock image ID mismatch")
    lock_artifacts = release_lock.get("artifacts", {})
    expected_artifacts = {
        "runtime_image": image.get("digest", image.get("sha256", "")),
        "upstream_runtime": manifest["artifacts"]["upstream_runtime"]["sha256"],
        "runtime_overlays": manifest["artifacts"]["runtime_overlays"]["sha256"],
        "site_assets": manifest["artifacts"]["site_assets"]["sha256"],
    }
    if lock_artifacts != expected_artifacts:
        raise ProvisionError("MANIFEST_INVALID", "release lock artifact binding mismatch")

    signature = _adjacent_release_file(
        manifest_path, trust["signature_file"], "manifest signature"
    )
    try:
        signature_bytes = signature.read_bytes()
        manifest_bytes = manifest_path.read_bytes()
        manifest_from_bytes = json.loads(manifest_bytes)
    except (OSError, json.JSONDecodeError) as exc:
        raise ProvisionError("MANIFEST_INVALID", f"cannot reread manifest: {exc}") from exc
    if manifest_from_bytes != manifest:
        raise ProvisionError("MANIFEST_INVALID", "manifest changed after it was loaded")
    _verify_ed25519_detached(public_key_bytes, signature_bytes, manifest_bytes)
    return {
        "public_key_sha256": actual_key_sha,
        "release_lock": str(release_lock_path),
        "release_lock_sha256": actual_lock_sha,
        "signature": str(signature),
    }


def verify_release_bundle(
    manifest: dict[str, Any], manifest_path: Path
) -> dict[str, Any]:
    """Verify downloaded release metadata before it is persisted or applied."""

    validate_manifest(manifest, allow_template=False)
    workspace = Path(manifest["workspace"]["host_path"])
    source = verify_source(workspace, manifest["source"])
    trust = verify_release_trust(manifest, manifest_path, workspace)
    return {
        "schema": "njrh.verified_release_bundle.v1",
        "release_id": manifest["release_id"],
        "manifest_digest": object_digest(manifest),
        "source": source,
        "trust": trust,
    }


def _collect_strict_hardware_preflight(manifest: dict[str, Any]) -> dict[str, Any]:
    module_path = Path(__file__).with_name("hardware_preflight.py")
    if not module_path.is_file():
        raise ProvisionError("HARDWARE_INVALID", "hardware preflight module is missing")
    module = types.ModuleType("njrh_hardware_preflight")
    module.__file__ = str(module_path)
    try:
        source = module_path.read_text(encoding="utf-8")
        exec(compile(source, str(module_path), "exec"), module.__dict__)
        report = module.collect_hardware_checks(manifest)
    except Exception as exc:
        raise ProvisionError(
            "HARDWARE_INVALID", f"hardware preflight crashed safely: {exc}"
        ) from exc
    if not isinstance(report, dict) or not isinstance(report.get("checks"), list):
        raise ProvisionError("HARDWARE_INVALID", "hardware preflight returned invalid data")
    return report


def collect_preflight(
    manifest: dict[str, Any], *, include_hardware: bool = True
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    facts: dict[str, Any] = {}

    def add(name: str, ok: bool, detail: Any, code: str) -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail, "code": code})

    expected_platform = manifest["platform"]
    actual_arch = platform.machine()
    facts["architecture"] = actual_arch
    add(
        "architecture",
        actual_arch == expected_platform["architecture"],
        actual_arch,
        "HOST_INCOMPATIBLE",
    )

    l4t_release, l4t_revision, l4t_raw = _read_l4t()
    facts.update(
        {
            "l4t_release": l4t_release,
            "l4t_revision": l4t_revision,
            "l4t_raw": l4t_raw,
        }
    )
    add(
        "l4t",
        l4t_release == expected_platform["l4t_release"]
        and l4t_revision == expected_platform["l4t_revision"],
        f"R{l4t_release} revision {l4t_revision}",
        "HOST_INCOMPATIBLE",
    )
    jetson_identity = {
        "model": _read_text_without_nuls(Path("/proc/device-tree/model")),
        "jetpack_version": _dpkg_version("nvidia-jetpack"),
        "l4t_core_version": _dpkg_version("nvidia-l4t-core"),
        "tnspec": _boot_control_value("TNSPEC"),
        "compatible_spec": _boot_control_value("COMPATIBLE_SPEC"),
    }
    facts["jetson_identity"] = jetson_identity
    expected_identity = {
        "model": expected_platform["jetson_model"],
        "jetpack_version": expected_platform["jetpack_version"],
        "l4t_core_version": expected_platform["l4t_core_version"],
        "tnspec": expected_platform["tnspec"],
        "compatible_spec": expected_platform["compatible_spec"],
    }
    add(
        "jetson_identity",
        jetson_identity == expected_identity,
        {"actual": jetson_identity, "expected": expected_identity},
        "HOST_INCOMPATIBLE",
    )

    online_path = Path("/sys/devices/system/cpu/online")
    online = (
        parse_cpu_list(online_path.read_text())
        if online_path.is_file()
        else set(range(os.cpu_count() or 0))
    )
    facts["online_cpu_ids"] = sorted(online)
    required_cpu_ids = set(manifest["hardware"]["required_cpu_ids"])
    add(
        "online_cpus",
        len(online) >= expected_platform["minimum_online_cpus"]
        and required_cpu_ids.issubset(online),
        sorted(online),
        "HOST_INCOMPATIBLE",
    )
    affinity_results = {
        str(cpu_id): _run(
            ["taskset", "-c", str(cpu_id), "true"], check=False, timeout=5
        ).returncode
        for cpu_id in sorted(required_cpu_ids)
    }
    add(
        "cpu_affinity",
        all(code == 0 for code in affinity_results.values()),
        affinity_results,
        "HOST_INCOMPATIBLE",
    )

    memory_total = _read_memory_total()
    facts["memory_total_bytes"] = memory_total
    add(
        "memory",
        memory_total >= expected_platform["minimum_memory_bytes"],
        memory_total,
        "HOST_INCOMPATIBLE",
    )

    workspace = Path(manifest["workspace"]["host_path"])
    workspace_probe = _existing_ancestor(workspace)
    disk = shutil.disk_usage(workspace_probe)
    workspace_vfs = os.statvfs(workspace_probe)
    facts["workspace_free_bytes"] = disk.free
    add(
        "workspace_capacity",
        disk.free >= expected_platform["minimum_workspace_free_bytes"],
        disk.free,
        "HOST_INCOMPATIBLE",
    )
    add(
        "workspace_inodes",
        workspace_vfs.f_favail >= expected_platform["minimum_free_inodes"],
        workspace_vfs.f_favail,
        "HOST_INCOMPATIBLE",
    )

    docker_version = _run(
        ["docker", "version", "--format", "{{.Server.Version}}"], check=False
    )
    facts["docker_server_version"] = docker_version.stdout.strip()
    add(
        "docker_daemon",
        docker_version.returncode == 0 and bool(docker_version.stdout.strip()),
        (docker_version.stderr or docker_version.stdout).strip(),
        "HOST_INCOMPATIBLE",
    )
    docker_info = _run(
        ["docker", "info", "--format", "{{json .Runtimes}}"], check=False
    )
    runtime_text = docker_info.stdout.strip()
    facts["docker_runtimes"] = runtime_text
    add(
        "nvidia_container_runtime",
        docker_info.returncode == 0 and '"nvidia"' in runtime_text,
        runtime_text or docker_info.stderr.strip(),
        "HOST_INCOMPATIBLE",
    )
    docker_root_result = _run(
        ["docker", "info", "--format", "{{.DockerRootDir}}"], check=False
    )
    docker_root = Path(docker_root_result.stdout.strip() or "/var/lib/docker")
    docker_probe = _existing_ancestor(docker_root)
    docker_free = shutil.disk_usage(docker_probe).free
    facts["docker_root"] = str(docker_root)
    facts["docker_free_bytes"] = docker_free
    add(
        "docker_capacity",
        docker_root_result.returncode == 0
        and docker_free >= expected_platform["minimum_docker_free_bytes"],
        {"root": str(docker_root), "free_bytes": docker_free},
        "HOST_INCOMPATIBLE",
    )
    state_probe = _existing_ancestor(
        Path(manifest["installation"]["provision_state_root"])
    )
    state_free = shutil.disk_usage(state_probe).free
    facts["state_free_bytes"] = state_free
    add(
        "state_capacity",
        state_free >= expected_platform["minimum_state_free_bytes"],
        {"root": str(state_probe), "free_bytes": state_free},
        "HOST_INCOMPATIBLE",
    )

    if not include_hardware:
        failed = [check for check in checks if not check["ok"]]
        return {
            "schema": "njrh.production_preflight.v1",
            "created_at": utc_now(),
            "release_id": manifest["release_id"],
            "scope": "host_only",
            "ok": not failed,
            "facts": facts,
            "checks": checks,
            "failed_codes": sorted({check["code"] for check in failed}),
        }

    can = manifest["hardware"]["can"]
    can_iface = can["interface"]
    can_path = Path("/sys/class/net") / can_iface
    can_details = _run(
        ["ip", "-details", "link", "show", can_iface], check=False
    )
    can_text = f"{can_details.stdout}\n{can_details.stderr}".strip()
    can_exists = can_path.exists() and can_details.returncode == 0
    bitrate_ok = (
        f"bitrate {can['bitrate']}" in can_text
        if "state UP" in can_text or "UP," in can_text
        else True
    )
    up_ok = not can["require_up_during_preflight"] or (
        "state UP" in can_text or "<UP," in can_text
    )
    add(
        "can_interface",
        can_exists and "link/can" in can_text and bitrate_ok and up_ok,
        can_text[-2000:],
        "HARDWARE_INVALID",
    )

    jt128 = manifest["hardware"]["jt128"]
    jt_iface = jt128["interface"]
    address = _run(
        ["ip", "-4", "-o", "addr", "show", "dev", jt_iface], check=False
    )
    route = _run(["ip", "route", "get", jt128["device_ip"]], check=False)
    default_route = _run(["ip", "route", "show", "default"], check=False)
    expected_ip = str(ipaddress.ip_interface(jt128["host_cidr"]).ip)
    address_ok = address.returncode == 0 and expected_ip in address.stdout
    route_ok = route.returncode == 0 and f"dev {jt_iface}" in route.stdout
    dedicated_ok = True
    if jt128["require_dedicated_interface"]:
        dedicated_ok = f" dev {jt_iface} " not in f" {default_route.stdout.strip()} "
    facts["jt128_address"] = address.stdout.strip()
    facts["jt128_route"] = route.stdout.strip()
    add(
        "jt128_network",
        address_ok and route_ok and dedicated_ok,
        {
            "address": address.stdout.strip(),
            "route": route.stdout.strip(),
            "default_route": default_route.stdout.strip(),
        },
        "HARDWARE_INVALID",
    )

    orbbec = manifest["hardware"]["orbbec"]
    orbbec_devices = _orbbec_devices(orbbec["usb_id"])
    facts["orbbec_devices"] = orbbec_devices
    auto_enroll = orbbec["serial"] == "AUTO_ENROLL"
    matching_serials = [
        item
        for item in orbbec_devices
        if auto_enroll or item["serial"] == orbbec["serial"]
    ]
    orbbec_ok = (
        not orbbec["required"]
        or (len(matching_serials) == 1 and len(orbbec_devices) == 1)
    )
    add(
        "orbbec_identity",
        orbbec_ok,
        orbbec_devices,
        "HARDWARE_INVALID",
    )

    strict_hardware = _collect_strict_hardware_preflight(manifest)
    facts["strict_hardware"] = strict_hardware.get("facts", {})
    for check in strict_hardware["checks"]:
        add(
            f"strict_{check.get('name', 'unknown')}",
            bool(check.get("ok")),
            check.get("detail"),
            str(check.get("code") or "HARDWARE_INVALID"),
        )

    failed = [check for check in checks if not check["ok"]]
    return {
        "schema": "njrh.production_preflight.v1",
        "created_at": utc_now(),
        "release_id": manifest["release_id"],
        "ok": not failed,
        "facts": facts,
        "checks": checks,
        "failed_codes": sorted({check["code"] for check in failed}),
    }


def write_report(manifest: dict[str, Any], name: str, report: dict[str, Any]) -> Path:
    report_root = Path(manifest["installation"]["report_root"])
    path = report_root / f"{name}.json"
    try:
        atomic_write_json(path, report, mode=0o644)
    except OSError as exc:
        raise ProvisionError("INSTALL_FAILED", f"cannot write report {path}: {exc}") from exc
    return path


def ensure_preflight(
    manifest: dict[str, Any],
    *,
    include_hardware: bool = True,
    report_name: str = "production_preflight",
) -> dict[str, Any]:
    report = collect_preflight(manifest, include_hardware=include_hardware)
    write_report(manifest, report_name, report)
    if not report["ok"]:
        code = (
            "HARDWARE_INVALID"
            if "HARDWARE_INVALID" in report["failed_codes"]
            else "HOST_INCOMPATIBLE"
        )
        failed_names = [
            item["name"] for item in report["checks"] if not item["ok"]
        ]
        raise ProvisionError(code, f"preflight failed: {', '.join(failed_names)}")
    return report


def _verify_materialized_artifact(
    path: Path,
    *,
    expected_sha256: str,
    expected_size: int | None,
    name: str,
) -> None:
    try:
        metadata = path.lstat()
    except OSError as exc:
        raise ProvisionError(
            "ARTIFACT_INVALID", f"artifact missing for {name}: {path}"
        ) from exc
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
        raise ProvisionError(
            "ARTIFACT_INVALID", f"artifact must be a regular file for {name}: {path}"
        )
    if metadata.st_size > MAX_ARTIFACT_BYTES or (
        expected_size is not None and metadata.st_size != expected_size
    ):
        raise ProvisionError(
            "ARTIFACT_INVALID", f"{name} artifact size does not match its manifest"
        )
    actual = sha256_file(path)
    if actual != expected_sha256:
        raise ProvisionError(
            "ARTIFACT_INVALID",
            f"{name} SHA-256 mismatch: expected {expected_sha256}, got {actual}",
        )


def _resolve_artifact_piece(
    descriptor: dict[str, Any], cache_dir: Path, name: str
) -> tuple[Path, bool]:
    uri = descriptor["uri"]
    expected_sha = descriptor["sha256"]
    expected_size = descriptor.get("size_bytes")
    parsed = urllib.parse.urlparse(uri)
    if parsed.scheme == "file":
        path = Path(urllib.request.url2pathname(parsed.path))
        downloaded = False
    elif parsed.scheme == "https":
        cache_dir.mkdir(parents=True, exist_ok=True)
        suffix = "".join(Path(parsed.path).suffixes[-2:]) or ".artifact"
        path = cache_dir / f"{name}-{expected_sha[:16]}{suffix}"
        downloaded = True
        if not _path_exists(path):
            temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.part")
            try:
                with urllib.request.urlopen(uri, timeout=120) as response:
                    final_url = urllib.parse.urlparse(response.geturl())
                    if final_url.scheme != "https":
                        raise ProvisionError(
                            "ARTIFACT_INVALID",
                            f"download redirected away from HTTPS for {name}",
                        )
                    with temporary.open("wb") as output:
                        total = 0
                        limit = expected_size or MAX_ARTIFACT_BYTES
                        while True:
                            chunk = response.read(8 * 1024 * 1024)
                            if not chunk:
                                break
                            total += len(chunk)
                            if total > limit:
                                raise ProvisionError(
                                    "ARTIFACT_INVALID",
                                    f"download exceeded declared size for {name}",
                                )
                            output.write(chunk)
                        output.flush()
                        os.fsync(output.fileno())
                os.replace(temporary, path)
            except Exception as exc:
                temporary.unlink(missing_ok=True)
                raise ProvisionError(
                    "ARTIFACT_INVALID", f"download failed for {name}: {exc}"
                ) from exc
    else:
        raise ProvisionError("ARTIFACT_INVALID", f"unsupported URI for {name}: {uri}")
    try:
        _verify_materialized_artifact(
            path,
            expected_sha256=expected_sha,
            expected_size=expected_size,
            name=name,
        )
    except ProvisionError:
        if downloaded:
            path.unlink(missing_ok=True)
        raise
    return path, downloaded


def resolve_artifact(
    descriptor: dict[str, Any], cache_dir: Path, name: str
) -> Path:
    parts = descriptor.get("parts")
    if parts is None:
        path, _ = _resolve_artifact_piece(
            {
                "uri": descriptor["uri"],
                "sha256": descriptor["sha256"],
                **(
                    {"size_bytes": descriptor["archive_size_bytes"]}
                    if "archive_size_bytes" in descriptor
                    else {}
                ),
            },
            cache_dir,
            name,
        )
        return path

    cache_dir.mkdir(parents=True, exist_ok=True)
    final_path = cache_dir / f"{name}-{descriptor['sha256'][:16]}.artifact"
    archive_size = descriptor["archive_size_bytes"]
    if _path_exists(final_path):
        _verify_materialized_artifact(
            final_path,
            expected_sha256=descriptor["sha256"],
            expected_size=archive_size,
            name=name,
        )
        return final_path

    temporary = final_path.with_name(
        f".{final_path.name}.{uuid.uuid4().hex}.assembling"
    )
    digest = hashlib.sha256()
    total = 0
    try:
        with temporary.open("xb") as output:
            for index, part in enumerate(parts):
                part_path, downloaded = _resolve_artifact_piece(
                    part, cache_dir, f"{name}-part-{index + 1:04d}"
                )
                try:
                    with part_path.open("rb") as source:
                        while True:
                            chunk = source.read(8 * 1024 * 1024)
                            if not chunk:
                                break
                            total += len(chunk)
                            if total > archive_size:
                                raise ProvisionError(
                                    "ARTIFACT_INVALID",
                                    f"{name} parts exceed the declared archive size",
                                )
                            output.write(chunk)
                            digest.update(chunk)
                finally:
                    if downloaded:
                        part_path.unlink(missing_ok=True)
            output.flush()
            os.fsync(output.fileno())
        if total != archive_size or digest.hexdigest() != descriptor["sha256"]:
            raise ProvisionError(
                "ARTIFACT_INVALID",
                f"{name} reassembled artifact does not match its signed digest",
            )
        os.replace(temporary, final_path)
        _fsync_directory(cache_dir)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise
    return final_path


def install_runtime_image(
    descriptor: dict[str, Any], cache_dir: Path
) -> dict[str, str]:
    pinned = ""
    inspect = _run(
        ["docker", "image", "inspect", "--format", "{{.Id}}", descriptor["reference"]],
        check=False,
    )
    if inspect.returncode != 0 or inspect.stdout.strip() != descriptor["image_id"]:
        if descriptor["kind"] == "oci":
            pinned = descriptor["uri"].removeprefix("oci://")
            _run(["docker", "pull", pinned])
            _run(["docker", "tag", pinned, descriptor["reference"]])
        else:
            archive_path = resolve_artifact(descriptor, cache_dir, "runtime_image")
            _run(["docker", "load", "--input", str(archive_path)], timeout=3600)
    inspect = _run(
        ["docker", "image", "inspect", "--format", "{{.Id}}", descriptor["reference"]]
    )
    actual_id = inspect.stdout.strip()
    if actual_id != descriptor["image_id"]:
        raise ProvisionError(
            "IMAGE_INVALID",
            f"runtime image ID mismatch: expected {descriptor['image_id']}, got {actual_id}",
        )
    return {
        "reference": descriptor["reference"],
        "source": pinned or descriptor["uri"],
        "image_id": actual_id,
    }


def receipt_matches(
    receipt_path: Path,
    release_id: str,
    inputs: dict[str, Any],
    *,
    tree: Path | None = None,
) -> bool:
    if not receipt_path.is_file():
        return False
    try:
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    matches = (
        receipt.get("release_id") == release_id
        and receipt.get("input_digest") == object_digest(inputs)
        and receipt.get("ok") is True
    )
    if not matches:
        return False
    if tree is not None:
        expected = receipt.get("details", {}).get("tree_digest")
        if not isinstance(expected, str) or not SHA256_RE.fullmatch(expected):
            return False
        try:
            return tree_digest(tree) == expected
        except ProvisionError:
            return False
    return True


def write_receipt(
    receipt_path: Path,
    release_id: str,
    inputs: dict[str, Any],
    details: dict[str, Any],
) -> None:
    atomic_write_json(
        receipt_path,
        {
            "schema": "njrh.provision_stage_receipt.v1",
            "release_id": release_id,
            "completed_at": utc_now(),
            "input_digest": object_digest(inputs),
            "ok": True,
            "details": details,
        },
    )


def _path_exists(path: Path) -> bool:
    return path.exists() or path.is_symlink()


def _site_asset_validator(root: Path) -> dict[str, Any]:
    module_path = Path(__file__).with_name("site_assets.py")
    if not module_path.is_file():
        raise ProvisionError("ARTIFACT_INVALID", "site asset validator is missing")
    module = types.ModuleType("njrh_site_assets")
    module.__file__ = str(module_path)
    try:
        source = module_path.read_text(encoding="utf-8")
        exec(compile(source, str(module_path), "exec"), module.__dict__)
        result = module.validate_site_tree(root)
    except ProvisionError:
        raise
    except Exception as exc:
        raise ProvisionError("ARTIFACT_INVALID", f"site asset validation failed: {exc}") from exc
    if not isinstance(result, dict):
        raise ProvisionError("ARTIFACT_INVALID", "site asset validator returned invalid data")
    return result


def stage_payload_for_transaction(
    archive_path: Path,
    destination: Path,
    descriptor: dict[str, Any],
    name: str,
    transaction_id: str,
) -> dict[str, Any]:
    if sha256_file(archive_path) != descriptor["sha256"]:
        raise ProvisionError(
            "ARTIFACT_INVALID", f"{name} changed after artifact verification"
        )
    allow_symlinks = True
    members = inspect_payload_archive(
        archive_path, allow_safe_symlinks=allow_symlinks
    )
    if name == "site_assets":
        forbidden_names = {
            "docking_contact_latch.json",
            ".map_activation_transaction.v1",
            ".map_asset_integrity_degraded.v1",
            "last_navigation_map.json",
            "runtime.env",
            "secrets.env",
            "hardware_acceptance.json",
        }
        for member in members:
            parts = PurePosixPath(member).parts
            if (
                any(part in forbidden_names for part in parts)
                or "drafts" in parts
                or any(part.endswith(".part") for part in parts)
            ):
                raise ProvisionError(
                    "ARTIFACT_INVALID",
                    f"site bundle contains runtime, draft or secret state: {member}",
                )
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = destination.parent / (
        f".{destination.name}.njrh-stage-{transaction_id}"
    )
    if _path_exists(staging):
        raise ProvisionError(
            "INSTALL_FAILED", f"transaction staging path already exists: {staging}"
        )
    staging.mkdir(mode=0o750)
    try:
        _safe_extract_payload(
            archive_path, staging, allow_safe_symlinks=allow_symlinks
        )
        site_validation: dict[str, Any] | None = None
        if name == "site_assets":
            site_validation = _site_asset_validator(staging)
        digest = tree_digest(staging)
        if sha256_file(archive_path) != descriptor["sha256"]:
            raise ProvisionError(
                "ARTIFACT_INVALID", f"{name} changed during extraction"
            )
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return {
        "name": name,
        "destination": str(destination),
        "staging": str(staging),
        "archive_sha256": descriptor["sha256"],
        "members": len(members),
        "tree_digest": digest,
        "site_validation": site_validation,
    }


def _fsync_directory(path: Path) -> None:
    try:
        descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    except OSError:
        return
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _transaction_journal_path(state_dir: Path, transaction_id: str) -> Path:
    return state_dir / "transactions" / transaction_id / "transaction.json"


def _write_transaction_journal(path: Path, journal: dict[str, Any]) -> None:
    atomic_write_json(path, journal, mode=0o600)
    _fsync_directory(path.parent)


def rollback_release_transaction(journal_path: Path) -> dict[str, Any]:
    journal = load_json(journal_path)
    if journal.get("status") == "ROLLED_BACK":
        return journal
    journal["status"] = "ROLLING_BACK"
    journal["rollback_started_at"] = utc_now()
    _write_transaction_journal(journal_path, journal)
    for item in reversed(journal.get("items", [])):
        destination = Path(item["destination"])
        staging = Path(item["staging"])
        backup = Path(item["backup"])
        failed = Path(item["failed"])
        switched = item.get("switched") is True or (
            not _path_exists(staging) and _path_exists(destination)
        )
        if not switched:
            if (
                item.get("had_destination")
                and _path_exists(backup)
                and not _path_exists(destination)
            ):
                os.replace(backup, destination)
                item["rolled_back"] = True
                _fsync_directory(destination.parent)
                _write_transaction_journal(journal_path, journal)
            continue
        if _path_exists(destination):
            if _path_exists(failed):
                failed = failed.with_name(f"{failed.name}-{uuid.uuid4().hex}")
                item["failed"] = str(failed)
            os.replace(destination, failed)
        if item.get("had_destination") and _path_exists(backup):
            os.replace(backup, destination)
        item["rolled_back"] = True
        _fsync_directory(destination.parent)
        _write_transaction_journal(journal_path, journal)
    journal["status"] = "ROLLED_BACK"
    journal["rolled_back_at"] = utc_now()
    _write_transaction_journal(journal_path, journal)
    return journal


def recover_incomplete_transactions(state_dir: Path) -> None:
    transactions = state_dir / "transactions"
    if not transactions.is_dir():
        return
    for journal_path in sorted(transactions.glob("*/transaction.json")):
        try:
            journal = load_json(journal_path)
        except ProvisionError:
            raise ProvisionError(
                "INSTALL_FAILED", f"unreadable transaction journal: {journal_path}"
            )
        if journal.get("status") in (
            "PREPARED",
            "SWITCHING",
            "COMMITTED",
            "ROLLING_BACK",
        ):
            rollback_release_transaction(journal_path)


def commit_release_transaction(
    manifest: dict[str, Any],
    state_dir: Path,
    transaction_id: str,
    prepared_items: list[dict[str, Any]],
) -> tuple[Path, dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for prepared in prepared_items:
        destination = Path(prepared["destination"])
        staging = Path(prepared["staging"])
        backup = destination.parent / (
            f".{destination.name}.njrh-backup-{manifest['release_id']}-{transaction_id}"
        )
        failed = destination.parent / (
            f".{destination.name}.njrh-failed-{manifest['release_id']}-{transaction_id}"
        )
        if _path_exists(backup) or _path_exists(failed):
            raise ProvisionError(
                "INSTALL_FAILED", f"transaction recovery path already exists: {backup}"
            )
        items.append(
            {
                **prepared,
                "backup": str(backup),
                "failed": str(failed),
                "had_destination": _path_exists(destination),
                "switched": False,
            }
        )
    journal = {
        "schema": "njrh.release_transaction.v1",
        "release_id": manifest["release_id"],
        "manifest_digest": object_digest(manifest),
        "transaction_id": transaction_id,
        "created_at": utc_now(),
        "status": "PREPARED",
        "items": items,
    }
    journal_path = _transaction_journal_path(state_dir, transaction_id)
    _write_transaction_journal(journal_path, journal)
    try:
        journal["status"] = "SWITCHING"
        _write_transaction_journal(journal_path, journal)
        for item in journal["items"]:
            destination = Path(item["destination"])
            staging = Path(item["staging"])
            backup = Path(item["backup"])
            if item["had_destination"]:
                os.replace(destination, backup)
            os.replace(staging, destination)
            item["switched"] = True
            _fsync_directory(destination.parent)
            _write_transaction_journal(journal_path, journal)
        journal["status"] = "COMMITTED"
        journal["committed_at"] = utc_now()
        _write_transaction_journal(journal_path, journal)
        return journal_path, journal
    except Exception as exc:
        try:
            rollback_release_transaction(journal_path)
        except Exception as rollback_exc:
            raise ProvisionError(
                "INSTALL_FAILED",
                f"release switch failed ({exc}) and rollback failed ({rollback_exc})",
            ) from rollback_exc
        if isinstance(exc, ProvisionError):
            raise
        raise ProvisionError("INSTALL_FAILED", f"release switch failed: {exc}") from exc


def finalize_release_transaction(journal_path: Path) -> dict[str, Any]:
    journal = load_json(journal_path)
    if journal.get("status") != "COMMITTED":
        raise ProvisionError("INSTALL_FAILED", "cannot finalize an uncommitted transaction")
    journal["status"] = "FINALIZED"
    journal["finalized_at"] = utc_now()
    _write_transaction_journal(journal_path, journal)
    return journal


def read_runtime_packages(workspace: Path, relative_path: str) -> list[str]:
    path = workspace / relative_path
    if not path.is_file():
        raise ProvisionError("BUILD_FAILED", f"package manifest missing: {path}")
    packages = [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    if packages != sorted(set(packages)) or not packages:
        raise ProvisionError(
            "BUILD_FAILED", "runtime package manifest must be sorted and unique"
        )
    return packages


def build_and_test_workspace(
    manifest: dict[str, Any],
    state_dir: Path,
    *,
    upstream_host_override: Path | None = None,
    runtime_overlay_override: Path | None = None,
) -> dict[str, Any]:
    workspace_host = Path(manifest["workspace"]["host_path"])
    workspace_container = manifest["workspace"]["container_path"]
    upstream_host = upstream_host_override or Path(
        manifest["workspace"]["upstream_host_path"]
    )
    upstream_container = manifest["workspace"]["upstream_container_path"]
    release_id = manifest["release_id"]
    release_root_host = workspace_host / ".deploy" / "releases" / release_id
    release_root_container = f"{workspace_container}/.deploy/releases/{release_id}"
    install_host = release_root_host / "install"
    receipt_path = state_dir / "stages" / "build_and_test.json"
    packages = read_runtime_packages(
        workspace_host, manifest["build"]["package_manifest"]
    )
    inputs = {
        "source_commit": manifest["source"]["commit"],
        "runtime_image_id": manifest["artifacts"]["runtime_image"]["image_id"],
        "upstream_sha256": manifest["artifacts"]["upstream_runtime"]["sha256"],
        "overlays_sha256": manifest["artifacts"]["runtime_overlays"]["sha256"],
        "packages": packages,
    }
    if (
        receipt_matches(receipt_path, release_id, inputs, tree=install_host)
        and (install_host / "local_setup.bash").is_file()
    ):
        return {
            "install": str(install_host),
            "tree_digest": tree_digest(install_host),
            "reused": True,
        }

    if release_root_host.exists():
        shutil.rmtree(release_root_host)
    release_root_host.mkdir(parents=True, mode=0o750)
    package_args = " ".join(shlex.quote(item) for item in packages)
    allowlist = (
        f"{workspace_container}/{manifest['build']['pytest_failure_allowlist']}"
    )
    pytest_xml = f"{release_root_container}/pytest-workspace.xml"
    script = f"""
set -euo pipefail
source /opt/ros/humble/setup.bash
if [[ -f {shlex.quote(upstream_container + '/ros2_ws/install/local_setup.bash')} ]]; then
  source {shlex.quote(upstream_container + '/ros2_ws/install/local_setup.bash')}
fi
if [[ -f {shlex.quote(upstream_container + '/install/local_setup.bash')} ]]; then
  source {shlex.quote(upstream_container + '/install/local_setup.bash')}
fi
cd {shlex.quote(workspace_container)}
actual_packages="$(colcon list --names-only | LC_ALL=C sort)"
expected_packages="$(grep -Ev '^[[:space:]]*(#|$)' {shlex.quote(workspace_container + '/' + manifest['build']['package_manifest'])} | LC_ALL=C sort)"
[[ "$actual_packages" == "$expected_packages" ]] || {{
  diff -u <(printf '%s\\n' "$expected_packages") <(printf '%s\\n' "$actual_packages") || true
  echo "colcon package set does not match production manifest" >&2
  exit 41
}}
colcon --log-base {shlex.quote(release_root_container + '/log')} build \
  --build-base {shlex.quote(release_root_container + '/build')} \
  --install-base {shlex.quote(release_root_container + '/install')} \
  --packages-select {package_args} \
  --executor sequential \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON
source {shlex.quote(release_root_container + '/install/local_setup.bash')}
colcon --log-base {shlex.quote(release_root_container + '/test-log')} test \
  --build-base {shlex.quote(release_root_container + '/build')} \
  --install-base {shlex.quote(release_root_container + '/install')} \
  --packages-skip robot_system_tests \
  --executor sequential \
  --event-handlers console_direct+
colcon test-result --test-result-base {shlex.quote(release_root_container + '/build')} --verbose
pytest_rc=0
python3 -m pytest -q src/robot_system_tests/test \
  --junitxml={shlex.quote(pytest_xml)} --disable-warnings || pytest_rc=$?
python3 {shlex.quote(workspace_container + '/scripts/jetson/provision/verify_pytest_allowlist.py')} \
  --junit {shlex.quote(pytest_xml)} \
  --allowlist {shlex.quote(allowlist)} \
  --pytest-exit-code "$pytest_rc"
"""
    command = [
        "docker",
        "run",
        "--rm",
        "--network",
        "none",
        "--ipc",
        "private",
        "--pid",
        "private",
        "--entrypoint",
        "/bin/bash",
        "-v",
        f"{workspace_host}:{workspace_container}:rw",
        "-v",
        f"{upstream_host}:{upstream_container}:ro",
    ]
    if runtime_overlay_override is not None:
        command.extend(
            [
                "-v",
                f"{runtime_overlay_override}:{workspace_container}/.runtime:ro",
            ]
        )
    command.extend(
        [
            manifest["artifacts"]["runtime_image"]["reference"],
            "-c",
            script,
        ]
    )
    result = _run(command, check=False)
    log_path = release_root_host / "factory-build.log"
    log_path.write_text(
        f"{result.stdout}\n{result.stderr}", encoding="utf-8", errors="replace"
    )
    if result.returncode != 0:
        raise ProvisionError(
            "BUILD_FAILED",
            f"isolated build/test failed ({result.returncode}); see {log_path}",
        )
    if not (install_host / "local_setup.bash").is_file():
        raise ProvisionError("BUILD_FAILED", "build did not produce local_setup.bash")
    details = {
        "install": str(install_host),
        "packages": packages,
        "log": str(log_path),
        "tree_digest": tree_digest(install_host),
        "reused": False,
    }
    write_receipt(receipt_path, release_id, inputs, details)
    return details


def stage_install_selector(
    manifest: dict[str, Any], transaction_id: str, install_tree_digest: str
) -> dict[str, Any]:
    workspace = Path(manifest["workspace"]["host_path"])
    release_id = manifest["release_id"]
    target = Path(".deploy") / "releases" / release_id / "install"
    target_abs = workspace / target
    if (
        not (target_abs / "local_setup.bash").is_file()
        or tree_digest(target_abs) != install_tree_digest
    ):
        raise ProvisionError(
            "INSTALL_FAILED", f"release install tree is incomplete or drifted: {target_abs}"
        )
    destination = workspace / "install"
    staging = workspace / f".install.njrh-stage-{transaction_id}"
    if _path_exists(staging):
        raise ProvisionError("INSTALL_FAILED", f"install selector staging exists: {staging}")
    os.symlink(target.as_posix(), staging, target_is_directory=True)
    return {
        "name": "install_selector",
        "destination": str(destination),
        "staging": str(staging),
        "target": target.as_posix(),
        "tree_digest": install_tree_digest,
    }


def validate_secret_file(manifest: dict[str, Any]) -> dict[str, Any]:
    path = Path(manifest["installation"]["secrets_env_file"])
    try:
        metadata = path.lstat()
    except OSError:
        raise ProvisionError("INSTALL_FAILED", f"secret file missing: {path}")
    mode = stat.S_IMODE(metadata.st_mode)
    if (
        stat.S_ISLNK(metadata.st_mode)
        or not stat.S_ISREG(metadata.st_mode)
        or metadata.st_uid != 0
        or metadata.st_nlink != 1
        or mode != 0o600
    ):
        raise ProvisionError(
            "INSTALL_FAILED",
            "secret file must be a root-owned, single-link regular file with mode 0600",
        )
    try:
        content = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise ProvisionError("INSTALL_FAILED", f"cannot read secret file: {exc}") from exc
    if any(
        (ord(char) < 32 and char != "\n") or ord(char) == 127 for char in content
    ):
        raise ProvisionError("INSTALL_FAILED", "secret file contains control characters")
    assignments = [
        line for line in content.splitlines() if line and not line.startswith("#")
    ]
    if len(assignments) != 1 or not assignments[0].startswith("ROBOT_API_TOKEN="):
        raise ProvisionError(
            "INSTALL_FAILED", "secret file must contain exactly one ROBOT_API_TOKEN"
        )
    token = assignments[0].split("=", 1)[1]
    if not token or token.lower() in ("change-me", "changeme", "replace-me"):
        raise ProvisionError("INSTALL_FAILED", "ROBOT_API_TOKEN is not provisioned")
    if token != token.strip():
        raise ProvisionError("INSTALL_FAILED", "ROBOT_API_TOKEN has surrounding whitespace")
    return {"path": str(path), "mode": "0600", "api_token_present": True}


def _detected_orbbec_serial(preflight: dict[str, Any]) -> str:
    devices = (
        preflight.get("facts", {})
        .get("strict_hardware", {})
        .get("orbbec", {})
        .get("devices", [])
    )
    if (
        not isinstance(devices, list)
        or len(devices) != 1
        or not isinstance(devices[0], dict)
        or not isinstance(devices[0].get("serial"), str)
        or not devices[0]["serial"]
    ):
        raise ProvisionError(
            "HARDWARE_INVALID", "cannot resolve one Orbbec serial from strict preflight"
        )
    return devices[0]["serial"]


def enroll_or_verify_device_identity(
    manifest: dict[str, Any],
    preflight: dict[str, Any],
    *,
    allow_enroll: bool,
) -> dict[str, Any]:
    detected_serial = _detected_orbbec_serial(preflight)
    orbbec = manifest["hardware"]["orbbec"]
    expected_serial = orbbec["serial"]
    if expected_serial != "AUTO_ENROLL" and detected_serial != expected_serial:
        raise ProvisionError("HARDWARE_INVALID", "Orbbec serial does not match release")
    path = Path(manifest["installation"]["device_identity_file"])
    if not _path_exists(path):
        if not allow_enroll or expected_serial != "AUTO_ENROLL":
            return {
                "path": "",
                "orbbec_serial": detected_serial,
                "policy": "release_pinned",
            }
        identity = {
            "schema": "njrh.device_identity.v1",
            "orbbec_usb_id": orbbec["usb_id"],
            "orbbec_serial": detected_serial,
            "jetson_model": manifest["platform"]["jetson_model"],
            "tnspec": manifest["platform"]["tnspec"],
            "enrolled_at": utc_now(),
        }
        atomic_write_json(path, identity, mode=0o600)
        os.chown(path, 0, 0)
    try:
        metadata = path.lstat()
    except OSError as exc:
        raise ProvisionError("HARDWARE_INVALID", f"device identity unavailable: {exc}") from exc
    if (
        stat.S_ISLNK(metadata.st_mode)
        or not stat.S_ISREG(metadata.st_mode)
        or metadata.st_uid != 0
        or metadata.st_nlink != 1
        or stat.S_IMODE(metadata.st_mode) != 0o600
    ):
        raise ProvisionError(
            "HARDWARE_INVALID",
            "device identity must be a root-owned single-link regular file mode 0600",
        )
    identity = load_json(path)
    if (
        identity.get("schema") != "njrh.device_identity.v1"
        or identity.get("orbbec_usb_id") != orbbec["usb_id"]
        or identity.get("orbbec_serial") != detected_serial
        or identity.get("jetson_model") != manifest["platform"]["jetson_model"]
        or identity.get("tnspec") != manifest["platform"]["tnspec"]
    ):
        raise ProvisionError(
            "HARDWARE_INVALID",
            "detected hardware does not match the enrolled device identity",
        )
    return {
        "path": str(path),
        "orbbec_serial": detected_serial,
        "policy": "auto_enrolled",
        "enrolled_at": identity.get("enrolled_at", ""),
    }


def stage_device_identity(
    manifest: dict[str, Any],
    preflight: dict[str, Any],
    transaction_id: str,
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    path = Path(manifest["installation"]["device_identity_file"])
    if _path_exists(path) or manifest["hardware"]["orbbec"]["serial"] != "AUTO_ENROLL":
        return (
            enroll_or_verify_device_identity(
                manifest, preflight, allow_enroll=False
            ),
            None,
        )
    detected_serial = _detected_orbbec_serial(preflight)
    identity = {
        "schema": "njrh.device_identity.v1",
        "orbbec_usb_id": manifest["hardware"]["orbbec"]["usb_id"],
        "orbbec_serial": detected_serial,
        "jetson_model": manifest["platform"]["jetson_model"],
        "tnspec": manifest["platform"]["tnspec"],
        "enrolled_at": utc_now(),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.with_name(f".{path.name}.njrh-stage-{transaction_id}")
    if _path_exists(staging):
        raise ProvisionError(
            "INSTALL_FAILED", f"device identity staging exists: {staging}"
        )
    atomic_write_json(staging, identity, mode=0o600)
    os.chown(staging, 0, 0)
    result = {
        "path": str(path),
        "orbbec_serial": detected_serial,
        "policy": "auto_enrolled",
        "enrolled_at": identity["enrolled_at"],
    }
    item = {
        "name": "device_identity",
        "destination": str(path),
        "staging": str(staging),
        "sha256": sha256_file(staging),
    }
    return result, item


def migrate_legacy_secret(manifest: dict[str, Any]) -> dict[str, Any]:
    secret_path = Path(manifest["installation"]["secrets_env_file"])
    if _path_exists(secret_path):
        return validate_secret_file(manifest)
    runtime_path = Path(manifest["installation"]["runtime_env_file"])
    if runtime_path.is_symlink() or not runtime_path.is_file():
        raise ProvisionError(
            "INSTALL_FAILED",
            f"create {secret_path} with a unique ROBOT_API_TOKEN before provisioning",
        )
    token_lines = [
        line
        for line in runtime_path.read_text(encoding="utf-8").splitlines()
        if line.startswith("ROBOT_API_TOKEN=")
    ]
    if len(token_lines) != 1:
        raise ProvisionError(
            "INSTALL_FAILED",
            f"create {secret_path} with a unique ROBOT_API_TOKEN before provisioning",
        )
    token = token_lines[0].split("=", 1)[1]
    if not token or token.lower() in ("change-me", "changeme", "replace-me"):
        raise ProvisionError("INSTALL_FAILED", "legacy ROBOT_API_TOKEN is not usable")
    if token != token.strip() or any(
        ord(char) < 33 or ord(char) == 127 for char in token
    ):
        raise ProvisionError(
            "INSTALL_FAILED", "legacy ROBOT_API_TOKEN contains unsafe characters"
        )
    secret_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(
        secret_path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0),
        0o600,
    )
    with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
        stream.write(f"ROBOT_API_TOKEN={token}\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.chown(secret_path, 0, 0)
    os.chmod(secret_path, 0o600)
    return validate_secret_file(manifest)


def runtime_env_values(
    manifest: dict[str, Any], *, orbbec_serial: str | None = None
) -> dict[str, str]:
    workspace = manifest["workspace"]
    hardware = manifest["hardware"]
    image = manifest["artifacts"]["runtime_image"]
    return {
        "NJRH_PRODUCTION_RELEASE_ID": manifest["release_id"],
        "NJRH_WORKSPACE_HOST": workspace["host_path"],
        "NJRH_WORKSPACE_CONTAINER": workspace["container_path"],
        "NJRH_UPSTREAM_WORKSPACE_HOST": workspace["upstream_host_path"],
        "NJRH_UPSTREAM_WORKSPACE_CONTAINER": workspace["upstream_container_path"],
        "NJRH_CONTAINER_NAME": manifest["installation"]["container_name"],
        "NJRH_RUNTIME_USER": manifest["installation"]["runtime_user"],
        "NJRH_IMAGE_NAME": image["reference"],
        "NJRH_EXPECTED_IMAGE_ID": image["image_id"],
        "NJRH_ALLOW_BASE_IMAGE_FALLBACK": "false",
        "NJRH_PROVISION_MOTION_LOCK": manifest["installation"]["motion_lock"],
        "CAN_IFACE": hardware["can"]["interface"],
        "CAN_BITRATE": str(hardware["can"]["bitrate"]),
        "NJRH_ORBBEC_SERIAL_NUMBER": (
            orbbec_serial or hardware["orbbec"]["serial"]
        ),
        "NJRH_NAV2_PLANNER_PROFILE": "ranger_lattice",
        "NJRH_NAV_LOCAL_STATE_MODE": "ekf",
        "NJRH_LOCAL_STATE_EKF_PROFILE": "wheel_spin_imu",
        "LOCAL_STATE_EKF_PROFILE": "wheel_spin_imu",
        "NJRH_AMCL_LOCALIZATION_MODE": "gated",
        "NJRH_REUSE_COMMON_SERVICES": "true",
        "NJRH_DOCKING_SENSOR_BACKEND": "orbbec_336l",
        "NJRH_GS2_AUTOSTART": "false",
        "NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION": "true",
        "NJRH_AMCL_READINESS_BEFORE_NAV2_LIFECYCLE": "true",
        "NJRH_COMMON_LOCAL_STATE_BACKGROUND_START": "true",
        "NJRH_COMMON_LOCAL_STATE_START_READY_MODE": "endpoint",
        "NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START": "false",
        "NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE": "true",
        "NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK": "false",
        "NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST": "false",
        "NJRH_NAV2_LIFECYCLE_PARALLEL_BT": "true",
        "NJRH_NAV2_LIFECYCLE_PARALLEL_CORE": "false",
        "NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION": "true",
        "NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE": "once",
        "NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE": "false",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4",
    }


def stage_runtime_env(
    manifest: dict[str, Any],
    transaction_id: str,
    *,
    orbbec_serial: str | None = None,
) -> dict[str, Any]:
    path = Path(manifest["installation"]["runtime_env_file"])
    values = runtime_env_values(manifest, orbbec_serial=orbbec_serial)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.njrh-stage-{transaction_id}")
    if _path_exists(temporary):
        raise ProvisionError("INSTALL_FAILED", f"runtime env staging exists: {temporary}")
    temporary.write_text(
        "".join(f"{key}={value}\n" for key, value in sorted(values.items())),
        encoding="utf-8",
    )
    os.chmod(temporary, 0o640)
    os.chown(temporary, 0, 0)
    return {
        "name": "runtime_env",
        "destination": str(path),
        "staging": str(temporary),
        "sha256": sha256_file(temporary),
        "mode": "0640",
        "keys": sorted(values),
    }


def install_systemd_disabled(manifest: dict[str, Any]) -> dict[str, Any]:
    workspace = Path(manifest["workspace"]["host_path"])
    env = os.environ.copy()
    env.update(
        {
            "NJRH_WORKSPACE_HOST": manifest["workspace"]["host_path"],
            "NJRH_WORKSPACE_CONTAINER": manifest["workspace"]["container_path"],
            "NJRH_UPSTREAM_WORKSPACE_HOST": manifest["workspace"][
                "upstream_host_path"
            ],
            "NJRH_UPSTREAM_WORKSPACE_CONTAINER": manifest["workspace"][
                "upstream_container_path"
            ],
            "NJRH_CONTAINER_NAME": manifest["installation"]["container_name"],
            "NJRH_AUTOSTART_ENV_FILE": manifest["installation"]["runtime_env_file"],
            "NJRH_SECRETS_ENV_FILE": manifest["installation"]["secrets_env_file"],
            "NJRH_PROVISION_MOTION_LOCK": manifest["installation"]["motion_lock"],
            "NJRH_SKIP_RUNTIME_ENV_WRITE": "true",
            "NJRH_HOST_SERVICE_USER": "nvidia",
            "CAN_IFACE": manifest["hardware"]["can"]["interface"],
            "CAN_BITRATE": str(manifest["hardware"]["can"]["bitrate"]),
        }
    )
    _run(
        ["bash", "scripts/jetson/install_njrh_autostart.sh", "install-disabled"],
        cwd=workspace,
        env=env,
    )
    return {
        "runtime_service": "njrh-runtime.service",
        "can_service": "njrh-can.service",
        "enabled": False,
    }


def prepare_can_for_preflight(manifest: dict[str, Any]) -> dict[str, Any]:
    workspace = Path(manifest["workspace"]["host_path"])
    script = workspace / "scripts/jetson/runtime_overlay/scripts/bringup_ranger_can.sh"
    if script.is_symlink() or not script.is_file():
        raise ProvisionError("HARDWARE_INVALID", f"CAN bringup script missing: {script}")
    env = os.environ.copy()
    env.update(
        {
            "CAN_IFACE": manifest["hardware"]["can"]["interface"],
            "CAN_BITRATE": str(manifest["hardware"]["can"]["bitrate"]),
        }
    )
    result = _run(["bash", str(script)], env=env, check=False, timeout=30)
    if result.returncode != 0:
        raise ProvisionError(
            "HARDWARE_INVALID",
            f"failed to configure CAN for passive preflight: {result.stderr[-2000:]}",
        )
    return {"configured": True, "detail": result.stdout[-2000:]}


@contextlib.contextmanager
def provision_lock() -> Any:
    if fcntl is None:
        raise ProvisionError("HOST_INCOMPATIBLE", "flock is unavailable")
    path = Path("/run/lock/njrh-provision.lock")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+", encoding="utf-8") as stream:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ProvisionError("LOCKED", "another provisioner is running") from exc
        yield


def _set_services_locked(*, strict: bool = False) -> dict[str, Any]:
    results: dict[str, Any] = {"services": {}, "container_stopped": False}
    for service in ("njrh-runtime.service", "njrh-can.service"):
        disabled = _run(
            ["systemctl", "disable", "--now", service], check=False, timeout=30
        )
        active = _run(["systemctl", "is-active", service], check=False, timeout=10)
        state = active.stdout.strip() or active.stderr.strip()
        results["services"][service] = {
            "disable_returncode": disabled.returncode,
            "active_state": state,
        }
        if strict and state == "active":
            raise ProvisionError("LOCKED", f"failed to stop active service: {service}")

    container = _run(
        [
            "docker",
            "ps",
            "--filter",
            "name=^/NJRH-car$",
            "--format",
            "{{.Names}}",
        ],
        check=False,
    )
    if container.returncode == 0 and container.stdout.strip() == "NJRH-car":
        stopped = _run(["docker", "stop", "--time", "30", "NJRH-car"], check=False, timeout=45)
        results["container_stopped"] = stopped.returncode == 0
        if strict and stopped.returncode != 0:
            raise ProvisionError("LOCKED", "failed to stop running NJRH-car container")
    remaining = _run(
        [
            "docker",
            "ps",
            "--filter",
            "name=^/NJRH-car$",
            "--format",
            "{{.Names}}",
        ],
        check=False,
    )
    if strict and remaining.stdout.strip():
        raise ProvisionError("LOCKED", "NJRH-car container is still running")
    return results


def _state_paths(manifest: dict[str, Any]) -> tuple[Path, Path, Path]:
    root = Path(manifest["installation"]["provision_state_root"])
    release_dir = root / manifest["release_id"]
    return root, release_dir, release_dir / "state.json"


def update_state(
    manifest: dict[str, Any],
    status: str,
    stage: str,
    *,
    error: ProvisionError | None = None,
    details: dict[str, Any] | None = None,
) -> dict[str, Any]:
    root, release_dir, state_path = _state_paths(manifest)
    state = {
        "schema": "njrh.provision_state.v1",
        "release_id": manifest["release_id"],
        "manifest_digest": object_digest(manifest),
        "updated_at": utc_now(),
        "status": status,
        "stage": stage,
        "motion_locked": Path(manifest["installation"]["motion_lock"]).exists(),
        "details": details or {},
    }
    if error is not None:
        state["error"] = {"code": error.code, "message": error.message}
    atomic_write_json(state_path, state)
    atomic_write_json(root / "current.json", state)
    return state


def apply_manifest(manifest: dict[str, Any], manifest_path: Path) -> dict[str, Any]:
    validate_manifest(manifest, allow_template=False)
    if os.geteuid() != 0:
        raise ProvisionError("INSTALL_FAILED", "apply must run as root")
    workspace = Path(manifest["workspace"]["host_path"])
    source = verify_source(workspace, manifest["source"])
    trust = verify_release_trust(manifest, manifest_path, workspace)
    _, state_dir, _ = _state_paths(manifest)
    state_dir.mkdir(parents=True, exist_ok=True)
    (state_dir / "stages").mkdir(exist_ok=True)
    write_motion_lock(manifest, "production_apply")
    services_locked = _set_services_locked(strict=True)
    recover_incomplete_transactions(state_dir)
    update_state(
        manifest,
        "APPLYING_LOCKED",
        "SOURCE_AND_TRUST_VERIFIED",
        details={"source": source, "trust": trust, "services_locked": services_locked},
    )

    update_state(manifest, "APPLYING_LOCKED", "HOST_PREFLIGHT")
    host_preflight = ensure_preflight(
        manifest,
        include_hardware=False,
        report_name="production_host_preflight",
    )

    update_state(manifest, "APPLYING_LOCKED", "RUNTIME_IMAGE")
    image = install_runtime_image(
        manifest["artifacts"]["runtime_image"], state_dir / "artifacts"
    )
    prepare_can = prepare_can_for_preflight(manifest)
    update_state(manifest, "APPLYING_LOCKED", "STRICT_HARDWARE_PREFLIGHT")
    hardware_preflight = ensure_preflight(
        manifest,
        include_hardware=True,
        report_name="production_hardware_preflight",
    )
    cache = state_dir / "artifacts"
    transaction_id = uuid.uuid4().hex
    payload_results: dict[str, Any] = {}
    prepared_items: list[dict[str, Any]] = []
    for name in ("upstream_runtime", "runtime_overlays", "site_assets"):
        descriptor = manifest["artifacts"][name]
        if not descriptor["required"] and not descriptor["uri"]:
            payload_results[name] = {"skipped": True}
            continue
        update_state(manifest, "APPLYING_LOCKED", f"PAYLOAD_{name.upper()}")
        artifact_path = resolve_artifact(descriptor, cache, name)
        prepared = stage_payload_for_transaction(
            artifact_path,
            Path(descriptor["destination"]),
            descriptor,
            name,
            transaction_id,
        )
        payload_results[name] = prepared
        prepared_items.append(prepared)

    update_state(manifest, "APPLYING_LOCKED", "FULL_BUILD_AND_TEST")
    upstream_stage = Path(payload_results["upstream_runtime"]["staging"])
    overlay_stage = Path(payload_results["runtime_overlays"]["staging"])
    build = build_and_test_workspace(
        manifest,
        state_dir,
        upstream_host_override=upstream_stage,
        runtime_overlay_override=overlay_stage,
    )
    device_identity, device_identity_item = stage_device_identity(
        manifest, hardware_preflight, transaction_id
    )
    if device_identity_item is not None:
        prepared_items.append(device_identity_item)
    update_state(manifest, "APPLYING_LOCKED", "SECRET_AND_SYSTEMD_PREPARE")
    secret = migrate_legacy_secret(manifest)
    systemd = install_systemd_disabled(manifest)
    install = stage_install_selector(
        manifest, transaction_id, build["tree_digest"]
    )
    runtime_env = stage_runtime_env(
        manifest,
        transaction_id,
        orbbec_serial=device_identity["orbbec_serial"],
    )
    prepared_items.extend([install, runtime_env])
    verify_source(workspace, manifest["source"])

    journal_path: Path | None = None
    try:
        update_state(manifest, "APPLYING_LOCKED", "ATOMIC_RELEASE_SWITCH")
        journal_path, transaction = commit_release_transaction(
            manifest, state_dir, transaction_id, prepared_items
        )
        for name, payload in payload_results.items():
            if payload.get("skipped"):
                continue
            destination = Path(payload["destination"])
            if tree_digest(destination) != payload["tree_digest"]:
                raise ProvisionError(
                    "VERIFY_FAILED", f"{name} tree changed during release switch"
                )
        expected_install_target = install["target"]
        install_path = Path(install["destination"])
        if (
            not install_path.is_symlink()
            or os.readlink(install_path) != expected_install_target
            or tree_digest(install_path.resolve()) != build["tree_digest"]
        ):
            raise ProvisionError("VERIFY_FAILED", "active install selector is incorrect")
        runtime_env_path = Path(runtime_env["destination"])
        if sha256_file(runtime_env_path) != runtime_env["sha256"]:
            raise ProvisionError("VERIFY_FAILED", "runtime env changed during release switch")
        verified_identity = enroll_or_verify_device_identity(
            manifest, hardware_preflight, allow_enroll=False
        )
        if verified_identity["orbbec_serial"] != device_identity["orbbec_serial"]:
            raise ProvisionError(
                "VERIFY_FAILED", "device identity changed during release switch"
            )

        has_site_assets = not payload_results["site_assets"].get("skipped", False)
        final_status = "READY_LOCKED" if has_site_assets else "READY_NO_MAP_LOCKED"
        details = {
            "source": source,
            "trust": trust,
            "host_preflight": host_preflight,
            "hardware_preflight": hardware_preflight,
            "device_identity": device_identity,
            "can_prepare": prepare_can,
            "image": image,
            "payloads": payload_results,
            "build": build,
            "install": install,
            "secret": secret,
            "runtime_env": runtime_env,
            "systemd": systemd,
            "transaction": {
                "id": transaction_id,
                "journal": str(journal_path),
                "status": transaction["status"],
            },
        }
        state = update_state(manifest, final_status, "COMPLETE", details=details)
        write_report(manifest, "production_deployment", state)
        finalize_release_transaction(journal_path)
        return state
    except Exception:
        if journal_path is not None:
            try:
                journal = load_json(journal_path)
                if journal.get("status") in ("COMMITTED", "SWITCHING"):
                    rollback_release_transaction(journal_path)
            except Exception:
                pass
        raise


def validate_hardware_acceptance(manifest: dict[str, Any]) -> dict[str, Any]:
    path = Path(manifest["installation"]["hardware_acceptance_file"])
    try:
        metadata = path.lstat()
    except OSError as exc:
        raise ProvisionError(
            "LOCKED",
            f"hardware acceptance is missing: {path}; factory inspection must sign off first",
        ) from exc
    if (
        stat.S_ISLNK(metadata.st_mode)
        or not stat.S_ISREG(metadata.st_mode)
        or metadata.st_uid != 0
        or metadata.st_nlink != 1
        or stat.S_IMODE(metadata.st_mode) != 0o600
    ):
        raise ProvisionError(
            "LOCKED",
            "hardware acceptance must be a root-owned single-link regular file mode 0600",
        )
    acceptance = load_json(path)
    if (
        acceptance.get("schema") != "njrh.hardware_acceptance.v1"
        or acceptance.get("release_id") != manifest["release_id"]
        or acceptance.get("manifest_digest") != object_digest(manifest)
        or acceptance.get("accepted") is not True
        or not isinstance(acceptance.get("inspection_id"), str)
        or not acceptance["inspection_id"]
        or not isinstance(acceptance.get("accepted_at"), str)
        or not acceptance["accepted_at"]
    ):
        raise ProvisionError(
            "LOCKED", "hardware acceptance does not bind this exact production release"
        )
    return {
        "path": str(path),
        "inspection_id": acceptance["inspection_id"],
        "accepted_at": acceptance["accepted_at"],
    }


def record_hardware_acceptance(
    manifest: dict[str, Any],
    manifest_path: Path,
    inspection_id: str,
    confirmed: bool,
) -> dict[str, Any]:
    if not confirmed:
        raise ProvisionError(
            "LOCKED", "hardware acceptance requires --confirm-hardware-inspected"
        )
    if os.geteuid() != 0:
        raise ProvisionError("INSTALL_FAILED", "hardware acceptance must run as root")
    if not SAFE_ID_RE.fullmatch(inspection_id):
        raise ProvisionError("MANIFEST_INVALID", "inspection_id is not safe")
    _, _, state_path = _state_paths(manifest)
    state = load_json(state_path)
    if (
        state.get("status") != "READY_LOCKED"
        or state.get("release_id") != manifest["release_id"]
        or state.get("manifest_digest") != object_digest(manifest)
    ):
        raise ProvisionError("LOCKED", "only an exact READY_LOCKED release can be accepted")
    prepare_can_for_preflight(manifest)
    verification = verify_deployment(manifest, manifest_path)
    path = Path(manifest["installation"]["hardware_acceptance_file"])
    acceptance = {
        "schema": "njrh.hardware_acceptance.v1",
        "release_id": manifest["release_id"],
        "manifest_digest": object_digest(manifest),
        "inspection_id": inspection_id,
        "accepted": True,
        "accepted_at": utc_now(),
        "accepted_by": os.environ.get("SUDO_USER") or os.environ.get("USER") or "root",
        "verification_digest": object_digest(verification),
    }
    atomic_write_json(path, acceptance, mode=0o600)
    os.chown(path, 0, 0)
    return validate_hardware_acceptance(manifest)


def verify_deployment(
    manifest: dict[str, Any], manifest_path: Path
) -> dict[str, Any]:
    validate_manifest(manifest, allow_template=False)
    workspace = Path(manifest["workspace"]["host_path"])
    checks: list[dict[str, Any]] = []

    def add(name: str, ok: bool, detail: Any) -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    source = verify_source(workspace, manifest["source"])
    add("source", True, source)
    trust = verify_release_trust(manifest, manifest_path, workspace)
    add("trust", True, trust)
    _, _, state_path = _state_paths(manifest)
    state = load_json(state_path)
    state_matches = (
        state.get("release_id") == manifest["release_id"]
        and state.get("manifest_digest") == object_digest(manifest)
        and state.get("status") in ("READY_LOCKED", "READY_NO_MAP_LOCKED", "ACTIVE")
    )
    add(
        "state_binding",
        state_matches,
        {
            "release_id": state.get("release_id"),
            "manifest_digest": state.get("manifest_digest"),
            "status": state.get("status"),
        },
    )
    preflight = collect_preflight(manifest)
    add("preflight", preflight["ok"], preflight["failed_codes"])
    try:
        device_identity = enroll_or_verify_device_identity(
            manifest, preflight, allow_enroll=False
        )
        add("device_identity", True, device_identity)
    except ProvisionError as exc:
        add("device_identity", False, exc.message)
    image = _run(
        [
            "docker",
            "image",
            "inspect",
            "--format",
            "{{.Id}}",
            manifest["artifacts"]["runtime_image"]["reference"],
        ],
        check=False,
    )
    add(
        "runtime_image",
        image.returncode == 0
        and image.stdout.strip() == manifest["artifacts"]["runtime_image"]["image_id"],
        image.stdout.strip() or image.stderr.strip(),
    )
    install = workspace / "install"
    expected_target = (
        Path(".deploy") / "releases" / manifest["release_id"] / "install"
    ).as_posix()
    build_digest = state.get("details", {}).get("build", {}).get("tree_digest")
    install_digest = ""
    if install.is_symlink() and (install / "local_setup.bash").is_file():
        try:
            install_digest = tree_digest(install.resolve())
        except ProvisionError:
            install_digest = ""
    add(
        "install",
        install.is_symlink()
        and os.readlink(install) == expected_target
        and isinstance(build_digest, str)
        and install_digest == build_digest,
        {
            "target": os.readlink(install) if install.is_symlink() else "",
            "expected_target": expected_target,
            "tree_digest": install_digest,
            "expected_tree_digest": build_digest,
        },
    )
    payload_checks: dict[str, Any] = {}
    for name in ("upstream_runtime", "runtime_overlays", "site_assets"):
        payload = state.get("details", {}).get("payloads", {}).get(name, {})
        if payload.get("skipped"):
            payload_checks[name] = {"skipped": True, "ok": name == "site_assets"}
            continue
        destination = Path(payload.get("destination", ""))
        try:
            actual_digest = tree_digest(destination)
        except (ProvisionError, OSError):
            actual_digest = ""
        ok = actual_digest == payload.get("tree_digest")
        site_validation: dict[str, Any] | None = None
        if ok and name == "site_assets":
            try:
                site_validation = _site_asset_validator(destination)
            except ProvisionError:
                ok = False
        payload_checks[name] = {
            "ok": ok,
            "destination": str(destination),
            "tree_digest": actual_digest,
            "expected_tree_digest": payload.get("tree_digest"),
            "site_validation": site_validation,
        }
    add(
        "payload_trees",
        all(item.get("ok") for item in payload_checks.values()),
        payload_checks,
    )
    runtime_env = Path(manifest["installation"]["runtime_env_file"])
    expected_runtime_sha = (
        state.get("details", {}).get("runtime_env", {}).get("sha256")
    )
    expected_orbbec_serial = (
        state.get("details", {}).get("device_identity", {}).get("orbbec_serial")
    )
    runtime_env_ok = False
    runtime_env_detail: dict[str, Any] = {"path": str(runtime_env)}
    try:
        runtime_metadata = runtime_env.lstat()
        runtime_text = runtime_env.read_text(encoding="utf-8")
        actual_runtime_sha = sha256_file(runtime_env)
        runtime_env_ok = (
            stat.S_ISREG(runtime_metadata.st_mode)
            and not stat.S_ISLNK(runtime_metadata.st_mode)
            and runtime_metadata.st_uid == 0
            and stat.S_IMODE(runtime_metadata.st_mode) == 0o640
            and actual_runtime_sha == expected_runtime_sha
            and "ROBOT_API_TOKEN=" not in runtime_text
            and "NJRH_NAV2_PLANNER_PROFILE=ranger_lattice" in runtime_text
            and "NJRH_LOCAL_STATE_EKF_PROFILE=wheel_spin_imu" in runtime_text
            and "NJRH_AMCL_LOCALIZATION_MODE=gated" in runtime_text
            and isinstance(expected_orbbec_serial, str)
            and f"NJRH_ORBBEC_SERIAL_NUMBER={expected_orbbec_serial}" in runtime_text
        )
        runtime_env_detail.update(
            {
                "sha256": actual_runtime_sha,
                "expected_sha256": expected_runtime_sha,
            }
        )
    except (OSError, UnicodeDecodeError):
        pass
    add("runtime_env", runtime_env_ok, runtime_env_detail)
    try:
        secret = validate_secret_file(manifest)
        add("secret", True, secret)
    except ProvisionError as exc:
        add("secret", False, exc.message)
    transaction_path_text = (
        state.get("details", {}).get("transaction", {}).get("journal", "")
    )
    transaction_status = ""
    if transaction_path_text:
        try:
            transaction_status = load_json(Path(transaction_path_text)).get("status", "")
        except ProvisionError:
            transaction_status = ""
    add("transaction_finalized", transaction_status == "FINALIZED", transaction_status)
    motion_lock = Path(manifest["installation"]["motion_lock"])
    locked_state = state.get("status") != "ACTIVE"
    add(
        "motion_lock",
        motion_lock.is_file() if locked_state else not motion_lock.exists(),
        {"path": str(motion_lock), "expected_locked": locked_state},
    )
    enabled: dict[str, str] = {}
    active: dict[str, str] = {}
    for service in ("njrh-runtime.service", "njrh-can.service"):
        result = _run(["systemctl", "is-enabled", service], check=False)
        enabled[service] = result.stdout.strip() or result.stderr.strip()
        result = _run(["systemctl", "is-active", service], check=False)
        active[service] = result.stdout.strip() or result.stderr.strip()
    add(
        "service_state",
        (
            all(value in ("disabled", "static") for value in enabled.values())
            and all(value != "active" for value in active.values())
            if locked_state
            else all(value == "enabled" for value in enabled.values())
            and all(value == "active" for value in active.values())
        ),
        {"enabled": enabled, "active": active, "expected_locked": locked_state},
    )
    failed = [item["name"] for item in checks if not item["ok"]]
    report = {
        "schema": "njrh.production_verify.v1",
        "created_at": utc_now(),
        "release_id": manifest["release_id"],
        "ok": not failed,
        "checks": checks,
    }
    write_report(manifest, "production_verify", report)
    if failed:
        raise ProvisionError("VERIFY_FAILED", f"verification failed: {', '.join(failed)}")
    return report


def activate_manifest(
    manifest: dict[str, Any], manifest_path: Path, confirmed: bool
) -> dict[str, Any]:
    validate_manifest(manifest, allow_template=False)
    if not confirmed:
        raise ProvisionError(
            "LOCKED", "activation requires --confirm-motion-enabled"
        )
    if os.geteuid() != 0:
        raise ProvisionError("INSTALL_FAILED", "activate must run as root")
    _, _, state_path = _state_paths(manifest)
    state = load_json(state_path)
    if (
        state.get("status") != "READY_LOCKED"
        or state.get("release_id") != manifest["release_id"]
        or state.get("manifest_digest") != object_digest(manifest)
    ):
        raise ProvisionError(
            "LOCKED", f"release is not ready for activation: {state.get('status')}"
        )
    acceptance = validate_hardware_acceptance(manifest)
    prepare_can_for_preflight(manifest)
    verification = verify_deployment(manifest, manifest_path)
    motion_lock = Path(manifest["installation"]["motion_lock"])
    motion_lock.unlink(missing_ok=True)
    try:
        _run(["systemctl", "enable", "--now", "njrh-can.service"], timeout=180)
        _run(["systemctl", "enable", "--now", "njrh-runtime.service"], timeout=300)
        for service in ("njrh-can.service", "njrh-runtime.service"):
            active = _run(["systemctl", "is-active", service], check=False, timeout=20)
            if active.stdout.strip() != "active":
                raise ProvisionError(
                    "VERIFY_FAILED", f"service did not become active: {service}"
                )
        workspace_container = manifest["workspace"]["container_path"]
        readiness = _run(
            [
                "docker",
                "exec",
                "-u",
                manifest["installation"]["runtime_user"],
                "--workdir",
                f"{workspace_container}/scripts/jetson/runtime_overlay",
                manifest["installation"]["container_name"],
                "bash",
                "scripts/check_commercial_runtime_ready.sh",
            ],
            check=False,
            timeout=360,
        )
        if readiness.returncode != 0:
            raise ProvisionError(
                "VERIFY_FAILED",
                "commercial runtime readiness failed: "
                + (readiness.stderr or readiness.stdout)[-4000:],
            )
        active_state = update_state(
            manifest,
            "ACTIVE",
            "ACTIVATED",
            details={
                **state.get("details", {}),
                "activation": {
                    "hardware_acceptance": acceptance,
                    "pre_activation_verify": verification,
                    "commercial_runtime_ready": True,
                },
            },
        )
    except Exception:
        write_motion_lock(manifest, "activation_failure")
        _set_services_locked(strict=True)
        raise
    return active_state


def status_document(state_root: Path) -> dict[str, Any]:
    current = state_root / "current.json"
    if current.is_file():
        state = load_json(current)
    else:
        state = {
            "schema": "njrh.provision_state.v1",
            "status": "UNPROVISIONED",
            "stage": "NONE",
        }
    state["motion_lock_exists"] = (state_root / "motion.lock").exists()
    return state


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="action", required=True)

    validate = subparsers.add_parser("validate")
    validate.add_argument("manifest", type=Path)
    validate.add_argument("--allow-template", action="store_true")

    verify_bundle = subparsers.add_parser("verify-bundle")
    verify_bundle.add_argument("manifest", type=Path)

    preflight = subparsers.add_parser("preflight")
    preflight.add_argument("manifest", type=Path)
    preflight.add_argument("--allow-template", action="store_true")

    apply = subparsers.add_parser("apply")
    apply.add_argument("manifest", type=Path)

    verify = subparsers.add_parser("verify")
    verify.add_argument("manifest", type=Path)

    activate = subparsers.add_parser("activate")
    activate.add_argument("manifest", type=Path)
    activate.add_argument("--confirm-motion-enabled", action="store_true")

    accept = subparsers.add_parser("accept-hardware")
    accept.add_argument("manifest", type=Path)
    accept.add_argument("--inspection-id", required=True)
    accept.add_argument("--confirm-hardware-inspected", action="store_true")

    status = subparsers.add_parser("status")
    status.add_argument(
        "--state-root", type=Path, default=Path("/var/lib/njrh/provision")
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    manifest: dict[str, Any] | None = None
    validated_for_mutation = False
    try:
        if args.action == "status":
            print(json.dumps(status_document(args.state_root), indent=2, ensure_ascii=False))
            return 0
        manifest = load_json(args.manifest)
        if args.action == "validate":
            validate_manifest(manifest, allow_template=args.allow_template)
            print(
                json.dumps(
                    {
                        "ok": True,
                        "release_id": manifest["release_id"],
                        "manifest_digest": object_digest(manifest),
                    },
                    indent=2,
                )
            )
            return 0
        if args.action == "verify-bundle":
            result = verify_release_bundle(manifest, args.manifest)
            print(json.dumps(result, indent=2, ensure_ascii=False))
            return 0
        if args.action == "preflight":
            validate_manifest(manifest, allow_template=args.allow_template)
            report = ensure_preflight(manifest)
            print(json.dumps(report, indent=2, ensure_ascii=False))
            return 0
        if args.action == "apply":
            validate_manifest(manifest, allow_template=False)
            validated_for_mutation = True
            with provision_lock():
                result = apply_manifest(manifest, args.manifest)
        elif args.action == "verify":
            validate_manifest(manifest, allow_template=False)
            result = verify_deployment(manifest, args.manifest)
        elif args.action == "activate":
            validate_manifest(manifest, allow_template=False)
            validated_for_mutation = True
            with provision_lock():
                result = activate_manifest(
                    manifest, args.manifest, args.confirm_motion_enabled
                )
        elif args.action == "accept-hardware":
            validate_manifest(manifest, allow_template=False)
            with provision_lock():
                result = record_hardware_acceptance(
                    manifest,
                    args.manifest,
                    args.inspection_id,
                    args.confirm_hardware_inspected,
                )
        else:
            raise ProvisionError("MANIFEST_INVALID", f"unsupported action: {args.action}")
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0
    except ProvisionError as exc:
        if (
            manifest is not None
            and validated_for_mutation
            and args.action in ("apply", "activate")
        ):
            try:
                write_motion_lock(manifest, f"{args.action}_failure_{exc.code}")
                _set_services_locked(strict=True)
                update_state(
                    manifest,
                    "FAILED_LOCKED",
                    getattr(args, "action", "unknown").upper(),
                    error=exc,
                )
            except Exception:
                pass
        print(
            json.dumps(
                {"ok": False, "code": exc.code, "message": exc.message},
                ensure_ascii=False,
            ),
            file=sys.stderr,
        )
        return EXIT_CODES.get(exc.code, 1)


if __name__ == "__main__":
    raise SystemExit(main())
