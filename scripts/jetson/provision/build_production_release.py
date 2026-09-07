#!/usr/bin/env python3
"""Build a signed, deterministic NJRH production release bundle.

This command is intended for a clean golden Jetson (or an equivalent release
CI runner), not for a target robot.  The Ed25519 private key remains external:
only its derived public-key SHA-256 is written to the release manifest.
"""

from __future__ import annotations

import argparse
import copy
import gzip
import hashlib
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import types
import urllib.parse
from pathlib import Path, PurePosixPath
from typing import Any, Iterable, NoReturn


WORKSPACE_ROOT = Path("/home/nvidia/workspaces/njrh-v3/workspace1")
UPSTREAM_RUNTIME_ROOT = Path("/home/nvidia/workspaces/isaac_ros-dev")
RUNTIME_OVERLAYS_ROOT = WORKSPACE_ROOT / ".runtime"
SITE_ASSETS_ROOT = WORKSPACE_ROOT / "maps_release"
WORKSPACE_CONTAINER_ROOT = "/workspaces/njrh-v3/workspace1"
UPSTREAM_CONTAINER_ROOT = "/workspaces/isaac_ros-dev"

MANIFEST_NAME = "device-release.json"
LOCK_NAME = "release-lock.json"
SIGNATURE_NAME = "device-release.json.sig"
UPSTREAM_ARCHIVE_NAME = "njrh-upstream-runtime.tar.gz"
OVERLAYS_ARCHIVE_NAME = "njrh-runtime-overlays.tar.gz"
SITE_ARCHIVE_NAME = "njrh-site-assets.tar.gz"
IMAGE_ARCHIVE_NAME = "njrh-runtime-image.tar"

RELEASE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,95}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
IMAGE_DIGEST_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
MAX_ARCHIVE_ENTRIES = 250_000
MAX_ARCHIVE_FILE_BYTES = 20 * 1024 * 1024 * 1024
MAX_ARCHIVE_EXPANDED_BYTES = 80 * 1024 * 1024 * 1024
# GitHub Release assets must be under 2 GiB.  Keep a margin for proxy and API
# implementations that enforce the boundary differently.
RELEASE_ASSET_PART_BYTES = 1900 * 1024 * 1024
MAX_RELEASE_ASSET_BYTES = 30 * 1024 * 1024 * 1024

# Compile caches and VCS internals are omitted deterministically.  Runtime
# payload and site data are rejected if they contain state or secret material.
SKIPPED_DIRECTORY_NAMES = {
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    "_tmp",
    "__pycache__",
    "bags",
    "build",
    "log",
    "logs",
    "rosbags",
    "runtime_logs",
    "tmp",
}
FORBIDDEN_DIRECTORY_NAMES = {
    ".gnupg",
    ".ssh",
    "drafts",
}
FORBIDDEN_EXACT_NAMES = {
    ".env",
    ".map_activation_transaction.v1",
    ".map_asset_integrity_degraded.v1",
    "docking_contact_latch.json",
    "hardware_acceptance.json",
    "last_navigation_map.json",
    "runtime.env",
    "secrets.env",
}
FORBIDDEN_SUFFIXES = {
    ".key",
    ".kdbx",
    ".p12",
    ".pem",
    ".pfx",
}
SKIPPED_SUFFIXES = {
    ".bag",
    ".db3",
    ".log",
    ".mcap",
}
SENSITIVE_NAME_FRAGMENTS = {
    "credential",
    "github_token",
    "password",
    "private_key",
    "registry_password",
    "robot_api_token",
    "secret",
}


class ReleaseBuildError(RuntimeError):
    """Fail-closed error raised for an invalid production release input."""


def _fail(message: str) -> NoReturn:
    raise ReleaseBuildError(message)


def _run(
    command: Iterable[str],
    *,
    cwd: Path | None = None,
    input_bytes: bytes | None = None,
    redacted_values: Iterable[str] = (),
) -> subprocess.CompletedProcess[bytes]:
    command_list = list(command)
    redacted = set(redacted_values)
    display_command = [
        "<redacted>" if argument in redacted else argument for argument in command_list
    ]
    try:
        result = subprocess.run(
            command_list,
            cwd=str(cwd) if cwd is not None else None,
            input=input_bytes,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
    except OSError as exc:
        _fail(
            f"cannot run {display_command[0] if display_command else 'command'}: {exc}"
        )
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", errors="replace").strip()
        _fail(f"command failed ({' '.join(display_command)}): {detail}")
    return result


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _json_bytes(value: Any) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")


def _write_bytes(path: Path, value: bytes, mode: int = 0o644) -> None:
    path.write_bytes(value)
    os.chmod(path, mode)


def _load_template(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        _fail(f"template manifest must be a regular file: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        _fail(f"cannot read template manifest: {exc}")
    if not isinstance(value, dict) or value.get("schema") != "njrh.device_release.v1":
        _fail("template manifest schema must be njrh.device_release.v1")
    for key in ("source", "platform", "workspace", "artifacts", "trust"):
        if not isinstance(value.get(key), dict):
            _fail(f"template manifest is missing object: {key}")
    return value


def assert_git_release_source(workspace: Path) -> dict[str, str]:
    """Return the immutable source identity after enforcing HEAD and cleanliness."""

    root = workspace.resolve(strict=True)
    head = _run(["git", "rev-parse", "HEAD"], cwd=root).stdout.decode().strip()
    if not re.fullmatch(r"[0-9a-f]{40}", head):
        _fail("git HEAD is not a full SHA-1 commit")
    status = _run(
        ["git", "status", "--porcelain=v1", "--untracked-files=all"], cwd=root
    ).stdout.decode("utf-8", errors="replace")
    if status.strip():
        _fail("release source worktree is not clean")
    branch = _run(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=root).stdout.decode().strip()
    return {"commit": head, "branch": branch}


def _validate_source_root(root: Path, label: str) -> Path:
    try:
        metadata = root.lstat()
        resolved = root.resolve(strict=True)
    except OSError as exc:
        _fail(f"{label} source is unavailable: {exc}")
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
        _fail(f"{label} source must be a real directory: {root}")
    return resolved


def _check_entry_name(relative: PurePosixPath, *, is_directory: bool) -> bool:
    """Return False for deterministic cache exclusions; reject unsafe material."""

    lowered_parts = tuple(part.lower() for part in relative.parts)
    name = lowered_parts[-1]
    if any("\x00" in part or "\\" in part for part in relative.parts):
        _fail(f"unsafe payload path: {relative}")
    if name in SKIPPED_DIRECTORY_NAMES:
        return False
    if name in FORBIDDEN_DIRECTORY_NAMES:
        _fail(f"forbidden runtime/sensitive directory: {relative}")
    suffix = PurePosixPath(name).suffix
    if suffix in SKIPPED_SUFFIXES:
        return False
    if (
        name in FORBIDDEN_EXACT_NAMES
        or name.startswith(".env.")
        or suffix in FORBIDDEN_SUFFIXES
        or any(fragment in name for fragment in SENSITIVE_NAME_FRAGMENTS)
    ):
        _fail(f"forbidden runtime/sensitive payload entry: {relative}")
    return True


def _link_target_for_archive(
    root: Path, link_path: Path, relative: PurePosixPath
) -> str:
    raw = os.readlink(link_path)
    if not raw or "\x00" in raw or "\\" in raw:
        _fail(f"unsafe symlink target at {relative}")
    target = Path(raw)
    if target.is_absolute():
        try:
            resolved_target = target.resolve(strict=True)
        except OSError as exc:
            _fail(f"absolute symlink target is unavailable at {relative}: {exc}")
        if resolved_target == root:
            _fail(f"absolute symlink resolves to archive root: {relative} -> {raw}")
        if root not in resolved_target.parents:
            _fail(f"absolute symlink escapes source tree: {relative} -> {raw}")
        rewritten = os.path.relpath(resolved_target, link_path.parent)
        return PurePosixPath(rewritten).as_posix()

    stack = list(relative.parent.parts)
    for part in PurePosixPath(raw).parts:
        if part in ("", "."):
            continue
        if part == "..":
            if not stack:
                _fail(f"relative symlink escapes source tree: {relative} -> {raw}")
            stack.pop()
        else:
            stack.append(part)
    if not stack:
        _fail(f"relative symlink resolves to archive root: {relative} -> {raw}")
    try:
        resolved_target = link_path.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        _fail(f"relative symlink target is unavailable at {relative}: {exc}")
    if resolved_target != root and root not in resolved_target.parents:
        _fail(f"relative symlink escapes source tree: {relative} -> {raw}")
    return PurePosixPath(raw).as_posix()


def _collect_archive_entries(root: Path) -> list[tuple[Path, PurePosixPath, os.stat_result]]:
    entries: list[tuple[Path, PurePosixPath, os.stat_result]] = []
    expanded_bytes = 0

    def visit(directory: Path, parent_relative: PurePosixPath) -> None:
        nonlocal expanded_bytes
        try:
            children = sorted(os.scandir(directory), key=lambda item: item.name)
        except OSError as exc:
            _fail(f"cannot enumerate payload source {directory}: {exc}")
        for child in children:
            relative = parent_relative / child.name
            try:
                metadata = child.stat(follow_symlinks=False)
            except OSError as exc:
                _fail(f"cannot inspect payload entry {relative}: {exc}")
            is_directory = stat.S_ISDIR(metadata.st_mode)
            if not _check_entry_name(relative, is_directory=is_directory):
                continue
            path = Path(child.path)
            if is_directory:
                entries.append((path, relative, metadata))
                visit(path, relative)
            elif stat.S_ISREG(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
                entries.append((path, relative, metadata))
                if stat.S_ISREG(metadata.st_mode):
                    if metadata.st_size > MAX_ARCHIVE_FILE_BYTES:
                        _fail(f"payload file exceeds deployment limit: {relative}")
                    expanded_bytes += metadata.st_size
                    if expanded_bytes > MAX_ARCHIVE_EXPANDED_BYTES:
                        _fail("payload expands beyond the deployment limit")
            else:
                _fail(f"special payload entry is forbidden: {relative}")
            if len(entries) > MAX_ARCHIVE_ENTRIES:
                _fail("payload contains too many entries")

    visit(root, PurePosixPath())
    if not entries:
        _fail(f"payload source is empty: {root}")
    return entries


def create_deterministic_tar_gz(source: Path, destination: Path) -> dict[str, Any]:
    """Archive a source tree with normalized metadata and safe symlink handling."""

    root = _validate_source_root(source, "payload")
    entries = _collect_archive_entries(root)
    included_paths = {relative.as_posix() for _, relative, _ in entries}
    link_targets: dict[str, str] = {}
    for path, relative, metadata in entries:
        if not stat.S_ISLNK(metadata.st_mode):
            continue
        link_targets[relative.as_posix()] = _link_target_for_archive(
            root, path, relative
        )
        resolved_relative = path.resolve(strict=True).relative_to(root).as_posix()
        if resolved_relative not in included_paths:
            _fail(
                f"symlink target was excluded from the payload: "
                f"{relative} -> {resolved_relative}"
            )
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() or destination.is_symlink():
        _fail(f"archive output already exists: {destination}")

    with destination.open("xb") as raw_output:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw_output, mtime=0) as zipped:
            with tarfile.open(
                fileobj=zipped, mode="w", format=tarfile.GNU_FORMAT
            ) as archive:
                for path, relative, metadata in entries:
                    info = tarfile.TarInfo(relative.as_posix())
                    info.uid = 0
                    info.gid = 0
                    info.uname = ""
                    info.gname = ""
                    info.mtime = 0
                    if stat.S_ISDIR(metadata.st_mode):
                        info.type = tarfile.DIRTYPE
                        info.mode = 0o755
                        info.size = 0
                        archive.addfile(info)
                    elif stat.S_ISLNK(metadata.st_mode):
                        info.type = tarfile.SYMTYPE
                        info.mode = 0o777
                        info.size = 0
                        info.linkname = link_targets[relative.as_posix()]
                        archive.addfile(info)
                    else:
                        info.type = tarfile.REGTYPE
                        info.mode = 0o755 if metadata.st_mode & 0o111 else 0o644
                        info.size = metadata.st_size
                        with path.open("rb") as stream:
                            archive.addfile(info, stream)
    os.chmod(destination, 0o644)
    return {
        "name": destination.name,
        "sha256": _sha256_file(destination),
        "entries": len(entries),
        "bytes": destination.stat().st_size,
    }


def split_release_asset(
    path: Path, *, maximum_part_bytes: int = RELEASE_ASSET_PART_BYTES
) -> list[dict[str, Any]]:
    """Split one oversized asset deterministically and remove the oversized file."""

    if maximum_part_bytes <= 0 or maximum_part_bytes >= 2 * 1024 * 1024 * 1024:
        _fail("release asset part size must be positive and under 2 GiB")
    try:
        total_bytes = path.stat().st_size
    except OSError as exc:
        _fail(f"cannot inspect release asset for splitting: {exc}")
    if total_bytes <= maximum_part_bytes:
        return []
    part_count = (total_bytes + maximum_part_bytes - 1) // maximum_part_bytes
    if part_count > 1000:
        _fail("release asset requires more than 1000 GitHub Release parts")
    parts: list[dict[str, Any]] = []
    created_parts: list[Path] = []
    try:
        with path.open("rb") as source:
            for index in range(1, part_count + 1):
                part = path.with_name(
                    f"{path.name}.part-{index:04d}-of-{part_count:04d}"
                )
                created_parts.append(part)
                digest = hashlib.sha256()
                written = 0
                remaining = min(
                    maximum_part_bytes,
                    total_bytes - ((index - 1) * maximum_part_bytes),
                )
                with part.open("xb") as output:
                    while remaining:
                        chunk = source.read(min(8 * 1024 * 1024, remaining))
                        if not chunk:
                            _fail("release asset ended while creating deterministic parts")
                        output.write(chunk)
                        digest.update(chunk)
                        written += len(chunk)
                        remaining -= len(chunk)
                    output.flush()
                    os.fsync(output.fileno())
                os.chmod(part, 0o644)
                parts.append(
                    {
                        "name": part.name,
                        "sha256": digest.hexdigest(),
                        "size_bytes": written,
                    }
                )
            if source.read(1):
                _fail("release asset changed while it was split")
        if sum(item["size_bytes"] for item in parts) != total_bytes:
            _fail("release asset split size does not match the source")
        path.unlink()
    except Exception:
        for part in created_parts:
            part.unlink(missing_ok=True)
        raise
    return parts


def prepare_release_asset(path: Path, metadata: dict[str, Any]) -> dict[str, Any]:
    result = dict(metadata)
    if (
        isinstance(result.get("bytes"), bool)
        or not isinstance(result.get("bytes"), int)
        or result["bytes"] <= 0
        or result["bytes"] > MAX_RELEASE_ASSET_BYTES
    ):
        _fail("release asset size is outside the production limit")
    parts = split_release_asset(path)
    if parts:
        result["parts"] = parts
    return result


def _load_site_validator() -> Any:
    module_path = Path(__file__).with_name("site_assets.py")
    module = types.ModuleType("njrh_release_site_assets")
    module.__file__ = str(module_path)
    try:
        source = module_path.read_text(encoding="utf-8")
        exec(compile(source, str(module_path), "exec"), module.__dict__)
    except Exception as exc:
        _fail(f"cannot load site asset validator: {exc}")
    return module


def validate_site_assets(root: Path) -> dict[str, Any]:
    module = _load_site_validator()
    try:
        result = module.validate_site_tree(root)
    except Exception as exc:
        _fail(f"site asset validation failed: {exc}")
    if not isinstance(result, dict):
        _fail("site asset validator returned an invalid report")
    return result


def _artifact_url(base_url: str, filename: str) -> str:
    parsed = urllib.parse.urlparse(base_url)
    if parsed.scheme != "https" or not parsed.netloc or parsed.query or parsed.fragment:
        _fail("artifact base URL must be an HTTPS directory URL")
    return f"{base_url.rstrip('/')}/{urllib.parse.quote(filename)}"


def _default_artifact_base_url(repository: str, release_id: str) -> str:
    parsed = urllib.parse.urlparse(repository)
    if parsed.scheme != "https" or parsed.netloc.lower() != "github.com":
        _fail("--artifact-base-url is required for a non-GitHub repository")
    path = parsed.path.removesuffix(".git").rstrip("/")
    if path.count("/") != 2:
        _fail("cannot infer GitHub Release URL from source.repository")
    return f"https://github.com{path}/releases/download/{urllib.parse.quote(release_id)}"


def _validate_private_key(
    private_key: Path,
    output_root: Path,
    prohibited_roots: Iterable[Path] = (),
) -> bytes:
    try:
        key_metadata = private_key.lstat()
    except OSError:
        _fail("Ed25519 private key must be an external regular file")
    if stat.S_ISLNK(key_metadata.st_mode) or not stat.S_ISREG(key_metadata.st_mode):
        _fail("Ed25519 private key must be an external regular file")
    key = private_key.resolve(strict=True)
    output = output_root.resolve(strict=False)
    if key == output or output in key.parents or key in output.parents:
        _fail("Ed25519 private key must be outside the release output tree")
    for root in prohibited_roots:
        resolved_root = root.resolve(strict=True)
        if key == resolved_root or resolved_root in key.parents:
            _fail("Ed25519 private key must be outside every archived source tree")
    if os.name != "nt" and stat.S_IMODE(key_metadata.st_mode) & 0o077:
        _fail("Ed25519 private key permissions must be 0600 or stricter")
    key_argument = str(key)
    public_key = _run(
        ["openssl", "pkey", "-in", key_argument, "-pubout"],
        redacted_values=(key_argument,),
    ).stdout
    if b"BEGIN PUBLIC KEY" not in public_key:
        _fail("private key did not yield an Ed25519 public key")
    key_details = _run(
        ["openssl", "pkey", "-in", key_argument, "-text", "-noout"],
        redacted_values=(key_argument,),
    ).stdout.lower()
    if b"ed25519" not in key_details:
        _fail("release private key must use Ed25519")
    return public_key


def _sign_manifest(manifest_path: Path, private_key: Path, signature_path: Path) -> None:
    key_argument = str(private_key.resolve(strict=True))
    _run(
        [
            "openssl",
            "pkeyutl",
            "-sign",
            "-inkey",
            key_argument,
            "-rawin",
            "-in",
            str(manifest_path),
            "-out",
            str(signature_path),
        ],
        redacted_values=(key_argument,),
    )
    if not signature_path.is_file() or signature_path.stat().st_size != 64:
        _fail("OpenSSL did not produce a valid Ed25519 detached signature")
    os.chmod(signature_path, 0o644)


def _published_runtime_image_is_valid(image: dict[str, Any]) -> None:
    kind = image.get("kind")
    image_id = image.get("image_id")
    reference = image.get("reference")
    if not isinstance(image_id, str) or not IMAGE_DIGEST_RE.fullmatch(image_id):
        _fail("runtime image_id must be an immutable sha256 digest")
    if not isinstance(reference, str) or not reference or ":latest" in reference:
        _fail("runtime image reference must be immutable and cannot use latest")
    if kind == "oci":
        digest = image.get("digest")
        uri = image.get("uri")
        if (
            not isinstance(digest, str)
            or not IMAGE_DIGEST_RE.fullmatch(digest)
            or digest == "sha256:" + ("0" * 64)
            or not isinstance(uri, str)
            or not uri.startswith("oci://")
            or not uri.endswith(f"@{digest}")
        ):
            _fail("template OCI runtime image is not publishable")
    elif kind == "docker_archive":
        sha = image.get("sha256")
        if not isinstance(sha, str) or not SHA256_RE.fullmatch(sha):
            _fail("docker runtime archive is missing SHA-256")
    else:
        _fail("runtime image kind must be oci or docker_archive")


def _validate_fixed_manifest_paths(manifest: dict[str, Any]) -> None:
    workspace = manifest["workspace"]
    expected_workspace = {
        "host_path": str(WORKSPACE_ROOT),
        "container_path": WORKSPACE_CONTAINER_ROOT,
        "upstream_host_path": str(UPSTREAM_RUNTIME_ROOT),
        "upstream_container_path": UPSTREAM_CONTAINER_ROOT,
    }
    for key, expected in expected_workspace.items():
        if workspace.get(key) != expected:
            _fail(f"template workspace.{key} must be fixed to {expected}")
    expected_destinations = {
        "upstream_runtime": str(UPSTREAM_RUNTIME_ROOT),
        "runtime_overlays": str(RUNTIME_OVERLAYS_ROOT),
        "site_assets": str(SITE_ASSETS_ROOT),
    }
    for name, expected in expected_destinations.items():
        descriptor = manifest["artifacts"].get(name)
        if not isinstance(descriptor, dict) or descriptor.get("destination") != expected:
            _fail(f"template artifacts.{name}.destination must be fixed to {expected}")


def _asset_transport(
    metadata: dict[str, Any], artifact_base_url: str, filename: str
) -> dict[str, Any]:
    parts = metadata.get("parts", [])
    if parts:
        return {
            "uri": "",
            "archive_size_bytes": metadata["bytes"],
            "parts": [
                {
                    "uri": _artifact_url(artifact_base_url, part["name"]),
                    "sha256": part["sha256"],
                    "size_bytes": part["size_bytes"],
                }
                for part in parts
            ],
        }
    return {
        "uri": _artifact_url(artifact_base_url, filename),
        "archive_size_bytes": metadata["bytes"],
    }


def _save_runtime_image(
    reference: str, staging: Path, artifact_base_url: str
) -> dict[str, Any]:
    if not reference or ":latest" in reference or reference.endswith("/latest"):
        _fail("docker image reference must use a release-specific tag")
    inspected = json.loads(
        _run(["docker", "image", "inspect", reference]).stdout.decode("utf-8")
    )
    if not isinstance(inspected, list) or len(inspected) != 1:
        _fail("docker image inspect returned an unexpected result")
    image_id = inspected[0].get("Id")
    if not isinstance(image_id, str) or not IMAGE_DIGEST_RE.fullmatch(image_id):
        _fail("docker image has no immutable image ID")
    archive = staging / IMAGE_ARCHIVE_NAME
    _run(["docker", "save", "--output", str(archive), reference])
    os.chmod(archive, 0o644)
    metadata = {
        "name": archive.name,
        "sha256": _sha256_file(archive),
        "bytes": archive.stat().st_size,
    }
    metadata = prepare_release_asset(archive, metadata)
    return {
        "kind": "docker_archive",
        "reference": reference,
        "image_id": image_id,
        "sha256": metadata["sha256"],
        **_asset_transport(metadata, artifact_base_url, IMAGE_ARCHIVE_NAME),
    }


def _pinned_oci_runtime_image(reference: str) -> dict[str, Any]:
    if (
        not reference
        or "@" in reference
        or ":latest" in reference
        or reference.endswith("/latest")
    ):
        _fail("OCI image reference must be a release-specific local tag")
    last_slash = reference.rfind("/")
    last_colon = reference.rfind(":")
    if last_colon <= last_slash:
        _fail("OCI image reference must include an immutable release tag")
    repository = reference[:last_colon]
    inspected = json.loads(
        _run(["docker", "image", "inspect", reference]).stdout.decode("utf-8")
    )
    if not isinstance(inspected, list) or len(inspected) != 1:
        _fail("docker image inspect returned an unexpected result")
    image_id = inspected[0].get("Id")
    repo_digests = inspected[0].get("RepoDigests", [])
    if not isinstance(image_id, str) or not IMAGE_DIGEST_RE.fullmatch(image_id):
        _fail("OCI image has no immutable local image ID")
    candidates = sorted(
        digest
        for digest in repo_digests
        if isinstance(digest, str)
        and digest.startswith(f"{repository}@sha256:")
        and IMAGE_DIGEST_RE.fullmatch(digest.split("@", 1)[1])
    )
    if len(candidates) != 1:
        _fail(
            "OCI image tag must have exactly one pushed RepoDigest for its repository"
        )
    digest = candidates[0].split("@", 1)[1]
    return {
        "kind": "oci",
        "uri": f"oci://{repository}@{digest}",
        "reference": reference,
        "digest": digest,
        "image_id": image_id,
    }


def _payload_descriptor(
    template: dict[str, Any],
    name: str,
    artifact_base_url: str,
    filename: str,
    metadata: dict[str, Any],
    *,
    required: bool,
) -> dict[str, Any]:
    descriptor = copy.deepcopy(template["artifacts"].get(name, {}))
    for transport_key in ("uri", "parts", "archive_size_bytes"):
        descriptor.pop(transport_key, None)
    descriptor.update(
        {
            "kind": "payload",
            "sha256": metadata["sha256"],
            "required": required,
            **_asset_transport(metadata, artifact_base_url, filename),
        }
    )
    return descriptor


def _release_lock(
    manifest: dict[str, Any],
    source_identity: dict[str, str],
    captured_at: str,
) -> dict[str, Any]:
    image = manifest["artifacts"]["runtime_image"]
    return {
        "schema": "njrh.production_release_lock.v1",
        "release_id": manifest["release_id"],
        "release_state": "published",
        "captured_at": captured_at,
        "source": {
            "repository": manifest["source"]["repository"],
            "commit": manifest["source"]["commit"],
            "branch": source_identity["branch"],
        },
        "platform": {
            "architecture": manifest["platform"]["architecture"],
            "l4t_release": manifest["platform"]["l4t_release"],
            "l4t_revision": manifest["platform"]["l4t_revision"],
            "jetson_model": manifest["platform"]["jetson_model"],
            "jetpack_version": manifest["platform"]["jetpack_version"],
            "l4t_core_version": manifest["platform"]["l4t_core_version"],
            "tnspec": manifest["platform"]["tnspec"],
            "compatible_spec": manifest["platform"]["compatible_spec"],
            "cpu_count": manifest["platform"]["minimum_online_cpus"],
            "minimum_memory_bytes": manifest["platform"]["minimum_memory_bytes"],
        },
        "golden_images": {
            "runtime_image": {
                "local_reference": image["reference"],
                "image_id": image["image_id"],
            }
        },
        "artifacts": {
            "runtime_image": image.get("digest", image.get("sha256", "")),
            "upstream_runtime": manifest["artifacts"]["upstream_runtime"]["sha256"],
            "runtime_overlays": manifest["artifacts"]["runtime_overlays"]["sha256"],
            "site_assets": manifest["artifacts"]["site_assets"]["sha256"],
        },
        "runtime_contract": {
            "ros_distro": "humble",
            "rmw": "rmw_fastrtps_cpp",
            "container_name": manifest["installation"]["container_name"],
            "motion_state_after_deploy": "locked",
            "base_image_fallback_allowed": False,
        },
    }


def build_release(
    *,
    template_path: Path,
    release_id: str,
    output_dir: Path,
    private_key: Path,
    artifact_base_url: str | None,
    include_site_assets: bool,
    docker_image: str | None,
    oci_image: str | None = None,
) -> dict[str, Any]:
    """Build and atomically publish one external GitHub Release bundle directory."""

    if not RELEASE_ID_RE.fullmatch(release_id):
        _fail("release ID contains unsafe characters")
    source_identity = assert_git_release_source(WORKSPACE_ROOT)
    template = _load_template(template_path)
    _validate_fixed_manifest_paths(template)
    repository = template["source"].get("repository")
    if not isinstance(repository, str) or not repository:
        _fail("template source.repository is missing")
    base_url = artifact_base_url or _default_artifact_base_url(repository, release_id)
    # Validate before creating output or temporary artifacts.
    _artifact_url(base_url, "probe")
    upstream_root = _validate_source_root(UPSTREAM_RUNTIME_ROOT, "upstream runtime")
    overlays_root = _validate_source_root(RUNTIME_OVERLAYS_ROOT, "runtime overlays")
    site_root: Path | None = None
    site_report: dict[str, Any] | None = None
    if include_site_assets:
        site_root = _validate_source_root(SITE_ASSETS_ROOT, "site assets")
        site_report = validate_site_assets(site_root)
    public_key = _validate_private_key(
        private_key,
        output_dir,
        (upstream_root, overlays_root, *([site_root] if site_root is not None else [])),
    )
    output_resolved = output_dir.resolve(strict=False)
    for source in filter(None, (upstream_root, overlays_root, site_root)):
        if output_resolved == source or source in output_resolved.parents:
            _fail("release output directory cannot be inside an archived source tree")
    if output_dir.exists() or output_dir.is_symlink():
        _fail(f"release output directory already exists: {output_dir}")
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(prefix=f".{output_dir.name}.stage-", dir=output_dir.parent)
    )
    try:
        archives: dict[str, dict[str, Any]] = {}
        upstream_archive = staging / UPSTREAM_ARCHIVE_NAME
        archives["upstream_runtime"] = prepare_release_asset(
            upstream_archive,
            create_deterministic_tar_gz(upstream_root, upstream_archive),
        )
        overlays_archive = staging / OVERLAYS_ARCHIVE_NAME
        archives["runtime_overlays"] = prepare_release_asset(
            overlays_archive,
            create_deterministic_tar_gz(overlays_root, overlays_archive),
        )
        if site_root is not None:
            site_archive = staging / SITE_ARCHIVE_NAME
            archives["site_assets"] = prepare_release_asset(
                site_archive,
                create_deterministic_tar_gz(site_root, site_archive),
            )

        manifest = copy.deepcopy(template)
        manifest["release_id"] = release_id
        manifest["release_state"] = "published"
        manifest["desired_state"] = "ready_locked"
        manifest["source"]["commit"] = source_identity["commit"]
        manifest["source"]["require_clean_tracked_tree"] = True
        manifest["source"]["require_clean_tree"] = True
        manifest["artifacts"]["upstream_runtime"] = _payload_descriptor(
            template,
            "upstream_runtime",
            base_url,
            UPSTREAM_ARCHIVE_NAME,
            archives["upstream_runtime"],
            required=True,
        )
        manifest["artifacts"]["runtime_overlays"] = _payload_descriptor(
            template,
            "runtime_overlays",
            base_url,
            OVERLAYS_ARCHIVE_NAME,
            archives["runtime_overlays"],
            required=True,
        )
        if site_root is not None:
            manifest["artifacts"]["site_assets"] = _payload_descriptor(
                template,
                "site_assets",
                base_url,
                SITE_ARCHIVE_NAME,
                archives["site_assets"],
                required=True,
            )
        else:
            site_descriptor = copy.deepcopy(
                template["artifacts"].get("site_assets", {})
            )
            for transport_key in ("parts", "archive_size_bytes"):
                site_descriptor.pop(transport_key, None)
            site_descriptor.update(
                {"kind": "payload", "uri": "", "sha256": "", "required": False}
            )
            manifest["artifacts"]["site_assets"] = site_descriptor

        if docker_image and oci_image:
            _fail("choose either OCI runtime image or docker archive, not both")
        if docker_image:
            manifest["artifacts"]["runtime_image"] = _save_runtime_image(
                docker_image, staging, base_url
            )
        elif oci_image:
            manifest["artifacts"]["runtime_image"] = _pinned_oci_runtime_image(
                oci_image
            )
        _published_runtime_image_is_valid(manifest["artifacts"]["runtime_image"])

        trust = manifest["trust"]
        trust["signature_algorithm"] = "ed25519"
        trust["release_lock"] = LOCK_NAME
        trust["signature_file"] = SIGNATURE_NAME
        trust["trusted_public_key_sha256"] = hashlib.sha256(public_key).hexdigest()

        captured_at = _run(
            ["git", "show", "-s", "--format=%cI", source_identity["commit"]],
            cwd=WORKSPACE_ROOT,
        ).stdout.decode().strip()
        lock = _release_lock(manifest, source_identity, captured_at)
        lock_path = staging / LOCK_NAME
        _write_bytes(lock_path, _json_bytes(lock))
        trust["release_lock_sha256"] = _sha256_file(lock_path)

        manifest_path = staging / MANIFEST_NAME
        _write_bytes(manifest_path, _json_bytes(manifest))
        signature_path = staging / SIGNATURE_NAME
        _sign_manifest(manifest_path, private_key, signature_path)

        os.replace(staging, output_dir)
        return {
            "release_id": release_id,
            "output_dir": str(output_dir),
            "source_commit": source_identity["commit"],
            "manifest": str(output_dir / MANIFEST_NAME),
            "release_lock": str(output_dir / LOCK_NAME),
            "signature": str(output_dir / SIGNATURE_NAME),
            "artifact_base_url": base_url,
            "site_validation": site_report,
        }
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a signed NJRH production GitHub Release bundle"
    )
    parser.add_argument("--template", required=True, type=Path)
    parser.add_argument("--release-id", required=True)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--private-key", required=True, type=Path)
    parser.add_argument(
        "--artifact-base-url",
        help="HTTPS asset directory; defaults to the source GitHub Release URL",
    )
    parser.add_argument(
        "--include-site-assets",
        action="store_true",
        help="validate and include the fixed maps_release site tree",
    )
    image_group = parser.add_mutually_exclusive_group()
    image_group.add_argument(
        "--docker-save-runtime-image",
        dest="docker_image",
        help="save this immutable-tag local image as a release asset",
    )
    image_group.add_argument(
        "--oci-runtime-image",
        dest="oci_image",
        help="bind an already pushed immutable-tag OCI/GHCR image and RepoDigest",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        result = build_release(
            template_path=args.template,
            release_id=args.release_id,
            output_dir=args.output_dir,
            private_key=args.private_key,
            artifact_base_url=args.artifact_base_url,
            include_site_assets=args.include_site_assets,
            docker_image=args.docker_image,
            oci_image=args.oci_image,
        )
    except ReleaseBuildError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
