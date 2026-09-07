#!/usr/bin/env python3
"""Read-only validation for a production ``maps_release`` site tree.

The public interface is intentionally small: :func:`validate_site_tree`
authenticates the persisted map/elevator identities and returns a deterministic
summary.  It never creates, repairs, follows, or rewrites managed assets.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import stat
import struct
from pathlib import Path, PurePosixPath
from typing import Any, Iterable, NoReturn


REPORT_SCHEMA = "njrh.site_asset_validation.v1"
MAP_MANIFEST_SCHEMA = "njrh.map_manifest.v2"
MAP_DIGEST_ALGORITHM = "sha256"
MAP_DIGEST_CONTRACT = "njrh-map-asset-bundle-v1"
REGISTRY_STATE_SCHEMA = "njrh.map_asset_epoch_state.v1"
REGISTRY_BINDINGS_SCHEMA = "njrh.map_asset_bindings.v1"

MAX_UINT64 = (1 << 64) - 1
MAX_REGISTRY_BYTES = 32 * 1024 * 1024
MAX_REGISTRY_BINDINGS = 100_000
MAX_MANIFEST_BYTES = 1024 * 1024
MAX_POSES_BYTES = 64 * 1024 * 1024
MAX_ASSET_BYTES = 512 * 1024 * 1024
MAX_BUNDLE_BYTES = 1024 * 1024 * 1024

SAFE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
SHA256_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
RELEASE_ID_RE = re.compile(r"^elevator-config-([0-9]{6})-([0-9a-f]{12})$")
DRAFT_REVISION_RE = re.compile(r"^draft-v([12])-[0-9a-f]{16}$")
ELEVATOR_ROLES_BY_SCHEMA = {
    1: ("hall_call", "hall_wait", "doorway", "cabin", "exit"),
    2: ("hall_call", "landing", "cabin"),
}
FORBIDDEN_EXACT_NAMES = {
    ".map_activation_transaction.v1",
    ".map_asset_integrity_degraded.v1",
    "docking_contact_latch.json",
    "draft.json",
    "last_navigation_map.json",
}
FORBIDDEN_DIRECTORY_NAMES = {
    ".runtime",
    "bags",
    "drafts",
    "rosbags",
    "runtime_logs",
}
FORBIDDEN_FILE_SUFFIXES = {
    ".bag",
    ".db3",
    ".env",
    ".key",
    ".kdbx",
    ".log",
    ".mcap",
    ".p12",
    ".pem",
    ".pfx",
}
SENSITIVE_NAME_FRAGMENTS = (
    "credential",
    "github_token",
    "password",
    "private_key",
    "registry_password",
    "robot_api_token",
    "secret",
)

ROLE_PATHS: dict[str, Path] = {
    "asset_report_json": Path("reports/asset_report.json"),
    "binary_mask_pgm": Path("filters/binary_mask.pgm"),
    "binary_mask_yaml": Path("filters/binary_mask.yaml"),
    "keepout_mask_pgm": Path("filters/keepout_mask.pgm"),
    "keepout_mask_yaml": Path("filters/keepout_mask.yaml"),
    "localizer_map_png": Path("localizer/{safe_map_name}.png"),
    "localizer_params_yaml": Path("localizer/{safe_map_name}.yaml"),
    "nav_map_pgm": Path("nav/{safe_map_name}.pgm"),
    "nav_map_yaml": Path("nav/{safe_map_name}.yaml"),
    "speed_mask_pgm": Path("filters/speed_mask.pgm"),
    "speed_mask_yaml": Path("filters/speed_mask.yaml"),
}
FIXED_ROLE_PATHS: dict[str, Path] = {
    "asset_report_json": Path("reports/asset_report.json"),
    "binary_mask_pgm": Path("filters/binary_mask.pgm"),
    "binary_mask_yaml": Path("filters/binary_mask.yaml"),
    "keepout_mask_pgm": Path("filters/keepout_mask.pgm"),
    "keepout_mask_yaml": Path("filters/keepout_mask.yaml"),
    "localizer_map_png": Path("localizer/localizer_map.png"),
    "localizer_params_yaml": Path("localizer/localizer_params.yaml"),
    "nav_map_pgm": Path("nav/nav_map.pgm"),
    "nav_map_yaml": Path("nav/nav_map.yaml"),
    "speed_mask_pgm": Path("filters/speed_mask.pgm"),
    "speed_mask_yaml": Path("filters/speed_mask.yaml"),
}
YAML_ROLE_IMAGE_NAMES = {
    "binary_mask_yaml": "binary_mask.pgm",
    "keepout_mask_yaml": "keepout_mask.pgm",
    "localizer_params_yaml": "localizer_map.png",
    "nav_map_yaml": "nav_map.pgm",
    "speed_mask_yaml": "speed_mask.pgm",
}


class SiteAssetValidationError(ValueError):
    """A fail-closed site asset validation failure."""


def _fail(message: str, path: Path | None = None) -> NoReturn:
    suffix = f": {path}" if path is not None else ""
    raise SiteAssetValidationError(f"{message}{suffix}")


def _safe_id(value: Any, label: str) -> str:
    if (
        not isinstance(value, str)
        or ".." in value
        or not SAFE_ID_RE.fullmatch(value)
    ):
        _fail(f"{label} is not a bounded path-safe identifier")
    return value


def _positive_u64(value: Any, label: str, *, string_only: bool = False) -> int:
    if string_only:
        if not isinstance(value, str) or not re.fullmatch(r"[1-9][0-9]*", value):
            _fail(f"{label} must be a canonical positive uint64 string")
        parsed = int(value)
    else:
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            _fail(f"{label} must be a positive uint64")
        parsed = value
    if parsed > MAX_UINT64:
        _fail(f"{label} exceeds uint64")
    return parsed


def _canonical_digest(value: Any, label: str) -> str:
    if not isinstance(value, str) or not SHA256_RE.fullmatch(value):
        _fail(f"{label} must be canonical lowercase sha256")
    return value


def _lstat(path: Path) -> os.stat_result:
    try:
        return path.lstat()
    except OSError as exc:
        _fail(f"cannot inspect managed path ({exc})", path)


def _require_real_directory(path: Path) -> None:
    status = _lstat(path)
    if not stat.S_ISDIR(status.st_mode) or stat.S_ISLNK(status.st_mode):
        _fail("managed directory must be a real non-symlink directory", path)


def _regular_status(path: Path, maximum_bytes: int) -> os.stat_result:
    status = _lstat(path)
    if (
        not stat.S_ISREG(status.st_mode)
        or stat.S_ISLNK(status.st_mode)
        or status.st_nlink != 1
    ):
        _fail("managed file must be a single-link non-symlink regular file", path)
    if status.st_size < 0 or status.st_size > maximum_bytes:
        _fail(f"managed file exceeds {maximum_bytes} bytes", path)
    return status


def _same_file_snapshot(left: os.stat_result, right: os.stat_result) -> bool:
    return (
        left.st_dev == right.st_dev
        and left.st_ino == right.st_ino
        and left.st_mode == right.st_mode
        and left.st_nlink == right.st_nlink
        and left.st_size == right.st_size
        and left.st_mtime_ns == right.st_mtime_ns
    )


def _open_regular(path: Path, maximum_bytes: int) -> tuple[int, os.stat_result]:
    initial = _regular_status(path, maximum_bytes)
    flags = os.O_RDONLY
    flags |= getattr(os, "O_BINARY", 0)
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        _fail(f"cannot open managed file without following links ({exc})", path)
    try:
        opened = os.fstat(descriptor)
        if (
            not stat.S_ISREG(opened.st_mode)
            or opened.st_nlink != 1
            or opened.st_size > maximum_bytes
            or not _same_file_snapshot(initial, opened)
        ):
            _fail("managed file changed while it was opened", path)
        return descriptor, opened
    except Exception:
        os.close(descriptor)
        raise


def _read_regular_bytes(path: Path, maximum_bytes: int) -> bytes:
    descriptor, opened = _open_regular(path, maximum_bytes)
    chunks: list[bytes] = []
    total = 0
    try:
        while True:
            chunk = os.read(descriptor, min(64 * 1024, maximum_bytes + 1 - total))
            if not chunk:
                break
            chunks.append(chunk)
            total += len(chunk)
            if total > maximum_bytes:
                _fail(f"managed file exceeds {maximum_bytes} bytes", path)
        final = os.fstat(descriptor)
        if total != opened.st_size or not _same_file_snapshot(opened, final):
            _fail("managed file changed while it was read", path)
    finally:
        os.close(descriptor)
    return b"".join(chunks)


def _read_text(path: Path, maximum_bytes: int) -> str:
    try:
        return _read_regular_bytes(path, maximum_bytes).decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        _fail(f"managed text is not valid UTF-8 ({exc})", path)


def _strict_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise SiteAssetValidationError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _parse_json(path: Path, maximum_bytes: int) -> dict[str, Any]:
    text = _read_text(path, maximum_bytes)
    try:
        value = json.loads(
            text,
            object_pairs_hook=_strict_object,
            parse_constant=lambda token: (_fail(f"non-finite JSON value {token}", path)),
        )
    except SiteAssetValidationError:
        raise
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        _fail(f"managed JSON is invalid ({exc})", path)
    if not isinstance(value, dict):
        _fail("managed JSON root must be an object", path)
    return value


def _require_exact_keys(
    value: dict[str, Any],
    required: Iterable[str],
    optional: Iterable[str],
    label: str,
) -> None:
    required_set = set(required)
    allowed = required_set | set(optional)
    missing = sorted(required_set - value.keys())
    extra = sorted(value.keys() - allowed)
    if missing or extra:
        _fail(f"{label} has missing={missing} unexpected={extra}")


def _entry_names(path: Path) -> set[str]:
    _require_real_directory(path)
    try:
        return {entry.name for entry in os.scandir(path)}
    except OSError as exc:
        _fail(f"cannot enumerate managed directory ({exc})", path)


def _require_exact_entries(
    path: Path,
    required: Iterable[str],
    optional: Iterable[str] = (),
) -> None:
    required_set = set(required)
    allowed = required_set | set(optional)
    actual = _entry_names(path)
    missing = sorted(required_set - actual)
    extra = sorted(actual - allowed)
    if missing or extra:
        _fail(f"managed directory has missing={missing} unexpected={extra}", path)


def _reject_forbidden_entries(root: Path) -> None:
    pending = [root]
    while pending:
        directory = pending.pop()
        _require_real_directory(directory)
        try:
            entries = list(os.scandir(directory))
        except OSError as exc:
            _fail(f"cannot enumerate managed directory ({exc})", directory)
        for entry in entries:
            name = entry.name
            lowered = name.casefold()
            suffix = Path(lowered).suffix
            forbidden = (
                lowered in FORBIDDEN_EXACT_NAMES
                or lowered in FORBIDDEN_DIRECTORY_NAMES
                or suffix in FORBIDDEN_FILE_SUFFIXES
                or any(fragment in lowered for fragment in SENSITIVE_NAME_FRAGMENTS)
                or lowered.endswith((".part", ".swp", ".tmp", "~"))
            )
            if forbidden:
                _fail("unexpected runtime/sensitive payload entry", Path(entry.path))
            try:
                if entry.is_dir(follow_symlinks=False):
                    pending.append(Path(entry.path))
            except OSError as exc:
                _fail(f"cannot inspect managed entry ({exc})", Path(entry.path))


def _parse_poses(path: Path) -> int:
    text = _read_text(path, MAX_POSES_BYTES)
    lines = text.splitlines()
    if not lines or lines[0].strip() != "poses:":
        _fail("poses.yaml must start with a poses sequence", path)
    poses: list[dict[str, str]] = []
    current: dict[str, str] | None = None
    for line_number, raw in enumerate(lines[1:], start=2):
        stripped = raw.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            _fail(f"poses.yaml comments are not allowed at line {line_number}", path)
        if stripped.startswith("- "):
            if current is not None:
                poses.append(current)
            current = {}
            stripped = stripped[2:].strip()
        if current is None or ":" not in stripped:
            _fail(f"poses.yaml has invalid structure at line {line_number}", path)
        key, raw_value = (part.strip() for part in stripped.split(":", 1))
        if key in current:
            _fail(f"poses.yaml repeats field {key!r}", path)
        current[key] = raw_value
    if current is not None:
        poses.append(current)
    if len(poses) > 100_000:
        _fail("poses.yaml exceeds the pose-count safety limit", path)

    seen: set[str] = set()
    aliases = {"pose_id": "id", "theta": "yaw", "heading": "yaw"}
    allowed = {"id", "pose_id", "name", "type", "x", "y", "yaw", "theta", "heading"}
    for index, pose in enumerate(poses):
        if not set(pose).issubset(allowed):
            _fail(f"poses[{index}] has unexpected fields")
        normalized: dict[str, str] = {}
        for key, value in pose.items():
            canonical = aliases.get(key, key)
            if canonical in normalized:
                _fail(f"poses[{index}] repeats normalized field {canonical}")
            normalized[canonical] = value
        if not {"id", "x", "y", "yaw"}.issubset(normalized):
            _fail(f"poses[{index}] is missing id/x/y/yaw")
        pose_id = _yaml_scalar(normalized["id"])
        _safe_id(pose_id, f"poses[{index}].id")
        if pose_id in seen:
            _fail(f"poses.yaml contains duplicate pose id {pose_id}")
        seen.add(pose_id)
        for field in ("x", "y", "yaw"):
            try:
                number = float(_yaml_scalar(normalized[field]))
            except ValueError:
                _fail(f"poses[{index}].{field} is not numeric")
            if not math.isfinite(number):
                _fail(f"poses[{index}].{field} is not finite")
    return len(poses)


def _yaml_scalar(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        if value[0] == '"':
            try:
                parsed = json.loads(value)
            except json.JSONDecodeError:
                _fail("managed YAML contains an invalid quoted scalar")
            if not isinstance(parsed, str):
                _fail("managed YAML scalar must decode to a string")
            return parsed
        return value[1:-1]
    return value


def _digest_role(
    digest: "hashlib._Hash", logical_name: str, path: Path
) -> int:
    descriptor, opened = _open_regular(path, MAX_ASSET_BYTES)
    encoded_name = logical_name.encode("ascii")
    digest.update(struct.pack(">Q", len(encoded_name)))
    digest.update(encoded_name)
    digest.update(struct.pack(">Q", opened.st_size))
    total = 0
    try:
        while True:
            chunk = os.read(descriptor, 64 * 1024)
            if not chunk:
                break
            digest.update(chunk)
            total += len(chunk)
        final = os.fstat(descriptor)
        if total != opened.st_size or not _same_file_snapshot(opened, final):
            _fail("map role changed while it was hashed", path)
    finally:
        os.close(descriptor)
    return total


def _regular_sha256(path: Path, maximum_bytes: int) -> tuple[int, str]:
    descriptor, opened = _open_regular(path, maximum_bytes)
    digest = hashlib.sha256()
    total = 0
    try:
        while True:
            chunk = os.read(descriptor, 64 * 1024)
            if not chunk:
                break
            digest.update(chunk)
            total += len(chunk)
        final = os.fstat(descriptor)
        if total != opened.st_size or not _same_file_snapshot(opened, final):
            _fail("managed file changed while it was hashed", path)
    finally:
        os.close(descriptor)
    return total, digest.hexdigest()


def _regular_fnv1a64(path: Path, maximum_bytes: int) -> str:
    descriptor, opened = _open_regular(path, maximum_bytes)
    value = 1469598103934665603
    total = 0
    try:
        while True:
            chunk = os.read(descriptor, 64 * 1024)
            if not chunk:
                break
            for byte in chunk:
                value ^= byte
                value = (value * 1099511628211) & MAX_UINT64
            total += len(chunk)
        final = os.fstat(descriptor)
        if total != opened.st_size or not _same_file_snapshot(opened, final):
            _fail("managed file changed while it was hashed", path)
    finally:
        os.close(descriptor)
    return f"{value:016x}"


def _validate_keepout_extras(filters: Path) -> bool:
    semantic = filters / "keepout_semantic_layer.json"
    commit = filters / "keepout_commit.json"
    names = _entry_names(filters)
    semantic_exists = semantic.name in names
    commit_exists = commit.name in names
    if semantic_exists != commit_exists:
        _fail("keepout semantic source and commit marker must be present together", filters)
    if not semantic_exists:
        return False
    _parse_json(semantic, 16 * 1024 * 1024)
    marker = _parse_json(commit, 1024 * 1024)
    _require_exact_keys(
        marker,
        {
            "schema",
            "revision",
            "semantic_file",
            "mask_yaml_file",
            "mask_pgm_file",
            "semantic_fnv64",
            "mask_yaml_fnv64",
            "mask_pgm_fnv64",
        },
        set(),
        "keepout commit marker",
    )
    expected_hashes = {
        "semantic_fnv64": _regular_fnv1a64(semantic, 16 * 1024 * 1024),
        "mask_yaml_fnv64": _regular_fnv1a64(
            filters / "keepout_mask.yaml", MAX_ASSET_BYTES
        ),
        "mask_pgm_fnv64": _regular_fnv1a64(
            filters / "keepout_mask.pgm", MAX_ASSET_BYTES
        ),
    }
    if (
        marker["schema"] != "njrh.keepout.commit.v1"
        or not isinstance(marker["revision"], str)
        or re.fullmatch(r"keepout-v1-fnv64-[0-9a-f]{16}", marker["revision"])
        is None
        or marker["semantic_file"] != "keepout_semantic_layer.json"
        or marker["mask_yaml_file"] != "keepout_mask.yaml"
        or marker["mask_pgm_file"] != "keepout_mask.pgm"
        or any(marker[key] != value for key, value in expected_hashes.items())
    ):
        _fail("keepout commit marker does not authenticate its projection", commit)
    return True


def _yaml_with_image_file(source: bytes, image_file: str, path: Path) -> bytes:
    try:
        text = source.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        _fail(f"map YAML is not valid UTF-8 ({exc})", path)
    output: list[str] = []
    replaced = False
    for line in text.splitlines():
        if not replaced and ":" in line:
            key = line.split(":", 1)[0].strip()
            if key == "image":
                output.append(f"image: {image_file}")
                replaced = True
                continue
        output.append(line)
    if not replaced:
        output.append(f"image: {image_file}")
    return ("\n".join(output) + "\n").encode("utf-8")


def _role_paths(map_root: Path, safe_map_name: str) -> dict[str, Path]:
    return {
        logical_name: map_root / Path(
            relative.as_posix().format(safe_map_name=safe_map_name)
        )
        for logical_name, relative in ROLE_PATHS.items()
    }


def _asset_path_matches(
    advertised: Any,
    building_id: str,
    floor_id: str,
    map_id: str,
    relative_path: Path,
) -> bool:
    if not isinstance(advertised, str) or not advertised or "\x00" in advertised:
        return False
    normalized = advertised.replace("\\", "/")
    if any(part == ".." for part in PurePosixPath(normalized).parts):
        return False
    relative = relative_path.as_posix()
    identity_suffix = f"{building_id}/{floor_id}/maps/{map_id}/{relative}"
    return normalized == relative or normalized.endswith(f"/{identity_suffix}")


def _validate_map_bundle(
    map_root: Path,
    building_id: str,
    floor_id: str,
    map_id: str,
    bindings: dict[tuple[str, str, str], tuple[int, str]],
) -> dict[str, Any]:
    _safe_id(map_id, "map directory")
    _require_exact_entries(
        map_root,
        {"manifest.json", "poses.yaml", "nav", "localizer", "filters", "reports"},
    )
    manifest_path = map_root / "manifest.json"
    manifest = _parse_json(manifest_path, MAX_MANIFEST_BYTES)
    _require_exact_keys(
        manifest,
        {
            "schema",
            "asset_epoch",
            "asset_digest_algorithm",
            "asset_digest_contract",
            "asset_digest",
            "map_id",
            "safe_map_name",
            "building_id",
            "floor_id",
            "active",
            "assets",
        },
        {"display_name", "map_name", "created_at"},
        "map manifest v2",
    )
    if manifest["schema"] != MAP_MANIFEST_SCHEMA:
        _fail("map manifest schema is not njrh.map_manifest.v2", manifest_path)
    if manifest["asset_digest_algorithm"] != MAP_DIGEST_ALGORITHM:
        _fail("map manifest digest algorithm is not sha256", manifest_path)
    if manifest["asset_digest_contract"] != MAP_DIGEST_CONTRACT:
        _fail("map manifest digest contract is not v1", manifest_path)
    epoch = _positive_u64(manifest["asset_epoch"], "map manifest asset_epoch")
    expected_digest = _canonical_digest(
        manifest["asset_digest"], "map manifest asset_digest"
    )
    safe_map_name = _safe_id(manifest["safe_map_name"], "safe_map_name")
    if (
        manifest["building_id"] != building_id
        or manifest["floor_id"] != floor_id
        or manifest["map_id"] != map_id
    ):
        _fail("map manifest identity does not match its directory", manifest_path)
    if not isinstance(manifest["active"], bool):
        _fail("map manifest active must be boolean", manifest_path)

    roles = _role_paths(map_root, safe_map_name)
    _require_exact_entries(
        map_root / "nav",
        {path.name for path in roles.values() if path.parent == map_root / "nav"},
    )
    _require_exact_entries(
        map_root / "localizer",
        {path.name for path in roles.values() if path.parent == map_root / "localizer"},
    )
    _require_exact_entries(
        map_root / "filters",
        {path.name for path in roles.values() if path.parent == map_root / "filters"},
        {"keepout_semantic_layer.json", "keepout_commit.json"},
    )
    has_keepout_overlay = _validate_keepout_extras(map_root / "filters")
    _require_exact_entries(map_root / "reports", {"asset_report.json"})

    assets = manifest["assets"]
    if not isinstance(assets, dict):
        _fail("map manifest assets must be an object", manifest_path)
    _require_exact_keys(assets, set(roles) | {"poses_yaml"}, set(), "map assets")
    for logical_name, role_path in roles.items():
        if not _asset_path_matches(
            assets[logical_name],
            building_id,
            floor_id,
            map_id,
            role_path.relative_to(map_root),
        ):
            _fail(f"manifest asset path does not match role {logical_name}", manifest_path)
    if not _asset_path_matches(
        assets["poses_yaml"],
        building_id,
        floor_id,
        map_id,
        Path("poses.yaml"),
    ):
        _fail("manifest poses path does not match its map", manifest_path)

    digest = hashlib.sha256()
    digest.update(b"njrh-map-asset-bundle-v1")
    aggregate = 0
    for logical_name, role_path in sorted(roles.items()):
        size = _digest_role(digest, logical_name, role_path)
        if size > MAX_BUNDLE_BYTES - aggregate:
            _fail("map role aggregate exceeds 1 GiB", map_root)
        aggregate += size
    actual_digest = f"sha256:{digest.hexdigest()}"
    if actual_digest != expected_digest:
        _fail(
            f"map canonical digest mismatch expected={expected_digest} actual={actual_digest}",
            map_root,
        )
    pose_count = _parse_poses(map_root / "poses.yaml")

    key = (building_id, floor_id, map_id)
    if key not in bindings:
        _fail("map manifest has no persistent registry binding", manifest_path)
    if bindings[key] != (epoch, expected_digest):
        _fail("map manifest identity disagrees with persistent registry binding", manifest_path)
    return {
        "building_id": building_id,
        "floor_id": floor_id,
        "map_id": map_id,
        "asset_epoch": epoch,
        "asset_digest": expected_digest,
        "active": manifest["active"],
        "pose_count": pose_count,
        "_root": map_root,
        "_roles": roles,
        "_manifest_path": manifest_path,
        "_safe_map_name": safe_map_name,
        "_has_keepout_overlay": has_keepout_overlay,
    }


def _validate_registry(root: Path) -> tuple[int, dict[tuple[str, str, str], tuple[int, str]]]:
    registry = root / ".map_asset_registry"
    _require_exact_entries(
        registry, {"registry.lock", "epoch_state.json", "bindings.json"}
    )
    _regular_status(registry / "registry.lock", 1024 * 1024)
    _regular_status(root / ".map_asset_identity_commit.lock", 1024 * 1024)

    state = _parse_json(registry / "epoch_state.json", MAX_REGISTRY_BYTES)
    _require_exact_keys(
        state, {"schema", "high_watermark"}, set(), "map epoch state"
    )
    if state["schema"] != REGISTRY_STATE_SCHEMA:
        _fail("map epoch state schema is invalid", registry / "epoch_state.json")
    high_watermark = _positive_u64(
        state["high_watermark"], "map epoch high_watermark", string_only=True
    )

    persisted = _parse_json(registry / "bindings.json", MAX_REGISTRY_BYTES)
    _require_exact_keys(
        persisted, {"schema", "bindings"}, set(), "map asset bindings"
    )
    if persisted["schema"] != REGISTRY_BINDINGS_SCHEMA:
        _fail("map asset bindings schema is invalid", registry / "bindings.json")
    records = persisted["bindings"]
    if not isinstance(records, list) or len(records) > MAX_REGISTRY_BINDINGS:
        _fail("map asset bindings must be a bounded list", registry / "bindings.json")

    bindings: dict[tuple[str, str, str], tuple[int, str]] = {}
    epochs: set[int] = set()
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            _fail(f"map asset binding[{index}] must be an object")
        _require_exact_keys(
            record,
            {"building_id", "floor_id", "map_id", "asset_epoch", "asset_digest"},
            set(),
            f"map asset binding[{index}]",
        )
        key = (
            _safe_id(record["building_id"], f"binding[{index}].building_id"),
            _safe_id(record["floor_id"], f"binding[{index}].floor_id"),
            _safe_id(record["map_id"], f"binding[{index}].map_id"),
        )
        epoch = _positive_u64(
            record["asset_epoch"],
            f"binding[{index}].asset_epoch",
            string_only=True,
        )
        digest = _canonical_digest(
            record["asset_digest"], f"binding[{index}].asset_digest"
        )
        if key in bindings:
            _fail(f"map asset bindings duplicate identity key {key}")
        if epoch in epochs:
            _fail(f"map asset bindings duplicate asset_epoch {epoch}")
        bindings[key] = (epoch, digest)
        epochs.add(epoch)
    if epochs and high_watermark < max(epochs):
        _fail("map epoch high_watermark regressed below bindings")
    return high_watermark, bindings


def _validate_fixed_projection(
    projection_root: Path,
    source: dict[str, Any],
    *,
    include_manifest: bool,
    check_root_entries: bool = True,
) -> None:
    required_root = {"poses.yaml", "nav", "localizer", "filters", "reports"}
    if include_manifest:
        required_root.add("manifest.json")
    if check_root_entries:
        _require_exact_entries(projection_root, required_root)
    fixed_roles = {
        logical_name: projection_root / relative
        for logical_name, relative in FIXED_ROLE_PATHS.items()
    }
    _require_exact_entries(
        projection_root / "nav",
        {path.name for path in fixed_roles.values() if path.parent == projection_root / "nav"},
    )
    _require_exact_entries(
        projection_root / "localizer",
        {
            path.name
            for path in fixed_roles.values()
            if path.parent == projection_root / "localizer"
        },
    )
    _require_exact_entries(
        projection_root / "filters",
        {
            path.name
            for path in fixed_roles.values()
            if path.parent == projection_root / "filters"
        },
        {"keepout_semantic_layer.json", "keepout_commit.json"},
    )
    _require_exact_entries(projection_root / "reports", {"asset_report.json"})
    projected_overlay = _validate_keepout_extras(projection_root / "filters")
    if projected_overlay != source["_has_keepout_overlay"]:
        _fail("fixed projection keepout overlay presence differs from source map", projection_root)

    source_roles: dict[str, Path] = source["_roles"]
    for logical_name, target in fixed_roles.items():
        source_path = source_roles[logical_name]
        if logical_name in YAML_ROLE_IMAGE_NAMES:
            source_bytes = _read_regular_bytes(source_path, 2 * 1024 * 1024)
            expected = _yaml_with_image_file(
                source_bytes, YAML_ROLE_IMAGE_NAMES[logical_name], source_path
            )
            if _read_regular_bytes(target, 2 * 1024 * 1024) != expected:
                _fail(f"fixed projection YAML differs for {logical_name}", target)
        else:
            if _regular_sha256(source_path, MAX_ASSET_BYTES) != _regular_sha256(
                target, MAX_ASSET_BYTES
            ):
                _fail(f"fixed projection content differs for {logical_name}", target)
    if _regular_sha256(
        source["_root"] / "poses.yaml", MAX_POSES_BYTES
    ) != _regular_sha256(projection_root / "poses.yaml", MAX_POSES_BYTES):
        _fail("fixed projection poses.yaml differs from source map", projection_root)
    _parse_poses(projection_root / "poses.yaml")

    if source["_has_keepout_overlay"]:
        for name in ("keepout_semantic_layer.json", "keepout_commit.json"):
            if _regular_sha256(
                source["_root"] / "filters" / name, 16 * 1024 * 1024
            ) != _regular_sha256(
                projection_root / "filters" / name, 16 * 1024 * 1024
            ):
                _fail(f"fixed projection {name} differs from source map", projection_root)
    if include_manifest:
        projected_manifest = _parse_json(
            projection_root / "manifest.json", MAX_MANIFEST_BYTES
        )
        for key, expected in (
            ("schema", MAP_MANIFEST_SCHEMA),
            ("building_id", source["building_id"]),
            ("floor_id", source["floor_id"]),
            ("map_id", source["map_id"]),
            ("asset_epoch", source["asset_epoch"]),
            ("asset_digest", source["asset_digest"]),
        ):
            if projected_manifest.get(key) != expected:
                _fail(f"fixed projection manifest disagrees on {key}", projection_root)
        if projected_manifest.get("active") is not True:
            _fail("fixed current projection manifest is not active", projection_root)


def _fnv1a64(content: bytes) -> int:
    value = 1469598103934665603
    for byte in content:
        value ^= byte
        value = (value * 1099511628211) & MAX_UINT64
    return value


def _release_id(generation: int, configuration: bytes, topology: bytes, internal: bytes) -> str:
    material = str(generation).encode("ascii") + b"\n" + configuration + topology + internal
    suffix = f"{_fnv1a64(material):016x}"[:12]
    return f"elevator-config-{generation:06d}-{suffix}"


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        _fail(f"{label} must be a finite number")
    return result


def _validate_configuration(
    document: dict[str, Any], building_id: str
) -> tuple[
    dict[tuple[str, str], tuple[int, str]],
    dict[tuple[str, str, str], dict[str, dict[str, float]]],
]:
    schema_version = document.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or schema_version not in ELEVATOR_ROLES_BY_SCHEMA
        or document.get("building_id") != building_id
    ):
        _fail("elevator configuration schema/building identity is inconsistent")
    roles = ELEVATOR_ROLES_BY_SCHEMA[schema_version]
    elevators = document.get("elevators")
    if not isinstance(elevators, list) or not 1 <= len(elevators) <= 16:
        _fail("elevator configuration must contain 1..16 elevators")

    bindings: dict[tuple[str, str], tuple[int, str]] = {}
    floors_by_elevator: dict[
        tuple[str, str, str], dict[str, dict[str, float]]
    ] = {}
    elevator_ids: set[str] = set()
    total_floors = 0
    for elevator_index, elevator in enumerate(elevators):
        if not isinstance(elevator, dict):
            _fail(f"elevators[{elevator_index}] must be an object")
        elevator_id = _safe_id(
            elevator.get("elevator_id"), f"elevators[{elevator_index}].elevator_id"
        )
        if elevator_id in elevator_ids:
            _fail(f"elevator configuration duplicates elevator_id {elevator_id}")
        elevator_ids.add(elevator_id)
        floors = elevator.get("floors")
        if not isinstance(floors, list) or not 1 <= len(floors) <= 64:
            _fail(f"elevator {elevator_id} must contain 1..64 floors")
        total_floors += len(floors)
        if total_floors > 128:
            _fail("elevator configuration exceeds 128 total floor bindings")
        seen_elevator_floors: set[str] = set()
        for floor_index, floor in enumerate(floors):
            if not isinstance(floor, dict):
                _fail(f"elevator {elevator_id} floor[{floor_index}] must be an object")
            floor_id = _safe_id(
                floor.get("floor_id"), f"elevator {elevator_id} floor_id"
            )
            map_id = _safe_id(
                floor.get("map_id"), f"elevator {elevator_id} map_id"
            )
            if floor_id in seen_elevator_floors:
                _fail(f"elevator {elevator_id} duplicates floor_id {floor_id}")
            seen_elevator_floors.add(floor_id)
            epoch = _positive_u64(
                floor.get("map_asset_epoch"), "elevator map_asset_epoch"
            )
            digest = _canonical_digest(
                floor.get("map_asset_digest"), "elevator map_asset_digest"
            )
            binding_key = (floor_id, map_id)
            existing = bindings.get(binding_key)
            if existing is not None and existing != (epoch, digest):
                _fail(f"elevator configuration conflicts on map binding {binding_key}")
            bindings[binding_key] = (epoch, digest)

            poses = floor.get("poses")
            if not isinstance(poses, dict) or set(poses) != set(roles):
                _fail(
                    f"elevator {elevator_id}/{floor_id} must define exactly "
                    f"{len(roles)} schema-v{schema_version} poses"
                )
            validated_poses: dict[str, dict[str, float]] = {}
            for role in roles:
                pose = poses[role]
                if not isinstance(pose, dict) or not {"x", "y", "yaw"}.issubset(pose):
                    _fail(f"elevator pose {elevator_id}/{floor_id}/{role} is incomplete")
                validated_poses[role] = {
                    axis: _finite_number(
                        pose[axis], f"elevator pose {elevator_id}/{floor_id}/{role}.{axis}"
                    )
                    for axis in ("x", "y", "yaw")
                }
            threshold = floor.get("threshold")
            if schema_version == 1:
                if not isinstance(threshold, dict):
                    _fail(f"elevator {elevator_id}/{floor_id} threshold is missing")
                for point_name in ("left", "right", "cabin_reference"):
                    point = threshold.get(point_name)
                    if not isinstance(point, list) or len(point) != 2:
                        _fail(
                            f"elevator {elevator_id}/{floor_id} threshold "
                            f"{point_name} is invalid"
                        )
                    _finite_number(point[0], f"threshold {point_name}[0]")
                    _finite_number(point[1], f"threshold {point_name}[1]")
                for clearance in ("clearance_m", "jamb_clearance_m"):
                    if clearance in threshold and _finite_number(
                        threshold[clearance], f"threshold {clearance}"
                    ) < 0:
                        _fail(f"threshold {clearance} must be non-negative")
            elif "threshold" in floor:
                _fail(
                    f"elevator {elevator_id}/{floor_id} schema v2 forbids threshold"
                )
            floors_by_elevator[(elevator_id, floor_id, map_id)] = validated_poses
    return bindings, floors_by_elevator


def _yaml_lines(text: str, label: str) -> list[tuple[int, str]]:
    result: list[tuple[int, str]] = []
    for number, raw in enumerate(text.splitlines(), start=1):
        if not raw.strip():
            continue
        if "\t" in raw or raw.lstrip().startswith("#"):
            _fail(f"{label} contains unsupported YAML at line {number}")
        result.append((len(raw) - len(raw.lstrip(" ")), raw.strip()))
    return result


def _parse_generated_topology(
    text: str, building_id: str, schema_version: int
) -> dict[tuple[str, str, str], dict[str, str]]:
    roles = ELEVATOR_ROLES_BY_SCHEMA[schema_version]
    lines = _yaml_lines(text, "elevator topology")
    headers = {
        stripped.split(":", 1)[0]: stripped.split(":", 1)[1].strip()
        for indent, stripped in lines
        if indent == 0 and ":" in stripped
    }
    if (
        headers.get("schema_version") != str(schema_version)
        or _yaml_scalar(headers.get("building_id", "")) != building_id
        or headers.get("mock_ports_enabled") != "false"
        or "elevators" not in headers
    ):
        _fail("elevator topology header is inconsistent")

    result: dict[tuple[str, str, str], dict[str, str]] = {}
    elevator_id: str | None = None
    floor_id: str | None = None
    map_id: str | None = None
    poses: dict[str, str] = {}
    section = ""
    threshold_seen = False

    def flush_floor() -> None:
        nonlocal floor_id, map_id, poses, section, threshold_seen
        if floor_id is None:
            return
        if (
            elevator_id is None
            or map_id is None
            or set(poses) != set(roles)
            or (schema_version == 1 and not threshold_seen)
        ):
            _fail("elevator topology floor is incomplete")
        key = (elevator_id, floor_id, map_id)
        if key in result:
            _fail(f"elevator topology duplicates floor {key}")
        result[key] = dict(poses)
        floor_id = None
        map_id = None
        poses = {}
        section = ""
        threshold_seen = False

    for indent, stripped in lines:
        if indent == 2 and stripped.startswith("- elevator_id:"):
            flush_floor()
            elevator_id = _safe_id(
                _yaml_scalar(stripped.split(":", 1)[1]), "topology elevator_id"
            )
        elif indent == 6 and stripped.startswith("- floor_id:"):
            flush_floor()
            if elevator_id is None:
                _fail("elevator topology floor precedes its elevator")
            floor_id = _safe_id(
                _yaml_scalar(stripped.split(":", 1)[1]), "topology floor_id"
            )
        elif indent == 8 and stripped.startswith("map_id:"):
            if floor_id is None:
                _fail("elevator topology map_id precedes its floor")
            map_id = _safe_id(
                _yaml_scalar(stripped.split(":", 1)[1]), "topology map_id"
            )
        elif indent == 8 and stripped == "poses:":
            section = "poses"
        elif indent == 8 and stripped == "threshold:":
            if schema_version == 2:
                _fail("schema-v2 elevator topology contains legacy threshold geometry")
            section = "threshold"
            threshold_seen = True
        elif indent == 10 and section == "poses" and ":" in stripped:
            role, value = (part.strip() for part in stripped.split(":", 1))
            if role not in roles or role in poses:
                _fail(f"elevator topology pose role is invalid: {role}")
            poses[role] = _safe_id(_yaml_scalar(value), "topology pose_id")
    flush_floor()
    if not result:
        _fail("elevator topology contains no floors")
    return result


def _parse_internal_poses(
    text: str, building_id: str, schema_version: int
) -> dict[str, dict[str, Any]]:
    roles = ELEVATOR_ROLES_BY_SCHEMA[schema_version]
    lines = _yaml_lines(text, "elevator internal poses")
    headers = {
        stripped.split(":", 1)[0]: stripped.split(":", 1)[1].strip()
        for indent, stripped in lines
        if indent == 0 and ":" in stripped
    }
    if (
        headers.get("schema_version") != str(schema_version)
        or _yaml_scalar(headers.get("building_id", "")) != building_id
        or "poses" not in headers
    ):
        _fail("elevator internal-pose header is inconsistent")
    records: list[dict[str, str]] = []
    current: dict[str, str] | None = None
    for indent, stripped in lines:
        if indent == 2 and stripped.startswith("- pose_id:"):
            if current is not None:
                records.append(current)
            current = {"pose_id": stripped.split(":", 1)[1].strip()}
        elif indent == 4 and current is not None and ":" in stripped:
            key, value = (part.strip() for part in stripped.split(":", 1))
            if key in current:
                _fail(f"elevator internal pose repeats field {key}")
            current[key] = value
    if current is not None:
        records.append(current)
    result: dict[str, dict[str, Any]] = {}
    required = {
        "pose_id",
        "type",
        "elevator_id",
        "floor_id",
        "map_id",
        "role",
        "x",
        "y",
        "yaw",
    }
    for index, record in enumerate(records):
        if set(record) != required:
            _fail(f"elevator internal pose[{index}] has incomplete fields")
        pose_id = _safe_id(_yaml_scalar(record["pose_id"]), "internal pose_id")
        role = _yaml_scalar(record["role"])
        if role not in roles:
            _fail(f"internal pose role is invalid: {role}")
        numeric: dict[str, float] = {}
        for axis in ("x", "y", "yaw"):
            try:
                numeric[axis] = float(_yaml_scalar(record[axis]))
            except ValueError:
                _fail(f"elevator internal pose[{index}].{axis} is not numeric")
        parsed = {
            "type": _yaml_scalar(record["type"]),
            "elevator_id": _safe_id(
                _yaml_scalar(record["elevator_id"]), "internal elevator_id"
            ),
            "floor_id": _safe_id(
                _yaml_scalar(record["floor_id"]), "internal floor_id"
            ),
            "map_id": _safe_id(_yaml_scalar(record["map_id"]), "internal map_id"),
            "role": role,
            "x": _finite_number(numeric["x"], "internal pose x"),
            "y": _finite_number(numeric["y"], "internal pose y"),
            "yaw": _finite_number(numeric["yaw"], "internal pose yaw"),
        }
        if parsed["type"] != "elevator_internal" or pose_id in result:
            _fail(f"elevator internal pose identity is invalid: {pose_id}")
        result[pose_id] = parsed
    return result


def _validate_relative_symlink(
    link: Path, expected_target: PurePosixPath, site_root: Path, *, directory: bool
) -> None:
    status = _lstat(link)
    if not stat.S_ISLNK(status.st_mode):
        _fail("managed selector must be a symbolic link", link)
    try:
        raw_target = os.readlink(link)
    except OSError as exc:
        _fail(f"cannot read managed selector ({exc})", link)
    if "\\" in raw_target:
        _fail("managed selector must use a POSIX relative target", link)
    target = PurePosixPath(raw_target)
    if target.is_absolute() or any(part in ("", ".", "..") for part in target.parts):
        _fail("managed selector target is unsafe", link)
    if target != expected_target:
        _fail(
            f"managed selector target must be {expected_target.as_posix()!r}", link
        )
    try:
        resolved = (link.parent / Path(*target.parts)).resolve(strict=True)
        resolved.relative_to(site_root.resolve(strict=True))
    except (OSError, ValueError):
        _fail("managed selector resolves outside the site tree or is dangling", link)
    resolved_status = _lstat(resolved)
    expected_mode = stat.S_ISDIR if directory else stat.S_ISREG
    if not expected_mode(resolved_status.st_mode):
        _fail("managed selector target has the wrong file type", link)


def _validate_release_bundle(
    release_root: Path,
    building_id: str,
) -> dict[str, Any]:
    release_id = release_root.name
    match = RELEASE_ID_RE.fullmatch(release_id)
    if match is None:
        _fail("elevator release directory has an invalid release_id", release_root)
    _require_exact_entries(
        release_root,
        {
            "configuration.yaml",
            "elevators.yaml",
            "elevator_internal_poses.yaml",
            "validation.json",
            "manifest.json",
            "current.json",
        },
    )
    paths = {
        name: release_root / name
        for name in (
            "configuration.yaml",
            "elevators.yaml",
            "elevator_internal_poses.yaml",
            "validation.json",
            "manifest.json",
            "current.json",
        )
    }
    raw = {
        name: _read_regular_bytes(path, 2 * 1024 * 1024)
        for name, path in paths.items()
    }
    if raw["validation.json"] != b'{"valid_for_publish":true,"issues":[]}\n':
        _fail("elevator release validation record is not publishable", paths["validation.json"])
    configuration = _parse_json(paths["configuration.yaml"], 2 * 1024 * 1024)
    manifest = _parse_json(paths["manifest.json"], 2 * 1024 * 1024)
    current = _parse_json(paths["current.json"], 2 * 1024 * 1024)
    configuration_bindings, configured_floors = _validate_configuration(
        configuration, building_id
    )
    content_schema_version = configuration["schema_version"]
    try:
        topology_text = raw["elevators.yaml"].decode("utf-8", "strict")
        internal_text = raw["elevator_internal_poses.yaml"].decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        _fail(f"elevator generated YAML is not valid UTF-8 ({exc})", release_root)
    topology = _parse_generated_topology(
        topology_text, building_id, content_schema_version
    )
    if set(topology) != set(configured_floors):
        _fail("elevator topology floors disagree with configuration", release_root)
    internal = _parse_internal_poses(
        internal_text, building_id, content_schema_version
    )
    expected_pose_ids: set[str] = set()
    for floor_key, role_ids in topology.items():
        for role, pose_id in role_ids.items():
            expected_pose_ids.add(pose_id)
            pose = internal.get(pose_id)
            if (
                pose is None
                or pose["elevator_id"] != floor_key[0]
                or pose["floor_id"] != floor_key[1]
                or pose["map_id"] != floor_key[2]
                or pose["role"] != role
            ):
                _fail("elevator topology and internal poses disagree", release_root)
    if set(internal) != expected_pose_ids:
        _fail("elevator internal pose catalog has unreferenced/missing poses", release_root)

    manifest_required = {
        "schema_version",
        "release_id",
        "parent_release_id",
        "source_draft_revision",
        "generation",
        "created_at",
        "actor_id",
        "rollback_of",
        "configuration_digest_algorithm",
        "configuration_digest",
        "map_bindings",
        "asset_published",
        "runtime_applied",
    }
    current_required = {
        "schema_version",
        "release_id",
        "parent_release_id",
        "generation",
        "published_at",
        "actor_id",
        "rollback_of",
    }
    _require_exact_keys(manifest, manifest_required, set(), "elevator release manifest")
    _require_exact_keys(current, current_required, set(), "elevator current metadata")
    generation = _positive_u64(manifest["generation"], "elevator generation")
    if (
        generation != int(match.group(1))
        or current["generation"] != generation
        or manifest["schema_version"] != 1
        or current["schema_version"] != 1
        or manifest["release_id"] != release_id
        or current["release_id"] != release_id
        or manifest["parent_release_id"] != current["parent_release_id"]
        or manifest["actor_id"] != current["actor_id"]
        or manifest["rollback_of"] != current["rollback_of"]
        or not isinstance(manifest["created_at"], str)
        or not manifest["created_at"]
        or not isinstance(current["published_at"], str)
        or not current["published_at"]
        or not isinstance(manifest["actor_id"], str)
        or not manifest["actor_id"]
        or manifest["asset_published"] is not True
        or manifest["runtime_applied"] is not False
    ):
        _fail("elevator release/current metadata identity is inconsistent", release_root)
    parent = manifest["parent_release_id"]
    rollback = manifest["rollback_of"]
    source_draft = manifest["source_draft_revision"]
    source_draft_match = (
        DRAFT_REVISION_RE.fullmatch(source_draft)
        if isinstance(source_draft, str)
        else None
    )
    if (
        (parent is not None and not isinstance(parent, str))
        or (rollback is not None and not isinstance(rollback, str))
        or (source_draft is not None and not isinstance(source_draft, str))
        or (parent is not None and RELEASE_ID_RE.fullmatch(parent) is None)
        or (rollback is not None and RELEASE_ID_RE.fullmatch(rollback) is None)
        or (source_draft is not None and source_draft_match is None)
        or (
            source_draft_match is not None
            and int(source_draft_match.group(1)) != content_schema_version
        )
        or ((source_draft is None) == (rollback is None))
        or (generation == 1 and parent is not None)
        or (generation > 1 and parent is None)
    ):
        _fail("elevator release lineage is inconsistent", release_root)

    content = (
        raw["configuration.yaml"]
        + raw["elevators.yaml"]
        + raw["elevator_internal_poses.yaml"]
    )
    calculated_content_digest = f"{_fnv1a64(content):016x}"
    calculated_release_id = _release_id(
        generation,
        raw["configuration.yaml"],
        raw["elevators.yaml"],
        raw["elevator_internal_poses.yaml"],
    )
    if (
        manifest["configuration_digest_algorithm"] != "fnv1a64"
        or manifest["configuration_digest"] != calculated_content_digest
        or calculated_release_id != release_id
    ):
        _fail("elevator release content digest/release_id is inconsistent", release_root)

    recorded = manifest["map_bindings"]
    if not isinstance(recorded, list) or not recorded:
        _fail("elevator release manifest has no map bindings", release_root)
    manifest_bindings: dict[tuple[str, str], tuple[int, str]] = {}
    for index, binding in enumerate(recorded):
        if not isinstance(binding, dict):
            _fail(f"elevator release binding[{index}] must be an object")
        _require_exact_keys(
            binding,
            {
                "floor_id",
                "map_id",
                "asset_epoch",
                "asset_digest_contract",
                "asset_digest_algorithm",
                "asset_digest",
            },
            set(),
            f"elevator release binding[{index}]",
        )
        if (
            binding["asset_digest_contract"] != MAP_DIGEST_CONTRACT
            or binding["asset_digest_algorithm"] != MAP_DIGEST_ALGORITHM
        ):
            _fail(f"elevator release binding[{index}] digest contract is invalid")
        key = (
            _safe_id(binding["floor_id"], "release binding floor_id"),
            _safe_id(binding["map_id"], "release binding map_id"),
        )
        identity = (
            _positive_u64(binding["asset_epoch"], "release binding asset_epoch"),
            _canonical_digest(binding["asset_digest"], "release binding asset_digest"),
        )
        if key in manifest_bindings:
            _fail(f"elevator release manifest duplicates binding {key}")
        manifest_bindings[key] = identity
    if manifest_bindings != configuration_bindings:
        _fail("elevator release manifest bindings disagree with configuration", release_root)
    return {
        "release_id": release_id,
        "generation": generation,
        "parent_release_id": parent,
        "bindings": manifest_bindings,
        "content_schema_version": content_schema_version,
    }


def _validate_building_elevators(
    site_root: Path,
    building_root: Path,
    map_index: dict[tuple[str, str, str], tuple[int, str]],
) -> dict[str, Any]:
    building_id = building_root.name
    config_root = building_root / ".elevator_config"
    _require_exact_entries(config_root, {"current", "current.json", "releases"})
    releases_root = config_root / "releases"
    release_ids = sorted(_entry_names(releases_root))
    if not release_ids:
        _fail("elevator release store is empty", releases_root)
    releases = {
        release_id: _validate_release_bundle(releases_root / release_id, building_id)
        for release_id in release_ids
    }
    generations: dict[int, str] = {}
    for release_id, release in releases.items():
        generation = release["generation"]
        if generation in generations:
            _fail(f"elevator release generation {generation} is duplicated", releases_root)
        generations[generation] = release_id
        parent = release["parent_release_id"]
        if parent is not None and parent not in releases:
            _fail(f"elevator release parent is missing: {parent}", releases_root)

    _lstat(config_root / "current")
    try:
        raw_current_target = os.readlink(config_root / "current")
    except OSError as exc:
        _fail(f"cannot read elevator current selector ({exc})", config_root / "current")
    current_target = PurePosixPath(raw_current_target)
    if len(current_target.parts) != 2 or current_target.parts[0] != "releases":
        _fail("elevator current selector is not a direct release target", config_root / "current")
    current_release_id = current_target.parts[1]
    if current_release_id not in releases:
        _fail("elevator current selector names an unknown release", config_root / "current")
    _validate_relative_symlink(
        config_root / "current",
        PurePosixPath("releases") / current_release_id,
        site_root,
        directory=True,
    )
    _validate_relative_symlink(
        config_root / "current.json",
        PurePosixPath("current/current.json"),
        site_root,
        directory=False,
    )
    _validate_relative_symlink(
        building_root / "elevators.yaml",
        PurePosixPath(".elevator_config/current/elevators.yaml"),
        site_root,
        directory=False,
    )
    _validate_relative_symlink(
        building_root / "elevator_internal_poses.yaml",
        PurePosixPath(".elevator_config/current/elevator_internal_poses.yaml"),
        site_root,
        directory=False,
    )
    current = releases[current_release_id]
    if current["content_schema_version"] != 2:
        _fail(
            "current elevator release uses legacy schema v1 and is read-only; "
            "publish a schema-v2 release before production deployment",
            releases_root / current_release_id,
        )
    for (floor_id, map_id), identity in current["bindings"].items():
        map_key = (building_id, floor_id, map_id)
        if map_index.get(map_key) != identity:
            _fail(
                f"current elevator release binding disagrees with map identity {map_key}",
                releases_root / current_release_id,
            )
    return {
        "release_count": len(releases),
        "current": {
            "building_id": building_id,
            "release_id": current_release_id,
            "generation": current["generation"],
        },
    }


def validate_site_tree(root: Path) -> dict[str, Any]:
    """Validate a complete, already-extracted ``maps_release`` tree.

    On success a deterministic summary is returned.  Any uncertainty raises
    :class:`SiteAssetValidationError`; the function never mutates ``root``.
    """

    if not isinstance(root, Path):
        root = Path(root)
    _require_real_directory(root)
    _reject_forbidden_entries(root)
    high_watermark, bindings = _validate_registry(root)

    root_entries = _entry_names(root)
    required_root = {".map_asset_registry", ".map_asset_identity_commit.lock"}
    if not required_root.issubset(root_entries):
        _fail("site root is missing registry coordination files", root)
    building_names = sorted(root_entries - required_root)
    if not building_names:
        _fail("site tree contains no building assets", root)

    maps: list[dict[str, Any]] = []
    elevator_buildings: list[Path] = []
    for building_id in building_names:
        _safe_id(building_id, "building directory")
        building_root = root / building_id
        _require_real_directory(building_root)
        building_entries = _entry_names(building_root)
        elevator_entries = {
            ".elevator_config",
            "elevators.yaml",
            "elevator_internal_poses.yaml",
        }
        if building_entries & elevator_entries:
            if not elevator_entries.issubset(building_entries):
                _fail("building has an incomplete elevator release projection", building_root)
            elevator_buildings.append(building_root)
        floor_names = sorted(building_entries - elevator_entries)
        if not floor_names:
            _fail("building contains no floor assets", building_root)
        for floor_id in floor_names:
            _safe_id(floor_id, "floor directory")
            floor_root = building_root / floor_id
            floor_entries = _entry_names(floor_root)
            has_current = "current" in floor_entries
            projection_entries = {"nav", "localizer", "filters", "reports", "poses.yaml"}
            required_floor = {"maps", "current"} | projection_entries if has_current else {"maps"}
            if floor_entries != required_floor:
                _fail(
                    "floor has incomplete/unknown current compatibility projection",
                    floor_root,
                )
            maps_root = floor_root / "maps"
            map_ids = sorted(_entry_names(maps_root))
            if not map_ids:
                _fail("floor maps directory is empty", maps_root)
            floor_maps: list[dict[str, Any]] = []
            for map_id in map_ids:
                verified_map = _validate_map_bundle(
                    maps_root / map_id,
                    building_id,
                    floor_id,
                    map_id,
                    bindings,
                )
                maps.append(verified_map)
                floor_maps.append(verified_map)
            active_maps = [item for item in floor_maps if item["active"]]
            if has_current:
                if len(active_maps) != 1:
                    _fail("floor current projection requires exactly one active map", floor_root)
                source = active_maps[0]
                _validate_fixed_projection(
                    floor_root / "current", source, include_manifest=True
                )
                _validate_fixed_projection(
                    floor_root,
                    source,
                    include_manifest=False,
                    check_root_entries=False,
                )
            elif active_maps:
                _fail("active map is missing its current projection", floor_root)

    if high_watermark < max(item["asset_epoch"] for item in maps):
        _fail("map epoch high_watermark regressed below committed manifests")
    map_index = {
        (item["building_id"], item["floor_id"], item["map_id"]): (
            item["asset_epoch"],
            item["asset_digest"],
        )
        for item in maps
    }
    elevator_results = [
        _validate_building_elevators(root, building_root, map_index)
        for building_root in elevator_buildings
    ]
    public_maps = [
        {
            key: value
            for key, value in item.items()
            if not key.startswith("_") and key not in {"active", "pose_count"}
        }
        for item in maps
    ]
    public_maps.sort(
        key=lambda item: (item["building_id"], item["floor_id"], item["map_id"])
    )
    return {
        "schema": REPORT_SCHEMA,
        "ok": True,
        "root": str(root.resolve(strict=True)),
        "registry": {
            "high_watermark": high_watermark,
            "binding_count": len(bindings),
        },
        "maps": {"count": len(public_maps), "identities": public_maps},
        "elevators": {
            "building_count": len(elevator_results),
            "release_count": sum(item["release_count"] for item in elevator_results),
            "current_release_count": len(elevator_results),
            **(
                {
                    "current_releases": sorted(
                        (item["current"] for item in elevator_results),
                        key=lambda item: item["building_id"],
                    )
                }
                if elevator_results
                else {}
            ),
        },
    }
