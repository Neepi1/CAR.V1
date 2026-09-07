import hashlib
import importlib.util
import json
import os
import struct
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = ROOT / "scripts" / "jetson" / "provision" / "site_assets.py"

ROLE_PATHS = {
    "asset_report_json": Path("reports/asset_report.json"),
    "binary_mask_pgm": Path("filters/binary_mask.pgm"),
    "binary_mask_yaml": Path("filters/binary_mask.yaml"),
    "keepout_mask_pgm": Path("filters/keepout_mask.pgm"),
    "keepout_mask_yaml": Path("filters/keepout_mask.yaml"),
    "localizer_map_png": Path("localizer/map_one.png"),
    "localizer_params_yaml": Path("localizer/map_one.yaml"),
    "nav_map_pgm": Path("nav/map_one.pgm"),
    "nav_map_yaml": Path("nav/map_one.yaml"),
    "speed_mask_pgm": Path("filters/speed_mask.pgm"),
    "speed_mask_yaml": Path("filters/speed_mask.yaml"),
}
FIXED_ROLE_PATHS = {
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
FIXED_IMAGE_NAMES = {
    "binary_mask_yaml": "binary_mask.pgm",
    "keepout_mask_yaml": "keepout_mask.pgm",
    "localizer_params_yaml": "localizer_map.png",
    "nav_map_yaml": "nav_map.pgm",
    "speed_mask_yaml": "speed_mask.pgm",
}


def load_module():
    spec = importlib.util.spec_from_file_location("njrh_site_assets", MODULE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def canonical_digest(entries: dict[str, bytes]) -> str:
    digest = hashlib.sha256()
    digest.update(b"njrh-map-asset-bundle-v1")
    for logical_name, payload in sorted(entries.items()):
        encoded_name = logical_name.encode("ascii")
        digest.update(struct.pack(">Q", len(encoded_name)))
        digest.update(encoded_name)
        digest.update(struct.pack(">Q", len(payload)))
        digest.update(payload)
    return f"sha256:{digest.hexdigest()}"


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, separators=(",", ":"), ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def create_map_bundle(root: Path) -> tuple[Path, str]:
    map_root = root / "building_1" / "F1" / "maps" / "map_1"
    payloads = {
        "asset_report_json": b'{"source":"test"}\n',
        "binary_mask_pgm": b"P5\n1 1\n255\n\x00",
        "binary_mask_yaml": b"image: binary_mask.pgm\nmode: trinary\n",
        "keepout_mask_pgm": b"P5\n1 1\n255\n\x00",
        "keepout_mask_yaml": b"image: keepout_mask.pgm\nmode: trinary\n",
        "localizer_map_png": b"\x89PNG\r\n\x1a\nminimal",
        "localizer_params_yaml": b"image: map_one.png\nresolution: 0.05\n",
        "nav_map_pgm": b"P5\n1 1\n255\n\xfe",
        "nav_map_yaml": b"image: map_one.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        "speed_mask_pgm": b"P5\n1 1\n255\n\x00",
        "speed_mask_yaml": b"image: speed_mask.pgm\nmode: trinary\n",
    }
    for logical_name, relative_path in ROLE_PATHS.items():
        target = map_root / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payloads[logical_name])
    (map_root / "poses.yaml").write_text("poses:\n", encoding="utf-8")

    asset_digest = canonical_digest(payloads)
    manifest = {
        "schema": "njrh.map_manifest.v2",
        "asset_epoch": 1,
        "asset_digest_algorithm": "sha256",
        "asset_digest_contract": "njrh-map-asset-bundle-v1",
        "asset_digest": asset_digest,
        "map_id": "map_1",
        "display_name": "Map One",
        "map_name": "Map One",
        "safe_map_name": "map_one",
        "building_id": "building_1",
        "floor_id": "F1",
        "created_at": "2026-07-25T00:00:00Z",
        "active": False,
        "assets": {
            **{
                logical_name: str(map_root / relative_path)
                for logical_name, relative_path in ROLE_PATHS.items()
            },
            "poses_yaml": str(map_root / "poses.yaml"),
        },
    }
    write_json(map_root / "manifest.json", manifest)
    return map_root, asset_digest


def create_registry(root: Path, asset_digest: str) -> None:
    registry = root / ".map_asset_registry"
    registry.mkdir(parents=True)
    (registry / "registry.lock").write_bytes(b"")
    (root / ".map_asset_identity_commit.lock").write_bytes(b"")
    (registry / "epoch_state.json").write_text(
        '{"schema":"njrh.map_asset_epoch_state.v1","high_watermark":"1"}\n',
        encoding="utf-8",
    )
    (registry / "bindings.json").write_text(
        '{"schema":"njrh.map_asset_bindings.v1","bindings":[\n'
        '{"building_id":"building_1","floor_id":"F1","map_id":"map_1",'
        f'"asset_epoch":"1","asset_digest":"{asset_digest}"}}\n'
        "]}\n",
        encoding="utf-8",
    )


def activate_map_projection(map_root: Path) -> None:
    manifest_path = map_root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["active"] = True
    write_json(manifest_path, manifest)
    floor_root = map_root.parents[1]

    def projected_yaml(source: bytes, image_name: str) -> bytes:
        lines = []
        replaced = False
        for line in source.decode().splitlines():
            if not replaced and ":" in line and line.split(":", 1)[0].strip() == "image":
                lines.append(f"image: {image_name}")
                replaced = True
            else:
                lines.append(line)
        if not replaced:
            lines.append(f"image: {image_name}")
        return ("\n".join(lines) + "\n").encode()

    for projection, include_manifest in (
        (floor_root / "current", True),
        (floor_root, False),
    ):
        for logical_name, fixed_relative in FIXED_ROLE_PATHS.items():
            target = projection / fixed_relative
            target.parent.mkdir(parents=True, exist_ok=True)
            payload = (map_root / ROLE_PATHS[logical_name]).read_bytes()
            if logical_name in FIXED_IMAGE_NAMES:
                payload = projected_yaml(payload, FIXED_IMAGE_NAMES[logical_name])
            target.write_bytes(payload)
        (projection / "poses.yaml").write_bytes((map_root / "poses.yaml").read_bytes())
        if include_manifest:
            (projection / "manifest.json").write_bytes(manifest_path.read_bytes())


def create_minimal_site(root: Path) -> tuple[Path, str]:
    map_root, asset_digest = create_map_bundle(root)
    create_registry(root, asset_digest)
    return map_root, asset_digest


def fnv1a64(payload: bytes) -> int:
    value = 1469598103934665603
    for byte in payload:
        value ^= byte
        value = (value * 1099511628211) & ((1 << 64) - 1)
    return value


def create_elevator_release(root: Path, asset_digest: str) -> str:
    building_root = root / "building_1"
    config_root = building_root / ".elevator_config"
    roles = ("hall_call", "landing", "cabin")
    pose_values = {
        role: {"x": 1.0 + index * 0.2, "y": 1.1 + index * 0.2, "yaw": 0.1}
        for index, role in enumerate(roles)
    }
    configuration = (
        json.dumps(
            {
                "schema_version": 2,
                "building_id": "building_1",
                "elevators": [
                    {
                        "elevator_id": "elevator_west",
                        "floors": [
                            {
                                "floor_id": "F1",
                                "map_id": "map_1",
                                "map_asset_epoch": 1,
                                "map_asset_digest": asset_digest,
                                "poses": pose_values,
                            }
                        ],
                    }
                ],
            },
            separators=(",", ":"),
        )
        + "\n"
    )
    topology_lines = [
        "schema_version: 2",
        "mock_ports_enabled: false",
        "building_id: building_1",
        "elevators:",
        "  - elevator_id: elevator_west",
        "    floors:",
        "      - floor_id: F1",
        "        map_id: map_1",
        "        poses:",
    ]
    for role in roles:
        topology_lines.append(f"          {role}: f1_west_{role}")
    topology = "\n".join(topology_lines) + "\n"

    internal_lines = [
        "schema_version: 2",
        "building_id: building_1",
        "poses:",
    ]
    for index, role in enumerate(roles):
        internal_lines.extend(
            [
                f"  - pose_id: f1_west_{role}",
                "    type: elevator_internal",
                "    elevator_id: elevator_west",
                "    floor_id: F1",
                "    map_id: map_1",
                f"    role: {role}",
                f"    x: {1.0 + index * 0.2}",
                f"    y: {1.1 + index * 0.2}",
                "    yaw: 0.1",
            ]
        )
    internal = "\n".join(internal_lines) + "\n"

    generation = 1
    release_material = (
        str(generation) + "\n" + configuration + topology + internal
    ).encode()
    release_hash = f"{fnv1a64(release_material):016x}"
    release_id = f"elevator-config-{generation:06d}-{release_hash[:12]}"
    content_hash = f"{fnv1a64((configuration + topology + internal).encode()):016x}"
    release_root = config_root / "releases" / release_id
    release_root.mkdir(parents=True)
    (release_root / "configuration.yaml").write_text(configuration, encoding="utf-8")
    (release_root / "elevators.yaml").write_text(topology, encoding="utf-8")
    (release_root / "elevator_internal_poses.yaml").write_text(
        internal, encoding="utf-8"
    )
    (release_root / "validation.json").write_text(
        '{"valid_for_publish":true,"issues":[]}\n', encoding="utf-8"
    )
    write_json(
        release_root / "manifest.json",
        {
            "schema_version": 1,
            "release_id": release_id,
            "parent_release_id": None,
            "source_draft_revision": "draft-v2-0123456789abcdef",
            "generation": generation,
            "created_at": "2026-07-25T00:00:00Z",
            "actor_id": "test",
            "rollback_of": None,
            "configuration_digest_algorithm": "fnv1a64",
            "configuration_digest": content_hash,
            "map_bindings": [
                {
                    "floor_id": "F1",
                    "map_id": "map_1",
                    "asset_epoch": 1,
                    "asset_digest_contract": "njrh-map-asset-bundle-v1",
                    "asset_digest_algorithm": "sha256",
                    "asset_digest": asset_digest,
                }
            ],
            "asset_published": True,
            "runtime_applied": False,
        },
    )
    write_json(
        release_root / "current.json",
        {
            "schema_version": 1,
            "release_id": release_id,
            "parent_release_id": None,
            "generation": generation,
            "published_at": "2026-07-25T00:00:00Z",
            "actor_id": "test",
            "rollback_of": None,
        },
    )

    try:
        os.symlink(
            f"releases/{release_id}",
            config_root / "current",
            target_is_directory=True,
        )
        os.symlink("current/current.json", config_root / "current.json")
        os.symlink(
            ".elevator_config/current/elevators.yaml",
            building_root / "elevators.yaml",
        )
        os.symlink(
            ".elevator_config/current/elevator_internal_poses.yaml",
            building_root / "elevator_internal_poses.yaml",
        )
    except OSError as exc:
        if os.name == "nt" and getattr(exc, "winerror", None) == 1314:
            pytest.skip("Windows test account cannot create symbolic links")
        raise
    return release_id


def test_valid_minimal_site_tree(tmp_path):
    provision = load_module()
    create_minimal_site(tmp_path)

    report = provision.validate_site_tree(tmp_path)

    assert report["ok"] is True
    assert report["schema"] == "njrh.site_asset_validation.v1"
    assert report["registry"]["high_watermark"] == 1
    assert report["maps"]["count"] == 1
    assert report["maps"]["identities"] == [
        {
            "building_id": "building_1",
            "floor_id": "F1",
            "map_id": "map_1",
            "asset_epoch": 1,
            "asset_digest": report["maps"]["identities"][0]["asset_digest"],
        }
    ]
    assert report["elevators"]["current_release_count"] == 0


def test_valid_active_map_current_and_compatibility_projections(tmp_path):
    provision = load_module()
    map_root, _ = create_minimal_site(tmp_path)
    activate_map_projection(map_root)

    report = provision.validate_site_tree(tmp_path)

    assert report["ok"] is True
    assert report["maps"]["count"] == 1


def test_valid_elevator_release_and_safe_relative_selectors(tmp_path):
    provision = load_module()
    _, asset_digest = create_minimal_site(tmp_path)
    release_id = create_elevator_release(tmp_path, asset_digest)

    report = provision.validate_site_tree(tmp_path)

    assert report["elevators"] == {
        "building_count": 1,
        "release_count": 1,
        "current_release_count": 1,
        "current_releases": [
            {"building_id": "building_1", "release_id": release_id, "generation": 1}
        ],
    }


def test_rejects_elevator_source_draft_schema_mismatch(tmp_path):
    provision = load_module()
    _, asset_digest = create_minimal_site(tmp_path)
    release_id = create_elevator_release(tmp_path, asset_digest)
    manifest_path = (
        tmp_path
        / "building_1"
        / ".elevator_config"
        / "releases"
        / release_id
        / "manifest.json"
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["source_draft_revision"] = "draft-v1-0123456789abcdef"
    write_json(manifest_path, manifest)

    with pytest.raises(
        provision.SiteAssetValidationError,
        match="lineage is inconsistent",
    ):
        provision.validate_site_tree(tmp_path)


def test_rejects_legacy_flat_floor_tree_without_immutable_map_bundles(tmp_path):
    provision = load_module()
    asset_digest = "sha256:" + ("a" * 64)
    create_registry(tmp_path, asset_digest)
    floor_root = tmp_path / "building_1" / "F1"
    for directory in ("nav", "localizer", "filters", "reports"):
        (floor_root / directory).mkdir(parents=True, exist_ok=True)
    (floor_root / "poses.yaml").write_text("poses:\n", encoding="utf-8")

    with pytest.raises(
        provision.SiteAssetValidationError,
        match="floor has incomplete/unknown current compatibility projection",
    ):
        provision.validate_site_tree(tmp_path)


def test_rejects_map_digest_and_registry_binding_disagreement(tmp_path):
    provision = load_module()
    _, asset_digest = create_minimal_site(tmp_path)
    wrong_digest = "sha256:" + ("f" * 64)
    assert wrong_digest != asset_digest
    (tmp_path / ".map_asset_registry" / "bindings.json").write_text(
        '{"schema":"njrh.map_asset_bindings.v1","bindings":[\n'
        '{"building_id":"building_1","floor_id":"F1","map_id":"map_1",'
        f'"asset_epoch":"1","asset_digest":"{wrong_digest}"}}\n'
        "]}\n",
        encoding="utf-8",
    )

    with pytest.raises(
        provision.SiteAssetValidationError,
        match="disagrees with persistent registry binding",
    ):
        provision.validate_site_tree(tmp_path)


def test_rejects_elevator_projection_symlink_that_escapes_site_root(tmp_path):
    provision = load_module()
    _, asset_digest = create_minimal_site(tmp_path)
    create_elevator_release(tmp_path, asset_digest)
    outside = tmp_path.parent / f"{tmp_path.name}_outside"
    outside.mkdir()
    (outside / "elevators.yaml").write_text("outside\n", encoding="utf-8")
    projection = tmp_path / "building_1" / "elevators.yaml"
    projection.unlink()
    target = os.path.relpath(outside / "elevators.yaml", projection.parent).replace(
        "\\", "/"
    )
    os.symlink(target, projection)

    with pytest.raises(
        provision.SiteAssetValidationError,
        match="selector target is unsafe|resolves outside",
    ):
        provision.validate_site_tree(tmp_path)


def test_rejects_map_role_content_that_no_longer_matches_canonical_digest(tmp_path):
    provision = load_module()
    map_root, _ = create_minimal_site(tmp_path)
    (map_root / "nav" / "map_one.pgm").write_bytes(b"tampered")

    with pytest.raises(
        provision.SiteAssetValidationError,
        match="map canonical digest mismatch",
    ):
        provision.validate_site_tree(tmp_path)


@pytest.mark.parametrize(
    "relative_path",
    (
        Path("docking_contact_latch.json"),
        Path("last_navigation_map.json"),
        Path("building_1/F1/maps/map_1/secrets.env"),
    ),
)
def test_rejects_runtime_or_sensitive_payload_entries(tmp_path, relative_path):
    provision = load_module()
    create_minimal_site(tmp_path)
    target = tmp_path / relative_path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text("must-not-ship\n", encoding="utf-8")

    with pytest.raises(
        provision.SiteAssetValidationError,
        match="unexpected",
    ):
        provision.validate_site_tree(tmp_path)
