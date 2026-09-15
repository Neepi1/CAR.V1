import importlib.util
import io
import json
import os
import shutil
import subprocess
import tarfile
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[3]
PROVISION_ROOT = ROOT / "scripts" / "jetson" / "provision"
MODULE_PATH = PROVISION_ROOT / "njrh_provision.py"


def load_module():
    spec = importlib.util.spec_from_file_location("njrh_provision", MODULE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample_manifest() -> dict:
    return json.loads(
        (PROVISION_ROOT / "device-release.example.json").read_text(encoding="utf-8")
    )


def mock_root_owned_identity_metadata(monkeypatch, provision, identity_path):
    original_lstat = provision.Path.lstat

    def root_owned_lstat(path):
        metadata = original_lstat(path)
        if Path(path) == identity_path:
            return SimpleNamespace(
                st_mode=metadata.st_mode,
                st_uid=0,
                st_nlink=metadata.st_nlink,
            )
        return metadata

    monkeypatch.setattr(provision.Path, "lstat", root_owned_lstat)


def test_factory_manifest_is_locked_safe_and_secret_free():
    provision = load_module()
    manifest = sample_manifest()

    provision.validate_manifest(manifest, allow_template=True)
    assert manifest["desired_state"] == "ready_locked"
    assert manifest["platform"]["architecture"] == "aarch64"
    assert manifest["platform"]["l4t_release"] == "36"
    assert manifest["platform"]["l4t_revision"] == "4.3"
    assert manifest["build"]["package_manifest"] == (
        "scripts/jetson/provision/runtime-packages.txt"
    )

    serialized = json.dumps(manifest).lower()
    for forbidden in (
        "robot_api_token",
        "password",
        "private_key",
        "github_token",
        "registry_password",
    ):
        assert forbidden not in serialized


def test_apply_rejects_template_and_mutable_runtime_image():
    provision = load_module()
    manifest = sample_manifest()

    with pytest.raises(provision.ProvisionError, match="not published"):
        provision.validate_manifest(manifest, allow_template=False)

    manifest["release_state"] = "published"
    manifest["artifacts"]["runtime_image"]["reference"] = "njrh-car:latest"
    with pytest.raises(provision.ProvisionError, match="mutable"):
        provision.validate_manifest(manifest, allow_template=False)


def test_runtime_package_manifest_is_exact_and_complete():
    packages = [
        line.strip()
        for line in (PROVISION_ROOT / "runtime-packages.txt").read_text(
            encoding="utf-8"
        ).splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]

    assert packages == sorted(packages)
    assert len(packages) == 28
    assert len(packages) == len(set(packages))
    for required in (
        "hesai_ros_driver",
        "ranger_base",
        "robot_map_asset_identity",
        "robot_pgo_mapping",
        "robot_safety",
        "robot_system_tests",
    ):
        assert required in packages


def test_archive_validation_rejects_traversal_and_links(tmp_path):
    provision = load_module()
    traversal = tmp_path / "traversal.tar.gz"
    with tarfile.open(traversal, "w:gz") as archive:
        info = tarfile.TarInfo("../escape")
        payload = b"escape"
        info.size = len(payload)
        archive.addfile(info, io.BytesIO(payload))

    with pytest.raises(provision.ProvisionError, match="unsafe archive path"):
        provision.inspect_payload_archive(traversal)

    symlink = tmp_path / "symlink.tar.gz"
    with tarfile.open(symlink, "w:gz") as archive:
        info = tarfile.TarInfo("payload/link")
        info.type = tarfile.SYMTYPE
        info.linkname = "/etc/passwd"
        archive.addfile(info)

    with pytest.raises(provision.ProvisionError, match="link"):
        provision.inspect_payload_archive(symlink)


def test_chunked_artifact_is_verified_and_reassembled(tmp_path):
    provision = load_module()
    payload = b"chunked-production-payload"
    first = tmp_path / "payload.part-0001"
    second = tmp_path / "payload.part-0002"
    first.write_bytes(payload[:10])
    second.write_bytes(payload[10:])
    descriptor = {
        "uri": "",
        "sha256": provision.hashlib.sha256(payload).hexdigest(),
        "archive_size_bytes": len(payload),
        "parts": [
            {
                "uri": first.resolve().as_uri(),
                "sha256": provision.sha256_file(first),
                "size_bytes": first.stat().st_size,
            },
            {
                "uri": second.resolve().as_uri(),
                "sha256": provision.sha256_file(second),
                "size_bytes": second.stat().st_size,
            },
        ],
    }

    resolved = provision.resolve_artifact(descriptor, tmp_path / "cache", "payload")

    assert resolved.read_bytes() == payload
    assert first.is_file()
    assert second.is_file()

    descriptor["parts"][1]["sha256"] = "0" * 64
    with pytest.raises(provision.ProvisionError, match="SHA-256 mismatch"):
        provision.resolve_artifact(
            descriptor, tmp_path / "second-cache", "tampered-payload"
        )


def test_manifest_validates_signed_release_parts():
    provision = load_module()
    manifest = sample_manifest()
    descriptor = manifest["artifacts"]["upstream_runtime"]
    descriptor.update(
        {
            "uri": "",
            "sha256": "a" * 64,
            "archive_size_bytes": 7,
            "parts": [
                {
                    "uri": "https://github.com/Neepi1/CAR.V1/releases/download/v1/"
                    "payload.part-0001-of-0001",
                    "sha256": "b" * 64,
                    "size_bytes": 7,
                }
            ],
        }
    )

    provision.validate_manifest(manifest, allow_template=True)
    descriptor["parts"][0]["size_bytes"] = provision.MAX_RELEASE_PART_BYTES + 1
    with pytest.raises(provision.ProvisionError, match="size_bytes is invalid"):
        provision.validate_manifest(manifest, allow_template=True)


def test_legacy_site_asset_archive_fails_before_destination_switch(tmp_path):
    provision = load_module()
    archive_path = tmp_path / "legacy-site-assets.tar.gz"
    asset_digest = "sha256:" + ("a" * 64)
    payloads = {
        ".map_asset_identity_commit.lock": b"",
        ".map_asset_registry/registry.lock": b"",
        ".map_asset_registry/epoch_state.json": (
            b'{"schema":"njrh.map_asset_epoch_state.v1","high_watermark":"1"}\n'
        ),
        ".map_asset_registry/bindings.json": (
            b'{"schema":"njrh.map_asset_bindings.v1","bindings":['
            b'{"building_id":"building_1","floor_id":"F1","map_id":"map_1",'
            b'"asset_epoch":"1","asset_digest":"'
            + asset_digest.encode("ascii")
            + b'"}]}\n'
        ),
        "building_1/F1/nav/nav_map.yaml": b"image: nav_map.pgm\n",
        "building_1/F1/localizer/localizer_params.yaml": b"image: localizer_map.png\n",
        "building_1/F1/filters/keepout_mask.yaml": b"image: keepout_mask.pgm\n",
        "building_1/F1/reports/asset_report.json": b"{}\n",
        "building_1/F1/poses.yaml": b"poses:\n",
    }
    with tarfile.open(archive_path, "w:gz") as archive:
        for name, payload in payloads.items():
            info = tarfile.TarInfo(name)
            info.mode = 0o644
            info.size = len(payload)
            archive.addfile(info, io.BytesIO(payload))

    destination = tmp_path / "maps_release"
    destination.mkdir()
    marker = destination / "existing-site-marker"
    marker.write_text("preserve\n", encoding="utf-8")
    descriptor = {"sha256": provision.sha256_file(archive_path)}

    with pytest.raises(
        provision.ProvisionError,
        match="site asset validation failed: "
        "floor has incomplete/unknown current compatibility projection",
    ) as caught:
        provision.stage_payload_for_transaction(
            archive_path,
            destination,
            descriptor,
            "site_assets",
            "legacy",
        )

    assert caught.value.code == "ARTIFACT_INVALID"
    assert marker.read_text(encoding="utf-8") == "preserve\n"
    assert not (tmp_path / ".maps_release.njrh-stage-legacy").exists()


def test_historical_failure_allowlist_is_exact():
    allowlist = json.loads(
        (PROVISION_ROOT / "historical-test-failures.json").read_text(
            encoding="utf-8"
        )
    )
    failures = allowlist["allowed_failures"]

    assert allowlist["schema"] == "njrh.pytest_failure_allowlist.v1"
    assert len(failures) == 6
    assert len(failures) == len(set(failures))
    assert all(
        item.startswith(
            "src/robot_system_tests/test/test_workspace_contracts.py::"
        )
        for item in failures
    )


def test_one_command_entrypoint_and_runtime_lock_contract():
    entrypoint = (ROOT / "scripts" / "jetson" / "provision_njrh.sh").read_text(
        encoding="utf-8"
    )
    installer = (
        ROOT / "scripts" / "jetson" / "install_njrh_autostart.sh"
    ).read_text(encoding="utf-8")
    systemd_runner = (
        ROOT / "scripts" / "jetson" / "njrh_systemd_runtime.sh"
    ).read_text(encoding="utf-8")
    container = (ROOT / "scripts" / "jetson" / "njrh_container.sh").read_text(
        encoding="utf-8"
    )
    can_wait = (
        ROOT / "scripts" / "jetson" / "bringup_ranger_can_wait.sh"
    ).read_text(encoding="utf-8")

    assert "njrh_provision.py" in entrypoint
    assert "apply|activate|accept-hardware|verify|verify-bundle|preflight" in entrypoint
    assert "deploy)" in entrypoint
    assert "enroll-trust)" in entrypoint
    assert "refusing implicit rotation" in entrypoint
    assert "--max-filesize 1048576" in entrypoint
    assert entrypoint.index('verify-bundle \\\n      "${DOWNLOAD_DIR}/device-release.json"') < (
        entrypoint.index('RELEASE_ID="$(python3')
    )
    assert entrypoint.index('RELEASE_ID="$(python3') < entrypoint.index(
        'GENERATED_API_TOKEN="$(openssl rand -hex 32)"'
    )
    assert "install-disabled" in installer
    assert "NJRH_PROVISION_MOTION_LOCK" in installer
    assert "NJRH_SECRETS_ENV_FILE" in installer
    assert "ExecStartPre=/usr/bin/test ! -e" in installer
    assert "PROVISION_MOTION_LOCK" in systemd_runner
    assert "NJRH_EXPECTED_IMAGE_ID" in container
    assert "NJRH_ALLOW_BASE_IMAGE_FALLBACK=false" in container
    assert "runtime_overlay/scripts/bringup_ranger_can.sh" in can_wait


def test_release_bundle_is_verified_before_use(monkeypatch, tmp_path):
    provision = load_module()
    manifest = sample_manifest()
    manifest_path = tmp_path / "device-release.json"
    calls = []

    monkeypatch.setattr(
        provision,
        "validate_manifest",
        lambda value, allow_template=False: calls.append(
            ("validate", value, allow_template)
        ),
    )
    monkeypatch.setattr(
        provision,
        "verify_source",
        lambda workspace, source: calls.append(("source", workspace, source))
        or {"head": source["commit"]},
    )
    monkeypatch.setattr(
        provision,
        "verify_release_trust",
        lambda value, path, workspace: calls.append(
            ("trust", value, path, workspace)
        )
        or {"public_key_sha256": "a" * 64},
    )

    result = provision.verify_release_bundle(manifest, manifest_path)

    assert [call[0] for call in calls] == ["validate", "source", "trust"]
    assert calls[0][2] is False
    assert result["release_id"] == manifest["release_id"]


@pytest.mark.skipif(
    os.name != "posix" or shutil.which("openssl") is None,
    reason="Linux OpenSSL is required",
)
def test_ed25519_verification_uses_seekable_frozen_bytes(tmp_path):
    provision = load_module()
    private_key = tmp_path / "private.pem"
    public_key = tmp_path / "public.pem"
    payload = tmp_path / "device-release.json"
    signature = tmp_path / "device-release.json.sig"
    payload.write_bytes(b'{"release_id":"signed-test"}\n')
    subprocess.run(
        ["openssl", "genpkey", "-algorithm", "Ed25519", "-out", str(private_key)],
        check=True,
        capture_output=True,
    )
    with public_key.open("wb") as stream:
        subprocess.run(
            ["openssl", "pkey", "-in", str(private_key), "-pubout"],
            check=True,
            stdout=stream,
            stderr=subprocess.PIPE,
        )
    subprocess.run(
        [
            "openssl",
            "pkeyutl",
            "-sign",
            "-inkey",
            str(private_key),
            "-rawin",
            "-in",
            str(payload),
            "-out",
            str(signature),
        ],
        check=True,
        capture_output=True,
    )

    provision._verify_ed25519_detached(
        public_key.read_bytes(), signature.read_bytes(), payload.read_bytes()
    )
    with pytest.raises(provision.ProvisionError, match="signature is invalid"):
        provision._verify_ed25519_detached(
            public_key.read_bytes(),
            signature.read_bytes(),
            payload.read_bytes() + b"tampered",
        )


def test_release_lock_records_golden_jetson_identity():
    release_lock = json.loads(
        (PROVISION_ROOT / "release-lock.json").read_text(encoding="utf-8")
    )

    assert release_lock["schema"] == "njrh.production_release_lock.v1"
    assert release_lock["platform"] == {
        "architecture": "aarch64",
        "l4t_release": "36",
        "l4t_revision": "4.3",
        "jetson_model": "NVIDIA Jetson Orin NX Engineering Reference Developer Kit Super",
        "jetpack_version": "6.2.1+b38",
        "l4t_core_version": "36.4.3-20250107174145",
        "tnspec": "3767-303-0000-D.1-1-1-jetson-orin-nano-devkit-super-",
        "compatible_spec": "3767-000-0000--1--jetson-orin-nano-devkit-super-",
        "cpu_count": 8,
        "minimum_memory_bytes": 15000000000,
    }
    assert release_lock["golden_images"]["base_image"]["image_id"] == (
        "sha256:80b6ec675107284658a40008ada248f1995fbd853ec493e80e4511fa57c29188"
    )
    assert release_lock["golden_images"]["runtime_image"]["image_id"] == (
        "sha256:2345545cf8b909d7b8fd0858af183fa0e801e59633b55f11c228b69b14253a3a"
    )


def test_manifest_rejects_path_and_hardware_drift():
    provision = load_module()
    manifest = sample_manifest()
    manifest["workspace"]["host_path"] = "/tmp/attacker-controlled"
    with pytest.raises(provision.ProvisionError, match="production path"):
        provision.validate_manifest(manifest, allow_template=True)

    manifest = sample_manifest()
    manifest["hardware"]["jt128"]["device_ip"] = "192.168.1.202"
    with pytest.raises(provision.ProvisionError, match="JT128 wiring"):
        provision.validate_manifest(manifest, allow_template=True)

    manifest = sample_manifest()
    manifest["hardware"]["orbbec"]["required"] = False
    with pytest.raises(provision.ProvisionError, match="Orbbec required must be true"):
        provision.validate_manifest(manifest, allow_template=True)


def test_published_manifest_requires_non_template_trust_binding():
    provision = load_module()
    manifest = sample_manifest()
    manifest["release_state"] = "published"
    image_digest = "sha256:" + ("1" * 64)
    manifest["artifacts"]["runtime_image"]["digest"] = image_digest
    manifest["artifacts"]["runtime_image"]["uri"] = (
        f"oci://ghcr.io/neepi1/car-v1/njrh-car@{image_digest}"
    )
    for index, name in enumerate(("upstream_runtime", "runtime_overlays"), start=2):
        manifest["artifacts"][name]["uri"] = (
            f"https://github.com/Neepi1/CAR.V1/releases/download/v1/{name}.tar.gz"
        )
        manifest["artifacts"][name]["sha256"] = str(index) * 64
    manifest["hardware"]["orbbec"]["serial"] = "CPC8563000LM"
    with pytest.raises(provision.ProvisionError, match="trust.release_lock_sha256"):
        provision.validate_manifest(manifest, allow_template=False)


def test_tree_receipt_detects_payload_drift(tmp_path):
    provision = load_module()
    tree = tmp_path / "tree"
    tree.mkdir()
    payload = tree / "payload.txt"
    payload.write_text("v1", encoding="utf-8")
    inputs = {"artifact": "abc"}
    receipt = tmp_path / "receipt.json"
    provision.write_receipt(
        receipt,
        "release-1",
        inputs,
        {"tree_digest": provision.tree_digest(tree)},
    )
    assert provision.receipt_matches(
        receipt, "release-1", inputs, tree=tree
    )
    payload.write_text("v2", encoding="utf-8")
    assert not provision.receipt_matches(
        receipt, "release-1", inputs, tree=tree
    )


def test_release_transaction_can_restore_all_old_destinations(tmp_path):
    provision = load_module()
    manifest = sample_manifest()
    state_dir = tmp_path / "state"
    prepared_items = []
    for name in ("upstream", "overlay", "runtime_env"):
        destination = tmp_path / name
        staging = tmp_path / f".{name}.stage"
        destination.mkdir()
        staging.mkdir()
        (destination / "value").write_text("old", encoding="utf-8")
        (staging / "value").write_text("new", encoding="utf-8")
        prepared_items.append(
            {
                "name": name,
                "destination": str(destination),
                "staging": str(staging),
            }
        )

    journal_path, journal = provision.commit_release_transaction(
        manifest, state_dir, "tx1", prepared_items
    )
    assert journal["status"] == "COMMITTED"
    assert all(
        (Path(item["destination"]) / "value").read_text(encoding="utf-8")
        == "new"
        for item in prepared_items
    )

    rolled_back = provision.rollback_release_transaction(journal_path)
    assert rolled_back["status"] == "ROLLED_BACK"
    assert all(
        (Path(item["destination"]) / "value").read_text(encoding="utf-8")
        == "old"
        for item in prepared_items
    )


def test_runtime_env_freezes_accepted_navigation_profile():
    provision = load_module()
    values = provision.runtime_env_values(sample_manifest())
    assert values["NJRH_NAV2_PLANNER_PROFILE"] == "ranger_lattice"
    assert values["NJRH_NAV_LOCAL_STATE_MODE"] == "ekf"
    assert values["NJRH_LOCAL_STATE_EKF_PROFILE"] == "wheel_spin_imu"
    assert values["NJRH_AMCL_LOCALIZATION_MODE"] == "gated"
    assert values["NJRH_ALLOW_BASE_IMAGE_FALLBACK"] == "false"


def test_runtime_env_enables_nav2_process_preload():
    provision = load_module()
    assert provision.runtime_env_values(sample_manifest())[
        "NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION"] == "true"


def test_auto_enrolled_device_identity_is_reused_and_rejects_camera_replacement(
    monkeypatch, tmp_path
):
    provision = load_module()
    manifest = sample_manifest()
    identity_path = tmp_path / "device-identity.json"
    manifest["installation"]["device_identity_file"] = str(identity_path)
    preflight = {
        "facts": {
            "strict_hardware": {
                "orbbec": {
                    "devices": [
                        {
                            "usb_id": "2bc5:0807",
                            "serial": "FACTORY-CAMERA-001",
                            "speed_mbps": 5000,
                        }
                    ]
                }
            }
        }
    }
    monkeypatch.setattr(
        provision.os, "chown", lambda path, uid, gid: None, raising=False
    )
    monkeypatch.setattr(provision.stat, "S_IMODE", lambda mode: 0o600)
    mock_root_owned_identity_metadata(
        monkeypatch, provision, identity_path
    )

    enrolled = provision.enroll_or_verify_device_identity(
        manifest, preflight, allow_enroll=True
    )
    verified = provision.enroll_or_verify_device_identity(
        manifest, preflight, allow_enroll=False
    )

    assert enrolled["policy"] == "auto_enrolled"
    assert verified["orbbec_serial"] == "FACTORY-CAMERA-001"
    identity = json.loads(identity_path.read_text(encoding="utf-8"))
    assert identity["orbbec_serial"] == "FACTORY-CAMERA-001"

    preflight["facts"]["strict_hardware"]["orbbec"]["devices"][0]["serial"] = (
        "FACTORY-CAMERA-002"
    )
    with pytest.raises(provision.ProvisionError, match="enrolled device identity"):
        provision.enroll_or_verify_device_identity(
            manifest, preflight, allow_enroll=False
        )


def test_auto_enrolled_device_identity_rejects_file_tampering(monkeypatch, tmp_path):
    provision = load_module()
    manifest = sample_manifest()
    identity_path = tmp_path / "device-identity.json"
    manifest["installation"]["device_identity_file"] = str(identity_path)
    preflight = {
        "facts": {
            "strict_hardware": {
                "orbbec": {
                    "devices": [
                        {
                            "usb_id": "2bc5:0807",
                            "serial": "FACTORY-CAMERA-001",
                            "speed_mbps": 5000,
                        }
                    ]
                }
            }
        }
    }
    monkeypatch.setattr(
        provision.os, "chown", lambda path, uid, gid: None, raising=False
    )
    monkeypatch.setattr(provision.stat, "S_IMODE", lambda mode: 0o600)
    mock_root_owned_identity_metadata(
        monkeypatch, provision, identity_path
    )
    provision.enroll_or_verify_device_identity(
        manifest, preflight, allow_enroll=True
    )
    identity = json.loads(identity_path.read_text(encoding="utf-8"))
    identity["tnspec"] = "tampered"
    identity_path.write_text(json.dumps(identity), encoding="utf-8")

    with pytest.raises(provision.ProvisionError, match="enrolled device identity"):
        provision.enroll_or_verify_device_identity(
            manifest, preflight, allow_enroll=False
        )
