from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import tarfile
from types import SimpleNamespace
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = (
    ROOT / "scripts" / "jetson" / "provision" / "build_production_release.py"
)
SPEC = importlib.util.spec_from_file_location("njrh_release_builder", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


def _archive_members(path: Path) -> dict[str, tarfile.TarInfo]:
    with tarfile.open(path, "r:gz") as archive:
        return {member.name: member for member in archive.getmembers()}


def test_deterministic_archive_bytes_and_normalized_metadata(tmp_path: Path) -> None:
    source = tmp_path / "source"
    (source / "bin").mkdir(parents=True)
    executable = source / "bin" / "run.sh"
    executable.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    os.chmod(executable, 0o751)
    (source / "payload.txt").write_text("stable\n", encoding="utf-8")
    os.utime(source / "payload.txt", (1_700_000_000, 1_700_000_000))

    first = tmp_path / "first.tar.gz"
    second = tmp_path / "second.tar.gz"
    builder.create_deterministic_tar_gz(source, first)
    os.utime(source / "payload.txt", (1_800_000_000, 1_800_000_000))
    builder.create_deterministic_tar_gz(source, second)

    assert first.read_bytes() == second.read_bytes()
    members = _archive_members(first)
    assert members["payload.txt"].mtime == 0
    assert members["payload.txt"].uid == 0
    assert members["payload.txt"].mode == 0o644
    assert members["bin/run.sh"].mode == (0o644 if os.name == "nt" else 0o755)


def test_oversized_release_asset_is_split_deterministically(tmp_path: Path) -> None:
    asset = tmp_path / "payload.tar.gz"
    payload = b"0123456789abcdef"
    asset.write_bytes(payload)

    parts = builder.split_release_asset(asset, maximum_part_bytes=6)

    assert not asset.exists()
    assert [part["size_bytes"] for part in parts] == [6, 6, 4]
    assert [part["name"] for part in parts] == [
        "payload.tar.gz.part-0001-of-0003",
        "payload.tar.gz.part-0002-of-0003",
        "payload.tar.gz.part-0003-of-0003",
    ]
    reconstructed = b"".join(
        (tmp_path / part["name"]).read_bytes() for part in parts
    )
    assert reconstructed == payload
    assert all(
        hashlib.sha256((tmp_path / part["name"]).read_bytes()).hexdigest()
        == part["sha256"]
        for part in parts
    )


@pytest.mark.skipif(os.name == "nt", reason="Windows symlink policy needs elevation")
def test_safe_relative_symlink_is_preserved(tmp_path: Path) -> None:
    source = tmp_path / "source"
    (source / "releases" / "v1").mkdir(parents=True)
    (source / "releases" / "v1" / "asset").write_text("ok", encoding="utf-8")
    (source / "current").symlink_to("releases/v1", target_is_directory=True)

    archive = tmp_path / "payload.tar.gz"
    builder.create_deterministic_tar_gz(source, archive)

    current = _archive_members(archive)["current"]
    assert current.issym()
    assert current.linkname == "releases/v1"


@pytest.mark.skipif(os.name == "nt", reason="Windows symlink policy needs elevation")
def test_internal_absolute_symlink_is_rewritten_relative(tmp_path: Path) -> None:
    source = tmp_path / "source"
    target = source / "releases" / "v1"
    target.mkdir(parents=True)
    (target / "asset").write_text("ok", encoding="utf-8")
    (source / "current").symlink_to(target, target_is_directory=True)

    archive = tmp_path / "payload.tar.gz"
    builder.create_deterministic_tar_gz(source, archive)

    current = _archive_members(archive)["current"]
    assert current.issym()
    assert current.linkname == "releases/v1"
    assert not Path(current.linkname).is_absolute()


@pytest.mark.skipif(os.name == "nt", reason="Windows symlink policy needs elevation")
def test_out_of_tree_absolute_symlink_is_rejected(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    outside = tmp_path / "outside"
    outside.write_text("secret", encoding="utf-8")
    (source / "escape").symlink_to(outside)

    with pytest.raises(builder.ReleaseBuildError, match="escapes source tree"):
        builder.create_deterministic_tar_gz(source, tmp_path / "payload.tar.gz")


def test_sensitive_or_runtime_state_is_rejected(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    (source / "secrets.env").write_text("ROBOT_API_TOKEN=nope\n", encoding="utf-8")

    with pytest.raises(builder.ReleaseBuildError, match="sensitive"):
        builder.create_deterministic_tar_gz(source, tmp_path / "payload.tar.gz")


def test_oci_runtime_image_binds_pushed_repo_digest(monkeypatch) -> None:
    image_id = "sha256:" + ("1" * 64)
    digest = "sha256:" + ("2" * 64)
    inspect_result = [
        {
            "Id": image_id,
            "RepoDigests": [f"ghcr.io/neepi1/car-v1/njrh-car@{digest}"],
        }
    ]
    monkeypatch.setattr(
        builder,
        "_run",
        lambda *args, **kwargs: SimpleNamespace(
            stdout=json.dumps(inspect_result).encode("utf-8")
        ),
    )

    descriptor = builder._pinned_oci_runtime_image(
        "ghcr.io/neepi1/car-v1/njrh-car:2026.07.25"
    )
    assert descriptor["image_id"] == image_id
    assert descriptor["digest"] == digest
    assert descriptor["uri"].endswith(f"@{digest}")


@pytest.mark.skipif(shutil.which("openssl") is None, reason="OpenSSL is required")
def test_private_key_never_enters_release_output(tmp_path: Path) -> None:
    private_key = tmp_path / "factory-private.pem"
    subprocess.run(
        ["openssl", "genpkey", "-algorithm", "Ed25519", "-out", str(private_key)],
        check=True,
        capture_output=True,
    )
    output = tmp_path / "release"
    output.mkdir()
    public_key = builder._validate_private_key(private_key, output)
    manifest = output / builder.MANIFEST_NAME
    manifest.write_text('{"release_state":"published"}\n', encoding="utf-8")
    signature = output / builder.SIGNATURE_NAME
    builder._sign_manifest(manifest, private_key, signature)

    private_bytes = private_key.read_bytes()
    assert len(signature.read_bytes()) == 64
    assert hashlib.sha256(public_key).hexdigest()
    assert all(
        private_bytes not in path.read_bytes()
        for path in output.iterdir()
        if path.is_file()
    )
    assert private_key.name not in {path.name for path in output.iterdir()}


def test_private_key_cannot_be_inside_output_tree(tmp_path: Path) -> None:
    output = tmp_path / "release"
    output.mkdir()
    private_key = output / "private.pem"
    private_key.write_text("not even a key", encoding="utf-8")

    with pytest.raises(builder.ReleaseBuildError, match="outside"):
        builder._validate_private_key(private_key, output)


@pytest.mark.skipif(
    shutil.which("openssl") is None or shutil.which("git") is None,
    reason="OpenSSL and Git are required",
)
def test_external_bundle_binds_commit_lock_hash_and_signature(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    workspace = tmp_path / "workspace1"
    upstream = tmp_path / "isaac_ros-dev"
    overlays = workspace / ".runtime"
    upstream.mkdir()
    overlays.mkdir(parents=True)
    (upstream / "install.marker").write_text("upstream\n", encoding="utf-8")
    (overlays / "overlay.marker").write_text("overlay\n", encoding="utf-8")
    (workspace / ".gitignore").write_text(".runtime/\n", encoding="utf-8")
    (workspace / "tracked.txt").write_text("source\n", encoding="utf-8")
    subprocess.run(["git", "init", str(workspace)], check=True, capture_output=True)
    subprocess.run(
        ["git", "config", "user.email", "release-test@example.invalid"],
        cwd=workspace,
        check=True,
    )
    subprocess.run(
        ["git", "config", "user.name", "Release Test"],
        cwd=workspace,
        check=True,
    )
    subprocess.run(["git", "add", "."], cwd=workspace, check=True)
    subprocess.run(
        ["git", "commit", "-m", "fixture"], cwd=workspace, check=True, capture_output=True
    )
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=workspace,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()

    monkeypatch.setattr(builder, "WORKSPACE_ROOT", workspace)
    monkeypatch.setattr(builder, "UPSTREAM_RUNTIME_ROOT", upstream)
    monkeypatch.setattr(builder, "RUNTIME_OVERLAYS_ROOT", overlays)
    monkeypatch.setattr(builder, "SITE_ASSETS_ROOT", workspace / "maps_release")

    zeros = "0" * 64
    image_digest = "sha256:" + ("2" * 64)
    image_id = "sha256:" + ("3" * 64)
    template = {
        "schema": "njrh.device_release.v1",
        "release_id": "template",
        "release_state": "template",
        "source": {
            "repository": "https://github.com/example/njrh.git",
            "commit": zeros[:40],
        },
        "platform": {
            "architecture": "aarch64",
            "l4t_release": "36",
            "l4t_revision": "4.3",
            "jetson_model": (
                "NVIDIA Jetson Orin NX Engineering Reference Developer Kit Super"
            ),
            "jetpack_version": "6.2.1+b38",
            "l4t_core_version": "36.4.3-20250107174145",
            "tnspec": "3767-303-0000-D.1-1-1-jetson-orin-nano-devkit-super-",
            "compatible_spec": (
                "3767-000-0000--1--jetson-orin-nano-devkit-super-"
            ),
            "minimum_online_cpus": 8,
            "minimum_memory_bytes": 15_000_000_000,
        },
        "workspace": {
            "host_path": str(workspace),
            "container_path": builder.WORKSPACE_CONTAINER_ROOT,
            "upstream_host_path": str(upstream),
            "upstream_container_path": builder.UPSTREAM_CONTAINER_ROOT,
        },
        "artifacts": {
            "runtime_image": {
                "kind": "oci",
                "uri": f"oci://ghcr.io/example/njrh@{image_digest}",
                "reference": "ghcr.io/example/njrh:r1",
                "digest": image_digest,
                "image_id": image_id,
            },
            "upstream_runtime": {
                "destination": str(upstream),
                "required": True,
            },
            "runtime_overlays": {
                "destination": str(overlays),
                "required": True,
            },
            "site_assets": {
                "destination": str(workspace / "maps_release"),
                "required": False,
            },
        },
        "trust": {},
        "installation": {"container_name": "NJRH-car"},
    }
    template_path = tmp_path / "template.json"
    template_path.write_text(json.dumps(template), encoding="utf-8")
    private_key = tmp_path / "factory-private.pem"
    public_key = tmp_path / "factory-public.pem"
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

    output = tmp_path / "release-output"
    result = builder.build_release(
        template_path=template_path,
        release_id="r1",
        output_dir=output,
        private_key=private_key,
        artifact_base_url=None,
        include_site_assets=False,
        docker_image=None,
    )

    manifest_path = output / builder.MANIFEST_NAME
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    lock_path = output / builder.LOCK_NAME
    lock = json.loads(lock_path.read_text(encoding="utf-8"))
    assert result["source_commit"] == head
    assert manifest["source"]["commit"] == head
    assert manifest["release_state"] == "published"
    assert manifest["trust"]["release_lock"] == builder.LOCK_NAME
    assert manifest["trust"]["signature_file"] == builder.SIGNATURE_NAME
    assert manifest["trust"]["release_lock_sha256"] == hashlib.sha256(
        lock_path.read_bytes()
    ).hexdigest()
    assert lock["source"]["commit"] == head
    assert lock["artifacts"]["upstream_runtime"] == manifest["artifacts"][
        "upstream_runtime"
    ]["sha256"]
    verify = subprocess.run(
        [
            "openssl",
            "pkeyutl",
            "-verify",
            "-pubin",
            "-inkey",
            str(public_key),
            "-sigfile",
            str(output / builder.SIGNATURE_NAME),
            "-rawin",
            "-in",
            str(manifest_path),
        ],
        capture_output=True,
    )
    assert verify.returncode == 0, verify.stderr.decode(errors="replace")
    private_bytes = private_key.read_bytes()
    assert all(
        private_bytes not in path.read_bytes()
        for path in output.iterdir()
        if path.is_file()
    )
