from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = (
    ROOT
    / "scripts"
    / "jetson"
    / "runtime_overlay"
    / "scripts"
    / "ensure_costmap_filter_masks.py"
)
FNV64_OFFSET_BASIS = 1469598103934665603
FNV64_PRIME = 1099511628211


def write_pgm(path: Path, width: int, height: int, value: int = 254) -> None:
    path.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + bytes([value]) * width * height
    )


def write_yaml(
    path: Path,
    image: str,
    *,
    resolution: float = 0.05,
    origin: str = "[1.0, 2.0, 0.3]",
) -> None:
    path.write_text(
        "\n".join(
            (
                f"image: {image}",
                "mode: trinary",
                f"resolution: {resolution}",
                f"origin: {origin}",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            )
        ),
        encoding="utf-8",
    )


def fnv1a64(data: bytes) -> str:
    value = FNV64_OFFSET_BASIS
    for byte in data:
        value ^= byte
        value = (value * FNV64_PRIME) & 0xFFFFFFFFFFFFFFFF
    return f"{value:016x}"


def write_keepout_commit(keepout_yaml: Path, keepout_pgm: Path) -> None:
    semantic_path = keepout_yaml.parent / "keepout_semantic_layer.json"
    semantic_path.write_text(
        '{"schema":"njrh.keepout.semantic.v1","building_id":"B1",'
        '"floor_id":"F1","map_id":"map_test","keepout_lines":[],'
        '"keepout_polygons":[]}\n',
        encoding="utf-8",
    )
    marker = {
        "schema": "njrh.keepout.commit.v1",
        "revision": "keepout-v1-fnv64-0123456789abcdef",
        "semantic_file": semantic_path.name,
        "mask_yaml_file": keepout_yaml.name,
        "mask_pgm_file": keepout_pgm.name,
        "semantic_fnv64": fnv1a64(semantic_path.read_bytes()),
        "mask_yaml_fnv64": fnv1a64(keepout_yaml.read_bytes()),
        "mask_pgm_fnv64": fnv1a64(keepout_pgm.read_bytes()),
    }
    (keepout_yaml.parent / "keepout_commit.json").write_text(
        json.dumps(marker, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )


def run_generator(
    nav_yaml: Path,
    output_dir: Path,
    keepout_yaml: Path | None,
) -> subprocess.CompletedProcess[str]:
    command = [
        sys.executable,
        str(SCRIPT),
        "--nav-yaml",
        str(nav_yaml),
        "--output-dir",
        str(output_dir),
        "--stable-wait-sec",
        "0.15",
    ]
    if keepout_yaml is not None:
        command.extend(("--keepout-yaml", str(keepout_yaml)))
    return subprocess.run(command, text=True, capture_output=True, check=False)


def test_selected_valid_keepout_mask_is_staged(tmp_path: Path) -> None:
    nav_pgm = tmp_path / "nav.pgm"
    nav_yaml = tmp_path / "nav.yaml"
    keepout_pgm = tmp_path / "keepout.pgm"
    keepout_yaml = tmp_path / "keepout.yaml"
    write_pgm(nav_pgm, 8, 6)
    write_yaml(nav_yaml, nav_pgm.name)
    write_pgm(keepout_pgm, 8, 6, value=0)
    write_yaml(keepout_yaml, keepout_pgm.name)
    write_keepout_commit(keepout_yaml, keepout_pgm)

    output = tmp_path / "runtime"
    result = run_generator(nav_yaml, output, keepout_yaml)

    assert result.returncode == 0, result.stderr
    assert (output / "keepout_mask.pgm").read_bytes().endswith(bytes([0]) * 48)
    assert "keepout:source" in result.stdout


def test_selected_non_neutral_keepout_without_commit_fails_closed(tmp_path: Path) -> None:
    nav_pgm = tmp_path / "nav.pgm"
    nav_yaml = tmp_path / "nav.yaml"
    keepout_pgm = tmp_path / "keepout.pgm"
    keepout_yaml = tmp_path / "keepout.yaml"
    write_pgm(nav_pgm, 8, 6)
    write_yaml(nav_yaml, nav_pgm.name)
    write_pgm(keepout_pgm, 8, 6, value=0)
    write_yaml(keepout_yaml, keepout_pgm.name)

    output = tmp_path / "runtime"
    result = run_generator(nav_yaml, output, keepout_yaml)

    assert result.returncode == 2
    assert "integrity validation failed" in result.stderr
    assert not (output / "keepout_mask.pgm").exists()


def test_selected_invalid_keepout_mask_fails_closed(tmp_path: Path) -> None:
    nav_pgm = tmp_path / "nav.pgm"
    nav_yaml = tmp_path / "nav.yaml"
    keepout_pgm = tmp_path / "keepout.pgm"
    keepout_yaml = tmp_path / "keepout.yaml"
    write_pgm(nav_pgm, 8, 6)
    write_yaml(nav_yaml, nav_pgm.name)
    write_pgm(keepout_pgm, 7, 6)
    write_yaml(keepout_yaml, keepout_pgm.name)

    output = tmp_path / "runtime"
    result = run_generator(nav_yaml, output, keepout_yaml)

    assert result.returncode == 2
    assert "refusing to replace" in result.stderr
    assert not (output / "keepout_mask.pgm").exists()


def test_selected_keepout_geometry_mismatch_fails_closed(tmp_path: Path) -> None:
    nav_pgm = tmp_path / "nav.pgm"
    nav_yaml = tmp_path / "nav.yaml"
    keepout_pgm = tmp_path / "keepout.pgm"
    keepout_yaml = tmp_path / "keepout.yaml"
    write_pgm(nav_pgm, 8, 6)
    write_yaml(nav_yaml, nav_pgm.name)
    write_pgm(keepout_pgm, 8, 6)
    write_yaml(keepout_yaml, keepout_pgm.name, origin="[1.1, 2.0, 0.3]")

    output = tmp_path / "runtime"
    result = run_generator(nav_yaml, output, keepout_yaml)

    assert result.returncode == 2
    assert "geometry does not match" in result.stderr
    assert not (output / "keepout_mask.pgm").exists()


def test_selected_keepout_accepts_subpixel_origin_rounding(tmp_path: Path) -> None:
    nav_pgm = tmp_path / "nav.pgm"
    nav_yaml = tmp_path / "nav.yaml"
    keepout_pgm = tmp_path / "keepout.pgm"
    keepout_yaml = tmp_path / "keepout.yaml"
    write_pgm(nav_pgm, 8, 6)
    write_yaml(nav_yaml, nav_pgm.name, origin="[-34.3426, -19.1852, 0]")
    write_pgm(keepout_pgm, 8, 6, value=0)
    write_yaml(
        keepout_yaml,
        keepout_pgm.name,
        origin="[-34.342629, -19.185219, 0.000000]",
    )
    write_keepout_commit(keepout_yaml, keepout_pgm)

    output = tmp_path / "runtime"
    result = run_generator(nav_yaml, output, keepout_yaml)

    assert result.returncode == 0, result.stderr
    assert (output / "keepout_mask.pgm").read_bytes().endswith(bytes([0]) * 48)
    assert "keepout:source" in result.stdout


def test_missing_optional_keepout_source_generates_neutral_mask(tmp_path: Path) -> None:
    nav_pgm = tmp_path / "nav.pgm"
    nav_yaml = tmp_path / "nav.yaml"
    write_pgm(nav_pgm, 8, 6)
    write_yaml(nav_yaml, nav_pgm.name)

    output = tmp_path / "runtime"
    result = run_generator(nav_yaml, output, None)

    assert result.returncode == 0, result.stderr
    assert (output / "keepout_mask.pgm").read_bytes().endswith(bytes([254]) * 48)
    assert "keepout:neutral" in result.stdout
