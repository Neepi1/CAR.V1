#!/usr/bin/env python3
from __future__ import annotations

import argparse
import binascii
import json
from pathlib import Path
import shutil
import struct
import zlib


REQUIRED_RELATIVE_ASSETS = (
    "nav/nav_map.yaml",
    "nav/nav_map.pgm",
    "localizer/localizer_map.png",
    "localizer/localizer_params.yaml",
    "filters/keepout_mask.yaml",
    "filters/keepout_mask.pgm",
    "filters/speed_mask.yaml",
    "filters/speed_mask.pgm",
    "filters/binary_mask.yaml",
    "filters/binary_mask.pgm",
    "reports/asset_report.json",
    "poses.yaml",
)


def save_pgm(path: Path, width: int, height: int, value: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + bytes([value]) * width * height)


def _png_chunk(tag: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + tag
        + payload
        + struct.pack(">I", binascii.crc32(tag + payload) & 0xFFFFFFFF)
    )


def save_png(path: Path, width: int, height: int, value: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = b"".join(b"\x00" + bytes([value]) * width for _ in range(height))
    payload = b"".join(
        [
            b"\x89PNG\r\n\x1a\n",
            _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)),
            _png_chunk(b"IDAT", zlib.compress(raw)),
            _png_chunk(b"IEND", b""),
        ]
    )
    path.write_bytes(payload)


def write_map_yaml(path: Path, image_name: str, resolution: float, origin_x: float, origin_y: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                f"resolution: {resolution:.6f}",
                f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "mode: trinary",
                "",
            ]
        ),
        encoding="utf-8",
    )


def write_map_yaml_from_values(path: Path, image_name: str, values: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                f"resolution: {values.get('resolution', '0.05')}",
                f"origin: {values.get('origin', '[0.0, 0.0, 0.0]')}",
                f"negate: {values.get('negate', '0')}",
                f"occupied_thresh: {values.get('occupied_thresh', '0.65')}",
                f"free_thresh: {values.get('free_thresh', '0.196')}",
                f"mode: {values.get('mode', 'trinary')}",
                "",
            ]
        ),
        encoding="utf-8",
    )


def read_map_yaml(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def resolve_yaml_image(source_yaml: Path, values: dict[str, str]) -> Path:
    if "image" not in values:
        raise RuntimeError(f"map yaml missing image field: {source_yaml}")
    source_image = Path(values["image"].strip().strip('"').strip("'"))
    if not source_image.is_absolute():
        source_image = (source_yaml.parent / source_image).resolve()
    if not source_image.exists():
        raise RuntimeError(f"map image missing: {source_image}")
    return source_image


def read_pgm_size(path: Path) -> tuple[int, int]:
    with path.open("rb") as handle:
        magic = handle.readline().strip()
        if magic != b"P5":
            raise RuntimeError(f"unsupported PGM format in {path}: {magic!r}")
        line = handle.readline().strip()
        while line.startswith(b"#"):
            line = handle.readline().strip()
        width_text, height_text = line.split()[:2]
        return int(width_text), int(height_text)


def copy_map_yaml_with_local_image(source_yaml: Path, target_yaml: Path, target_image: Path) -> None:
    values = read_map_yaml(source_yaml)
    source_image = resolve_yaml_image(source_yaml, values)

    target_image.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source_image, target_image)
    write_map_yaml_from_values(target_yaml, target_image.name, values)


def validate_floor_assets(floor_root: Path) -> list[str]:
    return [rel for rel in REQUIRED_RELATIVE_ASSETS if not (floor_root / rel).exists()]


def write_asset_report(floor_root: Path, building_id: str, floor_id: str, width: int, height: int, resolution: float) -> None:
    report_path = floor_root / "reports" / "asset_report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(
            {
                "producer": "robot_map_toolkit",
                "building_id": building_id,
                "floor_id": floor_id,
                "resolution": resolution,
                "width": width,
                "height": height,
                "nav_map": str(floor_root / "nav" / "nav_map.yaml"),
                "localizer_map": str(floor_root / "localizer" / "localizer_params.yaml"),
                "filters": {
                    "keepout": str(floor_root / "filters" / "keepout_mask.yaml"),
                    "speed": str(floor_root / "filters" / "speed_mask.yaml"),
                    "binary": str(floor_root / "filters" / "binary_mask.yaml"),
                },
            },
            indent=2,
        ),
        encoding="utf-8",
    )


def promote_flat_map(
    flat_maps_dir: Path,
    flat_map_name: str,
    floor_root: Path,
    building_id: str,
    floor_id: str,
    width: int,
    height: int,
    resolution: float,
) -> None:
    nav_source_yaml = flat_maps_dir / f"{flat_map_name}.yaml"
    localizer_source_yaml = flat_maps_dir / f"{flat_map_name}.localizer.yaml"
    if not nav_source_yaml.exists():
        raise RuntimeError(f"flat Nav2 map yaml does not exist: {nav_source_yaml}")
    if not localizer_source_yaml.exists():
        raise RuntimeError(f"flat localizer map yaml does not exist: {localizer_source_yaml}")

    nav_values = read_map_yaml(nav_source_yaml)
    nav_source_image = resolve_yaml_image(nav_source_yaml, nav_values)
    mask_width, mask_height = read_pgm_size(nav_source_image)
    nav_resolution = float(nav_values.get("resolution", resolution))

    copy_map_yaml_with_local_image(nav_source_yaml, floor_root / "nav" / "nav_map.yaml", floor_root / "nav" / "nav_map.pgm")
    copy_map_yaml_with_local_image(
        localizer_source_yaml,
        floor_root / "localizer" / "localizer_params.yaml",
        floor_root / "localizer" / "localizer_map.png",
    )

    for stem in ("keepout_mask", "speed_mask", "binary_mask"):
        pgm = floor_root / "filters" / f"{stem}.pgm"
        if not pgm.exists():
            save_pgm(pgm, mask_width, mask_height, 0)
        write_map_yaml_from_values(floor_root / "filters" / f"{stem}.yaml", f"{stem}.pgm", nav_values)

    poses = floor_root / "poses.yaml"
    if not poses.exists():
        poses.write_text("poses: []\n", encoding="utf-8")

    write_asset_report(floor_root, building_id, floor_id, mask_width, mask_height, nav_resolution)


def ensure_floor_assets(
    root: Path,
    building_id: str,
    floor_id: str,
    width: int,
    height: int,
    resolution: float,
) -> Path:
    floor_root = root / building_id / floor_id
    origin_x = -0.5 * width * resolution
    origin_y = -0.5 * height * resolution

    nav_pgm = floor_root / "nav" / "nav_map.pgm"
    localizer_png = floor_root / "localizer" / "localizer_map.png"
    if not nav_pgm.exists():
        save_pgm(nav_pgm, width, height, 205)
    if not localizer_png.exists():
        save_png(localizer_png, width, height, 205)

    write_map_yaml(floor_root / "nav" / "nav_map.yaml", "nav_map.pgm", resolution, origin_x, origin_y)
    write_map_yaml(
        floor_root / "localizer" / "localizer_params.yaml",
        "localizer_map.png",
        resolution,
        origin_x,
        origin_y,
    )

    for stem in ("keepout_mask", "speed_mask", "binary_mask"):
        pgm = floor_root / "filters" / f"{stem}.pgm"
        if not pgm.exists():
            save_pgm(pgm, width, height, 0)
        write_map_yaml(floor_root / "filters" / f"{stem}.yaml", f"{stem}.pgm", resolution, origin_x, origin_y)

    poses = floor_root / "poses.yaml"
    if not poses.exists():
        poses.write_text("poses: []\n", encoding="utf-8")

    write_asset_report(floor_root, building_id, floor_id, width, height, resolution)
    return floor_root


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--maps-root", type=Path, default=Path("maps_release"))
    parser.add_argument("--building-id", default="building_1")
    parser.add_argument("--floor-id", default="floor_1")
    parser.add_argument("--width", type=int, default=1)
    parser.add_argument("--height", type=int, default=1)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--flat-maps-dir", type=Path)
    parser.add_argument("--flat-map-name", default="")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()

    floor_root = args.maps_root / args.building_id / args.floor_id
    if not args.validate_only:
        if args.flat_map_name:
            if args.flat_maps_dir is None:
                raise RuntimeError("--flat-maps-dir is required with --flat-map-name")
            floor_root.mkdir(parents=True, exist_ok=True)
            promote_flat_map(
                args.flat_maps_dir,
                args.flat_map_name,
                floor_root,
                args.building_id,
                args.floor_id,
                args.width,
                args.height,
                args.resolution,
            )
        else:
            floor_root = ensure_floor_assets(
                args.maps_root,
                args.building_id,
                args.floor_id,
                args.width,
                args.height,
                args.resolution,
            )

    missing = validate_floor_assets(floor_root)
    if missing:
        print(json.dumps({"ok": False, "floor_root": str(floor_root), "missing": missing}, indent=2))
        return 2

    print(json.dumps({"ok": True, "floor_root": str(floor_root)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
