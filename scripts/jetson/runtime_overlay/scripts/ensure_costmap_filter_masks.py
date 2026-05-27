#!/usr/bin/env python3
"""Create neutral Nav2 costmap filter masks when a floor bundle is not selected.

Nav2 costmap filters consume OccupancyGrid values. With a trinary map_server
mask, white/free PGM pixels load as OccupancyGrid value 0, which is the neutral
"no keepout / no speed restriction" value. Black pixels would load as occupied.
"""

from __future__ import annotations

import argparse
import os
import re
import shlex
import struct
from pathlib import Path


NEUTRAL_FILTER_PIXEL = 254


def strip_yaml_value(value: str) -> str:
    value = value.split("#", 1)[0].strip()
    if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    return value


def read_simple_map_yaml(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path or not path.exists():
        return values
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        match = re.match(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*?)\s*$", line)
        if not match:
            continue
        key, value = match.groups()
        values[key] = strip_yaml_value(value)
    return values


def parse_origin(value: str | None) -> str:
    if not value:
        return "[0.0, 0.0, 0.0]"
    numbers = re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", value)
    if len(numbers) < 3:
        return "[0.0, 0.0, 0.0]"
    return f"[{float(numbers[0]):.6g}, {float(numbers[1]):.6g}, {float(numbers[2]):.6g}]"


def pgm_dimensions(path: Path) -> tuple[int, int] | None:
    data = path.read_bytes()
    if not data.startswith((b"P2", b"P5")):
        return None
    tokens: list[bytes] = []
    index = 0
    while index < len(data) and len(tokens) < 4:
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if index < len(data) and data[index:index + 1] == b"#":
            while index < len(data) and data[index:index + 1] not in (b"\n", b"\r"):
                index += 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        if start != index:
            tokens.append(data[start:index])
    if len(tokens) < 3:
        return None
    return max(1, int(tokens[1])), max(1, int(tokens[2]))


def png_dimensions(path: Path) -> tuple[int, int] | None:
    data = path.read_bytes()[:24]
    if len(data) < 24 or not data.startswith(b"\x89PNG\r\n\x1a\n"):
        return None
    return struct.unpack(">II", data[16:24])


def image_dimensions(path: Path | None) -> tuple[int, int]:
    if not path or not path.exists():
        return 1, 1
    if path.suffix.lower() == ".png":
        dims = png_dimensions(path)
    else:
        dims = pgm_dimensions(path)
    if dims is None:
        return 1, 1
    return dims


def write_pgm(path: Path, width: int, height: int, value: int = NEUTRAL_FILTER_PIXEL) -> None:
    value = max(0, min(255, int(value)))
    remaining = width * height
    chunk = bytes([value]) * min(1024 * 1024, max(1, remaining))
    with path.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        while remaining > 0:
            take = min(remaining, len(chunk))
            stream.write(chunk[:take])
            remaining -= take


def write_mask_yaml(path: Path, image_name: str, resolution: float, origin: str) -> None:
    path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                "mode: trinary",
                f"resolution: {resolution:.12g}",
                f"origin: {origin}",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nav-yaml", default="", help="Current Nav2 map yaml. Optional.")
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    nav_yaml = Path(args.nav_yaml).expanduser() if args.nav_yaml else None
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    values = read_simple_map_yaml(nav_yaml) if nav_yaml else {}
    resolution = float(values.get("resolution", "0.05"))
    origin = parse_origin(values.get("origin"))
    image_path = None
    if nav_yaml and values.get("image"):
        candidate = Path(values["image"])
        image_path = candidate if candidate.is_absolute() else nav_yaml.parent / candidate
    width, height = image_dimensions(image_path)

    keepout_pgm = output_dir / "keepout_mask.pgm"
    speed_pgm = output_dir / "speed_mask.pgm"
    binary_pgm = output_dir / "binary_mask.pgm"
    keepout_yaml = output_dir / "keepout_mask.yaml"
    speed_yaml = output_dir / "speed_mask.yaml"
    binary_yaml = output_dir / "binary_mask.yaml"

    for pgm in (keepout_pgm, speed_pgm, binary_pgm):
        write_pgm(pgm, width, height)
    write_mask_yaml(keepout_yaml, keepout_pgm.name, resolution, origin)
    write_mask_yaml(speed_yaml, speed_pgm.name, resolution, origin)
    write_mask_yaml(binary_yaml, binary_pgm.name, resolution, origin)

    exports = {
        "NAV2_KEEP_OUT_MASK_YAML": keepout_yaml,
        "NAV2_SPEED_MASK_YAML": speed_yaml,
        "NAV2_BINARY_MASK_YAML": binary_yaml,
    }
    for key, value in exports.items():
        print(f"export {key}={shlex.quote(os.fspath(value))}")
    print(f"echo '[runtime-overlay] using neutral costmap filter masks: {width}x{height}' >&2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
