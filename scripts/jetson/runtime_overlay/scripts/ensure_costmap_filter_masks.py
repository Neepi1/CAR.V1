#!/usr/bin/env python3
"""Create neutral Nav2 costmap filter masks when a floor bundle is not selected.

Nav2 costmap filters consume OccupancyGrid values. With a trinary map_server
mask, white/free PGM pixels load as OccupancyGrid value 0, which is the neutral
"no keepout / no speed restriction" value. Black pixels would load as occupied.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shlex
import shutil
import struct
import sys
import time
from pathlib import Path


NEUTRAL_FILTER_PIXEL = 254
# Match robot_api_server::fnv1a64. This project intentionally preserves its
# historical offset basis for revision compatibility.
FNV64_OFFSET_BASIS = 1469598103934665603
FNV64_PRIME = 1099511628211


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


def origin_numbers(value: str | None) -> tuple[float, float, float] | None:
    if not value:
        return None
    numbers = re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", value)
    if len(numbers) < 3:
        return None
    result = (float(numbers[0]), float(numbers[1]), float(numbers[2]))
    return result if all(math.isfinite(component) for component in result) else None


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


def fnv1a64(data: bytes) -> str:
    value = FNV64_OFFSET_BASIS
    for byte in data:
        value ^= byte
        value = (value * FNV64_PRIME) & 0xFFFFFFFFFFFFFFFF
    return f"{value:016x}"


def pgm_has_active_cells(path: Path) -> bool | None:
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
    if len(tokens) != 4:
        return None
    try:
        width = int(tokens[1])
        height = int(tokens[2])
        max_value = int(tokens[3])
    except ValueError:
        return None
    if width <= 0 or height <= 0 or max_value <= 0 or max_value > 255:
        return None
    sample_count = width * height
    if tokens[0] == b"P5":
        if index >= len(data) or not data[index:index + 1].isspace():
            return None
        if data[index:index + 2] == b"\r\n":
            index += 2
        else:
            index += 1
        samples = data[index:index + sample_count]
        if len(samples) != sample_count:
            return None
        threshold = int(249 * max_value / 255)
        return any(sample <= threshold for sample in samples)
    try:
        samples = [int(value) for value in data[index:].split()]
    except ValueError:
        return None
    if len(samples) != sample_count:
        return None
    threshold = int(249 * max_value / 255)
    return any(sample <= threshold for sample in samples)


def validate_keepout_commit(source_yaml: Path, image_path: Path) -> tuple[bool, str]:
    semantic_path = source_yaml.parent / "keepout_semantic_layer.json"
    marker_path = source_yaml.parent / "keepout_commit.json"
    if not semantic_path.exists():
        active = pgm_has_active_cells(image_path)
        if active is None:
            return False, "cannot inspect legacy keepout PGM occupancy"
        if active:
            return False, "non-neutral keepout mask has no semantic layer or commit marker"
        return True, "legacy neutral keepout mask"
    if not marker_path.is_file():
        return False, "managed keepout semantic layer has no commit marker"
    try:
        marker = json.loads(marker_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        return False, f"invalid keepout commit marker: {error}"
    if not isinstance(marker, dict) or marker.get("schema") != "njrh.keepout.commit.v1":
        return False, "keepout commit marker schema is invalid"
    expected_files = {
        "semantic_file": semantic_path.name,
        "mask_yaml_file": source_yaml.name,
        "mask_pgm_file": image_path.name,
    }
    for key, expected in expected_files.items():
        if marker.get(key) != expected:
            return False, f"keepout commit marker {key} mismatch"
    try:
        content_by_key = {
            "semantic_fnv64": semantic_path.read_bytes(),
            "mask_yaml_fnv64": source_yaml.read_bytes(),
            "mask_pgm_fnv64": image_path.read_bytes(),
        }
    except OSError as error:
        return False, f"failed to read committed keepout assets: {error}"
    for key, content in content_by_key.items():
        if marker.get(key) != fnv1a64(content):
            return False, f"keepout commit marker {key} mismatch"
    revision = marker.get("revision")
    if not isinstance(revision, str) or not revision.startswith("keepout-v1-fnv64-"):
        return False, "keepout commit marker revision is invalid"
    return True, revision


def write_bytes_atomic(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    tmp_path.write_bytes(data)
    os.replace(tmp_path, path)


def write_text_atomic(path: Path, text: str) -> None:
    write_bytes_atomic(path, text.encode("utf-8"))


def write_pgm(path: Path, width: int, height: int, value: int = NEUTRAL_FILTER_PIXEL) -> None:
    value = max(0, min(255, int(value)))
    remaining = width * height
    chunk = bytes([value]) * min(1024 * 1024, max(1, remaining))
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    with tmp_path.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        while remaining > 0:
            take = min(remaining, len(chunk))
            stream.write(chunk[:take])
            remaining -= take
    os.replace(tmp_path, path)


def write_mask_yaml(path: Path, image_name: str, resolution: float, origin: str) -> None:
    write_text_atomic(
        path,
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
    )


def resolve_image_path(mask_yaml: Path, values: dict[str, str]) -> Path | None:
    image = values.get("image")
    if not image:
        return None
    image_path = Path(image)
    return image_path if image_path.is_absolute() else mask_yaml.parent / image_path


def wait_for_stable_file(path: Path, timeout_sec: float) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_sec)
    last_signature: tuple[int, int] | None = None
    while time.monotonic() <= deadline:
        try:
            stat = path.stat()
        except OSError:
            time.sleep(0.05)
            continue
        if stat.st_size <= 0:
            time.sleep(0.05)
            continue
        signature = (stat.st_size, stat.st_mtime_ns)
        if signature == last_signature:
            return True
        last_signature = signature
        time.sleep(0.05)
    return False


def stage_source_mask(
    source_yaml: Path | None,
    output_yaml: Path,
    output_pgm: Path,
    fallback_resolution: float,
    fallback_origin: str,
    expected_dimensions: tuple[int, int],
    stable_wait_sec: float,
    require_keepout_integrity: bool = False,
) -> bool:
    if not source_yaml:
        return False
    source_yaml = source_yaml.expanduser()
    if not wait_for_stable_file(source_yaml, stable_wait_sec):
        print(
            f"[runtime-overlay] filter mask source not stable; using neutral mask: {source_yaml}",
            file=sys.stderr,
        )
        return False

    values = read_simple_map_yaml(source_yaml)
    image_path = resolve_image_path(source_yaml, values)
    if not image_path or not wait_for_stable_file(image_path, stable_wait_sec):
        print(
            f"[runtime-overlay] filter mask image not stable; using neutral mask: {source_yaml}",
            file=sys.stderr,
        )
        return False

    dimensions = image_dimensions(image_path)
    if dimensions != expected_dimensions:
        print(
            "[runtime-overlay] filter mask dimensions do not match Nav2 map; "
            f"using neutral mask: {source_yaml} dims={dimensions} expected={expected_dimensions}",
            file=sys.stderr,
        )
        return False

    try:
        source_resolution = float(values.get("resolution", "nan"))
    except ValueError:
        source_resolution = math.nan
    source_origin = origin_numbers(values.get("origin"))
    expected_origin = origin_numbers(fallback_origin)
    # Floor assets can serialize the same map origin with different decimal
    # precision (for example 4 versus 6 places).  Accept only sub-millimetre,
    # sub-pixel rounding on x/y; keep yaw strict so a genuinely different map
    # cannot be staged.
    origin_xy_tolerance = max(1e-6, fallback_resolution * 1e-3)
    origin_yaw_tolerance = 1e-6
    origin_mismatch = (
        source_origin is None
        or expected_origin is None
        or abs(source_origin[0] - expected_origin[0]) > origin_xy_tolerance
        or abs(source_origin[1] - expected_origin[1]) > origin_xy_tolerance
        or abs(source_origin[2] - expected_origin[2]) > origin_yaw_tolerance
    )
    if (
        not math.isfinite(source_resolution)
        or source_resolution <= 0.0
        or abs(source_resolution - fallback_resolution) > 1e-9
        or origin_mismatch
    ):
        print(
            "[runtime-overlay] filter mask geometry does not match Nav2 map; "
            f"source={source_yaml} resolution={values.get('resolution')} "
            f"origin={values.get('origin')} expected_resolution={fallback_resolution} "
            f"expected_origin={fallback_origin}",
            file=sys.stderr,
        )
        return False

    if require_keepout_integrity:
        integrity_ok, integrity_detail = validate_keepout_commit(source_yaml, image_path)
        if not integrity_ok:
            print(
                "[runtime-overlay] keepout source integrity validation failed; "
                f"source={source_yaml} detail={integrity_detail}",
                file=sys.stderr,
            )
            return False

    tmp_pgm = output_pgm.with_name(f".{output_pgm.name}.tmp.{os.getpid()}")
    output_pgm.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(image_path, tmp_pgm)
    if image_dimensions(tmp_pgm) != expected_dimensions:
        tmp_pgm.unlink(missing_ok=True)
        print(
            f"[runtime-overlay] copied filter mask failed validation; using neutral mask: {source_yaml}",
            file=sys.stderr,
        )
        return False
    os.replace(tmp_pgm, output_pgm)

    resolution = float(values.get("resolution", str(fallback_resolution)))
    origin = parse_origin(values.get("origin") or fallback_origin)
    write_mask_yaml(output_yaml, output_pgm.name, resolution, origin)
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nav-yaml", default="", help="Current Nav2 map yaml. Optional.")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--keepout-yaml", default="", help="Selected keepout mask yaml to stage. Optional.")
    parser.add_argument("--speed-yaml", default="", help="Selected speed mask yaml to stage. Optional.")
    parser.add_argument("--binary-yaml", default="", help="Selected binary mask yaml to stage. Optional.")
    parser.add_argument("--stable-wait-sec", type=float, default=3.0)
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

    staged_sources = []
    for source_yaml, yaml_path, pgm_path, label in (
        (args.keepout_yaml, keepout_yaml, keepout_pgm, "keepout"),
        (args.speed_yaml, speed_yaml, speed_pgm, "speed"),
        (args.binary_yaml, binary_yaml, binary_pgm, "binary"),
    ):
        staged = stage_source_mask(
            Path(source_yaml) if source_yaml else None,
            yaml_path,
            pgm_path,
            resolution,
            origin,
            (width, height),
            args.stable_wait_sec,
            require_keepout_integrity=label == "keepout",
        )
        if staged:
            staged_sources.append(f"{label}:source")
        else:
            if label == "keepout" and source_yaml:
                print(
                    "[runtime-overlay] refusing to replace a selected but invalid keepout mask "
                    f"with a neutral mask: {source_yaml}",
                    file=sys.stderr,
                )
                return 2
            write_pgm(pgm_path, width, height)
            write_mask_yaml(yaml_path, pgm_path.name, resolution, origin)
            staged_sources.append(f"{label}:neutral")

    exports = {
        "NAV2_KEEP_OUT_MASK_YAML": keepout_yaml,
        "NAV2_SPEED_MASK_YAML": speed_yaml,
        "NAV2_BINARY_MASK_YAML": binary_yaml,
    }
    for key, value in exports.items():
        print(f"export {key}={shlex.quote(os.fspath(value))}")
    source_text = ", ".join(staged_sources)
    print(
        "echo "
        + shlex.quote(
            f"[runtime-overlay] using staged costmap filter masks: {width}x{height} ({source_text}) "
            f"dir={output_dir}"
        )
        + " >&2"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
