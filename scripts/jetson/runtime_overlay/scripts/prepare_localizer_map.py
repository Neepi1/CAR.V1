#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import shutil

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - runtime fallback
    cv2 = None

try:
    from PIL import Image  # type: ignore
except ImportError:  # pragma: no cover - runtime fallback
    Image = None


SUPPORTED_LOCALIZER_EXTS = {".png", ".jpg", ".jpeg"}


def read_nav2_yaml(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def _write_png_copy(image_path: Path, target_png: Path) -> Path:
    target_png.parent.mkdir(parents=True, exist_ok=True)
    if image_path.resolve() == target_png.resolve():
        return target_png

    if image_path.suffix.lower() == ".png":
        shutil.copyfile(image_path, target_png)
        return target_png

    if cv2 is not None:
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise RuntimeError(f"Failed to load map image for PNG conversion: {image_path}")
        if not cv2.imwrite(str(target_png), image):
            raise RuntimeError(f"Failed to write PNG map image: {target_png}")
        return target_png

    if Image is not None:
        with Image.open(image_path) as image:
            image.save(target_png)
        return target_png

    raise RuntimeError("Neither OpenCV nor Pillow is available for localizer PNG conversion")


def ensure_png(image_path: Path) -> Path:
    return _write_png_copy(image_path, image_path.with_suffix(".png"))


def write_localizer_yaml(source_yaml: Path, localizer_yaml: Path, image_path: Path) -> None:
    values = read_nav2_yaml(source_yaml)
    image_ref = image_path.name if image_path.parent == localizer_yaml.parent else str(image_path)
    localizer_yaml.write_text(
        "\n".join([
            f"image: {image_ref}",
            f"resolution: {values.get('resolution', '0.05')}",
            f"origin: {values.get('origin', '[0.0, 0.0, 0.0]')}",
            f"negate: {values.get('negate', '0')}",
            f"occupied_thresh: {values.get('occupied_thresh', '0.65')}",
            f"free_thresh: {values.get('free_thresh', '0.196')}",
            f"mode: {values.get('mode', 'trinary')}",
            "",
        ]),
        encoding="utf-8",
    )


def prepare_localizer_assets(nav_yaml: Path, output_dir: Path | None) -> tuple[Path, Path]:
    values = read_nav2_yaml(nav_yaml)
    image_value = values.get("image", "").strip().strip('"').strip("'")
    if not image_value:
        raise RuntimeError(f"Nav2 map yaml is missing image: {nav_yaml}")

    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = (nav_yaml.parent / image_path).resolve()
    if not image_path.exists():
        raise RuntimeError(f"Nav2 map image does not exist: {image_path}")

    if image_path.suffix.lower() not in SUPPORTED_LOCALIZER_EXTS and image_path.suffix.lower() != ".pgm":
        raise RuntimeError(f"Unsupported localizer map source image: {image_path.suffix}")

    if output_dir is None:
        output_dir = nav_yaml.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    localizer_png = output_dir / f"{nav_yaml.stem}.localizer.png"
    localizer_yaml = output_dir / f"{nav_yaml.stem}.localizer.yaml"
    _write_png_copy(image_path, localizer_png)
    write_localizer_yaml(nav_yaml, localizer_yaml, localizer_png)
    return localizer_yaml, localizer_png


def prepare_localizer_map(nav_yaml: Path, output_dir: Path | None) -> Path:
    localizer_yaml, _ = prepare_localizer_assets(nav_yaml, output_dir)
    return localizer_yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nav-yaml", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    output_dir = args.output_dir.resolve() if args.output_dir is not None else None
    localizer_yaml = prepare_localizer_map(args.nav_yaml.resolve(), output_dir)
    print(localizer_yaml)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
