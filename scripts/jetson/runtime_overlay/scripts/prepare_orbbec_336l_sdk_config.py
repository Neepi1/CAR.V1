#!/usr/bin/env python3

import argparse
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional, Sequence


DEFAULT_SOURCE = Path(
    "/opt/ros/humble/share/orbbec_camera/config/OrbbecSDKConfig_v2.0.xml"
)
DEFAULT_OUTPUT = Path("/tmp/njrh_orbbec_336l_sdk_config.xml")
OVERRIDES = {
    "StreamFailedRetry": "1",
    "MaxStartStreamDelayMs": "3000",
    "MaxFrameIntervalMs": "3000",
}
SDK_CONFIG_NAME = "OrbbecSDKConfig_v2.0.xml"


def prepare_config(source: Path, output: Path) -> dict[str, tuple[str, str]]:
    source_text = source.read_text(encoding="utf-8")
    root = ET.fromstring(source_text)
    depth = root.find(".//Gemini336L/Depth")
    if depth is None:
        raise RuntimeError("Gemini336L/Depth is missing from the Orbbec SDK config")

    section_match = re.search(
        r"<Gemini336L>.*?</Gemini336L>", source_text, flags=re.DOTALL
    )
    if section_match is None:
        raise RuntimeError("Gemini336L section is missing from the Orbbec SDK config")
    section = section_match.group(0)
    changes = {}
    for name, new_value in OVERRIDES.items():
        element = depth.find(name)
        if element is None:
            raise RuntimeError(f"Gemini336L/Depth/{name} is missing from the Orbbec SDK config")
        old_value = (element.text or "").strip()
        pattern = re.compile(rf"(<{name}>\s*)[^<]*(\s*</{name}>)")
        section, count = pattern.subn(
            lambda match: f"{match.group(1)}{new_value}{match.group(2)}",
            section,
            count=1,
        )
        if count != 1:
            raise RuntimeError(f"expected one Gemini336L/Depth/{name} value, found {count}")
        changes[name] = (old_value, new_value)

    output_text = (
        source_text[: section_match.start()]
        + section
        + source_text[section_match.end() :]
    )
    ET.fromstring(output_text)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(output_text, encoding="utf-8")
    return changes


def _symlink(source: Path, target: Path) -> None:
    target.symlink_to(source, target_is_directory=source.is_dir())


def _symlink_directory_entries(source: Path, target: Path) -> None:
    target.mkdir(parents=True)
    for entry in source.iterdir():
        _symlink(entry, target / entry.name)


def prepare_overlay(
    package_prefix: Path, overlay_prefix: Path
) -> dict[str, tuple[str, str]]:
    package_prefix = package_prefix.resolve()
    overlay_prefix = overlay_prefix.absolute()
    if overlay_prefix == Path(overlay_prefix.anchor) or len(overlay_prefix.parts) < 3:
        raise RuntimeError(f"unsafe Orbbec overlay path: {overlay_prefix}")

    source_share = package_prefix / "share" / "orbbec_camera"
    source_config = source_share / "config" / SDK_CONFIG_NAME
    if not source_config.is_file():
        raise RuntimeError(f"Orbbec SDK config is missing: {source_config}")

    if overlay_prefix.is_symlink() or overlay_prefix.is_file():
        overlay_prefix.unlink()
    elif overlay_prefix.exists():
        shutil.rmtree(overlay_prefix)

    target_share = overlay_prefix / "share" / "orbbec_camera"
    target_config_dir = target_share / "config"
    target_config_dir.mkdir(parents=True)
    resource = (
        overlay_prefix
        / "share"
        / "ament_index"
        / "resource_index"
        / "packages"
        / "orbbec_camera"
    )
    resource.parent.mkdir(parents=True)
    resource.write_text("", encoding="utf-8")

    for entry in source_share.iterdir():
        if entry.name == "launch":
            _symlink_directory_entries(entry, target_share / entry.name)
        elif entry.name != "config":
            _symlink(entry, target_share / entry.name)
    for entry in (source_share / "config").iterdir():
        if entry.name != SDK_CONFIG_NAME:
            _symlink(entry, target_config_dir / entry.name)

    extensions = package_prefix / "lib" / "extensions"
    if extensions.exists():
        target_extensions = overlay_prefix / "lib" / "extensions"
        target_extensions.parent.mkdir(parents=True)
        _symlink(extensions, target_extensions)

    return prepare_config(source_config, target_config_dir / SDK_CONFIG_NAME)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a Gemini 336L SDK config with official depth-stream retry enabled"
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--package-prefix", type=Path)
    parser.add_argument("--overlay-prefix", type=Path)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if bool(args.package_prefix) != bool(args.overlay_prefix):
        raise RuntimeError("--package-prefix and --overlay-prefix must be provided together")
    if args.package_prefix:
        changes = prepare_overlay(args.package_prefix, args.overlay_prefix)
        output = args.overlay_prefix / "share/orbbec_camera/config" / SDK_CONFIG_NAME
    else:
        changes = prepare_config(args.source, args.output)
        output = args.output
    rendered = " ".join(
        f"{name}={old}->{new}" for name, (old, new) in changes.items()
    )
    print(f"[runtime-overlay] Orbbec Gemini336L stream retry config: {rendered}")
    print(f"[runtime-overlay] Orbbec SDK runtime config: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
