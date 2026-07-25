import importlib.util
import hashlib
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = (
    ROOT
    / "scripts"
    / "jetson"
    / "runtime_overlay"
    / "scripts"
    / "prepare_orbbec_336l_sdk_config.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("prepare_orbbec_336l_sdk_config", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_prepare_changes_only_gemini336l_depth_retry(tmp_path):
    module = load_module()
    source = tmp_path / "source.xml"
    output = tmp_path / "output.xml"
    source.write_text(
        """<?xml version="1.0"?>
<OrbbecSDKConfig>
  <!-- preserve this SDK-owned content byte-for-byte -->
  <Device>
    <Gemini336><Depth><StreamFailedRetry>0</StreamFailedRetry><MaxStartStreamDelayMs>2000</MaxStartStreamDelayMs><MaxFrameIntervalMs>2000</MaxFrameIntervalMs></Depth></Gemini336>
    <Gemini336L><Depth><StreamFailedRetry>0</StreamFailedRetry><MaxStartStreamDelayMs>2000</MaxStartStreamDelayMs><MaxFrameIntervalMs>2000</MaxFrameIntervalMs></Depth></Gemini336L>
  </Device>
</OrbbecSDKConfig>
""",
        encoding="utf-8",
    )

    changes = module.prepare_config(source, output)
    tree = ET.parse(output)

    assert changes == {
        "StreamFailedRetry": ("0", "1"),
        "MaxStartStreamDelayMs": ("2000", "3000"),
        "MaxFrameIntervalMs": ("2000", "3000"),
    }
    assert tree.findtext(".//Gemini336L/Depth/StreamFailedRetry") == "1"
    assert tree.findtext(".//Gemini336L/Depth/MaxStartStreamDelayMs") == "3000"
    assert tree.findtext(".//Gemini336L/Depth/MaxFrameIntervalMs") == "3000"
    assert tree.findtext(".//Gemini336/Depth/StreamFailedRetry") == "0"
    assert "  <!-- preserve this SDK-owned content byte-for-byte -->\n" in output.read_text(
        encoding="utf-8"
    )


@pytest.mark.skipif(os.name == "nt", reason="runtime ament overlay uses POSIX symlinks")
def test_prepare_overlay_preserves_package_and_overrides_only_sdk_xml(tmp_path):
    module = load_module()
    package_prefix = tmp_path / "base"
    package_share = package_prefix / "share" / "orbbec_camera"
    (package_share / "config").mkdir(parents=True)
    (package_share / "launch").mkdir()
    (package_prefix / "lib" / "extensions").mkdir(parents=True)
    (package_share / "launch" / "camera.launch.py").write_text("# launch\n")
    (package_share / "config" / "common.yaml").write_text("camera: true\n")
    (package_share / "config" / "OrbbecSDKConfig_v2.0.xml").write_text(
        """<OrbbecSDKConfig><Device><Gemini336L><Depth>
<StreamFailedRetry>0</StreamFailedRetry>
<MaxStartStreamDelayMs>2000</MaxStartStreamDelayMs>
<MaxFrameIntervalMs>2000</MaxFrameIntervalMs>
</Depth></Gemini336L></Device></OrbbecSDKConfig>""",
        encoding="utf-8",
    )
    overlay = tmp_path / "overlay"

    module.prepare_overlay(package_prefix, overlay)

    overlay_share = overlay / "share" / "orbbec_camera"
    assert (overlay / "share/ament_index/resource_index/packages/orbbec_camera").exists()
    assert (overlay_share / "launch").is_dir()
    assert not (overlay_share / "launch").is_symlink()
    assert (overlay_share / "launch/camera.launch.py").is_symlink()
    assert (overlay_share / "config/common.yaml").is_symlink()
    tree = ET.parse(overlay_share / "config/OrbbecSDKConfig_v2.0.xml")
    assert tree.findtext(".//Gemini336L/Depth/StreamFailedRetry") == "1"
    assert (overlay / "lib/extensions").is_symlink()


def test_launcher_uses_runtime_ament_overlay_not_ros_yaml_config_argument():
    launcher = (
        ROOT / "scripts/jetson/runtime_overlay/scripts/run_orbbec_336l_depth.sh"
    ).read_text(encoding="utf-8")

    assert "NJRH_ORBBEC_SDK_OVERLAY_PREFIX" in launcher
    assert 'AMENT_PREFIX_PATH="${SDK_OVERLAY_PREFIX}:${AMENT_PREFIX_PATH}"' in launcher
    assert "config_file_path:=" not in launcher


def test_launcher_uses_verified_amr_preset_with_explicit_default_rollback():
    launcher = (
        ROOT / "scripts/jetson/runtime_overlay/scripts/run_orbbec_336l_depth.sh"
    ).read_text(encoding="utf-8")
    preset_config = (
        ROOT / "scripts/jetson/runtime_overlay/config/orbbec_depth_preset.env"
    ).read_text(encoding="utf-8")
    preset = (
        ROOT
        / "scripts/jetson/runtime_overlay/config/orbbec_presets/G336X_AMR_Default_v0.0.5.bin"
    )

    assert 'NJRH_ORBBEC_DEVICE_PRESET="G336X AMR Default"' in preset_config
    assert "NJRH_ORBBEC_DEPTH_WIDTH=424" in preset_config
    assert "NJRH_ORBBEC_DEPTH_HEIGHT=266" in preset_config
    assert "NJRH_ORBBEC_DEPTH_FPS=30" in preset_config
    assert 'AMR_PRESET_NAME="G336X AMR Default"' in launcher
    assert 'DEVICE_PRESET="${NJRH_ORBBEC_DEVICE_PRESET:-${AMR_PRESET_NAME}}"' in launcher
    assert 'preset_firmware_path:="${PRESET_FIRMWARE_PATH}"' in launcher
    assert 'device_preset:="${DEVICE_PRESET}"' in launcher
    assert 'depth_width:="${DEPTH_WIDTH}"' in launcher
    assert 'depth_height:="${DEPTH_HEIGHT}"' in launcher
    assert 'depth_fps:="${DEPTH_FPS}"' in launcher
    assert "sha256sum" in launcher
    assert preset.is_file()
    assert hashlib.sha256(preset.read_bytes()).hexdigest() == (
        "db8f907e14380207b20b9f08018ac9a1aa2597894e234154141e2186fa355e3e"
    )
