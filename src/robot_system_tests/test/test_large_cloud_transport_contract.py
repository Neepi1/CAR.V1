from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
OVERLAY = ROOT / "scripts" / "jetson" / "runtime_overlay"


def test_resident_lidar_points_publisher_uses_udp_and_shm_transport():
    """The full-size resident cloud must not be forced through UDP locally."""
    helper = (
        OVERLAY / "scripts" / "large_cloud_fastdds_transport.sh"
    ).read_text(encoding="utf-8")
    run_driver = (OVERLAY / "scripts" / "run_driver.sh").read_text(
        encoding="utf-8"
    )

    assert "configure_large_cloud_fastdds_transport" in helper
    assert "NJRH_LARGE_CLOUD_FASTDDS_PROFILE_FILE" in helper
    assert "/tmp/njrh_large_cloud_fastdds_profile.xml" in helper
    assert "<type>UDPv4</type>" in helper
    assert "<type>SHM</type>" in helper
    assert "<useBuiltinTransports>false</useBuiltinTransports>" in helper
    assert "134217728" in helper

    assert 'source "${SCRIPT_DIR}/large_cloud_fastdds_transport.sh"' in run_driver
    assert "configure_large_cloud_fastdds_transport" in run_driver
    assert "njrh_run_large_cloud_affined" in run_driver
    assert "env -u FASTDDS_BUILTIN_TRANSPORTS" in run_driver
    assert (
        'FASTRTPS_DEFAULT_PROFILES_FILE="${NJRH_LARGE_CLOUD_FASTDDS_PROFILE_FILE}"'
        in run_driver
    )
    assert (
        'FASTDDS_DEFAULT_PROFILES_FILE="${NJRH_LARGE_CLOUD_FASTDDS_PROFILE_FILE}"'
        in run_driver
    )
    # Both supported /lidar_points owners must use the same transport contract.
    assert "njrh_run_large_cloud_affined hesai_ros_driver" in run_driver
    assert 'njrh_run_large_cloud_affined "${pointcloud_remap_service}"' in run_driver


def test_mapping_cloud_consumer_keeps_shm_and_udp_fallback():
    mapping_helper = (
        OVERLAY / "scripts" / "mapping_fastdds_transport.sh"
    ).read_text(encoding="utf-8")
    mapping_runner = (OVERLAY / "scripts" / "run_projected_map.sh").read_text(
        encoding="utf-8"
    )

    assert "<type>UDPv4</type>" in mapping_helper
    assert "<type>SHM</type>" in mapping_helper
    assert "env -u FASTDDS_BUILTIN_TRANSPORTS" in mapping_runner
    assert "NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE" in mapping_runner
