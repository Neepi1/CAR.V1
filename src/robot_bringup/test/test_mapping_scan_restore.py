"""No ROS traffic: replay the cold-discovery symptom through the real shell helper."""
import os
from pathlib import Path
import subprocess


ROOT = Path(os.environ.get("NJRH_TEST_ROOT", Path(__file__).resolve().parents[3]))


def test_cold_graph_does_not_reenable_an_already_enabled_scan():
    script = r'''
set -e
source "$1/scripts/jetson/runtime_overlay/scripts/scan_ownership_helpers.sh"
runtime_readiness_probe() {
  case "$1" in
    exact-publisher-owner) [[ "$5" != 1 ]] ;;
    publisher-count) return 0 ;;
    # The native replacement retains one participant and observes enabled
    # status while graph metadata is late; it never writes in this case.
    scan-handoff) [[ "$2" == ensure ]] ;;
    *) return 2 ;;
  esac
}
set_resident_scan_output() { echo UNNECESSARY_SCAN_ENABLE; }
restore_navigation_scan_owner /scan
'''
    result = subprocess.run(
        [os.environ.get("BASH", "bash"), "-c", script, "fixture", str(ROOT)],
        text=True, capture_output=True, timeout=5,
    )
    assert result.returncode == 0, result.stderr
    assert "UNNECESSARY_SCAN_ENABLE" not in result.stdout
