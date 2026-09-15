"""Source contract for the one-shot readiness node's auxiliary endpoints.

This does not instantiate ROS or prove endpoint/clock/readiness behavior. The
candidate binary still requires an isolated ROS graph and readiness smoke test.
"""

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PROBE_SOURCE = ROOT / "src/robot_bringup/src/runtime_readiness_probe.cpp"


def test_readiness_node_disables_unused_parameter_and_log_endpoints():
    source = PROBE_SOURCE.read_text(encoding="utf-8")
    main = source[source.index("int main("):]
    construction = re.search(
        r"auto\s+node\s*=\s*std::make_shared<rclcpp::Node>\s*\((.*?)\);",
        main,
        re.DOTALL,
    )
    assert construction is not None, "readiness node construction must be audited"
    arguments = re.sub(r"//[^\n]*|/\*.*?\*/", "", construction.group(1), flags=re.DOTALL)
    missing = [
        option
        for option in (
            "start_parameter_services",
            "start_parameter_event_publisher",
            "enable_rosout",
        )
        if not re.search(rf"\.{option}\s*\(\s*false\s*\)", arguments)
    ]
    assert not missing, f"unused observer endpoints were not disabled: {missing}"
