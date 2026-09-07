#!/usr/bin/env python3
"""Lock the top-level elevator vertical slice outside the API composition root."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--node-source", type=Path, required=True)
    parser.add_argument("--module-header", type=Path, required=True)
    parser.add_argument("--module-source", type=Path, required=True)
    args = parser.parse_args()

    node = args.node_source.read_text(encoding="utf-8")
    header = args.module_header.read_text(encoding="utf-8")
    source = args.module_source.read_text(encoding="utf-8")
    router_wiring_path = (
        args.node_source.parent
        / "application"
        / "routing"
        / "application_router_wiring.cpp"
    )
    router_wiring = router_wiring_path.read_text(encoding="utf-8")

    assert "class ElevatorModule" in header
    for seam in (
        "handle_http(",
        "interlock_response(",
        "capture_motion_admission_epoch(",
        "acquire_motion_admission(",
        "execution_interlock(",
        "query_map_reference(",
    ):
        assert seam in header, f"ElevatorModule is missing public seam: {seam}"

    for route in (
        "/api/v1/elevator-config",
        "/api/v1/elevator-config/draft",
        "/api/v1/elevator-config/publish",
        "/api/v1/elevator-config/rollback",
        "/api/v1/elevator-test/start",
        "/api/v1/elevator-test/state",
        "/api/v1/elevator-test/confirm",
        "/api/v1/elevator-test/cancel",
        "/api/v1/elevator-test/recover",
    ):
        assert route in source, f"ElevatorModule does not own route: {route}"

    assert "elevator->handle_http" in router_wiring
    assert "elevator->interlock_response" in router_wiring

    for forbidden in (
        "HttpResponse handle_get_elevator_configuration(",
        "HttpResponse handle_save_elevator_configuration_draft(",
        "HttpResponse handle_publish_elevator_configuration(",
        "HttpResponse handle_rollback_elevator_configuration(",
        "HttpResponse handle_start_elevator_test(",
        "HttpResponse handle_elevator_test_state(",
        "HttpResponse handle_confirm_elevator_test(",
        "HttpResponse handle_cancel_elevator_test(",
        "HttpResponse handle_recover_elevator_test(",
        "std::unique_ptr<ElevatorConfigurationModule> elevator_configuration_",
        "std::shared_ptr<ElevatorRosRuntimePort> elevator_runtime_port_",
        "std::unique_ptr<ElevatorTestModule> elevator_test_",
        "std::shared_ptr<ElevatorMotionAdmissionFence>\n    elevator_motion_admission_fence_",
        "std::make_unique<ElevatorConfigurationModule>(",
        "std::make_unique<ElevatorTestModule>(",
    ):
        assert forbidden not in node, (
            "API composition root still owns elevator implementation: " + forbidden
        )


if __name__ == "__main__":
    main()
