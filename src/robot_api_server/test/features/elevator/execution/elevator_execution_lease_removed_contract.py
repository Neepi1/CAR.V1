#!/usr/bin/env python3
"""Lock the elevator-test boundary that no longer uses an execution lease."""

from __future__ import annotations

import argparse
from pathlib import Path
import re


def function_block(source: str, start_marker: str, end_marker: str) -> str:
    start = source.index(start_marker)
    end = source.index(end_marker, start)
    return source[start:end]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime-source", type=Path, required=True)
    parser.add_argument("--fsm-source", type=Path, required=True)
    args = parser.parse_args()

    runtime = args.runtime_source.read_text(encoding="utf-8")
    fsm = args.fsm_source.read_text(encoding="utf-8")

    for forbidden in (
        "SetExecutionLease",
        "set_execution_lease(",
        "execution_client_",
    ):
        assert forbidden not in runtime, (
            f"elevator runtime must not own the retired execution-lease seam: {forbidden}"
        )

    begin_floor_switch = function_block(
        runtime,
        "RuntimeResult begin_floor_switch",
        "RuntimeResult await_floor_switch",
    )
    await_floor_switch = function_block(
        runtime,
        "RuntimeResult await_floor_switch",
        "RuntimeResult verify_floor_ready",
    )
    for block_name, block in (
        ("begin_floor_switch", begin_floor_switch),
        ("await_floor_switch", await_floor_switch),
    ):
        assert "mode_keepalive_failure" not in block, (
            f"{block_name} must not cancel a held FloorSwitch because mode renewal failed"
        )
        assert "renew_mode_lease" not in block, (
            f"{block_name} must leave mode reacquisition to the next motion boundary"
        )

    retired_effect = re.compile(
        r"make_effect\s*\(\s*ElevatorEffectKind::k(?:Acquire|Release)ExecutionLease"
    )
    assert retired_effect.search(fsm) is None, (
        "new elevator FSM transactions must not emit execution-lease effects"
    )


if __name__ == "__main__":
    main()
