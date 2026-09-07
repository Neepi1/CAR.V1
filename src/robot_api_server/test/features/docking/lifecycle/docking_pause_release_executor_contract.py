#!/usr/bin/env python3
"""Prevent synchronous ROS service waits inside the docking-status callback."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    args = parser.parse_args()

    source = args.source.read_text(encoding="utf-8")
    start = source.index("if (deferred_pause_release_job_id != 0U)")
    end = source.index("if (post_undock_job_id != 0U)", start)
    release_block = source[start:end]

    assert "ports_.post_deferred_work" in release_block, (
        "docking-status callbacks run on a SingleThreadedExecutor; the correction-pause "
        "release must be posted to the injected deferred work queue before waiting for its "
        "ROS service response"
    )


if __name__ == "__main__":
    main()
