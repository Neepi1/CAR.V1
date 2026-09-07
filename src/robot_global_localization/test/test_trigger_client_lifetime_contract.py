#!/usr/bin/env python3

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    args = parser.parse_args()

    source = args.source.read_text(encoding="utf-8")
    create_expression = "create_client<std_srvs::srv::Empty>("

    assert source.count(create_expression) == 1, (
        "the Isaac trigger client must be created exactly once during node "
        "construction so callback-group waitables are never mutated while the "
        "MultiThreadedExecutor is spinning"
    )
    assert "grid_search_trigger_client_.reset()" not in source, (
        "a persistent ROS 2 service client rediscovers the reloaded Isaac "
        "server; resetting it inside the reload callback races the executor"
    )


if __name__ == "__main__":
    main()
