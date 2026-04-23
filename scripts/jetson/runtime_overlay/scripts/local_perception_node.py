#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import runpy
import sys


REPO_ROOT = Path(__file__).resolve().parents[3]
REPO_NODE = REPO_ROOT / "src" / "robot_local_perception" / "scripts" / "local_perception_node.py"


def main() -> None:
    if not REPO_NODE.exists():
        raise FileNotFoundError(f"repo local perception node missing: {REPO_NODE}")
    sys.path.insert(0, str(REPO_NODE.parent))
    runpy.run_path(str(REPO_NODE), run_name="__main__")


if __name__ == "__main__":
    main()
