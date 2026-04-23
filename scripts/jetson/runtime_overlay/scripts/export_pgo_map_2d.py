#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import sys


def main() -> int:
    upstream_root = os.environ.get("NJRH_UPSTREAM_ROOT", "/workspaces/isaac_ros-dev")
    upstream_script = os.path.join(upstream_root, "scripts", "export_pgo_map_2d.py")
    if not os.path.isfile(upstream_script):
        print(f"[runtime-overlay] missing upstream export script: {upstream_script}", file=sys.stderr)
        return 1
    result = subprocess.run([sys.executable, upstream_script, *sys.argv[1:]], check=False)
    return int(result.returncode)


if __name__ == "__main__":
    raise SystemExit(main())
