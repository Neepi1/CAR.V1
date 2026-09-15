#!/usr/bin/env python3
"""Startup evidence from the pinned Isaac launch; no ROS or hardware access.

Nitros emits the completion marker after runGraphAsync AND enabling input.
This is not the live floor-reload protocol or a localization success receipt.
"""
import argparse
import json
import os
from pathlib import Path
import time


def process_start_ticks(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return "" if fields[0] == "Z" else fields[19]
    except (OSError, IndexError):
        return ""


class IsaacStartupState:
    def __init__(self, path, map_yaml, localizer_yaml):
        self.path = Path(path) if path else None
        self.map_yaml = str(map_yaml)
        self.localizer_yaml = str(localizer_yaml)
        self.pid = None
        self.buffers = {}
        self.ready = False

    def write(self, state):
        if self.path is None:
            return
        payload = dict(state=state, pid=self.pid,
                       process_start_ticks=process_start_ticks(self.pid),
                       map_yaml=self.map_yaml, localizer_yaml=self.localizer_yaml,
                       observed_at_unix=time.time())
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload), encoding="utf-8")
        os.replace(temporary, self.path)

    def started(self, pid):
        self.pid = pid
        self.ready = False
        self.buffers.clear()
        self.write("initializing")

    def output(self, pid, stream, data):
        if pid != self.pid or self.ready:
            return
        text = self.buffers.get(stream, "") + data.decode("utf-8", errors="replace")
        lines = text.split("\n")
        self.buffers[stream] = lines.pop()[-4096:]
        for line in lines:
            if ("[occupancy_grid_localizer]" in line and
                    "[NitrosNode] Node was started" in line):
                self.ready = True
                self.write("ready")
                break

    def exited(self, pid):
        if pid == self.pid:
            self.ready = False
            self.write("exited")


def startup_ready(path, map_yaml):
    try:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        pid = int(payload["pid"])
        if (payload["state"] != "ready" or payload["map_yaml"] != map_yaml or
                pid <= 0 or not payload.get("process_start_ticks") or
                process_start_ticks(pid) != payload["process_start_ticks"]):
            return False
        os.kill(pid, 0)
        return True
    except (OSError, ValueError, KeyError, TypeError):
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", required=True)
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--timeout-sec", type=float, required=True)
    args = parser.parse_args()
    deadline = time.monotonic() + max(0.0, args.timeout_sec)
    while True:
        if startup_ready(args.file, args.map_yaml):
            print("[runtime-overlay] Isaac internal graph ready (current launch/process)", flush=True)
            return 0
        if time.monotonic() >= deadline:
            print("[runtime-overlay] waiting for Isaac internal graph: " + args.file, flush=True)
            return 1
        time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))


if __name__ == "__main__":
    raise SystemExit(main())
