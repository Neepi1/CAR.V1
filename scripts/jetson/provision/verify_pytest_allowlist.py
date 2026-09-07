#!/usr/bin/env python3
"""Verify that pytest produced exactly the reviewed historical failure set."""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def load_allowlist(path: Path) -> set[str]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("schema") != "njrh.pytest_failure_allowlist.v1":
        raise ValueError("unsupported allowlist schema")
    failures = document.get("allowed_failures")
    if not isinstance(failures, list) or not all(
        isinstance(item, str) and "::" in item for item in failures
    ):
        raise ValueError("allowed_failures must be a list of pytest node IDs")
    if len(failures) != len(set(failures)):
        raise ValueError("allowlist contains duplicate node IDs")
    return set(failures)


def collect_failures(junit_path: Path, allowed: set[str]) -> set[str]:
    root = ET.parse(junit_path).getroot()
    by_test_name: dict[str, list[str]] = {}
    for node_id in allowed:
        by_test_name.setdefault(node_id.rsplit("::", 1)[-1], []).append(node_id)

    failures: set[str] = set()
    for testcase in root.iter("testcase"):
        if testcase.find("failure") is None and testcase.find("error") is None:
            continue
        name = testcase.attrib.get("name", "")
        file_name = testcase.attrib.get("file", "")
        if file_name:
            node_id = f"{file_name.replace(chr(92), '/')}::{name}"
        elif len(by_test_name.get(name, [])) == 1:
            node_id = by_test_name[name][0]
        else:
            classname = testcase.attrib.get("classname", "unknown")
            node_id = f"{classname}::{name}"
        failures.add(node_id)
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--junit", type=Path, required=True)
    parser.add_argument("--allowlist", type=Path, required=True)
    parser.add_argument("--pytest-exit-code", type=int, required=True)
    args = parser.parse_args()

    if args.pytest_exit_code not in (0, 1):
        print(
            json.dumps(
                {
                    "ok": False,
                    "reason": "pytest infrastructure/collection failure",
                    "pytest_exit_code": args.pytest_exit_code,
                }
            ),
            file=sys.stderr,
        )
        return 2

    allowed = load_allowlist(args.allowlist)
    actual = collect_failures(args.junit, allowed)
    missing = sorted(allowed - actual)
    unexpected = sorted(actual - allowed)
    report = {
        "schema": "njrh.pytest_failure_allowlist_result.v1",
        "ok": not missing and not unexpected,
        "allowed_count": len(allowed),
        "actual_count": len(actual),
        "missing_reviewed_failures": missing,
        "unexpected_failures": unexpected,
    }
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
