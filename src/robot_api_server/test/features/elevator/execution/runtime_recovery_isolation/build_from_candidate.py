#!/usr/bin/env python3
"""Build only the RuntimePort integration-test TU against a verified candidate.

No production write, colcon build or dependency installation. Candidate object
hashes are checked before reuse; --runtime-object is the explicit changed TU.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

def sha256_file(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

p = argparse.ArgumentParser()
p.add_argument("--manifest", required=True, type=Path)
p.add_argument("--compile-command", required=True, type=Path)
p.add_argument("--source", required=True, type=Path)
p.add_argument("--output", required=True, type=Path)
p.add_argument("--runtime-object", type=Path)
p.add_argument("--include", action="append", default=[])
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
m = json.loads(a.manifest.read_text())
for name, expected in m["inputs"].items():
    path = Path(name)
    if not path.is_absolute():
        path = Path(m["cwd"]) / path
    if sha256_file(path) != expected:
        raise SystemExit(f"candidate input drift: {path}")
compile = json.loads(a.compile_command.read_text())
compile = compile[:compile.index("-o")]
compile = [x for x in compile if x not in ("-O2", "-g")]
obj = a.output / "test_elevator_runtime_recovery.cpp.o"
source_snapshot = a.output / "test_elevator_runtime_recovery.cpp"
if a.source.resolve() != source_snapshot.resolve():
    shutil.copyfile(a.source, source_snapshot)
compile = compile[:1] + ["-I" + x for x in a.include] + compile[1:]
compile += ["-std=c++17", "-O0", "-g0", "-c", str(source_snapshot), "-o", str(obj)]
subprocess.run(compile, check=True)
link = list(m["link"])
link = [x for x in link if not x.endswith("/src/robot_api_server_node.cpp.o")]
if a.runtime_object:
    indices = [i for i, x in enumerate(link)
               if Path(x).name.startswith("elevator_ros_runtime_port")
               and x.endswith(".cpp.o")]
    if len(indices) != 1:
        raise SystemExit("expected exactly one runtime adapter object")
    link[indices[0]] = str(a.runtime_object)
binary = a.output / "test_elevator_runtime_recovery"
link[link.index("-o") + 1] = str(binary)
link += [str(obj), "-lgtest_main", "-lgtest", "-pthread"]
subprocess.run(link, cwd=m["cwd"], check=True)
(a.output / "build_commands.json").write_text(json.dumps({
    "compile": compile, "link": link,
    "source_snapshot": str(source_snapshot), "source_sha256": sha256_file(source_snapshot),
    "candidate_manifest": str(a.manifest),
    "runtime_object": str(a.runtime_object) if a.runtime_object else None,
    "runtime_object_sha256": sha256_file(a.runtime_object)
    if a.runtime_object else None,
}, indent=2))
print(binary)
