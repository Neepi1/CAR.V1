#!/usr/bin/env python3
"""Exercise the package's real configure prelude without ROS dependencies."""
import pathlib
import subprocess
import tempfile


def main():
    package = pathlib.Path(__file__).resolve().parents[1]
    prelude = (package / "CMakeLists.txt").read_text(encoding="utf-8").split(
        "find_package(", 1
    )[0]
    with tempfile.TemporaryDirectory(prefix="njrh_api_build_policy_") as temporary:
        root = pathlib.Path(temporary)
        (root / "CMakeLists.txt").write_text(
            prelude + '\nfile(WRITE "${CMAKE_BINARY_DIR}/observed.txt" '
            '"${CMAKE_BUILD_TYPE}\\n${CMAKE_CXX_FLAGS_RELWITHDEBINFO}\\n")\n',
            encoding="utf-8",
        )
        for name, arguments, expected in (
            ("default", [], "RelWithDebInfo"),
            ("empty", ["-DCMAKE_BUILD_TYPE="], "RelWithDebInfo"),
            ("debug", ["-DCMAKE_BUILD_TYPE=Debug"], "Debug"),
            ("release", ["-DCMAKE_BUILD_TYPE=Release"], "Release"),
            ("multi", ["-DCMAKE_CONFIGURATION_TYPES=Debug;Release"], ""),
        ):
            build = root / name
            command = ["cmake", "-S", str(root), "-B", str(build), *arguments]
            result = subprocess.run(command, text=True, capture_output=True)
            assert result.returncode == 0, result.stdout + result.stderr
            observed = (build / "observed.txt").read_text().splitlines()
            assert observed[0] == expected, (
                f"{name}: expected {expected!r}, got {observed[0]!r}; "
                "an unspecified API build must not silently remain unoptimized"
            )
            print(f"PASS {name}: {observed[0]!r}")


if __name__ == "__main__":
    main()
