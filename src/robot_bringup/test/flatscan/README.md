# FlatScan Observer Tests

Run `python -m pytest src/robot_bringup/test/test_runtime_flatscan_management.py`.
The shell harness extracts functions without sourcing or executing startup.
Child identity, graph queries and recovery are stubbed. No production process,
network, ROS graph or service is touched.

Native policy tests require a local C++ compiler and RapidJSON headers. Set
`CXX` and `RAPIDJSON_INCLUDE_DIR` if needed. A skipped native test is not adequate
for deployment: parent must build/run `test_runtime_flatscan_snapshot.cpp` in
isolation and complete the ROS cases in `docs/runtime_flatscan_management.md`.
