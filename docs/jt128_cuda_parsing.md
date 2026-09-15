# JT128 CUDA point parsing

## Scope

Only move the existing Hesai JT128 point-calculation branch to CUDA. UDP reception,
IMU extraction/publication, host receive timestamps, echo filter 2, point layout,
20 Hz frame setting, canonical remap, `/scan`, SHM/Fast DDS, CPU0–4 allocation,
FAST-LIO2 and mechanical-arm software are not changed.

The ROS driver uses `LidarPointXYZIRT`; the earlier SDK microbenchmark used
`LidarPointXYZICRT`, which has the same compared XYZ/ring/intensity/timestamp fields
plus confidence. Deployment validation must also cover the actual ROS driver.
Neither this distinction nor the benchmark authorizes changing the public fields.

## Build and runtime selection

The root Hesai CMake defaults `FIND_CUDA=ON`, requires CUDA when enabled,
uses CUDA C++17 for ROS 2 Humble, and defaults architectures to `87` for Orin.
`-DCMAKE_CUDA_ARCHITECTURES=...` can explicitly select another GPU. Native CUDA
targets inherit Release optimization; the obsolete fixed `sm_61`/device-debug
flag lists were removed from the nested SDK CMake files. No parser algorithm
was edited.

Build `hesai_ros_driver` in Release with `-DFIND_CUDA=ON
-DCMAKE_CUDA_ARCHITECTURES=87`. The runtime's `run_driver.sh` writes
`use_gpu: true` into its temporary YAML through `NJRH_HESAI_USE_GPU` (default
`true`). It never rewrites the source calibration/vendor configuration in place.
The downstream legacy launch helper preserves this field. A CUDA build alone
does not activate GPU if a field override requests `false`.

An explicit CPU build is possible with `-DFIND_CUDA=OFF`, paired with
`NJRH_HESAI_USE_GPU=false`. Do not silently treat the SDK's CPU-only warning as
proof of GPU activation.

## Deployment

Build to a new `.runtime/hesai_cuda_<release>/` prefix while the existing driver
continues to run. Validate the candidate before changing package selection.
Preserve the existing installed package directory as a deployment backup;
replace the package entry with a link to the candidate install prefix instead
of overwriting shared libraries mapped by the old process.

Apply only the authorized full `njrh-runtime.service` restart. Do not invoke
`run_driver.sh` by hand or restart individual navigation nodes. Do not move the
robot for this deployment.

## Verification

- `python src/robot_system_tests/test/test_jt128_cuda_runtime.py` checks GPU
  defaults, Orin/Release/C++17 configuration, parity of CPU/CUDA entry files,
  and executes the real configuration-generation fragment with default and
  explicit CPU selections. Only `use_gpu` changes in the fixture.
- An actual-data CPU/CUDA parser replay checks retained point counts, XYZ,
  ring, intensity and timestamps. The initial short replay showed approximately
  88.5% less CPU time for ComputeXYZI, including CUDA copies/postprocessing;
  this is not the entire driver's reduction.
- Run the candidate ROS driver on an existing PCAP, isolated from the production
  ROS domain and PID namespace. The vendor PCAP EOF handler invokes `pkill -f
  rviz2`; PID isolation prevents that existing cleanup from touching production.
- Check the new process executable, CUDA library mappings, generated YAML,
  source/deployed hashes, single driver/remap ownership and sensor/status rates.
- Recheck navigation readiness and CPU0–4 thread masks after restart.

Long-duration operation with arm inference, moving navigation, mapping and rare
sensor conditions still require separate real-hardware acceptance. This change
does not add navigation/safety gates or modify the arm black box.

Deployment evidence is stored under `/tmp/njrh_reports/jt128_gpu_deploy_20260911_dnNe7o`.

## 2026-09-11 deployment outcome

CUDA is deployed and verified in the live JT128 process. The exact production
XYZIRT replay compared 1,528,576 points with no retained-count, ring, intensity
or timestamp mismatches (maximum XYZ difference 0.027 mm). The candidate ROS
driver published 27 replay frames with the unchanged 32-byte public point layout.
Three five-second live CPU windows measured driver CPU at 64.49% before and
22.64% after (100% is one logical core); this is a short-window observation,
not a whole-navigation speedup or long-duration acceptance.

**Full-chain acceptance is not passed.** After the authorized complete restart,
local odometry delivery stalled and the existing health guard automatically
restarted the chain. The next launch reached Nav2 ready, but odometry/TF delivery
stalled again and the API still reported AMCL tracking not ready. Pointcloud and
scan production continued near 20/15 Hz. These observations do not establish
whether the odometry failure was caused by the CUDA change. Do not hide this with
an automatic CPU rollback, extra manual restarts, relocalization, or gate changes.
