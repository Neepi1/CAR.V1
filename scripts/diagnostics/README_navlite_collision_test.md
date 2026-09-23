# CollisionMonitor NAVLITE candidate validation

This test drives the **real** Humble 1.1.19 CollisionMonitor executable through
its Twist/LaserScan interfaces, observes its actual output subscription, and
checks emitted diagnostic events. It is not a vehicle test or synthetic-log
substitute. Run only inside private network, IPC and mount namespaces with a
fresh tmpfs `/dev/shm`; the test refuses the production namespaces.

The version-pinned patch adds diagnostics only. In particular, Scan timeout or
missing TF does **not** become a new STOP rule: the unchanged native behavior
and the real output are recorded without relabelling it as an obstacle stop.
`published=0 publication=no_message` means no outgoing message, not a zero
command. `previous` links action/reason changes; a DO_NOTHING nonzero command
after STOP is downstream release, not proof the vehicle moved.

`event=diagnostics_ready schema=1` is emitted once per configure at WARN so a
WARN-configured production launcher can prove diagnostic capability. It is not
an output command or a new business readiness gate. Other diagnostic events
are change-driven; identical continuing failures are at most 1 Hz. Existing
native upstream warnings are unchanged.

## Build identity and isolation

Only build this package (core **and** executable, with matching headers).
The patch changes private CollisionMonitor/Scan layout: replacing just the
core library while retaining an old entry executable is unsupported. All
other consumers of those headers must be built consistently before any later
deployment. No system dependency install or replacement is required.

The recorded 2026-09-20 candidate reused installed 1.1.19 headers and the fixed
official 1.1.19 package sources. Cached node/Scan sources were byte-identical
to that tag. CMake build used `-j1`, `nice -n 10`, CPU0-4, Release, and an
isolated `/tmp/njrh_reports/...` prefix; no API or complete Nav2 build.

The executable's test build must have an inherited RPATH to its own build
directory and `/opt/ros/humble/lib` (`-Wl,--disable-new-dtags`). The test removes
the child-only inherited `LD_LIBRARY_PATH` and asserts `/proc/PID/maps` contains
the exact candidate core library. This avoids accidentally exercising the
system core because the ROS setup path shadows the ordinary CMake RUNPATH.
It does not inject or change any ROS/DDS version.

Example **inside NJRH-car**, after a completed candidate build:

```bash
source /opt/ros/humble/setup.bash
unshare --net --ipc --mount bash
mount --make-rprivate /
mount -t tmpfs tmpfs /dev/shm
touch /dev/shm/.navlite_fixture_only
ip link set lo up
export NAVLITE_ISOLATED=1 ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 -B /path/to/test_navlite_collision_integration.py \
  --binary /tmp/njrh_reports/CANDIDATE/build/collision_monitor \
  --expected-library /tmp/njrh_reports/CANDIDATE/build/libcollision_monitor_core.so \
  --output /tmp/njrh_reports/CANDIDATE/new_fixture_result \
  --expect-navlite \
  --recorder /path/to/nav_event_lite.py
```

Never use the production ROS namespace/domain as a substitute for isolation.
The output directory must not already exist. Capture child processes only read
fixture logs; the fixture itself uses synthetic ROS data only in its private
network. The original production executable can be a baseline without
`--expected-library`, `--expect-navlite` or `--recorder`.

## Coverage and limits

Fourteen I/O cases cover no-scan/pass, signed x/y/yaw, SLOWDOWN, STOP,
stop-publication timeout, cleared-obstacle release, stale Scan, absent TF,
Scan/TF recovery, explicit zero, explicit-zero timeout, resumed commands,
NaN input and inactive output. Tests inspect actual received Twist samples,
not only diagnostic text, and preserve the native output contract.

The recorder must capture every real emitted NAVLITE line from the fixture's
`resident_navigation_runtime.log`. Each captured event must also pass the
current recorder's offline parser with no empty field or parse issue; inactive
must be classified as `no_message`. A missing or unparseable event fails
validation. Empty source lists are emitted as `sources=none`, never a bare
`sources=` token.

Not yet covered here: moving-footprint APPROACH integration, PointCloud/Range
source reason detail, real sensor timing/geometry, actual robot motion,
deployment file routing, and failure onset between log transitions. The test
does not certify the physical cause of any historical navigation stop.
