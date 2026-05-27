# GS2 driver integration report

## Reuse scan result

- Current repository: no existing GS2/YDLIDAR package was found.
- Local `D:\codespace\car`: no GS2/YDLIDAR package was found.
- Official ROS 1 package found: `YDLIDAR/EaiRosForGS2`.
- Official SDK found: `YDLIDAR/EaiSdkForGS2`.

## Decision

The generic ROS 2 YDLIDAR driver was not selected as the project-owned docking driver because GS2 has a dedicated official SDK and the available GS2 ROS package is ROS 1. The project now vendors the official GS2 SDK source subset and provides a ROS 2 C++ wrapper package.

## Added package

```text
src/robot_eai_gs2
```

Outputs:

```text
/dock/gs2_scan   sensor_msgs/msg/LaserScan   frame_id=gs2_link
/dock/gs2_points sensor_msgs/msg/PointCloud  frame_id=gs2_link
```

## TF policy

The driver does not publish TF. The GS2 mount is single-sourced in `robot_description` as `base_link -> gs2_link`.

Current mount values:

```text
parent: base_link
child: gs2_link
xyz: [0.36, 0.0, 0.255]
rpy: [0.0, 0.0, 0.0]
```

The `x=0.36m` value follows the current Nav2 footprint front edge for a flush front-center mount. Re-measure and tune only `gs2_x` if the final body front plane differs from the current footprint.

## Hardware validation still required

- Confirm serial baudrate: default is `921600`, override to `230400` if the live module requires it.
- Confirm scan fan orientation by moving a board left/right in front of the sensor.
- Confirm range scale using a ruler at 5cm, 10cm, and 20cm.
- Confirm whether the charger face is visible in `/dock/gs2_scan` at the intended mount height.
