# robot_eai_gs2

ROS 2 Humble wrapper for the EAI/YDLIDAR GS2 near-field lidar. This package is intended for the docking stack only.

It vendors the official `EaiSdkForGS2` BSD source subset and publishes:

- `/dock/gs2_scan` (`sensor_msgs/msg/LaserScan`)
- `/dock/gs2_points` (`sensor_msgs/msg/PointCloud`)

Default frame:

```text
gs2_link
```

This package deliberately does not publish static TF. Add `base_link -> gs2_link` once the physical docking mount is fixed, preferably through `robot_description`.

## Build

```bash
colcon build --packages-select robot_eai_gs2
source install/setup.bash
```

## Run

```bash
ros2 launch robot_eai_gs2 gs2.launch.py serial_port:=/dev/ttyUSB0
```

For production, create a stable udev alias such as `/dev/gs2` and use:

```bash
ros2 launch robot_eai_gs2 gs2.launch.py serial_port:=/dev/gs2
```

On the current Jetson, the CP2102 GS2 adapter appears as the Silicon Labs device and should be aliased to `/dev/gs2`:

```bash
sudo bash scripts/jetson/install_gs2_udev.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Notes

- The GS2 SDK reports ranges in millimeters; this wrapper converts to meters with `sdk_range_scale: 0.001`.
- Defaults are tuned for final docking alignment: `0.025m .. 0.30m`, `-50deg .. 50deg`.
- If the scan fan is vertical instead of horizontal, rotate the sensor physically; do not compensate by changing the robot's main TF tree.
