# hsi_camera_driver_cpp

ROS 2 Humble C++ node for HSI camera acquisition using the imec Camera API.

## Build

```bash
cd /home/hyper/hyperspectral_camera/ros2_ws
colcon build --packages-select hsi_camera_driver_cpp --symlink-install
source install/setup.bash
```

## Run

### Basic run

```bash
ros2 run hsi_camera_driver_cpp hsi_camera_node
```

### Run with parameters (free-run)

```bash
ros2 run hsi_camera_driver_cpp hsi_camera_node \
  --ros-args \
  -p free_run:=true \
  -p timelapse_hz:=1.0 \
  -p rate_hz:=20.0 \
  -p device_name:=nir \
  -p exposure_time_ms:=20.0 \
  -p gain:=1.0 \
  -p test_frame_count:=-1 \
  -p run_name_path:=/mnt/data/timelapses/20260101_120000_test_commands
```

### Run with parameters (fixed-rate software trigger)

```bash
ros2 run hsi_camera_driver_cpp hsi_camera_node \
  --ros-args \
  -p free_run:=false \
  -p rate_hz:=5.0 \
  -p device_name:=nir \
  -p exposure_time_ms:=10.0 \
  -p gain:=1.0 \
  -p test_frame_count:=100

```

## Parameters

- `free_run` (`bool`, default: `true`): free-running acquisition mode.
- `rate_hz` (`double`, default: `20.0`): loop rate used when `free_run:=false`.
- `device_name` (`string`, default: `""`): camera selector (`vis`, `nir`, or empty for first detected camera).
- `exposure_time_ms` (`double`, default: `10.0`): camera exposure time.
- `gain` (`double`, default: `1.0`): camera gain.
- `test_frame_count` (`int`, default: `-1`): if non-negative, stops after that many frames and prints elapsed time/FPS.
- `camera_start_delay_ms` (`int`, default: `0`): if positive, sleeps that many milliseconds immediately before `cameraStart()` (useful to stagger two nodes on the same USB bus).

## Launch both cameras

```bash
ros2 launch hsi_camera_driver_cpp hsi_dual_camera.launch.py
```

This launch file starts two node instances:
- VIS camera node with `device_name:=vis` and `camera_start_delay_ms:=0`
- NIR camera node with `device_name:=nir` and `camera_start_delay_ms:=2500` (starts streaming slightly later to reduce USB contention)

## Published Topics

- `/hsi/<device_name>/raw_image` (`sensor_msgs/msg/Image`)
- `/hsi/<device_name>/raw_metadata` (`std_msgs/msg/String`)
- `/hsi/<device_name>/temp` (`std_msgs/msg/Float32`)

## Notes

- The node links against imec SDK libraries in `/opt/imec/hsi-mosaic/bin`.
- If your shell cannot find those shared libraries at runtime, set:

```bash
export LD_LIBRARY_PATH=/opt/imec/hsi-mosaic/bin:$LD_LIBRARY_PATH
```

- once publishing enabled the rate is throttled to about 25Hz