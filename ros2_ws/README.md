# ROS 2 workspace


## Todo
1. stereo camera needs hardware sync adding, then recalibration
2. Eventually if other sensors are required, change from HSI API to custom implementation in order to be able to record everything in a rosbag to be able to have synced data between HSI and everything else. 





## Clone

For others cloning this repo: after a normal clone, initialize submodules with:

```bash
git submodule update --init --recursive
```

Or clone in one step:

```bash
git clone --recurse-submodules <your-repo-url>
```

### USB_GPS_EN submodule: required `gps.h` patches

The `gps_monitor_cpp` package vendors [DFRobotdl/USB_GPS_EN](https://github.com/DFRobotdl/USB_GPS_EN) as a git submodule. After initializing submodules, apply these manual edits to:

`src/sensors/gps_monitor_cpp/include/USB_GPS_EN/gps.h`

Inside `parse_GpsDATA()`, in the `switch (i)` block:

1. **Comment out line 62** — the `buffer` variable is unused and triggers a compiler warning:

```c
// char buffer[12];
```

2. **Add `break;` statements** so that every `memcpy` in the switch ends with `break;` (lines 72 and 76 in the patched file; upstream is also missing breaks after the case 3 and case 5 `memcpy` calls). The corrected switch body should look like:

```c
switch(i){
  case 1:RST_Buffer(Save_Data.UTCTime);
    memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break;
  case 2:RST_Buffer(usefullBuffer);
    memcpy(usefullBuffer, subString, subStringNext - subString);break;
  case 3:RST_Buffer(Save_Data.Slatitude);
    memcpy(Save_Data.Slatitude, subString, subStringNext - subString);break;
  case 4:RST_Buffer(Save_Data.N_S);
    memcpy(Save_Data.N_S, subString, subStringNext - subString);break;//获取 N/S
  case 5:RST_Buffer(Save_Data.Slongitude);
    memcpy(Save_Data.Slongitude, subString, subStringNext - subString);break;
  case 6:RST_Buffer(Save_Data.E_W);
    memcpy(Save_Data.E_W, subString, subStringNext - subString);break;//获取 E/W
  default:break;
}
```

Without these `break` statements, cases fall through and latitude/longitude parsing is corrupted.

### Serial port permissions

The GPS receiver connects over USB serial (default `/dev/ttyACM0`). Without access to that device, `gps_monitor_cpp` fails with `open serial port: Permission denied`.

Do **not** run the node with `sudo` — that breaks ROS networking. Add your user to the `dialout` group instead (requires admin/sudo once):

```bash
sudo usermod -aG dialout $USER
```

Log out and log back in (or reboot), then verify:

```bash
groups                    # should list dialout
ls -la /dev/ttyACM0
ros2 run gps_monitor_cpp gps_monitor
```

To test in the current shell without logging out:

```bash
newgrp dialout
ros2 run gps_monitor_cpp gps_monitor
```

If you do not have sudo access, ask a system administrator to run `sudo usermod -aG dialout <username>` for you.

## Build

From this directory:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## GPS monitor

Publishes `sensor_msgs/NavSatFix` on `/sensors/gps` from the DFRobot USB GPS receiver:

```bash
source install/setup.bash
ros2 run gps_monitor_cpp gps_monitor
```

Override the serial port if needed:

```bash
ros2 run gps_monitor_cpp gps_monitor --ros-args -p serial_port:=/dev/ttyACM0
```

To print raw parsed GPS data (via `printf`) and publish `NavSatFix` even without a satellite fix:

```bash
ros2 run gps_monitor_cpp gps_monitor --ros-args -p publish_invalid:=true
```

See [Serial port permissions](#serial-port-permissions) if the node cannot open the device.

## Bringup: HSI cameras and binned preview

Launches the VIS and NIR `hsi_camera_driver_cpp` nodes and the `hsi_binned_preview_cpp` preview node:

```bash
source install/setup.bash
ros2 launch hsi_bringup hsi_cameras_with_binned_preview.launch.py
```

or for multicam
```bash
source install/setup.bash
ros2 launch hsi_bringup hsi_cameras_multicam_binned_preview.launch.py
```


# Waveshare IMX219-83 Stereo Camera and IMU

Required to run this on Jetson Orin Nano: 

`sudo /opt/nvidia/jetson-io/jetson-io.py`

and configure for "Configure Jetson 24pin CSI Connector" >> "Configure for IMX219 Dual"

## Isaac ROS Argus stereo (recommended)

Stereo capture is provided by [`isaac_ros_argus_camera`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera) (Isaac ROS **release-3.2**, matching JetPack 6.x / Humble) inside the Isaac Dev Docker container. Host bringup keeps HSI/GPS/etc. on the host and expects Argus to publish:

- `/sensors/stereo/left`
- `/sensors/stereo/right`

(`left/image_raw` / `right/image_raw` remapped by `argus_stereo.launch.py`)

### One-time host prerequisites (sudo)

Rootfs is small; Docker **and containerd** data must live on `/mnt/data` (Isaac pulls otherwise fill `/var/lib/containerd` on eMMC):

```bash
bash ros2_ws/scripts/setup_isaac_docker_prereqs.sh
```

Then **log out and back in** (or `newgrp docker`) and verify:

```bash
docker ps
git lfs version
df -h /
```

If the container fails with `unresolvable CDI devices nvidia.com/gpu=all`, `run_dev.sh` is patched to use `NVIDIA_VISIBLE_DEVICES=all`. Optional CDI generate:

```bash
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
```

Repos are already cloned under `ros2_ws/src/isaac_ros_common` and `ros2_ws/src/isaac_ros_argus_camera` (branch `release-3.2`). Host `colcon build` skips them via `COLCON_IGNORE` in those trees.

### Two-terminal workflow

**Terminal A — Isaac Docker (Argus):**

```bash
cd /home/hyper/hyperspectral_camera/ros2_ws
./scripts/run_isaac_dev.sh
# inside container only:
./scripts/build_isaac_argus.sh
source /opt/ros/humble/setup.bash
ros2 launch /workspaces/isaac_ros-dev/src/bringup/hsi_bringup/launch/argus_stereo.launch.py
```

Notes:
- `build_isaac_argus.sh` installs apt package `ros-humble-isaac-ros-argus-camera`.
- Do **not** `source install/setup.bash` inside the container — that tree was built on the host with paths under `/home/hyper/...`, which break under `/workspaces/...`.
- Launch the remapped file by **absolute path** (no need to colcon-build `hsi_bringup` in Docker).

**Terminal B — host (rest of system):**

```bash
cd /home/hyper/hyperspectral_camera/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hsi_bringup full_system.launch.py run_name:=test
# or stereo-focused:
ros2 launch hsi_bringup stereo_only.launch.py run_name:=test
```

By default `use_legacy_stereo:=false` so the host does **not** start the OpenCV `stereo_camera` node.

### Legacy OpenCV stereo fallback

```bash
ros2 launch hsi_bringup full_system.launch.py use_legacy_stereo:=true
```

### Camera daemon

If Argus fails to open the sensor:

```bash
sudo systemctl restart nvargus-daemon.service
```

---

## Legacy OpenCV stereo notes

Demo code can be downloaded to ros2_ws/src/sensors/stereo_camera

```
wget https://files.waveshare.com/upload/e/eb/D219-9dof.tar.gz
tar zxvf D219-9dof.tar.gz
```

Other support here: https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera?srsltid=AfmBOorWuFfQRE-_HYuJ68v_UwhnOcubG3c15le9vecCWJEt1vMwFapg

For IMU use,respectively connect the SDA and SCL pins of the camera to pins 3 and 5
on the Jetson Nano.

To fix image looking overly red (from link):
```
wget https://files.waveshare.com/upload/e/eb/Camera_overrides.tar.gz
tar zxvf Camera_overrides.tar.gz 
sudo cp camera_overrides.isp /var/nvidia/nvcam/settings/
sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
```



# Spectrometer

Installs
```
sudo apt-get update
sudo apt-get install python3-pip i2c-tools libgpiod-dev
pip3 install Adafruit-Blinka adafruit-circuitpython-as7341
sudo usermod -aG i2c $USER
```

For testing:
```
export JETSON_MODEL_NAME=JETSON_ORIN_NANO
python3 spectrometer_test.py
```



# IMU

On pins 3 and 5 IMU shows up as addres 0x68 on i2c_7
`sudo i2cdetect -y -r 7`



## IMEC mosaic Python API — `LoadOpticalSetup` ctypes fix

`post_run_data_processing.ipynb` (and any code that calls `HSI_MOSAIC.LoadOpticalSetup()`) can fail with:

```text
ArgumentError: argument 1: TypeError: wrong type
```

This is **not** a missing file or UTF-8 encoding problem. The C API takes `char const*`, but the Python wrapper declares `ctypes.c_wchar_p` (expects a `str`) while passing `filename.encode('utf-8')` (`bytes`). ctypes rejects the argument before the DLL reads the XML.

**Fix** in `/opt/imec/hsi-mosaic/python_apis/hsi_mosaic/hsi_mosaic_api.py` (around the `mosaicLoadOpticalSetup` binding, line 487):

```python
# Before (broken):
__api_dll.mosaicLoadOpticalSetup.argtypes = [ctypes.c_wchar_p, ctypes.POINTER(OpticalSetup)]

# After (matches LoadContext and char const*):
__api_dll.mosaicLoadOpticalSetup.argtypes = [ctypes.c_char_p, ctypes.POINTER(OpticalSetup)]
```

Leave the call unchanged:

```python
result = __api_dll.mosaicLoadOpticalSetup(filename.encode('utf-8'), ctypes.byref(optical_setup))
```

Restart the Jupyter kernel (or re-import `hsi_mosaic`) after editing. This patch is overwritten if the IME SDK is reinstalled or upgraded.

