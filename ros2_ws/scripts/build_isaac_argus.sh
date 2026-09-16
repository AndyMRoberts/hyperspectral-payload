#!/usr/bin/env bash
# Install/run prep for isaac_ros_argus_camera INSIDE the Isaac Dev container.
# Do not run on the host.
set -euo pipefail

ISAAC_ROS_WS="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
cd "$ISAAC_ROS_WS"

if [[ ! -d /workspaces/isaac_ros-dev ]]; then
  echo "ERROR: This script must run inside the Isaac Dev container." >&2
  echo "  Host:  cd ros2_ws && ./scripts/run_isaac_dev.sh" >&2
  echo "  Then:  ./scripts/build_isaac_argus.sh" >&2
  exit 1
fi

echo "==> Installing prebuilt isaac_ros_argus_camera (apt)"
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-argus-camera

# Avoid host/container colcon path clashes and apt vs source conflicts.
touch "$ISAAC_ROS_WS/src/isaac_ros_common/COLCON_IGNORE" \
      "$ISAAC_ROS_WS/src/isaac_ros_argus_camera/COLCON_IGNORE"

# Do NOT colcon-build against the host's build/ + install/ here.
# Those embed absolute paths like /home/hyper/hyperspectral_camera/ros2_ws/install/...
# which do not exist inside the container (/workspaces/isaac_ros-dev/...).

ARGUS_LAUNCH="$ISAAC_ROS_WS/src/bringup/hsi_bringup/launch/argus_stereo.launch.py"
if [[ ! -f "$ARGUS_LAUNCH" ]]; then
  echo "ERROR: missing $ARGUS_LAUNCH" >&2
  exit 1
fi

echo
echo "Done. Do NOT 'source install/setup.bash' in the container (host path pollution)."
echo "Use the apt overlay + launch file path:"
echo
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 pkg prefix isaac_ros_argus_camera"
echo "  ros2 launch ${ARGUS_LAUNCH}"
echo
echo "Optional args: module_id:=-1 mode:=0"
