#!/usr/bin/env bash
# Enter the Isaac ROS Dev container with this workspace mounted.
# Prerequisites: run scripts/setup_isaac_docker_prereqs.sh once (sudo), then re-login.
set -euo pipefail

export ISAAC_ROS_WS="${ISAAC_ROS_WS:-/home/hyper/hyperspectral_camera/ros2_ws}"
RUN_DEV="${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/run_dev.sh"

if [[ ! -x "$RUN_DEV" ]]; then
  echo "Missing $RUN_DEV — clone isaac_ros_common (release-3.2) into src/ first." >&2
  exit 1
fi

exec "$RUN_DEV" -d "$ISAAC_ROS_WS" "$@"
