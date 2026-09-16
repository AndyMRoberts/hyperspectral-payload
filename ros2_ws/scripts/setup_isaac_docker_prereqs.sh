#!/usr/bin/env bash
# One-time host setup for Isaac ROS Dev Docker (requires sudo).
# Run: bash ros2_ws/scripts/setup_isaac_docker_prereqs.sh
set -euo pipefail

echo "==> Adding $USER to docker group"
sudo usermod -aG docker "$USER"

echo "==> Installing git-lfs"
sudo apt-get update -qq
sudo apt-get install -y git-lfs
git lfs install

DOCKER_DATA_ROOT="/mnt/data/docker"
CONTAINERD_ROOT="/mnt/data/containerd"
echo "==> Preparing Docker/containerd roots on NVMe"
sudo mkdir -p "$DOCKER_DATA_ROOT" "$CONTAINERD_ROOT"

DAEMON_JSON="/etc/docker/daemon.json"
echo "==> Writing $DAEMON_JSON (nvidia runtime + data-root on /mnt/data)"
sudo tee "$DAEMON_JSON" >/dev/null <<EOF
{
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    },
    "data-root": "$DOCKER_DATA_ROOT"
}
EOF

CONTAINERD_TOML="/etc/containerd/config.toml"
echo "==> Pointing containerd root at $CONTAINERD_ROOT (image layers live here)"
# Keep disabled_plugins; set root/state explicitly so pulls do not fill eMMC.
sudo tee "$CONTAINERD_TOML" >/dev/null <<EOF
disabled_plugins = ["cri"]

root = "$CONTAINERD_ROOT"
state = "/run/containerd"
EOF

echo "==> Stopping docker/containerd and clearing leftover eMMC store (frees ~20G if full)"
sudo systemctl stop docker containerd || true
if [[ -d /var/lib/containerd ]] && [[ "$(sudo du -s /var/lib/containerd 2>/dev/null | awk '{print $1}')" -gt 1000 ]]; then
  echo "    Removing /var/lib/containerd contents (incomplete pulls / old layers)"
  sudo rm -rf /var/lib/containerd/*
fi

echo "==> Generating NVIDIA CDI specs (optional; run_dev uses classic GPU env on Jetson)"
sudo mkdir -p /etc/cdi
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml || \
  echo "WARNING: nvidia-ctk cdi generate failed; continuing with classic NVIDIA_VISIBLE_DEVICES=all"

echo "==> Starting containerd + docker"
sudo systemctl start containerd
sudo systemctl start docker
sleep 2
sudo docker info | grep -E 'Docker Root Dir|Server Version' || true
echo "containerd root: $(grep '^root' "$CONTAINERD_TOML")"
df -h / /mnt/data | sed 's/^/    /'

echo
echo "Done. Log out and back in (or run: newgrp docker) so the docker group takes effect."
echo "Then verify: docker ps && df -h /"
echo "Next: cd ros2_ws && ./scripts/run_isaac_dev.sh"
echo "Inside the container: ./scripts/build_isaac_argus.sh"
echo "Optional camera debug: sudo systemctl restart nvargus-daemon.service"
