#!/usr/bin/env bash
set -euo pipefail

# Guarded installer for the competition Raspberry Pi image.
# This script is intentionally restricted to Ubuntu 24.04 (Noble) arm64.
# Do NOT run it on the current Waveshare Debian 12 image.

if [[ ! -r /etc/os-release ]]; then
  echo "ERROR: /etc/os-release not found" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
ARCH="$(dpkg --print-architecture)"

if [[ "${ID:-}" != "ubuntu" || "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "ERROR: this installer requires Ubuntu 24.04 (Noble)." >&2
  echo "Detected: ID=${ID:-unknown} VERSION_CODENAME=${VERSION_CODENAME:-unknown}" >&2
  echo "Keep the existing Debian 12 Waveshare image as a fallback; use a separate microSD/SSD for Ubuntu." >&2
  exit 2
fi

if [[ "$ARCH" != "arm64" ]]; then
  echo "ERROR: expected arm64 on Raspberry Pi 5, detected: $ARCH" >&2
  exit 2
fi

echo "Detected supported platform: Ubuntu ${VERSION_ID:-24.04} (${VERSION_CODENAME}) ${ARCH}"

sudo apt update
sudo apt install -y \
  curl \
  git \
  locales \
  software-properties-common \
  python3-serial

sudo add-apt-repository -y universe
sudo apt update

# ROS now recommends ros2-apt-source for repository/key management.
ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' \
  | head -n1 \
  | awk -F'"' '{print $4}')"

if [[ -z "$ROS_APT_SOURCE_VERSION" ]]; then
  echo "ERROR: could not determine ros2-apt-source release version." >&2
  exit 3
fi

curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-teleop-twist-keyboard

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

if ! grep -q '^source /opt/ros/jazzy/setup.bash$' "$HOME/.bashrc" 2>/dev/null; then
  printf '\n# ROS 2 Jazzy\nsource /opt/ros/jazzy/setup.bash\n' >> "$HOME/.bashrc"
fi

# Make sure the competition user can access UART/USB serial devices after next login.
sudo usermod -aG dialout "$USER"

# Install the project lidar udev rule when this script is run from the repository.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
if [[ -x "$REPO_DIR/scripts/install_udev_rules.sh" ]]; then
  "$REPO_DIR/scripts/install_udev_rules.sh"
elif [[ -f "$REPO_DIR/udev/99-rower-lidar.rules" ]]; then
  sudo install -m 0644 "$REPO_DIR/udev/99-rower-lidar.rules" /etc/udev/rules.d/99-rower-lidar.rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger
fi

echo
echo "ROS 2 Jazzy installation finished."
echo "Log out/in (or reboot) once so dialout membership is refreshed."
echo "Then verify with:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  ros2 --help >/dev/null && echo ROS2_OK"
echo "  ros2 pkg list | grep -E '^(nav2_bringup|slam_toolbox)$'"
echo "  ls -l /dev/serial0 /dev/rower_lidar 2>/dev/null || true"
