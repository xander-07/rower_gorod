#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RULE_SRC="$REPO_ROOT/udev/99-rower-lidar.rules"
RULE_DST="/etc/udev/rules.d/99-rower-lidar.rules"

echo "Installing $RULE_SRC -> $RULE_DST"
sudo install -m 0644 "$RULE_SRC" "$RULE_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty

echo
echo "Current lidar device links:"
ls -l /dev/rower_lidar /dev/serial/by-id/*CP2102* 2>/dev/null || true

echo
echo "If /dev/rower_lidar is not present, unplug/replug the STL-19P USB-UART adapter once."
