#!/usr/bin/env bash
set -u

echo "===== OS ====="
cat /etc/os-release | grep -E 'PRETTY_NAME|VERSION=' || true

echo
echo "===== ARCH ====="
uname -m
dpkg --print-architecture 2>/dev/null || true

echo
echo "===== ROS ====="
echo "ROS_DISTRO=${ROS_DISTRO:-<not set>}"
command -v ros2 || true

echo
echo "===== UART ====="
ls -l /dev/ttyAMA* /dev/serial* 2>/dev/null || true

echo
echo "===== USB/ACM ====="
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

echo
echo "===== USB DEVICES ====="
lsusb || true

echo
echo "===== SERIAL DEVICE DETAILS ====="
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$dev" ] || continue
    echo "--- $dev ---"
    udevadm info --query=property --name="$dev" 2>/dev/null \
      | grep -E '^(ID_VENDOR=|ID_MODEL=|ID_SERIAL=|ID_VENDOR_ID=|ID_MODEL_ID=)' || true
done

echo
echo "===== USER GROUPS ====="
id

echo
echo "===== POSSIBLE SERIAL USERS ====="
ps -ef | grep -E 'app.py|base_ctrl|ttyAMA0|ttyUSB0|ttyACM' | grep -v grep || true
