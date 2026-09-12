# ROS 2 Jazzy setup for the competition Raspberry Pi 5

## Decision

Use a separate microSD card or SSD with **Ubuntu Server 24.04 LTS 64-bit (arm64)** for the competition system. Keep the current Waveshare Debian 12 image unchanged as a fallback.

ROS 2 Jazzy officially supports Ubuntu 24.04 (Noble) on 64-bit ARM. The project therefore does not install Jazzy directly into the existing Debian 12 Waveshare image.

The Raspberry Pi image should be headless. RViz can be run later on a development PC if desired; the robot itself only needs ROS base, Nav2, SLAM Toolbox and project nodes.

## Before changing media

The real robot hardware has already been validated on the existing Debian installation:

- base UART: `/dev/serial0 -> /dev/ttyAMA0`, 115200 baud;
- STL-19P: CP2102, 230400 baud;
- stable lidar alias: `/dev/rower_lidar`;
- `T=1001` base feedback works;
- `T=1` and `T=13` motor control both work;
- wheel encoders/odometry work;
- battery voltage feedback works;
- built-in IMU fields currently remain zero and are not required for the first navigation stack.

See `docs/HARDWARE_VALIDATION.md` for measured results.

## Install Ubuntu

Use Raspberry Pi Imager and install Ubuntu Server 24.04 LTS 64-bit onto a separate microSD/SSD. Configure SSH and network access during imaging if convenient.

After first boot verify:

```bash
cat /etc/os-release
uname -m
dpkg --print-architecture
```

Expected:

```text
Ubuntu 24.04 / noble
aarch64
arm64
```

## Clone the project

```bash
cd ~
git clone https://github.com/xander-07/rower_gorod.git
cd ~/rower_gorod
```

Do not copy the old global Git setting `http.sslVerify=false` to the new system. A clean Ubuntu installation should have a working CA bundle.

## Install ROS 2 Jazzy and navigation dependencies

The repository contains a guarded installer which refuses to run unless the OS is Ubuntu 24.04 Noble arm64:

```bash
cd ~/rower_gorod
chmod +x scripts/setup_ros2_jazzy.sh
./scripts/setup_ros2_jazzy.sh
```

It installs:

- ROS 2 Jazzy `ros-base`;
- ROS development tools and colcon;
- Navigation2 and `nav2_bringup`;
- SLAM Toolbox;
- robot_state_publisher and xacro;
- TF2 tools;
- teleop_twist_keyboard;
- pyserial;
- project udev rule for `/dev/rower_lidar`.

After installation, reboot once:

```bash
sudo reboot
```

## Post-install verification

```bash
source /opt/ros/jazzy/setup.bash

echo "ROS_DISTRO=$ROS_DISTRO"
ros2 --help >/dev/null && echo ROS2_OK
ros2 pkg list | grep -E '^(nav2_bringup|slam_toolbox)$'

ls -l /dev/serial0 /dev/rower_lidar
id
```

Then re-run the safe hardware probes from this repository:

```bash
python3 scripts/stl19p_probe.py --port /dev/rower_lidar --frames 20
python3 scripts/ugv_base_probe.py --port /dev/serial0 --seconds 3
```

These two tests must pass before creating/starting the ROS driver nodes.

## Planned ROS graph

First working navigation stack:

```text
STL-19P
  -> rower_lidar
  -> /scan

ESP32 UGV02
  -> T=1001 feedback
  -> rower_base_bridge
  -> /odom
  -> /battery

/cmd_vel
  -> rower_base_bridge
  -> T=1 L/R @ 115200
  -> ESP32

TF:
map -> odom -> base_link -> laser

SLAM Toolbox:
/scan + TF + /odom -> map

Nav2:
map + /scan + TF + /odom -> /cmd_vel
```

The first version will not depend on the built-in IMU because the real robot test produced zero gyro/accelerometer/magnetometer fields even while the chassis was moved by hand.

## Next implementation step

Once Ubuntu/Jazzy is verified on the real Raspberry Pi, create the actual ROS 2 packages in this repository:

1. `rower_base_bridge` — `/cmd_vel`, `/odom`, battery and serial watchdog;
2. `rower_lidar` — STL-19P serial parser publishing `sensor_msgs/LaserScan` on `/scan`;
3. `rower_description` — URDF/xacro and static lidar transform;
4. `rower_bringup` — launch and configuration;
5. SLAM Toolbox config;
6. Nav2 config;
7. later: sign detector and mission manager for the hackathon logic.
