# ROS 2 Jazzy on the existing Waveshare Raspberry Pi OS

## Confirmed host platform

The competition computer stays on the existing Waveshare-compatible system:

- Raspberry Pi 4, 4 GB RAM;
- Debian GNU/Linux 12 (Bookworm), arm64/aarch64;
- Python 3.11.2;
- Waveshare project: `~/ugv_rpi`;
- Waveshare virtual environment: `~/ugv_rpi/ugv-env` (Python 3.11.2);
- ROS 2 is not installed natively on the host;
- Docker was initially not installed.

The OS is intentionally **not replaced**, because the vendor Waveshare stack is required on this robot.

## ROS decision

Run **ROS 2 Jazzy inside an Ubuntu 24.04 arm64 Docker container**, while keeping Debian 12 and `ugv_rpi` unchanged on the host.

Reasons:

- official Jazzy binary packages target Ubuntu 24.04 arm64;
- the official `ros:jazzy-ros-base-noble` image is available for linux/arm64;
- Docker Engine supports Debian 12 arm64;
- this isolates ROS dependencies from the vendor Python environment;
- the robot can keep the known-working Waveshare host configuration.

RViz should normally run on a development laptop, not on the Raspberry Pi 4.

## Host hardware paths already confirmed

- base controller UART: `/dev/ttyAMA0` (also `/dev/serial0` on this image), 115200;
- STL-19P: `/dev/ttyUSB0`, 230400;
- project lidar alias: `/dev/rower_lidar -> ttyUSB0`;
- `T=1001` base feedback works;
- `T=1` and `T=13` motor control work;
- wheel encoders/odometry work;
- battery feedback works;
- built-in IMU fields remain zero and are not used in the first navigation stack.

The stock `ugv-app.service` remains disabled during autonomous development because its `app.py` opens the first `/dev/ttyUSB*` and conflicts with the STL-19P.

## Files in this repository

```text
docker/Dockerfile
scripts/setup_docker_bookworm.sh
scripts/build_ros2_docker.sh
scripts/run_ros2_docker.sh
```

The Docker image contains ROS 2 Jazzy ros-base plus:

- Nav2;
- SLAM Toolbox;
- robot_state_publisher;
- xacro;
- TF2 tools;
- teleop_twist_keyboard;
- CycloneDDS RMW;
- pyserial and colcon tools.

## Install Docker on the Raspberry Pi

```bash
cd ~/rower_gorod
git pull
chmod +x scripts/setup_docker_bookworm.sh
./scripts/setup_docker_bookworm.sh
```

The installer is guarded and only accepts Debian 12 Bookworm arm64.

After it finishes, log out and back in, or reboot:

```bash
sudo reboot
```

Then verify:

```bash
docker --version
docker run --rm hello-world
```

## Build the ROS 2 image

```bash
cd ~/rower_gorod
chmod +x scripts/build_ros2_docker.sh scripts/run_ros2_docker.sh
./scripts/build_ros2_docker.sh
```

The image name is:

```text
rower-ros2:jazzy
```

## Start an interactive ROS 2 shell

Make sure the stock Waveshare app is still stopped:

```bash
systemctl --user is-enabled ugv-app.service || true
pgrep -af 'ugv_rpi/app.py' || true
sudo fuser -v /dev/ttyUSB0 || true
```

Then:

```bash
cd ~/rower_gorod
./scripts/run_ros2_docker.sh
```

The launcher uses host networking for ROS DDS and passes:

```text
host /dev/ttyAMA0  -> container /dev/ttyAMA0
host /dev/ttyUSB0  -> container /dev/rower_lidar
```

It also bind-mounts the repository at:

```text
/workspace/rower_gorod
```

Default ROS settings:

```text
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Verify inside the container

```bash
echo "$ROS_DISTRO"
echo "$RMW_IMPLEMENTATION"
ros2 --help >/dev/null && echo ROS2_OK
ros2 pkg list | grep -E '^(nav2_bringup|slam_toolbox)$'
ls -l /dev/ttyAMA0 /dev/rower_lidar
```

The existing safe Python hardware probes can also be run inside the container:

```bash
python3 scripts/stl19p_probe.py --port /dev/rower_lidar --frames 20
python3 scripts/ugv_base_probe.py --port /dev/ttyAMA0 --seconds 3
```

## Planned ROS graph

```text
STL-19P
  -> rower_lidar
  -> /scan

ESP32 UGV02
  -> T=1001
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
/scan + /odom + TF -> map

Nav2:
map + /scan + /odom + TF -> /cmd_vel
```

The first stack does not depend on the built-in IMU.

## Next implementation step

After the container passes both hardware probes, create the ROS 2 packages in this repository:

1. `rower_base_bridge`;
2. `rower_lidar`;
3. `rower_description`;
4. `rower_bringup`;
5. SLAM Toolbox configuration;
6. Nav2 configuration;
7. sign detector and mission manager.
