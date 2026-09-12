# SLAM Toolbox setup

The physical robot runs ROS 2 Jazzy inside the project Docker image on Debian 12. The mapping stack uses:

```text
/scan -> slam_toolbox
/odom + odom->base_link -> slam_toolbox
base_link->laser -> robot_state_publisher
slam_toolbox -> /map + map->odom

/map + /scan + /odom + /tf
        -> rosbridge WebSocket :9090
        -> local dashboard served by the robot on HTTP :8080
        -> Windows browser
```

The browser dashboard is fully local. It does not depend on Foxglove, a cloud service, CDN, or Internet access. The Windows computer only needs to be able to reach the Raspberry Pi over the local network.

## Rebuild Docker image after web-stack changes

The project ROS image includes `ros-jazzy-rosbridge-server`. After pulling a Dockerfile change, rebuild the image from the Debian host:

```bash
cd ~/rower_gorod
git pull
./scripts/build_ros2_docker.sh
```

## Build workspace package

```bash
cd ~/rower_gorod

./scripts/run_ros2_docker.sh \
  colcon build \
  --symlink-install \
  --packages-select rower_navigation
```

The `rower_navigation` package installs the dashboard from `src/rower_navigation/web/` together with its launch/config files.

## Safe startup check

Start the complete robot + SLAM stack with physical motion disabled:

```bash
cd ~/rower_gorod

./scripts/run_ros2_docker.sh \
  ros2 launch rower_navigation mapping.launch.py \
  enable_motion:=false
```

`mapping.launch.py` starts by default:

```text
robot bringup
slam_toolbox
rosbridge_websocket on 0.0.0.0:9090
local HTTP server on 0.0.0.0:8080
```

Expected TF graph:

```text
map -> odom -> base_link -> laser
```

Useful checks from another SSH terminal:

```bash
cd ~/rower_gorod
./scripts/run_ros2_docker.sh ros2 lifecycle get /slam_toolbox
```

```bash
cd ~/rower_gorod
./scripts/run_ros2_docker.sh ros2 topic hz /scan
```

```bash
cd ~/rower_gorod
./scripts/run_ros2_docker.sh ros2 topic hz /odom
```

```bash
cd ~/rower_gorod
./scripts/run_ros2_docker.sh ros2 topic hz /map
```

```bash
cd ~/rower_gorod
./scripts/run_ros2_docker.sh ros2 run tf2_ros tf2_echo map laser
```

```bash
cd ~/rower_gorod
./scripts/run_ros2_docker.sh ros2 node list | grep rosbridge
```

`/scan` should be close to 10 Hz, `/odom` close to 20 Hz, `/map` near 1 Hz, SLAM Toolbox should be `active`, and `tf2_echo map laser` should resolve through the complete TF chain.

## Windows browser: no external service

Find the robot address on the Raspberry Pi host:

```bash
hostname -I
```

Choose the LAN/Wi-Fi address reachable from the Windows computer, for example `192.168.1.50`.

Open this directly in Chrome or Edge:

```text
http://192.168.1.50:8080
```

No login and no Internet access are required.

The dashboard displays:

```text
live OccupancyGrid from /map
current LiDAR returns from /scan
robot pose using /odom + map->odom TF
battery voltage
ROS/WebSocket status
```

It also includes guarded manual control. Browser control starts disarmed after every page load. To use it, the operator must explicitly press `УПРАВЛЕНИЕ ЗАБЛОКИРОВАНО`, after which the button changes to `УПРАВЛЕНИЕ РАЗРЕШЕНО`.

Keyboard controls:

```text
W / Up       forward
S / Down     reverse
A / Left     in-place left turn
D / Right    in-place right turn
Space        stop
```

The browser sends commands at 10 Hz while a key/button is held and sends zero velocity on key release, page focus loss, visibility loss, or disarm. The base bridge watchdog remains the final stop layer if browser communication disappears.

The page intentionally does not combine linear and angular commands, so the first mapping pass uses only the already calibrated motion modes: straight segments and in-place turns.

The default dashboard speeds are:

```text
linear:  0.06 m/s
angular: 0.40 rad/s
```

They can be adjusted within conservative ranges in the page.

## Mapping with motion enabled

After the safe startup check and browser display both work, stop the previous launch and restart explicitly with drive motion enabled:

```bash
cd ~/rower_gorod

./scripts/run_ros2_docker.sh \
  ros2 launch rower_navigation mapping.launch.py \
  enable_motion:=true
```

Then open the same local page on Windows:

```text
http://ROBOT_IP:8080
```

Use a stop-turn-go pattern for the first map:

```text
straight -> stop -> in-place turn -> stop -> straight
```

If walls duplicate, rotate, or drift badly on the live map, stop immediately and diagnose before continuing.

The ROS launch still defaults to `enable_motion:=false`, so merely opening the dashboard cannot move the robot unless the operator explicitly restarted mapping with motion enabled and then armed browser control.

## Optional SSH teleop fallback

The browser dashboard is the preferred mapping control. If needed, keyboard teleop remains available from SSH:

```bash
cd ~/rower_gorod

./scripts/run_ros2_docker.sh \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p speed:=0.06 \
  -p turn:=0.40
```

## Disable the web dashboard

For a ROS-only run:

```bash
cd ~/rower_gorod

./scripts/run_ros2_docker.sh \
  ros2 launch rower_navigation mapping.launch.py \
  enable_motion:=false \
  enable_web:=false
```

Default ports can be changed with:

```text
web_port:=8081
rosbridge_port:=9091
```

Do not expose either port to the public Internet. They are intended only for the robot's local development network.

## Save a map

When the map is clean and the robot has revisited enough of the environment for loop closure, save it into the repository:

```bash
cd ~/rower_gorod

./scripts/run_ros2_docker.sh \
  ros2 run nav2_map_server map_saver_cli \
  -f /workspace/rower_gorod/maps/arena
```

This produces `maps/arena.yaml` and `maps/arena.pgm`. Commit both files after visually validating the saved map.
