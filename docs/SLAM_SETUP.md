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

## Build workspace packages

After pulling code changes, rebuild the packages that contain the base bridge, bringup and mapping dashboard:

```bash
cd ~/rower_gorod
git pull

./scripts/run_ros2_docker.sh \
  colcon build \
  --symlink-install \
  --packages-select rower_base_bridge rower_bringup rower_navigation
```

A Docker image rebuild is only required when `docker/Dockerfile` changes. The current motion-smoothing and dashboard changes do not require rebuilding the image.

## Smooth base motion

`rower_base_bridge` rate-limits normal left/right wheel commands before sending Waveshare `T=1` packets. Current defaults are:

```text
command rate:       20 Hz
wheel acceleration: 0.12 m/s^2
wheel deceleration: 0.18 m/s^2
```

At a straight command of `0.06 m/s`, target speed is reached in roughly 0.5 s instead of being applied as one step. Normal zero-velocity commands are also ramped down.

Safety behavior is intentionally different: if `/cmd_vel` becomes stale for more than `0.35 s`, an emergency-stop message arrives on `/base/emergency_stop`, or the ROS node shuts down, the bridge bypasses the ramp and commands an immediate zero. Smooth motion must never weaken the emergency stop.

The final wheel-counter yaw defaults are also aligned with the validated bringup values:

```text
odom_yaw_scale_left:  0.50
odom_yaw_scale_right: 0.50
```

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

`/scan` at the LiDAR source should remain close to 10 Hz and `/odom` close to 20 Hz. SLAM Toolbox intentionally processes every second scan, so its effective scan-matching rate is about 5 Hz. The map update period is 2 seconds. SLAM Toolbox should be `active`, and `tf2_echo map laser` should resolve through the complete TF chain.

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

The dashboard displays live `/map`, current LiDAR returns, robot pose and battery voltage. Browser visualization is throttled so it does not mirror every high-rate ROS sample: `/scan` is shown at up to about 5 Hz, `/odom` and `/tf` at up to about 10 Hz, and `/map` at up to 1 Hz. SLAM itself still consumes ROS topics directly and is not routed through the browser.

Browser control starts disarmed after every page load. To use it, the operator must explicitly press `УПРАВЛЕНИЕ ЗАБЛОКИРОВАНО`, after which the button changes to `УПРАВЛЕНИЕ РАЗРЕШЕНО`.

The page intentionally does not combine linear and angular commands. Default mapping speeds are:

```text
linear:  0.06 m/s
angular: 0.25 rad/s
```

The angular control range is deliberately limited to `0.15..0.40 rad/s` for mapping. Fast skid-steer turns distort a rotating LiDAR scan and increase the chance of scan-matching errors.

When a normal movement key/button is released, the page keeps publishing zero velocity briefly so the base bridge has enough time to perform its smooth deceleration. The red `АВАРИЙНЫЙ СТОП` button and the `Space` key publish `/base/emergency_stop`, which bypasses the ramp and immediately forces both wheel commands to zero. Losing browser communication still falls back to the `0.35 s` bridge watchdog.

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

For a clean map, use this motion pattern:

```text
straight
-> stop for 0.5..1.0 s
-> slow in-place turn
-> stop for 0.5..1.0 s
-> straight
```

Avoid continuous moving arcs during the first map. They have not been separately calibrated, and the six-wheel skid-steer chassis introduces additional slip during combined linear/angular motion.

If walls duplicate, fan out, rotate, or drift badly on the live map, stop immediately and diagnose before continuing. Do not save a visibly corrupted map.

The ROS launch still defaults to `enable_motion:=false`, so merely opening the dashboard cannot move the robot unless mapping was explicitly restarted with motion enabled and browser control was then armed.

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

## Autonomous mode uses a fixed map

The final autonomous run must not keep modifying the occupancy map. After a good map is saved, the navigation stack will switch from SLAM mapping to localization on that fixed map:

```text
saved arena map
      +
LiDAR localization
      +
wheel odometry
      -> map->odom correction
      -> Nav2
      -> /cmd_vel
      -> smooth base bridge
```

This means a skid-steer turn can temporarily make the robot pose estimate less accurate, but it cannot twist or overwrite the saved arena map. The localization/Nav2 launch should be added and validated only after `maps/arena.yaml` and its image have been created and visually accepted.
