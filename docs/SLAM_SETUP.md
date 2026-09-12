# SLAM Toolbox setup

The physical robot runs ROS 2 Jazzy inside the project Docker image on Debian 12. The mapping stack uses:

```text
/scan -> slam_toolbox
/odom + odom->base_link -> slam_toolbox
base_link->laser -> robot_state_publisher
slam_toolbox -> map + map->odom
```

## Build

From the Debian host:

```bash
cd ~/rower_gorod
git pull

./scripts/run_ros2_docker.sh \
  colcon build \
  --symlink-install \
  --packages-select rower_navigation
```

## Safe startup check

Start the complete robot + SLAM stack with physical motion disabled:

```bash
cd ~/rower_gorod

./scripts/run_ros2_docker.sh \
  ros2 launch rower_navigation mapping.launch.py \
  enable_motion:=false
```

Expected graph:

```text
map -> odom -> base_link -> laser
```

Useful checks from another SSH terminal:

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

`/scan` should be close to 10 Hz, `/odom` should be present continuously, `/map` should appear after SLAM Toolbox has started, and `tf2_echo map laser` should resolve through `map -> odom -> base_link -> laser`.

## Mapping with motion enabled

Only after the safe startup check passes, stop the previous launch and restart explicitly with drive motion enabled:

```bash
cd ~/rower_gorod

./scripts/run_ros2_docker.sh \
  ros2 launch rower_navigation mapping.launch.py \
  enable_motion:=true
```

Use slow manual motion. For the first mapping run, prefer straight segments and in-place turns because those modes have been calibrated. Moving arcs are not yet separately calibrated.

A keyboard teleop can be started from another SSH terminal:

```bash
cd ~/rower_gorod

./scripts/run_ros2_docker.sh \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p speed:=0.06 \
  -p turn:=0.40
```

Keep the area clear and be ready to stop the mapping launch with Ctrl+C if the chassis behaves unexpectedly.

## Save a map

When the map is clean and the robot has revisited enough of the environment for loop closure, save it into the repository:

```bash
cd ~/rower_gorod

./scripts/run_ros2_docker.sh \
  ros2 run nav2_map_server map_saver_cli \
  -f /workspace/rower_gorod/maps/arena
```

This produces `maps/arena.yaml` and `maps/arena.pgm`. Commit both files after visually validating the saved map.
