# ROS motion calibration

## Lifted-wheel ROS motion-path validation — 2026-09-12

The complete ROS drive path was tested with all six wheels off the ground:

```text
/cmd_vel
  -> rower_base_bridge
  -> Waveshare T=1 left/right command
  -> ESP32 drive controller
  -> wheel encoders
  -> T=1001 feedback
  -> /odom
```

Bringup was started explicitly with motion enabled:

```bash
ros2 launch rower_bringup robot.launch.py enable_motion:=true
```

The bridge reported:

```text
MOTION ENABLED: cmd_vel will be converted to T=1 wheel-speed commands.
Opened /dev/rower_base at 115200 baud; track_width=0.172 m
```

The guarded ROS motion probe commanded `linear.x=0.10 m/s`, `angular.z=0` for 1.0 s. Result:

```text
SUMMARY: odom_samples=27 dx=0.1178m dy=0.0031m displacement=0.1178m max|linear.x|=0.3802m/s max|angular.z|=2.2139rad/s
OK: ROS /cmd_vel motion path and odometry response are present.
```

This proves the complete software/firmware/encoder loop. It is **not** an odometry calibration result because the robot was lifted. With the chassis off the floor, the two skid-steer sides can accelerate and decelerate independently with essentially no tire-ground coupling. Instantaneous left/right encoder-speed differences can therefore create large transient `angular.z` values even though the commanded angular velocity is zero. The integrated distance also includes encoder motion while the wheels accelerate and settle after the command.

Do not tune `track_width`, wheel scale, or Nav2 parameters from the lifted-wheel numbers.

## First straight floor run — 2026-09-12

The robot was placed on the floor and commanded:

```text
linear.x = 0.06 m/s
angular.z = 0
duration = 1.5 s
nominal commanded travel = 0.09 m
```

Observed ROS result:

```text
SUMMARY: odom_samples=28 dx=0.0752m dy=-0.0083m odom_distance=0.0756m dyaw=-12.69deg max|linear.x|=0.3802m/s max|angular.z|=2.2206rad/s
```

Physical observation:

```text
travel: approximately 90 mm
straightness: confidently straight, no visible pull left/right
```

This is important: physical travel agrees well with the commanded 90 mm, but the current ROS pose integration reports only 75.6 mm and a false yaw change of about -12.7 degrees even though the chassis visibly drove straight. Therefore the command path is behaving well, while pose integration from instantaneous `L/R` speed samples is not yet trustworthy enough for SLAM/Nav2.

The approximate one-run distance ratio is:

```text
physical / ROS = 0.090 / 0.0756 ~= 1.19
```

This is only a provisional observation because the physical measurement was approximate and the run was short. Do not hard-code this scale yet.

## Next diagnostic: cumulative wheel odometer counters

The Waveshare `T=1001` feedback also carries cumulative `odl` and `odr` counters. These are potentially much better for pose integration than numerically integrating the noisy instantaneous `L/R` speed samples.

`rower_base_bridge` now republishes the useful raw T=1001 fields on:

```text
/base/raw_feedback
```

as compact JSON containing:

```text
L, R, odl, odr, v
```

`scripts/ros_floor_straight_probe.py` now captures the start/end `odl/odr` counters and prints a `RAW_COUNTERS` line. The next straight run should be somewhat longer to reduce measurement uncertainty, while remaining slow and safe. Recommended command:

```bash
python3 scripts/ros_floor_straight_probe.py \
  --run \
  --speed 0.06 \
  --seconds 3.0
```

Nominal travel is about 0.18 m. Measure the real center-to-center travel in millimetres and report whether the robot remained straight. Compare that physical distance to both raw counter deltas before changing the odometry implementation.

Only after straight-line scale is understood should effective skid-steer track width / turning odometry be calibrated.
