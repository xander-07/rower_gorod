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

## Next calibration: slow straight run on the floor

Use `scripts/ros_floor_straight_probe.py`. The default command is deliberately conservative:

```text
speed: 0.06 m/s
duration: 1.5 s
nominal commanded travel: 0.09 m
```

The test requires `--run`, waits three seconds before motion, repeatedly publishes zero before and after the test, and limits speed to at most 0.10 m/s and duration to at most 2.0 s.

Run only on a clear, flat floor with at least 1 m of free space in front of the robot. Mark the robot center before and after the run and measure the real displacement in millimetres. Also note whether the chassis visibly pulls left or right.

The first floor test is intended to compare physical travel with encoder-derived `/odom` and to check straightness. Effective skid-steer track width should only be calibrated after straight-line scale/sign behavior is understood.
