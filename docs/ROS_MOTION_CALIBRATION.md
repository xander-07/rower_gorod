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

## Second straight floor run — cumulative counter check

The next run used:

```text
linear.x = 0.06 m/s
angular.z = 0
duration = 3.0 s
nominal commanded travel = 0.18 m
```

Observed ROS result:

```text
SUMMARY: odom_samples=22 dx=0.1613m dy=0.0110m odom_distance=0.1617m dyaw=6.32deg max|linear.x|=0.3799m/s max|angular.z|=2.2162rad/s
RAW_COUNTERS: samples=21 odl=20->36 delta=16 odr=19->34 delta=15
```

Physical observation:

```text
travel: approximately 170-180 mm
straightness: approximately straight / no reported visible pull
```

The left/right cumulative counter deltas are close (`16` vs `15`), which matches the visual straightness much better than the noisy instantaneous `L/R`-derived yaw. In the current Waveshare firmware the counters are transmitted as `int(en_odom_* * 100)`, so one integer count is nominally 0.01 m. The average raw delta of 15.5 counts therefore corresponds to a nominal 0.155 m before physical scale correction.

Using the approximate midpoint of the physical estimate (0.175 m) would imply a provisional counter scale correction of about:

```text
0.175 / 0.155 ~= 1.13
```

but this must **not** be hard-coded yet because the physical measurement was only an estimate and the counters are quantized to whole centimetre-style units. The run does, however, strongly support switching pose integration from instantaneous `L/R` speed samples to cumulative `odl/odr` increments once their physical scale is measured accurately.

The false ROS yaw is still present (`+6.32 deg`) even though the robot was visually straight. This confirms that instantaneous `L/R` speed samples should not be used directly for long-term heading integration.

## Next calibration: longer precisely measured straight run

Use a longer run so the 1-count quantization of `odl/odr` becomes a smaller percentage of the total distance. Mark the robot center before and after the run and measure with a ruler/tape, preferably to within a few millimetres.

Recommended command:

```bash
python3 scripts/ros_floor_straight_probe.py \
  --run \
  --speed 0.06 \
  --seconds 5.0
```

Nominal commanded travel is 0.30 m. Use only with at least 1 m clear space ahead. The script now permits up to 6 seconds and reports `nominal_counter_distance` from the raw counters for convenience.

After one accurately measured longer run, calculate the physical metres-per-counter scale. Then update `rower_base_bridge` to integrate pose from cumulative `odl/odr` deltas. Keep instantaneous `L/R` only as velocity telemetry (and potentially filter it) rather than as the primary pose source.

Only after straight-line scale is fixed should effective skid-steer track width / turning odometry be calibrated.
