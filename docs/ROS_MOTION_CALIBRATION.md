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

## Third straight floor run — 5 second run and drive asymmetry

The longer run used:

```text
linear.x = 0.06 m/s
angular.z = 0
duration = 5.0 s
nominal commanded travel = 0.30 m
```

Observed result:

```text
SUMMARY: odom_samples=32 dx=0.3041m dy=0.0127m odom_distance=0.3044m dyaw=-12.77deg max|linear.x|=0.3802m/s max|angular.z|=2.2206rad/s
RAW_COUNTERS: samples=31 odl=38->66 delta=28 odr=36->62 delta=26 avg_delta=27.0 nominal_counter_distance=0.270m
```

Physical measurement/observation:

```text
forward travel: approximately 290 mm
lateral drift: approximately 60 mm to the right
```

This run exposes a repeatable drivetrain asymmetry that was less obvious in the shorter tests. The left cumulative counter increased by 28 counts while the right increased by 26 counts. A left side that travels farther than the right side produces a right-hand arc, which agrees with the observed physical drift.

The measured distance gives a provisional cumulative-counter scale:

```text
0.290 m / 27.0 counts ~= 0.01074 m/count
```

This is close to the firmware's nominal 0.01 m/count but indicates the real wheel/encoder scale is about 7% larger. Keep this value provisional until one more corrected straight run confirms it.

For straight-line command balancing, the observed raw-count ratio is:

```text
left/right = 28/26 ~= 1.077
```

A symmetric first correction that preserves approximately the same average command magnitude is therefore:

```text
left_command_scale  = 0.965
right_command_scale = 1.035
```

The bridge now supports these gains as ROS parameters. They are intentionally not made the defaults yet; the next floor run should explicitly launch with them and verify whether the rightward drift is substantially reduced.

The current `/odom` heading remains unreliable because it is still integrated from instantaneous `L/R`. The `-12.77 deg` reported yaw should not be used to judge the physical turn. Cumulative counters are the preferred candidate for the next odometry implementation.

## Next calibration: verify side-drive correction

Launch the robot with:

```text
left_command_scale=0.965
right_command_scale=1.035
```

and repeat the same 5 second, 0.06 m/s floor run. Measure both forward travel and lateral offset. If left/right counter deltas become equal (or nearly equal) and physical lateral drift falls substantially, adopt the gains as the initial straight-drive calibration.

After straightness is corrected, perform one accurately measured longer run to finalize `meters_per_counter`, then change pose integration to cumulative `odl/odr`. Only after that should effective skid-steer track width / turning odometry be calibrated.
