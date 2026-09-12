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

Physical travel agreed well with the commanded 90 mm, but the original ROS pose integration reported only 75.6 mm and a false yaw change of about -12.7 degrees even though the chassis visibly drove straight. That showed that pose integration from instantaneous `L/R` feedback was not trustworthy enough for SLAM/Nav2.

## Second straight floor run — cumulative counter check

Command:

```text
linear.x = 0.06 m/s
angular.z = 0
duration = 3.0 s
nominal commanded travel = 0.18 m
```

Observed result:

```text
SUMMARY: odom_samples=22 dx=0.1613m dy=0.0110m odom_distance=0.1617m dyaw=6.32deg max|linear.x|=0.3799m/s max|angular.z|=2.2162rad/s
RAW_COUNTERS: samples=21 odl=20->36 delta=16 odr=19->34 delta=15
```

Physical travel was approximately 170–180 mm. The cumulative counter deltas were nearly equal (`16` vs `15`) and matched the physical straightness better than the instantaneous velocity-derived yaw.

## Third straight floor run — drivetrain asymmetry observed

A 5 second / 0.06 m/s run produced:

```text
SUMMARY: odom_samples=32 dx=0.3041m dy=0.0127m odom_distance=0.3044m dyaw=-12.77deg max|linear.x|=0.3802m/s max|angular.z|=2.2206rad/s
RAW_COUNTERS: samples=31 odl=38->66 delta=28 odr=36->62 delta=26 avg_delta=27.0 nominal_counter_distance=0.270m
```

Physical observation:

```text
forward travel: approximately 290 mm
lateral drift: approximately 60 mm to the right
```

The left side accumulated about 7.7% more counter distance than the right side (`28` vs `26`), which matches the physical rightward curvature. To compensate, the command path was given independent left/right calibration gains:

```text
left_command_scale  = 0.965
right_command_scale = 1.035
```

## Fourth straight floor run — drivetrain correction validated

With the calibrated drive gains active, the same 5 second / 0.06 m/s test produced:

```text
SUMMARY: odom_samples=32 dx=0.2555m dy=-0.0890m odom_distance=0.2706m dyaw=-44.54deg max|linear.x|=0.1904m/s max|angular.z|=2.2139rad/s
RAW_COUNTERS: samples=31 odl=68->96 delta=28 odr=64->92 delta=28 avg_delta=28.0 nominal_counter_distance=0.280m
```

Physical observation:

```text
forward travel: approximately 290 mm
straightness: physically straight / no meaningful side pull observed
```

The key result is `28 / 28`: the cumulative left and right wheel-side counters are now balanced, and the chassis physically drove straight. The previous false ROS yaw (`-44.54 deg`) therefore came from integrating the noisy instantaneous `L/R` samples, not from a real 44 degree turn.

The physical scale from this longer straight run is approximately:

```text
0.290 m / 28 counts = 0.01036 m/count
```

Because the physical measurement was still approximate, the bridge uses a rounded initial value of:

```text
odom_meters_per_count = 0.0104 m/count
```

This is close to the firmware's nominal 0.01 m/count representation and can be refined later with a longer taped measurement.

## Odometry implementation after straight-line calibration

`rower_base_bridge` now uses cumulative `odl` / `odr` counter increments as the primary pose source:

```text
dl = delta_odl * odom_meters_per_count
dr = delta_odr * odom_meters_per_count
 ds = (dl + dr) / 2
dtheta = (dr - dl) / track_width
```

The instantaneous Waveshare `L/R` fields are no longer integrated into `x/y/yaw`. They remain available in `/base/raw_feedback` for diagnostics.

Because the cumulative counters are deliberately low-resolution, velocity is estimated over a rolling counter window rather than differentiating each individual count. This should remove the 0.38 m/s / 2.2 rad/s single-sample spikes from normal `/odom.twist` behavior.

The drive correction gains are now the project defaults:

```text
left_command_scale  = 0.965
right_command_scale = 1.035
```

The straight-line stage is therefore good enough to move on to validation of the new counter-based odometry and then effective skid-steer turning-width calibration.

## Next step

Rebuild `rower_base_bridge` and `rower_bringup`, run the same 5 second straight probe once more, and verify that the new `/odom` result is near the physical ~0.29 m with near-zero final yaw when the raw counters finish equal. After that, calibrate effective `track_width` using a controlled in-place rotation before starting SLAM Toolbox.
