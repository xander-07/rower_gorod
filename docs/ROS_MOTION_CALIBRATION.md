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

This proves the complete software/firmware/encoder loop. It is **not** an odometry calibration result because the robot was lifted. With the chassis off the floor, the two skid-steer sides can accelerate and decelerate independently with essentially no tire-ground coupling. Instantaneous left/right encoder-speed differences can therefore create large transient `angular.z` values even though the commanded angular velocity is zero.

## Straight-line calibration

The project moved pose integration from instantaneous `L/R` speed feedback to cumulative `odl/odr` counter increments. Independent drive-side gains were added to compensate the measured drivetrain asymmetry.

Final command-side defaults after floor testing:

```text
left_command_scale  = 0.965
right_command_scale = 1.035
```

A corrected 5 s straight run at `linear.x=0.06 m/s` physically travelled approximately 290 mm and stayed straight. Counter deltas were balanced at `28 / 28` in one run and `28 / 29` in the following validation run.

The final straight validation with counter-based odometry produced:

```text
SUMMARY: odom_samples=32 dx=0.2960m dy=0.0121m odom_distance=0.2963m dyaw=3.46deg max|linear.x|=0.0800m/s max|angular.z|=0.1008rad/s
RAW_COUNTERS: samples=31 odl=98->126 delta=28 odr=94->123 delta=29 avg_delta=28.5 nominal_counter_distance=0.285m
```

Physical observation:

```text
travel: approximately 290 mm
straightness: straight
```

This confirms that counter-based pose and rolling-window twist estimation removed the previous large false velocity spikes. The remaining `3.46 deg` yaw on a physically straight run is explained by the coarse counter quantization: one count of left/right difference is already several degrees when divided by the current track width.

The straight-run physical scale was refined to:

```text
odom_meters_per_count = 0.0102 m/count
```

because `28.5 * 0.0102 = 0.2907 m`, closely matching the measured ~0.290 m.

## First in-place turn calibration

The first controlled turn used:

```text
angular.z = +0.80 rad/s
command duration = 2.0 s
direction = left / CCW
nominal commanded angle = 91.7 deg
```

Observed ROS/counter result:

```text
SUMMARY: odom_samples=18 dx=0.0061m dy=-0.0006m center_drift=0.0061m dyaw=50.97deg max|linear.x|=0.0170m/s max|angular.z|=0.6387rad/s
RAW_COUNTERS: samples=17 odl=128->122 delta=-6 odr=124->132 delta=8 delta_difference=14
```

Physical heading change was estimated at approximately 45–50 degrees to the left. The robot rotated about its center with only small translational drift (~6 mm in ROS odometry).

Using the calibrated counter scale, the wheel-side differential path is:

```text
(dr - dl) = 14 counts * 0.0102 m/count = 0.1428 m
```

The effective track width inferred from the physical 45–50 degree estimate spans approximately:

```text
45 deg -> 0.1818 m
50 deg -> 0.1637 m
midpoint 47.5 deg -> ~0.1723 m
```

This is extremely close to the current `track_width = 0.172 m`. Therefore the current track width should **not** be changed from this first turn. A right-turn run is required to check symmetry and reduce uncertainty before locking the value for SLAM/Nav2.

## Next step

Run a controlled right/CW turn using the same calibrated bridge. A 3.0 s duration is preferred to collect more counter increments and reduce one-count quantization error:

```bash
python3 scripts/ros_floor_turn_probe.py \
  --run \
  --angular -0.80 \
  --seconds 3.0
```

Measure the real clockwise heading change from floor marks and report the `SUMMARY` and `RAW_COUNTERS`. If the inferred effective track width again clusters around ~0.172 m, keep `track_width=0.172` and proceed to SLAM Toolbox validation.
