# ROS motion calibration

## Lifted-wheel ROS motion-path validation — 2026-09-12

The complete ROS drive path was validated with all six wheels off the ground:

```text
/cmd_vel
  -> rower_base_bridge
  -> Waveshare T=1 left/right command
  -> ESP32 drive controller
  -> wheel encoders
  -> T=1001 feedback
  -> /odom
```

This proved the software/firmware/encoder loop, but lifted-wheel measurements were not used for calibration.

## Straight-line calibration

Pose integration was moved from noisy instantaneous `L/R` feedback to cumulative `odl/odr` counter increments.

Final drive-side command gains from floor testing:

```text
left_command_scale  = 0.965
right_command_scale = 1.035
```

A corrected 5 s straight run at `linear.x=0.06 m/s` travelled approximately 290 mm and remained physically straight.

Counter-based validation:

```text
SUMMARY: odom_samples=32 dx=0.2960m dy=0.0121m odom_distance=0.2963m dyaw=3.46deg max|linear.x|=0.0800m/s max|angular.z|=0.1008rad/s
RAW_COUNTERS: samples=31 odl=98->126 delta=28 odr=94->123 delta=29 avg_delta=28.5 nominal_counter_distance=0.285m
```

The straight-line counter scale was refined to:

```text
odom_meters_per_count = 0.0102 m/count
```

because `28.5 * 0.0102 = 0.2907 m`, closely matching the measured ~0.290 m.

## Turn calibration: important correction

Early turn tests estimated the chassis angle visually and initially suggested ~45–50 degrees for a `0.80 rad/s`, 2 s command. Later LiDAR scan matching showed that estimate was too large. The actual physical turn is about 23–24 degrees, which the operator confirmed visually after seeing the LiDAR result.

Therefore do **not** infer effective track width from the earlier 45–50 degree visual estimate.

The LiDAR scan matcher searches both circular-shift directions and reports the physical chassis-angle magnitude independently of wheel odometry.

### 0.80 rad/s command, 2.0 s

Left / CCW:

```text
LIDAR_RESULT: chassis_angle=23.25deg magnitude=23.25deg raw_scan_shift=-23.25deg shift=-31bins bin_size=0.750deg overlap=477 score=0.0043
```

Right / CW:

```text
LIDAR_RESULT: chassis_angle=-24.00deg magnitude=24.00deg raw_scan_shift=24.00deg shift=32bins bin_size=0.750deg overlap=478 score=0.0048
```

The real average angular rates are therefore approximately:

```text
left  = 23.25 deg / 2 s = 0.2029 rad/s
right = 24.00 deg / 2 s = 0.2094 rad/s
mean  = 0.2062 rad/s
```

For a commanded magnitude of `0.80 rad/s`, the physical response is only about 25.8% of the requested angular velocity.

### 1.60 rad/s command, 2.0 s

Left / CCW:

```text
LIDAR_RESULT: chassis_angle=67.50deg magnitude=67.50deg raw_scan_shift=-67.50deg shift=-90bins bin_size=0.750deg overlap=475 score=0.0138
ODOM_RESULT: angle=135.91deg center_drift=0.0083m
RAW_COUNTERS: odl=150->134 delta=-16 odr=103->121 delta=18 delta_difference=34
TURN_METRICS: commanded_avg=1.6000rad/s lidar_avg=0.5890rad/s response_ratio=0.3682 odom_angle=135.91deg raw_diff=34
```

Right / CW:

```text
LIDAR_RESULT: chassis_angle=-68.25deg magnitude=68.25deg raw_scan_shift=68.25deg shift=91bins bin_size=0.750deg overlap=477 score=0.0189
ODOM_RESULT: angle=-139.31deg center_drift=0.0024m
RAW_COUNTERS: odl=129->147 delta=18 odr=126->108 delta=-18 delta_difference=-36
TURN_METRICS: commanded_avg=1.6000rad/s lidar_avg=0.5956rad/s response_ratio=0.3722 odom_angle=-139.31deg raw_diff=-36
```

The real average angular rates are approximately:

```text
left  = 0.5890 rad/s
right = 0.5956 rad/s
mean  = 0.5923 rad/s
```

The drivetrain is physically very symmetric left vs right, but the angular response is strongly nonlinear: doubling the requested angular speed from `0.80` to `1.60 rad/s` increases the real angular speed from about `0.206` to `0.592 rad/s`.

A simple affine fit through the two measured mean operating points is:

```text
omega_real ~= 0.4827 * omega_command - 0.1800   (for positive magnitudes in this tested range)
```

This is consistent with a substantial low-speed dead-zone / friction threshold in the skid-steer drivetrain.

The inverse estimate is:

```text
omega_command ~= (omega_desired + 0.1800) / 0.4827
```

For a desired real `90 deg` turn in `2.0 s`, `omega_desired = pi/4 ~= 0.7854 rad/s`; the model predicts an internal command of almost exactly:

```text
omega_command ~= 2.00 rad/s
```

This must be validated experimentally before adding command linearization to `rower_base_bridge`.

## Wheel odometry yaw during turns

At `1.60 rad/s`, 2 s, wheel odometry reported roughly twice the LiDAR-measured physical yaw:

```text
left:  odom 135.91 deg vs LiDAR 67.50 deg
right: odom 139.31 deg vs LiDAR 68.25 deg
```

Therefore the current wheel-counter yaw model is not yet suitable as final heading truth for skid-steer turns. Linear distance from the cumulative counters remains useful and well calibrated, but heading requires separate calibration / fusion. Do not tune `track_width` from a single turn direction or from the earlier visual angle estimate.

## Next validation

Run matched left/right tests at `|angular.z| = 2.00 rad/s` for 2.0 s. This remains below the current wheel-speed clamp because the ideal differential side speed is approximately `0.172 m/s` before side gains.

If LiDAR reports approximately 90 degrees in both directions, the affine command-response model is validated well enough to add angular-command compensation in the bridge. After that, address yaw estimation separately before Nav2 closed-loop motion.
