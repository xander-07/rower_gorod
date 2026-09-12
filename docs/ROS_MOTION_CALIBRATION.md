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

## Turn calibration: LiDAR is the physical-angle reference

Early visual turn estimates were too coarse. The STL-19P scan-matching probe was corrected to search both circular-shift directions, and its result was then confirmed by direct physical observation. The LiDAR result is therefore used as the turn-calibration reference.

### 0.80 rad/s raw command, 2.0 s

Left / CCW:

```text
LIDAR_RESULT: chassis_angle=23.25deg magnitude=23.25deg raw_scan_shift=-23.25deg shift=-31bins bin_size=0.750deg overlap=477 score=0.0043
```

Right / CW:

```text
LIDAR_RESULT: chassis_angle=-24.00deg magnitude=24.00deg raw_scan_shift=24.00deg shift=32bins bin_size=0.750deg overlap=478 score=0.0048
```

Real average angular speed is about `0.206 rad/s` for a requested `0.80 rad/s` before compensation.

### 1.60 rad/s raw command, 2.0 s

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

Real average angular speed is about `0.592 rad/s` for a requested `1.60 rad/s`.

### 2.00 rad/s raw command, 2.0 s

Left / CCW:

```text
LIDAR_RESULT: chassis_angle=89.25deg magnitude=89.25deg raw_scan_shift=-89.25deg shift=-119bins bin_size=0.750deg overlap=476 score=0.0167
ODOM_RESULT: angle=173.29deg center_drift=0.0130m
RAW_COUNTERS: odl=152->132 delta=-20 odr=103->126 delta=23 delta_difference=43
TURN_METRICS: commanded_avg=2.0000rad/s lidar_avg=0.7789rad/s response_ratio=0.3894 odom_angle=173.29deg raw_diff=43
```

Right / CW:

```text
LIDAR_RESULT: chassis_angle=-85.50deg magnitude=85.50deg raw_scan_shift=85.50deg shift=114bins bin_size=0.750deg overlap=479 score=0.0191
ODOM_RESULT: angle=173.12deg center_drift=0.0063m
RAW_COUNTERS: odl=124->147 delta=23 odr=134->109 delta=-25 delta_difference=-48
TURN_METRICS: commanded_avg=2.0000rad/s lidar_avg=0.7461rad/s response_ratio=0.3731 odom_angle=173.12deg raw_diff=-48
```

These results were physically confirmed: the chassis really turns about `89 deg` left and `85.5 deg` right, not the roughly `173 deg` reported by the uncorrected wheel-yaw model.

## Angular command compensation

Separate affine fits from the measured operating points give approximately:

```text
left : omega_real ~= 0.4804 * omega_raw - 0.1810
right: omega_real ~= 0.4523 * omega_raw - 0.1463
```

The bridge applies the inverse relationship for near-in-place turns:

```text
left raw magnitude  ~= 2.0817 * desired_omega + 0.3767
right raw magnitude ~= 2.2110 * desired_omega + 0.3235
```

Defaults:

```text
angular_compensation_enabled = true
angular_compensation_linear_threshold = 0.02 m/s
angular_command_deadband = 0.03 rad/s
angular_command_max_raw = 2.50 rad/s
```

This correction is intentionally limited to `|linear.x| <= 0.02 m/s`. Moving arcs have not yet been separately calibrated and are left unchanged.

## Final compensated 0.80 rad/s validation

With angular command compensation enabled, a requested `0.80 rad/s` for 2.0 seconds produced almost exactly the requested physical yaw rate in both directions.

Left / CCW:

```text
LIDAR_RESULT: chassis_angle=91.50deg magnitude=91.50deg raw_scan_shift=-91.50deg shift=-122bins bin_size=0.750deg overlap=479 score=0.0185
ODOM_RESULT: angle=108.25deg center_drift=0.0054m
RAW_COUNTERS: odl=152->130 delta=-22 odr=104->127 delta=23 delta_difference=45
TURN_METRICS: commanded_avg=0.8000rad/s lidar_avg=0.7985rad/s response_ratio=0.9981 odom_angle=108.25deg raw_diff=45
```

Right / CW:

```text
LIDAR_RESULT: chassis_angle=-91.50deg magnitude=91.50deg raw_scan_shift=91.50deg shift=122bins bin_size=0.750deg overlap=476 score=0.0180
ODOM_RESULT: angle=-99.08deg center_drift=0.0100m
RAW_COUNTERS: odl=123->145 delta=22 odr=134->110 delta=-24 delta_difference=-46
TURN_METRICS: commanded_avg=0.8000rad/s lidar_avg=0.7985rad/s response_ratio=0.9981 odom_angle=-99.08deg raw_diff=-46
```

The physical angular-command calibration is therefore validated: requested `0.80 rad/s` produced `0.7985 rad/s` in both directions, a response ratio of `0.9981`.

## Wheel-odometry yaw refinement

The compensated test showed that the previous wheel-yaw scales (`0.60` left, `0.54` right) still over-reported heading somewhat. Refining them against the LiDAR reference gives values very close to `0.50` in both directions:

```text
left refinement  = 0.60 * 91.50 / 108.25 ~= 0.507
right refinement = 0.54 * 91.50 / 99.08  ~= 0.499
```

For simplicity and symmetry, bringup now uses:

```text
odom_yaw_scale_left  = 0.50
odom_yaw_scale_right = 0.50
```

Expected yaw with the same raw counter motion is then approximately `90.2 deg` left and `91.7 deg` right, both very close to the LiDAR `91.5 deg` reference.

Yaw covariance remains deliberately conservative because six-wheel skid-steer heading is floor/slip dependent and SLAM must be allowed to correct it.

## Status / next step

Straight distance, straight-line drivetrain balance, and in-place angular command response are now calibrated well enough to proceed to SLAM Toolbox testing. One short left/right validation with the final `0.50 / 0.50` wheel-yaw scale is useful before relying on wheel odometry for Nav2, but the physical turn command itself is already validated.
