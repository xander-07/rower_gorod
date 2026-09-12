# Hardware validation — 2026-09-12

This file records tests performed on the real Waveshare UGV02 / Raspberry Pi 5 platform.

## LDROBOT STL-19P

Confirmed interface:

- device: `/dev/rower_lidar -> /dev/ttyUSB0`
- USB-UART: Silicon Labs CP2102, VID:PID `10c4:ea60`, serial `0001`
- baud: `230400`
- packet format: LD19/STL-19P family, 47-byte frames, 12 points/frame

Validation result with `scripts/stl19p_probe.py --frames 50`:

- 50/50 frames passed CRC8
- 592/600 points were valid
- spin speed approximately 3566–3573 deg/s = 9.91–9.93 Hz
- continuous wrap through 360 -> 0 degrees confirmed

## Waveshare UGV02 lower controller

Confirmed interface:

- device: `/dev/serial0 -> /dev/ttyAMA0`
- baud: `115200`
- newline-delimited JSON feedback is present
- `T=1001` base feedback confirmed
- `v=1200` means approximately 12.00 V because firmware transmits voltage multiplied by 100

Read-only test result:

```text
SUMMARY: elapsed=4.24s valid_json=50 base_feedback_T1001=10 other_json=40 malformed=0
```

## Motion command validation

Both command paths are confirmed on the installed ESP32 firmware.

### T=1 — independent left/right speed

Command:

```json
{"T":1,"L":0.15,"R":0.15}
```

Observed result:

```text
SUMMARY: mode=t1 commands=15 T1001=4 max|L|=0.1735 max|R|=0.1744 odom_changed=True malformed=0
OK: motion detected using t1 command path.
```

### T=13 — ROS-style linear/angular speed

Command:

```json
{"T":13,"X":0.15,"Z":0.0}
```

Observed result:

```text
SUMMARY: mode=t13 commands=15 T1001=3 max|L|=0.1014 max|R|=0.1005 odom_changed=True malformed=0
OK: motion detected using t13 command path.
```

Therefore the installed firmware definitely supports both `T=1` and `T=13`.

For the future ROS 2 bridge, `T=1` is currently preferred as the transport command because the current Waveshare firmware explicitly updates `lastCmdRecvTime` and clears `heartbeatStopFlag` in the `T=1` handler. The `T=13` handler calls `rosCtrl()` but does not update those heartbeat fields. The bridge can convert ROS `linear.x` / `angular.z` to left/right speeds itself using the effective track width.

## Module mode / IMU

The initial feedback showed `moduleType=1` behavior: RoArm bus-servo messages (`T=1005`) and `ax/ay/az` overwritten by arm coordinates.

A safe non-motion test was executed:

```bash
python3 scripts/ugv_set_base_mode.py --port /dev/serial0 --seconds 4
```

The command sent was only:

```json
{"T":4,"cmd":0}
```

Observed result:

```text
SUMMARY: T1001=79 T1005=2 other=0 malformed=0 first_raw_acc=(-250.1553416, 3.063519383e-14, -236.82) last_raw_acc=(0, 0, 0)
OK: moduleType=0 command was sent and base feedback is present.
```

Interpretation:

- switching to `moduleType=0` succeeded;
- only two `T=1005` packets appeared immediately after the switch, consistent with already-buffered servo messages; after that the base stream was clean;
- battery feedback during this test was `v=1193`, i.e. approximately `11.93 V`;
- the first `ax/ay/az` sample still contained the previous RoArm coordinates, but all subsequent `gx/gy/gz`, `ax/ay/az`, and `mx/my/mz` values were zero;
- therefore the installed firmware does **not yet have a validated usable IMU stream**. Zero fields may mean that the current DMP/raw-sensor path is not producing samples; this does not affect the already-confirmed motor, encoder, battery, or lidar paths.

The public Waveshare ROS firmware reads ICM-20948 DMP FIFO data in the main loop and only updates raw accel/gyro/magnetometer variables when the corresponding DMP header bits are present. Its orientation/quaternion processing is currently commented out. Therefore IMU functionality must be validated independently rather than assumed from the presence of the hardware.

A safe activity probe has been added:

```bash
python3 scripts/ugv_imu_probe.py --port /dev/serial0 --seconds 8
```

It sends no motor commands. During the test, slowly tilt and rotate the robot by hand. The script reports min/max values for gyro, accelerometer and magnetometer fields and determines whether they actually change.

## Current hardware conclusion

The following hardware paths are proven:

```text
STL-19P -> CP2102 -> Raspberry Pi 5           OK
Raspberry Pi 5 /dev/serial0 -> ESP32 feedback OK
ESP32 T=1 left/right velocity control          OK
ESP32 T=13 X/Z velocity control                OK
wheel encoders / odometry feedback             OK
battery voltage feedback                       OK
moduleType=0 base-only selection               OK
IMU data stream                                 NOT YET VALIDATED (currently zeros)
```

The robot can proceed to ROS 2 base/lidar integration even if the built-in IMU remains unavailable. Wheel odometry plus lidar localization/SLAM are sufficient for the first navigation stack; IMU can be added later after firmware repair or with a separate sensor.
