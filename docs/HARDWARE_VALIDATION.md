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

A safe base-only test sent:

```json
{"T":4,"cmd":0}
```

The switch succeeded. After the transition, `T=1005` messages stopped and normal `T=1001` base feedback continued. During the 4-second observation the controller produced 79 `T=1001` messages and only two residual `T=1005` messages immediately after the mode change. Battery feedback at that point was `v=1193`, approximately `11.93 V`.

A separate 8-second IMU activity test kept `moduleType=0` while the robot chassis was manually tilted and rotated. Result:

```text
SUMMARY: T1001=161 T1005=0 malformed=0
RANGES:
  gx: 0 .. 0
  gy: 0 .. 0
  gz: 0 .. 0
  ax: 0 .. 0
  ay: 0 .. 0
  az: 0 .. 0
  mx: 0 .. 0
  my: 0 .. 0
  mz: 0 .. 0
NO IMU DATA: all observed gyro/accel/mag fields remained zero.
```

Therefore the built-in ICM-20948 data path is **not considered usable for the first navigation stack**. The robot can proceed with wheel odometry + STL-19P + SLAM Toolbox/Nav2. The IMU can be repaired later in the ESP32 firmware or replaced with a separate sensor without blocking the current project.

## Current hardware conclusion

The following hardware paths are now proven:

```text
STL-19P -> CP2102 -> Raspberry Pi 5           OK
Raspberry Pi 5 /dev/serial0 -> ESP32 feedback OK
ESP32 T=1 left/right velocity control          OK
ESP32 T=13 X/Z velocity control                OK
wheel encoders / odometry feedback             OK
battery voltage feedback                       OK
base-only module mode                           OK
built-in IMU data stream                        NOT USABLE (all zeros)
```

Next: move the competition software to Ubuntu 24.04 arm64 + ROS 2 Jazzy on separate media, re-run the safe hardware probes, then build the ROS 2 base/lidar integration.
