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

The initial feedback showed `moduleType=1` behavior: RoArm bus-servo messages (`T=1005`) and `ax/ay/az` overwritten by arm coordinates. For navigation this must be changed to base-only mode.

A safe non-motion diagnostic is available:

```bash
python3 scripts/ugv_set_base_mode.py --port /dev/serial0 --seconds 4
```

It sends only:

```json
{"T":4,"cmd":0}
```

which changes `moduleType` to 0 in RAM, then observes `T=1001`. In base-only mode the `ax/ay/az` fields should again contain raw IMU accelerometer data rather than RoArm coordinates.

## Current hardware conclusion

The following hardware paths are now proven:

```text
STL-19P -> CP2102 -> Raspberry Pi 5           OK
Raspberry Pi 5 /dev/serial0 -> ESP32 feedback OK
ESP32 T=1 left/right velocity control          OK
ESP32 T=13 X/Z velocity control                OK
wheel encoders / odometry feedback             OK
battery voltage feedback                       OK
```

Next: switch to base-only module mode, validate real IMU feedback, then build the ROS 2 base/lidar integration.
