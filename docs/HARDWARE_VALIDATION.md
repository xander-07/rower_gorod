# Hardware validation — 2026-09-12

This file records tests performed on the real Waveshare UGV02 / Raspberry Pi 4 platform.

## Host platform

Confirmed competition computer:

- Raspberry Pi 4, 4 GB RAM;
- Debian GNU/Linux 12 (Bookworm), arm64/aarch64;
- Python 3.11.2;
- vendor stack: `~/ugv_rpi` with `~/ugv_rpi/ugv-env`;
- ROS 2 Jazzy runs in the `rower-ros2:jazzy` Docker image rather than being installed natively on Debian.

## LDROBOT STL-19P

Confirmed host interface:

- device: `/dev/rower_lidar -> /dev/ttyUSB0`;
- USB-UART: Silicon Labs CP2102, VID:PID `10c4:ea60`, serial `0001`;
- baud: `230400`;
- packet format: LD19/STL-19P family, 47-byte frames, 12 points/frame.

Host validation with `scripts/stl19p_probe.py --frames 50`:

- 50/50 frames passed CRC8;
- 592/600 points were valid;
- spin speed approximately 3566–3573 deg/s = 9.91–9.93 Hz;
- continuous wrap through 360 -> 0 degrees confirmed.

Docker/ROS environment validation through `/dev/rower_lidar`:

```text
OK: received 20 CRC-valid STL-19P/LD19 frames in 0.05s; valid points=240/240
```

Observed spin speed was approximately 3564–3571 deg/s = 9.90–9.92 Hz, including a packet crossing `352.50 -> 0.37 deg`. Therefore the complete host -> Docker -> serial lidar path is confirmed.

## Waveshare UGV02 lower controller

Confirmed host interface:

- device: `/dev/serial0 -> /dev/ttyAMA0`;
- baud: `115200`;
- newline-delimited JSON feedback is present;
- `T=1001` base feedback confirmed;
- voltage is transmitted multiplied by 100 (`v=1213` = approximately `12.13 V`).

Initial host read-only validation:

```text
SUMMARY: elapsed=4.24s valid_json=50 base_feedback_T1001=10 other_json=40 malformed=0
```

Docker/ROS environment validation through `/dev/rower_base`:

```text
SUMMARY: elapsed=3.02s valid_json=35 base_feedback_T1001=7 other_json=28 malformed=1
OK: Waveshare base feedback is present on the GPIO UART.
```

The single malformed line is expected when a read-only process opens an already-streaming newline-delimited UART in the middle of one JSON record. All following records parsed normally.

The Docker launcher maps:

```text
host /dev/serial0      -> container /dev/rower_base
host /dev/rower_lidar  -> container /dev/rower_lidar
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

For the ROS 2 bridge, `T=1` is preferred as the transport command because the current Waveshare firmware explicitly refreshes its heartbeat in the `T=1` handler. The bridge converts ROS `linear.x` / `angular.z` to left/right wheel-side speeds itself.

## Module mode / IMU

After controller boot the observed stream again contains RoArm behavior (`T=1005` IDs 11/12/14/15 and arm-coordinate values in `ax/ay/az`). This is expected because the earlier `moduleType=0` command changes RAM only and is not persistent across controller restart.

A safe base-only test sent:

```json
{"T":4,"cmd":0}
```

The switch succeeded. After the transition, `T=1005` messages stopped and normal `T=1001` base feedback continued.

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

Therefore the built-in ICM-20948 data path is **not considered usable for the first navigation stack**. The ROS base bridge sends `T=4,cmd=0` at startup to select base-only mode, but the first navigation stack does not use IMU data.

## ROS 2 Docker validation

The Docker image `rower-ros2:jazzy` is built and confirmed working on the Raspberry Pi 4. Installed and verified packages include:

- ROS 2 Jazzy;
- `nav2_bringup`;
- `slam_toolbox`;
- `rmw_cyclonedds_cpp`.

The image occupies approximately 4.74 GB of local disk space. Both physical serial devices have been validated from inside the container.

## ROS node validation

The first real project packages build successfully with colcon:

```text
Summary: 2 packages finished [7.60s]
rower_base_bridge base_bridge
rower_lidar lidar_node
```

### `rower_lidar`

The node opens `/dev/rower_lidar` at 230400 and publishes `sensor_msgs/msg/LaserScan` on `/scan` with frame `laser`.

Measured ROS topic rate:

```text
average rate: approximately 9.91 Hz
min: approximately 0.093 s
max: approximately 0.107 s
```

A long run reached approximately 143451 valid serial frames and 3424 published scans with only one observed CRC error. The single CRC error is negligible relative to the total frame count and the driver recovered automatically.

### `rower_base_bridge`

The bridge was started with its default safe configuration:

```text
Motion is DISABLED (enable_motion:=false). Telemetry/odometry only.
Opened /dev/rower_base at 115200 baud; track_width=0.172 m
```

After the bridge sends the safe RAM-only `T=4,cmd=0` base-mode command, `/odom` is published at approximately 20 Hz.

Stationary odometry was confirmed:

```text
frame_id: odom
child_frame_id: base_link
position x: 0.0
position y: 0.0
orientation w: 1.0
linear.x: 0.0
angular.z: 0.0
```

Battery telemetry was confirmed on `/battery`:

```text
voltage: 12.140000343322754
present: true
```

The bridge also publishes the `odom -> base_link` transform on `/tf`. `/cmd_vel` exists as a subscription but is ignored for motor output while `enable_motion=false`.

## Mechanical geometry / lidar transform

The supplied UGV02 mechanical drawing gives a conservative overall envelope of approximately:

```text
length: 252 mm
width:  230 mm
height:  94 mm
```

For ROS/Nav2 we use `base_link` at the geometric center of the robot footprint on the ground plane, with the standard convention `+X forward`, `+Y left`, `+Z up`.

The real STL-19P installation was measured as:

```text
x: +35..+45 mm ahead of robot center -> nominal +40 mm
y: 0 mm (centered laterally unless later measurement says otherwise)
z: 115.5 mm scan-plane height
```

The lidar's marked arrow/zero direction points to the robot's left relative to the robot front. Therefore the nominal fixed transform uses:

```text
base_link -> laser
xyz = [0.040, 0.000, 0.1155] m
yaw = +90 deg = +1.57079632679 rad
```

The conservative Nav2 footprint based on the outer wheel envelope is:

```text
[[ 0.126,  0.115],
 [ 0.126, -0.115],
 [-0.126, -0.115],
 [-0.126,  0.115]]
```

These values are implemented in `rower_description`. The +40 mm X offset is the midpoint of the measured +35..+45 mm range and can be refined later.

## Bringup / TF validation

`rower_description` and `rower_bringup` build successfully together with the base and lidar packages:

```text
Summary: 4 packages finished [14.4s]
```

`ros2 launch rower_bringup robot.launch.py` successfully starts:

- `robot_state_publisher`;
- `rower_lidar`;
- `rower_base_bridge` in motion-disabled mode.

The lidar remained stable during bringup, reaching more than 56k serial frames with zero CRC errors in the observed run. Ctrl+C shut down all three launched processes cleanly.

The fixed transform was measured through TF as:

```text
base_link -> laser
Translation: [0.040, 0.000, 0.116]
RPY degrees: [0.000, -0.000, 90.000]
Quaternion xyzw: [0.000, 0.000, 0.707, 0.707]
```

The complete dynamic chain `odom -> base_link -> laser` was also resolved successfully. The one initial `Invalid frame ID` message from `tf2_echo` occurred only during startup before the first dynamic `odom` transform arrived; subsequent transforms were continuous and correct.

## Current conclusion

```text
Raspberry Pi 4 / Debian 12 host                        OK
Docker / Ubuntu Noble / ROS 2 Jazzy                   OK
host -> Docker -> ESP32 UART feedback                  OK
host -> Docker -> STL-19P serial                       OK
STL-19P CRC / full angular wrap                        OK
ROS /scan at ~9.91 Hz                                  OK
ROS /odom at ~20 Hz                                    OK
ROS /battery                                           OK
ROS odom -> base_link TF                               OK
ROS base_link -> laser static TF                       OK
ROS odom -> base_link -> laser full TF chain           OK
rower_bringup unified launch                           OK
ESP32 T=1 left/right velocity control                  OK
ESP32 T=13 X/Z velocity control                        OK
wheel encoders / odometry feedback                     OK
battery voltage feedback                               OK
base-only module selection via T=4,cmd=0               OK (RAM only)
built-in IMU data stream                               NOT USABLE (all zeros)
robot envelope / lidar mounting geometry               RECORDED
```

Next: validate the complete ROS `/cmd_vel -> rower_base_bridge -> ESP32 -> encoder -> /odom` motion path with all six wheels lifted. After that, perform slow floor calibration and start SLAM Toolbox. A guarded helper is provided as `scripts/ros_motion_probe.py`.
