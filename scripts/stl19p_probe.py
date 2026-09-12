#!/usr/bin/env python3
"""Non-invasive serial probe for LDROBOT STL-19P / LD19-family packets.

The STL-19P sends 47-byte measurement frames automatically at 230400 baud.
This tool only reads the serial stream; it does not transmit commands to the LiDAR.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial is not installed. Install it with: sudo apt install python3-serial", file=sys.stderr)
    raise SystemExit(2)

FRAME_LEN = 47
HEADER = 0x54
VER_LEN = 0x2C
POINTS_PER_FRAME = 12
CRC8_POLY = 0x4D


def crc8_ldrobot(data: bytes) -> int:
    """Calculate the LDROBOT CRC-8 used by 0x54/0x2C measurement packets."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC8_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def read_frame(ser: serial.Serial) -> bytes:
    """Synchronize to a probable STL-19P/LD19 frame and return 47 bytes."""
    while True:
        first = ser.read(1)
        if not first:
            raise TimeoutError("No serial data received")
        if first[0] != HEADER:
            continue

        second = ser.read(1)
        if not second:
            raise TimeoutError("Incomplete frame")
        if second[0] != VER_LEN:
            continue

        rest = ser.read(FRAME_LEN - 2)
        if len(rest) != FRAME_LEN - 2:
            raise TimeoutError("Incomplete frame")
        return first + second + rest


def parse_frame(frame: bytes) -> dict:
    speed = struct.unpack_from("<H", frame, 2)[0]
    start_angle = struct.unpack_from("<H", frame, 4)[0] / 100.0

    points = []
    offset = 6
    for _ in range(POINTS_PER_FRAME):
        distance_mm = struct.unpack_from("<H", frame, offset)[0]
        confidence = frame[offset + 2]
        points.append((distance_mm, confidence))
        offset += 3

    end_angle = struct.unpack_from("<H", frame, 42)[0] / 100.0
    timestamp_ms = struct.unpack_from("<H", frame, 44)[0]
    crc_received = frame[46]
    crc_calculated = crc8_ldrobot(frame[:-1])

    return {
        "speed_dps": speed,
        "spin_hz": speed / 360.0,
        "start_angle_deg": start_angle,
        "end_angle_deg": end_angle,
        "timestamp_ms": timestamp_ms,
        "crc_received": crc_received,
        "crc_calculated": crc_calculated,
        "crc_ok": crc_received == crc_calculated,
        "points": points,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--frames", type=int, default=10)
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud ...")
    print("Read-only probe: no commands will be sent to the LiDAR.")

    try:
        with serial.Serial(args.port, args.baud, timeout=2) as ser:
            ser.reset_input_buffer()
            started = time.monotonic()
            crc_errors = 0
            total_valid_points = 0

            for index in range(1, args.frames + 1):
                frame = read_frame(ser)
                info = parse_frame(frame)
                if not info["crc_ok"]:
                    crc_errors += 1

                valid_points = sum(1 for d, c in info["points"] if d > 0 and c > 0)
                total_valid_points += valid_points
                distances = [d for d, _ in info["points"] if d > 0]
                min_distance = min(distances) if distances else 0
                max_distance = max(distances) if distances else 0
                crc_text = (
                    "OK"
                    if info["crc_ok"]
                    else f"BAD rx=0x{info['crc_received']:02X} calc=0x{info['crc_calculated']:02X}"
                )
                print(
                    f"frame={index:02d} "
                    f"speed={info['speed_dps']}deg/s({info['spin_hz']:.2f}Hz) "
                    f"angle={info['start_angle_deg']:.2f}->{info['end_angle_deg']:.2f}deg "
                    f"points={valid_points}/{POINTS_PER_FRAME} "
                    f"distance={min_distance}..{max_distance}mm "
                    f"timestamp={info['timestamp_ms']} "
                    f"crc={crc_text}"
                )

            elapsed = time.monotonic() - started
            if crc_errors:
                print(
                    f"WARNING: received {args.frames} frames in {elapsed:.2f}s, "
                    f"but {crc_errors} frame(s) failed CRC validation.",
                    file=sys.stderr,
                )
                return 3

            print(
                f"OK: received {args.frames} CRC-valid STL-19P/LD19 frames in {elapsed:.2f}s; "
                f"valid points={total_valid_points}/{args.frames * POINTS_PER_FRAME}"
            )
            return 0

    except PermissionError:
        print(
            f"ERROR: permission denied for {args.port}. "
            "Add the user to dialout (sudo usermod -aG dialout $USER) and log in again.",
            file=sys.stderr,
        )
    except serial.SerialException as exc:
        print(f"ERROR: serial port problem: {exc}", file=sys.stderr)
    except TimeoutError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
