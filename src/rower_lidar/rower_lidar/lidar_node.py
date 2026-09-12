#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import serial


FRAME_LEN = 47
HEADER = b'\x54\x2c'
POINTS_PER_PACKET = 12


def crc8_ldrobot(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


class RowerLidar(Node):
    """STL-19P / LD19-family serial driver publishing sensor_msgs/LaserScan."""

    def __init__(self) -> None:
        super().__init__('rower_lidar')

        self.declare_parameter('serial_port', '/dev/rower_lidar')
        self.declare_parameter('baud', 230400)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 25.0)
        self.declare_parameter('min_confidence', 0)
        self.declare_parameter('bins', 480)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.min_confidence = int(self.get_parameter('min_confidence').value)
        self.bins = int(self.get_parameter('bins').value)

        if self.bins < 90:
            raise ValueError('bins must be >= 90')
        if not (0 <= self.min_confidence <= 255):
            raise ValueError('min_confidence must be in 0..255')

        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.ser = serial.Serial(self.serial_port, self.baud, timeout=0)
        self.ser.reset_input_buffer()

        self.rx = bytearray()
        self.ranges = [math.inf] * self.bins
        self.intensities = [0.0] * self.bins
        self.last_angle = None
        self.last_wrap_time = None
        self.valid_points_this_scan = 0
        self.scans = 0
        self.frames = 0
        self.crc_errors = 0
        self.last_stats = time.monotonic()

        self.create_timer(0.005, self._poll_serial)
        self.get_logger().info(
            f'Opened STL-19P on {self.serial_port} at {self.baud} baud; '
            f'publishing /scan with {self.bins} bins, frame={self.frame_id}'
        )

    @staticmethod
    def _u16le(buf: bytes, offset: int) -> int:
        return buf[offset] | (buf[offset + 1] << 8)

    def _poll_serial(self) -> None:
        waiting = self.ser.in_waiting
        if waiting > 0:
            self.rx.extend(self.ser.read(waiting))

        while True:
            idx = self.rx.find(HEADER)
            if idx < 0:
                if len(self.rx) > 1:
                    del self.rx[:-1]
                break
            if idx > 0:
                del self.rx[:idx]
            if len(self.rx) < FRAME_LEN:
                break

            frame = bytes(self.rx[:FRAME_LEN])
            if crc8_ldrobot(frame[:46]) != frame[46]:
                self.crc_errors += 1
                del self.rx[0]
                continue

            del self.rx[:FRAME_LEN]
            self.frames += 1
            self._handle_frame(frame)

        now = time.monotonic()
        if now - self.last_stats >= 5.0:
            self.get_logger().info(
                f'LiDAR stats: scans={self.scans} frames={self.frames} crc_errors={self.crc_errors}'
            )
            self.last_stats = now

    def _handle_frame(self, frame: bytes) -> None:
        start_deg = self._u16le(frame, 4) / 100.0
        end_deg = self._u16le(frame, 42) / 100.0
        delta = (end_deg - start_deg) % 360.0

        for i in range(POINTS_PER_PACKET):
            angle = (start_deg + delta * i / (POINTS_PER_PACKET - 1)) % 360.0
            offset = 6 + i * 3
            distance_mm = self._u16le(frame, offset)
            confidence = frame[offset + 2]

            if self.last_angle is not None and self.last_angle > 300.0 and angle < 60.0:
                self._publish_scan()
                self.ranges = [math.inf] * self.bins
                self.intensities = [0.0] * self.bins
                self.valid_points_this_scan = 0

            self.last_angle = angle
            index = int((angle / 360.0) * self.bins) % self.bins

            if distance_mm <= 0 or confidence < self.min_confidence:
                continue
            distance_m = distance_mm / 1000.0
            if not (self.range_min <= distance_m <= self.range_max):
                continue

            # If multiple raw points land in one angular bin, keep the nearest.
            if distance_m < self.ranges[index]:
                self.ranges[index] = distance_m
                self.intensities[index] = float(confidence)
            self.valid_points_this_scan += 1

    def _publish_scan(self) -> None:
        if self.valid_points_this_scan < 50:
            self.last_wrap_time = time.monotonic()
            return

        now_mono = time.monotonic()
        if self.last_wrap_time is None:
            scan_time = 0.1
        else:
            scan_time = max(0.01, min(0.5, now_mono - self.last_wrap_time))
        self.last_wrap_time = now_mono

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_increment = (2.0 * math.pi) / self.bins
        msg.angle_max = msg.angle_min + msg.angle_increment * (self.bins - 1)
        msg.scan_time = scan_time
        msg.time_increment = scan_time / self.bins
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = list(self.ranges)
        msg.intensities = list(self.intensities)
        self.pub.publish(msg)
        self.scans += 1

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RowerLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
