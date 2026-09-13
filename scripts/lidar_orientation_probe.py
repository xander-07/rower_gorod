#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarOrientationProbe(Node):
    def __init__(self) -> None:
        super().__init__('rower_lidar_orientation_probe')
        self.scans: list[LaserScan] = []
        self.sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scans.append(msg)


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_distance(a: float, b: float) -> float:
    return abs(wrap_pi(a - b))


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    values = sorted(values)
    idx = min(len(values) - 1, max(0, int(round((len(values) - 1) * q))))
    return values[idx]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Stationary LiDAR orientation check. Place the robot facing a flat wall; '
            'the script reports which base-relative direction contains the closest wall.'
        )
    )
    parser.add_argument('--seconds', type=float, default=3.0, help='Collection time, 1..10 s.')
    parser.add_argument(
        '--lidar-yaw-deg',
        type=float,
        default=90.0,
        help='Current base_link->laser yaw assumption in degrees (default: +90).',
    )
    parser.add_argument('--sector-deg', type=float, default=20.0, help='Half-width of each direction sector.')
    args = parser.parse_args()

    if not (1.0 <= args.seconds <= 10.0):
        print('ERROR: --seconds must be between 1 and 10.')
        return 2
    if not (5.0 <= args.sector_deg <= 45.0):
        print('ERROR: --sector-deg must be between 5 and 45 degrees.')
        return 2

    print('STATIONARY LIDAR ORIENTATION PROBE')
    print('Robot MUST stay still. Motion is not commanded by this script.')
    print('Place a broad flat wall about 0.4..0.8 m directly in front of the robot.')
    print('Keep left/right/back as clear as practical (>1.0 m is ideal).')
    print(f'Using current lidar yaw assumption: {args.lidar_yaw_deg:+.1f} deg')
    print(f'Collecting /scan for {args.seconds:.1f} s...')

    rclpy.init()
    node = LidarOrientationProbe()
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if not node.scans:
            print('ERROR: no /scan messages received.')
            return 3

        yaw = math.radians(args.lidar_yaw_deg)
        half = math.radians(args.sector_deg)
        directions = {
            'FRONT': 0.0,
            'LEFT': math.pi / 2.0,
            'BACK': math.pi,
            'RIGHT': -math.pi / 2.0,
        }
        buckets: dict[str, list[float]] = {name: [] for name in directions}
        nearest_range = math.inf
        nearest_base_angle = math.nan

        for scan in node.scans:
            angle = float(scan.angle_min)
            rmin = max(0.05, float(scan.range_min))
            rmax = float(scan.range_max)
            for r in scan.ranges:
                rr = float(r)
                if math.isfinite(rr) and rmin <= rr <= rmax:
                    base_angle = wrap_pi(angle + yaw)
                    if rr < nearest_range:
                        nearest_range = rr
                        nearest_base_angle = base_angle
                    for name, center in directions.items():
                        if angular_distance(base_angle, center) <= half:
                            buckets[name].append(rr)
                angle += float(scan.angle_increment)

        print(f'SCANS: n={len(node.scans)}')
        for name in ('FRONT', 'LEFT', 'BACK', 'RIGHT'):
            vals = buckets[name]
            if not vals:
                print(f'{name}: no valid points')
                continue
            print(
                f'{name}: p10={percentile(vals, 0.10):.3f}m '
                f'median={statistics.median(vals):.3f}m n={len(vals)}'
            )

        if math.isfinite(nearest_range):
            print(
                'NEAREST: '
                f'range={nearest_range:.3f}m '
                f'base_angle={math.degrees(nearest_base_angle):+.1f}deg '
                '(0=front, +90=left, -90=right, +/-180=back)'
            )
        else:
            print('NEAREST: unavailable')
            return 3

        front_p10 = percentile(buckets['FRONT'], 0.10)
        side_candidates = [
            (name, percentile(vals, 0.10))
            for name, vals in buckets.items()
            if vals
        ]
        side_candidates = [(n, v) for n, v in side_candidates if math.isfinite(v)]
        if side_candidates:
            closest_name, closest_value = min(side_candidates, key=lambda item: item[1])
            print(f'CLASSIFICATION: closest_sector={closest_name} p10={closest_value:.3f}m')
            if closest_name == 'FRONT':
                print('RESULT: current +90 deg LiDAR yaw is CONSISTENT with a wall placed in front.')
            else:
                print(
                    'RESULT: current LiDAR yaw is NOT CONSISTENT with the test scene. '
                    'Do not change URDF yet; send this output so the required correction can be computed.'
                )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
