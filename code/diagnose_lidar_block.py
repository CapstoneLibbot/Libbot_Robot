#!/usr/bin/env python3
"""Report LiDAR returns that fall inside the robot footprint."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanDiagnostic(Node):
    def __init__(self):
        super().__init__("scan_diagnostic", automatically_declare_parameters_from_overrides=True)
        self.create_subscription(LaserScan, "/scan", self._scan, 10)

    def _scan(self, scan):
        inside = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min < distance < scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            x = distance * math.cos(angle) - 0.10  # Laser is 10 cm behind base_link.
            y = distance * math.sin(angle)
            if abs(x) <= 0.165 and abs(y) <= 0.1825:
                inside.append((round(distance, 3), round(x, 3), round(y, 3)))
        print(f"valid scan points inside footprint: {inside}", flush=True)
        raise SystemExit


rclpy.init()
node = ScanDiagnostic()
try:
    rclpy.spin(node)
except SystemExit:
    pass
node.destroy_node()
rclpy.shutdown()
