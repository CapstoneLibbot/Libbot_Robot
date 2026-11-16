#!/usr/bin/env python3
"""Short Gazebo-only probe for mecanum wheel traction and direction."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MecanumProbe(Node):
    def __init__(self) -> None:
        super().__init__(
            "mecanum_drive_probe",
            automatically_declare_parameters_from_overrides=True,
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
        )
        self.pose: Pose | None = None
        self.odom: Pose | None = None
        # Inject upstream of Collision Monitor so the same safety path used by Nav2 is active.
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Pose, "/gazebo/ground_truth_model_pose", self._on_pose, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_pose(self, msg: Pose) -> None:
        self.pose = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg.pose.pose

    def yaw(self, pose: Pose) -> float:
        q = pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def publish_for(self, command: Twist, duration: float) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.publisher.publish(Twist())
        # Allow the command pipeline and the body to settle before sampling.
        settle_deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < settle_deadline:
            self.publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def main() -> int:
    rclpy.init()
    node = MecanumProbe()
    try:
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and (node.pose is None or node.odom is None) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose is None or node.odom is None:
            print("No Gazebo pose or odometry received.", flush=True)
            return 2

        for label, vx, vy, wz in (("forward", 0.08, 0.0, 0.0), ("lateral", 0.0, 0.08, 0.0), ("rotate", 0.0, 0.0, 0.25)):
            node.publish_for(Twist(), 0.2)
            start_truth, start_odom = node.pose, node.odom
            command = Twist()
            command.linear.x, command.linear.y, command.angular.z = vx, vy, wz
            node.publish_for(command, 0.75)
            end_truth, end_odom = node.pose, node.odom
            if end_truth is None or end_odom is None:
                print("Gazebo pose or odometry disappeared during the probe.", flush=True)
                return 2
            truth_yaw = math.atan2(math.sin(node.yaw(end_truth) - node.yaw(start_truth)), math.cos(node.yaw(end_truth) - node.yaw(start_truth)))
            odom_yaw = math.atan2(math.sin(node.yaw(end_odom) - node.yaw(start_odom)), math.cos(node.yaw(end_odom) - node.yaw(start_odom)))
            print(
                f"{label}: truth dx={end_truth.position.x - start_truth.position.x:.4f} "
                f"dy={end_truth.position.y - start_truth.position.y:.4f} dYaw={truth_yaw:.4f}; "
                f"odom dx={end_odom.position.x - start_odom.position.x:.4f} "
                f"dy={end_odom.position.y - start_odom.position.y:.4f} dYaw={odom_yaw:.4f}",
                flush=True,
            )
        return 0
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
