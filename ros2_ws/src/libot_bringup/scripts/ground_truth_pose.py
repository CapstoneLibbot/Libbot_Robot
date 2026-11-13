#!/usr/bin/env python3
"""Publish Gazebo's physical robot pose as a PoseStamped for RViz comparison."""

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node


class GroundTruthPose(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_ground_truth_pose")
        self.publisher = self.create_publisher(
            PoseStamped, "/gazebo/ground_truth_pose", 10
        )
        self.create_subscription(
            Pose, "/gazebo/ground_truth_model_pose", self.on_pose, 10
        )
        self.get_logger().info(
            "Publishing the Gazebo libot model pose on "
            "/gazebo/ground_truth_pose in the map frame"
        )

    def on_pose(self, model_pose: Pose) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        # The map-generated world uses Nav2 map coordinates as world coordinates.
        pose.header.frame_id = "map"
        pose.pose = model_pose
        self.publisher.publish(pose)


def main() -> None:
    rclpy.init()
    node = GroundTruthPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
