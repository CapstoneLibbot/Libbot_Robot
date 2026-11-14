#!/usr/bin/env python3
"""Run the clicked LiBot Gazebo goals and verify position and heading."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from lifecycle_msgs.srv import GetState


GOALS = [
    (2.5307161808013916, -0.3705052137374878),
    (2.144960641860962, -1.505142092704773),
    (-0.06196102872490883, 1.781994342803955),
    (-0.08786439150571823, 2.5784685611724854),
    (1.8821901082992554, 1.7937119007110596),
    (0.13504089415073395, -0.7867247462272644),
]
# Goal 1 lies beyond the lower end of a cross-map shelf, with a second obstacle
# south of that gap. Use the clear corridor between them and approach from the
# south side of the shelf.
APPROACH_WAYPOINTS: dict[int, list[tuple[float, float]]] = {
    1: [(1.5, -0.40), (2.55, -0.30)],
    2: [(2.65, -0.45), (2.65, -1.35)],
    3: [(1.9, 0.9), (1.9, 2.35), (0.5, 2.35)],
}

# These are the spawn coordinates used by the current Gazebo run command.
INITIAL_POSE = (0.807, -0.074, 0.0)
GOAL_XY_TOLERANCE = 0.11
GOAL_YAW_TOLERANCE = 0.35
GOAL_TIMEOUT_SEC = 180.0
GOAL_STALL_TIMEOUT_SEC = 60.0
EXECUTOR_SPIN_PERIOD_SEC = 0.01


def quaternion_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class GoalRunner(Node):
    def __init__(self, ground_truth_localization: bool = False) -> None:
        super().__init__(
            "gazebo_goal_sequence",
            automatically_declare_parameters_from_overrides=True,
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
        )
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.bt_state_client = self.create_client(GetState, "/bt_navigator/get_state")
        self.navigation_lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes",
        )
        self.ground_truth_pose: Pose | None = None
        self.odom_pose: Pose | None = None
        self.odom_stamp = None
        self.ground_truth_localization = ground_truth_localization
        self.tf_broadcaster = (
            TransformBroadcaster(self) if ground_truth_localization else None
        )
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.commands: dict[str, Twist | None] = {
            "/cmd_vel_nav": None,
            "/cmd_vel": None,
            "/cmd_vel_safe": None,
        }
        self.collision_state: CollisionMonitorState | None = None
        self.scan_points_inside_footprint: list[tuple[float, float, float]] = []

        self.create_subscription(Pose, "/gazebo/ground_truth_model_pose", self._on_truth_pose, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        if self.ground_truth_localization:
            self.create_timer(0.01, self._broadcast_truth_transform)
        for topic in self.commands:
            self.create_subscription(Twist, topic, self._command_callback(topic), 10)
        self.create_subscription(
            CollisionMonitorState,
            "/collision_monitor_state",
            self._on_collision_state,
            10,
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)

    def _on_truth_pose(self, msg: Pose) -> None:
        self.ground_truth_pose = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_pose = msg.pose.pose
        self.odom_stamp = msg.header.stamp

    def _broadcast_truth_transform(self) -> None:
        if self.tf_broadcaster is None:
            return
        truth, odom = self.ground_truth_pose, self.odom_pose
        if truth is None or odom is None:
            return

        yaw_delta = angle_error(
            quaternion_yaw(truth.orientation), quaternion_yaw(odom.orientation)
        )
        cos_yaw, sin_yaw = math.cos(yaw_delta), math.sin(yaw_delta)
        transform = TransformStamped()
        # Slightly lead the simulation clock so Nav2's 20 Hz control loop does
        # not query beyond the most recent transform during Gazebo scheduling
        # jitter. The 100 Hz broadcaster keeps earlier samples for scan-time
        # interpolation.
        transform.header.stamp = (
            self.get_clock().now() + rclpy.duration.Duration(seconds=0.1)
        ).to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = truth.position.x - (
            cos_yaw * odom.position.x - sin_yaw * odom.position.y
        )
        transform.transform.translation.y = truth.position.y - (
            sin_yaw * odom.position.x + cos_yaw * odom.position.y
        )
        transform.transform.rotation.z = math.sin(yaw_delta / 2.0)
        transform.transform.rotation.w = math.cos(yaw_delta / 2.0)
        self.tf_broadcaster.sendTransform(transform)

    def _command_callback(self, topic: str):
        def callback(msg: Twist) -> None:
            self.commands[topic] = msg

        return callback

    def _on_collision_state(self, msg: CollisionMonitorState) -> None:
        self.collision_state = msg

    def _on_scan(self, scan: LaserScan) -> None:
        points = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min < distance < scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            x = distance * math.cos(angle) - 0.10
            y = distance * math.sin(angle)
            if abs(x) <= 0.165 and abs(y) <= 0.1825:
                points.append((round(distance, 3), round(x, 3), round(y, 3)))
        self.scan_points_inside_footprint = points

    def pose_error(self, pose: Pose, x: float, y: float, yaw: float) -> tuple[float, float]:
        xy_error = math.hypot(pose.position.x - x, pose.position.y - y)
        yaw_error = abs(angle_error(quaternion_yaw(pose.orientation), yaw))
        return xy_error, yaw_error

    def wait_for_localization(self, timeout_sec: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=EXECUTOR_SPIN_PERIOD_SEC)
            self._broadcast_truth_transform()
            if self.ground_truth_pose is not None and self.odom_pose is not None:
                try:
                    if self.tf_buffer.can_transform("map", "base_footprint", Time()):
                        print(
                            (
                                "Gazebo truth localization is publishing map->odom."
                                if self.ground_truth_localization
                                else "AMCL localization is publishing map->odom."
                            ),
                            flush=True,
                        )
                        return True
                except Exception:
                    pass
        print(
            "Timed out waiting for Gazebo truth and /odom; "
            f"received_truth={self.ground_truth_pose is not None}, "
            f"received_odom={self.odom_pose is not None}.",
            flush=True,
        )
        return False

    def start_navigation(self, timeout_sec: float = 120.0) -> bool:
        # A resumed Gazebo run may already have active Nav2 lifecycle nodes.
        if self.bt_state_client.wait_for_service(timeout_sec=1.0):
            state_future = self.bt_state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, state_future, timeout_sec=2.0)
            state = state_future.result() if state_future.done() else None
            if state is not None and state.current_state.id == 3:
                print("Nav2 is already active.", flush=True)
                return True

        if not self.navigation_lifecycle_client.wait_for_service(timeout_sec=timeout_sec):
            print("Nav2 lifecycle manager service is unavailable.", flush=True)
            return False

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self.navigation_lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result() if future.done() else None
        if response is None or not response.success:
            print("Nav2 lifecycle startup failed.", flush=True)
            return False

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=EXECUTOR_SPIN_PERIOD_SEC)
            if not self.bt_state_client.wait_for_service(timeout_sec=0.1):
                continue
            state_future = self.bt_state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, state_future, timeout_sec=0.2)
            state = state_future.result() if state_future.done() else None
            if state is not None and state.current_state.id == 3:
                print("Nav2 lifecycle startup complete.", flush=True)
                return True
        print("Timed out waiting for bt_navigator to become active.", flush=True)
        return False

    def goal_yaws(self) -> list[float]:
        yaws = []
        for i, (goal_x, goal_y) in enumerate(GOALS):
            if i + 1 < len(GOALS):
                # A clicked point has no orientation. Face along the next
                # requested leg; use the incoming leg for the final point.
                target_x, target_y = GOALS[i + 1]
                dx = target_x - goal_x
                dy = target_y - goal_y
            else:
                approach_x, approach_y = GOALS[i - 1]
                dx = goal_x - approach_x
                dy = goal_y - approach_y
            yaws.append(math.atan2(dy, dx))
        return yaws

    def make_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        # A zero stamp tells Nav2 to transform this map-frame goal using the
        # latest TF. A nonzero stamp goes stale while long actions are running.
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def print_diagnostics(self, goal_x: float | None = None, goal_y: float | None = None) -> None:
        if self.ground_truth_pose is not None:
            pose = self.ground_truth_pose
            details = (
                f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, "
                f"yaw={quaternion_yaw(pose.orientation):.3f}"
            )
            if goal_x is not None and goal_y is not None:
                distance = math.hypot(pose.position.x - goal_x, pose.position.y - goal_y)
                details += f", goal XY error={distance:.3f} m"
            print(f"  Gazebo pose: {details}", flush=True)
        for topic, command in self.commands.items():
            if command is None:
                print(f"  {topic}: no command received", flush=True)
            else:
                print(
                    f"  {topic}: vx={command.linear.x:.3f}, "
                    f"vy={command.linear.y:.3f}, wz={command.angular.z:.3f}",
                    flush=True,
                )
        if self.collision_state is None:
            print("  /collision_monitor_state: no state received", flush=True)
        else:
            print(
                "  /collision_monitor_state: "
                f"polygon={self.collision_state.polygon_name!r}, "
                f"action_type={self.collision_state.action_type}",
                flush=True,
            )
        print(
            f"  LiDAR returns inside Collision Monitor footprint: "
            f"{self.scan_points_inside_footprint}",
            flush=True,
        )

    def navigate_one(
        self,
        index: int,
        x: float,
        y: float,
        yaw: float,
        verify_x: float | None = None,
        verify_y: float | None = None,
        verify_heading: bool = True,
        verify_xy_tolerance: float = GOAL_XY_TOLERANCE,
    ) -> bool:
        verify_x = x if verify_x is None else verify_x
        verify_y = y if verify_y is None else verify_y
        print(
            f"Goal {index}/{len(GOALS)}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}", flush=True
        )

        last_feedback_at = 0.0
        best_distance_remaining = math.inf
        last_progress_at = time.monotonic()

        def on_feedback(msg) -> None:
            nonlocal last_feedback_at, best_distance_remaining, last_progress_at
            now = time.monotonic()
            remaining = msg.feedback.distance_remaining
            if remaining < best_distance_remaining - 0.03:
                best_distance_remaining = remaining
                last_progress_at = now
            if now - last_feedback_at >= 2.0:
                print(
                    f"  distance remaining: {remaining:.2f} m",
                    flush=True,
                )
                last_feedback_at = now

        send_future = self.action_client.send_goal_async(
            self.make_goal(x, y, yaw),
            feedback_callback=on_feedback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print(f"Goal {index} rejected by Nav2.", flush=True)
            self.print_diagnostics(x, y)
            return False

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + GOAL_TIMEOUT_SEC
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=EXECUTOR_SPIN_PERIOD_SEC)
            stalled = time.monotonic() - last_progress_at >= GOAL_STALL_TIMEOUT_SEC
            truth_pose_is_at_goal = False
            if self.ground_truth_pose is not None:
                truth_xy_error, truth_yaw_error = self.pose_error(
                    self.ground_truth_pose, verify_x, verify_y, yaw
                )
                truth_pose_is_at_goal = (
                    truth_xy_error <= GOAL_XY_TOLERANCE
                    and (
                        not verify_heading
                        or truth_yaw_error <= GOAL_YAW_TOLERANCE
                    )
                )
            if stalled and not truth_pose_is_at_goal:
                print(
                    f"Goal {index} made no 3 cm progress for "
                    f"{GOAL_STALL_TIMEOUT_SEC:.0f} s; cancelling it.",
                    flush=True,
                )
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                self.print_diagnostics(verify_x, verify_y)
                return False

        if not result_future.done():
            print(f"Goal {index} timed out; cancelling it.", flush=True)
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            self.print_diagnostics(x, y)
            return False

        wrapped_result = result_future.result()
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            print(
                f"Goal {index} failed with action status {wrapped_result.status}.",
                flush=True,
            )
            self.print_diagnostics()
            return False

        truth_deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < truth_deadline:
            rclpy.spin_once(self, timeout_sec=EXECUTOR_SPIN_PERIOD_SEC)
            if self.ground_truth_pose is None:
                continue
            xy_error, yaw_error = self.pose_error(
                self.ground_truth_pose, verify_x, verify_y, yaw
            )
            if xy_error <= verify_xy_tolerance and (
                not verify_heading or yaw_error <= GOAL_YAW_TOLERANCE
            ):
                print(
                    f"Goal {index} SUCCEEDED; Gazebo error: "
                    f"XY={xy_error:.3f} m, yaw={yaw_error:.3f} rad",
                    flush=True,
                )
                return True

        print(
            f"Goal {index} action succeeded but Gazebo pose missed tolerance "
            f"(XY <= {verify_xy_tolerance:.2f} m, "
            f"yaw <= {GOAL_YAW_TOLERANCE:.2f} rad).",
            flush=True,
        )
        if self.ground_truth_pose is not None:
            xy_error, yaw_error = self.pose_error(
                self.ground_truth_pose, verify_x, verify_y, yaw
            )
            print(f"  Gazebo error: XY={xy_error:.3f} m, yaw={yaw_error:.3f} rad", flush=True)
        self.print_diagnostics(verify_x, verify_y)
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the clicked-point Gazebo goal sequence.")
    parser.add_argument(
        "--start-index", type=int, choices=range(1, len(GOALS) + 1), default=1,
        help="resume at this one-based clicked-point index after a Gazebo runner interruption",
    )
    parser.add_argument(
        "--ground-truth-localization",
        action="store_true",
        help="publish map->odom from Gazebo truth for an exact-pose validation run",
    )
    parser.add_argument(
        "--nav2-already-active",
        action="store_true",
        help="reuse an already active Nav2 lifecycle stack after a runner-only restart",
    )
    args = parser.parse_args()

    rclpy.init()
    runner = GoalRunner(ground_truth_localization=args.ground_truth_localization)
    try:
        # Let simulation_navigation's AMCL publish map->odom from the configured
        # initial pose and scan before activating the Nav2 navigation stack.
        if not runner.wait_for_localization():
            return 2
        if (
            args.start_index == 1
            and not args.nav2_already_active
            and not runner.start_navigation()
        ):
            return 2
        if args.nav2_already_active:
            print("Reusing the existing active Nav2 session.", flush=True)
        elif args.start_index > 1:
            print("Resuming with the existing active Gazebo/Nav2 session.", flush=True)
        if not runner.action_client.wait_for_server(timeout_sec=30.0):
            print("/navigate_to_pose action server is unavailable.", flush=True)
            return 2

        yaws = runner.goal_yaws()
        for index, ((x, y), yaw) in enumerate(zip(GOALS, yaws), start=1):
            if index < args.start_index:
                continue
            approaches = APPROACH_WAYPOINTS.get(index, [])
            if runner.ground_truth_pose is not None:
                current = runner.ground_truth_pose.position
                if math.hypot(current.x - x, current.y - y) <= 0.5:
                    print(
                        f"Already near goal {index}; resuming without its approach route.",
                        flush=True,
                    )
                    approaches = []
                elif index == 1 and 1.5 <= current.x <= 2.4 and current.y <= -0.20:
                    print(
                        "Already inside the clear shelf corridor; continuing to goal 1 directly.",
                        flush=True,
                    )
                    approaches = []
            previous_x = runner.ground_truth_pose.position.x if runner.ground_truth_pose else INITIAL_POSE[0]
            previous_y = runner.ground_truth_pose.position.y if runner.ground_truth_pose else INITIAL_POSE[1]
            for approach_number, approach in enumerate(approaches, start=1):
                if approach_number == 1 and runner.ground_truth_pose is not None:
                    # Keep the current orientation while escaping a close
                    # obstacle; the mecanum controller can translate sideways.
                    approach_yaw = quaternion_yaw(
                        runner.ground_truth_pose.orientation
                    )
                else:
                    approach_yaw = math.atan2(
                        approach[1] - previous_y, approach[0] - previous_x
                    )
                if runner.navigate_one(
                    f"{index}-approach-{approach_number}",
                    approach[0],
                    approach[1],
                    approach_yaw,
                    verify_heading=False,
                    verify_xy_tolerance=0.35,
                ) is False:
                    print(
                        f"Goal {index} approach waypoint failed; sequence not successful.",
                        flush=True,
                    )
                    return 1
                previous_x, previous_y = approach
            if not runner.navigate_one(
                index,
                x,
                y,
                yaw,
                verify_x=x,
                verify_y=y,
            ):
                print(f"Sequence stopped at goal {index}; not successful.", flush=True)
                return 1

        if args.start_index == 1:
            print(f"ALL {len(GOALS)} GOALS SUCCEEDED in Nav2 and Gazebo pose checks.", flush=True)
        else:
            print(
                f"GOALS {args.start_index}-{len(GOALS)} SUCCEEDED in Nav2 and Gazebo pose checks.",
                flush=True,
            )
        return 0
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
