from __future__ import annotations

import json
import math
import os
import threading
import time
from datetime import datetime, timezone
from typing import Any, Dict, Optional
from urllib import request


class RobotRuntimeGateway:
    """ROS-facing runtime adapter used by both Gazebo and the physical robot."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        config = settings or {}
        runtime = config.get("runtime", {})
        self.runtime_id = os.getenv("LIBOT_RUNTIME_ID", runtime.get("id", "real"))
        self.backend_url = os.getenv(
            "LIBOT_BACKEND_URL",
            runtime.get("backend_url", "http://127.0.0.1:5205"),
        ).rstrip("/")
        self.telemetry_hz = max(1.0, min(float(runtime.get("telemetry_hz", 5.0)), 10.0))
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.twist = {"linearX": 0.0, "linearY": 0.0, "angularZ": 0.0}
        self.battery: Optional[float] = None
        self.localization_state = "UNLOCALIZED"
        self.navigation_state = "IDLE"
        self.active_goal_id: Optional[str] = None
        self.planned_path: list[Dict[str, float]] = []
        self.error: Optional[str] = None
        self._node: Any = None
        self._executor: Any = None
        self._action_client: Any = None
        self._goal_handle: Any = None
        self._goal_timeout_timer: Optional[threading.Timer] = None
        self._initial_pose_pub: Any = None
        self._stop_pub: Any = None
        self._lock = threading.RLock()
        self._running = False

    def start(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
            from nav2_msgs.action import NavigateToPose
            from nav_msgs.msg import Odometry, Path
            from rclpy.action import ActionClient
            from rclpy.executors import MultiThreadedExecutor
            from sensor_msgs.msg import BatteryState
        except ImportError as exc:
            self.error = f"ROS_UNAVAILABLE: {exc}"
            return

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(f"libot_runtime_gateway_{self.runtime_id}")
        self._initial_pose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._stop_pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
        self._node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
        )
        self._node.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self._node.create_subscription(Path, "/plan", self._on_plan, 10)
        self._node.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        self._action_client = ActionClient(self._node, NavigateToPose, "/navigate_to_pose")
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._running = True
        threading.Thread(target=self._executor.spin, daemon=True).start()
        threading.Thread(target=self._telemetry_loop, daemon=True).start()

    def stop(self) -> None:
        self._running = False
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.0)
        if self._node is not None:
            self._node.destroy_node()

    def handle_request(self, payload: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(payload, dict):
            return None
        command = str(payload.get("command", "")).lower()
        if command == "set_initial_pose":
            return self._set_initial_pose(payload.get("pose") or {})
        if command == "start_navigation":
            return self._start_navigation(payload)
        if command == "cancel_navigation":
            return self._cancel_navigation(str(payload.get("command_id", "")))
        return None

    def _set_initial_pose(self, pose: Dict[str, Any]) -> Dict[str, Any]:
        if self._node is None or self._initial_pose_pub is None:
            return self._failure(self.error or "ROS is unavailable.")
        values = self._pose_values(pose)
        if values is None:
            return self._failure("Initial pose requires finite x, y and yaw values.")
        from geometry_msgs.msg import PoseWithCovarianceStamped

        x, y, yaw = values
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        self._initial_pose_pub.publish(message)
        with self._lock:
            self.localization_state = "LOCALIZING"
            self.error = None
        return {"status": "accepted", "localization_state": "LOCALIZING"}

    def _start_navigation(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        target = payload.get("target") or {}
        values = self._pose_values(target)
        if values is None:
            return self._failure("Navigation target requires finite x, y and theta values.")
        if self._node is None or self._action_client is None:
            return self._failure(self.error or "ROS is unavailable.")
        with self._lock:
            if self.navigation_state in {"SENDING", "NAVIGATING", "CANCELING"}:
                return self._failure("A navigation goal is already active.")
            self.active_goal_id = str(payload.get("command_id") or "")
            self.navigation_state = "SENDING"
            self.error = None

        if not self._action_client.server_is_ready():
            self._action_client.wait_for_server(timeout_sec=2.0)
        if not self._action_client.server_is_ready():
            with self._lock:
                self.navigation_state = "FAILED"
                self.error = "NAV2_UNAVAILABLE"
            return self._failure("Nav2 action server is unavailable.")

        from nav2_msgs.action import NavigateToPose

        x, y, yaw = values
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        future = self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)
        timeout_sec = max(1.0, min(float(payload.get("timeout_sec", 180)), 600.0))
        self._goal_timeout_timer = threading.Timer(
            timeout_sec,
            self._on_goal_timeout,
            args=(self.active_goal_id,),
        )
        self._goal_timeout_timer.daemon = True
        self._goal_timeout_timer.start()
        return {"status": "accepted", "command_id": self.active_goal_id}

    def _cancel_navigation(self, command_id: str) -> Dict[str, Any]:
        with self._lock:
            if command_id and self.active_goal_id and command_id != self.active_goal_id:
                return self._failure("The requested command is not the active goal.")
            goal_handle = self._goal_handle
            self.navigation_state = "CANCELING"
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        if self._goal_timeout_timer is not None:
            self._goal_timeout_timer.cancel()
        self._publish_stop()
        return {"status": "accepted", "command_id": self.active_goal_id}

    def _on_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                if self._goal_timeout_timer is not None:
                    self._goal_timeout_timer.cancel()
                with self._lock:
                    self.navigation_state = "REJECTED"
                    self.error = "GOAL_REJECTED"
                    self.active_goal_id = None
                return
            with self._lock:
                self._goal_handle = goal_handle
                self.navigation_state = "NAVIGATING"
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_navigation_result)
        except Exception as exc:
            if self._goal_timeout_timer is not None:
                self._goal_timeout_timer.cancel()
            with self._lock:
                self.navigation_state = "FAILED"
                self.error = str(exc)
                self.active_goal_id = None

    def _on_navigation_result(self, future: Any) -> None:
        if self._goal_timeout_timer is not None:
            self._goal_timeout_timer.cancel()
        try:
            status = int(future.result().status)
            state = {4: "SUCCEEDED", 5: "CANCELED", 6: "FAILED"}.get(status, f"STATUS_{status}")
        except Exception as exc:
            state = "FAILED"
            self.error = str(exc)
        with self._lock:
            self.navigation_state = state
            self._goal_handle = None
            if state in {"SUCCEEDED", "CANCELED", "FAILED"}:
                self.active_goal_id = None

    def _on_goal_timeout(self, command_id: Optional[str]) -> None:
        with self._lock:
            if not command_id or self.active_goal_id != command_id:
                return
            goal_handle = self._goal_handle
            self.navigation_state = "FAILED"
            self.error = "TIMEOUT"
            self.active_goal_id = None
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        self._publish_stop()

    def _on_feedback(self, _: Any) -> None:
        with self._lock:
            self.navigation_state = "NAVIGATING"

    def _on_amcl_pose(self, message: Any) -> None:
        pose = message.pose.pose
        with self._lock:
            self.pose = self._message_pose(pose)
            self.localization_state = "LOCALIZED"

    def _on_odom(self, message: Any) -> None:
        twist = message.twist.twist
        with self._lock:
            self.twist = {
                "linearX": float(twist.linear.x),
                "linearY": float(twist.linear.y),
                "angularZ": float(twist.angular.z),
            }

    def _on_plan(self, message: Any) -> None:
        points = [{"x": float(p.pose.position.x), "y": float(p.pose.position.y)} for p in message.poses]
        with self._lock:
            self.planned_path = points[:500]

    def _on_battery(self, message: Any) -> None:
        percentage = float(message.percentage)
        with self._lock:
            self.battery = percentage * 100.0 if 0.0 <= percentage <= 1.0 else percentage

    def _telemetry_loop(self) -> None:
        interval = 1.0 / self.telemetry_hz
        while self._running:
            started = time.monotonic()
            try:
                self._post_telemetry()
            except OSError:
                pass
            time.sleep(max(0.0, interval - (time.monotonic() - started)))

    def _post_telemetry(self) -> None:
        with self._lock:
            payload = {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "pose": dict(self.pose),
                "twist": dict(self.twist),
                "battery": self.battery,
                "localizationState": self.localization_state,
                "navigationState": self.navigation_state,
                "activeGoalId": self.active_goal_id,
                "plannedPath": list(self.planned_path),
                "error": self.error,
            }
        body = json.dumps(payload).encode("utf-8")
        req = request.Request(
            f"{self.backend_url}/api/robot-runtimes/{self.runtime_id}/telemetry",
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with request.urlopen(req, timeout=2.0):
            pass

    def _publish_stop(self) -> None:
        if self._stop_pub is None:
            return
        from geometry_msgs.msg import Twist

        stop_message = Twist()
        for _ in range(3):
            self._stop_pub.publish(stop_message)

    @staticmethod
    def _pose_values(pose: Dict[str, Any]) -> Optional[tuple[float, float, float]]:
        try:
            values = (float(pose["x"]), float(pose["y"]), float(pose.get("yaw", pose.get("theta", 0.0))))
            return values if all(math.isfinite(value) for value in values) else None
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def _message_pose(pose: Any) -> Dict[str, float]:
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        return {"x": float(pose.position.x), "y": float(pose.position.y), "yaw": yaw}

    @staticmethod
    def _failure(message: str) -> Dict[str, Any]:
        return {"status": "error", "message": message}

