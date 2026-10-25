from __future__ import annotations

import math
import os
import threading
import time
import uuid
from typing import Any, Dict, Optional


class NavigationController:
    """Send navigation goals to Nav2."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        self.settings = settings or {}
        self.runtime_id = os.getenv(
            "LIBOT_RUNTIME_ID",
            str(self.settings.get("runtime_id", "real")),
        ).strip().lower()
        self.use_sim_time = self.runtime_id == "sim"
        self.action_name = self.settings.get(
            "action_name",
            "/navigate_to_pose",
        )
        self._goal_handle = None
        self._goal_lock = threading.RLock()

    def navigate(self, goal: Dict[str, Any]) -> Dict[str, Any]:
        started_at = time.monotonic()
        target = goal.get("target") or {}
        timeout_sec = float(goal.get("timeout_sec") or 60.0)

        if "x" not in target or "y" not in target:
            return self._result(
                "FAILED",
                "MISSING_TARGET_POSE",
                started_at,
                "MOVE requires target.x and target.y.",
            )

        try:
            import rclpy
            from action_msgs.msg import GoalStatus
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            from rclpy.parameter import Parameter
        except ImportError as exc:
            return self._result(
                "FAILED",
                "ROS_UNAVAILABLE",
                started_at,
                str(exc),
            )

        x = float(target["x"])
        y = float(target["y"])
        theta = float(target.get("theta", 0.0))

        if not all(math.isfinite(value) for value in (x, y, theta)):
            return self._result(
                "FAILED",
                "INVALID_TARGET_POSE",
                started_at,
                "x, y and theta must be finite numbers.",
            )

        if not rclpy.ok():
            rclpy.init(args=None)

        node = None
        try:
            node = rclpy.create_node(
                f"libot_navigation_{uuid.uuid4().hex[:8]}",
                parameter_overrides=[
                    Parameter("use_sim_time", value=self.use_sim_time)
                ],
            )

            if self.use_sim_time and not self._wait_for_sim_clock(node, rclpy):
                return self._result(
                    "FAILED",
                    "SIM_CLOCK_UNAVAILABLE",
                    started_at,
                    "No nonzero ROS simulated time was received on /clock within 10 seconds.",
                )

            action_client = ActionClient(
                node,
                NavigateToPose,
                self.action_name,
            )

            if not action_client.wait_for_server(timeout_sec=10.0):
                return self._result(
                    "FAILED",
                    "NAV2_UNAVAILABLE",
                    started_at,
                    f"Action server {self.action_name} is unavailable.",
                )

            action_goal = NavigateToPose.Goal()
            action_goal.pose.header.frame_id = "map"
            action_goal.pose.header.stamp = node.get_clock().now().to_msg()

            action_goal.pose.pose.position.x = x
            action_goal.pose.pose.position.y = y
            action_goal.pose.pose.position.z = 0.0

            action_goal.pose.pose.orientation.x = 0.0
            action_goal.pose.pose.orientation.y = 0.0
            action_goal.pose.pose.orientation.z = math.sin(theta / 2.0)
            action_goal.pose.pose.orientation.w = math.cos(theta / 2.0)

            last_feedback_at = [0.0]

            def report_feedback(feedback_message: Any) -> None:
                detail = getattr(feedback_message, "feedback", None)
                distance = getattr(detail, "distance_remaining", None)
                try:
                    distance = float(distance)
                    if not math.isfinite(distance):
                        return
                except (TypeError, ValueError, OverflowError):
                    return
                now = time.monotonic()
                if now - last_feedback_at[0] >= 1.0:
                    print(f"Nav2 distance remaining: {distance:.2f} m", flush=True)
                    last_feedback_at[0] = now

            send_future = action_client.send_goal_async(
                action_goal,
                feedback_callback=report_feedback,
            )

            rclpy.spin_until_future_complete(
                node,
                send_future,
                timeout_sec=10.0,
            )

            if not send_future.done():
                return self._result(
                    "FAILED",
                    "GOAL_SEND_TIMEOUT",
                    started_at,
                    "Timed out while sending navigation goal.",
                )

            goal_handle = send_future.result()

            if goal_handle is None or not goal_handle.accepted:
                return self._result(
                    "FAILED",
                    "GOAL_REJECTED",
                    started_at,
                    "Nav2 rejected the navigation goal.",
                )

            with self._goal_lock:
                self._goal_handle = goal_handle

            result_future = goal_handle.get_result_async()

            rclpy.spin_until_future_complete(
                node,
                result_future,
                timeout_sec=timeout_sec,
            )

            if not result_future.done():
                cancel_future = goal_handle.cancel_goal_async()

                rclpy.spin_until_future_complete(
                    node,
                    cancel_future,
                    timeout_sec=5.0,
                )

                # Keep spinning for the action's terminal result after requesting
                # cancellation; the cancel response alone does not finish the goal.
                rclpy.spin_until_future_complete(
                    node,
                    result_future,
                    timeout_sec=5.0,
                )
                if result_future.done():
                    terminal_status = result_future.result().status
                    terminal_name = {
                        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                        GoalStatus.STATUS_CANCELED: "CANCELED",
                        GoalStatus.STATUS_ABORTED: "ABORTED",
                    }.get(terminal_status, f"STATUS_{terminal_status}")
                    detail = f" Terminal Nav2 result after cancellation: {terminal_name}."
                    result_code = "TIMEOUT"
                else:
                    detail = " Cancellation was requested, but no terminal Nav2 result arrived within 5 seconds."
                    result_code = "TIMEOUT_CANCEL_RESULT_PENDING"

                return self._result(
                    "FAILED",
                    result_code,
                    started_at,
                    f"Navigation exceeded {timeout_sec} seconds.{detail}",
                )

            wrapped_result = result_future.result()
            action_status = wrapped_result.status

            if action_status == GoalStatus.STATUS_SUCCEEDED:
                return {
                    "move_status": "COMPLETED",
                    "final_pose": None,
                    "nav2_result_code": "SUCCEEDED",
                    "duration_ms": int(
                        (time.monotonic() - started_at) * 1000
                    ),
                }

            if action_status == GoalStatus.STATUS_CANCELED:
                result_code = "CANCELED"
            elif action_status == GoalStatus.STATUS_ABORTED:
                result_code = "ABORTED"
            else:
                result_code = f"STATUS_{action_status}"

            return self._result(
                "FAILED",
                result_code,
                started_at,
                "Nav2 did not complete the goal.",
            )

        except Exception as exc:
            return self._result(
                "FAILED",
                "NAVIGATION_EXCEPTION",
                started_at,
                str(exc),
            )
        finally:
            with self._goal_lock:
                self._goal_handle = None
            if node is not None:
                node.destroy_node()

    @staticmethod
    def _wait_for_sim_clock(node: Any, rclpy: Any, timeout_sec: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if node.get_clock().now().nanoseconds > 0:
                return True
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
        return node.get_clock().now().nanoseconds > 0

    def stop(self) -> None:
        with self._goal_lock:
            goal_handle = self._goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass

    def _result(
        self,
        move_status: str,
        result_code: str,
        started_at: float,
        message: Optional[str] = None,
    ) -> Dict[str, Any]:
        result = {
            "move_status": move_status,
            "final_pose": None,
            "nav2_result_code": result_code,
            "duration_ms": int(
                (time.monotonic() - started_at) * 1000
            ),
        }

        if message:
            result["message"] = message

        return result
