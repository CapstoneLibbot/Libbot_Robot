from __future__ import annotations

import math
import time
import uuid
from typing import Any, Dict, Optional


class NavigationController:
    """Send navigation goals to Nav2."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        self.settings = settings or {}
        self.action_name = self.settings.get(
            "action_name",
            "/navigate_to_pose",
        )

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

        node = rclpy.create_node(
            f"libot_navigation_{uuid.uuid4().hex[:8]}"
        )

        action_client = ActionClient(
            node,
            NavigateToPose,
            self.action_name,
        )

        try:
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

            send_future = action_client.send_goal_async(action_goal)

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

                return self._result(
                    "FAILED",
                    "TIMEOUT",
                    started_at,
                    f"Navigation exceeded {timeout_sec} seconds.",
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
            node.destroy_node()

    def stop(self) -> None:
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