from __future__ import annotations

import time
from typing import Any, Dict, Optional


class NavigationController:
    """Interface for movement and route control."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        self.settings = settings or {}

    def navigate(self, goal: Dict[str, Any]) -> Dict[str, Any]:
        """Submit a navigation goal.

        This is the HAL boundary for Nav2. The current implementation is a
        deterministic stub; replace the body with a Nav2 action client when
        the ROS navigation stack is ready.
        """
        started_at = time.monotonic()
        target = goal.get("target") or {}
        path_id = goal.get("path_id")

        if path_id:
            final_pose = None
            nav2_result_code = "SIMULATED_PATH_COMPLETED"
        elif "x" in target and "y" in target:
            final_pose = {
                "x": float(target.get("x", 0.0)),
                "y": float(target.get("y", 0.0)),
                "theta": float(target.get("theta", 0.0)),
            }
            nav2_result_code = "SIMULATED_GOAL_REACHED"
        elif target.get("endpoint_id"):
            final_pose = None
            nav2_result_code = "SIMULATED_ENDPOINT_ACCEPTED"
        else:
            return {
                "move_status": "FAILED",
                "final_pose": None,
                "nav2_result_code": "INVALID_GOAL",
                "duration_ms": int((time.monotonic() - started_at) * 1000),
            }

        return {
            "move_status": "COMPLETED",
            "final_pose": final_pose,
            "nav2_result_code": nav2_result_code,
            "duration_ms": int((time.monotonic() - started_at) * 1000),
        }

    def move_to(self, x: float, y: float) -> Dict[str, float]:
        return {"x": x, "y": y}

    def stop(self) -> None:
        pass
