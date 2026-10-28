from __future__ import annotations

from typing import Any, Dict, Iterable, Optional, Tuple

from core.context import RobotContext
from hardware.navigation import NavigationController
from steps.base_step import BaseStep


class MoveStep(BaseStep):
    """Step Type: MOVE

    Executes AMR navigation on a 2D map through the navigation HAL.
    """

    SPEED_MODES = {"NORMAL", "SLOW_GUIDE", "PATROL"}
    OBSTACLE_POLICIES = {"WAIT_RESUME", "REROUTE", "DETECT_AND_WAIT"}
    OBSTACLE_POLICY_ALIASES = {"WAIT": "WAIT_RESUME"}

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.target = self.config.get("target")
        self.endpoint_id = self.config.get("endpoint_id")
        self.speed_mode = self._normalize_enum(self.config.get("speed_mode", self.config.get("speed")), "NORMAL")
        self.obstacle_policy = self._normalize_obstacle_policy(self.config.get("obstacle_policy"))
        self.scan_while_moving = bool(self.config.get("scan_while_moving", False))
        self.timeout_sec = self._extract_optional_int(self.config.get("timeout_sec"))
        self.path_id = self.config.get("path_id")
        self.navigation = NavigationController()
        self.zone_action_checker = self.config.get("_zone_action_checker")

    def _normalize_enum(self, value: Any, default: str) -> str:
        if value is None:
            return default
        return str(value).upper()

    def _normalize_obstacle_policy(self, value: Any) -> str:
        policy = self._normalize_enum(value, "WAIT_RESUME")
        return self.OBSTACLE_POLICY_ALIASES.get(policy, policy)

    def _extract_optional_int(self, value: Any) -> Optional[int]:
        if value is None:
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _extract_numeric(self, value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def _normalize_target(self, raw_target: Any) -> Optional[Dict[str, Any]]:
        if raw_target is None:
            return None

        if isinstance(raw_target, dict):
            if "x" in raw_target or "y" in raw_target:
                target = {
                    "x": self._extract_numeric(raw_target.get("x"), 0.0),
                    "y": self._extract_numeric(raw_target.get("y"), 0.0),
                }
                if raw_target.get("endpoint_id") is not None:
                    target["endpoint_id"] = raw_target["endpoint_id"]
                elif raw_target.get("id") is not None:
                    target["endpoint_id"] = raw_target["id"]
                if raw_target.get("theta") is not None:
                    target["theta"] = self._extract_numeric(raw_target.get("theta"), 0.0)
                return target

            nested = raw_target.get("target") or raw_target.get("destination") or raw_target.get("endpoint")
            if nested is not None:
                return self._normalize_target(nested)

            if "endpoint_id" in raw_target or "id" in raw_target:
                target = {"endpoint_id": raw_target.get("endpoint_id", raw_target.get("id"))}
                if raw_target.get("x") is not None:
                    target["x"] = self._extract_numeric(raw_target.get("x"), 0.0)
                if raw_target.get("y") is not None:
                    target["y"] = self._extract_numeric(raw_target.get("y"), 0.0)
                return target

            return None

        if isinstance(raw_target, (list, tuple)):
            if len(raw_target) >= 2:
                return {
                    "x": self._extract_numeric(raw_target[0], 0.0),
                    "y": self._extract_numeric(raw_target[1], 0.0),
                }
            return None

        if isinstance(raw_target, str):
            return {"endpoint_id": raw_target}

        return None

    def _resolve_target(
        self,
        context: RobotContext,
    ) -> Optional[Dict[str, Any]]:
        
        if self.target is not None:
            normalized = self._normalize_target(self.resolve(context, self.target))

            if normalized is not None:
                return normalized

        if self.endpoint_id is not None:
            runtime_destination = self._normalize_target(context.get("destination"))
            if runtime_destination is not None and "x" in runtime_destination and "y" in runtime_destination:
                runtime_destination["endpoint_id"] = self.resolve(context, self.endpoint_id)
                return runtime_destination
            return {
                "endpoint_id": self.resolve(context, self.endpoint_id),
            }

        # Context chỉ là fallback.
        candidate_keys: Iterable[str] = (
            "destination",
            "target_endpoint_id",
            "selected_facility_id",
            "current_shelf_endpoint",
            "target_shelves",
            "endpoint_id",
            "target",
        )

        for key in candidate_keys:
            value = context.get(key)

            if value is not None:
                normalized = self._normalize_target(value)

                if normalized is not None:
                    return normalized

        return None

    def _resolve_goal(self, context: RobotContext) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
        if self.path_id is not None:
            return None, "MOVE path_id requires a route resolver; use a resolved target pose instead."

        target = self._resolve_target(context)
        if target is None:
            return None, "No valid endpoint_id, target, or path_id was provided for the move step."

        destination = dict(target)
        if "endpoint_id" in destination and not destination.get("endpoint_id"):
            return None, "endpoint_id must be a non-empty string."

        if "x" not in destination or "y" not in destination:
            return (
                None,
                "MOVE requires x and y. "
                "endpoint_id is only used for identification.",
            )

        if "x" in destination or "y" in destination:
            x_value = self._extract_numeric(destination.get("x"), 0.0)
            y_value = self._extract_numeric(destination.get("y"), 0.0)
            target_pose = {
                "x": x_value,
                "y": y_value,
                "theta": self._extract_numeric(destination.get("theta"), 0.0),
            }
            if destination.get("endpoint_id") is not None:
                target_pose["endpoint_id"] = destination["endpoint_id"]
            return {"target": target_pose, "path_id": None}, None

        return {"target": {"endpoint_id": destination["endpoint_id"]}, "path_id": None}, None

    def _validate_navigation_options(self) -> Optional[str]:
        if self.speed_mode not in self.SPEED_MODES:
            return f"Invalid speed_mode '{self.speed_mode}'. Expected one of: {', '.join(sorted(self.SPEED_MODES))}."
        if self.obstacle_policy not in self.OBSTACLE_POLICIES:
            return (
                f"Invalid obstacle_policy '{self.obstacle_policy}'. "
                f"Expected one of: {', '.join(sorted(self.OBSTACLE_POLICIES))}."
            )
        return None

    def _error_payload(self, context: RobotContext, message: str) -> Dict[str, Any]:
        current_pose = context.get("current_pose", {"x": 0.0, "y": 0.0, "theta": 0.0})
        context.set("move_status", "FAILED")
        return {
            "status": "error",
            "type": "MOVE",
            "move_status": "FAILED",
            "message": message,
            "target": None,
            "final_pose": None,
            "nav2_result_code": "INVALID_INPUT",
            "duration_ms": 0,
            "obstacle_info": None,
            "speed_mode": self.speed_mode,
            "obstacle_policy": self.obstacle_policy,
            "scan_while_moving": self.scan_while_moving,
            "timeout_sec": self.timeout_sec,
            "current_pose": current_pose,
        }

    def execute(self, context: RobotContext) -> Any:
        current_pose = context.get("current_pose", {"x": 0.0, "y": 0.0, "theta": 0.0})
        validation_error = self._validate_navigation_options()
        if validation_error is not None:
            return self._error_payload(context, validation_error)

        resolved_goal, goal_error = self._resolve_goal(context)
        if goal_error is not None or resolved_goal is None:
            return self._error_payload(context, goal_error or "Invalid navigation goal.")

        requested_target = resolved_goal.get("target") or {"path_id": resolved_goal.get("path_id")}
        if callable(self.zone_action_checker) and isinstance(requested_target, dict):
            try:
                blocking_zone = self.zone_action_checker("GOAL", requested_target)
                if blocking_zone:
                    return self._error_payload(
                        context,
                        f"Navigation target is inside a {blocking_zone} zone.",
                    )
            except (TypeError, ValueError):
                pass
        goal = {
            "target": resolved_goal.get("target"),
            "path_id": resolved_goal.get("path_id"),
            "speed_mode": self.speed_mode,
            "obstacle_policy": self.obstacle_policy,
            "scan_while_moving": self.scan_while_moving,
            "timeout_sec": self.timeout_sec,
            "current_pose": current_pose,
        }
        nav_result = self.navigation.navigate(goal)

        move_status = nav_result.get("move_status", "FAILED")
        final_pose = nav_result.get("final_pose")
        nav2_result_code = nav_result.get("nav2_result_code")
        duration_ms = nav_result.get("duration_ms")
        obstacle_info = nav_result.get("obstacle_info")
        message = nav_result.get("message")

        context.set("last_move_target", requested_target)
        if isinstance(requested_target, dict) and requested_target.get("endpoint_id") is not None:
            context.set("target_endpoint_id", requested_target["endpoint_id"])
        context.set("move_status", move_status)
        if final_pose is not None:
            context.set("current_pose", final_pose)

        result = {
            "status": "ok" if move_status == "COMPLETED" else "error",
            "type": "MOVE",
            "move_status": move_status,
            "target": requested_target,
            "final_pose": final_pose,
            "nav2_result_code": nav2_result_code,
            "duration_ms": duration_ms,
            "obstacle_info": obstacle_info,
            "speed_mode": self.speed_mode,
            "obstacle_policy": self.obstacle_policy,
            "path_id": self.path_id,
            "scan_while_moving": self.scan_while_moving,
            "timeout_sec": self.timeout_sec,
            "current_pose": current_pose,
        }
        if message:
            result["message"] = message
        return result
