from __future__ import annotations

from typing import Any, Dict, Iterable, Optional

from core.context import RobotContext
from hardware.navigation import NavigationController
from steps.base_step import BaseStep


class MoveStep(BaseStep):
    """Step Type: MOVE

    Executes AMR navigation on a 2D map. The move target may come from the
    step configuration (for example "endpoint_id" or "target") or from runtime
    context variables such as "destination", "target_endpoint_id",
    "selected_facility_id", "current_shelf_endpoint", or "current_path".
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.target = self.config.get("target")
        self.endpoint_id = self.config.get("endpoint_id")
        self.speed_mode = self.config.get("speed_mode", self.config.get("speed", "NORMAL"))
        self.obstacle_policy = self.config.get("obstacle_policy", "WAIT_RESUME")
        self.scan_while_moving = bool(self.config.get("scan_while_moving", False))
        self.path_id = self.config.get("path_id")
        self.navigation = NavigationController()

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

    def _resolve_target(self, context: RobotContext) -> Optional[Dict[str, Any]]:
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

        if self.target is not None:
            normalized = self._normalize_target(self.target)
            if normalized is not None:
                return normalized

        if self.endpoint_id is not None:
            return {"endpoint_id": self.endpoint_id}

        return None

    def execute(self, context: RobotContext) -> Any:
        current_pose = context.get("current_pose", {"x": 0.0, "y": 0.0, "theta": 0.0})
        target = self._resolve_target(context)

        if target is None:
            error_payload = {
                "status": "error",
                "type": "MOVE",
                "move_status": "failed",
                "message": "No valid target or endpoint was provided for the move step.",
                "target": None,
                "speed_mode": self.speed_mode,
                "obstacle_policy": self.obstacle_policy,
                "current_pose": current_pose,
            }
            context.set("move_status", "failed")
            return error_payload

        destination = dict(target)
        if "endpoint_id" in destination and "x" not in destination and "y" not in destination:
            endpoint_id = destination.get("endpoint_id")
            context.set("target_endpoint_id", endpoint_id)
            context.set("last_move_target", {"endpoint_id": endpoint_id})
            context.set("move_status", "completed")
            return {
                "status": "ok",
                "type": "MOVE",
                "move_status": "completed",
                "target": {"endpoint_id": endpoint_id},
                "endpoint_id": endpoint_id,
                "speed_mode": self.speed_mode,
                "obstacle_policy": self.obstacle_policy,
                "current_pose": current_pose,
                "message": f"Move goal accepted for endpoint {endpoint_id}.",
            }

        x_value = self._extract_numeric(destination.get("x"), current_pose.get("x", 0.0))
        y_value = self._extract_numeric(destination.get("y"), current_pose.get("y", 0.0))
        target_pose = {"x": x_value, "y": y_value, "theta": self._extract_numeric(destination.get("theta"), current_pose.get("theta", 0.0))}

        route = self.navigation.move_to(x_value, y_value)
        requested_target = {"x": x_value, "y": y_value, **({"theta": target_pose["theta"]} if "theta" in target_pose else {})}

        if destination.get("endpoint_id") is not None:
            requested_target["endpoint_id"] = destination["endpoint_id"]
        context.set("last_move_target", requested_target)
        context.set("target_endpoint_id", destination.get("endpoint_id") or context.get("target_endpoint_id"))

        move_status = "completed" if route is not None else "failed"
        context.set("current_pose", target_pose)
        context.set("move_status", move_status)

        return {
            "status": "ok" if move_status == "completed" else "error",
            "type": "MOVE",
            "move_status": move_status,
            "target": requested_target,
            "speed_mode": self.speed_mode,
            "obstacle_policy": self.obstacle_policy,
            "current_pose": target_pose,
            "route": route,
            "path_id": self.path_id,
            "scan_while_moving": self.scan_while_moving,
        }
