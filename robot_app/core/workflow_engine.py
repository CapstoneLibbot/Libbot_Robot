from __future__ import annotations

import threading
from typing import Any, Dict, List

from core.context import RobotContext
from steps import (
    BaseStep,
    DisplayStep,
    EvaluateStep,
    MoveStep,
    NotifyStep,
    ProcessDataStep,
    ScanStep,
    WaitStep,
)


class WorkflowEngine:
    """Manages a list of steps and coordinates execution using RobotContext."""

    STEP_TYPES = {
        "MOVE": MoveStep,
        "SCAN": ScanStep,
        "PROCESS_DATA": ProcessDataStep,
        "EVALUATE": EvaluateStep,
        "DISPLAY": DisplayStep,
        "WAIT": WaitStep,
        "NOTIFY": NotifyStep,
    }

    def __init__(self, settings: Dict[str, Any], tcp_server: Any):
        self.settings = settings
        self.tcp_server = tcp_server
        self.context = RobotContext()
        self.steps: List[BaseStep] = []
        self.lock = threading.RLock()

    def load_default_workflow(self) -> None:
        workflow_steps = self.settings.get("workflow", {}).get("steps", [])
        self.steps = [self.create_step(step_def) for step_def in workflow_steps]

    def create_step(self, step_def: Dict[str, Any]) -> BaseStep:
        enriched_step = self._with_default_step_config(step_def)
        step_type = enriched_step.get("type", "WAIT")
        step_class = self.STEP_TYPES.get(step_type.upper(), WaitStep)
        return step_class(enriched_step)

    def run(self) -> None:
        self.run_steps([step.config for step in self.steps])

    def add_step(self, step_def: Dict[str, Any]) -> None:
        self.steps.append(self.create_step(step_def))

    def execute_step(self, step_def: Dict[str, Any]) -> Dict[str, Any]:
        step = self.create_step(step_def)
        result = step.execute(self.context)
        with self.lock:
            self.context.set_step_result(step.name, result)
        return {
            "status": "ok",
            "step": step.name,
            "step_type": step.step_type,
            "result": result,
        }

    def _step_succeeded(
        self,
        step_result: Dict[str, Any],
    ) -> bool:
        step_type = step_result.get("step_type")
        result = step_result.get("result", {})

        if step_type == "MOVE":
            return (
                result.get("status") == "ok"
                and result.get("move_status") == "COMPLETED"
            )

        if step_type == "SCAN":
            return result.get("status") == "scanned"

        if step_type == "WAIT":
            return result.get("status") == "waited"

        return result.get("status") not in {
            "error",
            "failed",
            "timeout",
        }
    def run_steps(
        self,
        step_defs: List[Dict[str, Any]],
        fail_fast: bool = True,
        task_id: Any = None,
    ) -> Dict[str, Any]:
        results = []
        failed_step = None

        with self.lock:
            for step_def in step_defs:
                step_result = self.execute_step(step_def)
                results.append(step_result)

                if not self._step_succeeded(step_result):
                    failed_step = step_result.get("step")

                    if fail_fast:
                        break

        return {
            "status": "error" if failed_step else "ok",
            "task_id": task_id,
            "requested_count": len(step_defs),
            "completed_count": len(results),
            "failed_step": failed_step,
            "results": results,
        }

    def handle_request(self, request: Any) -> Dict[str, Any]:
        if isinstance(request, list):
            return self.run_steps(request)
        if not isinstance(request, dict):
            return self._error("Request must be a JSON object or an array of steps.")

        command = str(request.get("command", request.get("action", ""))).lower()
        if not command and "type" in request:
            command = "run_step"

        if command in {"ping", "health"}:
            return {
                "status": "ok",
                "message": "libot workflow engine is running",
                "step_types": sorted(self.STEP_TYPES.keys()),
            }

        if command == "run_workflow":
            steps = request.get("steps")

            if not isinstance(steps, list):
                return self._error(
                    "run_workflow requires a steps array."
                )

            return self.run_steps(
                steps,
                fail_fast=bool(request.get("fail_fast", True)),
                task_id=request.get("task_id"),
            )
        if command == "run_step":
            step_def = request.get("step", request if "type" in request else None)
            if not isinstance(step_def, dict):
                return self._error("run_step requires a step object or top-level type.")
            return self.execute_step(step_def)

        if command == "run_workflow":
            steps = request.get("steps")
            if not isinstance(steps, list):
                return self._error("run_workflow requires a steps array.")
            return self.run_steps(steps)

        if command == "load_workflow":
            steps = request.get("steps")
            if not isinstance(steps, list):
                return self._error("load_workflow requires a steps array.")
            with self.lock:
                self.steps = [self.create_step(step_def) for step_def in steps]
            return {
                "status": "ok",
                "count": len(self.steps),
            }

        if command == "get_context":
            return {
                "status": "ok",
                "context": self.context.snapshot(),
            }

        if command == "set_context":
            values = request.get("values", request.get("variables"))
            if not isinstance(values, dict):
                return self._error("set_context requires values or variables object.")
            with self.lock:
                self.context.update_variables(values)
            return {
                "status": "ok",
                "context": self.context.snapshot(),
            }

        if command == "supported_step_types":
            return {
                "status": "ok",
                "step_types": sorted(self.STEP_TYPES.keys()),
            }

        return self._error(f"Unsupported command: {command or '<missing>'}")

    def _error(self, message: str) -> Dict[str, Any]:
        return {
            "status": "error",
            "message": message,
        }

    def _with_default_step_config(self, step_def: Dict[str, Any]) -> Dict[str, Any]:
        enriched = dict(step_def)
        step_type = str(enriched.get("type", "WAIT")).upper()
        hardware = self.settings.get("hardware", {})

        if step_type == "SCAN":
            scanner = hardware.get("scanner", {})
            enriched.setdefault("device", scanner.get("device", enriched.get("sensor", "camera")))
            enriched.setdefault("timeout_sec", scanner.get("timeout_sec", 5))

        if step_type == "WAIT":
            motion_safety = hardware.get("motion_safety")
            if motion_safety and "motion_safety" not in enriched:
                enriched["motion_safety"] = motion_safety

        return enriched
