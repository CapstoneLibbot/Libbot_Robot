from __future__ import annotations

import json
import math
import threading
import uuid
from typing import Any, Dict, List, Optional
from urllib import request

from core.context import RobotContext
from steps import (
    BaseStep,
    DisplayStep,
    EndStep,
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
        "END": EndStep,
    }

    def __init__(self, settings: Dict[str, Any], tcp_server: Any):
        self.settings = settings
        self.tcp_server = tcp_server
        self.context = RobotContext()
        self.steps: List[BaseStep] = []
        self.lock = threading.RLock()
        self.executions: Dict[str, Dict[str, Any]] = {}

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

    def _apply_output_mapping(self, step_def: Dict[str, Any], step_result: Dict[str, Any]) -> None:
        mapping = step_def.get("output_mapping", {})
        if not isinstance(mapping, dict):
            return
        result = step_result.get("result", {})
        for source_path, context_key in mapping.items():
            value: Any = result
            for part in str(source_path).split("."):
                if not isinstance(value, dict):
                    value = None
                    break
                value = value.get(part)
            if context_key and value is not None:
                self.context.set(str(context_key), value)

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
            return result.get("status") in {"waited", "event"}

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
        execution_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        results = []
        failed_step = None
        indexed = {
            str(step.get("id", step.get("name", index))): (index, step)
            for index, step in enumerate(step_defs)
        }
        index = 0
        visited = 0
        maximum_steps = max(len(step_defs) * 4, 1)

        while 0 <= index < len(step_defs) and visited < maximum_steps:
            step_def = step_defs[index]
            if execution_id is not None:
                self._set_current_step(execution_id, step_def)
            step_result = self.execute_step(step_def)
            self._apply_output_mapping(step_def, step_result)
            results.append(step_result)
            visited += 1

            if str(step_result.get("step_type", "")).upper() == "END":
                if step_result.get("result", {}).get("workflow_status") != "SUCCEEDED":
                    failed_step = step_result.get("step")
                break

            succeeded = self._step_succeeded(step_result)
            next_id = self._next_step_id(step_def, step_result, succeeded)
            if next_id is not None:
                if step_result.get("result", {}).get("status") in {"error", "failed"}:
                    failed_step = failed_step or step_result.get("step")
                target = indexed.get(str(next_id))
                if target is None:
                    failed_step = step_result.get("step")
                    step_result["transition_error"] = f"Unknown target step: {next_id}"
                    break
                index = target[0]
                continue

            if not succeeded:
                failed_step = step_result.get("step")
            if not succeeded and fail_fast:
                break
            index += 1

        if visited >= maximum_steps and index < len(step_defs):
            failed_step = failed_step or "workflow_transition_limit"

        return {
            "status": "error" if failed_step else "ok",
            "task_id": task_id,
            "requested_count": len(step_defs),
            "completed_count": len(results),
            "failed_step": failed_step,
            "results": results,
        }

    def _next_step_id(
        self,
        step_def: Dict[str, Any],
        step_result: Dict[str, Any],
        succeeded: bool,
    ) -> Any:
        transitions = step_def.get("transitions", {})
        if not isinstance(transitions, dict):
            transitions = {}
        transitions = {**step_def, **transitions}
        result = step_result.get("result", {})
        step_type = str(step_result.get("step_type", "")).upper()

        if step_type == "EVALUATE":
            if result.get("result"):
                return transitions.get("on_true", transitions.get("on_true_step"))
            return transitions.get("on_false", transitions.get("on_false_step"))
        if step_type == "WAIT":
            status = result.get("status")
            if status == "event":
                return transitions.get("on_event", transitions.get("on_success"))
            if status == "timeout":
                return transitions.get("on_timeout", transitions.get("on_timeout_step"))
            if status == "cancelled":
                return transitions.get("on_cancel", transitions.get("on_cancel_step"))
        if succeeded:
            return transitions.get("on_success")
        if result.get("status") == "timeout":
            return transitions.get("on_timeout", transitions.get("on_timeout_step", transitions.get("on_error")))
        return transitions.get("on_error")

    def _run_execution(self, execution_id: str, steps: List[Dict[str, Any]], task_id: Any) -> None:
        with self.lock:
            self.executions[execution_id] = {
                "status": "running",
                "task_id": task_id,
                "current_step": None,
                "current_step_type": None,
                "expected_event_type": None,
                "allowed_values": [],
            }
        result = self.run_steps(steps, task_id=task_id, execution_id=execution_id)
        with self.lock:
            self.executions[execution_id] = {
                **self.executions.get(execution_id, {}),
                "status": "completed" if result.get("status") == "ok" else "failed",
                "task_id": task_id,
                "expected_event_type": None,
                "allowed_values": [],
                "result": result,
            }
        self._report_completion(task_id, result)

    def _set_current_step(self, execution_id: str, step_def: Dict[str, Any]) -> None:
        step_type = str(step_def.get("type", "WAIT")).upper()
        waits_for_event = (
            step_type == "WAIT"
            and str(step_def.get("wait_for", "")).upper() == "USER_TOUCH"
            and bool(step_def.get("event_type"))
        )
        with self.lock:
            execution = self.executions.get(execution_id)
            if execution is None:
                return
            execution.update({
                "current_step": str(step_def.get("id", step_def.get("name", ""))),
                "current_step_type": step_type,
                "expected_event_type": (
                    str(step_def.get("event_type")).upper() if waits_for_event else None
                ),
                "allowed_values": list(step_def.get("allowed_values") or []) if waits_for_event else [],
                "result_variable": step_def.get("result_variable") if waits_for_event else None,
            })

    def _report_completion(self, task_id: Any, result: Dict[str, Any]) -> None:
        base_url = str(self.context.get("backend_base_url") or "").rstrip("/")
        if not base_url or not task_id:
            return
        body = json.dumps({
            "status": "SUCCEEDED" if result.get("status") == "ok" else "FAILED",
            "message": result.get("failed_step"),
        }).encode("utf-8")
        callback = request.Request(
            f"{base_url}/api/book-guidance/tasks/{task_id}/complete",
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            with request.urlopen(callback, timeout=10):
                pass
        except OSError as exc:
            self.context.update_metadata(completion_callback_error=str(exc))

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
        if command == "start_workflow":
            steps = request.get("steps")
            if not isinstance(steps, list):
                return self._error("start_workflow requires a steps array.")
            initial_context = request.get("initial_context", {})
            if not isinstance(initial_context, dict):
                return self._error("initial_context must be an object.")
            execution_id = str(request.get("execution_id") or uuid.uuid4())
            with self.lock:
                if any(item.get("status") in {"accepted", "running"} for item in self.executions.values()):
                    return self._error("The robot is already running another workflow.")
                current = self.executions.get(execution_id)
                if current and current.get("status") in {"accepted", "running"}:
                    return self._error(f"Execution {execution_id} is already running.")
                self.context = RobotContext()
                self.context.update_variables(initial_context)
                self.executions[execution_id] = {
                    "status": "accepted",
                    "task_id": request.get("task_id"),
                    "current_step": None,
                    "current_step_type": None,
                    "expected_event_type": None,
                    "allowed_values": [],
                }
            thread = threading.Thread(
                target=self._run_execution,
                args=(execution_id, steps, request.get("task_id")),
                daemon=True,
            )
            thread.start()
            return {"status": "accepted", "execution_id": execution_id, "task_id": request.get("task_id")}
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

        if command == "submit_event":
            event_type = request.get("event_type")
            if not event_type:
                return self._error("submit_event requires event_type.", "INVALID_EVENT_DATA")
            return self._submit_event(
                str(event_type).upper(),
                request.get("data", request.get("value")),
            )

        if command == "get_execution":
            execution_id = str(request.get("execution_id", ""))
            with self.lock:
                execution = self.executions.get(execution_id)
                if execution is None:
                    return self._error(f"Execution {execution_id or '<missing>'} was not found.")
                execution_snapshot = dict(execution)
                execution_snapshot["currentStep"] = execution_snapshot.pop("current_step", None)
                execution_snapshot["currentStepType"] = execution_snapshot.pop("current_step_type", None)
                execution_snapshot["expectedEventType"] = execution_snapshot.pop("expected_event_type", None)
                execution_snapshot["allowedValues"] = execution_snapshot.pop("allowed_values", [])
                execution_snapshot.pop("result_variable", None)
            return {"status": "ok", "execution_id": execution_id, "execution": execution_snapshot}

        if command == "supported_step_types":
            return {
                "status": "ok",
                "step_types": sorted(self.STEP_TYPES.keys()),
            }

        return self._error(f"Unsupported command: {command or '<missing>'}")

    def _submit_event(self, event_type: str, data: Any) -> Dict[str, Any]:
        with self.lock:
            active = next(
                (
                    (execution_id, execution)
                    for execution_id, execution in self.executions.items()
                    if execution.get("status") in {"accepted", "running"}
                ),
                None,
            )
            if active is None:
                return self._error(
                    "There is no active workflow waiting for an event.",
                    "NO_ACTIVE_EXECUTION",
                )

            execution_id, execution = active
            expected_event_type = execution.get("expected_event_type")
            if not expected_event_type or event_type != expected_event_type:
                return self._error(
                    f"Workflow expects event '{expected_event_type or '<none>'}', not '{event_type}'.",
                    "UNEXPECTED_EVENT",
                    current_step=execution.get("current_step"),
                    expected_event_type=expected_event_type,
                )

            selected = self._event_value(data, execution.get("result_variable"))
            allowed_values = execution.get("allowed_values") or []
            if allowed_values and selected not in allowed_values:
                return self._error(
                    f"Value '{selected}' is not allowed at step {execution.get('current_step')}.",
                    "INVALID_EVENT_DATA",
                    current_step=execution.get("current_step"),
                    allowed_values=allowed_values,
                )

            if selected == "START_GUIDANCE" and self.context.get("book_detail") is not None:
                target_error = self._validate_book_navigation_target()
                if target_error is not None:
                    return self._error(
                        target_error,
                        "INVALID_NAVIGATION_TARGET",
                        current_step=execution.get("current_step"),
                    )

            self.context.push_event(event_type, data)
            return {
                "status": "accepted",
                "execution_id": execution_id,
                "current_step": execution.get("current_step"),
                "event_type": event_type,
            }

    def _event_value(self, data: Any, result_variable: Any = None) -> Any:
        if not isinstance(data, dict):
            return data
        if result_variable and result_variable in data:
            return data[result_variable]
        for key in ("value", "action", "selectedBookId", "selected_book_id", "bookId", "book_id", "rating"):
            if key in data:
                return data[key]
        return data

    def _validate_book_navigation_target(self) -> Optional[str]:
        book_detail = self.context.get("book_detail")
        location = book_detail.get("location") if isinstance(book_detail, dict) else None
        endpoint = location.get("endpoint") if isinstance(location, dict) else None
        if not isinstance(endpoint, dict):
            return "The selected book has no active navigation endpoint."

        for coordinate in ("x", "y", "theta"):
            value = endpoint.get(coordinate)
            try:
                if value is None or not math.isfinite(float(value)):
                    raise ValueError
            except (TypeError, ValueError):
                return f"The selected book endpoint has an invalid {coordinate} coordinate."
        return None

    def _error(self, message: str, code: Optional[str] = None, **details: Any) -> Dict[str, Any]:
        result = {
            "status": "error",
            "message": message,
        }
        if code:
            result["code"] = code
        result.update(details)
        return result

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

        if step_type == "DISPLAY" and "ui" not in enriched:
            enriched["ui"] = hardware.get("ui", {})

        if step_type == "NOTIFY" and "speaker" not in enriched:
            enriched["speaker"] = hardware.get("speaker", {})

        if step_type == "PROCESS_DATA":
            backend = self.settings.get("backend", {})
            if backend.get("base_url"):
                enriched.setdefault("backend_base_url", backend["base_url"])

        return enriched
