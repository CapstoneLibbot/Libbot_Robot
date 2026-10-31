from __future__ import annotations

import json
import math
import os
import shutil
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

    def __init__(self, settings: Dict[str, Any], tcp_server: Any, runtime_gateway: Any = None):
        self.settings = settings
        self.tcp_server = tcp_server
        self.runtime_gateway = runtime_gateway
        self.context = RobotContext()
        self.steps: List[BaseStep] = []
        self.lock = threading.RLock()
        self.executions: Dict[str, Dict[str, Any]] = {}
        self._active_navigation: Optional[NavigationController] = None
        self._queued_auto_charge: Optional[Dict[str, Any]] = None

    def load_default_workflow(self) -> None:
        workflow_steps = self.settings.get("workflow", {}).get("steps", [])
        self.steps = [self.create_step(step_def) for step_def in workflow_steps]

    def create_step(self, step_def: Dict[str, Any]) -> BaseStep:
        enriched_step = self._with_default_step_config(step_def)
        if (str(enriched_step.get("type", "")).upper() == "WAIT"
                and self.runtime_gateway is not None
                and getattr(self.runtime_gateway, "use_sim_time", False)):
            enriched_step["_simulation_speed_provider"] = self._simulation_speed
        if self.runtime_gateway is not None:
            enriched_step["_zone_action_checker"] = self.runtime_gateway.navigation_zone_blocking_action
        enriched_step["_event_callback"] = self._submit_touch_event
        step_type = enriched_step.get("type", "WAIT")
        step_class = self.STEP_TYPES.get(step_type.upper(), WaitStep)
        return step_class(enriched_step)

    def _simulation_speed(self) -> float:
        try:
            return min(4.0, max(1.0, float(self.runtime_gateway.simulation_speed)))
        except (AttributeError, TypeError, ValueError):
            return 1.0

    def run(self) -> None:
        self.run_steps([step.config for step in self.steps])

    def add_step(self, step_def: Dict[str, Any]) -> None:
        self.steps.append(self.create_step(step_def))

    def execute_step(self, step_def: Dict[str, Any]) -> Dict[str, Any]:
        runtime_id = str(self.context.get("runtime_id") or os.getenv("LIBOT_RUNTIME_ID") or "real").lower()
        if (runtime_id != "sim"
                and str(step_def.get("type", "")).upper() == "SCAN"
                and str(step_def.get("sensor", "")).lower() == "patrol_observation"
                and self.runtime_gateway is not None):
            observation = self.runtime_gateway.wait_for_patrol_observation(
                float(step_def.get("timeout_sec", 10))
            )
            if observation is not None:
                self.context.set(str(step_def.get("context_key", "patrol_observation")), observation)
        step = self.create_step(step_def)
        if isinstance(step, MoveStep):
            with self.lock:
                self._active_navigation = step.navigation
        try:
            result = step.execute(self.context)
        finally:
            if isinstance(step, MoveStep):
                with self.lock:
                    if self._active_navigation is step.navigation:
                        self._active_navigation = None
        if str(step_def.get("type", "")).upper() == "SCAN" and str(step_def.get("sensor", "")).lower() == "patrol_observation":
            self.context.set(str(step_def.get("context_key", "patrol_observation")), None)
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
        step_def: Optional[Dict[str, Any]] = None,
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
            if result.get("status") in {"waited", "event"}:
                return True
            transitions = step_def.get("transitions", {}) if isinstance(step_def, dict) else {}
            if not isinstance(transitions, dict):
                transitions = {}
            transitions = {**step_def, **transitions} if isinstance(step_def, dict) else transitions
            return result.get("status") == "timeout" and bool(
                transitions.get("on_timeout", transitions.get("on_timeout_step"))
            )

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
        failure_message = None
        indexed = {
            str(step.get("id", step.get("name", index))): (index, step)
            for index, step in enumerate(step_defs)
        }
        index = 0
        visited = 0
        maximum_steps = max(len(step_defs) * 4, 1)

        while 0 <= index < len(step_defs) and visited < maximum_steps:
            if execution_id is not None and (
                    self._is_cancelled(execution_id)
                    or self._is_low_battery_preempt_requested(execution_id)):
                break
            step_def = step_defs[index]
            if execution_id is not None:
                self._set_current_step(execution_id, step_def)
            try:
                step_result = self.execute_step(step_def)
            except Exception as exc:
                step_result = {
                    "status": "ok",
                    "step": step_def.get("name", step_def.get("id", f"step-{index + 1}")),
                    "step_type": str(step_def.get("type", "WAIT")).upper(),
                    "result": {"status": "error", "message": str(exc)},
                }
            self._apply_output_mapping(step_def, step_result)
            results.append(step_result)
            visited += 1
            succeeded = self._step_succeeded(step_result, step_def)
            if execution_id is not None:
                with self.lock:
                    execution = self.executions.get(execution_id)
                    if execution is not None:
                        execution["last_result"] = step_result.get("result")
                        if succeeded:
                            execution["last_error"] = None
                        else:
                            detail = step_result.get("result", {})
                            execution["last_error"] = detail.get("message") or detail.get("error") or f"Step {step_result.get('step')} failed."
                self._publish_execution(
                    execution_id,
                    "STEP_COMPLETED" if succeeded else "STEP_FAILED",
                    result=step_result.get("result"),
                    error=None if succeeded else str(step_result.get("result", {}).get("message") or step_result.get("result", {}).get("error") or f"Step {step_result.get('step')} failed."),
                )

            # A low-battery request takes effect only after the current step returns.
            if execution_id is not None and self._is_low_battery_preempt_requested(execution_id):
                break

            if str(step_result.get("step_type", "")).upper() == "END":
                if step_result.get("result", {}).get("workflow_status") != "SUCCEEDED":
                    failed_step = step_result.get("step")
                    failure_message = step_result.get("result", {}).get("message") or "Workflow ended with a failure status."
                break

            next_id = self._next_step_id(step_def, step_result, succeeded)
            if next_id is not None:
                if step_result.get("result", {}).get("status") in {"error", "failed"}:
                    failed_step = failed_step or step_result.get("step")
                    detail = step_result.get("result", {})
                    failure_message = failure_message or detail.get("message") or detail.get("error") or f"Step {step_result.get('step')} failed."
                target = indexed.get(str(next_id))
                if target is None:
                    failed_step = step_result.get("step")
                    step_result["transition_error"] = f"Unknown target step: {next_id}"
                    failure_message = step_result["transition_error"]
                    break
                index = target[0]
                continue

            if not succeeded:
                failed_step = step_result.get("step")
                detail = step_result.get("result", {})
                failure_message = failure_message or detail.get("message") or detail.get("error") or f"Step {step_result.get('step')} failed."
            if not succeeded and fail_fast:
                break
            index += 1

        if visited >= maximum_steps and index < len(step_defs):
            failed_step = failed_step or "workflow_transition_limit"
            failure_message = failure_message or "Workflow transition limit was reached."

        low_battery_preempted = execution_id is not None and self._is_low_battery_preempt_requested(execution_id)
        cancelled = execution_id is not None and self._is_cancelled(execution_id)
        return {
            "status": "cancelled" if cancelled or low_battery_preempted else "error" if failed_step else "ok",
            "task_id": task_id,
            "requested_count": len(step_defs),
            "completed_count": len(results),
            "failed_step": failed_step,
            "error": "LOW_BATTERY: mission interrupted at a safe step boundary." if low_battery_preempted else failure_message,
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
            execution = self.executions.get(execution_id, {})
            cancelled_before_start = execution.get("status") == "cancelled"
            self.executions[execution_id] = {
                **execution,
                "status": "cancelled" if cancelled_before_start else "running",
                "task_id": task_id,
                "mission_id": task_id,
                "runtime_id": execution.get("runtime_id") or self.context.get("runtime_id"),
                "current_step": None,
                "current_step_name": None,
                "current_step_type": None,
                "expected_event_type": None,
                "allowed_values": [],
            }
        result = ({"status": "cancelled", "task_id": task_id, "requested_count": len(steps),
                   "completed_count": 0, "failed_step": None}
                  if cancelled_before_start else self.run_steps(steps, task_id=task_id, execution_id=execution_id))
        if (not cancelled_before_start
                and self._is_low_battery_preempt_requested(execution_id)
                and result.get("status") != "cancelled"):
            result["status"] = "cancelled"
            result["error"] = "LOW_BATTERY: mission interrupted at a safe step boundary."
        final_status = "cancelled" if result.get("status") == "cancelled" else "completed" if result.get("status") == "ok" else "failed"
        with self.lock:
            self.executions[execution_id] = {
                **self.executions.get(execution_id, {}),
                "status": final_status,
                "task_id": task_id,
                "expected_event_type": None,
                "allowed_values": [],
                "result": result,
            }
        self._publish_execution(execution_id, final_status.upper(), result=result)
        self._report_completion(task_id, execution_id, result)
        with self.lock:
            queued_charge = self._queued_auto_charge if self.executions.get(execution_id, {}).get("low_battery_preempt_requested") else None
            if queued_charge is not None:
                self._queued_auto_charge = None
        if queued_charge is not None:
            self._start_workflow(queued_charge, allow_safe_queue=True)

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
                "current_step_name": str(step_def.get("name", step_def.get("id", ""))),
                "current_step_type": step_type,
                "expected_event_type": (
                    str(step_def.get("event_type")).upper() if waits_for_event else None
                ),
                "allowed_values": list(step_def.get("allowed_values") or []) if waits_for_event else [],
                "result_variable": step_def.get("result_variable") if waits_for_event else None,
                "step_status": "waiting" if waits_for_event else "running",
            })
        self._publish_execution(execution_id, "WAITING" if waits_for_event else "RUNNING")

    def _publish_execution(
        self,
        execution_id: str,
        status: str,
        result: Any = None,
        error: Optional[str] = None,
    ) -> None:
        with self.lock:
            execution = dict(self.executions.get(execution_id, {}))
        if result is None:
            result = execution.get("last_result")
        mission_id = execution.get("mission_id") or execution.get("task_id")
        base_url = str(os.getenv("LIBOT_BACKEND_URL") or self.context.get("backend_base_url") or "").strip().rstrip("/")
        if not base_url or not mission_id:
            return
        if error is None:
            error = execution.get("last_error")
        body = json.dumps({
            "executionId": execution_id,
            "runtimeId": execution.get("runtime_id") or self.context.get("runtime_id"),
            "status": status,
            "stepId": execution.get("current_step"),
            "stepName": execution.get("current_step_name"),
            "stepType": execution.get("current_step_type"),
            "expectedEventType": execution.get("expected_event_type"),
            "allowedValues": execution.get("allowed_values") or [],
            "expectedBookId": execution.get("assigned_book_id"),
            "result": result,
            "error": error,
        }).encode("utf-8")
        callback = request.Request(
            f"{base_url}/api/missions/{mission_id}/execution/events/state",
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            with request.urlopen(callback, timeout=5):
                pass
        except OSError as exc:
            self.context.update_metadata(execution_callback_error=str(exc))

    def _report_completion(self, mission_id: Any, execution_id: str, result: Dict[str, Any]) -> None:
        base_url = str(
            os.getenv("LIBOT_BACKEND_URL") or self.context.get("backend_base_url") or ""
        ).strip().rstrip("/")
        if not base_url or not mission_id:
            return
        body = json.dumps({
            "status": "CANCELLED" if result.get("status") == "cancelled" else "SUCCEEDED" if result.get("status") == "ok" else "FAILED",
            "executionId": execution_id,
            "message": result.get("error") or result.get("failed_step"),
        }).encode("utf-8")
        callback = request.Request(
            f"{base_url}/api/missions/{mission_id}/execution/completed",
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            with request.urlopen(callback, timeout=10):
                pass
        except OSError as exc:
            self.context.update_metadata(completion_callback_error=str(exc))

    def _is_cancelled(self, execution_id: str) -> bool:
        with self.lock:
            execution = self.executions.get(execution_id, {})
            return execution.get("status") == "cancelled"

    def _is_low_battery_preempt_requested(self, execution_id: str) -> bool:
        with self.lock:
            return bool(self.executions.get(execution_id, {}).get("low_battery_preempt_requested"))

    def _submit_touch_event(self, event_type: str, data: Any) -> None:
        with self.lock:
            active = next((key for key, item in self.executions.items() if item.get("status") in {"accepted", "running"}), None)
        if active:
            self._submit_event(event_type.upper(), data, active)

    def _validate_steps(self, steps: List[Any]) -> Optional[str]:
        if not steps:
            return "Workflow must contain at least one step."
        identifiers = []
        for index, definition in enumerate(steps):
            if not isinstance(definition, dict):
                return f"Step {index + 1} must be an object."
            step_type = str(definition.get("type", "")).upper()
            if step_type not in self.STEP_TYPES:
                return f"Step {index + 1} has unsupported type '{step_type}'."
            identifiers.append(str(definition.get("id", definition.get("name", index))))
        if len(set(identifiers)) != len(identifiers):
            return "Workflow step ids must be unique."
        known = set(identifiers)
        transition_names = {
            "on_success", "on_error", "on_timeout", "on_event", "on_cancel",
            "on_true", "on_false", "on_true_step", "on_false_step", "on_timeout_step", "on_cancel_step"
        }
        for identifier, definition in zip(identifiers, steps):
            transitions = definition.get("transitions", {})
            if not isinstance(transitions, dict):
                return f"Step '{identifier}' transitions must be an object."
            transition_values = dict(definition)
            transition_values.update(transitions)
            for name, target in transition_values.items():
                if name.lower().startswith("on_") and name not in transition_names:
                    return f"Step '{identifier}' has unsupported transition '{name}'."
                if name in transition_names and name != name.lower():
                    return f"Step '{identifier}' transition names must use lowercase snake_case."
                if name in transition_names and target is not None and not isinstance(target, str):
                    return f"Step '{identifier}' transition '{name}' must name a step id."
                if name in transition_names and target is not None and target not in known:
                    return f"Step '{identifier}' transitions to unknown step '{target}'."
        return None

    def _preflight(self, require_hardware: bool, allow_active_workflow: bool = False) -> Dict[str, Any]:
        runtime_id = str(os.getenv("LIBOT_RUNTIME_ID") or self.settings.get("runtime", {}).get("id", "real")).lower()
        checks: Dict[str, Any] = {"runtime_id": runtime_id}
        with self.lock:
            active_workflow = any(item.get("status") in {"accepted", "running"} for item in self.executions.values())
            if not allow_active_workflow and active_workflow:
                return self._error("The robot is already running another workflow.", "PREFLIGHT_FAILED", checks=checks)
        if allow_active_workflow:
            checks["active_workflow"] = active_workflow
        if self.runtime_gateway is None or getattr(self.runtime_gateway, "_node", None) is None:
            return self._error("ROS/Nav2 runtime is unavailable.", "PREFLIGHT_FAILED", checks=checks)
        if getattr(self.runtime_gateway, "error", None):
            return self._error(str(self.runtime_gateway.error), "PREFLIGHT_FAILED", checks=checks)
        try:
            if not self.runtime_gateway.navigation_ready(timeout_sec=2.0):
                return self._error("Nav2 navigate_to_pose action server is unavailable.", "PREFLIGHT_FAILED", checks=checks)
        except Exception as exc:
            return self._error(f"Nav2 preflight failed: {exc}", "PREFLIGHT_FAILED", checks=checks)
        checks["navigation"] = "ready"

        if require_hardware and runtime_id == "real":
            hardware = self.settings.get("hardware", {})
            scanner = hardware.get("scanner", {})
            scanner_device = str(scanner.get("device", "")).lower()
            if not scanner.get("enabled", False) or not scanner_device.startswith("imx500"):
                return self._error("Physical workflow requires the configured IMX500 camera scanner.", "PREFLIGHT_FAILED", checks=checks)
            from hardware.barcode_scanner import BarcodeScanner
            scanner_check = BarcodeScanner(scanner).probe()
            checks["scanner"] = scanner_check
            if scanner_check.get("status") != "ready":
                return self._error(scanner_check.get("message", "IMX500 camera is unavailable."), "PREFLIGHT_FAILED", checks=checks)
            ui = hardware.get("ui", {})
            if not ui.get("enabled", False) or str(ui.get("driver", "tkinter")).lower() != "tkinter":
                return self._error("Physical workflow requires the robot display/touchscreen.", "PREFLIGHT_FAILED", checks=checks)
            if str(ui.get("driver", "tkinter")).lower() == "tkinter" and not os.getenv("DISPLAY") and os.name != "nt":
                return self._error("DISPLAY is not set for the robot touchscreen.", "PREFLIGHT_FAILED", checks=checks)
            from hardware.ui_controller import UIController
            ui_check = UIController.probe()
            checks["display"] = ui_check
            if ui_check.get("status") != "ready":
                return self._error(ui_check.get("message", "Touchscreen display is unavailable."), "PREFLIGHT_FAILED", checks=checks)
            speaker = hardware.get("speaker", {})
            if not speaker.get("enabled", False) or not (shutil.which("espeak-ng") or shutil.which("espeak")):
                return self._error("Physical workflow requires an enabled speaker and espeak-ng/espeak.", "PREFLIGHT_FAILED", checks=checks)
            if os.name != "nt" and not os.path.exists("/dev/snd"):
                return self._error("Physical workflow speaker device /dev/snd is unavailable.", "PREFLIGHT_FAILED", checks=checks)
            checks["speaker"] = "ready"
        else:
            checks["sensors"] = "simulated"
        return {"status": "ok", "checks": checks}

    def _start_workflow(self, request: Dict[str, Any], allow_safe_queue: bool = False) -> Dict[str, Any]:
        steps = request.get("steps")
        if not isinstance(steps, list):
            return self._error("start_workflow requires a steps array.")
        validation_error = self._validate_steps(steps)
        if validation_error:
            return self._error(validation_error, "INVALID_WORKFLOW")
        initial_context = request.get("initial_context", {})
        if not isinstance(initial_context, dict):
            return self._error("initial_context must be an object.")
        runtime_id = str(request.get("runtime_id", "")).lower()
        if allow_safe_queue and runtime_id != "real":
            return self._error("Automatic charging is available only for the real runtime.")
        execution_id = str(request.get("execution_id") or uuid.uuid4())

        queued = False
        with self.lock:
            active = next(
                ((active_id, item) for active_id, item in self.executions.items()
                 if item.get("status") in {"accepted", "running"}),
                None,
            )
            if active and allow_safe_queue:
                active_id, active_execution = active
                if active_execution.get("is_auto_charge"):
                    return {"status": "accepted", "execution_id": active_id, "task_id": active_execution.get("task_id"), "already_charging": True}
                if self._queued_auto_charge is not None:
                    return self._error("A low-battery charging workflow is already queued.")
                self._queued_auto_charge = dict(request)
                active_execution["low_battery_preempt_requested"] = True
                self.context.set("low_battery_preempt_requested", True)
                self.executions[execution_id] = {
                    "status": "queued",
                    "task_id": request.get("task_id"),
                    "mission_id": request.get("mission_id", request.get("task_id")),
                    "runtime_id": runtime_id,
                    "is_auto_charge": True,
                }
                queued = True
            elif active:
                return self._error("The robot is already running another workflow.")
            else:
                current = self.executions.get(execution_id)
                if current and current.get("status") in {"accepted", "running"}:
                    return self._error(f"Execution {execution_id} is already running.")
                self.context = RobotContext()
                self.context.update_variables(initial_context)
                self.executions[execution_id] = {
                    "status": "accepted",
                    "task_id": request.get("task_id"),
                    "mission_id": request.get("mission_id", request.get("task_id")),
                    "runtime_id": request.get("runtime_id", self.context.get("runtime_id")),
                    "assigned_book_id": initial_context.get("book_id"),
                    "current_step": None,
                    "current_step_name": None,
                    "current_step_type": None,
                    "expected_event_type": None,
                    "allowed_values": [],
                    "is_auto_charge": bool(allow_safe_queue),
                }

        if queued:
            self._publish_execution(execution_id, "QUEUED", result={"message": "Waiting for the current safe step to finish.", "preemptedExecutionId": active[0]})
            return {"status": "accepted", "execution_id": execution_id, "task_id": request.get("task_id"), "queued": True}

        thread = threading.Thread(
            target=self._run_execution,
            args=(execution_id, steps, request.get("task_id")),
            daemon=True,
        )
        self._publish_execution(execution_id, "ACCEPTED")
        thread.start()
        return {"status": "accepted", "execution_id": execution_id, "task_id": request.get("task_id")}

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

        if command == "preflight_workflow":
            return self._preflight(
                bool(request.get("require_hardware", False)),
                allow_active_workflow=bool(request.get("allow_active_workflow", False)),
            )

        if command == "cancel_workflow":
            execution_id = str(request.get("execution_id", ""))
            with self.lock:
                execution = self.executions.get(execution_id)
                if execution is None:
                    return self._error("Workflow execution was not found.")
                if execution.get("status") not in {"accepted", "running"}:
                    return self._error("Workflow is not active.")
                execution["status"] = "cancelled"
                self.context.set("workflow_cancelled", True)
                self.context.push_event("WORKFLOW_CANCELLED", {"status": "cancelled"})
                active_navigation = self._active_navigation
            if active_navigation is not None:
                active_navigation.stop()
            if self.runtime_gateway is not None:
                self.runtime_gateway.cancel_active_navigation()
            self._publish_execution(execution_id, "CANCELLED")
            return {"status": "accepted", "execution_id": execution_id}

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
            return self._start_workflow(request)
        if command == "start_auto_charge":
            return self._start_workflow(request, allow_safe_queue=True)
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
                str(request.get("execution_id") or ""),
            )

        if command == "get_execution":
            execution_id = str(request.get("execution_id", ""))
            with self.lock:
                execution = self.executions.get(execution_id)
                if execution is None:
                    return self._error(f"Execution {execution_id or '<missing>'} was not found.")
                execution_snapshot = dict(execution)
                execution_snapshot["currentStep"] = execution_snapshot.pop("current_step", None)
                execution_snapshot["currentStepName"] = execution_snapshot.pop("current_step_name", None)
                execution_snapshot["currentStepType"] = execution_snapshot.pop("current_step_type", None)
                execution_snapshot["expectedEventType"] = execution_snapshot.pop("expected_event_type", None)
                execution_snapshot["allowedValues"] = execution_snapshot.pop("allowed_values", [])
                execution_snapshot["expectedBookId"] = execution_snapshot.pop("assigned_book_id", None)
                execution_snapshot.pop("result_variable", None)
            return {"status": "ok", "execution_id": execution_id, "execution": execution_snapshot}

        if command == "supported_step_types":
            return {
                "status": "ok",
                "step_types": sorted(self.STEP_TYPES.keys()),
            }

        return self._error(f"Unsupported command: {command or '<missing>'}")

    def _submit_event(self, event_type: str, data: Any, requested_execution_id: str = "") -> Dict[str, Any]:
        with self.lock:
            if requested_execution_id:
                candidate = self.executions.get(requested_execution_id)
                active = (requested_execution_id, candidate) if candidate and candidate.get("status") in {"accepted", "running"} else None
            else:
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
            runtime_id = str(execution.get("runtime_id") or self.context.get("runtime_id") or "").lower()
            if event_type == "BOOK_SELECTED":
                expected_book_id = self.context.get("book_id")
                if runtime_id == "sim":
                    if expected_book_id and str(selected) != str(expected_book_id):
                        self.context.set("book_item_id", None)
                    self.context.set("book_id", selected)
                elif expected_book_id and str(selected) != str(expected_book_id):
                    return self._error(
                        "Selected book does not match the book assigned to this execution.",
                        "INVALID_EVENT_DATA",
                    )
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
        for key in (
            "value", "action", "selectedBookId", "selected_book_id", "bookId", "book_id",
            "facilityId", "facility_id", "selectedFacilityId", "selected_facility_id", "observation", "rating",
        ):
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
            runtime_id = str(self.context.get("runtime_id") or os.getenv("LIBOT_RUNTIME_ID")
                             or self.settings.get("runtime", {}).get("id", "real")).lower()
            if runtime_id == "real" and str(enriched.get("sensor", "camera")).lower() != "patrol_observation":
                enriched["device"] = scanner.get("device", enriched.get("device", enriched.get("sensor", "camera")))
            elif runtime_id == "sim":
                enriched["device"] = "simulation"
            else:
                enriched.setdefault("device", scanner.get("device", enriched.get("sensor", "camera")))
            enriched.setdefault("timeout_sec", scanner.get("timeout_sec", 5))

        if step_type == "WAIT":
            motion_safety = hardware.get("motion_safety")
            if motion_safety and "motion_safety" not in enriched:
                enriched["motion_safety"] = motion_safety
            if hardware.get("ui") and "ui" not in enriched:
                enriched["ui"] = hardware["ui"]

        if step_type == "DISPLAY" and "ui" not in enriched:
            enriched["ui"] = hardware.get("ui", {})

        if step_type == "NOTIFY" and "speaker" not in enriched:
            enriched["speaker"] = hardware.get("speaker", {})

        if step_type == "PROCESS_DATA":
            backend = self.settings.get("backend", {})
            if backend.get("base_url"):
                enriched.setdefault("backend_base_url", backend["base_url"])

        return enriched
