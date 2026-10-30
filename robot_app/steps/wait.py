from __future__ import annotations

import time
from typing import Any, Dict, Optional

from core.context import RobotContext
from hardware.motion_safety import MotionSafetyController
from hardware.ui_controller import UIController
from steps.base_step import BaseStep


class WaitStep(BaseStep):
    """Step Type: WAIT"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        default_seconds = self.config.get("duration_sec", self._duration_ms_default())
        self.seconds = self._read_float("seconds", default_seconds)
        self.context_key = self.config.get("context_key") or self.config.get("until_context_key")
        self.wait_for = str(self.config.get("wait_for", "CONTEXT" if self.context_key else "DURATION")).upper()
        self.event_type = self.config.get("event_type")
        self.result_variable = self.config.get("result_variable") or self.config.get("result")
        self.allowed_values = self.config.get("allowed_values")
        default_timeout = 30.0 if self.context_key else self.seconds
        self.timeout_sec = self._read_float("timeout_sec", default_timeout)
        self.poll_interval_sec = self._read_float("poll_interval_sec", 0.1)
        self.expected_value = self.config.get("expected_value")
        self.simulation_speed_provider = self.config.get("_simulation_speed_provider")
        self.zone_action_checker = self.config.get("_zone_action_checker")
        self.ui_screen = self.config.get("ui_screen", "WAIT_FOR_TOUCH")
        self.prompt = self.config.get("prompt", "Choose an option to continue")
        self.ui = UIController(self.config.get("ui", {}), self.config.get("_event_callback"))
        self.stop_motion = self._read_bool(self.config.get("stop_motion", self.config.get("stop_motors", True)))
        safety_settings = self.config.get("motion_safety", {})
        self.motion_safety = MotionSafetyController(safety_settings)

    def execute(self, context: RobotContext) -> Any:
        started_at = time.monotonic()
        if callable(self.zone_action_checker):
            try:
                blocking_zone = self.zone_action_checker("WAIT")
                if blocking_zone:
                    payload = {
                        "status": "error",
                        "type": "WAIT",
                        "message": f"WAIT/PAUSE is not allowed inside a {blocking_zone} zone.",
                        "seconds": self.seconds,
                        "timeout_sec": self.timeout_sec,
                        "motion_stop_before": None,
                        "motion_stop_after": None,
                        "duration_ms": 0,
                    }
                    context.set("last_wait_result", payload)
                    return payload
            except (TypeError, ValueError):
                pass
        stop_before = self.motion_safety.stop("wait_start") if self.stop_motion else None

        event_data = None
        if self.wait_for == "USER_TOUCH" and self.event_type:
            self.ui.show_screen(self.ui_screen, {
                "prompt": self.prompt,
                "options": self.allowed_values or [],
                "event_type": self.event_type,
            }, event_type=str(self.event_type).upper())
            status, event_data = self._wait_for_event(context)
        elif self.context_key:
            status = self._wait_for_context(context)
        else:
            delay = self.timeout_sec if self.wait_for == "TIMEOUT" and "timeout_sec" in self.config else self.seconds
            status = self._wait_duration(context, delay)
            if status != "cancelled":
                status = "timeout" if self.wait_for == "TIMEOUT" else "waited"

        stop_after = self.motion_safety.stop("wait_end") if self.stop_motion else None
        payload = {
            "status": status,
            "type": "WAIT",
            "seconds": self.seconds,
            "timeout_sec": self.timeout_sec,
            "context_key": self.context_key,
            "expected_value": self.expected_value,
            "event_type": self.event_type,
            "event_data": event_data,
            "duration_ms": int((time.monotonic() - started_at) * 1000),
            "motion_stop_before": stop_before,
            "motion_stop_after": stop_after,
        }
        context.set("last_wait", self.seconds)
        context.set("last_wait_result", payload)
        return payload

    def _wait_for_event(self, context: RobotContext) -> tuple[str, Any]:
        deadline = time.monotonic() + max(self.timeout_sec, 0.0)
        while time.monotonic() <= deadline:
            if context.get("workflow_cancelled", False):
                return "cancelled", None
            value = context.pop_event(str(self.event_type))
            if value is not None:
                selected = self._event_value(value)
                if self.allowed_values and selected not in self.allowed_values:
                    continue
                if self.result_variable:
                    context.set(str(self.result_variable), selected)
                return "event", selected
            time.sleep(max(self.poll_interval_sec, 0.01))
        return "timeout", None

    def _event_value(self, value: Any) -> Any:
        if not isinstance(value, dict):
            return value
        if self.result_variable in value:
            return value[self.result_variable]
        for key in (
            "value", "action", "selectedBookId", "selected_book_id", "bookId", "book_id",
            "facilityId", "facility_id", "selectedFacilityId", "selected_facility_id", "rating",
            "observation",
        ):
            if key in value:
                return value[key]
        return value

    def _wait_for_context(self, context: RobotContext) -> str:
        remaining = max(self.timeout_sec, 0.0)
        last_tick = time.monotonic()
        while remaining > 0:
            if context.get("workflow_cancelled", False):
                return "cancelled"
            value = context.get(str(self.context_key))
            if self._matches(value):
                return "waited"
            now = time.monotonic()
            speed = self._simulation_speed()
            remaining -= (now - last_tick) * speed
            last_tick = now
            if remaining > 0:
                time.sleep(min(max(self.poll_interval_sec, 0.01) / speed, remaining / speed))
        return "timeout"

    def _wait_duration(self, context: RobotContext, delay: float) -> str:
        remaining = max(delay, 0.0)
        while remaining > 0:
            if context.get("workflow_cancelled", False):
                return "cancelled"
            speed = self._simulation_speed()
            started = time.monotonic()
            time.sleep(min(0.1, remaining / speed))
            remaining -= (time.monotonic() - started) * speed
        return "waited"

    def _simulation_speed(self) -> float:
        if not callable(self.simulation_speed_provider):
            return 1.0
        try:
            return min(4.0, max(1.0, float(self.simulation_speed_provider())))
        except (TypeError, ValueError):
            return 1.0

    def _matches(self, value: Any) -> bool:
        if self.expected_value is None:
            return bool(value)
        return value == self.expected_value

    def _read_float(self, key: str, default: Any) -> float:
        try:
            return float(self.config.get(key, default))
        except (TypeError, ValueError):
            return float(default or 0.0)

    def _duration_ms_default(self) -> float:
        try:
            return float(self.config.get("duration_ms", 0.0)) / 1000.0
        except (TypeError, ValueError):
            return 0.0

    def _read_bool(self, value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() not in {"0", "false", "no", "off"}
        return bool(value)
