from __future__ import annotations

import time
from typing import Any, Dict, Optional

from core.context import RobotContext
from hardware.motion_safety import MotionSafetyController
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
        self.stop_motion = self._read_bool(self.config.get("stop_motion", self.config.get("stop_motors", True)))
        safety_settings = self.config.get("motion_safety", {})
        self.motion_safety = MotionSafetyController(safety_settings)

    def execute(self, context: RobotContext) -> Any:
        started_at = time.monotonic()
        stop_before = self.motion_safety.stop("wait_start") if self.stop_motion else None

        event_data = None
        if self.wait_for == "USER_TOUCH" and self.event_type:
            status, event_data = self._wait_for_event(context, started_at)
        elif self.context_key:
            status = self._wait_for_context(context, started_at)
        else:
            delay = self.timeout_sec if self.wait_for == "TIMEOUT" and "timeout_sec" in self.config else self.seconds
            time.sleep(max(delay, 0.0))
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

    def _wait_for_event(self, context: RobotContext, started_at: float) -> tuple[str, Any]:
        deadline = started_at + max(self.timeout_sec, 0.0)
        while time.monotonic() <= deadline:
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
        for key in ("value", "action", "selectedBookId", "selected_book_id", "bookId", "book_id", "rating"):
            if key in value:
                return value[key]
        return value

    def _wait_for_context(self, context: RobotContext, started_at: float) -> str:
        deadline = started_at + max(self.timeout_sec, 0.0)
        while time.monotonic() <= deadline:
            value = context.get(str(self.context_key))
            if self._matches(value):
                return "waited"
            time.sleep(max(self.poll_interval_sec, 0.01))
        return "timeout"

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
