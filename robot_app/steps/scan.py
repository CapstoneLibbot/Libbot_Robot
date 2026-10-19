from __future__ import annotations

import time
from typing import Any, Dict, Optional

from core.context import RobotContext
from hardware.barcode_scanner import BarcodeScanner
from steps.base_step import BaseStep


class ScanStep(BaseStep):
    """Step Type: SCAN"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.sensor = self.config.get("sensor", "camera")
        self.timeout_sec = float(self.config.get("timeout_sec", 5.0))
        self.context_key = self.config.get("context_key", "scan_input")
        self.output_key = self.config.get("output_key", "scanned_code")
        self.expected_prefix = self.config.get("expected_prefix")
        scanner_settings = {
            "device": self.config.get("device", self.sensor),
            "mock_value": self.config.get("mock_value"),
            "encoding": self.config.get("encoding", "utf-8"),
            "baudrate": self.config.get("baudrate", 9600),
        }
        self.scanner = BarcodeScanner(scanner_settings)

    def execute(self, context: RobotContext) -> Any:
        started_at = time.monotonic()
        default_queue = "patrol_observation_queue" if self.sensor == "patrol_observation" else "scan_queue"
        queue_key = str(self.config.get("queue_key", default_queue))
        runtime_id = str(context.get("runtime_id", "") or "").lower()
        context_value = context.pop_next(queue_key) if runtime_id == "sim" else None
        if runtime_id != "sim" and self.sensor != "patrol_observation":
            # Legacy one-step callers that do not name a runtime may inject an
            # explicit test/manual value; dispatched missions always set runtime_id.
            context_value = context.get(self.context_key) if not runtime_id else None
        if context_value is None and self.sensor == "patrol_observation":
            context_value = context.get(self.context_key)
        scan_result = self._from_context(context_value)
        if scan_result is None and (runtime_id == "sim" or self.sensor == "patrol_observation"):
            scan_result = {"status": "timeout", "value": None, "source": "simulated" if runtime_id == "sim" else self.sensor}
        if scan_result is None:
            scan_result = self.scanner.scan(self.timeout_sec)

        value = scan_result.get("value")
        validation_error = self._validate(value)
        status = scan_result.get("status", "timeout")
        if validation_error is not None:
            status = "error"

        payload = {
            "status": status,
            "type": "SCAN",
            "sensor": self.sensor,
            "result": value if value is not None else "no_data",
            "value": value,
            "source": scan_result.get("source", self.sensor),
            "message": validation_error or scan_result.get("message"),
            "duration_ms": int((time.monotonic() - started_at) * 1000),
        }
        context.set("last_scan", payload)
        if status == "scanned":
            context.set(self.output_key, value)
        return payload

    def _from_context(self, context_value: Any) -> Optional[Dict[str, Any]]:
        if context_value in (None, ""):
            return None
        if isinstance(context_value, dict):
            value = context_value.get("value", context_value.get("result"))
            if value is None and self.sensor == "patrol_observation":
                value = context_value
            return {
                "status": context_value.get("status", "scanned"),
                "value": value,
                "source": self.context_key,
                "message": context_value.get("message"),
            }
        return {
            "status": "scanned",
            "value": str(context_value),
            "source": self.context_key,
        }

    def _validate(self, value: Any) -> Optional[str]:
        if value in (None, ""):
            return None
        if self.expected_prefix and not str(value).startswith(str(self.expected_prefix)):
            return f"Scanned value does not start with expected prefix '{self.expected_prefix}'."
        return None
