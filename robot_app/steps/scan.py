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
        self.scan_type = str(self.config.get("scan_type", "BARCODE")).upper()
        self.target_type = str(self.config.get("target_type", "BOOK")).upper()
        self.continuous = bool(self.config.get("continuous", False))
        self.timeout_sec = float(self.config.get("timeout_sec", 5.0))
        self.context_key = self.config.get("context_key", "scan_input")
        self.output_key = self.config.get("output_key", "scanned_code")
        self.expected_prefix = self.config.get("expected_prefix")
        self.expected_value = self.config.get("expected_value")
        scanner_settings = {
            "device": self.config.get("device", self.sensor),
            "mock_value": self.config.get("mock_value"),
            "encoding": self.config.get("encoding", "utf-8"),
            "baudrate": self.config.get("baudrate", 9600),
            "quiet_period_sec": self.config.get("quiet_period_sec", 0.75),
        }
        self.scanner = BarcodeScanner(scanner_settings)
    def execute(self, context: RobotContext) -> Any:
        if not self.config.get("_inventory_shelf_scan"):
            return self._scan_once(context)
        if context.get("workflow_cancelled") or context.get("low_battery_preempt_requested"):
            return {"status": "error", "type": "SCAN", "scan_status": "FAILED", "message": "Inventory scan cancelled."}
        result = self._scan_once(context)
        if result.get("status") != "scanned":
            return result
        previous = context.get("inventory_scanned_items")
        if not isinstance(previous, list):
            return {"status": "error", "type": "SCAN", "scan_status": "FAILED", "message": "Inventory scan state is missing."}
        items = list(previous)
        seen = {(item["qr_code"].casefold(), item["level_number"]) for item in items}
        for item in result.get("scanned_items", []):
            key = (item["qr_code"].casefold(), item["level_number"])
            if key not in seen:
                seen.add(key)
                items.append(item)
        context.set("inventory_scanned_items", items)
        if context.get("inventory_scan_complete") and not items:
            return {"status": "error", "type": "SCAN", "scan_status": "FAILED",
                    "message": "No books were read across the shelf; inventory cannot be verified safely."}
        result["scanned_items"] = items
        result["scanned_count"] = len(items)
        result["scan_status"] = "COMPLETED" if items else "NO_DATA"
        context.set("last_scan", result)
        return result

    def _scan_once(self, context: RobotContext) -> Any:
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
            scan_result = self.scanner.scan(
                self.timeout_sec,
                scan_type=self.scan_type,
                multiple=self.continuous and self.scan_type == "QR" and self.target_type == "BOOK",
            )

        if self.scan_type == "QR" and self.target_type == "BOOK":
            return self._execute_qr_book_scan(scan_result, context, started_at)

        value = scan_result.get("value")
        expected_value = self.resolve(context, self.expected_value) if self.expected_value is not None else None
        validation_error = (
            "Expected scan value could not be resolved."
            if self.expected_value is not None and expected_value in (None, "")
            else self._validate(value, expected_value)
        )
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
        if isinstance(context_value, list):
            return {"status": "scanned", "items": context_value, "source": self.context_key}
        if isinstance(context_value, dict):
            value = context_value.get("items", context_value.get("scanned_items",
                    context_value.get("value", context_value.get("result"))))
            if value is None and self.sensor == "patrol_observation":
                value = context_value
            return {
                "status": context_value.get("status", "scanned"),
                "value": value,
                "items": value if isinstance(value, list) else context_value.get("items"),
                "source": self.context_key,
                "message": context_value.get("message"),
            }
        return {
            "status": "scanned",
            "value": str(context_value),
            "source": self.context_key,
        }

    def _execute_qr_book_scan(
        self,
        scan_result: Dict[str, Any],
        context: RobotContext,
        started_at: float,
    ) -> Dict[str, Any]:
        raw_items = scan_result.get("items")
        if raw_items is None:
            raw_items = scan_result.get("value")
        if raw_items is None:
            raw_items = []
        if isinstance(raw_items, (str, dict)):
            raw_items = [raw_items]
        if not isinstance(raw_items, list):
            return self._qr_scan_error("QR scanner returned an invalid item list.", started_at, scan_result)

        items: list[dict[str, Any]] = []
        seen: set[tuple[str, int]] = set()
        for raw_item in raw_items:
            if isinstance(raw_item, str):
                qr_code, level_number = raw_item.strip(), 1
                center_x = center_y = None
            elif isinstance(raw_item, dict):
                qr_code = str(raw_item.get("qr_code", raw_item.get("qrCode", raw_item.get("value", "")))).strip()
                center_x = raw_item.get("center_x", raw_item.get("centerX"))
                center_y = raw_item.get("center_y", raw_item.get("centerY"))
                level_number = raw_item.get("level_number", raw_item.get("levelNumber"))
                if level_number is None and isinstance(center_y, (int, float)):
                    level_number = 2 if center_y < 360 else 1
                if level_number is None:
                    level_number = 1
            else:
                return self._qr_scan_error("QR scanner returned an item with an invalid shape.", started_at, scan_result)

            if not qr_code:
                continue
            try:
                level_number = int(level_number)
            except (TypeError, ValueError):
                return self._qr_scan_error("QR item has an invalid level number.", started_at, scan_result)
            if level_number not in (1, 2):
                return self._qr_scan_error("QR item level number must be 1 or 2.", started_at, scan_result)
            validation_error = self._validate(qr_code)
            if validation_error:
                return self._qr_scan_error(validation_error, started_at, scan_result)
            key = (qr_code.casefold(), level_number)
            if key in seen:
                continue
            seen.add(key)
            items.append({
                "qr_code": qr_code,
                "level_number": level_number,
                "center_x": center_x,
                "center_y": center_y,
            })

        scanner_status = str(scan_result.get("status", "timeout")).lower()
        if scanner_status == "error":
            return self._qr_scan_error(scan_result.get("message") or "QR camera scan failed.", started_at, scan_result)

        scan_status = "COMPLETED" if items else "NO_DATA"
        payload = {
            "status": "scanned",
            "type": "SCAN",
            "sensor": self.sensor,
            "scan_type": "QR",
            "target_type": "BOOK",
            "scan_status": scan_status,
            "result": items if items else "no_data",
            "value": None,
            "scanned_items": items,
            "scanned_count": len(items),
            "source": scan_result.get("source", self.sensor),
            "message": scan_result.get("message"),
            "duration_ms": int((time.monotonic() - started_at) * 1000),
        }
        context.set("last_scan", payload)
        context.set(self.output_key, items)
        return payload

    def _qr_scan_error(self, message: str, started_at: float, scan_result: Dict[str, Any]) -> Dict[str, Any]:
        payload = {
            "status": "error",
            "type": "SCAN",
            "sensor": self.sensor,
            "scan_type": "QR",
            "target_type": "BOOK",
            "scan_status": "FAILED",
            "result": "error",
            "value": None,
            "scanned_items": [],
            "scanned_count": 0,
            "source": scan_result.get("source", self.sensor),
            "message": str(message),
            "duration_ms": int((time.monotonic() - started_at) * 1000),
        }
        return payload

    def _validate(self, value: Any, expected_value: Any = None) -> Optional[str]:
        if value in (None, ""):
            return None
        if expected_value is not None and str(value) != str(expected_value):
            return "Scanned barcode does not match the selected book."
        if self.expected_prefix and not str(value).startswith(str(self.expected_prefix)):
            return f"Scanned value does not start with expected prefix '{self.expected_prefix}'."
        return None
