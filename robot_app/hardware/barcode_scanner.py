from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Dict, Optional


class BarcodeScanner:
    """Interface for barcode or QR code scanning devices."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        self.settings = settings or {}
        self.device = self.settings.get("device")
        self.mock_value = self.settings.get("mock_value")
        self.encoding = self.settings.get("encoding", "utf-8")

    def scan(self, timeout_sec: float = 5.0) -> Dict[str, Any]:
        started_at = time.monotonic()

        if self.mock_value not in (None, ""):
            return self._result("scanned", self.mock_value, "mock", started_at)

        if not self.device or self.device == "camera":
            return self._result("timeout", None, "unconfigured", started_at)

        if str(self.device).startswith("serial://"):
            port = str(self.device).replace("serial://", "", 1)
            return self._scan_serial(port, timeout_sec, started_at)

        path = Path(str(self.device))
        if path.exists():
            return self._scan_text_device(path, timeout_sec, started_at)

        return {
            "status": "error",
            "value": None,
            "source": str(self.device),
            "message": f"Scanner device not found: {self.device}",
            "duration_ms": int((time.monotonic() - started_at) * 1000),
        }

    def _scan_text_device(self, path: Path, timeout_sec: float, started_at: float) -> Dict[str, Any]:
        deadline = started_at + max(timeout_sec, 0.0)
        last_position = 0

        while time.monotonic() <= deadline:
            try:
                with path.open("r", encoding=self.encoding, errors="ignore") as file:
                    if last_position:
                        file.seek(last_position)
                    for line in file:
                        value = line.strip()
                        if value:
                            return self._result("scanned", value, str(path), started_at)
                    last_position = file.tell()
            except OSError as exc:
                return {
                    "status": "error",
                    "value": None,
                    "source": str(path),
                    "message": str(exc),
                    "duration_ms": int((time.monotonic() - started_at) * 1000),
                }
            time.sleep(0.05)

        return self._result("timeout", None, str(path), started_at)

    def _scan_serial(self, port: str, timeout_sec: float, started_at: float) -> Dict[str, Any]:
        try:
            import serial
        except ImportError:
            return {
                "status": "error",
                "value": None,
                "source": port,
                "message": "pyserial is required for serial barcode scanners.",
                "duration_ms": int((time.monotonic() - started_at) * 1000),
            }

        try:
            baudrate = int(self.settings.get("baudrate", 9600))
            with serial.Serial(port, baudrate, timeout=max(timeout_sec, 0.1)) as device:
                raw_line = device.readline()
        except (OSError, serial.SerialException) as exc:
            return {
                "status": "error",
                "value": None,
                "source": port,
                "message": str(exc),
                "duration_ms": int((time.monotonic() - started_at) * 1000),
            }

        value = raw_line.decode(self.encoding, errors="ignore").strip()
        if not value:
            return self._result("timeout", None, port, started_at)
        return self._result("scanned", value, port, started_at)

    def _result(self, status: str, value: Optional[str], source: str, started_at: float) -> Dict[str, Any]:
        return {
            "status": status,
            "value": value,
            "source": source,
            "duration_ms": int((time.monotonic() - started_at) * 1000),
        }
