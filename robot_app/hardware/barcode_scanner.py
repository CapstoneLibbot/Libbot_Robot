from __future__ import annotations

import time
import json
import socket
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import urlparse


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

        if self.device == "imx500":
            return self._scan_imx500(timeout_sec, started_at)
        if str(self.device or "").startswith("imx500-tcp://"):
            return self._scan_imx500_tcp(timeout_sec, started_at)

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

    def probe(self) -> Dict[str, Any]:
        if str(self.device or "").startswith("imx500-tcp://"):
            try:
                host, port = self._scanner_address(str(self.device))
                with socket.create_connection((host, port), timeout=2) as connection:
                    connection.sendall(b'{"command":"ping"}\n')
                    response = self._read_json_line(connection)
                return {"status": "ready"} if response.get("status") == "ok" else {
                    "status": "error", "message": response.get("message", "IMX500 scanner service is unavailable.")
                }
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                return {"status": "error", "message": f"IMX500 scanner service unavailable: {exc}"}
        if self.device != "imx500":
            return {"status": "error", "message": "Scanner device must be configured as imx500."}
        try:
            from picamera2 import Picamera2

            cameras = Picamera2.global_camera_info()
            if not cameras:
                return {"status": "error", "message": "No Raspberry Pi camera was detected."}
            return {"status": "ready", "camera_count": len(cameras), "camera": cameras[0]}
        except Exception as exc:
            return {"status": "error", "message": f"IMX500 camera unavailable: {exc}"}

    def _scan_imx500_tcp(self, timeout_sec: float, started_at: float) -> Dict[str, Any]:
        try:
            host, port = self._scanner_address(str(self.device))
            with socket.create_connection((host, port), timeout=3) as connection:
                connection.settimeout(max(timeout_sec, 0.5) + 3)
                connection.sendall((json.dumps({"command": "scan", "timeout_sec": timeout_sec}) + "\n").encode("utf-8"))
                response = self._read_json_line(connection)
            response.setdefault("source", "imx500")
            response.setdefault("duration_ms", int((time.monotonic() - started_at) * 1000))
            return response
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            return {
                "status": "error", "value": None, "source": "imx500",
                "message": f"IMX500 scanner service failed: {exc}",
                "duration_ms": int((time.monotonic() - started_at) * 1000),
            }

    @staticmethod
    def _scanner_address(device: str) -> tuple[str, int]:
        parsed = urlparse(device)
        if parsed.scheme != "imx500-tcp" or not parsed.hostname or parsed.port is None:
            raise ValueError("Expected imx500-tcp://host:port scanner URL.")
        return parsed.hostname, parsed.port

    @staticmethod
    def _read_json_line(connection: socket.socket) -> Dict[str, Any]:
        chunks = bytearray()
        while len(chunks) < 65536:
            data = connection.recv(4096)
            if not data:
                break
            chunks.extend(data)
            if b"\n" in chunks:
                break
        line = bytes(chunks).split(b"\n", 1)[0].decode("utf-8")
        response = json.loads(line)
        if not isinstance(response, dict):
            raise ValueError("Scanner service response must be a JSON object.")
        return response

    def _scan_imx500(self, timeout_sec: float, started_at: float) -> Dict[str, Any]:
        camera = None
        try:
            from picamera2 import Picamera2
            from PIL import Image
            from pyzbar.pyzbar import decode

            camera = Picamera2()
            camera.configure(camera.create_preview_configuration(
                main={"size": (1280, 720), "format": "RGB888"}
            ))
            camera.start()
            deadline = started_at + max(timeout_sec, 0.0)
            while time.monotonic() <= deadline:
                frame = camera.capture_array()
                codes = decode(Image.fromarray(frame))
                if codes:
                    value = codes[0].data.decode(self.encoding, errors="ignore").strip()
                    if value:
                        return self._result("scanned", value, "imx500", started_at)
                time.sleep(0.05)
            return self._result("timeout", None, "imx500", started_at)
        except Exception as exc:
            return {
                "status": "error",
                "value": None,
                "source": "imx500",
                "message": f"IMX500 barcode scan failed: {exc}",
                "duration_ms": int((time.monotonic() - started_at) * 1000),
            }
        finally:
            if camera is not None:
                try:
                    camera.stop()
                    camera.close()
                except Exception:
                    pass

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
