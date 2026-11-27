from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional
from urllib import request

from hardware.barcode_scanner import BarcodeScanner


class PatrolDetectionReporter:
    """Streams camera detection episodes and their JPEG evidence to the LiBot API."""

    def __init__(
        self,
        scanner_device: str,
        backend_base_url: str,
        job_id: str,
        execution_id: str,
        path_id: str,
        waypoint: Any,
        threshold: float = 0.70,
    ) -> None:
        self.scanner = BarcodeScanner({"device": scanner_device})
        self.backend_base_url = backend_base_url.rstrip("/")
        self.job_id = job_id
        self.execution_id = execution_id
        self.path_id = path_id
        self.waypoint = waypoint
        self.threshold = threshold
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.last_error: Optional[str] = None
        self.started = False

    def start(self) -> bool:
        if not self.backend_base_url or not self.job_id or not self.execution_id or not self.path_id:
            self.last_error = "Patrol evidence upload requires backend, job, execution, and path identifiers."
            return False
        try:
            result = self.scanner.patrol_command("patrol_begin", threshold=self.threshold)
            if result.get("status") != "ready":
                self.last_error = str(result.get("message") or "Patrol detector did not become ready.")
                return False
        except Exception as exc:
            self.last_error = f"Could not start camera patrol detection: {exc}"
            return False
        self.started = True
        self._thread = threading.Thread(target=self._run, name="patrol-detection-uploader", daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        if not self.started:
            return
        try:
            self.scanner.patrol_command("patrol_end")
        except Exception as exc:
            self.last_error = self.last_error or f"Could not stop camera patrol detection: {exc}"
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=14.0)
        self.started = False

    def _run(self) -> None:
        drain_deadline: Optional[float] = None
        while True:
            if self._stop.is_set() and drain_deadline is None:
                drain_deadline = time.monotonic() + 3.0
            if drain_deadline is not None and time.monotonic() >= drain_deadline:
                break
            try:
                result = self.scanner.patrol_command("patrol_detection")
                if result.get("status") == "error":
                    self.last_error = str(result.get("message") or "Camera patrol inference failed.")
                    time.sleep(0.3)
                    continue
                detection = result.get("detection")
                if not isinstance(detection, dict):
                    if self._stop.is_set():
                        break
                    self._stop.wait(0.2)
                    continue
                payload = {
                    "jobId": self.job_id,
                    "runtimeId": "real",
                    "executionId": self.execution_id,
                    "pathId": self.path_id,
                    "observationId": detection.get("observation_id"),
                    "confidence": detection.get("confidence"),
                    "tiltDegrees": detection.get("tilt_degrees"),
                    "observedAtUtc": detection.get("observed_at_utc"),
                    "waypoint": self.waypoint,
                    "imageBase64": detection.get("image_base64"),
                }
                self._post_detection(payload)
                self.scanner.patrol_command("patrol_detection_ack", observation_id=detection.get("observation_id"))
                self.last_error = None
            except Exception as exc:
                self.last_error = f"Could not upload patrol detection evidence: {exc}"
                self._stop.wait(0.5)

    def _post_detection(self, payload: Dict[str, Any]) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        req = request.Request(
            f"{self.backend_base_url}/api/workflow-data/patrol/evidence",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with request.urlopen(req, timeout=12.0) as response:
            if not 200 <= response.status < 300:
                raise RuntimeError(f"Patrol evidence API returned HTTP {response.status}.")

