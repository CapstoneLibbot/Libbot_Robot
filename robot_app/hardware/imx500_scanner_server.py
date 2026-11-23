from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import threading
import time
import uuid
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from socketserver import StreamRequestHandler, ThreadingTCPServer
from typing import Any

from hardware.qr_code_reader import decode_qr_codes


def _imx500_cameras() -> list[tuple[int, dict[str, Any]]]:
    from picamera2 import Picamera2

    camera_info = Picamera2.global_camera_info()
    return [
        (index, info)
        for index, info in enumerate(camera_info)
        if "imx500" in str(info.get("Model", info.get("model", ""))).lower()
    ]


class CameraFrames:
    """Owns the camera once and shares its frames with streaming and barcode scans."""

    def __init__(
        self,
        width: int = 1280,
        height: int = 720,
        jpeg_quality: int = 75,
        fps: float = 10.0,
        patrol_model: str | None = None,
        patrol_labels: str | None = None,
        patrol_threshold: float = 0.70,
        patrol_consecutive_frames: int = 3,
        patrol_rearm_clear_sec: float = 2.0,
    ):
        self.width = width
        self.height = height
        self.jpeg_quality = jpeg_quality
        self.fps = max(fps, 1.0)
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._frame: Any = None
        self._jpeg: bytes | None = None
        self._sequence = 0
        self._error: str | None = None
        self.patrol_model = patrol_model or os.getenv("LIBOT_PATROL_MODEL_RPK")
        self.patrol_labels = patrol_labels or os.getenv("LIBOT_PATROL_LABELS")
        self.patrol_threshold = min(max(float(patrol_threshold), 0.0), 1.0)
        self.patrol_consecutive_frames = max(int(patrol_consecutive_frames), 1)
        self.patrol_rearm_clear_sec = max(float(patrol_rearm_clear_sec), 0.1)
        self._imx500: Any = None
        self._patrol_model_error: str | None = None
        self._label_names: list[str] = []
        self._patrol_active = False
        self._patrol_events: deque[dict[str, Any]] = deque(maxlen=20)
        self._positive_frame_count = 0
        self._armed = True
        self._clear_since: float | None = None
        self._last_inference: dict[str, Any] | None = None

    def start(self) -> None:
        with self._condition:
            if self._thread is not None:
                return
            self._thread = threading.Thread(target=self._capture_loop, name="imx500-capture", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=5)

    def wait_for_frame(self, after_sequence: int = -1, timeout: float = 5.0) -> tuple[int, Any, bytes]:
        self.start()
        deadline = time.monotonic() + max(timeout, 0.0)
        with self._condition:
            while self._sequence <= after_sequence or self._jpeg is None:
                if self._error:
                    raise RuntimeError(self._error)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("Timed out waiting for an IMX500 frame.")
                self._condition.wait(remaining)
            return self._sequence, self._frame, self._jpeg

    def probe(self) -> dict[str, Any]:
        try:
            cameras = _imx500_cameras()
            return {"status": "ok", "device": "imx500", "camera_count": len(cameras)} if cameras else {
                "status": "error", "message": "No IMX500 AI Camera was detected."
            }
        except Exception as exc:
            return {"status": "error", "message": str(exc)}

    def patrol_status(self) -> dict[str, Any]:
        if not self.patrol_model:
            return {"status": "error", "detector": "unavailable", "message": "Patrol detector model is not configured (LIBOT_PATROL_MODEL_RPK)."}
        if not os.path.isfile(self.patrol_model):
            return {"status": "error", "detector": "unavailable", "message": f"Patrol model file was not found: {self.patrol_model}"}
        if self._patrol_model_error:
            return {"status": "error", "detector": "unavailable", "message": self._patrol_model_error}
        if not self._label_names:
            return {"status": "error", "detector": "unavailable", "message": "Patrol labels file is missing or invalid."}
        try:
            self.wait_for_frame(timeout=12.0)
        except Exception as exc:
            return {"status": "error", "detector": "unavailable", "message": str(exc)}
        with self._condition:
            ready = self._imx500 is not None and self._error is None
            return {"status": "ready" if ready else "error", "detector": "imx500_binary_classifier",
                    "model": os.path.basename(self.patrol_model), "labels": self._label_names,
                    "threshold": self.patrol_threshold,
                    "message": None if ready else (self._error or "IMX500 patrol model did not load.")}

    def begin_patrol(self, threshold: float | None = None) -> dict[str, Any]:
        status = self.patrol_status()
        if status.get("status") != "ready":
            return status
        if threshold is not None:
            try:
                requested_threshold = float(threshold)
            except (TypeError, ValueError):
                return {"status": "error", "message": "Patrol threshold must be numeric."}
            if not 0.0 <= requested_threshold <= 1.0:
                return {"status": "error", "message": "Patrol threshold must be between 0 and 1."}
            self.patrol_threshold = requested_threshold
        with self._condition:
            self._patrol_active = True
            self._patrol_events.clear()
            self._positive_frame_count = 0
            self._armed = True
            self._clear_since = None
        return {"status": "ready", "detector": status.get("detector"), "threshold": self.patrol_threshold}

    def end_patrol(self) -> dict[str, Any]:
        with self._condition:
            self._patrol_active = False
            self._positive_frame_count = 0
            self._armed = True
            self._clear_since = None
        return {"status": "ok"}

    def next_patrol_detection(self) -> dict[str, Any]:
        with self._condition:
            if self._patrol_model_error:
                return {"status": "error", "message": self._patrol_model_error}
            if self._error:
                return {"status": "error", "message": self._error}
            event = self._patrol_events[0] if self._patrol_events else None
            return {"status": "detected", "detection": event} if event else {"status": "no_data"}

    def acknowledge_patrol_detection(self, observation_id: str) -> dict[str, Any]:
        with self._condition:
            if self._patrol_events and self._patrol_events[0].get("observation_id") == observation_id:
                self._patrol_events.popleft()
                return {"status": "ok"}
            return {"status": "not_found"}

    def capture(self) -> dict[str, Any]:
        sequence, _, jpeg = self.wait_for_frame(timeout=5.0)
        return {"status": "captured", "sequence": sequence, "image_base64": base64.b64encode(jpeg).decode("ascii")}

    def scan(
        self,
        timeout_sec: float,
        scan_type: str = "BARCODE",
        multiple: bool = False,
        quiet_period_sec: float = 0.75,
    ) -> dict[str, Any]:
        started = time.monotonic()
        sequence = -1
        try:
            deadline = started + max(timeout_sec, 0.0)
            last_new_code_at = started
            items: dict[tuple[str, int], dict[str, Any]] = {}
            while time.monotonic() <= deadline:
                remaining = deadline - time.monotonic()
                sequence, frame, _ = self.wait_for_frame(sequence, min(max(remaining, 0.05), 2.0))
                if str(scan_type).upper() == "QR":
                    for item in decode_qr_codes(frame):
                        key = (item["qr_code"].casefold(), item["level_number"])
                        if key not in items:
                            items[key] = item
                            last_new_code_at = time.monotonic()
                    if multiple and items and time.monotonic() - last_new_code_at >= max(quiet_period_sec, 0.1):
                        break
                    if not multiple and items:
                        return {"status": "scanned", "value": next(iter(items.values()))["qr_code"], "source": "imx500"}
                else:
                    from PIL import Image
                    from pyzbar.pyzbar import decode

                    for code in decode(Image.fromarray(frame)):
                        value = code.data.decode("utf-8", errors="ignore").strip()
                        if value:
                            return {"status": "scanned", "value": value, "source": "imx500"}
            if multiple:
                result_items = list(items.values())
                return {
                    "status": "scanned" if result_items else "no_data",
                    "value": None,
                    "items": result_items,
                    "source": "imx500",
                    "duration_ms": int((time.monotonic() - started) * 1000),
                }
            return {"status": "timeout", "value": None, "source": "imx500"}
        except TimeoutError:
            return {"status": "timeout", "value": None, "source": "imx500"}
        except Exception as exc:
            return {"status": "error", "value": None, "source": "imx500", "message": str(exc)}

    def _capture_loop(self) -> None:
        camera = None
        try:
            from picamera2 import Picamera2
            from PIL import Image

            cameras = _imx500_cameras()
            if not cameras:
                raise RuntimeError("No IMX500 AI Camera was detected.")
            imx500 = None
            if self.patrol_model:
                try:
                    if not os.path.isfile(self.patrol_model):
                        raise RuntimeError(f"Patrol model file was not found: {self.patrol_model}")
                    labels_path = self.patrol_labels or f"{self.patrol_model}.labels.json"
                    if not os.path.isfile(labels_path):
                        raise RuntimeError(f"Patrol labels file was not found: {labels_path}")
                    with open(labels_path, "r", encoding="utf-8") as labels_file:
                        labels_data = json.load(labels_file)
                    labels = labels_data.get("classes") if isinstance(labels_data, dict) else labels_data
                    if not isinstance(labels, list) or not labels or not all(isinstance(label, str) for label in labels):
                        raise RuntimeError("Patrol labels file must be a JSON array or an object with a string 'classes' array.")
                    if "LEANING_PRESENT" not in labels or "NORMAL_SHELF" not in labels:
                        raise RuntimeError("Patrol labels must include LEANING_PRESENT and NORMAL_SHELF.")
                    from picamera2.devices.imx500 import IMX500

                    imx500 = IMX500(self.patrol_model)
                    self._label_names = labels
                except Exception as exc:
                    self._patrol_model_error = str(exc)
            camera = Picamera2(cameras[0][0])
            camera.configure(camera.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            ))
            if imx500 is not None:
                try:
                    imx500.show_network_fw_progress_bar()
                except Exception as exc:
                    self._patrol_model_error = f"Could not initialize the IMX500 patrol model: {exc}"
                    imx500 = None
                    self._label_names = []
            camera.start()
            with self._condition:
                self._imx500 = imx500
                self._condition.notify_all()
            while not self._stop.is_set():
                captured_at = time.monotonic()
                request = camera.capture_request()
                try:
                    frame = request.make_array("main")
                    metadata = request.get_metadata()
                finally:
                    request.release()
                output = io.BytesIO()
                Image.fromarray(frame).save(output, format="JPEG", quality=self.jpeg_quality)
                jpeg = output.getvalue()
                with self._condition:
                    self._frame = frame
                    self._jpeg = jpeg
                    self._sequence += 1
                    if imx500 is not None:
                        self._process_patrol_output(imx500, metadata, jpeg)
                    self._condition.notify_all()
                self._stop.wait(max(0.0, (1.0 / self.fps) - (time.monotonic() - captured_at)))
        except Exception as exc:
            with self._condition:
                self._error = str(exc)
                self._imx500 = None
                self._condition.notify_all()
        finally:
            if camera is not None:
                try:
                    camera.stop()
                    camera.close()
                except Exception:
                    pass

    def _process_patrol_output(self, imx500: Any, metadata: dict[str, Any], jpeg: bytes) -> None:
        if not self._patrol_active:
            return
        try:
            outputs = imx500.get_outputs(metadata)
            if not outputs:
                return
            raw_scores = outputs[0]
            if hasattr(raw_scores, "tolist"):
                raw_scores = raw_scores.tolist()
            scores: list[float] = []

            def flatten(values: Any) -> None:
                if isinstance(values, (list, tuple)):
                    for value in values:
                        flatten(value)
                else:
                    scores.append(float(values))

            flatten(raw_scores)
            if len(scores) != len(self._label_names):
                raise RuntimeError(f"Patrol model returned {len(scores)} scores for {len(self._label_names)} labels.")
            if min(scores) < 0.0 or max(scores) > 1.0 or abs(sum(scores) - 1.0) > 0.2:
                shifted = [score - max(scores) for score in scores]
                exponentials = [math.exp(score) for score in shifted]
                total = sum(exponentials)
                scores = [score / total for score in exponentials]
            best_index = max(range(len(scores)), key=scores.__getitem__)
            label = self._label_names[best_index]
            confidence = scores[best_index]
            positive = label == "LEANING_PRESENT" and confidence >= self.patrol_threshold
            now = time.monotonic()
            self._last_inference = {"label": label, "confidence": confidence}
            if positive:
                self._clear_since = None
                self._positive_frame_count += 1
                if self._armed and self._positive_frame_count >= self.patrol_consecutive_frames:
                    self._armed = False
                    self._patrol_events.append({
                        "observation_id": str(uuid.uuid4()),
                        "category": "LEANING_BOOK_PRESENT",
                        "confidence": confidence,
                        "observed_at_utc": datetime.now(timezone.utc).isoformat(),
                        "image_base64": base64.b64encode(jpeg).decode("ascii"),
                    })
            else:
                self._positive_frame_count = 0
                if self._clear_since is None:
                    self._clear_since = now
                elif not self._armed and now - self._clear_since >= self.patrol_rearm_clear_sec:
                    self._armed = True
        except Exception as exc:
            self._patrol_model_error = f"Patrol inference failed: {exc}"
            self._patrol_active = False


class ScannerRequestHandler(StreamRequestHandler):
    def handle(self) -> None:
        camera: CameraFrames = self.server.camera  # type: ignore[attr-defined]
        try:
            request = self.rfile.readline(65536)
            payload = json.loads(request.decode("utf-8")) if request else {}
            if not isinstance(payload, dict):
                response = {"status": "error", "message": "Request must be a JSON object."}
            elif payload.get("command") == "ping":
                response = camera.probe()
            elif payload.get("command") == "patrol_status":
                response = camera.patrol_status()
            elif payload.get("command") == "patrol_begin":
                response = camera.begin_patrol(payload.get("threshold"))
            elif payload.get("command") == "patrol_end":
                response = camera.end_patrol()
            elif payload.get("command") == "patrol_detection":
                response = camera.next_patrol_detection()
            elif payload.get("command") == "patrol_detection_ack":
                response = camera.acknowledge_patrol_detection(str(payload.get("observation_id", "")))
            elif payload.get("command") == "capture":
                response = camera.capture()
            elif payload.get("command") == "scan":
                try:
                    timeout_sec = min(max(float(payload.get("timeout_sec", 5)), 0.0), 120.0)
                    quiet_period_sec = min(max(float(payload.get("quiet_period_sec", 0.75)), 0.1), 5.0)
                    scan_type = str(payload.get("scan_type", "BARCODE")).upper()
                    if scan_type not in {"BARCODE", "QR", "RFID", "CAMERA"}:
                        response = {"status": "error", "message": f"Unsupported scan_type '{scan_type}'."}
                    else:
                        response = camera.scan(timeout_sec, scan_type, bool(payload.get("multiple", False)), quiet_period_sec)
                except (TypeError, ValueError):
                    response = {"status": "error", "message": "timeout_sec and quiet_period_sec must be numeric."}
            else:
                response = {"status": "error", "message": "Unsupported scanner command."}
        except Exception as exc:
            response = {"status": "error", "message": str(exc)}
        self.wfile.write(json.dumps(response, ensure_ascii=False).encode("utf-8") + b"\n")


class ScannerServer(ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], camera: CameraFrames):
        self.camera = camera
        super().__init__(address, ScannerRequestHandler)


class CameraStreamHandler(BaseHTTPRequestHandler):
    server_version = "LiBotCamera/1.0"

    def do_GET(self) -> None:
        server: CameraStreamServer = self.server  # type: ignore[assignment]
        if server.token and self.headers.get("Authorization") != f"Bearer {server.token}":
            self.send_error(401, "Camera stream token is missing or invalid.")
            return
        if self.path == "/health":
            payload = json.dumps(server.camera.probe()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        if self.path == "/snapshot.jpg":
            try:
                _, _, jpeg = server.camera.wait_for_frame(timeout=5)
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(jpeg)
            except Exception as exc:
                self.send_error(503, str(exc))
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        sequence = -1
        try:
            while True:
                sequence, _, jpeg = server.camera.wait_for_frame(sequence, timeout=10)
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode("ascii") + b"\r\n\r\n" + jpeg + b"\r\n"
                )
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            print(f"Camera stream ended: {exc}", flush=True)

    def log_message(self, format: str, *args: Any) -> None:
        print(f"Camera HTTP {self.address_string()}: {format % args}", flush=True)


class CameraStreamServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], camera: CameraFrames, token: str | None):
        self.camera = camera
        self.token = token
        super().__init__(address, CameraStreamHandler)


def main() -> None:
    parser = argparse.ArgumentParser(description="Host scanner and realtime stream for the LiBot IMX500 AI Camera.")
    parser.add_argument("--host", default="127.0.0.1", help="Barcode scanner TCP bind address.")
    parser.add_argument("--port", type=int, default=9011, help="Barcode scanner TCP port.")
    parser.add_argument("--stream-host", default="0.0.0.0", help="Realtime camera HTTP bind address.")
    parser.add_argument("--stream-port", type=int, default=9012, help="Realtime camera HTTP port.")
    parser.add_argument("--stream-fps", type=float, default=10.0, help="MJPEG frame rate (default: 10).")
    parser.add_argument("--stream-token", default=os.getenv("LIBOT_CAMERA_STREAM_TOKEN"))
    parser.add_argument("--patrol-model", default=os.getenv("LIBOT_PATROL_MODEL_RPK"), help="Optional IMX500 RPK binary classifier for patrol scans.")
    parser.add_argument("--patrol-labels", default=os.getenv("LIBOT_PATROL_LABELS"), help="JSON labels file (defaults to <RPK>.labels.json).")
    parser.add_argument("--patrol-threshold", type=float, default=0.70)
    parser.add_argument("--patrol-consecutive-frames", type=int, default=3)
    parser.add_argument("--patrol-rearm-clear-sec", type=float, default=2.0)
    args = parser.parse_args()

    camera = CameraFrames(fps=args.stream_fps, patrol_model=args.patrol_model, patrol_labels=args.patrol_labels,
                          patrol_threshold=args.patrol_threshold, patrol_consecutive_frames=args.patrol_consecutive_frames,
                          patrol_rearm_clear_sec=args.patrol_rearm_clear_sec)
    scanner = ScannerServer((args.host, args.port), camera)
    stream = CameraStreamServer((args.stream_host, args.stream_port), camera, args.stream_token)
    stream_thread = threading.Thread(target=stream.serve_forever, name="camera-http", daemon=True)
    stream_thread.start()
    print(f"IMX500 scanner listening on {args.host}:{args.port}", flush=True)
    print(f"IMX500 realtime stream listening on {args.stream_host}:{args.stream_port}", flush=True)
    if not args.stream_token and args.stream_host not in {"127.0.0.1", "::1", "localhost"}:
        print("WARNING: realtime camera stream has no token; restrict port 9012 to the backend host.", flush=True)
    try:
        scanner.serve_forever()
    finally:
        scanner.server_close()
        stream.shutdown()
        stream.server_close()
        camera.stop()


if __name__ == "__main__":
    main()
