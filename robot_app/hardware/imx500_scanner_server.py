from __future__ import annotations

import argparse
import io
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from socketserver import StreamRequestHandler, ThreadingTCPServer
from typing import Any


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

    def __init__(self, width: int = 1280, height: int = 720, jpeg_quality: int = 75, fps: float = 10.0):
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

    def scan(self, timeout_sec: float) -> dict[str, Any]:
        started = time.monotonic()
        sequence = -1
        try:
            from PIL import Image
            from pyzbar.pyzbar import decode

            deadline = started + max(timeout_sec, 0.0)
            while time.monotonic() <= deadline:
                remaining = deadline - time.monotonic()
                sequence, frame, _ = self.wait_for_frame(sequence, min(max(remaining, 0.05), 2.0))
                for code in decode(Image.fromarray(frame)):
                    value = code.data.decode("utf-8", errors="ignore").strip()
                    if value:
                        return {"status": "scanned", "value": value, "source": "imx500"}
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
            camera = Picamera2(cameras[0][0])
            camera.configure(camera.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            ))
            camera.start()
            while not self._stop.is_set():
                captured_at = time.monotonic()
                frame = camera.capture_array()
                output = io.BytesIO()
                Image.fromarray(frame).save(output, format="JPEG", quality=self.jpeg_quality)
                with self._condition:
                    self._frame = frame
                    self._jpeg = output.getvalue()
                    self._sequence += 1
                    self._condition.notify_all()
                self._stop.wait(max(0.0, (1.0 / self.fps) - (time.monotonic() - captured_at)))
        except Exception as exc:
            with self._condition:
                self._error = str(exc)
                self._condition.notify_all()
        finally:
            if camera is not None:
                try:
                    camera.stop()
                    camera.close()
                except Exception:
                    pass


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
            elif payload.get("command") == "scan":
                try:
                    timeout_sec = min(max(float(payload.get("timeout_sec", 5)), 0.0), 120.0)
                    response = camera.scan(timeout_sec)
                except (TypeError, ValueError):
                    response = {"status": "error", "message": "timeout_sec must be numeric."}
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
    args = parser.parse_args()

    camera = CameraFrames(fps=args.stream_fps)
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
