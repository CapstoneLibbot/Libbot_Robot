from __future__ import annotations

import argparse
import json
import threading
import time
from socketserver import BaseRequestHandler, ThreadingTCPServer
from typing import Any


_camera_lock = threading.Lock()


def _imx500_cameras() -> list[tuple[int, dict[str, Any]]]:
    from picamera2 import Picamera2

    camera_info = Picamera2.global_camera_info()
    return [
        (index, info)
        for index, info in enumerate(camera_info)
        if "imx500" in str(info.get("Model", info.get("model", ""))).lower()
    ]


def scan_camera(timeout_sec: float) -> dict[str, Any]:
    camera = None
    started = time.monotonic()
    try:
        from picamera2 import Picamera2
        from PIL import Image
        from pyzbar.pyzbar import decode

        with _camera_lock:
            imx500_cameras = _imx500_cameras()
            if not imx500_cameras:
                return {"status": "error", "value": None, "source": "imx500", "message": "No IMX500 AI Camera was detected."}
            camera = Picamera2(imx500_cameras[0][0])
            camera.configure(camera.create_preview_configuration(
                main={"size": (1280, 720), "format": "RGB888"}
            ))
            camera.start()
            deadline = started + max(timeout_sec, 0.0)
            while time.monotonic() <= deadline:
                frame = camera.capture_array()
                codes = decode(Image.fromarray(frame))
                for code in codes:
                    value = code.data.decode("utf-8", errors="ignore").strip()
                    if value:
                        return {"status": "scanned", "value": value, "source": "imx500"}
                time.sleep(0.05)
        return {"status": "timeout", "value": None, "source": "imx500"}
    except Exception as exc:
        return {"status": "error", "value": None, "source": "imx500", "message": str(exc)}
    finally:
        if camera is not None:
            try:
                camera.stop()
                camera.close()
            except Exception:
                pass


class ScannerRequestHandler(BaseRequestHandler):
    def handle(self) -> None:
        try:
            request = self.rfile.readline(65536)
            payload = json.loads(request.decode("utf-8")) if request else {}
            if not isinstance(payload, dict):
                response = {"status": "error", "message": "Request must be a JSON object."}
            elif payload.get("command") == "ping":
                cameras = _imx500_cameras()
                response = {"status": "ok", "device": "imx500", "camera_count": len(cameras)} if cameras else {
                    "status": "error", "message": "No IMX500 AI Camera was detected."
                }
            elif payload.get("command") == "scan":
                try:
                    timeout_sec = min(max(float(payload.get("timeout_sec", 5)), 0.0), 120.0)
                    response = scan_camera(timeout_sec)
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Host camera adapter for the LiBot IMX500 AI Camera.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9011)
    args = parser.parse_args()
    with ScannerServer((args.host, args.port), ScannerRequestHandler) as server:
        print(f"IMX500 scanner listening on {args.host}:{args.port}", flush=True)
        server.serve_forever()


if __name__ == "__main__":
    main()
