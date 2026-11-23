#!/usr/bin/env python3
"""Capture labeled shelf images into an ImageFolder-compatible dataset."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlparse
import socket


def send_command(device: str, command: dict) -> dict:
    parsed = urlparse(device)
    if parsed.scheme != "imx500-tcp" or not parsed.hostname or parsed.port is None:
        raise ValueError("--device must look like imx500-tcp://127.0.0.1:9011")
    with socket.create_connection((parsed.hostname, parsed.port), timeout=4) as connection:
        connection.settimeout(8)
        connection.sendall((json.dumps(command) + "\n").encode("utf-8"))
        chunks = bytearray()
        while len(chunks) < 12 * 1024 * 1024:
            chunk = connection.recv(4096)
            if not chunk:
                break
            chunks.extend(chunk)
            if b"\n" in chunks:
                break
    return json.loads(bytes(chunks).split(b"\n", 1)[0])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="imx500-tcp://127.0.0.1:9011")
    parser.add_argument("--output", default="patrol-dataset")
    parser.add_argument("--label", required=True, choices=("NORMAL_SHELF", "LEANING_PRESENT"))
    parser.add_argument("--count", type=int, default=100, help="Number of frames to save.")
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between frames.")
    args = parser.parse_args()
    if args.count < 1 or args.interval < 0:
        parser.error("--count must be positive and --interval cannot be negative")

    destination = Path(args.output) / args.label
    destination.mkdir(parents=True, exist_ok=True)
    manifest = destination / "capture_manifest.jsonl"
    for index in range(args.count):
        frame = send_command(args.device, {"command": "capture"})
        if frame.get("status") != "captured" or not frame.get("image_base64"):
            raise RuntimeError(frame.get("message", "Camera did not return a frame."))
        import base64

        image = base64.b64decode(frame["image_base64"], validate=True)
        timestamp = datetime.now(timezone.utc)
        image_name = f"{timestamp.strftime('%Y%m%dT%H%M%S_%fZ')}_{index:04d}.jpg"
        (destination / image_name).write_bytes(image)
        with manifest.open("a", encoding="utf-8") as output:
            output.write(json.dumps({"image": image_name, "label": args.label, "captured_at_utc": timestamp.isoformat(), "frame_sequence": frame.get("sequence")}) + "\n")
        print(f"[{index + 1}/{args.count}] {destination / image_name}", flush=True)
        if index + 1 < args.count:
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
