"""Roboflow segmentation and image-plane leaning-book detection for patrol."""
from __future__ import annotations

import math
import os
import tempfile
from typing import Any


def create_client(api_key: str) -> Any:
    from inference_sdk import InferenceHTTPClient, InferenceConfiguration
    return InferenceHTTPClient(api_url="https://serverless.roboflow.com", api_key=api_key).configure(
        InferenceConfiguration(api_key_transport="header", workflow_run_retries_enabled=False)
    )


def tilt_from_mask(rle: dict[str, Any]) -> float | None:
    import numpy as np
    from pycocotools import mask as mask_util

    size = rle.get("size")
    counts = rle.get("counts")
    if not isinstance(size, list) or len(size) != 2 or not isinstance(counts, str):
        raise ValueError("Invalid segmentation mask")
    if any(not isinstance(x, int) or x <= 0 for x in size) or size[0] * size[1] > 20_000_000:
        raise ValueError("Invalid segmentation dimensions")
    mask = mask_util.decode({"size": size, "counts": counts.encode("ascii")})
    y, x = np.nonzero(mask)
    if len(x) < 100:
        return None
    covariance = np.cov(np.stack((x, y)))
    values, vectors = np.linalg.eigh(covariance)
    if values[0] <= 0 or values[1] / values[0] < 1.5:
        return None
    axis = vectors[:, 1]
    return math.degrees(math.atan2(abs(float(axis[0])), abs(float(axis[1]))))


def detect_leaning_book(client: Any, jpeg: bytes, threshold: float) -> dict[str, float] | None:
    # The SDK accepts image paths; always delete the temporary camera frame.
    path = None
    try:
        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as image:
            path = image.name
            image.write(jpeg)
        result = client.run_workflow(workspace_name="ok-ok-ziiym", workflow_id="general-segmentation-api-7",
                                     images={"image": path}, parameters={"classes": "cardboard box"}, use_cache=True)
    finally:
        if path:
            os.unlink(path)
    if not isinstance(result, list) or len(result) != 1 or not isinstance(result[0], dict):
        raise ValueError("Invalid Roboflow workflow response")
    output = result[0].get("predictions")
    if isinstance(output, dict):
        output = output.get("predictions")
    if not isinstance(output, list):
        raise ValueError("Missing Roboflow predictions")
    matches = []
    for item in output:
        if not isinstance(item, dict) or item.get("class") != "cardboard box":
            continue
        confidence = float(item.get("confidence", 0))
        if not math.isfinite(confidence) or confidence < threshold:
            continue
        angle = tilt_from_mask(item.get("rle_mask") or {})
        if angle is not None and angle >= 15.0:
            matches.append({"confidence": confidence, "tilt_degrees": round(angle, 2)})
    return max(matches, key=lambda item: item["confidence"]) if matches else None
